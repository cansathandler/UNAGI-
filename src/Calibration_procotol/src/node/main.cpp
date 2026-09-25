// main.cpp — UNAGI モータノード (XIAO ESP32-C3)
// SS39ET × 2 (差動) / DRV8833 / ESP-NOW
//
// 同一バイナリを10台に書き込みます。node_id と較正係数は NVS に保存。
//   シリアル:  id 3  /  cal 0.1 0.021 0 0  /  save  /  show
//   無線:      PC から  cal 3 0.1 0.021 0 0
//
// 配線 (既存の hall_motor_node と同一):
//   D3 (GPIO5)  --> DRV8833 AIN1
//   D4 (GPIO6)  --> DRV8833 AIN2
//   D2 (GPIO4)  --> enable / DRV8833 SLP   (HIGH = 駆動可)
//   A0 (GPIO2)  <-- Hall S1  (ADC1)
//   A1 (GPIO3)  <-- Hall S2  (ADC1)
//   D10(GPIO10) --> SYNC パルス出力 (SYNC-1 のオシロ観測用、新規)
//
// 重要: ESP32-C3 は WiFi 動作中 ADC2 が使用不可。ホール入力は必ず ADC1 (GPIO0-4) に。
//       GPIO5 は ADC2 だが PWM 出力に使う分には問題ない（制約はアナログ入力のみ）。
//       ホール出力は 2.5 V 以下に収めること (11 dB 減衰時の実用レンジ)。

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "unagi_protocol.h"   // include/ に置いてあります

// ---------------- 設定 ----------------
#define FW_HASH        0x00010004

#define PIN_HALL1      2          // A0  ADC1_CH2
#define PIN_HALL2      3          // A1  ADC1_CH3
#define PIN_AIN1       5          // D3  DRV8833 AIN1
#define PIN_AIN2       6          // D4  DRV8833 AIN2
#define PIN_EN         4          // D2  enable / SLP
#define PIN_SYNC_OUT  10          // D10 SYNC-1 のオシロ観測用

static const bool EN_ACTIVE_HIGH = true;   // HIGH でドライバが起きる
static const bool SLOW_DECAY     = true;   // 既存ノードで検証済みの設定

#define CTRL_HZ        1000
#define PWM_HZ         25000      // CTRL_HZ の整数倍にすること
#define PWM_BITS       10
#define OVERSAMPLE     4          // ADC 平均回数。missed が出たら 2 に下げる

// ---------------- Arduino core 2.x / 3.x 互換 ----------------
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_CH_A PIN_AIN1
  #define PWM_CH_B PIN_AIN2
#else
  #define PWM_CH_A 0
  #define PWM_CH_B 1
#endif

static hw_timer_t* timer = nullptr;
static void IRAM_ATTR on_timer();

static void pwm_init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_AIN1, PWM_HZ, PWM_BITS);
  ledcAttach(PIN_AIN2, PWM_HZ, PWM_BITS);
#else
  ledcSetup(PWM_CH_A, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_AIN1, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_AIN2, PWM_CH_B);
#endif
}

static void timer_init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timer = timerBegin(1000000);                 // 1 MHz
  timerAttachInterrupt(timer, &on_timer);
  timerStop(timer);
#else
  timer = timerBegin(0, 80, true);             // 80 分周 = 1 MHz
  timerAttachInterrupt(timer, &on_timer, true);
  timerAlarmWrite(timer, 1000000 / CTRL_HZ, true);
#endif
}

static void timer_start() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerWrite(timer, 0);
  timerAlarm(timer, 1000000 / CTRL_HZ, true, 0);
  timerStart(timer);
#else
  timerWrite(timer, 0);
  timerAlarmEnable(timer);
#endif
}

static void timer_stop() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerStop(timer);
#else
  timerAlarmDisable(timer);
#endif
}

// ---------------- 前方宣言 ----------------
void fit_sync();
static void send_head(const void* p, size_t n);
void save_cfg();
void load_cfg();
void control_task(void*);
void serial_task(void*);

// ---------------- NVS 設定 ----------------
Preferences prefs;
uint8_t NODE_ID  = 0;                             // 0 = 未設定
float   CAL[4]   = {0.0f, 0.02f, 0.0f, 0.0f};     // theta = C0 + C1*d + C2*d^2 + C3*d^3
float   MIN_DUTY = 0.0f;                          // デッドゾーン補償（個体差が大きい）

void load_cfg() {
  prefs.begin("unagi", true);
  NODE_ID = prefs.getUChar("id", 0);
  CAL[0] = prefs.getFloat("c0", 0.0f);
  CAL[1] = prefs.getFloat("c1", 0.02f);
  CAL[2] = prefs.getFloat("c2", 0.0f);
  CAL[3] = prefs.getFloat("c3", 0.0f);
  MIN_DUTY = prefs.getFloat("md", 0.0f);
  prefs.end();
}

void save_cfg() {
  prefs.begin("unagi", false);
  prefs.putUChar("id", NODE_ID);
  prefs.putFloat("c0", CAL[0]); prefs.putFloat("c1", CAL[1]);
  prefs.putFloat("c2", CAL[2]); prefs.putFloat("c3", CAL[3]);
  prefs.putFloat("md", MIN_DUTY);
  prefs.end();
}

// ---------------- 状態 ----------------
enum State { IDLE, SYNCING, SYNCED, ARMED, WAIT_T, RUNNING, DONE };
volatile State state = IDLE;

uint8_t head_mac[6];
bool    head_known = false;

#define MAX_SYNC_PTS 400
static double   sx[MAX_SYNC_PTS], sy[MAX_SYNC_PTS];
static uint16_t n_sync = 0, want_sync = 300;
double sync_a = 0.0, sync_b = 1.0;
float  sync_sigma = 1e9f;

MsgParam par;
uint16_t trial = 0;
int64_t  t_start_head = 0, t_start_local = 0;

static int16_t logbuf[UNAGI_MAX_SAMPLES][4];
volatile uint32_t n_log = 0;

volatile uint32_t missed = 0, max_jitter = 0;
volatile float    adc_us_max = 0, err_sq_sum = 0;
volatile uint32_t err_n = 0;

int16_t bias1 = 0, bias2 = 0;

SemaphoreHandle_t tick_sem;

// ---------------- ADC / モータ ----------------
static inline void read_hall(int16_t& v1, int16_t& v2) {
  int32_t a = 0, b = 0;
  for (int i = 0; i < OVERSAMPLE; i++) { a += analogRead(PIN_HALL1); b += analogRead(PIN_HALL2); }
  v1 = (int16_t)(a / OVERSAMPLE);
  v2 = (int16_t)(b / OVERSAMPLE);
}

static inline float hall_to_deg(int16_t v1, int16_t v2) {
  float d = (float)(v1 - v2) - (float)(bias1 - bias2);
  return CAL[0] + d * (CAL[1] + d * (CAL[2] + d * CAL[3]));
}

volatile bool en_state = false;

static inline void set_enable(bool on) {
  en_state = on;
  digitalWrite(PIN_EN, (on == EN_ACTIVE_HIGH) ? HIGH : LOW);
}

// u in [-1, 1]。既存 hall_motor_node と同じスローディケイ＋デッドゾーン補償。
static inline void drive(float u) {
  const int MAXD = (1 << PWM_BITS) - 1;
  u = constrain(u, -1.0f, 1.0f);
  float mag = fabsf(u);
  mag = (mag > 1e-4f) ? (MIN_DUTY + mag * (1.0f - MIN_DUTY)) : 0.0f;
  const int d = (int)lroundf(mag * MAXD);

  int a, b;
  if (!en_state) { a = 0; b = 0; }                       // コースト
  else if (SLOW_DECAY) {
    if (u >= 0) { a = MAXD;     b = MAXD - d; }
    else        { a = MAXD - d; b = MAXD;     }
  } else {
    if (u >= 0) { a = d; b = 0; }
    else        { a = 0; b = d; }
  }
  ledcWrite(PWM_CH_A, a);
  ledcWrite(PWM_CH_B, b);
}

// 走行終了・STOP 時: PWM を落としてイネーブルも切る
static inline void brake() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  set_enable(false);
}

// ---------------- 同期回帰 ----------------
void fit_sync() {
  if (n_sync < 10) { sync_sigma = 1e9f; return; }
  double mx = 0, my = 0;
  for (uint16_t i = 0; i < n_sync; i++) { mx += sx[i]; my += sy[i]; }
  mx /= n_sync; my /= n_sync;
  double sxy = 0, sxx = 0;
  for (uint16_t i = 0; i < n_sync; i++) {
    sxy += (sx[i] - mx) * (sy[i] - my);
    sxx += (sx[i] - mx) * (sx[i] - mx);
  }
  sync_b = sxy / sxx;
  sync_a = my - sync_b * mx;
  double s2 = 0;
  for (uint16_t i = 0; i < n_sync; i++) {
    double r = sy[i] - (sync_a + sync_b * sx[i]);
    s2 += r * r;
  }
  sync_sigma = (float)sqrt(s2 / n_sync);
}

// ---------------- ESP-NOW ----------------
static void send_head(const void* p, size_t n) {
  if (head_known) esp_now_send(head_mac, (const uint8_t*)p, n);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src = info->src_addr;
#else
void on_recv(const uint8_t* mac, const uint8_t* data, int len) {
  const uint8_t* src = mac;
#endif
  int64_t t_local = esp_timer_get_time();          // ★ 最優先でタイムスタンプ
  if (len < (int)sizeof(Hdr)) return;
  const Hdr* h = (const Hdr*)data;

  if (!head_known) {
    memcpy(head_mac, src, 6); head_known = true;
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, head_mac, 6);
    p.channel = UNAGI_CHANNEL; p.encrypt = false;
    esp_now_add_peer(&p);
  }
  // HELLO への応答は ID 未設定でも返す（存在確認に使うため）
  if (h->type == MSG_HELLO) {
    MsgHello r = {}; r.h = {MSG_HELLO, NODE_ID, 0};
    WiFi.macAddress(r.mac); r.fw_hash = FW_HASH;
    send_head(&r, sizeof(r));
    return;
  }
  if (NODE_ID == 0) return;                        // ID 未設定なら以降は無視

  switch (h->type) {
    case MSG_SYNC_BEGIN: {
      const MsgSyncBegin* m = (const MsgSyncBegin*)data;
      want_sync = m->n_points < MAX_SYNC_PTS ? m->n_points : MAX_SYNC_PTS;
      n_sync = 0; state = SYNCING;
      break;
    }
    case MSG_BEACON: {
      if (state != SYNCING || n_sync >= want_sync) break;
      const MsgBeacon* m = (const MsgBeacon*)data;
      sx[n_sync] = (double)t_local;
      sy[n_sync] = (double)m->t_head_us;
      n_sync++;
      if (n_sync >= want_sync) { fit_sync(); state = SYNCED; }
      break;
    }
    case MSG_PARAM: {
      if (h->node_id != NODE_ID) break;
      memcpy(&par, data, sizeof(par));
      trial = h->trial_id;
      break;
    }
    case MSG_CAL_SET: {
      if (h->node_id != NODE_ID) break;
      const MsgCalSet* m = (const MsgCalSet*)data;
      memcpy(CAL, m->c, sizeof(CAL));
      save_cfg();
      MsgCalAck a = {}; a.h = {MSG_CAL_ACK, NODE_ID, trial};
      memcpy(a.c, CAL, sizeof(CAL));
      send_head(&a, sizeof(a));
      break;
    }
    case MSG_ARM: {
      n_log = 0; missed = 0; max_jitter = 0; adc_us_max = 0;
      err_sq_sum = 0; err_n = 0;
      int16_t v1, v2; read_hall(v1, v2); bias1 = v1; bias2 = v2;
      MsgReady r = {}; r.h = {MSG_READY, NODE_ID, trial};
      r.fw_hash = FW_HASH; r.sigma_us = sync_sigma; r.b = sync_b; r.a = sync_a;
      r.param_crc = crc16((const uint8_t*)&par, sizeof(par));
      r.free_ram = ESP.getFreeHeap();
      r.hall1_bias = v1; r.hall2_bias = v2;
      r.ready = (state == SYNCED && sync_sigma < 500.0f) ? 1 : 0;
      send_head(&r, sizeof(r));
      if (r.ready) state = ARMED;
      break;
    }
    case MSG_START: {
      if (state != ARMED || h->trial_id != trial) break;
      const MsgStart* m = (const MsgStart*)data;
      t_start_head  = m->t_start_head_us;
      t_start_local = (int64_t)(((double)t_start_head - sync_a) / sync_b);
      state = WAIT_T;
      break;
    }
    case MSG_STOP: {
      if (state == RUNNING || state == WAIT_T) { brake(); timer_stop(); state = DONE; }
      break;
    }
    case MSG_DUMP_REQ: {
      if (h->node_id != NODE_ID) break;
      const MsgDumpReq* m = (const MsgDumpReq*)data;
      MsgDumpData d = {}; d.h = {MSG_DUMP_DATA, NODE_ID, trial}; d.block = m->block;
      uint32_t off = (uint32_t)m->block * DUMP_SAMPLES_PER_BLOCK;
      uint32_t n = (off >= n_log) ? 0
                 : ((n_log - off) < DUMP_SAMPLES_PER_BLOCK ? (n_log - off) : DUMP_SAMPLES_PER_BLOCK);
      d.n = (uint16_t)n;
      for (uint32_t i = 0; i < n; i++) memcpy(d.d[i], logbuf[off + i], 8);
      send_head(&d, sizeof(d));
      break;
    }
  }
}

// ---------------- 制御 ----------------
static void IRAM_ATTR on_timer() {
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(tick_sem, &hp);
  if (hp) portYIELD_FROM_ISR();
}

void control_task(void*) {
  const float dt = 1.0f / CTRL_HZ;
  uint32_t n = 0;
  float integ = 0, e_prev = 0;
  int64_t t_next = 0;

  for (;;) {
    if (state != WAIT_T && state != RUNNING) { vTaskDelay(1); continue; }

    // --- t_start までスピン待ち ---
    if (state == WAIT_T) {
      while (state == WAIT_T && esp_timer_get_time() < t_start_local) { /* spin */ }
      if (state != WAIT_T) continue;

      if (par.mode == MODE_GPIO_SYNC) {                 // SYNC-1
        digitalWrite(PIN_SYNC_OUT, HIGH);
        delayMicroseconds(200);
        digitalWrite(PIN_SYNC_OUT, LOW);
        state = DONE;
        continue;
      }
      n = 0; integ = 0; e_prev = 0;
      t_next = t_start_local;
      set_enable(par.mode != MODE_HALL_RAW);   // HALL-1 は駆動しない
      state = RUNNING;
      timer_start();
    }

    // --- 1 kHz 周期 ---
    if (xSemaphoreTake(tick_sem, pdMS_TO_TICKS(10)) != pdTRUE) continue;
    if (state != RUNNING) continue;

    int64_t t_now = esp_timer_get_time();
    t_next += 1000000 / CTRL_HZ;
    int32_t jit = (int32_t)(t_now - t_next);
    if (jit < 0) jit = -jit;
    if ((uint32_t)jit > max_jitter) max_jitter = (uint32_t)jit;
    if (jit > 1000000 / CTRL_HZ) missed++;

    int64_t ta = esp_timer_get_time();
    int16_t v1, v2; read_hall(v1, v2);
    float dus = (float)(esp_timer_get_time() - ta);
    if (dus > adc_us_max) adc_us_max = dus;

    float t = n * dt;
    float theta_meas = hall_to_deg(v1, v2);
    float theta_cmd = 0, u = 0;
    bool  use_pid = false;

    switch (par.mode) {
      case MODE_HALL_RAW:                                // 駆動しない
        break;
      case MODE_PWM_SWEEP:                               // デューティ掃引
        u = par.aux * (t / (par.duration_ms / 1000.0f));
        break;
      case MODE_STEP:
        theta_cmd = (t < 0.2f) ? 0.0f : par.aux;
        use_pid = true;
        break;
      default:                                           // MODE_SWIM
        theta_cmd = par.A_deg * sinf(2.0f * PI * par.f_hz * t + par.phi_deg * PI / 180.0f)
                  + par.off_deg;
        use_pid = true;
        break;
    }

    if (use_pid) {
      float e = theta_cmd - theta_meas;
      integ += e * dt;
      integ = constrain(integ, -50.0f, 50.0f);
      u = par.kp * e + par.ki * integ + par.kd * (e - e_prev) / dt;
      e_prev = e;
      u = constrain(u, -1.0f, 1.0f);
      err_sq_sum += e * e; err_n++;
    }
    drive(u);

    // --- ログ（間引き） ---
    uint32_t div = (par.log_hz > 0) ? (CTRL_HZ / par.log_hz) : 1;
    if (div == 0) div = 1;
    if (n % div == 0 && n_log < UNAGI_MAX_SAMPLES) {
      if (par.mode == MODE_HALL_RAW) {
        logbuf[n_log][0] = 0;
        logbuf[n_log][1] = (int16_t)(theta_meas * 100.0f);
        logbuf[n_log][2] = v2;                           // 生値
        logbuf[n_log][3] = v1;
      } else {
        logbuf[n_log][0] = (int16_t)(theta_cmd * 100.0f);
        logbuf[n_log][1] = (int16_t)(theta_meas * 100.0f);
        logbuf[n_log][2] = (int16_t)(u * 10000.0f);
        logbuf[n_log][3] = (int16_t)((v1 + v2) >> 1);    // コモンモード
      }
      n_log++;
    }

    n++;
    if (t * 1000.0f >= par.duration_ms) {
      brake(); timer_stop(); state = DONE;
      MsgReport r = {}; r.h = {MSG_REPORT, NODE_ID, trial};
      r.n_samples = n_log; r.t_start_local_us = t_start_local;
      r.missed = missed; r.max_jitter_us = max_jitter;
      r.rms_err_deg = err_n ? sqrtf(err_sq_sum / err_n) : 0.0f;
      r.adc_us_max = adc_us_max;
      send_head(&r, sizeof(r));
    }
  }
}

// ---------------- シリアルコンソール ----------------
void serial_task(void*) {
  for (;;) {
    if (Serial.available()) {
      String c = Serial.readStringUntil('\n'); c.trim();
      if (c.startsWith("id")) {
        int v = c.substring(2).toInt();
        if (v >= 1 && v <= UNAGI_N_NODES) { NODE_ID = (uint8_t)v; Serial.printf("id = %d (save で保存)\n", v); }
        else Serial.println("id は 1..10");
      } else if (c.startsWith("cal")) {
        float a, b, d, e;
        if (sscanf(c.c_str(), "cal %f %f %f %f", &a, &b, &d, &e) == 4) {
          CAL[0]=a; CAL[1]=b; CAL[2]=d; CAL[3]=e;
          Serial.println("cal 設定 (save で保存)");
        } else Serial.println("cal <c0> <c1> <c2> <c3>");
      } else if (c.startsWith("minduty")) {
        MIN_DUTY = constrain(c.substring(7).toFloat(), 0.0f, 0.9f);
        Serial.printf("minduty = %.3f (save で保存)\n", MIN_DUTY);
      } else if (c.startsWith("en")) {
        set_enable(c.substring(2).toInt() != 0);
        Serial.printf("en = %d\n", en_state ? 1 : 0);
      } else if (c.startsWith("save")) {
        save_cfg(); Serial.println("NVS に保存しました");
      } else if (c.startsWith("show")) {
        Serial.printf("id=%d  mac=%s  fw=%08X\n", NODE_ID, WiFi.macAddress().c_str(), FW_HASH);
        Serial.printf("CAL = {%g, %g, %g, %g}  minduty=%.3f\n",
                      CAL[0], CAL[1], CAL[2], CAL[3], MIN_DUTY);
        Serial.printf("state=%d  sigma=%.1fus  b=%.9f  freeheap=%u\n",
                      (int)state, sync_sigma, sync_b, (unsigned)ESP.getFreeHeap());
        int16_t v1, v2; read_hall(v1, v2);
        Serial.printf("hall v1=%d v2=%d diff=%d deg=%.2f\n", v1, v2, v1-v2, hall_to_deg(v1,v2));
      } else if (c.length()) {
        Serial.println("commands: id <n> | cal <c0> <c1> <c2> <c3> | minduty <x> "
                       "| en <0|1> | save | show");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_SYNC_OUT, OUTPUT); digitalWrite(PIN_SYNC_OUT, LOW);
  pinMode(PIN_EN, OUTPUT);
  set_enable(false);             // 起動時はドライバを寝かせておく

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_HALL1, ADC_11db);
  analogSetPinAttenuation(PIN_HALL2, ADC_11db);

  pwm_init();
  brake();
  load_cfg();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(UNAGI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_now_init();
  esp_now_register_recv_cb(on_recv);

  esp_now_peer_info_t bp = {};
  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  memcpy(bp.peer_addr, bcast, 6);
  bp.channel = UNAGI_CHANNEL; bp.encrypt = false;
  esp_now_add_peer(&bp);

  Serial.printf("\nUNAGI node  id=%d  mac=%s\n", NODE_ID, WiFi.macAddress().c_str());
  if (NODE_ID == 0) Serial.println("★ ID 未設定です。 'id <1-10>' のあと 'save' を実行してください。");

  tick_sem = xSemaphoreCreateBinary();
  timer_init();
  xTaskCreate(control_task, "ctrl", 8192, nullptr, 24, nullptr);
  xTaskCreate(serial_task,  "cli",  4096, nullptr, 2,  nullptr);

  // ヘッドに自己紹介（ブロードキャスト）
  MsgHello hello = {}; hello.h = {MSG_HELLO, NODE_ID, 0};
  WiFi.macAddress(hello.mac); hello.fw_hash = FW_HASH;
  esp_now_send(bcast, (uint8_t*)&hello, sizeof(hello));
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
