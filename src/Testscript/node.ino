// node.ino — UNAGI モータノード (XIAO ESP32-C3)
// SS39ET × 2 (差動) / DRV8833 / ESP-NOW
//
// 重要な制約:
//   ESP32-C3 は WiFi 動作中 ADC2 が使用不可。ホールは必ず ADC1 (GPIO0-4) に。
//   XIAO ESP32-C3: A0=GPIO2, A1=GPIO3 (両方 ADC1)。A3=GPIO5 は ADC2 なので不可。

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "unagi_protocol.h"

// ---------------- 設定 ----------------
#define NODE_ID        1          // ★ ノードごとに 1..10 を書き換える
#define FW_HASH        0x00010003

#define PIN_HALL1      2          // ADC1_CH2
#define PIN_HALL2      3          // ADC1_CH3
#define PIN_AIN1       6          // DRV8833
#define PIN_AIN2       7
#define PIN_NSLEEP     10
#define PIN_SYNC_OUT   4          // SYNC-1 のオシロ観測用

#define CTRL_HZ        1000
#define PWM_HZ         25000      // CTRL_HZ の整数倍にすること（うなりを避ける）
#define PWM_BITS       10
#define OVERSAMPLE     4          // ADC 平均回数。増やすと 1ms 予算を圧迫する

// 較正係数 (HALL-1 で求めた値をノードごとに書き込む)
// theta_deg = C0 + C1*d + C2*d^2 + C3*d^3,  d = (v1 - v2) [ADC counts]
float CAL[4] = {0.0f, 0.02f, 0.0f, 0.0f};

// ---------------- 状態 ----------------
enum State { IDLE, SYNCING, SYNCED, ARMED, WAIT_T, RUNNING, DONE };
volatile State state = IDLE;

uint8_t head_mac[6];
bool    head_known = false;

// 同期
#define MAX_SYNC_PTS 400
static double  sx[MAX_SYNC_PTS], sy[MAX_SYNC_PTS];
static uint16_t n_sync = 0, want_sync = 300;
double sync_a = 0.0, sync_b = 1.0;
float  sync_sigma = 1e9f;

// パラメータ
MsgParam par;
uint16_t trial = 0;
int64_t  t_start_head = 0, t_start_local = 0;

// ログ (int16 × 4: cmd, meas, u, aux)
static int16_t logbuf[UNAGI_MAX_SAMPLES][4];
volatile uint32_t n_log = 0;

// 統計
volatile uint32_t missed = 0, max_jitter = 0;
volatile float    adc_us_max = 0, err_sq_sum = 0;
volatile uint32_t err_n = 0;

int16_t bias1 = 0, bias2 = 0;

SemaphoreHandle_t tick_sem;
hw_timer_t* timer = nullptr;

// ---------------- ADC ----------------
static inline void read_hall(int16_t& v1, int16_t& v2) {
  int32_t a = 0, b = 0;
  for (int i = 0; i < OVERSAMPLE; i++) { a += analogRead(PIN_HALL1); b += analogRead(PIN_HALL2); }
  v1 = a / OVERSAMPLE;
  v2 = b / OVERSAMPLE;
}

static inline float hall_to_deg(int16_t v1, int16_t v2) {
  float d = (float)(v1 - v2) - (float)(bias1 - bias2);
  return CAL[0] + d * (CAL[1] + d * (CAL[2] + d * CAL[3]));
}

// ---------------- モータ ----------------
static inline void drive(float u) {              // u in [-1, 1]
  const int MAXD = (1 << PWM_BITS) - 1;
  if (u >= 0) { ledcWrite(PIN_AIN1, (int)(u * MAXD)); ledcWrite(PIN_AIN2, 0); }
  else        { ledcWrite(PIN_AIN1, 0); ledcWrite(PIN_AIN2, (int)(-u * MAXD)); }
}
static inline void brake() { ledcWrite(PIN_AIN1, 0); ledcWrite(PIN_AIN2, 0); }

// ---------------- ESP-NOW ----------------
static void send_head(const void* p, size_t n) {
  if (head_known) esp_now_send(head_mac, (const uint8_t*)p, n);
}

void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int64_t t_local = esp_timer_get_time();          // ★ 最優先でタイムスタンプ
  if (len < (int)sizeof(Hdr)) return;
  const Hdr* h = (const Hdr*)data;

  if (!head_known) { memcpy(head_mac, info->src_addr, 6); head_known = true;
    esp_now_peer_info_t p{}; memcpy(p.peer_addr, head_mac, 6);
    p.channel = UNAGI_CHANNEL; p.encrypt = false; esp_now_add_peer(&p); }

  switch (h->type) {
    case MSG_SYNC_BEGIN: {
      auto* m = (const MsgSyncBegin*)data;
      want_sync = min((uint16_t)MAX_SYNC_PTS, m->n_points);
      n_sync = 0; state = SYNCING;
      break;
    }
    case MSG_BEACON: {
      if (state != SYNCING || n_sync >= want_sync) break;
      auto* m = (const MsgBeacon*)data;
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
    case MSG_ARM: {
      n_log = 0; missed = 0; max_jitter = 0; adc_us_max = 0;
      err_sq_sum = 0; err_n = 0;
      int16_t v1, v2; read_hall(v1, v2); bias1 = v1; bias2 = v2;
      MsgReady r{}; r.h = {MSG_READY, NODE_ID, trial};
      r.fw_hash = FW_HASH; r.sigma_us = sync_sigma; r.b = sync_b; r.a = sync_a;
      r.param_crc = crc16((uint8_t*)&par, sizeof(par));
      r.free_ram = ESP.getFreeHeap();
      r.hall1_bias = v1; r.hall2_bias = v2;
      r.ready = (state == SYNCED && sync_sigma < 500.0f) ? 1 : 0;
      send_head(&r, sizeof(r));
      if (r.ready) state = ARMED;
      break;
    }
    case MSG_START: {
      if (state != ARMED) break;
      auto* m = (const MsgStart*)data;
      if (h->trial_id != trial) break;
      t_start_head  = m->t_start_head_us;
      t_start_local = (int64_t)(((double)t_start_head - sync_a) / sync_b);
      state = WAIT_T;
      break;
    }
    case MSG_STOP: {
      if (state == RUNNING || state == WAIT_T) { brake(); state = DONE; }
      break;
    }
    case MSG_DUMP_REQ: {
      if (h->node_id != NODE_ID) break;
      auto* m = (const MsgDumpReq*)data;
      MsgDumpData d{}; d.h = {MSG_DUMP_DATA, NODE_ID, trial}; d.block = m->block;
      uint32_t off = (uint32_t)m->block * DUMP_SAMPLES_PER_BLOCK;
      uint32_t n = (off >= n_log) ? 0 : min((uint32_t)DUMP_SAMPLES_PER_BLOCK, n_log - off);
      d.n = n;
      for (uint32_t i = 0; i < n; i++) memcpy(d.d[i], logbuf[off + i], 8);
      send_head(&d, sizeof(d));
      break;
    }
  }
}

// 最小二乗: t_head = a + b * t_local
void fit_sync() {
  double mx = 0, my = 0;
  for (uint16_t i = 0; i < n_sync; i++) { mx += sx[i]; my += sy[i]; }
  mx /= n_sync; my /= n_sync;
  double sxy = 0, sxx = 0;
  for (uint16_t i = 0; i < n_sync; i++) { sxy += (sx[i]-mx)*(sy[i]-my); sxx += (sx[i]-mx)*(sx[i]-mx); }
  sync_b = sxy / sxx;
  sync_a = my - sync_b * mx;
  double s2 = 0;
  for (uint16_t i = 0; i < n_sync; i++) { double r = sy[i] - (sync_a + sync_b*sx[i]); s2 += r*r; }
  sync_sigma = (float)sqrt(s2 / n_sync);
}

// ---------------- 制御 ----------------
void IRAM_ATTR on_timer() {
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(tick_sem, &hp);
  if (hp) portYIELD_FROM_ISR();
}

void control_task(void*) {
  const float dt = 1.0f / CTRL_HZ;
  uint32_t n = 0; float integ = 0, e_prev = 0;
  int64_t t_next = 0;

  for (;;) {
    if (state != WAIT_T && state != RUNNING) { vTaskDelay(1); continue; }

    // --- t_start までスピン待ち ---
    if (state == WAIT_T) {
      while (esp_timer_get_time() < t_start_local) { if (state != WAIT_T) break; }
      if (state != WAIT_T) continue;

      if (par.mode == MODE_GPIO_SYNC) {           // SYNC-1
        digitalWrite(PIN_SYNC_OUT, HIGH);
        delayMicroseconds(200);
        digitalWrite(PIN_SYNC_OUT, LOW);
        state = DONE; continue;
      }
      n = 0; integ = 0; e_prev = 0;
      t_next = t_start_local;
      timerAlarm(timer, 1000000 / CTRL_HZ, true, 0);
      state = RUNNING;
    }

    // --- 1 kHz 周期 ---
    if (xSemaphoreTake(tick_sem, pdMS_TO_TICKS(10)) != pdTRUE) continue;

    int64_t t_now = esp_timer_get_time();
    t_next += 1000000 / CTRL_HZ;
    int32_t jit = (int32_t)(t_now - t_next);
    if (jit < 0) jit = -jit;
    if ((uint32_t)jit > max_jitter) max_jitter = jit;
    if (jit > 1000000 / CTRL_HZ) missed++;

    int64_t ta = esp_timer_get_time();
    int16_t v1, v2; read_hall(v1, v2);
    float dus = (float)(esp_timer_get_time() - ta);
    if (dus > adc_us_max) adc_us_max = dus;

    float t = n * dt;
    float theta_meas = hall_to_deg(v1, v2);
    float theta_cmd = 0, u = 0;

    switch (par.mode) {
      case MODE_HALL_RAW:                          // HALL-1: 駆動しない
        u = 0; theta_cmd = 0; break;
      case MODE_PWM_SWEEP:                         // HALL-2: デューティ掃引
        u = par.aux * (t / (par.duration_ms / 1000.0f));
        theta_cmd = 0; break;
      case MODE_STEP:                              // JOINT-1
        theta_cmd = (t < 0.2f) ? 0.0f : par.aux;
        goto pid;
      default:                                     // MODE_SWIM
        theta_cmd = par.A_deg * sinf(2*PI*par.f_hz*t + par.phi_deg*PI/180.0f) + par.off_deg;
      pid: {
        float e = theta_cmd - theta_meas;
        integ += e * dt;
        integ = constrain(integ, -50.0f, 50.0f);
        u = par.kp*e + par.ki*integ + par.kd*(e - e_prev)/dt;
        e_prev = e;
        u = constrain(u, -1.0f, 1.0f);
        err_sq_sum += e*e; err_n++;
        break;
      }
    }
    drive(u);

    // --- ログ（間引き） ---
    uint32_t div = CTRL_HZ / par.log_hz;
    if (div == 0) div = 1;
    if (n % div == 0 && n_log < UNAGI_MAX_SAMPLES) {
      logbuf[n_log][0] = (int16_t)(theta_cmd * 100);
      logbuf[n_log][1] = (int16_t)(theta_meas * 100);
      logbuf[n_log][2] = (int16_t)(u * 10000);
      // MODE_HALL_RAW では生値、それ以外はコモンモード (v1+v2) を残す
      logbuf[n_log][3] = (par.mode == MODE_HALL_RAW) ? v1 : (int16_t)((v1 + v2) >> 1);
      if (par.mode == MODE_HALL_RAW && n_log + 1 < UNAGI_MAX_SAMPLES) {
        logbuf[n_log][2] = v2;                    // 生値モードでは u の枠に v2 を入れる
      }
      n_log++;
    }

    n++;
    if (t * 1000.0f >= par.duration_ms) {
      brake(); timerAlarm(timer, 1000000, false, 0); state = DONE;
      MsgReport r{}; r.h = {MSG_REPORT, NODE_ID, trial};
      r.n_samples = n_log; r.t_start_local_us = t_start_local;
      r.missed = missed; r.max_jitter_us = max_jitter;
      r.rms_err_deg = err_n ? sqrtf(err_sq_sum / err_n) : 0;
      r.adc_us_max = adc_us_max;
      send_head(&r, sizeof(r));
    }
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_SYNC_OUT, OUTPUT); digitalWrite(PIN_SYNC_OUT, LOW);
  pinMode(PIN_NSLEEP, OUTPUT);   digitalWrite(PIN_NSLEEP, HIGH);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_HALL1, ADC_11db);
  analogSetPinAttenuation(PIN_HALL2, ADC_11db);

  ledcAttach(PIN_AIN1, PWM_HZ, PWM_BITS);
  ledcAttach(PIN_AIN2, PWM_HZ, PWM_BITS);
  brake();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(UNAGI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);                 // ★ パワーセーブ無効
  esp_now_init();
  esp_now_register_recv_cb(on_recv);

  Serial.printf("node %d  mac %s\n", NODE_ID, WiFi.macAddress().c_str());

  tick_sem = xSemaphoreCreateBinary();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &on_timer);
  xTaskCreate(control_task, "ctrl", 8192, nullptr, 24, nullptr);
}

void loop() { vTaskDelay(1000); }
