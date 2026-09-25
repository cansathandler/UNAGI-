// =====================================================================
//  UNAGI モータノード  (Seeed XIAO ESP32-C3 + DRV8833 + SS39ET ×2)
//  protocol v2 / 10台に同じバイナリを書き込む。ID と較正は NVS に保存。
//
//  配線（hall_motor_node と同一。D10 のみ追加）
//    D3  (GPIO5)  --> DRV8833 AIN1
//    D4  (GPIO6)  --> DRV8833 AIN2
//    D2  (GPIO4)  --> enable / DRV8833 SLP   (HIGH = 駆動可)
//    A0  (GPIO2)  <-- SS39ET S1  (ADC1)
//    A1  (GPIO3)  <-- SS39ET S2  (ADC1)
//    D10 (GPIO10) --> SYNC パルス（T の瞬間に 2 ms High。未接続でも可）
//
//  USB シリアル (115200) のコマンド:  help で一覧
//    id <1-10> / cal c0 c1 c2 c3 / minduty x / zero / save / show
//    hall        ホール生値を 10 Hz で表示（もう一度で停止）
//    pwm <u> [ms] 開ループで u (−1..1) を ms だけ出す（配線確認用）
//
//  v1 (Calibration_procotol) からの修正点
//   1. ヘッド以外（他ノード）からのパケットを無視する（magic で判定）。
//      v1 では起動時の HELLO をノード同士が受け取り、相手をヘッドと誤認して
//      HELLO を無限に投げ合っていた。以後 READY/REPORT は別ノードに送られていた。
//   2. ARM で trial を確定し、START はその trial だけ受理する。
//      v1 は ARM で trial を更新せず、START が常に無視されていた。
//   3. 同期は常時ビーコン(20 Hz)の移動窓回帰。同期のやり直しは不要。
//   4. 受信コールバックはキューに積むだけ。NVS 書き込み・ADC・送信は loop() で。
//   5. 角度は生の差分 (v1−v2) に多項式を当てるだけ。ARM 時のバイアス減算を廃止
//      （較正時と走行時で式が食い違っていた）。
//   6. 開始待ちで数百 ms 空回りしない（直前 3 ms だけスピン）。
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <Preferences.h>
#include <math.h>
#include "unagi_protocol.h"

#define FW_VERSION      0x00020001u

// ------------------------------ pins ---------------------------------
#define PIN_HALL1       2      // A0  ADC1_CH2
#define PIN_HALL2       3      // A1  ADC1_CH3
#define PIN_AIN1        5      // D3
#define PIN_AIN2        6      // D4
#define PIN_EN          4      // D2
#define PIN_SYNC_OUT   10      // D10

static const bool EN_ACTIVE_HIGH = true;
static const bool SLOW_DECAY     = true;

// ------------------------------ control ------------------------------
#define CTRL_HZ         1000
#define PWM_HZ          25000
#define PWM_BITS        10
#define OVERSAMPLE      4
#define ANGLE_LIMIT_DEG 45.0f   // 閉ループ中にこれを 50 ms 超えたら停止
#define SYNC_RING       400     // 20 Hz × 20 s
#define SYNC_MIN_PTS    40
#define SYNC_MAX_SIGMA  500.0f  // us

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ------------------------------ PWM ----------------------------------
static const int PWM_MAX = (1 << PWM_BITS) - 1;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwm_init() {
  ledcAttach(PIN_AIN1, PWM_HZ, PWM_BITS);
  ledcAttach(PIN_AIN2, PWM_HZ, PWM_BITS);
}
static inline void pwm_write(uint32_t a, uint32_t b) { ledcWrite(PIN_AIN1, a); ledcWrite(PIN_AIN2, b); }
#else
static void pwm_init() {
  ledcSetup(0, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_AIN1, 0);
  ledcSetup(1, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_AIN2, 1);
}
static inline void pwm_write(uint32_t a, uint32_t b) { ledcWrite(0, a); ledcWrite(1, b); }
#endif

// ------------------------------ NVS ----------------------------------
// 名前空間とキーは v1 と同じ。既に書いた ID はそのまま残る。
static Preferences prefs;
static uint8_t NODE_ID  = 0;
static float   CAL[4]   = {0.0f, 0.02f, 0.0f, 0.0f};   // theta = C0 + C1 d + C2 d^2 + C3 d^3, d = v1 − v2
static float   MIN_DUTY = 0.0f;

static void load_cfg() {
  prefs.begin("unagi", true);
  NODE_ID  = prefs.getUChar("id", 0);
  CAL[0]   = prefs.getFloat("c0", 0.0f);
  CAL[1]   = prefs.getFloat("c1", 0.02f);
  CAL[2]   = prefs.getFloat("c2", 0.0f);
  CAL[3]   = prefs.getFloat("c3", 0.0f);
  MIN_DUTY = prefs.getFloat("md", 0.0f);
  prefs.end();
  if (NODE_ID > UNAGI_N_NODES) NODE_ID = 0;
}
static void save_cfg() {
  prefs.begin("unagi", false);
  prefs.putUChar("id", NODE_ID);
  prefs.putFloat("c0", CAL[0]); prefs.putFloat("c1", CAL[1]);
  prefs.putFloat("c2", CAL[2]); prefs.putFloat("c3", CAL[3]);
  prefs.putFloat("md", MIN_DUTY);
  prefs.end();
}

// ------------------------------ state --------------------------------
enum : uint8_t { S_IDLE = 0, S_ARMED = 1, S_WAIT = 2, S_RUN = 3, S_DONE = 4 };
static const char* STATE_NAME[] = {"IDLE", "ARMED", "WAIT", "RUN", "DONE"};
static volatile uint8_t state = S_IDLE;
static volatile bool    stop_req = false;

static uint8_t  head_mac[6];
static bool     head_known = false;
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static ParamBody par;
static bool      par_valid = false;
static uint16_t  par_crc = 0;
static uint16_t  armed_trial = 0;

// run-local copies (control task only reads these)
static ParamBody run_par;
static float     run_cal[4];
static float     run_md;
static double    run_b = 1.0;
static int64_t   t_start_local = 0;

static LogSample logbuf[UNAGI_MAX_SAMPLES];
static volatile uint32_t n_log = 0;

static MsgReport report;
static volatile bool report_pending = false;

static TaskHandle_t ctrl_task = nullptr;

// ------------------------------ sync ---------------------------------
// ヘッド時刻 y とローカル時刻 x を移動窓で回帰:  (y−y0) = a + b (x−x0)
static int64_t  sx[SYNC_RING], sy[SYNC_RING];
static uint16_t s_head = 0, s_count = 0;
static int64_t  s_last_x = 0, s_last_y = 0;
static uint32_t s_last_seq = 0;

static int64_t  fit_x0 = 0, fit_y0 = 0;
static double   fit_a = 0.0, fit_b = 1.0;
static float    fit_sigma = 1e9f;
static uint16_t fit_n = 0;
static bool     fit_ok = false;

static void sync_add(int64_t x, int64_t y, uint32_t seq) {
  if (s_count) {
    int64_t dx = x - s_last_x, dy = y - s_last_y;
    // ヘッド再起動・長い欠落を検出したら窓を捨てる
    if (dy <= 0 || dy > 2000000 || llabs(dx - dy) > 20000 || seq <= s_last_seq) {
      s_count = 0; s_head = 0; fit_ok = false;
    }
  }
  sx[s_head] = x; sy[s_head] = y;
  s_head = (s_head + 1) % SYNC_RING;
  if (s_count < SYNC_RING) s_count++;
  s_last_x = x; s_last_y = y; s_last_seq = seq;
}

static bool sync_fit() {
  fit_ok = false;
  if (s_count < SYNC_MIN_PTS) { fit_n = s_count; fit_sigma = 1e9f; return false; }
  static double X[SYNC_RING], Y[SYNC_RING];
  static bool   use[SYNC_RING];
  uint16_t start = (s_head + SYNC_RING - s_count) % SYNC_RING;
  int64_t x0 = sx[start], y0 = sy[start];
  for (uint16_t i = 0; i < s_count; i++) {
    uint16_t k = (start + i) % SYNC_RING;
    X[i] = (double)(sx[k] - x0);
    Y[i] = (double)(sy[k] - y0);
    use[i] = true;
  }
  double a = 0, b = 1, sigma = 1e9;
  uint16_t m = 0;
  for (int iter = 0; iter < 3; iter++) {
    double mx = 0, my = 0; m = 0;
    for (uint16_t i = 0; i < s_count; i++) if (use[i]) { mx += X[i]; my += Y[i]; m++; }
    if (m < SYNC_MIN_PTS / 2) return false;
    mx /= m; my /= m;
    double sxx = 0, sxy = 0;
    for (uint16_t i = 0; i < s_count; i++) if (use[i]) {
      sxx += (X[i] - mx) * (X[i] - mx);
      sxy += (X[i] - mx) * (Y[i] - my);
    }
    if (sxx <= 0) return false;
    b = sxy / sxx;
    a = my - b * mx;
    double s2 = 0;
    for (uint16_t i = 0; i < s_count; i++) if (use[i]) {
      double r = Y[i] - (a + b * X[i]); s2 += r * r;
    }
    sigma = sqrt(s2 / (m > 2 ? m - 2 : 1));
    double lim = 3.0 * sigma; if (lim < 50.0) lim = 50.0;   // 受信遅れ（外れ値）を除いて再回帰
    for (uint16_t i = 0; i < s_count; i++) {
      double r = Y[i] - (a + b * X[i]);
      use[i] = fabs(r) < lim;
    }
  }
  fit_x0 = x0; fit_y0 = y0; fit_a = a; fit_b = b;
  fit_sigma = (float)sigma; fit_n = m;
  fit_ok = (sigma < SYNC_MAX_SIGMA) && (fabs(b - 1.0) < 200e-6);
  return fit_ok;
}

static inline int64_t head_to_local(int64_t t_head) {
  return fit_x0 + (int64_t)llround(((double)(t_head - fit_y0) - fit_a) / fit_b);
}

// ------------------------------ I/O ----------------------------------
static inline void read_hall(int16_t& v1, int16_t& v2) {
  int32_t a = 0, b = 0;
  for (int i = 0; i < OVERSAMPLE; i++) { a += analogRead(PIN_HALL1); b += analogRead(PIN_HALL2); }
  v1 = (int16_t)(a / OVERSAMPLE);
  v2 = (int16_t)(b / OVERSAMPLE);
}
static inline float poly_deg(const float* c, float d) {
  return c[0] + d * (c[1] + d * (c[2] + d * c[3]));
}
static inline void set_enable(bool on) {
  digitalWrite(PIN_EN, (on == EN_ACTIVE_HIGH) ? HIGH : LOW);
}
// u in [−1, 1]。hall_motor_node と同じスローディケイ + デッドゾーン補償。
static inline void drive(float u, float md) {
  if (u >  1.0f) u =  1.0f;
  if (u < -1.0f) u = -1.0f;
  float mag = fabsf(u);
  mag = (mag > 1e-4f) ? (md + mag * (1.0f - md)) : 0.0f;
  uint32_t d = (uint32_t)lroundf(mag * PWM_MAX);
  if (SLOW_DECAY) {
    if (u >= 0) pwm_write(PWM_MAX, PWM_MAX - d);
    else        pwm_write(PWM_MAX - d, PWM_MAX);
  } else {
    if (u >= 0) pwm_write(d, 0);
    else        pwm_write(0, d);
  }
}
static inline void motor_off() { pwm_write(0, 0); set_enable(false); }

// ------------------------------ ESP-NOW ------------------------------
struct RxItem { int64_t t; uint8_t src[6]; uint8_t len; uint8_t data[250]; };
static QueueHandle_t rxq;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src = info->src_addr;
#else
static void on_recv(const uint8_t* src, const uint8_t* data, int len) {
#endif
  int64_t t = esp_timer_get_time();               // 最初にタイムスタンプ
  if (len < (int)sizeof(Hdr) || len > 250) return;
  if (data[0] != UNAGI_MAGIC_HEAD) return;        // ヘッド以外（他ノード）は無視
  RxItem it;
  it.t = t; memcpy(it.src, src, 6); it.len = (uint8_t)len; memcpy(it.data, data, len);
  xQueueSend(rxq, &it, 0);
}

static void add_peer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;              // 0 = 現在のチャンネル
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static void learn_head(const uint8_t* mac) {
  if (head_known && memcmp(head_mac, mac, 6) == 0) return;
  if (head_known) esp_now_del_peer(head_mac);     // ヘッドが替わった
  memcpy(head_mac, mac, 6);
  head_known = true;
  add_peer(head_mac);
  Serial.printf("EVT head = %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void send_head(const void* p, size_t n) {
  const uint8_t* dst = head_known ? head_mac : BCAST;
  for (int k = 0; k < 3; k++) {
    esp_err_t e = esp_now_send(dst, (const uint8_t*)p, n);
    if (e == ESP_OK) return;
    delay(2);                                      // NO_MEM のときは少し待って再送
  }
}

static void send_hello() {
  MsgHello m = {};
  hdr_init(m.h, UNAGI_MAGIC_NODE, M_HELLO, NODE_ID, 0);
  m.fw = FW_VERSION; m.proto = UNAGI_PROTO_VER; m.state = state;
  memcpy(m.c, CAL, sizeof(CAL)); m.min_duty = MIN_DUTY;
  send_head(&m, sizeof(m));
}

// 10台が同時に返すと衝突するので ID ごとにずらす
static inline void stagger() { delay(2 + 4 * (NODE_ID ? NODE_ID : 11)); }

static void do_peek(uint16_t n) {
  if (n < 1) n = 1;
  if (n > 500) n = 500;
  double s1 = 0, s2 = 0, sd = 0, sdd = 0;
  for (uint16_t i = 0; i < n; i++) {
    int16_t v1, v2; read_hall(v1, v2);
    double d = (double)v1 - v2;
    s1 += v1; s2 += v2; sd += d; sdd += d * d;
    delayMicroseconds(800);
  }
  sync_fit();
  MsgPeek m = {};
  hdr_init(m.h, UNAGI_MAGIC_NODE, M_PEEK, NODE_ID, armed_trial);
  m.state = state; m.n = n;
  m.v1 = s1 / n; m.v2 = s2 / n; m.diff = sd / n;
  double var = sdd / n - (sd / n) * (sd / n);
  m.diff_sd = var > 0 ? sqrt(var) : 0;
  m.deg = poly_deg(CAL, m.diff);
  m.sigma_us = fit_sigma; m.skew_ppm = (float)((fit_b - 1.0) * 1e6); m.sync_n = fit_n;
  stagger();
  send_head(&m, sizeof(m));
}

static void handle_packet(const RxItem& it) {
  Hdr h; memcpy(&h, it.data, sizeof(h));
  learn_head(it.src);

  // ビーコンと DISCOVER と STOP は ID 未設定でも処理する
  switch (h.type) {
    case M_BEACON: {
      if (it.len < sizeof(MsgBeacon)) return;
      MsgBeacon m; memcpy(&m, it.data, sizeof(m));
      sync_add(it.t, m.t_head_us, m.seq);
      return;
    }
    case M_DISCOVER:
      stagger();
      send_hello();
      return;
    case M_STOP:
      if (state == S_RUN || state == S_WAIT) stop_req = true;
      if (state == S_ARMED) state = S_IDLE;
      return;
  }

  if (NODE_ID == 0) return;                                // 以降は ID 必須
  bool for_me = (h.node_id == NODE_ID);

  switch (h.type) {
    case M_PARAM: {
      if (!for_me || it.len < sizeof(MsgParam)) return;
      if (state == S_RUN || state == S_WAIT) return;
      MsgParam m; memcpy(&m, it.data, sizeof(m));
      par = m.p; par_valid = true; par_crc = crc16(&par, sizeof(par));
      if (state == S_ARMED) state = S_IDLE;                // パラメータが変わったら要 ARM
      MsgParamAck a = {};
      hdr_init(a.h, UNAGI_MAGIC_NODE, M_PARAM_ACK, NODE_ID, 0);
      a.crc = par_crc; a.mode = par.mode;
      send_head(&a, sizeof(a));
      return;
    }
    case M_ARM: {
      if (state == S_RUN || state == S_WAIT) return;
      bool ok = sync_fit();
      int16_t v1, v2; read_hall(v1, v2);
      MsgReady r = {};
      hdr_init(r.h, UNAGI_MAGIC_NODE, M_READY, NODE_ID, h.trial);
      r.param_crc = par_crc; r.sync_n = fit_n; r.sigma_us = fit_sigma;
      r.skew_ppm = (float)((fit_b - 1.0) * 1e6);
      r.free_heap = ESP.getFreeHeap(); r.v1 = v1; r.v2 = v2;
      if (s_count < SYNC_MIN_PTS) r.why = WHY_NO_SYNC;
      else if (!ok)               r.why = WHY_BAD_SYNC;
      else if (!par_valid)        r.why = WHY_NO_PARAM;
      else                        r.why = WHY_OK;
      r.ready = (r.why == WHY_OK);
      if (r.ready) { armed_trial = h.trial; state = S_ARMED; }
      else if (state == S_ARMED) state = S_IDLE;
      stagger();
      send_head(&r, sizeof(r));
      return;
    }
    case M_START: {
      if (it.len < sizeof(MsgStart)) return;
      if (state != S_ARMED || h.trial != armed_trial) return;   // 2回目以降・別 trial は無視
      MsgStart m; memcpy(&m, it.data, sizeof(m));
      int64_t tl = head_to_local(m.t_start_head_us);
      int64_t now = esp_timer_get_time();
      if (tl < now + 2000 || tl > now + 10000000) {       // 過去 or 10 s 以上先は拒否
        Serial.printf("EVT START rejected (t-now = %lld us)\n", (long long)(tl - now));
        state = S_IDLE;
        return;
      }
      run_par = par;
      memcpy(run_cal, CAL, sizeof(CAL));
      run_md = MIN_DUTY;
      run_b = fit_b;
      t_start_local = tl;
      stop_req = false;
      report_pending = false;
      state = S_WAIT;
      xTaskNotifyGive(ctrl_task);
      return;
    }
    case M_DUMP_REQ: {
      if (!for_me || it.len < sizeof(MsgDumpReq)) return;
      if (state == S_RUN || state == S_WAIT) return;
      MsgDumpReq q; memcpy(&q, it.data, sizeof(q));
      MsgDumpData d = {};
      hdr_init(d.h, UNAGI_MAGIC_NODE, M_DUMP_DATA, NODE_ID, armed_trial);
      d.block = q.block; d.n_total = (uint16_t)n_log;
      uint32_t off = (uint32_t)q.block * DUMP_PER_BLOCK;
      uint32_t n = (off >= n_log) ? 0 : (n_log - off);
      if (n > DUMP_PER_BLOCK) n = DUMP_PER_BLOCK;
      d.n = (uint8_t)n;
      memcpy(d.s, &logbuf[off], n * sizeof(LogSample));
      send_head(&d, sizeof(d));
      return;
    }
    case M_CAL_SET: {
      if (!for_me || it.len < sizeof(MsgCalSet)) return;
      if (state == S_RUN || state == S_WAIT) return;       // 走行中にフラッシュを書かない
      MsgCalSet m; memcpy(&m, it.data, sizeof(m));
      if (m.flags & CAL_F_SET_COEF) memcpy(CAL, m.c, sizeof(CAL));
      if (m.flags & CAL_F_SET_MD)   MIN_DUTY = constrain(m.min_duty, 0.0f, 0.9f);
      if (m.flags & CAL_F_ZERO) {
        double sd = 0;
        for (int i = 0; i < 64; i++) { int16_t a, b; read_hall(a, b); sd += (double)a - b; delay(1); }
        CAL[0] -= poly_deg(CAL, (float)(sd / 64.0));
      }
      save_cfg();
      MsgCalAck a = {};
      hdr_init(a.h, UNAGI_MAGIC_NODE, M_CAL_ACK, NODE_ID, 0);
      memcpy(a.c, CAL, sizeof(CAL)); a.min_duty = MIN_DUTY;
      send_head(&a, sizeof(a));
      Serial.printf("EVT CAL saved {%g, %g, %g, %g} md=%.3f\n", CAL[0], CAL[1], CAL[2], CAL[3], MIN_DUTY);
      return;
    }
    case M_PEEK_REQ: {
      if (!for_me || it.len < sizeof(MsgPeekReq)) return;
      if (state == S_RUN || state == S_WAIT) return;
      MsgPeekReq q; memcpy(&q, it.data, sizeof(q));
      do_peek(q.n_avg);
      return;
    }
  }
}

// ------------------------------ control task -------------------------
static void control_task(void*) {
  for (;;) {
    if (state != S_WAIT) { ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)); continue; }

    // ---- T まで待つ: 大半は眠り、最後の 3 ms だけスピン ----
    const int64_t t0 = t_start_local;
    for (;;) {
      int64_t rem = t0 - esp_timer_get_time();
      if (stop_req || state != S_WAIT || rem <= 3000) break;
      vTaskDelay(pdMS_TO_TICKS((rem - 2500) / 1000));
    }
    if (stop_req || state != S_WAIT) { stop_req = false; state = S_IDLE; continue; }
    while (esp_timer_get_time() < t0) { }
    const int64_t t_act = esp_timer_get_time();
    digitalWrite(PIN_SYNC_OUT, HIGH);

    const ParamBody& P = run_par;
    memset(&report, 0, sizeof(report));
    report.start_err_us = (int32_t)(t_act - t0);
    n_log = 0;

    if (P.mode == MODE_SYNC_TEST) {
      delayMicroseconds(2000);
      digitalWrite(PIN_SYNC_OUT, LOW);
      state = S_DONE; report_pending = true;
      continue;
    }

    const bool closed = (P.mode == MODE_SWIM || P.mode == MODE_STEP);
    const bool drives = (P.mode != MODE_HALL_RAW);
    uint32_t div = (P.log_hz > 0 && P.log_hz <= CTRL_HZ) ? (CTRL_HZ / P.log_hz) : 1;
    if (div == 0) div = 1;
    const float dur_s = P.duration_ms / 1000.0f;
    const float w = 2.0f * PI * P.f_hz;
    const float phi = P.phi_deg * (PI / 180.0f);
    const float i_lim = (P.ki > 1e-6f) ? (1.0f / P.ki) : 0.0f;

    float integ = 0, dfilt = 0, meas_prev = 0, err_sq = 0;
    uint32_t err_n = 0, over_ms = 0;
    set_enable(drives);
    state = S_RUN;

    TickType_t wake = xTaskGetTickCount();
    uint32_t k = 0;
    int64_t prev = t_act;
    for (;;) {
      if (k) vTaskDelayUntil(&wake, 1);
      const int64_t now  = esp_timer_get_time();
      const int64_t el   = now - t0;                              // ローカル経過 [us]
      const double  t_us = (double)el * run_b;                    // ヘッド時間換算 [us]
      const float   t    = (float)(t_us * 1e-6);                  // [s]
      // CPG の位相は実時刻 t から計算するので、tick の揺れは位相誤差として積み上がらない。

      // ジッタ: 周期 (1 ms) からのずれ。1.5 周期を超えたら取りこぼし
      if (k) {
        int32_t per = (int32_t)(now - prev);
        int32_t jit = per - 1000000 / CTRL_HZ; if (jit < 0) jit = -jit;
        if ((uint32_t)jit > report.max_jitter_us) report.max_jitter_us = jit;
        if (per > (3 * 1000000) / (2 * CTRL_HZ)) report.missed++;
      }
      prev = now;

      if (k == 2) digitalWrite(PIN_SYNC_OUT, LOW);

      if (stop_req) { report.abort_reason = ABORT_STOP_MSG; break; }
      if (t >= dur_s) break;

      int16_t v1, v2; read_hall(v1, v2);
      const float meas = poly_deg(run_cal, (float)(v1 - v2));
      float cmd = 0, u = 0;

      switch (P.mode) {
        case MODE_SWIM:
          cmd = P.A * sinf(w * t - phi) + P.off;
          break;
        case MODE_STEP:
          cmd = (t < 0.2f) ? P.off : (P.off + P.aux);
          break;
        case MODE_OPENLOOP:
          u = P.A * sinf(w * t - phi) + P.off;
          break;
        case MODE_PWM_SWEEP: {                                   // 0 → +aux → −aux → 0
          float x = t / dur_s;
          float tri = (x < 0.25f) ? 4 * x : (x < 0.75f) ? 2 - 4 * x : 4 * x - 4;
          u = P.aux * tri;
          break;
        }
        default: break;                                          // HALL_RAW
      }

      if (closed) {
        const float dt = 1.0f / CTRL_HZ;
        const float e = cmd - meas;
        integ += e * dt;
        if (i_lim > 0) integ = constrain(integ, -i_lim, i_lim); else integ = 0;
        // 微分は計測値に掛ける（目標のステップで蹴らない）+ 1 次ローパス
        const float dm = (k ? (meas - meas_prev) / dt : 0.0f);
        dfilt += 0.2f * (dm - dfilt);
        meas_prev = meas;
        u = P.kp * e + P.ki * integ - P.kd * dfilt;
        err_sq += e * e; err_n++;
        if (fabsf(meas) > ANGLE_LIMIT_DEG) { if (++over_ms > 50) { report.abort_reason = ABORT_ANGLE; break; } }
        else over_ms = 0;
      }
      if (drives) drive(u, run_md);

      if ((k % div) == 0) {
        if (n_log < UNAGI_MAX_SAMPLES) {
          LogSample& s = logbuf[n_log];
          s.t_us = (uint32_t)t_us;
          s.cmd  = (int16_t)lroundf(constrain(cmd,  -320.0f, 320.0f) * 100.0f);
          s.meas = (int16_t)lroundf(constrain(meas, -320.0f, 320.0f) * 100.0f);
          s.u    = (int16_t)lroundf(constrain(u, -1.0f, 1.0f) * 10000.0f);
          s.v1 = v1; s.v2 = v2;
          n_log = n_log + 1;
        }
      }
      const uint32_t us = (uint32_t)(esp_timer_get_time() - now);
      if (us > report.loop_us_max) report.loop_us_max = (uint16_t)(us > 65535 ? 65535 : us);
      k++;
    }

    motor_off();
    digitalWrite(PIN_SYNC_OUT, LOW);
    report.ticks = k;
    report.n_samples = n_log;
    report.rms_err_deg = err_n ? sqrtf(err_sq / err_n) : 0.0f;
    stop_req = false;
    state = S_DONE;
    report_pending = true;
  }
}

// ------------------------------ serial CLI ---------------------------
static bool     hall_print = false;
static uint32_t hall_print_ms = 0;
static uint32_t pwm_until_ms = 0;

static void print_show() {
  sync_fit();
  int16_t v1, v2; read_hall(v1, v2);
  uint8_t ch = 0; wifi_second_chan_t sc;
  esp_wifi_get_channel(&ch, &sc);
  Serial.printf("id=%d  mac=%s  ch=%d  fw=%08X  proto=%d  state=%s\n",
                NODE_ID, WiFi.macAddress().c_str(), ch, FW_VERSION, UNAGI_PROTO_VER, STATE_NAME[state]);
  if (head_known)
    Serial.printf("head=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  head_mac[0], head_mac[1], head_mac[2], head_mac[3], head_mac[4], head_mac[5]);
  else
    Serial.println("head=(未受信)  → ヘッドの電源 / チャンネルを確認");
  Serial.printf("CAL={%g, %g, %g, %g}  minduty=%.3f\n", CAL[0], CAL[1], CAL[2], CAL[3], MIN_DUTY);
  Serial.printf("sync: n=%u  sigma=%.1f us  skew=%.2f ppm  ok=%d\n",
                fit_n, fit_sigma, (fit_b - 1.0) * 1e6, fit_ok ? 1 : 0);
  Serial.printf("param: valid=%d crc=%u mode=%d\n", par_valid ? 1 : 0, par_crc, par.mode);
  Serial.printf("hall v1=%d v2=%d diff=%d deg=%.2f   freeheap=%u\n",
                v1, v2, v1 - v2, poly_deg(CAL, (float)(v1 - v2)), (unsigned)ESP.getFreeHeap());
}

static void handle_cli(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  for (char* p = cmd; *p; ++p) *p = tolower(*p);
  char* a1 = strtok(nullptr, " \t");
  bool busy = (state == S_RUN || state == S_WAIT);

  if (!strcmp(cmd, "id")) {
    int v = a1 ? atoi(a1) : -1;
    if (v >= 1 && v <= UNAGI_N_NODES) { NODE_ID = (uint8_t)v; Serial.printf("id = %d  (save で保存)\n", v); }
    else Serial.println("id は 1..10");
  } else if (!strcmp(cmd, "cal")) {
    float c[4]; int n = 0;
    for (char* a = a1; a && n < 4; a = strtok(nullptr, " \t")) c[n++] = atof(a);
    if (n == 4) { memcpy(CAL, c, sizeof(CAL)); Serial.println("cal 設定  (save で保存)"); }
    else Serial.println("cal <c0> <c1> <c2> <c3>");
  } else if (!strcmp(cmd, "minduty")) {
    MIN_DUTY = constrain(a1 ? atof(a1) : 0.0f, 0.0f, 0.9f);
    Serial.printf("minduty = %.3f  (save で保存)\n", MIN_DUTY);
  } else if (!strcmp(cmd, "zero")) {
    if (busy) { Serial.println("走行中は使えません"); return; }
    double sd = 0;
    for (int i = 0; i < 64; i++) { int16_t a, b; read_hall(a, b); sd += (double)a - b; delay(1); }
    CAL[0] -= poly_deg(CAL, (float)(sd / 64.0));
    Serial.printf("今の姿勢を 0 deg にしました  C0=%g  (save で保存)\n", CAL[0]);
  } else if (!strcmp(cmd, "save")) {
    if (busy) { Serial.println("走行中は保存できません"); return; }
    save_cfg(); Serial.println("NVS に保存しました");
  } else if (!strcmp(cmd, "show")) {
    if (busy) { Serial.printf("state=%s（走行中）\n", STATE_NAME[state]); return; }
    print_show();
  } else if (!strcmp(cmd, "hall")) {
    hall_print = !hall_print;
    Serial.printf("hall 表示 %s\n", hall_print ? "ON" : "OFF");
  } else if (!strcmp(cmd, "pwm")) {
    if (busy) { Serial.println("走行中は使えません"); return; }
    float u = a1 ? constrain(atof(a1), -1.0f, 1.0f) : 0.0f;
    char* a2 = strtok(nullptr, " \t");
    uint32_t ms = a2 ? (uint32_t)constrain(atoi(a2), 10, 5000) : 500;
    set_enable(true); drive(u, MIN_DUTY);
    pwm_until_ms = millis() + ms;
    Serial.printf("pwm %.3f for %u ms\n", u, (unsigned)ms);
  } else if (!strcmp(cmd, "hello")) {
    send_hello(); Serial.println("HELLO sent");
  } else {
    Serial.println("commands: id <n> | cal <c0> <c1> <c2> <c3> | minduty <x> | zero | save");
    Serial.println("          show | hall | pwm <u> [ms] | hello");
  }
}

static void poll_cli() {
  static char buf[96]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; if (n) handle_cli(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

// ------------------------------ setup / loop -------------------------
void setup() {
  pinMode(PIN_EN, OUTPUT);       set_enable(false);
  pinMode(PIN_SYNC_OUT, OUTPUT); digitalWrite(PIN_SYNC_OUT, LOW);
  pwm_init();
  pwm_write(0, 0);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ESP_ARDUINO_VERSION_MAJOR < 3
  Serial.setTxTimeoutMs(0);      // USB 未接続（電源ケーブル給電）でも止まらない
#endif
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_HALL1, ADC_11db);
  analogSetPinAttenuation(PIN_HALL2, ADC_11db);

  load_cfg();

  // --- WiFi: STA で起動するが AP には繋がない。チャンネルを固定 ---
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);  // 過去のファームが保存した AP 情報で勝手に繋ぎに行かせない
  delay(50);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(UNAGI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  rxq = xQueueCreate(24, sizeof(RxItem));
  if (esp_now_init() != ESP_OK) Serial.println("ERR esp_now_init");
  esp_now_register_recv_cb(on_recv);
  add_peer(BCAST);

  xTaskCreate(control_task, "ctrl", 6144, nullptr, configMAX_PRIORITIES - 1, &ctrl_task);

  uint8_t ch = 0; wifi_second_chan_t sc; esp_wifi_get_channel(&ch, &sc);
  Serial.printf("\nUNAGI node  id=%d  mac=%s  ch=%d  fw=%08X\n",
                NODE_ID, WiFi.macAddress().c_str(), ch, FW_VERSION);
  if (NODE_ID == 0) Serial.println("★ ID 未設定です。 'id <1-10>' → 'save' を実行してください。");

  // 起動の自己紹介（ヘッドが MAC を覚える）。他ノードは magic で無視する。
  delay(100 + 15 * NODE_ID);
  for (int k = 0; k < 3; k++) { send_hello(); delay(120); }
}

void loop() {
  RxItem it;
  while (xQueueReceive(rxq, &it, 0) == pdTRUE) handle_packet(it);

  if (report_pending) {
    report_pending = false;
    hdr_init(report.h, UNAGI_MAGIC_NODE, M_REPORT, NODE_ID, armed_trial);
    stagger();
    send_head(&report, sizeof(report));
    Serial.printf("EVT run done  n=%u ticks=%u missed=%u jit=%uus loop=%uus abort=%u start_err=%dus\n",
                  (unsigned)report.n_samples, (unsigned)report.ticks, (unsigned)report.missed,
                  (unsigned)report.max_jitter_us, report.loop_us_max, report.abort_reason,
                  (int)report.start_err_us);
  }

  poll_cli();

  const uint32_t now = millis();
  if (pwm_until_ms && (int32_t)(now - pwm_until_ms) >= 0) { motor_off(); pwm_until_ms = 0; Serial.println("pwm off"); }
  const bool busy = (state == S_RUN || state == S_WAIT);   // 走行中は ADC を制御タスク専用にする
  if (hall_print && !busy && now - hall_print_ms >= 100) {
    hall_print_ms = now;
    int16_t v1, v2; read_hall(v1, v2);
    Serial.printf("v1=%4d v2=%4d diff=%5d deg=%7.2f\n", v1, v2, v1 - v2, poly_deg(CAL, (float)(v1 - v2)));
  }
  delay(1);
}
