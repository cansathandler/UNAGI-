// =====================================================================
//  UNAGI モータノード  (XIAO ESP32-C3 + Pololu DRV8833 並列 + SS39ET ×2 + SP3485)
//  protocol v3 (RS-485) / 10台に同じバイナリを書き込む。ID と較正は NVS に保存。
//
//  配線（ノード基板 V3）
//    D2  (GPIO4)  --> DRV8833 IN1 (AIN1+BIN1 並列)   PWM
//    D3  (GPIO5)  --> DRV8833 IN2 (AIN2+BIN2 並列)   PWM
//    D5  (GPIO7)  <-- DRV8833 nFAULT（オープンドレイン、内部プルアップ使用）
//    A0  (GPIO2)  <-- SS39ET S1  (ADC1)
//    A1  (GPIO3)  <-- SS39ET S2  (ADC1)
//    D4  (GPIO6)  --> SP3485 DE + /RE   （基板上 10 kΩ プルダウン = 受信）
//    D6  (GPIO21) --> SP3485 DI   (UART1 TX)
//    D7  (GPIO20) <-- SP3485 RO   (UART1 RX、基板上 10 kΩ プルアップ)
//    D10 (GPIO10) --> SYNC パルス（T の瞬間に 2 ms High。未接続でも可）
//    DRV8833 SLP はキャリア基板上でプルアップ（常に有効）。止めるときは IN1=IN2=0（惰性）。
//
//  USB シリアル (115200) のコマンド:  help で一覧
//    id <1-10> / cal c0 c1 c2 c3 / minduty x / dir <1|-1> / zero / save / show
//    hall        ホール生値を 10 Hz で表示（もう一度で停止）
//    pwm <u> [ms] 開ループで u (−1..1) を ms だけ出す（配線確認用）
//    bus         RS-485 の受信カウンタ
//
//  v2 (ESP-NOW 版) からの変更
//   1. 通信を ESP-NOW → RS-485 (Serial1, 1 Mbps)。WiFi は使わない。
//   2. ノードは「自分宛て」の要求にだけ返事をする。stagger（ID ずらし）は廃止。
//      全体宛て（node_id 0）の BEACON / START / STOP には返事をしない。
//   3. REPORT は自動送信しない。ヘッドの REPORT_REQ に返す。
//   4. ピン配置を V3 基板に合わせた（旧 D2=EN は廃止、D2/D3 が PWM）。
//   5. nFAULT が 5 ms 以上 Low なら走行を止める（ABORT_FAULT）。ARM 時に Low なら WHY_FAULT。
//   6. 回転方向の符号 dir を NVS に保存（モータ配線が逆の個体用）。
// =====================================================================

#include <Arduino.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <Preferences.h>
#include <math.h>
#include "unagi_protocol.h"
#include "unagi_bus.h"

#define FW_VERSION      0x00030002u

// 校正エンジン（HALL ストリーム・波形駆動・GETCAL/SETCAL…のテキストプロトコル）
//   UNAGI_CAL_WIFI  : ノード自身の WiFi AP から使う（env:node_calwifi）
//   UNAGI_CAL_BUS   : ヘッドの校正ブリッジ（WiFi UNAGI-HEAD, TCP 5000）から RS-485 で使う（既定で有効）
//   走行（S_WAIT / S_RUN）中はどちらも駆動・ストリームを受け付けない。
#ifndef UNAGI_CAL_BUS
#define UNAGI_CAL_BUS 1
#endif
#if defined(UNAGI_CAL_WIFI) || UNAGI_CAL_BUS
#define CAL_ENGINE 1
#else
#define CAL_ENGINE 0
#endif

// ------------------------------ pins ---------------------------------
#define PIN_HALL1       2      // A0  ADC1_CH2
#define PIN_HALL2       3      // A1  ADC1_CH3
#define PIN_PWM1        4      // D2  → DRV8833 IN1
#define PIN_PWM2        5      // D3  → DRV8833 IN2
#define PIN_BUS_DE      6      // D4  → SP3485 DE//RE
#define PIN_FLT         7      // D5  ← DRV8833 nFAULT
#define PIN_SYNC_OUT   10      // D10
#define PIN_BUS_TX     21      // D6  → SP3485 DI
#define PIN_BUS_RX     20      // D7  ← SP3485 RO

static const bool SLOW_DECAY = true;

// ------------------------------ control ------------------------------
#define CTRL_HZ         1000
#define PWM_HZ          25000
#define PWM_BITS        10
#define OVERSAMPLE      4
#define ANGLE_LIMIT_DEG 45.0f   // 閉ループ中にこれを 50 ms 超えたら停止
#define FAULT_LIMIT_MS  5       // nFAULT がこれを超えて Low なら停止
#define SYNC_RING       400     // 20 Hz × 20 s
#define SYNC_MIN_PTS    40
#define SYNC_MAX_SIGMA  500.0f  // us
#define REPLY_GUARD_US  100     // 返事の前に空ける時間（ヘッドの DE が確実に落ちてから送る）

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ------------------------------ PWM ----------------------------------
static const int PWM_MAX = (1 << PWM_BITS) - 1;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwm_init() {
  ledcAttach(PIN_PWM1, PWM_HZ, PWM_BITS);
  ledcAttach(PIN_PWM2, PWM_HZ, PWM_BITS);
}
static inline void pwm_write(uint32_t a, uint32_t b) { ledcWrite(PIN_PWM1, a); ledcWrite(PIN_PWM2, b); }
#else
static void pwm_init() {
  ledcSetup(0, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_PWM1, 0);
  ledcSetup(1, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_PWM2, 1);
}
static inline void pwm_write(uint32_t a, uint32_t b) { ledcWrite(0, a); ledcWrite(1, b); }
#endif

// ------------------------------ NVS ----------------------------------
// 名前空間とキーは v1/v2 と同じ。既に書いた ID と較正はそのまま残る。
static Preferences prefs;
static uint8_t NODE_ID  = 0;
static float   CAL[4]   = {0.0f, 0.02f, 0.0f, 0.0f};   // theta = C0 + C1 d + C2 d^2 + C3 d^3
// d = s − K_COIL·u,  s = v1 − v2 (HALL_SIG 0, 既定 = 従来どおり) または v1 + v2 (HALL_SIG 1)
// u は「今コイルに流れている」指令（直前に出力した値、dir を掛ける前、−1..1）
static uint8_t HALL_SIG = 0;
static float   K_COIL   = 0.0f;                         // counts per unit u
// 電流制限: すべての駆動で |u| <= ULIM（関節固定・最大電源電圧で測った電流から決める。既定 1 = 制限なし）
static float   ULIM     = 1.0f;
static float   MIN_DUTY = 0.0f;
static int8_t  MOTOR_DIR = 1;                           // +1 / −1（u の符号を反転する）

static void load_cfg() {
  prefs.begin("unagi", true);
  NODE_ID   = prefs.getUChar("id", 0);
  CAL[0]    = prefs.getFloat("c0", 0.0f);
  CAL[1]    = prefs.getFloat("c1", 0.02f);
  CAL[2]    = prefs.getFloat("c2", 0.0f);
  CAL[3]    = prefs.getFloat("c3", 0.0f);
  MIN_DUTY  = prefs.getFloat("md", 0.0f);
  MOTOR_DIR = prefs.getChar("dir", 1) < 0 ? -1 : 1;
  HALL_SIG  = prefs.getUChar("sig", 0) ? 1 : 0;
  K_COIL    = prefs.getFloat("kc", 0.0f);
  ULIM      = constrain(prefs.getFloat("ul", 1.0f), 0.0f, 1.0f);
  prefs.end();
  if (NODE_ID > UNAGI_N_NODES) NODE_ID = 0;
}
static void save_cfg() {
  prefs.begin("unagi", false);
  prefs.putUChar("id", NODE_ID);
  prefs.putFloat("c0", CAL[0]); prefs.putFloat("c1", CAL[1]);
  prefs.putFloat("c2", CAL[2]); prefs.putFloat("c3", CAL[3]);
  prefs.putFloat("md", MIN_DUTY);
  prefs.putChar("dir", MOTOR_DIR);
  prefs.putUChar("sig", HALL_SIG);
  prefs.putFloat("kc", K_COIL);
  prefs.putFloat("ul", ULIM);
  prefs.end();
}

// ------------------------------ state --------------------------------
enum : uint8_t { S_IDLE = 0, S_ARMED = 1, S_WAIT = 2, S_RUN = 3, S_DONE = 4 };
static const char* STATE_NAME[] = {"IDLE", "ARMED", "WAIT", "RUN", "DONE"};
static volatile uint8_t state = S_IDLE;
static volatile bool    stop_req = false;

static uint8_t   my_mac[6];
static uint32_t  last_head_ms = 0;        // 最後にヘッドのフレームを受けた時刻

static ParamBody par;
static bool      par_valid = false;
static uint16_t  par_crc = 0;
static uint16_t  armed_trial = 0;

// run-local copies (control task only reads these)
static ParamBody run_par;
static float     run_cal[4];
static float     run_md;
static uint8_t   run_sig = 0;
static float     run_k = 0.0f;
static float     run_ulim = 1.0f;
static int8_t    run_dir = 1;
static double    run_b = 1.0;
static int64_t   t_start_local = 0;

static LogSample logbuf[UNAGI_MAX_SAMPLES];
static volatile uint32_t n_log = 0;

static MsgReport report;
static volatile bool report_valid = false;   // 走行完了。REPORT_REQ に返せる
static volatile bool run_done_evt = false;   // USB に結果を1回表示する

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
// 角度用の信号（上の d）。u は直前に出力した指令（待機中は 0）
static inline float hall_sig(int16_t v1, int16_t v2, uint8_t sig, float k, float u) {
  const float s = sig ? ((float)v1 + (float)v2) : ((float)v1 - (float)v2);
  return s - k * u;
}
static inline float poly_deg(const float* c, float d) {
  return c[0] + d * (c[1] + d * (c[2] + d * c[3]));
}
static inline bool fault_active() { return digitalRead(PIN_FLT) == LOW; }

// u in [−1, 1]。スローディケイ + デッドゾーン補償。dir で符号反転。
static inline void drive(float u, float md, int8_t dir) {
  if (dir < 0) u = -u;
  if (u >  1.0f) u =  1.0f;
  if (u < -1.0f) u = -1.0f;
  float mag = fabsf(u);
  mag = (mag > 1e-4f) ? (md + mag * (1.0f - md)) : 0.0f;
  uint32_t d = (uint32_t)lroundf(mag * PWM_MAX);
  if (d == 0) { pwm_write(0, 0); return; }               // 0 は惰性（コースト）
  if (SLOW_DECAY) {
    if (u >= 0) pwm_write(PWM_MAX, PWM_MAX - d);
    else        pwm_write(PWM_MAX - d, PWM_MAX);
  } else {
    if (u >= 0) pwm_write(d, 0);
    else        pwm_write(0, d);
  }
}
static inline void motor_off() { pwm_write(0, 0); }

// =====================================================================
//  校正用 WiFi モード（env:node_calwifi でだけコンパイルされる）
//  ノード自身が SoftAP "UNAGI-NODE-<id>" / unagi12345 になり、192.168.4.1:5000 で
//  旧 hall_motor_node と同じテキストプロトコルを話す → hall_calib_gui.py がそのまま使える。
//    HALL 1|0            1 kHz で  time_us,v1,v2,u_milli  を流す（STREAM_START / STREAM_END 付き）
//    WAVE DC|SINE|SQUARE|TRI, AMP, FREQ, DUTY, PHASE, OFFSET   波形パラメータ
//    RUN / STOP          開ループ駆動（u は dir 適用前の値。+u で角度が増える向き）
//    RUNMAX <ms>         RUN の自動停止時間（既定 15000 ms）
//    STATUS / PING / HELP
//  安全: TCP 切断・nFAULT・RUNMAX 経過で必ず停止。走行（S_WAIT/S_RUN）中は何もしない。
//  RS-485 版（UNAGI_CAL_BUS）: ヘッドの校正ブリッジが同じテキストを CALX_REQ で運ぶ。
// =====================================================================
#if CAL_ENGINE
#ifdef UNAGI_CAL_WIFI
#include <WiFi.h>
#endif

struct CalSample { uint32_t t; int16_t v1, v2, u; };
enum : uint8_t { CW_DC = 0, CW_SINE, CW_SQUARE, CW_TRI };
static const char* CW_NAME[] = {"DC", "SINE", "SQUARE", "TRI"};

static volatile bool     cal_stream = false;     // ホール値を流す
static volatile bool     cal_run    = false;     // 波形で駆動中
static volatile bool     cal_evt_autostop = false;
static volatile uint8_t  cal_stop_why = 0;       // 1 = RUNMAX, 2 = nFAULT
static volatile uint32_t cal_dropped = 0;
static QueueHandle_t     cal_q = nullptr;
static int64_t           cal_t0 = 0;
static uint32_t          cal_runmax_ms = 15000;
static struct {
  uint8_t wave = CW_SQUARE;
  float amp = 0.5f, freq = 1.0f, duty = 0.5f, phase = 0.0f, offset = 0.0f, limit = 1.0f;
} cal_p;

static inline float t_frac(float v) { v = fmodf(v, 1.0f); return v < 0.0f ? v + 1.0f : v; }

static float cal_wave(float th) {          // th: 0..1
  switch (cal_p.wave) {
    case CW_DC:     return 1.0f;
    case CW_SINE:   return sinf(2.0f * PI * th);
    case CW_SQUARE: return (th < cal_p.duty) ? 1.0f : -1.0f;
    case CW_TRI:    return (th < 0.5f) ? (4.0f * th - 1.0f) : (3.0f - 4.0f * th);
  }
  return 0.0f;
}

// control_task から呼ぶ。HALL 1 か RUN の間だけ 1 kHz で回る。
static void cal_tick_loop() {
  TickType_t wake = xTaskGetTickCount();
  bool drove = false;
  uint32_t flt_ms = 0;
  while ((cal_stream || cal_run) && state != S_WAIT && state != S_RUN) {
    vTaskDelayUntil(&wake, 1);
    const int64_t now = esp_timer_get_time();
    float u = 0.0f;
    if (cal_run) {
      const float t = (float)((now - cal_t0) * 1e-6);
      if (fault_active()) { if (++flt_ms > FAULT_LIMIT_MS) { cal_run = false; cal_stop_why = 2; cal_evt_autostop = true; } }
      else flt_ms = 0;
      if (t * 1000.0f >= (float)cal_runmax_ms) { cal_run = false; cal_stop_why = 1; cal_evt_autostop = true; }
    }
    if (cal_run) {
      float th = t_frac(((float)((now - cal_t0) * 1e-6)) * cal_p.freq - cal_p.phase / 360.0f);
      u = cal_p.amp * cal_wave(th) + cal_p.offset;
      const float lim = (cal_p.limit < ULIM) ? cal_p.limit : ULIM;   // 電流制限を適用
      u = constrain(u, -lim, lim);
      drive(u, MIN_DUTY, MOTOR_DIR);
      drove = true;
    } else if (drove) {
      motor_off();
      drove = false;
    }
    if (cal_stream && cal_q) {
      int16_t v1, v2; read_hall(v1, v2);
      CalSample s = {(uint32_t)now, v1, v2, (int16_t)lroundf(u * 1000.0f)};
      if (xQueueSend(cal_q, &s, 0) != pdTRUE) cal_dropped = cal_dropped + 1;
    }
  }
  if (drove) motor_off();
  cal_run = false;
}

// 走行中でない限り使える。どちらの経路で使っているか（同時に1つ）
enum : uint8_t { CAL_OWNER_NONE = 0, CAL_OWNER_WIFI = 1, CAL_OWNER_BUS = 2 };
static uint8_t  cal_owner = CAL_OWNER_NONE;
static uint32_t cal_last_bus_ms = 0;      // 最後に CALX_REQ を受けた時刻（途絶したら止める）
static void cal_cmd(char* line);
static void calx_on_req(const BusRx& it);
#endif  // CAL_ENGINE

// ------------------------------ RS-485 -------------------------------
static void reply(const void* p, size_t n) {
  delayMicroseconds(REPLY_GUARD_US);
  bus::send(p, n);
}

static void send_hello() {
  MsgHello m = {};
  hdr_init(m.h, UNAGI_MAGIC_NODE, M_HELLO, NODE_ID, 0);
  m.fw = FW_VERSION; m.proto = UNAGI_PROTO_VER; m.state = state;
  memcpy(m.c, CAL, sizeof(CAL)); m.min_duty = MIN_DUTY;
  memcpy(m.mac, my_mac, 6);
  reply(&m, sizeof(m));
}

static void do_peek(uint16_t n) {
  if (n < 1) n = 1;
  if (n > 500) n = 500;
  double s1 = 0, s2 = 0, sd = 0, sdd = 0;
  for (uint16_t i = 0; i < n; i++) {
    int16_t v1, v2; read_hall(v1, v2);
    double d = hall_sig(v1, v2, HALL_SIG, K_COIL, 0.0f);
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
  reply(&m, sizeof(m));
}

static void do_arm(uint16_t trial, bool answer) {
  bool ok = sync_fit();
  int16_t v1, v2; read_hall(v1, v2);
  MsgReady r = {};
  hdr_init(r.h, UNAGI_MAGIC_NODE, M_READY, NODE_ID, trial);
  r.param_crc = par_crc; r.sync_n = fit_n; r.sigma_us = fit_sigma;
  r.skew_ppm = (float)((fit_b - 1.0) * 1e6);
  r.free_heap = ESP.getFreeHeap(); r.v1 = v1; r.v2 = v2;
  if (s_count < SYNC_MIN_PTS) r.why = WHY_NO_SYNC;
  else if (!ok)               r.why = WHY_BAD_SYNC;
  else if (!par_valid)        r.why = WHY_NO_PARAM;
  else if (fault_active())    r.why = WHY_FAULT;
  else                        r.why = WHY_OK;
  r.ready = (r.why == WHY_OK);
  if (r.ready) { armed_trial = trial; state = S_ARMED; report_valid = false; }
  else if (state == S_ARMED) state = S_IDLE;
  if (answer) reply(&r, sizeof(r));
}

static void handle_packet(const BusRx& it) {
  if (it.len < sizeof(Hdr)) return;
  Hdr h; memcpy(&h, it.data, sizeof(h));
  last_head_ms = millis();

  // 全体宛て: ID 未設定でも処理する。返事はしない。
  switch (h.type) {
    case M_BEACON: {
      if (it.len < sizeof(MsgBeacon)) return;
      MsgBeacon m; memcpy(&m, it.data, sizeof(m));
      sync_add(it.t_us, m.t_head_us, m.seq);
      return;
    }
    case M_STOP:
      if (state == S_RUN || state == S_WAIT) stop_req = true;
      if (state == S_ARMED) state = S_IDLE;
      return;
    case M_DISCOVER:
      // node_id 0 の DISCOVER には ID 未設定のノードだけが返事をする
      if (h.node_id == NODE_ID) send_hello();
      return;
  }

  if (NODE_ID == 0) return;                                // 以降は ID 必須
  const bool for_me = (h.node_id == NODE_ID);
  const bool busy   = (state == S_RUN || state == S_WAIT);

  switch (h.type) {
    case M_PARAM: {
      if (!for_me || it.len < sizeof(MsgParam) || busy) return;
      MsgParam m; memcpy(&m, it.data, sizeof(m));
      par = m.p; par_valid = true; par_crc = crc16(&par, sizeof(par));
      if (state == S_ARMED) state = S_IDLE;                // パラメータが変わったら要 ARM
      MsgParamAck a = {};
      hdr_init(a.h, UNAGI_MAGIC_NODE, M_PARAM_ACK, NODE_ID, 0);
      a.crc = par_crc; a.mode = par.mode;
      reply(&a, sizeof(a));
      return;
    }
    case M_ARM: {
      if (busy) return;
      if (for_me)            do_arm(h.trial, true);
      else if (h.node_id == 0) do_arm(h.trial, false);     // 全体宛て ARM は黙って ARM する
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
      run_sig = HALL_SIG;
      run_k = K_COIL;
      run_ulim = ULIM;
      run_dir = MOTOR_DIR;
      run_b = fit_b;
      t_start_local = tl;
      stop_req = false;
      report_valid = false;
      state = S_WAIT;
      xTaskNotifyGive(ctrl_task);
      return;
    }
    case M_REPORT_REQ: {
      if (!for_me || !report_valid || h.trial != armed_trial) return;   // 未完なら無言
      hdr_init(report.h, UNAGI_MAGIC_NODE, M_REPORT, NODE_ID, armed_trial);
      reply(&report, sizeof(report));
      return;
    }
    case M_DUMP_REQ: {
      if (!for_me || it.len < sizeof(MsgDumpReq) || busy) return;
      MsgDumpReq q; memcpy(&q, it.data, sizeof(q));
      static MsgDumpData d; memset(&d, 0, sizeof(d));
      hdr_init(d.h, UNAGI_MAGIC_NODE, M_DUMP_DATA, NODE_ID, armed_trial);
      d.block = q.block; d.n_total = (uint16_t)n_log;
      uint32_t off = (uint32_t)q.block * DUMP_PER_BLOCK;
      uint32_t n = (off >= n_log) ? 0 : (n_log - off);
      if (n > DUMP_PER_BLOCK) n = DUMP_PER_BLOCK;
      d.n = (uint8_t)n;
      if (n) memcpy(d.s, &logbuf[off], n * sizeof(LogSample));
      // 送るのは実際に詰めたサンプルまで（最終ブロックを短くする）
      reply(&d, offsetof(MsgDumpData, s) + n * sizeof(LogSample));
      return;
    }
    case M_CAL_SET: {
      if (!for_me || it.len < sizeof(MsgCalSet) || busy) return;   // 走行中にフラッシュを書かない
      MsgCalSet m; memcpy(&m, it.data, sizeof(m));
      if (m.flags & CAL_F_SET_COEF) memcpy(CAL, m.c, sizeof(CAL));
      if (m.flags & CAL_F_SET_MD)   MIN_DUTY = constrain(m.min_duty, 0.0f, 0.9f);
      if (m.flags & CAL_F_ZERO) {
        double sd = 0;
        for (int i = 0; i < 64; i++) { int16_t a, b; read_hall(a, b); sd += hall_sig(a, b, HALL_SIG, K_COIL, 0.0f); delay(1); }
        CAL[0] -= poly_deg(CAL, (float)(sd / 64.0));
      }
      save_cfg();
      MsgCalAck a = {};
      hdr_init(a.h, UNAGI_MAGIC_NODE, M_CAL_ACK, NODE_ID, 0);
      memcpy(a.c, CAL, sizeof(CAL)); a.min_duty = MIN_DUTY;
      reply(&a, sizeof(a));
      Serial.printf("EVT CAL saved {%g, %g, %g, %g} md=%.3f\n", CAL[0], CAL[1], CAL[2], CAL[3], MIN_DUTY);
      return;
    }
    case M_PEEK_REQ: {
      if (!for_me || it.len < sizeof(MsgPeekReq) || busy) return;
      MsgPeekReq q; memcpy(&q, it.data, sizeof(q));
      do_peek(q.n_avg);
      return;
    }
#if UNAGI_CAL_BUS
    case M_CALX_REQ: {
      if (!for_me) return;
      calx_on_req(it);               // 走行中もポーリングには返事をする（コマンドは cal_cmd が busy で断る）
      return;
    }
#endif
  }
}

// ------------------------------ control task -------------------------
static void control_task(void*) {
  for (;;) {
    if (state != S_WAIT) {
#if CAL_ENGINE
      if (cal_stream || cal_run) { cal_tick_loop(); continue; }
#endif
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
      continue;
    }

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
      state = S_DONE; report_valid = true; run_done_evt = true;
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
    float u_applied = 0.0f;          // 今コイルに出ている指令（K_COIL 補正用、dir 前・−1..1）
    uint32_t err_n = 0, over_ms = 0, flt_ms = 0;
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

      // DRV8833 nFAULT（過電流 / 過熱 / 低電圧）
      if (drives && fault_active()) { if (++flt_ms > FAULT_LIMIT_MS) { report.abort_reason = ABORT_FAULT; break; } }
      else flt_ms = 0;

      int16_t v1, v2; read_hall(v1, v2);
      const float meas = poly_deg(run_cal, hall_sig(v1, v2, run_sig, run_k, u_applied));
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
      u = constrain(u, -run_ulim, run_ulim);                   // 電流制限（ログの u も制限後）
      if (drives) drive(u, run_md, run_dir);
      u_applied = drives ? u : 0.0f;

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
    report_valid = true;
    run_done_evt = true;
  }
}

// ------------------------------ serial CLI ---------------------------
static bool     hall_print = false;
static uint32_t hall_print_ms = 0;
static uint32_t pwm_until_ms = 0;

static void print_bus() {
  const BusStats& s = bus::st;
  Serial.printf("bus: %lu bps  rx_ok=%lu crc=%lu len=%lu other=%lu qfull=%lu uart_err=%lu tx=%lu  head=%s\n",
                (unsigned long)BUS_BAUD, (unsigned long)s.rx_ok, (unsigned long)s.rx_crc, (unsigned long)s.rx_len,
                (unsigned long)s.rx_other, (unsigned long)s.rx_qfull, (unsigned long)s.rx_uart_err,
                (unsigned long)s.tx,
                last_head_ms ? (millis() - last_head_ms < 500 ? "受信中" : "途絶") : "(未受信)");
}

static void print_show() {
  sync_fit();
  int16_t v1, v2; read_hall(v1, v2);
  Serial.printf("id=%d  mac=%02X:%02X:%02X:%02X:%02X:%02X  fw=%08X  proto=%d  state=%s\n",
                NODE_ID, my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5],
                FW_VERSION, UNAGI_PROTO_VER, STATE_NAME[state]);
  print_bus();
  if (!last_head_ms)
    Serial.println("  → ヘッドのビーコンが届いていません。A/B の向き・終端・ヘッドの電源を確認");
  Serial.printf("CAL={%g, %g, %g, %g}  minduty=%.3f  dir=%+d  sig=%s  k=%g  ulim=%.3f\n", CAL[0], CAL[1], CAL[2], CAL[3],
                MIN_DUTY, MOTOR_DIR, HALL_SIG ? "v1+v2" : "v1-v2", K_COIL, ULIM);
  Serial.printf("sync: n=%u  sigma=%.1f us  skew=%.2f ppm  ok=%d\n",
                fit_n, fit_sigma, (fit_b - 1.0) * 1e6, fit_ok ? 1 : 0);
  Serial.printf("param: valid=%d crc=%u mode=%d\n", par_valid ? 1 : 0, par_crc, par.mode);
  Serial.printf("hall v1=%d v2=%d diff=%d deg=%.2f   nFAULT=%s   freeheap=%u\n",
                v1, v2, v1 - v2, poly_deg(CAL, hall_sig(v1, v2, HALL_SIG, K_COIL, 0.0f)),
                fault_active() ? "LOW(異常)" : "high", (unsigned)ESP.getFreeHeap());
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
  } else if (!strcmp(cmd, "dir")) {
    int v = a1 ? atoi(a1) : 0;
    if (v == 1 || v == -1) { MOTOR_DIR = (int8_t)v; Serial.printf("dir = %+d  (save で保存)\n", v); }
    else Serial.println("dir <1|-1>   pwm +0.3 で角度 (deg) が増える向きが +1 になるように");
  } else if (!strcmp(cmd, "sig")) {
    if (a1 && (!strcmp(a1, "sum") || !strcmp(a1, "1"))) HALL_SIG = 1;
    else if (a1 && (!strcmp(a1, "diff") || !strcmp(a1, "0"))) HALL_SIG = 0;
    else { Serial.println("sig <diff|sum>   角度に使う信号: v1-v2 / v1+v2"); return; }
    Serial.printf("sig = %s  (save で保存)\n", HALL_SIG ? "v1+v2" : "v1-v2");
  } else if (!strcmp(cmd, "k")) {
    if (!a1) { Serial.println("k <counts per unit u>   コイル漏れ補正 d = s - k*u"); return; }
    K_COIL = atof(a1);
    Serial.printf("k = %g  (save で保存)\n", K_COIL);
  } else if (!strcmp(cmd, "ulim")) {
    if (!a1) { Serial.printf("ulim = %.3f   (ulim <0..1>  |u| の上限 = 電流制限)\n", ULIM); return; }
    ULIM = constrain(atof(a1), 0.0f, 1.0f);
    Serial.printf("ulim = %.3f  (save で保存)\n", ULIM);
  } else if (!strcmp(cmd, "zero")) {
    if (busy) { Serial.println("走行中は使えません"); return; }
    double sd = 0;
    for (int i = 0; i < 64; i++) { int16_t a, b; read_hall(a, b); sd += hall_sig(a, b, HALL_SIG, K_COIL, 0.0f); delay(1); }
    CAL[0] -= poly_deg(CAL, (float)(sd / 64.0));
    Serial.printf("今の姿勢を 0 deg にしました  C0=%g  (save で保存)\n", CAL[0]);
  } else if (!strcmp(cmd, "save")) {
    if (busy) { Serial.println("走行中は保存できません"); return; }
    save_cfg(); Serial.println("NVS に保存しました");
  } else if (!strcmp(cmd, "show")) {
    if (busy) { Serial.printf("state=%s（走行中）\n", STATE_NAME[state]); return; }
    print_show();
  } else if (!strcmp(cmd, "bus")) {
    print_bus();
  } else if (!strcmp(cmd, "hall")) {
    hall_print = !hall_print;
    Serial.printf("hall 表示 %s\n", hall_print ? "ON" : "OFF");
  } else if (!strcmp(cmd, "pwm")) {
    if (busy) { Serial.println("走行中は使えません"); return; }
    float u = a1 ? constrain(atof(a1), -ULIM, ULIM) : 0.0f;   // 電流制限を適用
    char* a2 = strtok(nullptr, " \t");
    uint32_t ms = a2 ? (uint32_t)constrain(atoi(a2), 10, 5000) : 500;
    drive(u, MIN_DUTY, MOTOR_DIR);
    pwm_until_ms = millis() + ms;
    Serial.printf("pwm %.3f for %u ms\n", u, (unsigned)ms);
  } else {
    Serial.println("commands: id <n> | cal <c0> <c1> <c2> <c3> | minduty <x> | dir <1|-1> | sig <diff|sum> | k <x> | ulim <x> | zero | save");
    Serial.println("          show | bus | hall | pwm <u> [ms]");
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

// ------------------------------ 校正エンジン（テキスト側） --------------
//  cal_cmd() が node_calwifi と同じテキストプロトコルを処理し、返事の行を cal_out に積む。
//  cal_out の中身は、WiFi なら TCP へ、RS-485 ならヘッドの CALX_REQ への返事に詰めて送る。
#if CAL_ENGINE
static char     cal_out[4096];
static size_t   cal_out_n = 0;
static bool     cal_end_pending = false;          // RS-485: キューが空になったら STREAM_END を出す

static void cal_write(const char* s, size_t n) {
  if (cal_out_n + n > sizeof(cal_out)) return;    // あふれたら捨てる（ストリーム中は行が少ないので起きない）
  memcpy(cal_out + cal_out_n, s, n);
  cal_out_n += n;
}
static void cal_printf(const char* fmt, ...) {
  char b[220];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(b) - 2) n = sizeof(b) - 2;
  b[n++] = '\n';
  cal_write(b, n);
}

static void cal_motor_stop() {
  cal_run = false;
  motor_off();
}

// 経路が変わった・切れた: 駆動とストリームを止め、送り残しを捨てる
static void cal_session_reset() {
  cal_motor_stop();
  cal_stream = false;
  delay(3);
  motor_off();
  if (cal_q) xQueueReset(cal_q);
  cal_out_n = 0;
  cal_end_pending = false;
  cal_dropped = 0;
}

#ifdef UNAGI_CAL_WIFI
static void cal_drain();
#endif

static void cal_set_stream(bool on) {
  if (on && !cal_stream) {
    if (cal_q) xQueueReset(cal_q);
    cal_dropped = 0;
    cal_end_pending = false;
#if UNAGI_CAL_BUS
    if (cal_owner == CAL_OWNER_BUS) sync_fit();    // サンプル時刻をヘッド時刻に直すための回帰
#endif
    cal_printf("STREAM_START");
    cal_printf("time_us,S1_raw,S2_raw,cmd_milli");
    cal_stream = true;
    xTaskNotifyGive(ctrl_task);
  } else if (!on && cal_stream) {
    cal_stream = false;
    delay(3);                                   // 制御タスクが最後の 1 点を積み終えるのを待つ
#ifdef UNAGI_CAL_WIFI
    if (cal_owner == CAL_OWNER_WIFI) {
      cal_drain();
      if (cal_dropped) cal_printf("INFO dropped %lu samples", (unsigned long)cal_dropped);
      cal_printf("STREAM_END");
      return;
    }
#endif
    cal_end_pending = true;                     // RS-485: 残りのサンプルを送り終えてから
  } else {
    cal_printf("OK HALL %d", cal_stream ? 1 : 0);
  }
}

static void cal_status() {
#if UNAGI_CAL_BUS
  if (cal_owner == CAL_OWNER_BUS) sync_fit();
#endif
  cal_printf("STATUS run=%d hall=%d wave=%s amp=%.3f freq=%.3f duty=%.3f phase=%.1f offset=%.3f "
             "limit=%.3f ulim=%.3f runmax=%lu id=%d dir=%+d minduty=%.3f fw=%08X calwifi=%d link=%s sync=%d state=%s",
             cal_run ? 1 : 0, cal_stream ? 1 : 0, CW_NAME[cal_p.wave], cal_p.amp, cal_p.freq,
             cal_p.duty, cal_p.phase, cal_p.offset, cal_p.limit, ULIM, (unsigned long)cal_runmax_ms,
             NODE_ID, MOTOR_DIR, MIN_DUTY, FW_VERSION,
#ifdef UNAGI_CAL_WIFI
             1,
#else
             0,
#endif
             cal_owner == CAL_OWNER_BUS ? "bus" : "wifi", fit_ok ? 1 : 0, STATE_NAME[state]);
}

static void cal_cmd(char* line) {
  char* verb = strtok(line, " \t");
  if (!verb) return;
  char* a1 = strtok(nullptr, " \t");
  for (char* p = verb; *p; ++p) *p = toupper(*p);
  const float fv = a1 ? atof(a1) : 0.0f;
  const bool busy = (state == S_RUN || state == S_WAIT);

  if (!strcmp(verb, "PING"))   { cal_printf("OK PING"); return; }
  if (!strcmp(verb, "STATUS")) { cal_status(); return; }
  if (!strcmp(verb, "HELP")) {
    cal_printf("CMDS: HALL 0|1  WAVE DC|SINE|SQUARE|TRI  AMP FREQ DUTY PHASE OFFSET LIMIT RUNMAX  RUN STOP  STATUS PING");
    if (cal_owner == CAL_OWNER_BUS) cal_printf("      (via head bridge: NODE <id>  HEAD POWER 0|1  HEAD INFO  are handled by the head)");
    cal_printf("      GETCAL  SETCAL c0 c1 c2 c3  SETSIG diff|sum  SETK k  SETULIM u  SAVE");
    return;
  }
  if (!strcmp(verb, "HALL")) {
    if (busy) { cal_printf("ERR busy (bus run)"); return; }
    cal_set_stream(fv > 0.5f); return;
  }
  if (!strcmp(verb, "STOP")) { cal_motor_stop(); cal_printf("OK RUN 0"); return; }
  if (!strcmp(verb, "RUN")) {
    if (busy) { cal_printf("ERR busy (bus run)"); return; }
    if (fault_active()) { cal_printf("ERR nFAULT low"); return; }
    cal_t0 = esp_timer_get_time();
    cal_stop_why = 0;
    cal_run = true;
    xTaskNotifyGive(ctrl_task);
    cal_printf("OK RUN 1"); return;
  }
  if (!strcmp(verb, "WAVE")) {
    if (!a1) { cal_printf("ERR missing wave"); return; }
    for (char* p = a1; *p; ++p) *p = toupper(*p);
    for (uint8_t i = 0; i < 4; i++)
      if (!strcmp(a1, CW_NAME[i])) { cal_p.wave = i; cal_printf("OK WAVE %s", CW_NAME[i]); return; }
    cal_printf("ERR wave must be DC|SINE|SQUARE|TRI"); return;
  }
  if (!strcmp(verb, "AMP"))    { cal_p.amp    = constrain(fv, 0.0f, 1.0f);   cal_printf("OK AMP %.3f", cal_p.amp); return; }
  if (!strcmp(verb, "FREQ"))   { cal_p.freq   = constrain(fv, 0.0f, 50.0f);  cal_printf("OK FREQ %.3f", cal_p.freq); return; }
  if (!strcmp(verb, "DUTY"))   { cal_p.duty   = constrain(fv, 0.01f, 0.99f); cal_printf("OK DUTY %.3f", cal_p.duty); return; }
  if (!strcmp(verb, "PHASE"))  { cal_p.phase  = fmodf(fv, 360.0f);           cal_printf("OK PHASE %.1f", cal_p.phase); return; }
  if (!strcmp(verb, "OFFSET")) { cal_p.offset = constrain(fv, -1.0f, 1.0f);  cal_printf("OK OFFSET %.3f", cal_p.offset); return; }
  if (!strcmp(verb, "LIMIT"))  { cal_p.limit  = constrain(fv, 0.0f, 1.0f);   cal_printf("OK LIMIT %.3f", cal_p.limit); return; }
  if (!strcmp(verb, "RUNMAX")) { cal_runmax_ms = (uint32_t)constrain(fv, 100.0f, 60000.0f); cal_printf("OK RUNMAX %lu", (unsigned long)cal_runmax_ms); return; }
  // ---- 較正値の書き込み（NVS は SAVE で保存。走行中は不可）----
  if (!strcmp(verb, "GETCAL")) {
    int16_t v1, v2; read_hall(v1, v2);
    const float d = hall_sig(v1, v2, HALL_SIG, K_COIL, 0.0f);
    cal_printf("CAL %.9g %.9g %.9g %.9g sig=%s k=%.6g ulim=%.4f md=%.3f dir=%+d id=%d v1=%d v2=%d d=%.1f deg=%.2f",
               CAL[0], CAL[1], CAL[2], CAL[3], HALL_SIG ? "sum" : "diff", K_COIL, ULIM, MIN_DUTY, MOTOR_DIR,
               NODE_ID, v1, v2, d, poly_deg(CAL, d));
    return;
  }
  if (!strcmp(verb, "SETCAL")) {
    if (busy) { cal_printf("ERR busy"); return; }
    char* a2 = strtok(nullptr, " \t"); char* a3 = strtok(nullptr, " \t"); char* a4 = strtok(nullptr, " \t");
    if (!a1 || !a2 || !a3 || !a4) { cal_printf("ERR usage: SETCAL c0 c1 c2 c3"); return; }
    CAL[0] = atof(a1); CAL[1] = atof(a2); CAL[2] = atof(a3); CAL[3] = atof(a4);
    cal_printf("OK SETCAL %.9g %.9g %.9g %.9g (SAVE to keep)", CAL[0], CAL[1], CAL[2], CAL[3]);
    return;
  }
  if (!strcmp(verb, "SETSIG")) {
    if (busy) { cal_printf("ERR busy"); return; }
    if (!a1) { cal_printf("ERR usage: SETSIG diff|sum"); return; }
    for (char* p = a1; *p; ++p) *p = tolower(*p);
    if (!strcmp(a1, "sum")) HALL_SIG = 1;
    else if (!strcmp(a1, "diff")) HALL_SIG = 0;
    else { cal_printf("ERR usage: SETSIG diff|sum"); return; }
    cal_printf("OK SETSIG %s (SAVE to keep)", HALL_SIG ? "sum" : "diff");
    return;
  }
  if (!strcmp(verb, "SETK")) {
    if (busy) { cal_printf("ERR busy"); return; }
    if (!a1) { cal_printf("ERR usage: SETK <counts per unit u>"); return; }
    K_COIL = atof(a1);
    cal_printf("OK SETK %.6g (SAVE to keep)", K_COIL);
    return;
  }
  if (!strcmp(verb, "SETULIM")) {
    if (busy) { cal_printf("ERR busy"); return; }
    if (!a1) { cal_printf("ERR usage: SETULIM <0..1>"); return; }
    ULIM = constrain(fv, 0.0f, 1.0f);
    cal_printf("OK SETULIM %.4f (SAVE to keep)", ULIM);
    return;
  }
  if (!strcmp(verb, "SAVE")) {
    if (busy || cal_run || cal_stream) { cal_printf("ERR stop RUN and HALL first"); return; }
    save_cfg();
    cal_printf("OK SAVE id=%d", NODE_ID);
    Serial.printf("CAL WiFi: NVS saved CAL={%g,%g,%g,%g} sig=%s k=%g ulim=%.3f\n", CAL[0], CAL[1], CAL[2], CAL[3],
                  HALL_SIG ? "sum" : "diff", K_COIL, ULIM);
    return;
  }
  // 旧ファームのコマンドで、ここでは意味のないもの（GUI 互換のため OK を返す）
  if (!strcmp(verb, "EN") || !strcmp(verb, "STREAM") || !strcmp(verb, "WDT") ||
      !strcmp(verb, "DECAY") || !strcmp(verb, "PWMFREQ") || !strcmp(verb, "MINDUTY")) {
    cal_printf("OK %s (ignored in calwifi)", verb); return;
  }
  cal_printf("ERR unknown cmd %s", verb);
}


// 経路によらない後始末（loop から呼ぶ）
static void cal_service() {
  if (cal_evt_autostop) {
    cal_evt_autostop = false;
    motor_off();
    cal_printf("EVT %s -> STOP", cal_stop_why == 2 ? "nFAULT low" : "RUNMAX reached");
  }
#if UNAGI_CAL_BUS
  if (cal_owner == CAL_OWNER_BUS) {
    if ((cal_run || cal_stream) && millis() - cal_last_bus_ms > 500) {   // ヘッドが来なくなった
      cal_session_reset();
      Serial.println("CAL bus: no poll from the head for 0.5 s -> STOP");
    }
    static uint32_t refit_ms = 0;
    if (cal_stream && millis() - refit_ms >= 1000) { refit_ms = millis(); sync_fit(); }
  }
#endif
}

static void cal_engine_begin() {
  cal_q = xQueueCreate(2000, sizeof(CalSample));          // 2 s 分
}
#endif  // CAL_ENGINE

// ------------------------------ 校正: RS-485 側 ------------------------
#if UNAGI_CAL_BUS
static uint8_t calx_frame[UNAGI_MAX_PAYLOAD];
static uint8_t calx_len = 0;
static uint8_t calx_seq = 0;
static bool    calx_have = false;

static inline uint32_t local_to_head32(uint32_t t_local32) {
  const int64_t now = esp_timer_get_time();
  const int64_t x = now - (int64_t)(uint32_t)((uint32_t)now - t_local32);   // 64 bit に戻す
  if (!fit_ok) return t_local32;
  const double y = (double)fit_y0 + fit_a + fit_b * (double)(x - fit_x0);
  return (uint32_t)(int64_t)llround(y);
}

static void calx_build(uint8_t seq) {
  MsgCalxData* d = (MsgCalxData*)calx_frame;
  memset(d, 0, sizeof(*d));
  hdr_init(d->h, UNAGI_MAGIC_NODE, M_CALX_DATA, NODE_ID, 0);
  d->seq = seq;
  int waiting = cal_q ? (int)uxQueueMessagesWaiting(cal_q) : 0;
  if (cal_end_pending && !cal_stream && waiting == 0) {
    cal_end_pending = false;
    if (cal_dropped) cal_printf("INFO dropped %lu samples", (unsigned long)cal_dropped);
    cal_printf("STREAM_END");
  }
  // テキスト: 行の切れ目まで（1行が長すぎるときはそのまま切る）
  int nt = (int)cal_out_n;
  if (nt > CALX_ROOM) {
    nt = CALX_ROOM;
    while (nt > 0 && cal_out[nt - 1] != '\n') nt--;
    if (nt == 0) nt = CALX_ROOM;
  }
  int ns = (CALX_ROOM - nt) / (int)sizeof(CalxSample);
  if (ns > waiting) ns = waiting;
  uint8_t* p = calx_frame + CALX_HDR_LEN;
  int got = 0;
  CalSample cs;
  while (got < ns && xQueueReceive(cal_q, &cs, 0) == pdTRUE) {
    CalxSample o;
    o.t_us = local_to_head32(cs.t);
    o.v1 = cs.v1; o.v2 = cs.v2; o.u_milli = cs.u;
    memcpy(p, &o, sizeof(o));
    p += sizeof(o);
    got++;
  }
  memcpy(p, cal_out, nt);
  memmove(cal_out, cal_out + nt, cal_out_n - nt);
  cal_out_n -= nt;
  d->n_s = (uint8_t)got;
  d->n_txt = (uint8_t)nt;
  d->dropped = (uint16_t)(cal_dropped > 65535 ? 65535 : cal_dropped);
  d->flags = (fit_ok ? CALX_F_SYNCED : 0) | (cal_stream ? CALX_F_STREAM : 0) | (cal_run ? CALX_F_RUN : 0);
  if (cal_out_n || (cal_q && uxQueueMessagesWaiting(cal_q) > 0) || cal_end_pending) d->flags |= CALX_F_MORE;
  calx_len = (uint8_t)(CALX_HDR_LEN + got * (int)sizeof(CalxSample) + nt);
}

static void calx_on_req(const BusRx& it) {
  if (it.len < offsetof(MsgCalxReq, text)) return;
  MsgCalxReq q; memset(&q, 0, sizeof(q));
  memcpy(&q, it.data, it.len < sizeof(q) ? it.len : sizeof(q));
  cal_last_bus_ms = millis();
  const bool open = q.flags & CALX_F_OPEN;
  if (open || cal_owner != CAL_OWNER_BUS) {
    cal_owner = CAL_OWNER_BUS;
    cal_session_reset();
    calx_have = false;
    sync_fit();
    if (open) {
      cal_printf("READY unagi_node_calbus id=%d", NODE_ID);
      cal_status();
    }
  } else if (calx_have && q.seq == calx_seq) {           // ヘッドの再送: 同じ返事をもう一度
    reply(calx_frame, calx_len);
    return;
  }
  if (q.n) {
    char line[CALX_TEXT_MAX + 1];
    uint8_t n = q.n > CALX_TEXT_MAX ? CALX_TEXT_MAX : q.n;
    memcpy(line, q.text, n); line[n] = 0;
    cal_cmd(line);
  }
  calx_build(q.seq);
  calx_seq = q.seq;
  calx_have = true;
  reply(calx_frame, calx_len);
}
#endif  // UNAGI_CAL_BUS

// ------------------------------ 校正: WiFi 側 --------------------------
#ifdef UNAGI_CAL_WIFI
static const char*    CAL_PASS = "unagi12345";
static const uint16_t CAL_PORT = 5000;
static WiFiServer cal_server(CAL_PORT);
static WiFiClient cal_cli;
static uint32_t cal_flush_ms = 0;

static void cal_flush() {
  if (cal_out_n && cal_cli && cal_cli.connected()) cal_cli.write((const uint8_t*)cal_out, cal_out_n);
  cal_out_n = 0;
  cal_flush_ms = millis();
}

static void cal_drain() {
  CalSample s; char line[48];
  while (cal_q && xQueueReceive(cal_q, &s, 0) == pdTRUE) {
    int n = snprintf(line, sizeof(line), "%lu,%d,%d,%d\n", (unsigned long)s.t, s.v1, s.v2, s.u);
    if (cal_out_n + n > sizeof(cal_out)) cal_flush();
    cal_write(line, n);
  }
}

static void cal_begin() {
  char ssid[24];
  snprintf(ssid, sizeof(ssid), "UNAGI-NODE-%d", NODE_ID);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, CAL_PASS);
  WiFi.setSleep(false);
  cal_server.begin();
  cal_server.setNoDelay(true);
  Serial.printf("CAL WiFi: SSID %s / %s  ->  %s:%u\n", ssid, CAL_PASS,
                WiFi.softAPIP().toString().c_str(), CAL_PORT);
}

static void cal_poll() {
  static char buf[96]; static uint8_t n = 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  WiFiClient nc = cal_server.accept();
#else
  WiFiClient nc = cal_server.available();
#endif
  if (nc) {
    if (cal_cli && cal_cli.connected()) cal_cli.stop();     // 1 クライアントのみ
    cal_owner = CAL_OWNER_WIFI;
    cal_session_reset();
    cal_cli = nc;
    cal_cli.setNoDelay(true);
    n = 0;
    cal_printf("READY unagi_node_calwifi id=%d", NODE_ID);
    cal_status();
    Serial.println("CAL WiFi: client connected");
  }
  if (cal_cli && !cal_cli.connected()) {                    // 切断 = 必ず停止
    cal_cli.stop();
    if (cal_owner == CAL_OWNER_WIFI) { cal_session_reset(); cal_owner = CAL_OWNER_NONE; }
    Serial.println("CAL WiFi: client gone -> STOP");
  }
  while (cal_cli && cal_cli.connected() && cal_cli.available()) {
    char c = cal_cli.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = 0;
      if (n) {
        if (cal_owner != CAL_OWNER_WIFI) { cal_owner = CAL_OWNER_WIFI; cal_session_reset(); }
        cal_cmd(buf);
      }
      n = 0;
    }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  if (cal_owner != CAL_OWNER_WIFI) return;
  cal_drain();
  // ストリーム中は 20 ms ごとにまとめて送る。それ以外はすぐ送る。
  if (cal_out_n && (!cal_stream || millis() - cal_flush_ms >= 20)) cal_flush();
}
#endif  // UNAGI_CAL_WIFI

// ------------------------------ setup / loop -------------------------
void setup() {
  // まずモータを止める（IN1 = IN2 = 0 → 惰性）
  pinMode(PIN_PWM1, OUTPUT); digitalWrite(PIN_PWM1, LOW);
  pinMode(PIN_PWM2, OUTPUT); digitalWrite(PIN_PWM2, LOW);
  pwm_init();
  pwm_write(0, 0);
  pinMode(PIN_FLT, INPUT_PULLUP);
  pinMode(PIN_SYNC_OUT, OUTPUT); digitalWrite(PIN_SYNC_OUT, LOW);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ESP_ARDUINO_VERSION_MAJOR < 3
  Serial.setTxTimeoutMs(0);      // USB 未接続（電源ケーブル給電）でも止まらない
#endif
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_HALL1, ADC_11db);
  analogSetPinAttenuation(PIN_HALL2, ADC_11db);

  load_cfg();
  esp_read_mac(my_mac, ESP_MAC_WIFI_STA);      // v2 で表示していた MAC と同じ値

  bus::begin(PIN_BUS_RX, PIN_BUS_TX, PIN_BUS_DE, UNAGI_MAGIC_HEAD, 16);

  xTaskCreate(control_task, "ctrl", 6144, nullptr, configMAX_PRIORITIES - 1, &ctrl_task);

  Serial.printf("\nUNAGI node  id=%d  mac=%02X:%02X:%02X:%02X:%02X:%02X  fw=%08X  RS-485 %lu bps\n",
                NODE_ID, my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5],
                FW_VERSION, (unsigned long)BUS_BAUD);
  if (NODE_ID == 0) Serial.println("★ ID 未設定です。 'id <1-10>' → 'save' を実行してください。");
#if CAL_ENGINE
  cal_engine_begin();
#endif
#ifdef UNAGI_CAL_WIFI
  cal_begin();
#endif
}

void loop() {
  // フレームが来たらすぐ起きる（最大 1 tick 待ち）
  BusRx it;
  if (bus::recv(it, 1)) {
    handle_packet(it);
    while (bus::recv(it, 0)) handle_packet(it);
  }

  if (run_done_evt) {
    run_done_evt = false;
    Serial.printf("EVT run done  n=%u ticks=%u missed=%u jit=%uus loop=%uus abort=%u start_err=%dus\n",
                  (unsigned)report.n_samples, (unsigned)report.ticks, (unsigned)report.missed,
                  (unsigned)report.max_jitter_us, report.loop_us_max, report.abort_reason,
                  (int)report.start_err_us);
  }

  poll_cli();
#ifdef UNAGI_CAL_WIFI
  cal_poll();
#endif
#if CAL_ENGINE
  cal_service();
#endif

  const uint32_t now = millis();
  if (pwm_until_ms && (int32_t)(now - pwm_until_ms) >= 0) { motor_off(); pwm_until_ms = 0; Serial.println("pwm off"); }
  const bool busy = (state == S_RUN || state == S_WAIT);   // 走行中は ADC を制御タスク専用にする
#if CAL_ENGINE
  const bool cal_busy = cal_stream || cal_run;
#else
  const bool cal_busy = false;
#endif
  if (hall_print && !busy && !cal_busy && now - hall_print_ms >= 100) {
    hall_print_ms = now;
    int16_t v1, v2; read_hall(v1, v2);
    Serial.printf("v1=%4d v2=%4d diff=%5d sum=%5d deg=%7.2f\n", v1, v2, v1 - v2, v1 + v2,
                  poly_deg(CAL, hall_sig(v1, v2, HALL_SIG, K_COIL, 0.0f)));
  }
}
