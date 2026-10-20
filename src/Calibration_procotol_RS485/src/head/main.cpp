// =====================================================================
//  UNAGI head — 電源監視 (INA226) + RS-485 ゲートウェイ   protocol v3
//  Target : Seeed XIAO ESP32-C3 / Board: Unagi_head_V2 (R3 = 2 mOhm, SP3485)
//
//  ・PowerMeasurement の機能（JSON テレメトリ、ON/OFF/INFO/SELFTEST/TRIP...）は
//    そのまま。既存の Unagi_head_gui.py が使えます。
//  ・PC とは今まで通り WiFi SoftAP "UNAGI-HEAD" + TCP 5555（または USB シリアル）。
//  ・ノード10台とは RS-485 半二重バス（1 Mbps）。ヘッドが1台ずつ指名して返事をもらう。
//  ・ロードスイッチ (PIN_GATE) がノード10台への給電そのもの。
//  ・過電流で スイッチ OFF + 全ノードへ STOP。
//
//  配線（ヘッド基板 V2）
//    D1 (GPIO3)  --> ロードスイッチ Q1/Q2 ゲート駆動
//    D2 (GPIO4)  <-- INA226 ALERT
//    D4 (GPIO6)  <-> INA226 SDA      D5 (GPIO7) --> INA226 SCL
//    D3 (GPIO5)  --> SP3485 DE + /RE （基板上 10 kΩ プルダウン = 受信）
//    D6 (GPIO21) --> SP3485 DI   (UART1 TX)
//    D7 (GPIO20) <-- SP3485 RO   (UART1 RX、基板上 10 kΩ プルアップ)
//    120 Ω 終端 (R18) と 390 Ω バイアス (R16/R17) はヘッド基板上。
//
//  v2 (ESP-NOW 版) からの変更
//   1. ESP-NOW を廃止し RS-485。WiFi は PC との接続だけに使う。
//   2. バスは「1度に1つの要求だけ」。要求ごとに返事待ち（タイムアウト・再送つき）。
//      返事待ちの間はビーコンも出さない（衝突しない）。
//   3. discover / arm はノード1台ずつ順に指名する（ID ずらしの待ち時間が不要）。
//   4. go の後、各ノードの走行時間が過ぎたらヘッドが REPORT を取りに行く。
//   5. MAC を事前に知らなくても ID 1..10 に送れる（ESP-NOW のペア登録が不要）。
//   PC 側 (pc/unagi.py) に返す行の書式は v2 と同じ。
//
//  v3.1: 校正ブリッジ（TCP 5000）
//   ・WiFi UNAGI-HEAD の 192.168.4.1:5000 に hall_calib_gui.py / hall_live.py をつなぐと、
//     node_calwifi と同じテキストプロトコルで、指定したノード1台を RS-485 越しに使える。
//     行はそのままノードへ（CALX_REQ）、ノードの返事と 1 kHz の Hall データはそのまま PC へ。
//   ・Hall の各サンプルに INA226 の電圧・電流（ヘッド時刻で補間）を付ける:
//       time_us,S1_raw,S2_raw,cmd_milli,bus_mV,cur_mA
//     time_us はヘッド時刻（ノードがビーコン同期で換算）。電流は「ノード給電全体」の値。
//   ・ブリッジ専用コマンド: NODE <id> / HEAD POWER 0|1 / HEAD INFO
//   ・ブリッジ接続中は INA226 を高速設定（平均 1 回・変換 1.1 ms → 約 2.2 ms ごとに更新）。
// =====================================================================

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <Preferences.h>
#include "unagi_protocol.h"
#include "unagi_bus.h"

#define HEAD_FW            "head-v2-rs485-3.1.0"

// ------------------------- build options -----------------------------
#define GATE_ACTIVE_HIGH   1
#define R_SHUNT_OHMS       0.002f
#define I_LSB_AMPS         0.00125f

#define NET_SSID           "UNAGI-HEAD"
#define NET_PASS           "unagi12345"
#define TCP_PORT           5555
#define BRIDGE_PORT        5000
#define INA_FAST_DELAY_US  1650     // 高速設定で、読んだ値の測定時刻の遅れ（変換時間の中央）

#define DROP_LINK_CUTS_LOAD 1
#define DEFAULT_TRIP_A     28.0f
#define BEACON_HZ          20

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ------------------------------ pins ---------------------------------
static const uint8_t PIN_SDA    = 6;   // D4
static const uint8_t PIN_SCL    = 7;   // D5
static const uint8_t PIN_ALERT  = 4;   // D2
static const uint8_t PIN_GATE   = 3;   // D1
static const int8_t  PIN_BUS_DE = 5;   // D3
static const int8_t  PIN_BUS_TX = 21;  // D6
static const int8_t  PIN_BUS_RX = 20;  // D7

// --------------------------- INA226 ----------------------------------
static const uint8_t INA_ADDR = 0x40;
static const uint8_t REG_CONFIG = 0x00, REG_SHUNT = 0x01, REG_BUS = 0x02, REG_POWER = 0x03,
                     REG_CURRENT = 0x04, REG_CALIB = 0x05, REG_MASK = 0x06, REG_ALERTLIM = 0x07,
                     REG_MANUF = 0xFE, REG_DIEID = 0xFF;
static const uint16_t MANUF_TI = 0x5449, DIE_INA226 = 0x2260;
static const float SHUNT_LSB_V = 2.5e-6f, BUS_LSB_V = 1.25e-3f;
static const uint16_t AVG_CODE[8] = {1, 4, 16, 64, 128, 256, 512, 1024};
static uint16_t cfgAvgIndex = 2;
static uint16_t buildConfig(uint16_t i) { return 0x4000 | ((i & 7) << 9) | (0x4 << 6) | (0x4 << 3) | 0x7; }
static bool ina_fast = false;            // 校正ブリッジ接続中: 平均 1 回（約 2.2 ms ごとに新しい値）

// ---------------------------- state ----------------------------------
WiFiServer  server(TCP_PORT);
WiFiClient  client;
WiFiServer  bserver(BRIDGE_PORT);       // 校正ブリッジ
WiFiClient  bcli;
Preferences prefs;

bool     streaming = true;
uint32_t streamHz = 20, streamGapMs = 50, lastStream = 0, lastSample = 0;
bool     loadOn = false, dropCutsLoad = DROP_LINK_CUTS_LOAD, sensorOk = false;
volatile bool alertFlag = false;
bool     alertLatched = false;
float    busV = 0, curA = 0, powW = 0, shuntmV = 0, curRawA = 0;
double   charge_mAh = 0, energy_Wh = 0;
uint32_t lastIntegrate = 0;
float    tripAmps = DEFAULT_TRIP_A, calGain = 1.0f, calOffset = 0.0f;

// ---- gateway ----
static uint8_t  node_mac[UNAGI_N_NODES + 1][6];
static bool     node_known[UNAGI_N_NODES + 1] = {false};
static uint32_t node_seen[UNAGI_N_NODES + 1] = {0};
static bool     param_set[UNAGI_N_NODES + 1] = {false};
static uint32_t param_dur[UNAGI_N_NODES + 1] = {0};
static bool     ready_ok[UNAGI_N_NODES + 1] = {false};
static uint16_t ready_trial[UNAGI_N_NODES + 1] = {0};
static volatile uint16_t trial = 0;
static int64_t  last_go_T = 0;

// ------------------------- output ------------------------------------
// Serial は USB 未接続でも止まらないよう非ブロッキング。TCP は loop() からのみ書く。
static void outSerial(const char* s, size_t n) {
#if ARDUINO_USB_CDC_ON_BOOT
  if ((size_t)Serial.availableForWrite() >= n + 2) { Serial.write((const uint8_t*)s, n); Serial.write("\r\n"); }
#else
  Serial.write((const uint8_t*)s, n); Serial.write("\r\n");
#endif
}
static void outTcp(const char* s, size_t n) {
  if (client && client.connected()) { client.write((const uint8_t*)s, n); client.write("\n", 1); }
}
static void emit(const char* s)   { size_t n = strlen(s); outSerial(s, n); outTcp(s, n); }
static void emitf(const char* fmt, ...) {
  char b[512];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  emit(b);
}
// 大量データ（DUMP）は TCP が繋がっていれば TCP のみ
static void emitData(const char* s) {
  size_t n = strlen(s);
  if (client && client.connected()) outTcp(s, n); else outSerial(s, n);
}

// ------------------------- I2C / INA226 -------------------------------
static bool ina_write(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(val >> 8)); Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}
static bool ina_read(uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA_ADDR, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}
static void setLoad(bool on) {
  loadOn = on;
#if GATE_ACTIVE_HIGH
  digitalWrite(PIN_GATE, on ? HIGH : LOW);
#else
  digitalWrite(PIN_GATE, on ? LOW : HIGH);
#endif
}
static void IRAM_ATTR onAlert() { alertFlag = true; }
static void setTrip(float amps) {
  tripAmps = amps;
  long lsbs = lround((amps * R_SHUNT_OHMS) / SHUNT_LSB_V);
  if (lsbs > 32767) lsbs = 32767;
  if (lsbs < 0) lsbs = 0;
  ina_write(REG_ALERTLIM, (uint16_t)lsbs);
  ina_write(REG_MASK, 0x8000 | 0x0001);
}
static void clearAlert() { uint16_t d; ina_read(REG_MASK, d); alertLatched = false; alertFlag = false; }

static void sample() {
  uint16_t raw;
  if (ina_read(REG_BUS, raw))     busV = raw * BUS_LSB_V;
  if (ina_read(REG_SHUNT, raw))   shuntmV = (int16_t)raw * SHUNT_LSB_V * 1000.0f;
  if (ina_read(REG_CURRENT, raw)) { curRawA = (int16_t)raw * I_LSB_AMPS; curA = curRawA * calGain - calOffset; }
  if (ina_read(REG_POWER, raw))   powW = raw * 25.0f * I_LSB_AMPS * calGain;
  uint32_t now = millis();
  if (lastIntegrate) {
    double dt_h = (now - lastIntegrate) / 3600000.0;
    charge_mAh += curA * 1000.0 * dt_h;
    energy_Wh  += powW * dt_h;
  }
  lastIntegrate = now;
}
static void sendTelemetry() {
  emitf("{\"t\":%lu,\"v\":%.4f,\"i\":%.4f,\"p\":%.4f,\"sh\":%.4f,\"mah\":%.3f,\"wh\":%.5f,"
        "\"sw\":%d,\"alrt\":%d,\"vok\":%d,\"ok\":%d}",
        (unsigned long)millis(), busV, curA, powW, shuntmV, charge_mAh, energy_Wh,
        loadOn ? 1 : 0, alertLatched ? 1 : 0, loadOn ? 1 : 0, sensorOk ? 1 : 0);
}
static int nodesKnown() { int n = 0; for (int i = 1; i <= UNAGI_N_NODES; i++) n += node_known[i]; return n; }
static void sendInfo() {
  uint16_t manuf = 0, die = 0, cfg = 0, cal = 0;
  ina_read(REG_MANUF, manuf); ina_read(REG_DIEID, die); ina_read(REG_CONFIG, cfg); ina_read(REG_CALIB, cal);
  emitf("{\"info\":1,\"board\":\"v2-rs485\",\"fw\":\"%s\",\"manuf\":\"0x%04X\",\"die\":\"0x%04X\","
        "\"cfg\":\"0x%04X\",\"cal\":%u,\"rshunt\":%.5f,\"ilsb\":%.6f,\"fs\":%.2f,\"avg\":%u,\"hz\":%lu,"
        "\"trip\":%.2f,\"gain\":%.5f,\"offset\":%.5f,\"ok\":%d,\"ch\":%d,\"bus\":%lu,\"nodes\":%d,\"ip\":\"%s\"}",
        HEAD_FW, manuf, die, cfg, cal, R_SHUNT_OHMS, I_LSB_AMPS, 32767.0f * I_LSB_AMPS,
        AVG_CODE[cfgAvgIndex], (unsigned long)streamHz, tripAmps, calGain, calOffset,
        sensorOk ? 1 : 0, UNAGI_CHANNEL, (unsigned long)BUS_BAUD, nodesKnown(), WiFi.softAPIP().toString().c_str());
}
static void calLoad() {
  prefs.begin("unagi", true);
  calGain = prefs.getFloat("gain", 1.0f); calOffset = prefs.getFloat("offs", 0.0f);
  tripAmps = prefs.getFloat("trip", DEFAULT_TRIP_A);
  prefs.end();
}
static void calSave() {
  prefs.begin("unagi", false);
  prefs.putFloat("gain", calGain); prefs.putFloat("offs", calOffset); prefs.putFloat("trip", tripAmps);
  prefs.end();
}
static void tare() {
  double acc = 0;
  for (int i = 0; i < 64; i++) { sample(); acc += curRawA * calGain; delay(5); }
  calOffset = acc / 64.0; charge_mAh = 0; energy_Wh = 0;
}
static void selfTest() {
  emit("{\"test\":\"start\"}");
  uint16_t manuf = 0, die = 0;
  bool idOk = ina_read(REG_MANUF, manuf) && ina_read(REG_DIEID, die) && manuf == MANUF_TI && die == DIE_INA226;
  emitf("{\"test\":\"i2c\",\"pass\":%d,\"manuf\":\"0x%04X\",\"die\":\"0x%04X\"}", idOk ? 1 : 0, manuf, die);
  if (!idOk) { emit("{\"test\":\"abort\",\"why\":\"INA226 not responding\"}"); return; }
  uint16_t cal = 0; ina_read(REG_CALIB, cal);
  emitf("{\"test\":\"cal\",\"pass\":%d,\"read\":%u,\"expect\":2048}", cal == 2048 ? 1 : 0, cal);
  setLoad(false);
  uint32_t t0 = millis(); float vmin = 1e9;
  while (millis() - t0 < 8000) { sample(); vmin = min(vmin, busV); delay(20); }
  emitf("{\"test\":\"switch_off\",\"pass\":%d,\"vbus\":%.3f,\"note\":\"expect <0.5 V\"}", vmin < 0.5f ? 1 : 0, vmin);
  double acc = 0;
  for (int i = 0; i < 32; i++) { sample(); acc += curRawA; delay(5); }
  float off = acc / 32.0;
  emitf("{\"test\":\"zero\",\"pass\":%d,\"amps\":%.4f,\"note\":\"expect <0.05 A\"}", fabs(off) < 0.05f ? 1 : 0, off);
  emit("{\"test\":\"done\",\"note\":\"switch left OFF\"}");
}

// ===================== RS-485 gateway ===============================
// バスの使い方
//  ・ヘッドだけが話し始める。ノードは自分が指名されたときだけ返事をする。
//  ・返事を待つ要求は1度に1つ (cur)。待っている間はビーコンも出さない。
//  ・要求は PC コマンドからキュー (rq) に積まれ、ループが順に送る。
//    キューが空ならダンプ → REPORT 取りの順にバスを使う。
//  ・STOP だけは何があっても即送る（3回）。

static SemaphoreHandle_t sched_mtx = nullptr;   // 「返事待ちか調べて送る」を不可分にする
static TaskHandle_t      beacon_h  = nullptr;
static volatile bool     bus_await  = false;    // 返事待ち中
static volatile bool     beacon_due = false;    // ビーコンを出す時刻を過ぎた
static uint32_t          n_timeouts = 0;

enum : uint8_t { K_PLAIN = 0, K_DISCOVER, K_ARM, K_START, K_DUMP, K_REPORT, K_CALX };

struct Req {
  uint8_t  kind, id, expect, tries;   // expect = 待つ返事の type（0 = 返事なし）
  uint16_t timeout_ms;
  uint16_t aux;                       // K_START: 開始までの遅れ [ms]
  char     name[10];                  // タイムアウト時の表示名
  uint8_t  len;
  uint8_t  buf[UNAGI_MAX_PAYLOAD];
};
#define RQ_N 24
static Req      rq[RQ_N];
static uint8_t  rq_head = 0, rq_count = 0;
static Req      cur;
static uint8_t  cur_state = 0;        // 0 = なし, 1 = 送信待ち, 2 = 返事待ち
static uint8_t  cur_try = 0;
static uint32_t cur_sent_ms = 0;

static bool enqueue(uint8_t kind, uint8_t id, uint8_t expect, uint16_t timeout_ms, uint8_t tries,
                    const char* name, const void* p, size_t n, uint16_t aux = 0) {
  if (rq_count >= RQ_N) { emitf("ERR,busy,queue_full,%s", name); return false; }
  Req& r = rq[(rq_head + rq_count) % RQ_N];
  r.kind = kind; r.id = id; r.expect = expect; r.timeout_ms = timeout_ms;
  r.tries = tries ? tries : 1; r.aux = aux;
  strncpy(r.name, name, sizeof(r.name) - 1); r.name[sizeof(r.name) - 1] = 0;
  r.len = (uint8_t)n; memcpy(r.buf, p, n);
  rq_count++;
  return true;
}
static void drop_queued(uint8_t kind) {
  uint8_t w = 0;
  for (uint8_t i = 0; i < rq_count; i++) {
    Req& r = rq[(rq_head + i) % RQ_N];
    if (r.kind == kind) continue;
    if (w != i) rq[(rq_head + w) % RQ_N] = r;
    w++;
  }
  rq_count = w;
}

static void mac_str(const uint8_t* m, char* out) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}
static inline bool valid_id(int id) { return id >= 1 && id <= UNAGI_N_NODES; }

// 全体宛てをすぐ送る（返事なし）
static void bcast_now(const void* p, size_t n, int times, int gap_us) {
  xSemaphoreTake(sched_mtx, portMAX_DELAY);
  for (int k = 0; k < times; k++) { bus::send(p, n); if (k + 1 < times) delayMicroseconds(gap_us); }
  xSemaphoreGive(sched_mtx);
}
static void bcast_stop(uint8_t reason) {
  MsgStop m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_STOP, 0, trial); m.reason = reason;
  bcast_now(&m, sizeof(m), 3, 300);
  drop_queued(K_START);                // まだ送っていない go は取り消す
}

// ---- ビーコン: 20 Hz。返事待ちの間は待ってから出す。時刻は送信直前に取る ----
static void beacon_task(void*) {
  uint32_t seq = 1;
  const int64_t period = 1000000 / BEACON_HZ;
  int64_t next = esp_timer_get_time() + period;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, 2);
    const int64_t now = esp_timer_get_time();
    if (now >= next) { beacon_due = true; next += period; if (next <= now) next = now + period; }
    if (!beacon_due) continue;
    xSemaphoreTake(sched_mtx, portMAX_DELAY);
    if (!bus_await) {
      MsgBeacon b = {}; hdr_init(b.h, UNAGI_MAGIC_HEAD, M_BEACON, 0, trial);
      b.seq = seq++;
      b.t_head_us = esp_timer_get_time();
      bus::send(&b, sizeof(b));
      beacon_due = false;
    }
    xSemaphoreGive(sched_mtx);
  }
}

// ---- ダンプ: ヘッドが自分でブロックを順に取りに行く ----
struct DumpSM {
  bool active = false; uint8_t id = 0; uint16_t block = 0, n_blocks = 1, n_total = 0;
  uint16_t missing = 0;
} dm;
static const uint16_t DUMP_TIMEOUT_MS = 40;
static const uint8_t  DUMP_TRIES = 4;

static void dump_finish() {
  emitf("DUMPEND,%u,%u,%u,%u", dm.id, dm.n_total, dm.n_blocks, dm.missing);
  dm.active = false;
}
static void dump_advance() {
  dm.block++;
  if (dm.block >= dm.n_blocks) dump_finish();
}
static bool dump_make(Req& r) {
  if (!dm.active) return false;
  MsgDumpReq q = {}; hdr_init(q.h, UNAGI_MAGIC_HEAD, M_DUMP_REQ, dm.id, trial); q.block = dm.block;
  r.kind = K_DUMP; r.id = dm.id; r.expect = M_DUMP_DATA; r.timeout_ms = DUMP_TIMEOUT_MS;
  r.tries = DUMP_TRIES; r.aux = 0; strcpy(r.name, "dump");
  r.len = sizeof(q); memcpy(r.buf, &q, sizeof(q));
  return true;
}
static void dump_on_data(const MsgDumpData& d) {
  if (!dm.active || d.h.node_id != dm.id || d.block != dm.block) return;
  dm.n_total = d.n_total;
  dm.n_blocks = (d.n_total + DUMP_PER_BLOCK - 1) / DUMP_PER_BLOCK;
  if (dm.n_blocks == 0) { dump_finish(); return; }
  static char line[1400];
  int p = snprintf(line, sizeof(line), "DUMP,%u,%u,%u", d.h.node_id, d.block, d.n);
  for (int i = 0; i < d.n && i < DUMP_PER_BLOCK; i++) {
    const LogSample& s = d.s[i];
    p += snprintf(line + p, sizeof(line) - p, ",%lu,%d,%d,%d,%d,%d",
                  (unsigned long)s.t_us, s.cmd, s.meas, s.u, s.v1, s.v2);
  }
  emitData(line);
  dump_advance();
}
static void dump_on_fail() {
  emitf("DUMPMISS,%u,%u", dm.id, dm.block);
  dm.missing++;
  if (dm.block == 0) { dm.n_blocks = 0; dump_finish(); return; }   // 応答なし
  dump_advance();
}

// ---- 走行後の REPORT 取り ----
static bool     rp_on[UNAGI_N_NODES + 1] = {false};
static uint32_t rp_due[UNAGI_N_NODES + 1], rp_giveup[UNAGI_N_NODES + 1];
static const uint32_t RP_INTERVAL_MS = 300;

static bool report_make(Req& r) {
  const uint32_t now = millis();
  for (uint8_t id = 1; id <= UNAGI_N_NODES; id++) {
    if (!rp_on[id] || (int32_t)(now - rp_due[id]) < 0) continue;
    rp_due[id] = now + RP_INTERVAL_MS;                 // 返事がなければ次はこの時刻
    Hdr h; hdr_init(h, UNAGI_MAGIC_HEAD, M_REPORT_REQ, id, trial);
    r.kind = K_REPORT; r.id = id; r.expect = M_REPORT; r.timeout_ms = 25;
    r.tries = 1; r.aux = 0; strcpy(r.name, "report");
    r.len = sizeof(h); memcpy(r.buf, &h, sizeof(h));
    return true;
  }
  return false;
}
static void report_on_fail(uint8_t id) {
  if (!valid_id(id) || !rp_on[id]) return;
  if ((int32_t)(millis() - rp_giveup[id]) >= 0) {
    rp_on[id] = false;
    emitf("WARN,no_report,%u,%u", id, trial);
  }
}
static void schedule_reports(uint16_t dly_ms) {
  bool any_ready = false, any_known = false;
  for (int i = 1; i <= UNAGI_N_NODES; i++) {
    any_ready |= (ready_ok[i] && ready_trial[i] == trial);
    any_known |= node_known[i];
  }
  const uint32_t now = millis();
  for (int i = 1; i <= UNAGI_N_NODES; i++) {
    bool target = any_ready ? (ready_ok[i] && ready_trial[i] == trial) : (any_known ? node_known[i] : true);
    rp_on[i] = target;
    if (!target) continue;
    if (param_set[i]) {
      rp_due[i]    = now + dly_ms + param_dur[i] + 200;
      rp_giveup[i] = rp_due[i] + 5000;
    } else {                                            // 走行時間が分からない（ヘッド再起動後など）
      rp_due[i]    = now + dly_ms + 200;
      rp_giveup[i] = now + dly_ms + 65000;
    }
  }
}

// ===================== 校正ブリッジ (TCP 5000) =====================
// PC の1行 → CALX_REQ（1行ずつ、seq つき）→ ノードの cal_cmd()。ノードの返事 (CALX_DATA) の
// テキストはそのまま、サンプルには INA226 の V/I を付けて PC へ。
static uint8_t  br_node = 0;              // 転送先ノード（NVS "calnode"）
static bool     br_active = false;        // PC がつながっている
static bool     br_closing = false;       // 切断後、STOP / HALL 0 を送り終えるまで
static bool     br_open = false;          // 次の要求に CALX_F_OPEN を付ける
static uint8_t  br_seq = 0, br_last_rx_seq = 0;
static bool     br_have_rx = false;
static bool     br_node_stream = false, br_node_more = false, br_synced_warned = false;
static uint32_t br_last_poll_ms = 0, br_fail_ms = 0;
static uint32_t br_samples = 0;
#define BR_CMDQ 16
static char     br_cmd[BR_CMDQ][CALX_TEXT_MAX + 1];
static uint8_t  br_cmd_head = 0, br_cmd_n = 0;
static char     br_out[8192];
static size_t   br_out_n = 0;
static uint32_t br_flush_ms = 0;

static void br_flush() {
  if (br_out_n && bcli && bcli.connected()) bcli.write((const uint8_t*)br_out, br_out_n);
  br_out_n = 0;
  br_flush_ms = millis();
}
static void br_write(const char* s, size_t n) {
  if (!br_active) return;
  if (br_out_n + n > sizeof(br_out)) br_flush();
  if (n > sizeof(br_out)) return;
  memcpy(br_out + br_out_n, s, n);
  br_out_n += n;
}
static void br_printf(const char* fmt, ...) {
  char b[256];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(b) - 2) n = sizeof(b) - 2;
  b[n++] = '\n';
  br_write(b, n);
}
static void br_push_cmd(const char* t) {
  if (br_cmd_n >= BR_CMDQ) { br_printf("ERR head: bridge command queue full, dropped '%s'", t); return; }
  char* d = br_cmd[(br_cmd_head + br_cmd_n) % BR_CMDQ];
  strncpy(d, t, CALX_TEXT_MAX); d[CALX_TEXT_MAX] = 0;
  br_cmd_n++;
}

// ---- INA226 の V/I をヘッド時刻つきで保存（ブリッジ接続中は 2 ms ごとに読む）----
struct ViPt { uint32_t t; float v, i; };
#define VI_N 1024
static ViPt     vi_ring[VI_N];
static uint16_t vi_head = 0, vi_cnt = 0;
static void vi_push(uint32_t t, float v, float i) {
  vi_ring[vi_head] = {t, v, i};
  vi_head = (vi_head + 1) % VI_N;
  if (vi_cnt < VI_N) vi_cnt++;
}
// 時刻 t（ヘッド時刻 µs, 32 bit）の V/I を直線補間。範囲外は端の値。
static bool vi_at(uint32_t t, float& v, float& i) {
  if (!vi_cnt) return false;
  int k = (vi_head + VI_N - 1) % VI_N;                 // 最新
  if ((int32_t)(t - vi_ring[k].t) >= 0) { v = vi_ring[k].v; i = vi_ring[k].i; return true; }
  for (uint16_t n = 1; n < vi_cnt; n++) {
    int j = (k + VI_N - 1) % VI_N;                     // ひとつ前
    if ((int32_t)(t - vi_ring[j].t) >= 0) {
      const ViPt& a = vi_ring[j]; const ViPt& b = vi_ring[k];
      float span = (float)(int32_t)(b.t - a.t), f = span > 0 ? (float)(int32_t)(t - a.t) / span : 0.0f;
      v = a.v + f * (b.v - a.v); i = a.i + f * (b.i - a.i);
      return true;
    }
    k = j;
  }
  v = vi_ring[k].v; i = vi_ring[k].i;                  // 最古より前
  return true;
}
static void ina_set_fast(bool on) {
  ina_fast = on;
  ina_write(REG_CONFIG, on ? (uint16_t)(0x4000 | (0x4 << 6) | (0x4 << 3) | 0x7) : buildConfig(cfgAvgIndex));
  vi_cnt = 0; vi_head = 0;
}
static void fast_sample() {                            // 電流と電圧だけ読む（約 0.3 ms）
  uint16_t raw;
  const uint32_t t = (uint32_t)esp_timer_get_time() - INA_FAST_DELAY_US;
  bool ok = true;
  if (ina_read(REG_CURRENT, raw)) { curRawA = (int16_t)raw * I_LSB_AMPS; curA = curRawA * calGain - calOffset; } else ok = false;
  if (ina_read(REG_BUS, raw))     busV = raw * BUS_LSB_V; else ok = false;
  powW = busV * curA;
  uint32_t now = millis();
  if (lastIntegrate) {
    double dt_h = (now - lastIntegrate) / 3600000.0;
    charge_mAh += curA * 1000.0 * dt_h;
    energy_Wh  += powW * dt_h;
  }
  lastIntegrate = now;
  if (ok) vi_push(t, busV, curA);
}

// ---- 要求づくり（スケジューラから。キュー・ダンプ・REPORT がないとき）----
static bool calx_make(Req& r) {
  if (!valid_id(br_node) || !(br_active || br_closing)) return false;
  const uint32_t now = millis();
  const bool have_cmd = br_cmd_n > 0;
  if (!have_cmd && !br_open) {
    if (br_closing) { br_closing = false; return false; }          // STOP / HALL 0 を送り終えた
    uint32_t gap = br_node_more ? 0 : (br_node_stream ? 10 : 50);
    if (now - br_last_poll_ms < gap) return false;
    if (now - br_fail_ms < 300) return false;                      // 応答がないノードを叩き続けない
  }
  MsgCalxReq q; memset(&q, 0, sizeof(q));
  hdr_init(q.h, UNAGI_MAGIC_HEAD, M_CALX_REQ, br_node, trial);
  q.seq = ++br_seq;
  q.flags = br_open ? CALX_F_OPEN : 0;
  if (have_cmd) {
    const char* t = br_cmd[br_cmd_head];
    q.n = (uint8_t)strnlen(t, CALX_TEXT_MAX);
    memcpy(q.text, t, q.n);
    br_cmd_head = (br_cmd_head + 1) % BR_CMDQ; br_cmd_n--;
  }
  br_open = false;
  r.kind = K_CALX; r.id = br_node; r.expect = M_CALX_DATA; r.timeout_ms = 20; r.tries = 3; r.aux = 0;
  strcpy(r.name, "calx");
  r.len = (uint8_t)(offsetof(MsgCalxReq, text) + q.n);
  memcpy(r.buf, &q, r.len);
  br_last_poll_ms = now;
  return true;
}
static void calx_on_fail(const Req& r) {
  MsgCalxReq q; memset(&q, 0, sizeof(q)); memcpy(&q, r.buf, r.len);
  if (q.n) { q.text[q.n < CALX_TEXT_MAX ? q.n : CALX_TEXT_MAX - 1] = 0; br_printf("ERR head: node %u did not answer, command '%s' may be lost", r.id, q.text); }
  else if (millis() - br_fail_ms > 2000) br_printf("ERR head: node %u does not answer on RS-485 (power? id? cable?)", r.id);
  if (q.flags & CALX_F_OPEN) br_open = true;                       // 開始はやり直す
  br_fail_ms = millis();
  br_node_more = false;
}
static void calx_on_data(const BusRx& it) {
  if (it.len < CALX_HDR_LEN) return;
  MsgCalxData d; memcpy(&d, it.data, CALX_HDR_LEN);
  if (d.h.node_id != br_node) return;
  if (br_have_rx && d.seq == br_last_rx_seq) return;               // 遅れて届いた同じ返事
  br_have_rx = true; br_last_rx_seq = d.seq;
  br_node_stream = d.flags & CALX_F_STREAM;
  br_node_more   = d.flags & CALX_F_MORE;
  const int ns = d.n_s, nt = d.n_txt;
  if (CALX_HDR_LEN + ns * (int)sizeof(CalxSample) + nt > it.len) return;
  const uint8_t* p = it.data + CALX_HDR_LEN;
  // 1) テキスト（行ごと。Hall の見出し行には V/I の列を足す）
  const char* tx = (const char*)(p + ns * sizeof(CalxSample));
  int a = 0;
  for (int k = 0; k < nt; k++) {
    if (tx[k] != '\n') continue;
    if (k - a >= 8 && !strncmp(tx + a, "time_us,", 8)) br_printf("time_us,S1_raw,S2_raw,cmd_milli,bus_mV,cur_mA");
    else { br_write(tx + a, k - a); br_write("\n", 1); }
    a = k + 1;
  }
  if (a < nt) { br_write(tx + a, nt - a); br_write("\n", 1); }
  // 2) サンプル + V/I
  if (ns && !(d.flags & CALX_F_SYNCED) && !br_synced_warned) {
    br_synced_warned = true;
    br_printf("INFO head: node %u is not time-synced yet - V/I matched by arrival time (about +-20 ms)", br_node);
  }
  char line[64];
  for (int k = 0; k < ns; k++) {
    CalxSample s; memcpy(&s, p + k * sizeof(CalxSample), sizeof(s));
    float v = busV, i = curA;
    if (d.flags & CALX_F_SYNCED) vi_at(s.t_us, v, i);
    int n = snprintf(line, sizeof(line), "%lu,%d,%d,%d,%ld,%ld\n", (unsigned long)s.t_us, s.v1, s.v2, s.u_milli,
                     lroundf(v * 1000.0f), lroundf(i * 1000.0f));
    br_write(line, n);
  }
  br_samples += ns;
}

static void br_handle_line(char* line) {
  char tmp[CALX_TEXT_MAX + 1];
  strncpy(tmp, line, CALX_TEXT_MAX); tmp[CALX_TEXT_MAX] = 0;
  char* verb = strtok(tmp, " \t");
  if (!verb) return;
  for (char* p = verb; *p; ++p) *p = toupper(*p);
  if (!strcmp(verb, "NODE")) {
    char* a = strtok(nullptr, " \t");
    if (a) {
      int id = atoi(a);
      if (!valid_id(id)) { br_printf("ERR NODE must be 1..%d", UNAGI_N_NODES); return; }
      if (id != br_node) {
        if (valid_id(br_node)) { br_push_cmd("STOP"); br_push_cmd("HALL 0"); }   // 前のノードを止める
        br_node = (uint8_t)id; br_open = true; br_have_rx = false; br_synced_warned = false;
        prefs.begin("unagi", false); prefs.putUChar("calnode", br_node); prefs.end();
      }
    }
    br_printf("OK NODE %u", br_node);
    return;
  }
  if (!strcmp(verb, "HEAD")) {
    char* a = strtok(nullptr, " \t");
    for (char* p = a; p && *p; ++p) *p = toupper(*p);
    if (a && !strcmp(a, "POWER")) {
      char* b = strtok(nullptr, " \t");
      bool on = b && atoi(b);
      if (on && !sensorOk)   { br_printf("ERR HEAD POWER: INA226 not found (use 'power 1 force' on port 5555)"); return; }
      if (on && alertLatched) { br_printf("ERR HEAD POWER: overcurrent latched (send CLEAR on port 5555)"); return; }
      if (!on) bcast_stop(2);
      setLoad(on);
      br_printf("OK HEAD POWER %d", loadOn ? 1 : 0);
      return;
    }
    br_printf("HEAD fw=%s load=%d sensor=%d v=%.3f i=%.4f node=%u nodes=%d fast=%d samples=%lu", HEAD_FW, loadOn ? 1 : 0,
              sensorOk ? 1 : 0, busV, curA, br_node, nodesKnown(), ina_fast ? 1 : 0, (unsigned long)br_samples);
    return;
  }
  if (!valid_id(br_node)) { br_printf("ERR head: no node selected - send NODE <id>"); return; }
  br_push_cmd(line);
}

static void br_on_connect() {
  br_active = true; br_closing = false; br_open = true;
  br_cmd_n = 0; br_out_n = 0; br_have_rx = false; br_synced_warned = false;
  br_node_stream = br_node_more = false; br_fail_ms = 0;
  ina_set_fast(true);
  br_printf("READY unagi_head_bridge fw=%s node=%u load=%d", HEAD_FW, br_node, loadOn ? 1 : 0);
  if (!loadOn) br_printf("INFO head: node power is OFF - send HEAD POWER 1");
  if (!valid_id(br_node)) {
    for (int i = 1; i <= UNAGI_N_NODES && !valid_id(br_node); i++) if (node_known[i]) br_node = i;
    if (!valid_id(br_node)) br_printf("INFO head: no node selected - send NODE <id>");
  }
}
static void br_on_disconnect() {
  br_active = false;
  br_cmd_n = 0;
  if (valid_id(br_node)) { br_closing = true; br_push_cmd("STOP"); br_push_cmd("HALL 0"); }
  br_node_stream = br_node_more = false;
  ina_set_fast(false);
}
static void br_poll() {
  static char buf[CALX_TEXT_MAX + 1]; static uint16_t n = 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  WiFiClient inc = bserver.accept();
#else
  WiFiClient inc = bserver.available();
#endif
  if (inc) {
    if (bcli) { bcli.stop(); br_on_disconnect(); }
    bcli = inc; bcli.setNoDelay(true); n = 0;
    br_on_connect();
  }
  if (bcli && !bcli.connected()) { bcli.stop(); br_on_disconnect(); }
  while (bcli && bcli.connected() && bcli.available()) {
    char c = bcli.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; if (n) br_handle_line(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  if (br_out_n && (millis() - br_flush_ms >= 20 || br_out_n > 6000)) br_flush();
}

// ---- スケジューラ（loop から呼ぶ）----
static void on_fail(const Req& r) {
  switch (r.kind) {
    case K_DISCOVER: break;                                        // いないノードは黙る
    case K_ARM:      if (node_known[r.id]) emitf("WARN,no_ready,%u,%u", r.id, trial); break;
    case K_DUMP:     dump_on_fail(); break;
    case K_REPORT:   report_on_fail(r.id); break;
    case K_CALX:     calx_on_fail(r); break;
    default:         emitf("ERR,timeout,%s,%u", r.name, r.id); break;
  }
}

static void sched_pick() {
  if (rq_count) {
    cur = rq[rq_head]; rq_head = (rq_head + 1) % RQ_N; rq_count--;
  } else if (!dump_make(cur) && !report_make(cur) && !calx_make(cur)) {
    return;
  }
  cur_try = 0; cur_state = 1;
}

static void sched_send() {
  if (beacon_due) { xTaskNotifyGive(beacon_h); return; }          // 先にビーコンを出させる
  xSemaphoreTake(sched_mtx, portMAX_DELAY);
  if (beacon_due) { xSemaphoreGive(sched_mtx); xTaskNotifyGive(beacon_h); return; }

  if (cur.kind == K_START) {                                        // 開始時刻は送る瞬間に決める
    MsgStart m; memcpy(&m, cur.buf, sizeof(m));
    m.t_start_head_us = esp_timer_get_time() + (int64_t)cur.aux * 1000;
    for (int k = 0; k < 3; k++) { bus::send(&m, sizeof(m)); if (k < 2) delayMicroseconds(300); }
    xSemaphoreGive(sched_mtx);
    last_go_T = m.t_start_head_us;
    cur_state = 0;
    emitf("OK,go,%u,%lld", m.h.trial, (long long)m.t_start_head_us);
    schedule_reports(cur.aux);
    return;
  }

  bus_await = (cur.expect != 0);
  bus::send(cur.buf, cur.len);
  xSemaphoreGive(sched_mtx);
  cur_sent_ms = millis();
  cur_state = cur.expect ? 2 : 0;
}

static void scheduler() {
  if (cur_state == 2) {
    if (millis() - cur_sent_ms < cur.timeout_ms) return;
    bus_await = false;
    n_timeouts++;
    if (++cur_try < cur.tries) cur_state = 1;                       // 同じ要求を再送
    else { cur_state = 0; on_fail(cur); }
  }
  if (cur_state == 0) sched_pick();
  if (cur_state == 1) sched_send();
}

// ---- 受信処理（loop から呼ぶ）----
static inline void take(void* dst, size_t n, const BusRx& it) {
  memset(dst, 0, n);
  memcpy(dst, it.data, it.len < n ? it.len : n);
}

static void handle_rx(const BusRx& it) {
  Hdr h; memcpy(&h, it.data, sizeof(h));
  const uint8_t id = h.node_id;
  if (id > UNAGI_N_NODES) return;

  if (cur_state == 2 && id == cur.id && h.type == cur.expect) {    // 待っていた返事
    cur_state = 0;
    bus_await = false;
  }
  node_seen[id] = millis();
  if (id >= 1) node_known[id] = true;

  switch (h.type) {
    case M_HELLO: {
      MsgHello m; take(&m, sizeof(m), it);
      char mb[18]; mac_str(m.mac, mb);
      if (id >= 1) {
        static const uint8_t zero6[6] = {0};
        if (memcmp(node_mac[id], zero6, 6) && memcmp(node_mac[id], m.mac, 6)) {
          char o[18]; mac_str(node_mac[id], o);
          emitf("WARN,node_mac_changed,%u,%s,%s  (同じ ID のノードが2台ある？)", id, o, mb);
        }
        memcpy(node_mac[id], m.mac, 6);
      }
      emitf("HELLO,%u,%s,%08lX,%u,%u,%g,%g,%g,%g,%.3f", id, mb, (unsigned long)m.fw, m.proto,
            m.state, m.c[0], m.c[1], m.c[2], m.c[3], m.min_duty);
      if (m.proto != UNAGI_PROTO_VER) emitf("WARN,proto_mismatch,%u,%u,%u", id, m.proto, UNAGI_PROTO_VER);
      if (br_active && !valid_id(br_node) && valid_id(id)) {
        br_node = id; br_open = true;
        br_printf("INFO head: bridge -> node %u (found by discover)", id);
      }
      break;
    }
    case M_PARAM_ACK: {
      MsgParamAck m; take(&m, sizeof(m), it);
      emitf("PACK,%u,%u,%u", id, m.crc, m.mode);
      break;
    }
    case M_READY: {
      MsgReady m; take(&m, sizeof(m), it);
      if (id >= 1) { ready_ok[id] = m.ready; ready_trial[id] = h.trial; }
      emitf("READY,%u,%u,%u,%u,%u,%u,%.1f,%.3f,%lu,%d,%d", id, h.trial, m.ready, m.why, m.param_crc,
            m.sync_n, m.sigma_us, m.skew_ppm, (unsigned long)m.free_heap, m.v1, m.v2);
      break;
    }
    case M_REPORT: {
      MsgReport m; take(&m, sizeof(m), it);
      if (h.trial == trial) rp_on[id] = false;
      emitf("REPORT,%u,%u,%lu,%lu,%lu,%lu,%.3f,%u,%u,%ld", id, h.trial,
            (unsigned long)m.n_samples, (unsigned long)m.ticks, (unsigned long)m.missed,
            (unsigned long)m.max_jitter_us, m.rms_err_deg, m.loop_us_max, m.abort_reason,
            (long)m.start_err_us);
      break;
    }
    case M_DUMP_DATA: {
      static MsgDumpData m; take(&m, sizeof(m), it);
      dump_on_data(m);
      break;
    }
    case M_CAL_ACK: {
      MsgCalAck m; take(&m, sizeof(m), it);
      emitf("CALACK,%u,%.9g,%.9g,%.9g,%.9g,%.4f", id, m.c[0], m.c[1], m.c[2], m.c[3], m.min_duty);
      break;
    }
    case M_CALX_DATA: {
      calx_on_data(it);
      break;
    }
    case M_PEEK: {
      MsgPeek m; take(&m, sizeof(m), it);
      emitf("PEEK,%u,%u,%u,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.3f,%u", id, m.state, m.n,
            m.v1, m.v2, m.diff, m.diff_sd, m.deg, m.sigma_us, m.skew_ppm, m.sync_n);
      break;
    }
  }
}

// ===================== commands =====================================
static void printHelp() {
  emit("# gateway commands (case-insensitive)");
  emit("#  ver | help | nodes | bus | discover | power <0|1> [force] | keepload <0|1> | tele <0|1>");
  emit("#  param <id> <mode> <A> <f> <phi> <off> <kp> <ki> <kd> <aux> <dur_ms> <log_hz>");
  emit("#  arm | go [delay_ms] | halt | dump <id> | peek <id> [n]");
  emit("#  cal <id> <c0> <c1> <c2> <c3> | minduty <id> <x> | setzero <id>");
  emit("# power-monitor commands (PowerMeasurement 互換)");
  emit("#  ON OFF START STOP INFO CLEAR ZERO SELFTEST TARE SAVE GAIN OFFSET RATE AVG TRIP");
}

static void arm_targets(bool* tgt) {
  bool any = false;
  for (int i = 1; i <= UNAGI_N_NODES; i++) any |= node_known[i];
  for (int i = 1; i <= UNAGI_N_NODES; i++) tgt[i] = any ? node_known[i] : true;
}

// 返り値 true = 処理した
static bool handleGateway(char* verb) {
  auto next = [&]() -> char* { return strtok(nullptr, " \t"); };

  if (!strcmp(verb, "help")) { printHelp(); return true; }
  if (!strcmp(verb, "ver")) {
    uint8_t ch = 0; wifi_second_chan_t sc; esp_wifi_get_channel(&ch, &sc);
    emitf("OK,ver,%s,proto=%d,ch=%u,bus=%lu,load=%d,sensor=%d,vbus=%.2f,keepload=%d,nodes=%d,trial=%u",
          HEAD_FW, UNAGI_PROTO_VER, ch, (unsigned long)BUS_BAUD, loadOn, sensorOk, busV,
          dropCutsLoad ? 0 : 1, nodesKnown(), trial);
    return true;
  }
  if (!strcmp(verb, "tele")) {
    char* a = next(); streaming = a && atoi(a);
    emitf("OK,tele,%d", streaming ? 1 : 0);
    return true;
  }
  if (!strcmp(verb, "keepload")) {
    char* a = next(); dropCutsLoad = !(a && atoi(a));
    emitf("OK,keepload,%d", dropCutsLoad ? 0 : 1);
    return true;
  }
  if (!strcmp(verb, "power")) {
    char* a = next(); char* f = next();
    bool on = a && atoi(a);
    bool force = f && !strcasecmp(f, "force");
    if (on && !sensorOk && !force) { emit("ERR,power,sensor_not_ok (power 1 force で強制)"); return true; }
    if (on && alertLatched)        { emit("ERR,power,overcurrent_latched (CLEAR で解除)"); return true; }
    if (!on) bcast_stop(2);
    setLoad(on);
    emitf("OK,power,%d", loadOn ? 1 : 0);
    return true;
  }
  if (!strcmp(verb, "discover")) {
    // ID 1..10 を順に指名し、最後に ID 未設定 (0) のノードを呼ぶ
    for (int i = 1; i <= UNAGI_N_NODES + 1; i++) {
      uint8_t id = (i <= UNAGI_N_NODES) ? i : 0;
      Hdr h; hdr_init(h, UNAGI_MAGIC_HEAD, M_DISCOVER, id, trial);
      enqueue(K_DISCOVER, id, M_HELLO, 20, id ? 2 : 1, "discover", &h, sizeof(h));
    }
    emit("OK,discover");
    return true;
  }
  if (!strcmp(verb, "nodes")) {
    uint32_t now = millis();
    for (int i = 1; i <= UNAGI_N_NODES; i++) {
      if (!node_known[i]) continue;
      char b[18]; mac_str(node_mac[i], b);
      emitf("NODE,%d,%s,%lu", i, b, (unsigned long)(now - node_seen[i]));
    }
    emitf("OK,nodes,%d", nodesKnown());
    return true;
  }
  if (!strcmp(verb, "bus")) {
    const BusStats& s = bus::st;
    emitf("OK,bus,%lu,rx_ok=%lu,crc=%lu,len=%lu,other=%lu,qfull=%lu,uart_err=%lu,tx=%lu,timeouts=%lu,queue=%u",
          (unsigned long)BUS_BAUD, (unsigned long)s.rx_ok, (unsigned long)s.rx_crc, (unsigned long)s.rx_len,
          (unsigned long)s.rx_other, (unsigned long)s.rx_qfull, (unsigned long)s.rx_uart_err,
          (unsigned long)s.tx, (unsigned long)n_timeouts, rq_count);
    return true;
  }
  if (!strcmp(verb, "param")) {
    char* tok[12]; int n = 0;
    for (char* a = next(); a && n < 12; a = next()) tok[n++] = a;
    if (n != 12) { emit("ERR,param,need 12 args: id mode A f phi off kp ki kd aux dur_ms log_hz"); return true; }
    uint8_t id = (uint8_t)atoi(tok[0]);
    if (!valid_id(id)) { emitf("ERR,unknown_node,%u", id); return true; }
    MsgParam m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_PARAM, id, 0);
    m.p.mode = (uint8_t)atoi(tok[1]);
    m.p.A = atof(tok[2]); m.p.f_hz = atof(tok[3]); m.p.phi_deg = atof(tok[4]); m.p.off = atof(tok[5]);
    m.p.kp = atof(tok[6]); m.p.ki = atof(tok[7]); m.p.kd = atof(tok[8]); m.p.aux = atof(tok[9]);
    m.p.duration_ms = (uint32_t)atol(tok[10]); m.p.log_hz = (uint16_t)atoi(tok[11]);
    if (enqueue(K_PLAIN, id, M_PARAM_ACK, 30, 3, "param", &m, sizeof(m))) {
      param_set[id] = true; param_dur[id] = m.p.duration_ms;
      emitf("OK,param,%u,%u", id, crc16(&m.p, sizeof(m.p)));
    }
    return true;
  }
  if (!strcmp(verb, "arm")) {
    trial++;
    emitf("OK,arm,%u", trial);
    for (int i = 1; i <= UNAGI_N_NODES; i++) { ready_ok[i] = false; rp_on[i] = false; }
    bool tgt[UNAGI_N_NODES + 1]; arm_targets(tgt);
    for (int i = 1; i <= UNAGI_N_NODES; i++) {
      if (!tgt[i]) continue;
      Hdr h; hdr_init(h, UNAGI_MAGIC_HEAD, M_ARM, i, trial);
      enqueue(K_ARM, i, M_READY, 100, node_known[i] ? 2 : 1, "arm", &h, sizeof(h));
    }
    return true;
  }
  if (!strcmp(verb, "go")) {
    if (alertLatched) { emit("ERR,go,overcurrent_latched"); return true; }
    if (trial == 0)   { emit("ERR,go,arm_first"); return true; }
    char* a = next(); int dly = a ? atoi(a) : 300;
    if (dly < 50) dly = 50;
    if (dly > 5000) dly = 5000;
    MsgStart m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_START, 0, trial);
    drop_queued(K_START);
    if (enqueue(K_START, 0, 0, 0, 1, "go", &m, sizeof(m), (uint16_t)dly)) {   // OK,go は送った瞬間に出る
      // 列の先頭へ回す（残っている ARM の再送などより先に出す）
      if (rq_count > 1) {
        static Req tmp; tmp = rq[(rq_head + rq_count - 1) % RQ_N];
        for (uint8_t j = rq_count - 1; j > 0; j--) rq[(rq_head + j) % RQ_N] = rq[(rq_head + j - 1) % RQ_N];
        rq[rq_head] = tmp;
      }
    }
    return true;
  }
  if (!strcmp(verb, "halt")) { bcast_stop(1); emit("OK,halt"); return true; }
  if (!strcmp(verb, "dump")) {
    char* a = next(); uint8_t id = a ? (uint8_t)atoi(a) : 0;
    if (dm.active) { emit("ERR,dump,busy"); return true; }
    if (!valid_id(id)) { emitf("ERR,unknown_node,%u", id); return true; }
    dm = DumpSM(); dm.active = true; dm.id = id; dm.n_blocks = 1;
    return true;
  }
  if (!strcmp(verb, "peek")) {
    char* a = next(); char* b = next();
    uint8_t id = a ? (uint8_t)atoi(a) : 0;
    if (!valid_id(id)) { emitf("ERR,unknown_node,%u", id); return true; }
    MsgPeekReq q = {}; hdr_init(q.h, UNAGI_MAGIC_HEAD, M_PEEK_REQ, id, trial);
    int n = b ? atoi(b) : 64;
    if (n < 1) n = 1;
    if (n > 500) n = 500;
    q.n_avg = (uint16_t)n;
    enqueue(K_PLAIN, id, M_PEEK, (uint16_t)(n * 3 / 2 + 100), 1, "peek", &q, sizeof(q));
    return true;
  }
  // 注意: "ZERO" は電力計の積算リセット（互換）。関節のゼロ点は "setzero <id>"
  if (!strcmp(verb, "cal") || !strcmp(verb, "minduty") || !strcmp(verb, "setzero")) {
    char* a = next(); uint8_t id = a ? (uint8_t)atoi(a) : 0;
    if (!valid_id(id)) { emitf("ERR,unknown_node,%u", id); return true; }
    MsgCalSet m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_CAL_SET, id, 0);
    if (verb[0] == 'c') {
      int n = 0;
      for (char* x = next(); x && n < 4; x = next()) m.c[n++] = atof(x);
      if (n != 4) { emit("ERR,cal,need id c0 c1 c2 c3"); return true; }
      m.flags = CAL_F_SET_COEF;
    } else if (verb[0] == 'm') {
      char* x = next(); if (!x) { emit("ERR,minduty,need id x"); return true; }
      m.min_duty = atof(x); m.flags = CAL_F_SET_MD;
    } else {
      m.flags = CAL_F_ZERO;
    }
    // setzero は2回やるとずれるので再送しない（cal / minduty は同じ値の上書きなので再送可）
    uint8_t tries = (m.flags & CAL_F_ZERO) ? 1 : 2;
    if (enqueue(K_PLAIN, id, M_CAL_ACK, 500, tries, verb, &m, sizeof(m))) emitf("OK,%s,%u", verb, id);
    return true;
  }
  return false;
}

static void handleLegacy(char* verb) {
  // PowerMeasurement 互換（引数は strtok の続きから取る）
  for (char* p = verb; *p; ++p) *p = toupper(*p);
  char* a = strtok(nullptr, " \t");
  if (!strcmp(verb, "ON")) {
    if (!sensorOk) { emit("{\"err\":\"sensor not ok, refusing to close the switch\"}"); return; }
    setLoad(true); emit("{\"ack\":\"ON\"}");
  }
  else if (!strcmp(verb, "OFF"))      { bcast_stop(2); setLoad(false); emit("{\"ack\":\"OFF\"}"); }
  else if (!strcmp(verb, "START"))    { streaming = true;  emit("{\"ack\":\"START\"}"); }
  else if (!strcmp(verb, "STOP"))     { streaming = false; emit("{\"ack\":\"STOP\"}"); }
  else if (!strcmp(verb, "INFO"))     { sendInfo(); }
  else if (!strcmp(verb, "CLEAR"))    { clearAlert(); emit("{\"ack\":\"CLEAR\"}"); }
  else if (!strcmp(verb, "ZERO"))     { charge_mAh = 0; energy_Wh = 0; emit("{\"ack\":\"ZERO\"}"); }
  else if (!strcmp(verb, "SELFTEST")) { selfTest(); }
  else if (!strcmp(verb, "TARE"))     { tare(); sendInfo(); }
  else if (!strcmp(verb, "SAVE"))     { calSave(); emit("{\"ack\":\"SAVE\"}"); }
  else if (!strcmp(verb, "GAIN")) {
    float g = a ? atof(a) : 0;
    if (g > 0.5f && g < 2.0f) { calGain = g; emit("{\"ack\":\"GAIN\"}"); } else emit("{\"err\":\"gain 0.5..2.0\"}");
  }
  else if (!strcmp(verb, "OFFSET")) { calOffset = a ? atof(a) : 0; emit("{\"ack\":\"OFFSET\"}"); }
  else if (!strcmp(verb, "RATE")) {
    long hz = a ? atol(a) : 0;
    if (hz >= 1 && hz <= 200) { streamHz = hz; streamGapMs = 1000 / hz; emit("{\"ack\":\"RATE\"}"); }
    else emit("{\"err\":\"rate 1..200\"}");
  }
  else if (!strcmp(verb, "AVG")) {
    long n = a ? atol(a) : 0; int idx = -1;
    for (int i = 0; i < 8; i++) if (AVG_CODE[i] == n) idx = i;
    if (idx >= 0) { cfgAvgIndex = idx; ina_write(REG_CONFIG, buildConfig(idx)); emit("{\"ack\":\"AVG\"}"); }
    else emit("{\"err\":\"avg 1|4|16|64|128|256|512|1024\"}");
  }
  else if (!strcmp(verb, "TRIP")) {
    float amps = a ? atof(a) : 0, fs = 32767.0f * I_LSB_AMPS;
    if (amps > 0 && amps <= fs) { setTrip(amps); emit("{\"ack\":\"TRIP\"}"); }
    else emitf("{\"err\":\"trip 0..%.1f A\"}", fs);
  }
  else emitf("{\"err\":\"unknown command\",\"got\":\"%s\"}", verb);
}

static void handleLine(char* line) {
  char* verb = strtok(line, " \t");
  if (!verb) return;
  char lower[24]; size_t i = 0;
  for (; verb[i] && i < sizeof(lower) - 1; i++) lower[i] = tolower(verb[i]);
  lower[i] = 0;
  if (handleGateway(lower)) return;
  handleLegacy(verb);
}

template <typename T>
static void pollLines(T& src, char* buf, uint16_t& n, uint16_t cap) {
  while (src.available()) {
    char c = src.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; if (n) handleLine(buf); n = 0; }
    else if (n < cap - 1) buf[n++] = c;
  }
}

// ------------------------------ setup --------------------------------
void setup() {
  pinMode(PIN_GATE, OUTPUT);
  setLoad(false);
  pinMode(PIN_ALERT, INPUT_PULLUP);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ESP_ARDUINO_VERSION_MAJOR < 3
  Serial.setTxTimeoutMs(0);
#endif

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  ina_write(REG_CONFIG, 0x8000); delay(5);
  ina_write(REG_CONFIG, buildConfig(cfgAvgIndex));
  ina_write(REG_CALIB, (uint16_t)lround(0.00512f / (I_LSB_AMPS * R_SHUNT_OHMS)));
  uint16_t manuf = 0, die = 0;
  sensorOk = ina_read(REG_MANUF, manuf) && ina_read(REG_DIEID, die) && manuf == MANUF_TI && die == DIE_INA226;
  calLoad();
  prefs.begin("unagi", true); br_node = prefs.getUChar("calnode", 0); prefs.end();
  if (!valid_id(br_node)) br_node = 0;
  setTrip(tripAmps);
  clearAlert();
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT), onAlert, FALLING);

  // --- RS-485 バス ---
  sched_mtx = xSemaphoreCreateMutex();
  bus::begin(PIN_BUS_RX, PIN_BUS_TX, PIN_BUS_DE, UNAGI_MAGIC_NODE, 32);
  xTaskCreate(beacon_task, "beacon", 3072, nullptr, 10, &beacon_h);

  // --- WiFi: PC 接続用の AP のみ ---
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(NET_SSID, NET_PASS, UNAGI_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);

  server.begin();
  server.setNoDelay(true);
  bserver.begin();
  bserver.setNoDelay(true);
  streamGapMs = 1000 / streamHz;

  delay(300);
  Serial.printf("UNAGI head %s  AP \"%s\" %s:%d  RS-485 %lu bps  INA226=%s\n", HEAD_FW, NET_SSID,
                WiFi.softAPIP().toString().c_str(), TCP_PORT, (unsigned long)BUS_BAUD, sensorOk ? "OK" : "NOT FOUND");
  Serial.printf("calibration bridge: %s:%d -> node %u (NODE <id> to change)\n",
                WiFi.softAPIP().toString().c_str(), BRIDGE_PORT, br_node);
  if (!sensorOk)
    Serial.printf("INA226 NOT FOUND - manuf 0x%04X die 0x%04X  → 'power 1 force' でノード給電は可能\n", manuf, die);
}

// ------------------------------- loop --------------------------------
void loop() {
  static char sbuf[160]; static uint16_t sn = 0;
  static char nbuf[160]; static uint16_t nn = 0;
  static BusRx it;

  // --- TCP クライアント（1台のみ）---
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  WiFiClient inc = server.accept();
#else
  WiFiClient inc = server.available();
#endif
  if (inc) {
    if (client) client.stop();
    client = inc; client.setNoDelay(true); nn = 0;
    sendInfo();
  }
  if (client && !client.connected()) {
    client.stop();
    if (dropCutsLoad && loadOn) {
      bcast_stop(3); setLoad(false);
      emit("{\"evt\":\"link lost, load off\"}");
    }
  }
  if (client && client.connected()) pollLines(client, nbuf, nn, sizeof(nbuf));
  pollLines(Serial, sbuf, sn, sizeof(sbuf));
  br_poll();

  // --- RS-485 ---
  for (int k = 0; k < 16 && bus::recv(it, 0); k++) handle_rx(it);
  scheduler();

  // --- 過電流 ---
  if (alertFlag) {
    alertFlag = false; alertLatched = true;
    bcast_stop(1); setLoad(false);
    emit("{\"evt\":\"OVERCURRENT\"}");
    br_printf("EVT head OVERCURRENT -> node power OFF");
  }

  // --- INA226 ---
  uint32_t now = millis();
  if (ina_fast) {
    static uint32_t last_fast_us = 0;
    uint32_t nu = micros();
    if (nu - last_fast_us >= 2000) { last_fast_us = nu; fast_sample(); }   // INA226 は約 2.2 ms ごとに更新
  } else if (now - lastSample >= 5) { lastSample = now; sample(); }
  if (streaming && now - lastStream >= streamGapMs) { lastStream = now; sendTelemetry(); }

  // 次のフレームが来たらすぐ起きる（最大 1 tick 待ち）
  if (bus::recv(it, 1)) { handle_rx(it); scheduler(); }
}
