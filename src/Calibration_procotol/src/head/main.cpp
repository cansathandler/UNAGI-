// =====================================================================
//  UNAGI head — 電源監視 (v31, INA226) + ESP-NOW ゲートウェイ   protocol v2
//  Target : Seeed XIAO ESP32-C3 / Board: Unagi_head_V2 v31 (R3 = 2 mOhm)
//
//  ・PowerMeasurement の機能（JSON テレメトリ、ON/OFF/INFO/SELFTEST/TRIP...）は
//    そのまま。既存の Unagi_head_gui.py が使えます。
//  ・ゲートウェイ用コマンドを追加（大文字小文字は区別しない）。一覧は "help"。
//  ・ロードスイッチ (PIN_GATE) がノード10台への給電そのもの。
//  ・過電流で スイッチ OFF + 全ノードへ STOP。
//
//  v1 (Calibration_procotol) からの修正点
//   1. ESP-NOW 受信コールバックはキューに積むだけ。TCP/Serial への書き出しは
//      loop() だけが行う（v1 はコールバック＝WiFi タスク内で TCP に書いていて、
//      ダンプ中に詰まる / 行が混ざる原因になっていた）。
//   2. TCP・Serial の行読みがノンブロッキング（readStringUntil の 1 s 待ちを廃止）。
//   3. ビーコンは 20 Hz 常時。ノードは移動窓で同期を維持するので "sync" 不要。
//   4. ダンプはヘッドが自分で全ブロックを順に要求・再送する（PC 往復なし）。
//   5. 走行開始は "go"（テレメトリの START/STOP と名前が衝突しないように）。
// =====================================================================

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <Preferences.h>
#include "unagi_protocol.h"

#define HEAD_FW            "head-v31-gw-2.0.1"

// ------------------------- build options -----------------------------
#define GATE_ACTIVE_HIGH   1
#define R_SHUNT_OHMS       0.002f
#define I_LSB_AMPS         0.00125f

#define NET_SSID           "UNAGI-HEAD"
#define NET_PASS           "unagi12345"
#define TCP_PORT           5555

#define DROP_LINK_CUTS_LOAD 1
#define DEFAULT_TRIP_A     28.0f
#define BEACON_HZ          20

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ------------------------------ pins ---------------------------------
static const uint8_t PIN_SDA   = 6;   // D4
static const uint8_t PIN_SCL   = 7;   // D5
static const uint8_t PIN_ALERT = 4;   // D2
static const uint8_t PIN_GATE  = 3;   // D1

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

// ---------------------------- state ----------------------------------
WiFiServer  server(TCP_PORT);
WiFiClient  client;
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
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t  node_mac[UNAGI_N_NODES + 1][6];
static bool     node_known[UNAGI_N_NODES + 1] = {false};
static uint32_t node_seen[UNAGI_N_NODES + 1] = {0};
static uint16_t trial = 0;
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
  emitf("{\"info\":1,\"board\":\"v31\",\"fw\":\"%s\",\"manuf\":\"0x%04X\",\"die\":\"0x%04X\","
        "\"cfg\":\"0x%04X\",\"cal\":%u,\"rshunt\":%.5f,\"ilsb\":%.6f,\"fs\":%.2f,\"avg\":%u,\"hz\":%lu,"
        "\"trip\":%.2f,\"gain\":%.5f,\"offset\":%.5f,\"ok\":%d,\"ch\":%d,\"nodes\":%d,\"ip\":\"%s\"}",
        HEAD_FW, manuf, die, cfg, cal, R_SHUNT_OHMS, I_LSB_AMPS, 32767.0f * I_LSB_AMPS,
        AVG_CODE[cfgAvgIndex], (unsigned long)streamHz, tripAmps, calGain, calOffset,
        sensorOk ? 1 : 0, UNAGI_CHANNEL, nodesKnown(), WiFi.softAPIP().toString().c_str());
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

// ===================== ESP-NOW ======================================
struct RxItem { uint8_t src[6]; uint8_t len; uint8_t data[250]; };
static QueueHandle_t rxq;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src = info->src_addr;
#else
static void on_recv(const uint8_t* src, const uint8_t* data, int len) {
#endif
  if (len < (int)sizeof(Hdr) || len > 250 || data[0] != UNAGI_MAGIC_NODE) return;
  RxItem it; memcpy(it.src, src, 6); it.len = (uint8_t)len; memcpy(it.data, data, len);
  xQueueSend(rxq, &it, 0);
}

static void mac_str(const uint8_t* m, char* out) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}
static void add_peer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.ifidx   = WIFI_IF_AP;          // ヘッドは AP モードなので AP インターフェースから送る
  p.encrypt = false;
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK) emitf("ERR,add_peer,%s", esp_err_to_name(e));
}
static esp_err_t now_send(const uint8_t* dst, const void* p, size_t n) {
  esp_err_t e = ESP_FAIL;
  for (int k = 0; k < 4; k++) {
    e = esp_now_send(dst, (const uint8_t*)p, n);
    if (e == ESP_OK) break;
    delay(2);
  }
  return e;
}
static void bcast(const void* p, size_t n, int times = 1, int gap_ms = 5) {
  for (int k = 0; k < times; k++) { now_send(BCAST, p, n); if (k + 1 < times) delay(gap_ms); }
}
static bool unicast(uint8_t id, const void* p, size_t n) {
  if (id < 1 || id > UNAGI_N_NODES || !node_known[id]) { emitf("ERR,unknown_node,%u", id); return false; }
  esp_err_t e = now_send(node_mac[id], p, n);
  if (e != ESP_OK) { emitf("ERR,send,%u,%s", id, esp_err_to_name(e)); return false; }
  return true;
}
static void learn_node(uint8_t id, const uint8_t* mac) {
  if (id < 1 || id > UNAGI_N_NODES) return;
  node_seen[id] = millis();
  if (node_known[id] && memcmp(node_mac[id], mac, 6) == 0) return;
  char b[18]; mac_str(mac, b);
  if (node_known[id]) {
    char o[18]; mac_str(node_mac[id], o);
    emitf("WARN,node_mac_changed,%u,%s,%s  (同じ ID のノードが2台ある？)", id, o, b);
    esp_now_del_peer(node_mac[id]);
  }
  memcpy(node_mac[id], mac, 6);
  node_known[id] = true;
  add_peer(mac);
  emitf("MAC,%u,%s", id, b);
}
static void bcast_stop(uint8_t reason) {
  MsgStop m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_STOP, 0, trial); m.reason = reason;
  bcast(&m, sizeof(m), 3, 3);
}

// ---- ビーコン: 20 Hz 常時。タイムスタンプは送信直前に取る ----
static void beacon_task(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t seq = 1;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / BEACON_HZ));
    MsgBeacon b = {}; hdr_init(b.h, UNAGI_MAGIC_HEAD, M_BEACON, 0, trial);
    b.seq = seq++;
    b.t_head_us = esp_timer_get_time();
    esp_now_send(BCAST, (const uint8_t*)&b, sizeof(b));
  }
}

// ---- ダンプ: ヘッドが自分でブロックを順に取りに行く ----
struct DumpSM {
  bool active = false; uint8_t id = 0; uint16_t block = 0, n_blocks = 1, n_total = 0;
  uint8_t tries = 0; uint32_t sent_ms = 0; uint16_t missing = 0;
} dm;
static const uint32_t DUMP_TIMEOUT_MS = 60;
static const uint8_t  DUMP_TRIES = 6;

static void dump_request() {
  MsgDumpReq q = {}; hdr_init(q.h, UNAGI_MAGIC_HEAD, M_DUMP_REQ, dm.id, trial); q.block = dm.block;
  unicast(dm.id, &q, sizeof(q));
  dm.sent_ms = millis();
}
static void dump_finish() {
  emitf("DUMPEND,%u,%u,%u,%u", dm.id, dm.n_total, dm.n_blocks, dm.missing);
  dm.active = false;
}
static void dump_advance() {
  dm.block++; dm.tries = 0;
  if (dm.block >= dm.n_blocks) dump_finish(); else dump_request();
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
static void dump_poll() {
  if (!dm.active || millis() - dm.sent_ms < DUMP_TIMEOUT_MS) return;
  if (++dm.tries >= DUMP_TRIES) {
    emitf("DUMPMISS,%u,%u", dm.id, dm.block);
    dm.missing++;
    if (dm.block == 0) { dm.n_blocks = 0; dump_finish(); return; }   // 応答なし
    dump_advance();
  } else dump_request();
}

// ---- 受信処理（loop から呼ぶ）----
static void handle_rx(const RxItem& it) {
  Hdr h; memcpy(&h, it.data, sizeof(h));
  learn_node(h.node_id, it.src);
  char mb[18]; mac_str(it.src, mb);

  switch (h.type) {
    case M_HELLO: {
      if (it.len < sizeof(MsgHello)) return;
      MsgHello m; memcpy(&m, it.data, sizeof(m));
      emitf("HELLO,%u,%s,%08lX,%u,%u,%g,%g,%g,%g,%.3f", h.node_id, mb, (unsigned long)m.fw, m.proto,
            m.state, m.c[0], m.c[1], m.c[2], m.c[3], m.min_duty);
      break;
    }
    case M_PARAM_ACK: {
      MsgParamAck m; memcpy(&m, it.data, sizeof(m));
      emitf("PACK,%u,%u,%u", h.node_id, m.crc, m.mode);
      break;
    }
    case M_READY: {
      MsgReady m; memcpy(&m, it.data, sizeof(m));
      emitf("READY,%u,%u,%u,%u,%u,%u,%.1f,%.3f,%lu,%d,%d", h.node_id, h.trial, m.ready, m.why, m.param_crc,
            m.sync_n, m.sigma_us, m.skew_ppm, (unsigned long)m.free_heap, m.v1, m.v2);
      break;
    }
    case M_REPORT: {
      MsgReport m; memcpy(&m, it.data, sizeof(m));
      emitf("REPORT,%u,%u,%lu,%lu,%lu,%lu,%.3f,%u,%u,%ld", h.node_id, h.trial,
            (unsigned long)m.n_samples, (unsigned long)m.ticks, (unsigned long)m.missed,
            (unsigned long)m.max_jitter_us, m.rms_err_deg, m.loop_us_max, m.abort_reason,
            (long)m.start_err_us);
      break;
    }
    case M_DUMP_DATA: {
      static MsgDumpData m; memset(&m, 0, sizeof(m));
      memcpy(&m, it.data, it.len < sizeof(m) ? it.len : sizeof(m));
      dump_on_data(m);
      break;
    }
    case M_CAL_ACK: {
      MsgCalAck m; memcpy(&m, it.data, sizeof(m));
      emitf("CALACK,%u,%.9g,%.9g,%.9g,%.9g,%.4f", h.node_id, m.c[0], m.c[1], m.c[2], m.c[3], m.min_duty);
      break;
    }
    case M_PEEK: {
      MsgPeek m; memcpy(&m, it.data, sizeof(m));
      emitf("PEEK,%u,%u,%u,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.3f,%u", h.node_id, m.state, m.n,
            m.v1, m.v2, m.diff, m.diff_sd, m.deg, m.sigma_us, m.skew_ppm, m.sync_n);
      break;
    }
  }
}

// ===================== commands =====================================
static void printHelp() {
  emit("# gateway commands (case-insensitive)");
  emit("#  ver | help | nodes | discover | power <0|1> [force] | keepload <0|1> | tele <0|1>");
  emit("#  param <id> <mode> <A> <f> <phi> <off> <kp> <ki> <kd> <aux> <dur_ms> <log_hz>");
  emit("#  arm | go [delay_ms] | halt | dump <id> | peek <id> [n]");
  emit("#  cal <id> <c0> <c1> <c2> <c3> | minduty <id> <x> | setzero <id>");
  emit("# power-monitor commands (PowerMeasurement 互換)");
  emit("#  ON OFF START STOP INFO CLEAR ZERO SELFTEST TARE SAVE GAIN OFFSET RATE AVG TRIP");
}

// 返り値 true = 処理した
static bool handleGateway(char* verb, char* rest) {
  auto next = [&]() -> char* { return strtok(nullptr, " \t"); };
  (void)rest;

  if (!strcmp(verb, "help")) { printHelp(); return true; }
  if (!strcmp(verb, "ver")) {
    uint8_t ch = 0; wifi_second_chan_t sc; esp_wifi_get_channel(&ch, &sc);
    emitf("OK,ver,%s,proto=%d,ch=%u,load=%d,sensor=%d,vbus=%.2f,keepload=%d,nodes=%d,trial=%u",
          HEAD_FW, UNAGI_PROTO_VER, ch, loadOn, sensorOk, busV, dropCutsLoad ? 0 : 1, nodesKnown(), trial);
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
    Hdr h; hdr_init(h, UNAGI_MAGIC_HEAD, M_DISCOVER, 0, trial);
    bcast(&h, sizeof(h), 2, 80);
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
  if (!strcmp(verb, "param")) {
    char* tok[12]; int n = 0;
    for (char* a = next(); a && n < 12; a = next()) tok[n++] = a;
    if (n != 12) { emit("ERR,param,need 12 args: id mode A f phi off kp ki kd aux dur_ms log_hz"); return true; }
    uint8_t id = (uint8_t)atoi(tok[0]);
    MsgParam m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_PARAM, id, 0);
    m.p.mode = (uint8_t)atoi(tok[1]);
    m.p.A = atof(tok[2]); m.p.f_hz = atof(tok[3]); m.p.phi_deg = atof(tok[4]); m.p.off = atof(tok[5]);
    m.p.kp = atof(tok[6]); m.p.ki = atof(tok[7]); m.p.kd = atof(tok[8]); m.p.aux = atof(tok[9]);
    m.p.duration_ms = (uint32_t)atol(tok[10]); m.p.log_hz = (uint16_t)atoi(tok[11]);
    if (unicast(id, &m, sizeof(m))) emitf("OK,param,%u,%u", id, crc16(&m.p, sizeof(m.p)));
    return true;
  }
  if (!strcmp(verb, "arm")) {
    trial++;
    Hdr h; hdr_init(h, UNAGI_MAGIC_HEAD, M_ARM, 0, trial);
    bcast(&h, sizeof(h), 2, 60);
    emitf("OK,arm,%u", trial);
    return true;
  }
  if (!strcmp(verb, "go")) {
    if (alertLatched) { emit("ERR,go,overcurrent_latched"); return true; }
    if (trial == 0)   { emit("ERR,go,arm_first"); return true; }
    char* a = next(); int dly = a ? atoi(a) : 300;
    if (dly < 50) dly = 50;
    if (dly > 5000) dly = 5000;
    MsgStart m = {}; hdr_init(m.h, UNAGI_MAGIC_HEAD, M_START, 0, trial);
    m.t_start_head_us = esp_timer_get_time() + (int64_t)dly * 1000;
    last_go_T = m.t_start_head_us;
    bcast(&m, sizeof(m), 3, 5);           // 絶対時刻なので何回届いても同じ
    emitf("OK,go,%u,%lld", trial, (long long)m.t_start_head_us);
    return true;
  }
  if (!strcmp(verb, "halt")) { bcast_stop(1); emit("OK,halt"); return true; }
  if (!strcmp(verb, "dump")) {
    char* a = next(); uint8_t id = a ? (uint8_t)atoi(a) : 0;
    if (dm.active) { emit("ERR,dump,busy"); return true; }
    if (id < 1 || id > UNAGI_N_NODES || !node_known[id]) { emitf("ERR,unknown_node,%u", id); return true; }
    dm = DumpSM(); dm.active = true; dm.id = id; dm.n_blocks = 1;
    dump_request();
    return true;
  }
  if (!strcmp(verb, "peek")) {
    char* a = next(); char* b = next();
    uint8_t id = a ? (uint8_t)atoi(a) : 0;
    MsgPeekReq q = {}; hdr_init(q.h, UNAGI_MAGIC_HEAD, M_PEEK_REQ, id, trial);
    q.n_avg = b ? (uint16_t)atoi(b) : 64;
    unicast(id, &q, sizeof(q));
    return true;
  }
  // 注意: "ZERO" は電力計の積算リセット（互換）。関節のゼロ点は "setzero <id>"
  if (!strcmp(verb, "cal") || !strcmp(verb, "minduty") || !strcmp(verb, "setzero")) {
    char* a = next(); uint8_t id = a ? (uint8_t)atoi(a) : 0;
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
    if (unicast(id, &m, sizeof(m))) emitf("OK,%s,%u", verb, id);
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
  if (handleGateway(lower, nullptr)) return;
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
  setTrip(tripAmps);
  clearAlert();
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT), onAlert, FALLING);

  // --- WiFi: AP のみ。チャンネルを固定（ESP-NOW はノードと同一チャンネル必須）---
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(NET_SSID, NET_PASS, UNAGI_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);

  rxq = xQueueCreate(32, sizeof(RxItem));
  if (esp_now_init() != ESP_OK) Serial.println("ERR esp_now_init");
  esp_now_register_recv_cb(on_recv);
  add_peer(BCAST);
  xTaskCreate(beacon_task, "beacon", 3072, nullptr, 10, nullptr);

  server.begin();
  server.setNoDelay(true);
  streamGapMs = 1000 / streamHz;

  delay(300);
  uint8_t ch = 0; wifi_second_chan_t sc; esp_wifi_get_channel(&ch, &sc);
  Serial.printf("UNAGI head %s  AP \"%s\" %s:%d  ch=%u  INA226=%s\n", HEAD_FW, NET_SSID,
                WiFi.softAPIP().toString().c_str(), TCP_PORT, ch, sensorOk ? "OK" : "NOT FOUND");
  if (!sensorOk)
    Serial.printf("INA226 NOT FOUND - manuf 0x%04X die 0x%04X  → 'power 1 force' でノード給電は可能\n", manuf, die);
}

// ------------------------------- loop --------------------------------
void loop() {
  static char sbuf[160]; static uint16_t sn = 0;
  static char nbuf[160]; static uint16_t nn = 0;

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

  // --- ESP-NOW 受信 ---
  RxItem it;
  for (int k = 0; k < 8 && xQueueReceive(rxq, &it, 0) == pdTRUE; k++) handle_rx(it);
  dump_poll();

  // --- 過電流 ---
  if (alertFlag) {
    alertFlag = false; alertLatched = true;
    bcast_stop(1); setLoad(false);
    emit("{\"evt\":\"OVERCURRENT\"}");
  }

  // --- INA226 ---
  uint32_t now = millis();
  if (now - lastSample >= 5) { lastSample = now; sample(); }
  if (streaming && now - lastStream >= streamGapMs) { lastStream = now; sendTelemetry(); }
  delay(1);
}
