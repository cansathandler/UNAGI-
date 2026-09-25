// =====================================================================
//  UNAGI head — power monitor (v31) + ESP-NOW gateway
//  Target : Seeed XIAO ESP32-C3  (U2)
//  Board  : Unagi_head_V2 v31 — R3 = 2 mOhm, 2S, 20 A cont / 24 A peak
//  Sensor : TI INA226, I2C addr 0x40
//
//  既存の v31 電源監視ファームに、モータノード10台を束ねる ESP-NOW
//  ゲートウェイを足したものです。両方が同じ基板の上で動きます。
//
//  ・JSON テレメトリと大文字コマンド (ON/OFF/INFO/SELFTEST/TRIP...) は
//    従来どおりなので、既存の Unagi_head_gui.py がそのまま使えます。
//  ・小文字のコマンド (sync/param/arm/start/dump/power/ver...) が
//    ゲートウェイ用で、pc/unagi_test.py が使います。
//  ・ロードスイッチ (PIN_GATE) がノードへの給電そのものなので、
//    ゲートウェイの "power" コマンドは setLoad() に直結しています。
//  ・過電流を検知したら、スイッチを開くと同時に全ノードへ STOP を
//    ブロードキャストします。
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "unagi_protocol.h"

// ------------------------- build options -----------------------------

#define HEAD_FW            "head-v31-gw-0.2.0"

#define GATE_ACTIVE_HIGH   1         // v31 has the 2N7002 level shifter
#define R_SHUNT_OHMS       0.002f    // R3 on the assembled board
#define I_LSB_AMPS         0.00125f  // 1.25 mA/bit -> +/-40.96 A full scale

#define NET_SSID           "UNAGI-HEAD"
#define NET_PASS           "unagi12345"
#define TCP_PORT           5555

#define DROP_LINK_CUTS_LOAD 1
#define DEFAULT_TRIP_A     28.0f
#define NODE_BOOT_MS       2500      // 給電からノードが立ち上がるまで

// ------------------------------ pins ---------------------------------
//  回路図のシンボルは SAMD21 版 XIAO のピン名なので注意。
//  ESP32-C3 では D0=GPIO2 D1=3 D2=4 D3=5 D4=6 D5=7 D6=21 D7=20 D8=8 D9=9 D10=10

static const uint8_t PIN_SDA   = 6;   // D4
static const uint8_t PIN_SCL   = 7;   // D5
static const uint8_t PIN_ALERT = 4;   // D2, open drain, active low
static const uint8_t PIN_GATE  = 3;   // D1, load switch gate

// --------------------------- INA226 map ------------------------------

static const uint8_t INA_ADDR      = 0x40;
static const uint8_t REG_CONFIG    = 0x00;
static const uint8_t REG_SHUNT     = 0x01;
static const uint8_t REG_BUS       = 0x02;
static const uint8_t REG_POWER     = 0x03;
static const uint8_t REG_CURRENT   = 0x04;
static const uint8_t REG_CALIB     = 0x05;
static const uint8_t REG_MASK      = 0x06;
static const uint8_t REG_ALERTLIM  = 0x07;
static const uint8_t REG_MANUF     = 0xFE;
static const uint8_t REG_DIEID     = 0xFF;

static const uint16_t MANUF_TI     = 0x5449;
static const uint16_t DIE_INA226   = 0x2260;

static const float SHUNT_LSB_V = 2.5e-6f;
static const float BUS_LSB_V   = 1.25e-3f;

static const uint16_t AVG_CODE[8] = {1, 4, 16, 64, 128, 256, 512, 1024};

static uint16_t cfgAvgIndex = 2;
static uint16_t buildConfig(uint16_t avgIdx) {
  return (0x4000) | ((avgIdx & 0x7) << 9) | (0x4 << 6) | (0x4 << 3) | 0x7;
}

// ---------------------------- state ----------------------------------

WiFiServer  server(TCP_PORT);
WiFiClient  client;
Preferences prefs;

bool     streaming   = true;
uint32_t streamHz    = 20;
uint32_t streamGapMs = 50;
uint32_t lastStream  = 0;
uint32_t lastSample  = 0;

bool     loadOn        = false;
bool     sensorOk      = false;
volatile bool alertFlag = false;
bool     alertLatched   = false;

float    busV = 0, curA = 0, powW = 0, shuntmV = 0, curRawA = 0;
double   charge_mAh = 0, energy_Wh = 0;
uint32_t lastIntegrate = 0;
float    tripAmps = DEFAULT_TRIP_A;

float    calGain   = 1.0f;
float    calOffset = 0.0f;

// ---- ゲートウェイ側の状態 ----
uint8_t  node_mac[UNAGI_N_NODES + 1][6];
bool     node_known[UNAGI_N_NODES + 1] = {false};
uint8_t  BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
volatile bool beacon_on = false;
uint32_t beacon_seq = 0;
uint16_t trial = 0;

// ------------------------- I2C helpers -------------------------------

static bool ina_write(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool ina_read(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA_ADDR, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

// --------------------------- load switch -----------------------------

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
  if (lsbs < 0)     lsbs = 0;
  ina_write(REG_ALERTLIM, (uint16_t)lsbs);
  ina_write(REG_MASK, 0x8000 | 0x0001);
}

static void clearAlert() {
  uint16_t dummy;
  ina_read(REG_MASK, dummy);
  alertLatched = false;
  alertFlag    = false;
}

// ---------------------------- sampling -------------------------------

static void sample() {
  uint16_t raw;
  if (ina_read(REG_BUS, raw))     busV    = raw * BUS_LSB_V;
  if (ina_read(REG_SHUNT, raw))   shuntmV = (int16_t)raw * SHUNT_LSB_V * 1000.0f;
  if (ina_read(REG_CURRENT, raw)) {
    curRawA = (int16_t)raw * I_LSB_AMPS;
    curA    = curRawA * calGain - calOffset;
  }
  if (ina_read(REG_POWER, raw))   powW = raw * 25.0f * I_LSB_AMPS * calGain;

  uint32_t now = millis();
  if (lastIntegrate) {
    double dt_h = (now - lastIntegrate) / 3600000.0;
    charge_mAh += curA * 1000.0 * dt_h;
    energy_Wh  += powW * dt_h;
  }
  lastIntegrate = now;
}

// ----------------------------- output --------------------------------

static void emit(const String &line) {
  Serial.println(line);
  if (client && client.connected()) client.println(line);
}
static void tx(const String &s) { emit(s); }   // ゲートウェイ用の別名

static void sendTelemetry() {
  char buf[260];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lu,\"v\":%.4f,\"i\":%.4f,\"p\":%.4f,\"sh\":%.4f,"
    "\"mah\":%.3f,\"wh\":%.5f,\"sw\":%d,\"alrt\":%d,\"vok\":%d,\"ok\":%d}",
    (unsigned long)millis(), busV, curA, powW, shuntmV,
    charge_mAh, energy_Wh, loadOn ? 1 : 0, alertLatched ? 1 : 0,
    loadOn ? 1 : 0, sensorOk ? 1 : 0);
  emit(String(buf));
}

static void sendInfo() {
  uint16_t manuf = 0, die = 0, cfg = 0, cal = 0;
  ina_read(REG_MANUF, manuf);
  ina_read(REG_DIEID, die);
  ina_read(REG_CONFIG, cfg);
  ina_read(REG_CALIB, cal);
  int nodes = 0;
  for (int i = 1; i <= UNAGI_N_NODES; i++) if (node_known[i]) nodes++;
  char buf[420];
  snprintf(buf, sizeof(buf),
    "{\"info\":1,\"board\":\"v31\",\"fw\":\"%s\",\"manuf\":\"0x%04X\",\"die\":\"0x%04X\","
    "\"cfg\":\"0x%04X\",\"cal\":%u,\"rshunt\":%.5f,\"ilsb\":%.6f,"
    "\"fs\":%.2f,\"avg\":%u,\"hz\":%lu,\"trip\":%.2f,"
    "\"gain\":%.5f,\"offset\":%.5f,\"ok\":%d,\"ch\":%d,\"nodes\":%d,\"ip\":\"%s\"}",
    HEAD_FW, manuf, die, cfg, cal, R_SHUNT_OHMS, I_LSB_AMPS,
    32767.0f * I_LSB_AMPS, AVG_CODE[cfgAvgIndex],
    (unsigned long)streamHz, tripAmps, calGain, calOffset, sensorOk ? 1 : 0,
    UNAGI_CHANNEL, nodes,
    WiFi.softAPIP().toString().c_str());
  emit(String(buf));
}

// ------------------------- calibration store -------------------------

static void calLoad() {
  prefs.begin("unagi", true);
  calGain   = prefs.getFloat("gain", 1.0f);
  calOffset = prefs.getFloat("offs", 0.0f);
  tripAmps  = prefs.getFloat("trip", DEFAULT_TRIP_A);
  prefs.end();
}

static void calSave() {
  prefs.begin("unagi", false);
  prefs.putFloat("gain", calGain);
  prefs.putFloat("offs", calOffset);
  prefs.putFloat("trip", tripAmps);
  prefs.end();
}

static void tare() {
  double acc = 0;
  for (int i = 0; i < 64; i++) { sample(); acc += curRawA * calGain; delay(5); }
  calOffset = acc / 64.0;
  charge_mAh = 0; energy_Wh = 0;
}

// ---------------------------- self test ------------------------------

static void selfTest() {
  char buf[200];
  emit("{\"test\":\"start\"}");

  uint16_t manuf = 0, die = 0;
  bool idOk = ina_read(REG_MANUF, manuf) && ina_read(REG_DIEID, die)
              && manuf == MANUF_TI && die == DIE_INA226;
  snprintf(buf, sizeof(buf),
    "{\"test\":\"i2c\",\"pass\":%d,\"manuf\":\"0x%04X\",\"die\":\"0x%04X\"}",
    idOk ? 1 : 0, manuf, die);
  emit(String(buf));
  if (!idOk) { emit("{\"test\":\"abort\",\"why\":\"INA226 not responding\"}"); return; }

  uint16_t cal = 0;
  ina_read(REG_CALIB, cal);
  snprintf(buf, sizeof(buf), "{\"test\":\"cal\",\"pass\":%d,\"read\":%u,\"expect\":2048}",
           cal == 2048 ? 1 : 0, cal);
  emit(String(buf));

  setLoad(false);
  uint32_t t0 = millis();
  float vmin = 1e9;
  while (millis() - t0 < 8000) { sample(); vmin = min(vmin, busV); delay(20); }
  snprintf(buf, sizeof(buf),
    "{\"test\":\"switch_off\",\"pass\":%d,\"vbus\":%.3f,\"note\":\"expect <0.5 V\"}",
    vmin < 0.5f ? 1 : 0, vmin);
  emit(String(buf));

  double acc = 0;
  for (int i = 0; i < 32; i++) { sample(); acc += curRawA; delay(5); }
  float off = acc / 32.0;
  snprintf(buf, sizeof(buf),
    "{\"test\":\"zero\",\"pass\":%d,\"amps\":%.4f,\"note\":\"expect <0.05 A\"}",
    fabs(off) < 0.05f ? 1 : 0, off);
  emit(String(buf));

  emit("{\"test\":\"done\",\"note\":\"switch left OFF\"}");
}

// ===================== ESP-NOW ゲートウェイ ==========================

static void mac_str(const uint8_t* m, char* out) {
  snprintf(out, 20, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void learn_peer(uint8_t id, const uint8_t* src) {
  if (id < 1 || id > UNAGI_N_NODES || node_known[id]) return;
  memcpy(node_mac[id], src, 6);
  node_known[id] = true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, src, 6);
  p.channel = UNAGI_CHANNEL; p.encrypt = false;
  esp_now_add_peer(&p);
  char b[20]; mac_str(src, b);
  tx(String("MAC,") + id + "," + b);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void on_recv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  const uint8_t* src = info->src_addr;
#else
void on_recv(const uint8_t* mac, const uint8_t* d, int len) {
  const uint8_t* src = mac;
#endif
  if (len < (int)sizeof(Hdr)) return;
  const Hdr* h = (const Hdr*)d;
  uint8_t id = h->node_id;
  learn_peer(id, src);

  switch (h->type) {
    case MSG_HELLO: {
      if (len < (int)sizeof(MsgHello)) break;
      const MsgHello* m = (const MsgHello*)d;
      char b[20]; mac_str(m->mac, b);
      tx(String("HELLO,") + id + "," + b + "," + String(m->fw_hash, HEX));
      break;
    }
    case MSG_READY: {
      const MsgReady* m = (const MsgReady*)d;
      tx(String("READY,") + id + "," + m->ready + "," + String(m->sigma_us, 1) + ","
         + String((m->b - 1.0) * 1e6, 3) + "," + m->param_crc + "," + m->free_ram + ","
         + m->hall1_bias + "," + m->hall2_bias);
      break;
    }
    case MSG_REPORT: {
      const MsgReport* m = (const MsgReport*)d;
      tx(String("REPORT,") + id + "," + m->n_samples + "," + (long)m->t_start_local_us + ","
         + m->missed + "," + m->max_jitter_us + "," + String(m->rms_err_deg, 3) + ","
         + String(m->adc_us_max, 0));
      break;
    }
    case MSG_CAL_ACK: {
      const MsgCalAck* m = (const MsgCalAck*)d;
      tx(String("CALACK,") + id + "," + String(m->c[0], 6) + "," + String(m->c[1], 6) + ","
         + String(m->c[2], 9) + "," + String(m->c[3], 12));
      break;
    }
    case MSG_DUMP_DATA: {
      const MsgDumpData* m = (const MsgDumpData*)d;
      String s = String("DUMP,") + id + "," + m->block + "," + m->n;
      for (int i = 0; i < m->n; i++)
        for (int k = 0; k < 4; k++) { s += ","; s += m->d[i][k]; }
      tx(s);
      break;
    }
  }
}

static void bcast_stop(uint8_t reason) {
  MsgStop m = {}; m.h = {MSG_STOP, 0, trial}; m.reason = reason;
  esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
}

void beacon_task(void*) {
  for (;;) {
    if (beacon_on) {
      MsgBeacon b = {}; b.h = {MSG_BEACON, 0, trial};
      b.seq = beacon_seq++; b.t_head_us = esp_timer_get_time();
      esp_now_send(BCAST, (uint8_t*)&b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(100));      // 10 Hz
  }
}

// 小文字のゲートウェイコマンド。処理したら true。
static bool handleGateway(const String& c) {
  if (c.startsWith("ver")) {
    tx(String("OK,ver,") + HEAD_FW + ",ch=" + UNAGI_CHANNEL
       + ",load=" + (loadOn ? 1 : 0) + ",sensor=" + (sensorOk ? 1 : 0));
    return true;
  }
  if (c.startsWith("power")) {
    int v = c.substring(6).toInt();
    if (v && !sensorOk) { tx("ERR,power,sensor_not_ok"); return true; }
    setLoad(v != 0);
    tx(String("OK,power,") + (loadOn ? 1 : 0) + "," + (loadOn ? NODE_BOOT_MS : 0));
    return true;
  }
  if (c.startsWith("ping")) {
    Hdr h = {MSG_HELLO, 0, trial};
    esp_err_t e = esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
    tx(String("OK,ping,") + (int)e + "," + UNAGI_CHANNEL);
    return true;
  }
  if (c.startsWith("sync")) {
    int n = c.substring(5).toInt(); if (n <= 0) n = 300;
    MsgSyncBegin m = {}; m.h = {MSG_SYNC_BEGIN, 0, trial}; m.n_points = (uint16_t)n;
    for (int k = 0; k < 3; k++) { esp_now_send(BCAST, (uint8_t*)&m, sizeof(m)); delay(20); }
    beacon_seq = 0; beacon_on = true;
    tx(String("OK,sync,") + n);
    return true;
  }
  if (c.startsWith("beacon_off")) { beacon_on = false; tx("OK,beacon_off"); return true; }
  if (c.startsWith("param")) {
    int id = 0; float v[11] = {0};
    sscanf(c.c_str(), "param %d %f %f %f %f %f %f %f %f %f %f %f",
           &id, &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9],&v[10]);
    MsgParam p = {}; p.h = {MSG_PARAM, (uint8_t)id, trial};
    p.A_deg=v[0]; p.f_hz=v[1]; p.phi_deg=v[2]; p.off_deg=v[3];
    p.kp=v[4]; p.ki=v[5]; p.kd=v[6];
    p.duration_ms=(uint16_t)v[7]; p.log_hz=(uint16_t)v[8];
    p.mode=(uint8_t)v[9]; p.aux=v[10];
    if (id >= 1 && id <= UNAGI_N_NODES && node_known[id])
      esp_now_send(node_mac[id], (uint8_t*)&p, sizeof(p));
    tx(String("OK,param,") + id + "," + crc16((uint8_t*)&p, sizeof(p)));
    return true;
  }
  if (c.startsWith("cal ")) {
    int id = 0; float k[4] = {0};
    if (sscanf(c.c_str(), "cal %d %f %f %f %f", &id, &k[0], &k[1], &k[2], &k[3]) == 5) {
      MsgCalSet m = {}; m.h = {MSG_CAL_SET, (uint8_t)id, trial};
      memcpy(m.c, k, sizeof(k));
      if (id >= 1 && id <= UNAGI_N_NODES && node_known[id])
        esp_now_send(node_mac[id], (uint8_t*)&m, sizeof(m));
      tx(String("OK,cal,") + id);
    } else tx("ERR,cal");
    return true;
  }
  if (c.startsWith("arm")) {
    trial++;
    Hdr h = {MSG_ARM, 0, trial};
    esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
    tx(String("OK,arm,") + trial);
    return true;
  }
  if (c.startsWith("start")) {
    if (alertLatched) { tx("ERR,start,overcurrent_latched"); return true; }
    int dly = c.substring(6).toInt(); if (dly <= 0) dly = 200;
    MsgStart m = {}; m.h = {MSG_START, 0, trial};
    m.t_start_head_us = esp_timer_get_time() + (int64_t)dly * 1000;
    for (int k = 0; k < 3; k++) {
      esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
      delay(5);
    }
    tx(String("OK,start,") + (long)m.t_start_head_us);
    return true;
  }
  if (c.startsWith("halt")) { bcast_stop(1); tx("OK,halt"); return true; }
  if (c.startsWith("dump")) {
    int id = 0, blk = 0;
    sscanf(c.c_str(), "dump %d %d", &id, &blk);
    MsgDumpReq m = {}; m.h = {MSG_DUMP_REQ, (uint8_t)id, trial}; m.block = (uint16_t)blk;
    if (id >= 1 && id <= UNAGI_N_NODES && node_known[id])
      esp_now_send(node_mac[id], (uint8_t*)&m, sizeof(m));
    return true;
  }
  if (c.startsWith("mac")) {
    for (int i = 1; i <= UNAGI_N_NODES; i++)
      if (node_known[i]) { char b[20]; mac_str(node_mac[i], b); tx(String("MAC,") + i + "," + b); }
    tx("OK,mac");
    return true;
  }
  return false;
}

// ---------------------------- commands -------------------------------
//  小文字のゲートウェイコマンドを先に見て、残りを従来の大文字コマンドに回す。

static void handleCommand(String raw) {
  raw.trim();
  if (!raw.length()) return;

  if (handleGateway(raw)) return;       // 小文字、そのまま比較

  String cmd = raw;
  cmd.toUpperCase();

  if (cmd == "ON") {
    if (!sensorOk) { emit("{\"err\":\"sensor not ok, refusing to close the switch\"}"); return; }
    setLoad(true);  emit("{\"ack\":\"ON\"}");
  }
  else if (cmd == "OFF")    { bcast_stop(2); setLoad(false); emit("{\"ack\":\"OFF\"}"); }
  else if (cmd == "START")  { streaming = true;  emit("{\"ack\":\"START\"}"); }
  else if (cmd == "STOP")   { streaming = false; emit("{\"ack\":\"STOP\"}"); }
  else if (cmd == "INFO")   { sendInfo(); }
  else if (cmd == "CLEAR")  { clearAlert(); emit("{\"ack\":\"CLEAR\"}"); }
  else if (cmd == "ZERO")   { charge_mAh = 0; energy_Wh = 0; emit("{\"ack\":\"ZERO\"}"); }
  else if (cmd == "SELFTEST") { selfTest(); }
  else if (cmd == "TARE")   { tare(); sendInfo(); }
  else if (cmd == "SAVE")   { calSave(); emit("{\"ack\":\"SAVE\"}"); }
  else if (cmd.startsWith("GAIN ")) {
    float g = cmd.substring(5).toFloat();
    if (g > 0.5f && g < 2.0f) { calGain = g; emit("{\"ack\":\"GAIN\"}"); }
    else emit("{\"err\":\"gain 0.5..2.0\"}");
  }
  else if (cmd.startsWith("OFFSET ")) {
    calOffset = cmd.substring(7).toFloat();
    emit("{\"ack\":\"OFFSET\"}");
  }
  else if (cmd.startsWith("RATE ")) {
    long hz = cmd.substring(5).toInt();
    if (hz >= 1 && hz <= 200) { streamHz = hz; streamGapMs = 1000 / hz; emit("{\"ack\":\"RATE\"}"); }
    else emit("{\"err\":\"rate 1..200\"}");
  }
  else if (cmd.startsWith("AVG ")) {
    long n = cmd.substring(4).toInt();
    int idx = -1;
    for (int i = 0; i < 8; i++) if (AVG_CODE[i] == n) idx = i;
    if (idx >= 0) {
      cfgAvgIndex = idx;
      ina_write(REG_CONFIG, buildConfig(cfgAvgIndex));
      emit("{\"ack\":\"AVG\"}");
    } else emit("{\"err\":\"avg 1|4|16|64|128|256|512|1024\"}");
  }
  else if (cmd.startsWith("TRIP ")) {
    float a = cmd.substring(5).toFloat();
    float fs = 32767.0f * I_LSB_AMPS;
    if (a > 0 && a <= fs) { setTrip(a); emit("{\"ack\":\"TRIP\"}"); }
    else {
      char b[80]; snprintf(b, sizeof(b), "{\"err\":\"trip 0..%.1f A\"}", fs);
      emit(String(b));
    }
  }
  else emit(String("{\"err\":\"unknown command\",\"got\":\"") + raw + "\"}");
}

// ------------------------------ setup --------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_GATE, OUTPUT);
  setLoad(false);                       // always start with the load off
  pinMode(PIN_ALERT, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  ina_write(REG_CONFIG, 0x8000);        // soft reset
  delay(5);
  ina_write(REG_CONFIG, buildConfig(cfgAvgIndex));

  uint16_t cal = (uint16_t)lround(0.00512f / (I_LSB_AMPS * R_SHUNT_OHMS));
  ina_write(REG_CALIB, cal);

  uint16_t manuf = 0, die = 0;
  sensorOk = ina_read(REG_MANUF, manuf) && ina_read(REG_DIEID, die)
             && manuf == MANUF_TI && die == DIE_INA226;
  if (!sensorOk) {
    Serial.printf("INA226 NOT FOUND - manuf 0x%04X die 0x%04X (expect 0x5449 / 0x2260)\n",
                  manuf, die);
    Serial.println("Check I2C wiring, U1 solder joints and the 3V3 rail.");
  }

  calLoad();
  setTrip(tripAmps);
  clearAlert();
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT), onAlert, FALLING);

  // --- WiFi: SoftAP のチャンネルを明示する。ESP-NOW は同一チャンネル必須 ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(NET_SSID, NET_PASS, UNAGI_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("AP \"%s\" up at %s:%d  ch=%d\n",
                NET_SSID, WiFi.softAPIP().toString().c_str(), TCP_PORT, UNAGI_CHANNEL);

  // --- ESP-NOW ---
  if (esp_now_init() != ESP_OK) Serial.println("esp_now_init FAILED");
  esp_now_register_recv_cb(on_recv);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, BCAST, 6);
  p.channel = UNAGI_CHANNEL; p.encrypt = false;
  esp_now_add_peer(&p);
  xTaskCreate(beacon_task, "beacon", 4096, nullptr, 5, nullptr);

  server.begin();
  server.setNoDelay(true);
  streamGapMs = 1000 / streamHz;
  sendInfo();
}

// ------------------------------- loop --------------------------------

void loop() {
  if (!client || !client.connected()) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    WiFiClient incoming = server.accept();
#else
    WiFiClient incoming = server.available();
#endif
    if (incoming) {
      if (client) client.stop();
      client = incoming;
      client.setNoDelay(true);
      sendInfo();
    } else if (client) {
      client.stop();
#if DROP_LINK_CUTS_LOAD
      if (loadOn) {
        bcast_stop(3);
        setLoad(false);
        Serial.println("{\"evt\":\"link lost, load off\"}");
      }
#endif
    }
  }

  if (client && client.connected() && client.available())
    handleCommand(client.readStringUntil('\n'));
  if (Serial.available())
    handleCommand(Serial.readStringUntil('\n'));

  // 過電流。D3 (BAT54) は v31 に未実装なので、このパスだけがスイッチを開く。
  // 同時に全ノードへ STOP をブロードキャストする。
  if (alertFlag) {
    alertFlag    = false;
    alertLatched = true;
    bcast_stop(1);
    setLoad(false);
    emit("{\"evt\":\"OVERCURRENT\"}");
  }

  uint32_t now = millis();
  if (now - lastSample >= 5) { lastSample = now; sample(); }
  if (streaming && now - lastStream >= streamGapMs) {
    lastStream = now;
    sendTelemetry();
  }
}
