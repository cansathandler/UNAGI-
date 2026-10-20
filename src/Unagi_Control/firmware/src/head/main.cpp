#include <Arduino.h>
// =====================================================================
//  UNAGI head firmware  (Seeed XIAO ESP32-C3)
//
//  Laptop --(WiFi UDP)--> HEAD --(RS-485 broadcast, 1 Mbaud)--> 10 nodes
//
//  The head is a dumb, safe relay:
//    * receives one duty vector per camera frame from the laptop
//    * re-broadcasts it on RS-485 (each node picks its own slot)
//    * replies with INA226 power telemetry
//    * watchdog: no command for CMD_TIMEOUT_MS -> broadcast STOP
//    * overcurrent: cut motor power (AON6407) and broadcast STOP
//
//  PlatformIO env "head" (board seeed_xiao_esp32c3). Builds on Arduino-ESP32 core 2.x and 3.x.
//  Pins match the head schematic (XIAO U2, SP3485 U3, INA226 U1, AON6407 Q1/Q2).
// =====================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ---------------- pins (from head schematic, U2 = XIAO ESP32-C3) ----------------
//   D1 GATE-CTL -> R11 -> 2N7002 -> PGATE -> AON6407 x2 (BATT+ -> VBUS). HIGH = motor power ON
//   D2 ALERT    <- INA226 ALERT (open drain, R6 4.7k pull-up)
//   D3 DE       -> SP3485 DE + /RE (R15 10k pull-down)
//   D4/D5 SDA/SCL (INA226, A0=A1=GND -> 0x40), D6 TX -> DI, D7 RX <- RO
//   INA226 VBUS and shunt are AFTER the switch: V reads 0 while disarmed.
static const int  PIN_MOTOR_EN = D1;
static const bool MOTOR_EN_ACTIVE_HIGH = true;
static const int  PIN_INA_ALERT = D2;
static const int  PIN_RS485_DE = D3;
static const int  PIN_SDA = D4;
static const int  PIN_SCL = D5;
static const int  PIN_RS485_TX = D6;
static const int  PIN_RS485_RX = D7;    // not used: head only talks

// ---------------- network ----------------
static const char*    AP_SSID  = "UNAGI-HEAD";
static const char*    AP_PASS  = "unagi1234";   // >= 8 chars
static const uint16_t UDP_PORT = 4210;          // laptop sends to 192.168.4.1:4210

// ---------------- bus / safety ----------------
static const uint32_t RS485_BAUD          = 1000000;
static const uint32_t CMD_TIMEOUT_MS      = 200;   // laptop silent this long -> STOP
static const uint32_t IDLE_STOP_PERIOD_MS = 50;    // keep-quiet STOP broadcast when idle
static const float    I_LIMIT_A           = 15.0f; // total motor current trip
static const int      MAX_NODES           = 16;
static const int16_t  DUTY_MAX            = 1000;  // duty unit: per-mille (-1000..1000)

// ---------------- INA226 (2 mOhm shunt, 1 mA LSB) ----------------
static const uint8_t  INA_ADDR   = 0x40;
static const uint16_t INA_CONFIG = 0x42DF;  // AVG=4, VBUS/VSH conv 588us, continuous
static const uint16_t INA_CAL    = 2560;    // 0.00512 / (1mA * 2mOhm)
static const float    INA_I_LSB  = 0.001f;  // A/bit
static const float    INA_V_LSB  = 0.00125f;// V/bit
static const float    INA_P_LSB  = 0.025f;  // W/bit (25 * I_LSB)

// ---------------- protocol ----------------
// RS-485 frame: A5 5A | type | seq | len | payload[len] | crc16 lo hi
//   crc16 = CCITT-FALSE over [type .. end of payload]
//   CMD payload = int16 duty[n] little-endian, slot i -> node id i+1
enum : uint8_t { BUS_CMD = 0x01, BUS_STOP = 0x02 };

// UDP laptop->head: 'U''N' | type u8 | n u8 | seq u16 | t_ms u32 | int16 duty[n]
enum : uint8_t { UDP_CMD = 1, UDP_STOP = 2, UDP_ARM = 3, UDP_DISARM = 4, UDP_PING = 5 };
static const int UDP_HDR = 10;
// UDP head->laptop: 'U''T' | seq_echo u16 | head_ms u32 | V f32 | I f32 | P f32 | E_J f32 | flags u8  (25 B)
enum : uint8_t { F_ARMED = 1, F_STREAMING = 2, F_TIMEOUT = 4, F_OVERCURRENT = 8, F_INA_FAIL = 16 };

// ---------------- state ----------------
WiFiUDP   udp;
IPAddress peerIp;
uint16_t  peerPort = 0;

bool     armed = false, streaming = false, inaOk = false;
uint8_t  flags = 0;
uint8_t  busSeq = 0;
uint32_t lastCmdMs = 0, lastStopMs = 0, lastInaMs = 0, lastPrintMs = 0;
uint32_t cmdCount = 0, badCount = 0;
float    vbus = 0, ibus = 0, pbus = 0;
double   energyJ = 0;

// ---------------------------------------------------------------------
uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

void busSend(uint8_t type, const int16_t* duty, uint8_t n) {
  uint8_t f[5 + 2 * MAX_NODES + 2];
  size_t k = 0;
  f[k++] = 0xA5; f[k++] = 0x5A; f[k++] = type; f[k++] = busSeq++; f[k++] = (uint8_t)(2 * n);
  for (uint8_t i = 0; i < n; i++) {
    uint16_t v = (uint16_t)duty[i];
    f[k++] = v & 0xFF; f[k++] = v >> 8;
  }
  uint16_t c = crc16(f + 2, k - 2);
  f[k++] = c & 0xFF; f[k++] = c >> 8;
  Serial1.write(f, k);
}

void busStop() { busSend(BUS_STOP, nullptr, 0); lastStopMs = millis(); }

void setMotorPower(bool on) {
  digitalWrite(PIN_MOTOR_EN, (on == MOTOR_EN_ACTIVE_HIGH) ? HIGH : LOW);
}

// ---------------- INA226 ----------------
bool inaWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
bool inaRead(uint8_t reg, uint16_t& v) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA_ADDR, 2) != 2) return false;
  v = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}
void inaInit() {
  inaOk = inaWrite(0x00, 0x8000);        // reset
  delay(2);
  inaOk = inaOk && inaWrite(0x00, INA_CONFIG) && inaWrite(0x05, INA_CAL);
  if (!inaOk) flags |= F_INA_FAIL;
}
void inaPoll(uint32_t now) {
  if (!inaOk || now - lastInaMs < 5) return;
  float dt = (now - lastInaMs) * 1e-3f;
  lastInaMs = now;
  uint16_t rv, ri, rp;
  if (!inaRead(0x02, rv) || !inaRead(0x04, ri) || !inaRead(0x03, rp)) { flags |= F_INA_FAIL; return; }
  vbus = rv * INA_V_LSB;
  ibus = (int16_t)ri * INA_I_LSB;
  pbus = rp * INA_P_LSB;
  energyJ += pbus * dt;
  if (armed && fabsf(ibus) > I_LIMIT_A) {
    busStop();
    setMotorPower(false);
    armed = false; streaming = false;
    flags |= F_OVERCURRENT;
    Serial.printf("!! OVERCURRENT %.2f A -> motor power OFF\n", ibus);
  }
}

// ---------------- UDP ----------------
void sendTelemetry(uint16_t seqEcho) {
  if (peerPort == 0) return;
  uint8_t b[25];
  uint32_t ms = millis();
  float e = (float)energyJ;
  uint8_t fl = flags | (armed ? F_ARMED : 0) | (streaming ? F_STREAMING : 0);
  b[0] = 'U'; b[1] = 'T';
  memcpy(b + 2, &seqEcho, 2);
  memcpy(b + 4, &ms, 4);
  memcpy(b + 8, &vbus, 4);
  memcpy(b + 12, &ibus, 4);
  memcpy(b + 16, &pbus, 4);
  memcpy(b + 20, &e, 4);
  b[24] = fl;
  udp.beginPacket(peerIp, peerPort);
  udp.write(b, sizeof(b));
  udp.endPacket();
}

void handleUdp() {
  int len;
  while ((len = udp.parsePacket()) > 0) {
    uint8_t buf[UDP_HDR + 2 * MAX_NODES];
    int n = udp.read(buf, sizeof(buf));
    if (n < UDP_HDR || buf[0] != 'U' || buf[1] != 'N') { badCount++; continue; }
    peerIp = udp.remoteIP(); peerPort = udp.remotePort();

    uint8_t  type   = buf[2];
    uint8_t  nNodes = buf[3];
    uint16_t seq;  memcpy(&seq, buf + 4, 2);
    uint32_t now = millis();

    switch (type) {
      case UDP_CMD: {
        if (nNodes > MAX_NODES || n != UDP_HDR + 2 * nNodes) { badCount++; break; }
        if (!armed) break;                  // ignore drive commands while disarmed
        int16_t duty[MAX_NODES];
        for (uint8_t i = 0; i < nNodes; i++) {
          int16_t d; memcpy(&d, buf + UDP_HDR + 2 * i, 2);
          duty[i] = constrain(d, -DUTY_MAX, DUTY_MAX);
        }
        busSend(BUS_CMD, duty, nNodes);
        lastCmdMs = now; streaming = true; cmdCount++;
        flags &= ~F_TIMEOUT;
        break;
      }
      case UDP_STOP:
        busStop(); streaming = false;
        break;
      case UDP_ARM:
        flags &= ~F_OVERCURRENT;            // deliberate re-arm clears the trip
        busStop(); setMotorPower(true); armed = true; streaming = false;
        Serial.printf("ARMED (from laptop %s)\n", peerIp.toString().c_str());
        break;
      case UDP_DISARM:
        busStop(); setMotorPower(false); armed = false; streaming = false;
        Serial.println("DISARMED");
        break;
      case UDP_PING:
      default:
        break;
    }
    sendTelemetry(seq);
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  setMotorPower(false);
  pinMode(PIN_INA_ALERT, INPUT);          // external pull-up; never drive this pin
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, HIGH);        // head is the only talker on the bus
  Serial1.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  inaInit();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);                    // lower, steadier latency
  udp.begin(UDP_PORT);

  delay(200);
  Serial.printf("UNAGI head ready  AP=%s  IP=%s  UDP=%u  INA226=%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT, inaOk ? "ok" : "FAIL");
}

// ---------------- USB serial console (bench debugging without the laptop) ----------------
//   arm / disarm   same as the laptop's ARM / DISARM
//   raw 1 / raw 0  drive PIN_MOTOR_EN pin HIGH / LOW directly (find the polarity of the switch)
//   quiet          toggle the 2 Hz status line
//   ?              status + pin info
bool quiet = false;
void handleConsole() {
  static char line[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof(line) - 1) line[n++] = c; continue; }
    line[n] = 0; n = 0;
    if (!line[0]) continue;
    if (!strcmp(line, "arm")) {
      flags &= ~F_OVERCURRENT; busStop(); setMotorPower(true); armed = true; streaming = false;
      Serial.println("ARMED (console)");
    } else if (!strcmp(line, "disarm")) {
      busStop(); setMotorPower(false); armed = false; streaming = false;
      Serial.println("DISARMED (console)");
    } else if (!strncmp(line, "raw", 3)) {
      int v = atoi(line + 3 + (line[3] == ' ' || line[3] == '='));
      digitalWrite(PIN_MOTOR_EN, v ? HIGH : LOW);
      Serial.printf("PIN_MOTOR_EN (GPIO%d) forced %s -> measure motor rail / check node LEDs\n",
                    PIN_MOTOR_EN, v ? "HIGH" : "LOW");
    } else if (!strcmp(line, "quiet")) {
      quiet = !quiet;
    } else if (line[0] == '?') {
      Serial.printf("PIN_MOTOR_EN=GPIO%d level=%d  ACTIVE_HIGH=%d  armed=%d  WiFi clients=%d\n",
                    PIN_MOTOR_EN, digitalRead(PIN_MOTOR_EN), MOTOR_EN_ACTIVE_HIGH, armed,
                    WiFi.softAPgetStationNum());
    } else {
      Serial.println("commands: arm, disarm, raw 1, raw 0, quiet, ?");
    }
  }
}

void loop() {
  uint32_t now = millis();
  handleUdp();
  handleConsole();

  if (streaming && now - lastCmdMs > CMD_TIMEOUT_MS) {
    streaming = false;
    flags |= F_TIMEOUT;
    Serial.println("command timeout -> STOP");
  }
  if (!streaming && now - lastStopMs >= IDLE_STOP_PERIOD_MS) busStop();

  inaPoll(now);

  if (!quiet && now - lastPrintMs >= 500) {
    lastPrintMs = now;
    Serial.printf("armed=%d stream=%d cmds=%lu bad=%lu  V=%.2f I=%.3f P=%.2f E=%.1fJ flags=0x%02X\n",
                  armed, streaming, (unsigned long)cmdCount, (unsigned long)badCount,
                  vbus, ibus, pbus, energyJ, flags);
  }
}
