#include <Arduino.h>
// =====================================================================
//  UNAGI motor node firmware  (Seeed XIAO ESP32-C3 + SP3485 + DRV8833)
//
//  Listen-only RS-485 node. Every frame from the head carries the duty
//  for ALL nodes; this node takes slot (id-1) and drives its coil.
//  No sensors, no replies, no local control loop.
//
//  Safety: no valid frame for RX_TIMEOUT_MS -> coil off (coast).
//
//  Node ID (1..16) is stored in flash, so all 10 nodes run the SAME
//  binary. Set it once over USB serial (115200):
//      id 3        -> store node id 3
//      t 500       -> local test: duty +500 for 0.5 s (bus must be idle)
//      ?           -> status
//
//  PlatformIO env "node" (board seeed_xiao_esp32c3). Builds on Arduino-ESP32 core 2.x and 3.x.
//  Pins match the node schematic (XIAO U1, SP3485 U2, DRV8833 carrier U$1).
// =====================================================================
#include <Preferences.h>
#include <esp_system.h>

// ---------------- pins (from node schematic, U1 = XIAO ESP32-C3) ----------------
//   D0/D1 S1/S2 (old Hall inputs, unused now)
//   D2 PWM1 -> DRV8833 INA1 + INB1 (bridges paralleled), D3 PWM2 -> INA2 + INB2
//   D4 DE -> SP3485 DE + /RE (R3 10k pull-down), D5 <- DRV8833 nFAULT
//   D6 TX -> DI, D7 RX <- RO.  nSLEEP not wired to the MCU.
static const int PIN_IN1      = D2;
static const int PIN_IN2      = D3;
static const int PIN_RS485_DE = D4;   // LOW = receive
static const int PIN_FAULT    = D5;   // DRV8833 nFAULT, low = fault
static const int PIN_RS485_TX = D6;   // unused, node never talks
static const int PIN_RS485_RX = D7;
static const int PIN_NSLEEP   = -1;

static const uint32_t RS485_BAUD     = 1000000;
static const uint32_t RX_TIMEOUT_MS  = 100;
static const uint32_t PWM_FREQ_HZ    = 20000;
static const uint8_t  PWM_BITS       = 10;
static const uint32_t PWM_FULL       = (1u << PWM_BITS);   // 1024 = 100 % on
static const int16_t  DUTY_MAX       = 1000;
static const uint32_t FRAME_GAP_US   = 300;                 // resync if a frame stalls

enum : uint8_t { BUS_CMD = 0x01, BUS_STOP = 0x02 };

Preferences prefs;
uint8_t  nodeId = 0;              // 0 = not configured -> never drives
int16_t  dutyNow = 0;
uint32_t lastRxMs = 0, testUntilMs = 0;
// local test waveform (bus ignored while active): 't DUTY' = 0.5 s step, 's AMP FREQ SEC' = sine
float    sineAmp = 0, sineFreq = 0;
uint32_t sineT0 = 0, lastSineUpdMs = 0;
// command-path diagnostics
int16_t  lastCmd = 0;
uint32_t cmdZeroDrops = 0, cmdJumps = 0;
uint32_t okFrames = 0, crcErrors = 0, lenErrors = 0;
uint32_t faultEvents = 0, rxTimeouts = 0;   // DRV8833 nFAULT pulses, bus-silence cut-offs
bool     faultLow = false;

const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply dipped)";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC (crash)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_USB:      return "usb";
    default:               return "other";
  }
}

// ---------------------------------------------------------------------
uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// ---------------- PWM: works on Arduino-ESP32 core 2.x and 3.x ----------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmInit(int pin, uint8_t)        { ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS); }
static void pwmWrite(int pin, uint8_t, uint32_t d) { ledcWrite(pin, d); }
#else
static void pwmInit(int pin, uint8_t ch)     { ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
static void pwmWrite(int, uint8_t ch, uint32_t d)  { ledcWrite(ch, d); }
#endif
static const uint8_t CH_IN1 = 0, CH_IN2 = 1;
#define OUT1(d) pwmWrite(PIN_IN1, CH_IN1, (d))
#define OUT2(d) pwmWrite(PIN_IN2, CH_IN2, (d))

// Sign-magnitude, slow decay (drive / brake). Smoother coil current than fast decay.
void applyDuty(int16_t d) {
  d = constrain(d, -DUTY_MAX, DUTY_MAX);
  dutyNow = d;
  if (d == 0) {                         // coast: both low -> coil current decays to 0
    OUT1(0);
    OUT2(0);
    return;
  }
  uint32_t mag = (uint32_t)abs(d) * PWM_FULL / DUTY_MAX;
  if (d > 0) { OUT1(PWM_FULL);        OUT2(PWM_FULL - mag); }
  else       { OUT1(PWM_FULL - mag);  OUT2(PWM_FULL);       }
}

// ---------------- RS-485 frame parser ----------------
// A5 5A | type | seq | len | payload[len] | crc lo | crc hi
enum ParseState { P_SYNC1, P_SYNC2, P_TYPE, P_SEQ, P_LEN, P_PAYLOAD, P_CRC1, P_CRC2 };
ParseState ps = P_SYNC1;
uint8_t  fb[3 + 64];   // type, seq, len, payload
uint8_t  fpos = 0, flen = 0, crcLo = 0;
uint32_t lastByteUs = 0;

void onFrame(uint8_t type, const uint8_t* pay, uint8_t len) {
  lastRxMs = millis();
  if (testUntilMs) return;                                    // local test owns the coil
  if (type == BUS_STOP) { applyDuty(0); lastCmd = 0; return; }
  if (type != BUS_CMD || nodeId == 0) return;
  uint16_t off = 2 * (nodeId - 1);
  if (off + 1 >= len) { lenErrors++; applyDuty(0); return; }   // frame has no slot for us
  int16_t d = (int16_t)((uint16_t)pay[off] | ((uint16_t)pay[off + 1] << 8));
  if (d == 0 && abs(lastCmd) > 150) cmdZeroDrops++;           // sudden drop to 0 in the command
  if (abs(d - lastCmd) > 300) cmdJumps++;                     // sudden jump in the command
  lastCmd = d;
  applyDuty(d);
}

void parseByte(uint8_t b) {
  uint32_t nowUs = micros();
  if (ps != P_SYNC1 && nowUs - lastByteUs > FRAME_GAP_US) ps = P_SYNC1;   // stale partial frame
  lastByteUs = nowUs;

  switch (ps) {
    case P_SYNC1: if (b == 0xA5) ps = P_SYNC2; break;
    case P_SYNC2: ps = (b == 0x5A) ? P_TYPE : (b == 0xA5 ? P_SYNC2 : P_SYNC1); break;
    case P_TYPE:  fb[0] = b; ps = P_SEQ; break;
    case P_SEQ:   fb[1] = b; ps = P_LEN; break;
    case P_LEN:
      if (b > sizeof(fb) - 3) { lenErrors++; ps = P_SYNC1; break; }
      fb[2] = b; flen = b; fpos = 0;
      ps = (flen == 0) ? P_CRC1 : P_PAYLOAD;
      break;
    case P_PAYLOAD:
      fb[3 + fpos++] = b;
      if (fpos >= flen) ps = P_CRC1;
      break;
    case P_CRC1: crcLo = b; ps = P_CRC2; break;
    case P_CRC2: {
      uint16_t rx = (uint16_t)crcLo | ((uint16_t)b << 8);
      if (rx == crc16(fb, 3 + flen)) { okFrames++; onFrame(fb[0], fb + 3, flen); }
      else crcErrors++;
      ps = P_SYNC1;
      break;
    }
  }
}

// ---------------- USB serial console ----------------
void handleConsole() {
  static char line[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof(line) - 1) line[n++] = c; continue; }
    line[n] = 0; n = 0;
    if (line[0] == 0) continue;

    if (strncmp(line, "id", 2) == 0) {            // accepts "id 3", "id=3", "id3"
      const char* p = line + 2;
      while (*p == ' ' || *p == '=') p++;
      int v = atoi(p);
      if (v >= 1 && v <= 16) {
        nodeId = v; prefs.putUChar("id", nodeId);
        Serial.printf("node id set to %u (stored)\n", nodeId);
      } else Serial.println("id must be 1..16");
    } else if (line[0] == 't' && line[1] == ' ') {
      int v = atoi(line + 2);
      sineAmp = 0;
      testUntilMs = millis() + 500;
      applyDuty((int16_t)v);
      Serial.printf("test duty %d for 0.5 s\n", dutyNow);
    } else if (line[0] == 's' && line[1] == ' ') {
      float a = 0, f = 1, sec = 5;
      sscanf(line + 2, "%f %f %f", &a, &f, &sec);
      sineAmp = constrain(a, 0.0f, (float)DUTY_MAX); sineFreq = f;
      sineT0 = millis(); testUntilMs = sineT0 + (uint32_t)(sec * 1000);
      Serial.printf("local sine amp %.0f, %.2f Hz for %.1f s (bus ignored)\n", sineAmp, sineFreq, sec);
    } else if (!strcmp(line, "r")) {
      okFrames = crcErrors = lenErrors = faultEvents = rxTimeouts = cmdZeroDrops = cmdJumps = 0;
      Serial.println("counters reset");
    } else if (line[0] == '?') {
      Serial.printf("id=%u duty=%d fault=%s faultEvents=%lu rxTimeouts=%lu ok=%lu crcErr=%lu lenErr=%lu "
                    "lastRx=%lums ago  up=%lus  lastReset=%s\n",
                    nodeId, dutyNow, digitalRead(PIN_FAULT) ? "no" : "YES", (unsigned long)faultEvents,
                    (unsigned long)rxTimeouts, (unsigned long)okFrames, (unsigned long)crcErrors,
                    (unsigned long)lenErrors, (unsigned long)(millis() - lastRxMs),
                    (unsigned long)(millis() / 1000), resetReasonStr());
      Serial.printf("   cmdZeroDrops=%lu cmdJumps=%lu\n", (unsigned long)cmdZeroDrops, (unsigned long)cmdJumps);
    } else {
      Serial.println("commands: 'id N', 't DUTY', 's AMP FREQ SEC', 'r' (reset counters), '?'");
    }
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pwmInit(PIN_IN1, CH_IN1);
  pwmInit(PIN_IN2, CH_IN2);
  applyDuty(0);
  pinMode(PIN_FAULT, INPUT_PULLUP);
  if (PIN_NSLEEP >= 0) { pinMode(PIN_NSLEEP, OUTPUT); digitalWrite(PIN_NSLEEP, HIGH); }

  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW);          // receive only, forever
  Serial1.setRxBufferSize(512);
  Serial1.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  prefs.begin("unagi", false);
  nodeId = prefs.getUChar("id", 0);

  delay(200);
  if (nodeId == 0) Serial.println("UNAGI node: ID NOT SET -> type 'id N' (1..16). Coil disabled.");
  else             Serial.printf("UNAGI node %u ready\n", nodeId);
  Serial.printf("reset reason: %s\n", resetReasonStr());
  Serial.println("commands: 'id N' (or id=N), 't DUTY', 's AMP FREQ SEC', 'r', '?'   then press Enter");
}

void loop() {
  while (Serial1.available()) parseByte((uint8_t)Serial1.read());
  handleConsole();

  uint32_t now = millis();
  if (testUntilMs && now > testUntilMs) { testUntilMs = 0; sineAmp = 0; applyDuty(0); lastCmd = 0; }
  if (testUntilMs && sineAmp > 0 && now - lastSineUpdMs >= 2) {      // local sine, 500 Hz update
    lastSineUpdMs = now;
    applyDuty((int16_t)(sineAmp * sinf(2.0f * PI * sineFreq * (now - sineT0) * 1e-3f)));
  }
  if (!testUntilMs && dutyNow != 0 && now - lastRxMs > RX_TIMEOUT_MS) {   // bus lost
    applyDuty(0);
    rxTimeouts++;
  }
  bool fl = !digitalRead(PIN_FAULT);           // count each nFAULT pulse (OCP / UVLO / thermal)
  if (fl && !faultLow) faultEvents++;
  faultLow = fl;
}
