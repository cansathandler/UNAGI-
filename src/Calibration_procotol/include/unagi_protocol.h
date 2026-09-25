// unagi_protocol.h — ヘッド / ノード共有の ESP-NOW メッセージ定義 (protocol v2)
//
// v1 からの主な変更
//  ・先頭バイトに magic を入れ、ヘッド発 (0xA5) とノード発 (0x5A) を区別する。
//    v1 ではノード同士の HELLO ブロードキャストを「ヘッドからの通信」と誤認し、
//    ノード同士が HELLO を投げ合い続ける不具合があった。
//  ・PARAM は trial に紐付けない。trial は ARM が決め、START はその trial のみ受理。
//  ・ログ1サンプルに時刻とホール生値を含める（較正と検算に使う）。
#pragma once
#include <stdint.h>
#include <stddef.h>

#define UNAGI_PROTO_VER    2
#define UNAGI_N_NODES      10
#define UNAGI_CHANNEL      6        // ヘッド SoftAP とノードで必ず一致させる
#define UNAGI_MAX_SAMPLES  4000     // 14 B/sample → 56 kB (RAM)

#define UNAGI_MAGIC_HEAD   0xA5     // ヘッド → ノード
#define UNAGI_MAGIC_NODE   0x5A     // ノード → ヘッド

enum : uint8_t {
  // head -> node
  M_DISCOVER  = 0x10,   // bcast  : 全ノードに HELLO を返させる
  M_BEACON    = 0x11,   // bcast  : 時刻同期用 (20 Hz 常時)
  M_PARAM     = 0x12,   // unicast: 走行パラメータ
  M_ARM       = 0x13,   // bcast  : trial を確定し READY を返させる
  M_START     = 0x14,   // bcast  : 絶対時刻 T で開始
  M_STOP      = 0x15,   // bcast  : 非常停止
  M_DUMP_REQ  = 0x16,   // unicast: ログ1ブロック要求
  M_CAL_SET   = 0x17,   // unicast: 較正係数 / minduty / ゼロ点
  M_PEEK_REQ  = 0x18,   // unicast: ホール値を平均して返す（較正・配線確認）

  // node -> head
  M_HELLO     = 0x40,
  M_PARAM_ACK = 0x41,
  M_READY     = 0x42,
  M_REPORT    = 0x43,
  M_DUMP_DATA = 0x44,
  M_CAL_ACK   = 0x45,
  M_PEEK      = 0x46,
};

// 走行モード
enum : uint8_t {
  MODE_SWIM      = 0,  // 閉ループ正弦追従   theta = A sin(2π f t − φ) + off   [deg]
  MODE_SYNC_TEST = 1,  // SYNC-1: T で D10 にパルスを出すだけ
  MODE_HALL_RAW  = 2,  // 駆動しない。ホール生値を記録（較正用）
  MODE_PWM_SWEEP = 3,  // 開ループ三角波 0 → +aux → −aux → 0（関節を固定してクロストーク測定）
  MODE_STEP      = 4,  // 閉ループステップ  0.2 s 後に off → off+aux [deg]
  MODE_OPENLOOP  = 5,  // 開ループ正弦 u = A sin(2π f t − φ) + off   [duty, −1..1]
};

// READY.why
enum : uint8_t {
  WHY_OK = 0, WHY_NO_SYNC = 1, WHY_BAD_SYNC = 2, WHY_NO_PARAM = 3, WHY_BUSY = 4,
};

// REPORT.abort
enum : uint8_t {
  ABORT_NONE = 0, ABORT_STOP_MSG = 1, ABORT_ANGLE = 2, ABORT_LOG_FULL = 3,
};

#pragma pack(push, 1)

struct Hdr {
  uint8_t  magic;
  uint8_t  type;
  uint8_t  node_id;     // 宛先 (head→node) / 送信元 (node→head)。0 = 全体 / 未設定
  uint8_t  rsv;
  uint16_t trial;
};

// ---- head -> node ----
struct MsgBeacon  { Hdr h; uint32_t seq; int64_t t_head_us; };
struct MsgStart   { Hdr h; int64_t t_start_head_us; };
struct MsgStop    { Hdr h; uint8_t reason; };
struct MsgDumpReq { Hdr h; uint16_t block; };
struct MsgPeekReq { Hdr h; uint16_t n_avg; };

struct ParamBody {
  float    A;            // SWIM/STEP: deg,  OPENLOOP: duty
  float    f_hz;
  float    phi_deg;      // 位相遅れ（+ で遅れる）
  float    off;          // SWIM/STEP: deg,  OPENLOOP: duty
  float    kp, ki, kd;   // duty/deg, duty/(deg·s), duty·s/deg
  float    aux;          // STEP: ステップ量 deg / PWM_SWEEP: 最大 duty
  uint32_t duration_ms;
  uint16_t log_hz;       // 1000 の約数 (1000, 500, 200, 100, 50 ...)
  uint8_t  mode;
};
struct MsgParam   { Hdr h; ParamBody p; };

#define CAL_F_SET_COEF  0x01
#define CAL_F_SET_MD    0x02
#define CAL_F_ZERO      0x04     // 今の姿勢を 0 deg にする（C0 を調整）
struct MsgCalSet  { Hdr h; uint8_t flags; float c[4]; float min_duty; };

// ---- node -> head ----
struct MsgHello {
  Hdr      h;
  uint32_t fw;
  uint8_t  proto;
  uint8_t  state;
  float    c[4];
  float    min_duty;
};
struct MsgParamAck { Hdr h; uint16_t crc; uint8_t mode; };
struct MsgReady {
  Hdr      h;
  uint8_t  ready;
  uint8_t  why;
  uint16_t param_crc;
  uint16_t sync_n;
  float    sigma_us;
  float    skew_ppm;
  uint32_t free_heap;
  int16_t  v1, v2;
};
struct MsgReport {
  Hdr      h;
  uint32_t n_samples;
  uint32_t ticks;
  uint32_t missed;
  uint32_t max_jitter_us;
  float    rms_err_deg;
  uint16_t loop_us_max;
  uint8_t  abort_reason;
  int32_t  start_err_us;   // 実際の開始時刻 − 目標時刻（ノードのローカル時計で）
};

struct LogSample {         // 14 B
  uint32_t t_us;           // T からの経過（ヘッド時間換算）
  int16_t  cmd;            // 目標角 deg×100（開ループ系は 0）
  int16_t  meas;           // 計測角 deg×100
  int16_t  u;              // 出力 duty×10000
  int16_t  v1, v2;         // ホール生値 (12 bit)
};
#define DUMP_PER_BLOCK 16
struct MsgDumpData {
  Hdr      h;
  uint16_t block;
  uint16_t n_total;
  uint8_t  n;
  LogSample s[DUMP_PER_BLOCK];
};
struct MsgCalAck { Hdr h; float c[4]; float min_duty; };
struct MsgPeek {
  Hdr      h;
  uint8_t  state;
  uint16_t n;
  float    v1, v2, diff, diff_sd, deg;
  float    sigma_us, skew_ppm;
  uint16_t sync_n;
};

#pragma pack(pop)

static_assert(sizeof(MsgDumpData) <= 250, "ESP-NOW payload is limited to 250 bytes");
static_assert(sizeof(MsgParam)    <= 250, "ESP-NOW payload is limited to 250 bytes");

static inline uint16_t crc16(const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data;
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

static inline void hdr_init(Hdr& h, uint8_t magic, uint8_t type, uint8_t id, uint16_t trial) {
  h.magic = magic; h.type = type; h.node_id = id; h.rsv = 0; h.trial = trial;
}
