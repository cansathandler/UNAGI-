// unagi_protocol.h — ヘッド / ノード共有のメッセージ定義 (protocol v3)
//
// v3 (RS-485 版) での変更
//  ・伝送路を ESP-NOW から RS-485 半二重バスに変更。メッセージ本体はそのまま。
//    フレーム化（0x7E, len, payload, CRC16）は unagi_bus.h が担当する。
//  ・ノードは「自分宛て」の要求にだけ返事をする（ヘッドが1台ずつ指名する）。
//  ・M_REPORT_REQ を追加。REPORT は走行後にヘッドが取りに行く（自動送信を廃止）。
//  ・MsgHello の末尾に MAC（eFuse 固有値）を追加。同じ ID のノードが2台あると分かる。
//  ・ABORT_FAULT / WHY_FAULT を追加（DRV8833 nFAULT）。
//
// v2 の変更
// v1 からの主な変更
//  ・先頭バイトに magic を入れ、ヘッド発 (0xA5) とノード発 (0x5A) を区別する。
//    v1 ではノード同士の HELLO ブロードキャストを「ヘッドからの通信」と誤認し、
//    ノード同士が HELLO を投げ合い続ける不具合があった。
//  ・PARAM は trial に紐付けない。trial は ARM が決め、START はその trial のみ受理。
//  ・ログ1サンプルに時刻とホール生値を含める（較正と検算に使う）。
#pragma once
#include <stdint.h>
#include <stddef.h>

#define UNAGI_PROTO_VER    3
#define UNAGI_N_NODES      10
#define UNAGI_CHANNEL      6        // ヘッド SoftAP（PC 接続用）の WiFi チャンネル。ノードは WiFi 不使用
#define UNAGI_MAX_SAMPLES  4000     // 14 B/sample → 56 kB (RAM)

#define UNAGI_MAGIC_HEAD   0xA5     // ヘッド → ノード
#define UNAGI_MAGIC_NODE   0x5A     // ノード → ヘッド

enum : uint8_t {
  // head -> node
  // 「指名」= node_id で1台を指名し、そのノードだけが返事をする。
  // 「全体」= node_id 0。返事はしない（返事をすると衝突するので）。
  M_DISCOVER  = 0x10,   // 指名   : HELLO を返させる（node_id 0 = ID 未設定のノード）
  M_BEACON    = 0x11,   // 全体   : 時刻同期用 (20 Hz 常時)
  M_PARAM     = 0x12,   // 指名   : 走行パラメータ → PARAM_ACK
  M_ARM       = 0x13,   // 指名   : trial を確定し READY を返させる
  M_START     = 0x14,   // 全体   : 絶対時刻 T で開始
  M_STOP      = 0x15,   // 全体   : 非常停止
  M_DUMP_REQ  = 0x16,   // 指名   : ログ1ブロック要求 → DUMP_DATA
  M_CAL_SET   = 0x17,   // 指名   : 較正係数 / minduty / ゼロ点 → CAL_ACK
  M_PEEK_REQ  = 0x18,   // 指名   : ホール値を平均して返す（較正・配線確認）→ PEEK
  M_REPORT_REQ= 0x19,   // 指名   : 走行が終わっていれば REPORT を返す（未完なら無言）
  M_CALX_REQ  = 0x1A,   // 指名   : 校正トンネル。テキストコマンド1行 + データ取り → CALX_DATA

  // node -> head
  M_HELLO     = 0x40,
  M_PARAM_ACK = 0x41,
  M_READY     = 0x42,
  M_REPORT    = 0x43,
  M_DUMP_DATA = 0x44,
  M_CAL_ACK   = 0x45,
  M_PEEK      = 0x46,
  M_CALX_DATA = 0x47,
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
  WHY_OK = 0, WHY_NO_SYNC = 1, WHY_BAD_SYNC = 2, WHY_NO_PARAM = 3, WHY_BUSY = 4, WHY_FAULT = 5,
};

// REPORT.abort
enum : uint8_t {
  ABORT_NONE = 0, ABORT_STOP_MSG = 1, ABORT_ANGLE = 2, ABORT_LOG_FULL = 3, ABORT_FAULT = 4,
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
  uint8_t  mac[6];         // v3: eFuse の MAC（個体識別用）
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

// ---- 校正トンネル (CALX) ----
//  ヘッドの校正ブリッジ (TCP 5000) が PC の1行を MsgCalxReq.text に入れて送り、ノードは
//  node_calwifi と同じテキストプロトコルで処理する。返事 (MsgCalxData) は
//  [ヘッダ][サンプル n_s 個][テキスト n_txt バイト（改行区切りの行）]。
//  seq: 新しい seq = 前の返事は届いた。text があれば1回だけ実行し、新しい返事を作る。
//       同じ seq（ヘッドの再送）= 前の返事をそのまま再送（コマンドを2回実行しない・データを失わない）。
#define CALX_F_OPEN      0x01     // req: セッション開始（ノードは駆動とストリームを止めて初期化）
#define CALX_F_SYNCED    0x01     // data: t_us はヘッド時刻（ビーコン同期済み）
#define CALX_F_MORE      0x02     // data: まだ送るものがある（すぐ次を取りに来て）
#define CALX_F_STREAM    0x04     // data: HALL 1 で流している
#define CALX_F_RUN       0x08     // data: RUN で駆動中
#define CALX_TEXT_MAX    200
struct MsgCalxReq  { Hdr h; uint8_t seq; uint8_t flags; uint8_t n; char text[CALX_TEXT_MAX]; };
struct CalxSample  { uint32_t t_us; int16_t v1, v2, u_milli; };          // 10 B
struct MsgCalxData { Hdr h; uint8_t seq; uint8_t flags; uint8_t n_s; uint8_t n_txt; uint16_t dropped; };
#define CALX_HDR_LEN   ((int)sizeof(MsgCalxData))                       // 12 B
#define CALX_ROOM      (250 - CALX_HDR_LEN)                             // 238 B（サンプル + テキスト）

#pragma pack(pop)

#define UNAGI_MAX_PAYLOAD 250
static_assert(sizeof(MsgDumpData) <= UNAGI_MAX_PAYLOAD, "bus frame payload is limited to 250 bytes");
static_assert(sizeof(MsgParam)    <= UNAGI_MAX_PAYLOAD, "bus frame payload is limited to 250 bytes");
static_assert(sizeof(MsgCalxReq)  <= UNAGI_MAX_PAYLOAD, "bus frame payload is limited to 250 bytes");
static_assert(sizeof(CalxSample) == 10, "CalxSample must be packed");

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
