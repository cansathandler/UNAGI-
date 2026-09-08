// unagi_protocol.h — ヘッド / ノード共有の ESP-NOW メッセージ定義
#pragma once
#include <stdint.h>

#define UNAGI_N_NODES     10
#define UNAGI_CHANNEL      6      // RF-0 のチャンネルサーベイ結果に合わせて変更
#define UNAGI_MAX_SAMPLES 20000   // RAM 実測後に調整（8B/sample）

enum : uint8_t {
  MSG_BEACON      = 0x01,
  MSG_PARAM       = 0x02,
  MSG_ARM         = 0x03,
  MSG_READY       = 0x04,
  MSG_START       = 0x05,
  MSG_STOP        = 0x06,
  MSG_REPORT      = 0x07,
  MSG_DUMP_REQ    = 0x08,
  MSG_DUMP_DATA   = 0x09,
  MSG_SYNC_BEGIN  = 0x0A,
  MSG_HELLO       = 0x0B,   // 起動時に MAC を通知
};

// 走行モード
enum : uint8_t {
  MODE_SWIM      = 0,  // 通常の CPG 追従
  MODE_GPIO_SYNC = 1,  // SYNC-1: t_start で GPIO を1回トグルするだけ
  MODE_HALL_RAW  = 2,  // HALL-1: モータ停止、v1/v2 生値を記録（較正用）
  MODE_PWM_SWEEP = 3,  // HALL-2: 関節を機械的に固定し PWM を掃引（クロストーク）
  MODE_STEP      = 4,  // JOINT-1: 目標角ステップ応答
};

#pragma pack(push, 1)

struct Hdr {
  uint8_t  type;
  uint8_t  node_id;      // 1..10、ブロードキャストでは 0
  uint16_t trial_id;
};

struct MsgBeacon {
  Hdr      h;
  uint32_t seq;
  int64_t  t_head_us;
};

struct MsgSyncBegin {
  Hdr      h;
  uint16_t n_points;     // 集める点数（例 300）
};

struct MsgParam {
  Hdr     h;
  float   A_deg;         // 振幅
  float   f_hz;          // 周波数
  float   phi_deg;       // 位相オフセット
  float   off_deg;       // 直流オフセット
  float   kp, ki, kd;
  uint16_t duration_ms;
  uint16_t log_hz;       // 200 / 500 / 1000
  uint8_t  mode;         // MODE_*
  float    aux;          // モード依存（STEP の振幅 / PWM_SWEEP の最大デューティ）
};

struct MsgStart {
  Hdr     h;
  int64_t t_start_head_us;
};

struct MsgReady {
  Hdr      h;
  uint32_t fw_hash;
  float    sigma_us;     // 同期回帰の残差 SD
  double   b;            // クロック比
  double   a;            // オフセット [us]
  uint16_t param_crc;
  uint32_t free_ram;
  int16_t  hall1_bias;   // 無励磁・中立姿勢での ADC 生値
  int16_t  hall2_bias;
  uint8_t  ready;        // 1 = 走行可
};

struct MsgReport {
  Hdr      h;
  uint32_t n_samples;
  int64_t  t_start_local_us;
  uint32_t missed;        // 締切を落とした回数
  uint32_t max_jitter_us;
  float    rms_err_deg;
  float    adc_us_max;    // ADC 読み出しの最悪所要時間
};

struct MsgDumpReq {
  Hdr      h;
  uint16_t block;         // 0 から順に
};

// 1ブロック = 28 サンプル × 8 バイト = 224 B（ESP-NOW 上限 250 B 内）
#define DUMP_SAMPLES_PER_BLOCK 28
struct MsgDumpData {
  Hdr      h;
  uint16_t block;
  uint16_t n;             // このブロックの有効サンプル数
  int16_t  d[DUMP_SAMPLES_PER_BLOCK][4];
};

struct MsgStop { Hdr h; uint8_t reason; };
struct MsgHello { Hdr h; uint8_t mac[6]; };

#pragma pack(pop)

static inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}
