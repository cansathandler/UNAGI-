// unagi_protocol.h — ヘッド / ノード共有の ESP-NOW メッセージ定義
#pragma once
#include <stdint.h>
#include <stddef.h>

#define UNAGI_N_NODES     10
#define UNAGI_CHANNEL      6      // RF-0 のチャンネルサーベイ結果に合わせて変更
#define UNAGI_MAX_SAMPLES 4000    // 8B/sample → 32 kB。RAM 実測後に調整

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
  MSG_HELLO       = 0x0B,
  MSG_CAL_SET     = 0x0C,   // 較正係数を NVS に書き込む（無線）
  MSG_CAL_ACK     = 0x0D,
};

// 走行モード
enum : uint8_t {
  MODE_SWIM      = 0,  // 通常の CPG 追従
  MODE_GPIO_SYNC = 1,  // SYNC-1: t_start で GPIO を1回トグルするだけ
  MODE_HALL_RAW  = 2,  // HALL-1: モータ停止、v1/v2 生値を記録（較正用）
  MODE_PWM_SWEEP = 3,  // HALL-2: 関節を機械的に固定し PWM を掃引
  MODE_STEP      = 4,  // JOINT-1: 目標角ステップ応答
};

#pragma pack(push, 1)

struct Hdr {
  uint8_t  type;
  uint8_t  node_id;      // 1..10、ブロードキャストでは 0
  uint16_t trial_id;
};

struct MsgBeacon   { Hdr h; uint32_t seq; int64_t t_head_us; };
struct MsgSyncBegin{ Hdr h; uint16_t n_points; };
struct MsgStart    { Hdr h; int64_t t_start_head_us; };
struct MsgStop     { Hdr h; uint8_t reason; };
struct MsgHello    { Hdr h; uint8_t mac[6]; uint32_t fw_hash; };

struct MsgParam {
  Hdr      h;
  float    A_deg, f_hz, phi_deg, off_deg;
  float    kp, ki, kd;
  uint16_t duration_ms;
  uint16_t log_hz;       // 200 / 500 / 1000
  uint8_t  mode;
  float    aux;          // STEP の振幅 / PWM_SWEEP の最大デューティ
};

struct MsgReady {
  Hdr      h;
  uint32_t fw_hash;
  float    sigma_us;     // 同期回帰の残差 SD
  double   b;            // クロック比
  double   a;            // オフセット [us]
  uint16_t param_crc;
  uint32_t free_ram;
  int16_t  hall1_bias;
  int16_t  hall2_bias;
  uint8_t  ready;
};

struct MsgReport {
  Hdr      h;
  uint32_t n_samples;
  int64_t  t_start_local_us;
  uint32_t missed;
  uint32_t max_jitter_us;
  float    rms_err_deg;
  float    adc_us_max;
};

struct MsgCalSet { Hdr h; float c[4]; };
struct MsgCalAck { Hdr h; float c[4]; };

struct MsgDumpReq  { Hdr h; uint16_t block; };
#define DUMP_SAMPLES_PER_BLOCK 28        // 28 × 8 B = 224 B（ESP-NOW 上限 250 B 内）
struct MsgDumpData { Hdr h; uint16_t block; uint16_t n; int16_t d[DUMP_SAMPLES_PER_BLOCK][4]; };

#pragma pack(pop)

static inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}
