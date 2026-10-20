// unagi_bus.h — RS-485 半二重バスのフレーム送受信（ヘッド / ノード共通）
//
//  フレーム:  [0x7E] [len] [payload: len バイト] [CRC16 下位] [CRC16 上位]
//             len = 6..250。CRC16 (Modbus 多項式) は [len]+[payload] に掛ける。
//             payload は unagi_protocol.h のメッセージ構造体そのもの。
//             0x7E の後ろは HDLC と同じバイトスタッフィング（0x7E / 0x7D → 0x7D, x^0x20）。
//             なので 0x7E は「フレームの頭」にしか現れず、途中で壊れても次の頭で必ず復帰する。
//
//  ・送信は Serial1 (UART1)。DE は UART の RS-485 半二重モードが自動で切り替える
//    （RTS 出力を DE に割り当て。送信の最終ビットが出終わった瞬間に割り込みで LOW に戻る）。
//    BUS_HW_DE=0 にするとソフトで切り替える（write → flush → LOW）。
//  ・受信は onReceive コールバック（UART イベントタスク）でバイトを拾い、
//    完成したフレームだけを FreeRTOS キューに積む。タイムスタンプはコールバック入口の時刻。
//  ・SP3485 の /RE は DE と直結なので、送信中は自分の送信を受信しない（エコーなし）。
#pragma once
#include <Arduino.h>
#include <esp_timer.h>
#include "unagi_protocol.h"

#ifndef BUS_BAUD
#define BUS_BAUD        1000000     // 1 Mbps（SP3485 は 10 Mbps まで。配線が長く不安定なら 500000）
#endif
#ifndef BUS_HW_DE
#define BUS_HW_DE       1           // 1 = UART ハードウェアで DE 制御（推奨） / 0 = ソフト制御
#endif
#define BUS_SOF         0x7E
#define BUS_ESC         0x7D
#define BUS_RX_TIMEOUT_SYMBOLS 3    // 3 文字ぶん無音で「フレーム終わり」とみなしコールバックを呼ぶ

struct BusRx {
  int64_t t_us;                     // 受信コールバック入口の esp_timer 時刻
  uint8_t len;
  uint8_t data[UNAGI_MAX_PAYLOAD];
};

struct BusStats {
  uint32_t rx_ok;        // 受理したフレーム
  uint32_t rx_crc;       // CRC 不一致
  uint32_t rx_len;       // 長さ異常
  uint32_t rx_other;     // 相手違い（ノード同士の返事など。正常でも増える）
  uint32_t rx_qfull;     // キューあふれ
  uint32_t rx_uart_err;  // UART エラー（フレーミング / オーバーフロー）
  uint32_t tx;           // 送信フレーム数
};

namespace bus {

static HardwareSerial&  U = Serial1;
static int8_t           pin_de = -1;
static uint8_t          accept_magic = 0;
static QueueHandle_t    rxq = nullptr;
static SemaphoreHandle_t tx_mtx = nullptr;
static BusStats         st = {};

// ---- 受信パーサ（UART イベントタスクの中だけで動く）----
static bool     p_active = false;
static bool     p_esc = false;
static uint8_t  p_len = 0;
static uint16_t p_idx = 0;
static uint8_t  p_buf[1 + UNAGI_MAX_PAYLOAD + 2];     // [len][payload][crc lo][crc hi]
static int64_t  p_last_us = 0;
static BusRx    p_frame;

static inline void feed(uint8_t b, int64_t t) {
  if (b == BUS_SOF) {                                 // フレームの頭（どの状態からでもやり直す）
    if (p_active && p_idx) st.rx_len++;               // 前のフレームが途中で切れていた
    p_active = true; p_esc = false; p_idx = 0;
    return;
  }
  if (!p_active) return;
  if (b == BUS_ESC) { p_esc = true; return; }
  if (p_esc) { b ^= 0x20; p_esc = false; }
  if (p_idx == 0) {
    if (b < sizeof(Hdr) || b > UNAGI_MAX_PAYLOAD) { st.rx_len++; p_active = false; return; }
    p_len = b;
  }
  p_buf[p_idx++] = b;
  if (p_idx < (uint16_t)(1 + p_len + 2)) return;
  p_active = false;
  uint16_t c  = crc16(p_buf, 1 + p_len);
  uint16_t rc = (uint16_t)p_buf[1 + p_len] | ((uint16_t)p_buf[2 + p_len] << 8);
  if (c != rc)                  { st.rx_crc++;   return; }
  if (p_buf[1] != accept_magic) { st.rx_other++; return; }
  p_frame.t_us = t;
  p_frame.len  = p_len;
  memcpy(p_frame.data, p_buf + 1, p_len);
  if (xQueueSend(rxq, &p_frame, 0) == pdTRUE) st.rx_ok++;
  else                                        st.rx_qfull++;
}

static void on_receive() {
  const int64_t t = esp_timer_get_time();             // まず時刻（同期ビーコン用）
  if (p_active && t - p_last_us > 2000) p_active = false;   // 途中で切れたフレームは捨てる
  p_last_us = t;
  uint8_t tmp[64];
  int n;
  while ((n = U.available()) > 0) {
    if (n > (int)sizeof(tmp)) n = sizeof(tmp);
    n = (int)U.read(tmp, n);
    for (int i = 0; i < n; i++) feed(tmp[i], t);
  }
}

static void on_error(hardwareSerial_error_t) { st.rx_uart_err++; p_active = false; }

// rx/tx/de: GPIO 番号。magic: 受け取るフレームの先頭バイト（ノード = HEAD、ヘッド = NODE）
static void begin(int8_t rx, int8_t tx, int8_t de, uint8_t magic, uint16_t queue_len) {
  pin_de = de;
  accept_magic = magic;
  rxq    = xQueueCreate(queue_len, sizeof(BusRx));
  tx_mtx = xSemaphoreCreateMutex();
#if !BUS_HW_DE
  pinMode(de, OUTPUT); digitalWrite(de, LOW);
#endif
  U.setRxBufferSize(2048);
  U.begin(BUS_BAUD, SERIAL_8N1, rx, tx);
#if BUS_HW_DE
  U.setPins(rx, tx, -1, de);                    // RTS → DE
  if (!U.setMode(UART_MODE_RS485_HALF_DUPLEX)) Serial.println("ERR bus setMode(RS485)");
#endif
  U.setRxTimeout(BUS_RX_TIMEOUT_SYMBOLS);
  U.onReceiveError(on_error);
  U.onReceive(on_receive, false);
}

// 1 フレーム送る。複数タスクから呼んでよい（フレーム単位で排他）。
static bool send(const void* payload, size_t n) {
  if (n < sizeof(Hdr) || n > UNAGI_MAX_PAYLOAD) return false;
  static uint8_t raw[1 + UNAGI_MAX_PAYLOAD + 2];
  static uint8_t fb[1 + 2 * sizeof(raw)];
  xSemaphoreTake(tx_mtx, portMAX_DELAY);
  raw[0] = (uint8_t)n;
  memcpy(raw + 1, payload, n);
  uint16_t c = crc16(raw, n + 1);
  raw[1 + n] = (uint8_t)(c & 0xFF);
  raw[2 + n] = (uint8_t)(c >> 8);
  size_t m = 0;
  fb[m++] = BUS_SOF;
  for (size_t i = 0; i < n + 3; i++) {
    uint8_t b = raw[i];
    if (b == BUS_SOF || b == BUS_ESC) { fb[m++] = BUS_ESC; fb[m++] = b ^ 0x20; }
    else fb[m++] = b;
  }
#if BUS_HW_DE
  U.write(fb, m);
#else
  digitalWrite(pin_de, HIGH);
  delayMicroseconds(1);
  U.write(fb, m);
  U.flush();                                    // 最終ビットが出るまで待つ
  digitalWrite(pin_de, LOW);
#endif
  st.tx++;
  xSemaphoreGive(tx_mtx);
  return true;
}

static inline bool recv(BusRx& out, TickType_t wait) { return xQueueReceive(rxq, &out, wait) == pdTRUE; }

}  // namespace bus
