// head.ino — UNAGI ヘッド (ESP32)
// PC とは SoftAP + TCP、ノードとは ESP-NOW。両者は同一チャンネルであること。

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "unagi_protocol.h"

#define AP_SSID "UNAGI"
#define AP_PASS "unagi12345"
#define TCP_PORT 5000

WiFiServer server(TCP_PORT);
WiFiClient pc;

uint8_t node_mac[UNAGI_N_NODES + 1][6];
bool    node_known[UNAGI_N_NODES + 1] = {false};
uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

volatile bool beacon_on = false;
uint32_t beacon_seq = 0;
uint16_t trial = 0;

// --- PC へ1行送る ---
void tx(const String& s) { if (pc && pc.connected()) { pc.print(s); pc.print('\n'); } }

void on_recv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  if (len < (int)sizeof(Hdr)) return;
  const Hdr* h = (const Hdr*)d;
  uint8_t id = h->node_id;
  if (id >= 1 && id <= UNAGI_N_NODES && !node_known[id]) {
    memcpy(node_mac[id], info->src_addr, 6); node_known[id] = true;
    esp_now_peer_info_t p{}; memcpy(p.peer_addr, info->src_addr, 6);
    p.channel = UNAGI_CHANNEL; p.encrypt = false; esp_now_add_peer(&p);
  }

  switch (h->type) {
    case MSG_READY: {
      auto* m = (const MsgReady*)d;
      tx(String("READY,") + id + "," + m->ready + "," + String(m->sigma_us,1) + ","
         + String((m->b - 1.0) * 1e6, 3) + "," + m->param_crc + "," + m->free_ram + ","
         + m->hall1_bias + "," + m->hall2_bias);
      break;
    }
    case MSG_REPORT: {
      auto* m = (const MsgReport*)d;
      tx(String("REPORT,") + id + "," + m->n_samples + "," + (long)m->t_start_local_us + ","
         + m->missed + "," + m->max_jitter_us + "," + String(m->rms_err_deg,3) + ","
         + String(m->adc_us_max,0));
      break;
    }
    case MSG_DUMP_DATA: {
      auto* m = (const MsgDumpData*)d;
      String s = String("DUMP,") + id + "," + m->block + "," + m->n;
      for (int i = 0; i < m->n; i++)
        for (int k = 0; k < 4; k++) { s += ","; s += m->d[i][k]; }
      tx(s);
      break;
    }
  }
}

void beacon_task(void*) {
  for (;;) {
    if (beacon_on) {
      MsgBeacon b{}; b.h = {MSG_BEACON, 0, trial};
      b.seq = beacon_seq++; b.t_head_us = esp_timer_get_time();
      esp_now_send(BCAST, (uint8_t*)&b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(100));      // 10 Hz
  }
}

// --- PC コマンド処理 ---
// sync <n> | param <id> <A> <f> <phi> <off> <kp> <ki> <kd> <dur_ms> <log_hz> <mode> <aux>
// arm | start <delay_ms> | stop | dump <id> <block> | mac
void handle(String c) {
  c.trim();
  if (c.startsWith("sync")) {
    int n = c.substring(5).toInt(); if (n <= 0) n = 300;
    MsgSyncBegin m{}; m.h = {MSG_SYNC_BEGIN, 0, trial}; m.n_points = n;
    esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
    beacon_seq = 0; beacon_on = true;
    tx(String("OK,sync,") + n);
  }
  else if (c.startsWith("beacon_off")) { beacon_on = false; tx("OK,beacon_off"); }
  else if (c.startsWith("param")) {
    float v[12]; int id; 
    sscanf(c.c_str(), "param %d %f %f %f %f %f %f %f %f %f %f %f",
           &id, &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9],&v[10]);
    MsgParam p{}; p.h = {MSG_PARAM, (uint8_t)id, trial};
    p.A_deg=v[0]; p.f_hz=v[1]; p.phi_deg=v[2]; p.off_deg=v[3];
    p.kp=v[4]; p.ki=v[5]; p.kd=v[6];
    p.duration_ms=(uint16_t)v[7]; p.log_hz=(uint16_t)v[8];
    p.mode=(uint8_t)v[9]; p.aux=v[10];
    if (node_known[id]) esp_now_send(node_mac[id], (uint8_t*)&p, sizeof(p));
    tx(String("OK,param,") + id + "," + crc16((uint8_t*)&p, sizeof(p)));
  }
  else if (c.startsWith("arm")) {
    trial++;
    Hdr h{MSG_ARM, 0, trial};
    esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
    tx(String("OK,arm,") + trial);
  }
  else if (c.startsWith("start")) {
    int dly = c.substring(6).toInt(); if (dly <= 0) dly = 200;
    MsgStart m{}; m.h = {MSG_START, 0, trial};
    m.t_start_head_us = esp_timer_get_time() + (int64_t)dly * 1000;
    for (int k = 0; k < 3; k++) {                 // 取りこぼし対策に 3 回
      esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
      delay(5);
    }
    tx(String("OK,start,") + (long)m.t_start_head_us);
  }
  else if (c.startsWith("stop")) {
    MsgStop m{}; m.h = {MSG_STOP, 0, trial}; m.reason = 1;
    esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
    tx("OK,stop");
  }
  else if (c.startsWith("dump")) {
    int id, blk; sscanf(c.c_str(), "dump %d %d", &id, &blk);
    MsgDumpReq m{}; m.h = {MSG_DUMP_REQ, (uint8_t)id, trial}; m.block = blk;
    if (node_known[id]) esp_now_send(node_mac[id], (uint8_t*)&m, sizeof(m));
  }
  else if (c.startsWith("mac")) {
    for (int i = 1; i <= UNAGI_N_NODES; i++)
      if (node_known[i]) {
        char b[32]; snprintf(b, 32, "%02X:%02X:%02X:%02X:%02X:%02X",
          node_mac[i][0],node_mac[i][1],node_mac[i][2],node_mac[i][3],node_mac[i][4],node_mac[i][5]);
        tx(String("MAC,") + i + "," + b);
      }
    tx("OK,mac");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, UNAGI_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_now_init();
  esp_now_register_recv_cb(on_recv);
  esp_now_peer_info_t p{}; memcpy(p.peer_addr, BCAST, 6);
  p.channel = UNAGI_CHANNEL; p.encrypt = false; esp_now_add_peer(&p);
  server.begin();
  xTaskCreate(beacon_task, "beacon", 4096, nullptr, 5, nullptr);
  Serial.printf("head ready  ip=%s\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  if (!pc || !pc.connected()) { pc = server.available(); }
  if (pc && pc.available()) handle(pc.readStringUntil('\n'));
  delay(1);
}
