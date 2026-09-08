# UNAGI テストスクリプト一式

```
firmware/common/unagi_protocol.h   共有メッセージ定義
firmware/head/head.ino             ヘッド (SoftAP + TCP ↔ ESP-NOW)
firmware/node/node.ino             モータノード ×10
pc/unagi_test.py                   テストランナー
pc/hall_calib.py                   SS39ET 較正
```

`unagi_protocol.h` は head / node 両方のスケッチフォルダにコピーするか、
Arduino のライブラリパスに置いてください。

---

## 配線 (XIAO ESP32-C3)

| 信号 | ピン | 備考 |
|---|---|---|
| SS39ET #1 out | GPIO2 (A0) | **ADC1** |
| SS39ET #2 out | GPIO3 (A1) | **ADC1** |
| DRV8833 AIN1 | GPIO6 | |
| DRV8833 AIN2 | GPIO7 | |
| DRV8833 nSLEEP | GPIO10 | |
| SYNC 出力 | GPIO4 | SYNC-1 のオシロ観測用 |

**GPIO5 (A3) は ADC2 なので使えません。** ESP32-C3 では WiFi 動作中に ADC2 が
無効になります。ホール2本は必ず GPIO0〜4 に載せてください。

SS39ET は 3.3 V で駆動します。データシート上は 2.7〜6.5 V ですが、5 V にすると
中点が 2.5 V になり ESP32-C3 の ADC レンジ (減衰 12 dB で約 3.1 V) を超えます。
3.3 V なら中点 1.65 V、振幅も収まります。比例出力なので感度は電圧に比例して
下がる点に注意（HALL-1 の較正で実測されます）。

2本の信号線は必ずツイストペアにし、モータ配線から離してください。差動で
コモンモードは消えますが、配線が非対称だと消えません。

---

## 実施順序

```bash
# 0. ノードの MAC を確認（各ノードは起動時に HELLO を送る）
python pc/unagi_test.py mac

# 1. 同期が成立するか
python pc/unagi_test.py sync

# 2. RF-0  ブロードキャスト受信率
python pc/unagi_test.py rf0 --minutes 5

# 3. SYNC-1  位相同期精度（要オシロ、10ch）
python pc/unagi_test.py sync1 --reps 20

# 4. DRIFT-1  クロック比の安定性
python pc/unagi_test.py drift1 --minutes 10

# 5. HALL-1  各ノードの較正（10回繰り返す）
python pc/unagi_test.py hall1 --node 1
python pc/hall_calib.py --hall data/hall1_node1.csv --ref data/ref_steps.csv --node 1
#   → 出力された CAL[4] を node.ino に書き込んで再フラッシュ

# 6. HALL-2  PWM クロストーク（関節を治具で固定して実行）
python pc/unagi_test.py hall2 --node 1

# 7. JOINT-1  ステップ応答
python pc/unagi_test.py joint1 --node 1 --step 15

# 8. 全関節遊泳
python pc/unagi_test.py swim --freq 2.0 --amp 20 --dphi 36 --seconds 10
```

---

## .md の手順書からの変更点

**制御ループを ISR から高優先度タスクに移しました。**
手順書では制御を ISR 内で完結させると書きましたが、`analogRead()` は ISR-safe
ではありません。タイマ ISR はセマフォを渡すだけにし、優先度 24 のタスクで
ADC・PID・ログを実行します。ジッタは ISR 直接実行より数十 µs 悪化しますが、
1 kHz に対しては十分で、ADC が安全に使えます。

**HALL-1 / HALL-2 を追加しました。**
AS5600 は絶対角が出るので較正が不要でしたが、SS39ET はアナログ出力なので
角度較正が必須です。HALL-2 は差動配置がモータ駆動ノイズを実際に打ち消せて
いるかを検証するもので、SS39ET を2本使う設計の根拠そのものになります。
論文にこの数値を載せてください。

---

## つまずきやすい点

**`missed > 0` が出る場合。** ADC のオーバーサンプリングが 1 ms 予算を食って
います。`OVERSAMPLE` を 4 → 2 に下げるか、`log_hz` を 1000 → 200 に下げて
ください。REPORT の `adc_us_max` が実測値です。400 µs を超えていたら要対策。

**ADC 連続変換への移行。** oneshot の `analogRead()` は 1 回 20〜30 µs かかります。
ノイズを下げるためにオーバーサンプリングを増やしたいなら、ESP-IDF の
ADC continuous (DMA) モードに移行してください。20 kHz で回しっぱなしにして、
制御タスクは直近 20 サンプルを平均するだけ、という構成にできます。

**PWM と ADC のビート。** `PWM_HZ` は `CTRL_HZ` の整数倍 (25 kHz = 25 × 1 kHz)
にしてあります。これを崩すと、見かけの角度に低周波のうなりが乗ります。
HALL-2 でこれが観測できます。

**RAM。** `UNAGI_MAX_SAMPLES` を 20000 にすると `logbuf` だけで 320 kB になり、
ESP32-C3 では確保できません。まず `ESP.getFreeHeap()` (READY の RAM 列) を
確認し、実際に取れる値まで下げてください。200 Hz × 20 秒 = 4000 サンプル
(64 kB) あたりが安全な出発点です。
