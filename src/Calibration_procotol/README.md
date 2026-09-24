# UNAGI テストスクリプト一式 (PlatformIO)

```
firmware/
  common/unagi_protocol.h        共有メッセージ定義
  node/
    platformio.ini
    src/main.cpp                 モータノード（同一バイナリを10台に）
  head/
    platformio.ini
    src/main.cpp                 ヘッド (SoftAP + TCP ↔ ESP-NOW)
pc/
  unagi_test.py                  テストランナー
  hall_calib.py                  SS39ET 較正の多項式フィット
  push_cal.py                    較正係数を無線で NVS に書き込む
```

`build_flags = -I../common` で共有ヘッダを参照しています。`firmware/` の
ディレクトリ構造を崩すとインクルードが通らなくなります。

---

## ビルドと書き込み

```powershell
cd firmware\node
pio run -t upload            # ← 10台すべてに同じバイナリを書く

cd ..\head
pio run -t upload
```

`platformio.ini` には環境が2つあります。既定は Arduino core 2.x（公式
プラットフォーム、安定）です。core 3.x を使いたい場合は `-e node_core3` を
付けてください。`main.cpp` は `ESP_ARDUINO_VERSION_MAJOR` でタイマ API と
LEDC API と ESP-NOW コールバック署名を出し分けているので、どちらでも通ります。

```powershell
pio run -e node_core3 -t upload
```

pioarduino の URL にあるバージョン番号は
https://github.com/pioarduino/platform-espressif32/releases
で最新を確認してください。

---

## ノードIDの設定（1台につき1回だけ）

**バイナリは全台共通です。** ノードIDは NVS に保存されるので、個体ごとの
ビルドは不要です。書き込み後、各ノードをUSBで繋いで1回だけ設定します。

```powershell
pio device monitor
```

```
> id 3
id = 3 (save で保存)
> save
NVS に保存しました
> show
id=3  mac=A1:B2:...  fw=00010004
CAL = {0, 0.02, 0, 0}
hall v1=2047 v2=2051 diff=-4 deg=-0.08
```

`show` はホール生値もその場で出すので、配線確認にも使えます。関節を手で
動かして `diff` が変化すれば配線は正常です。

デッドゾーン補償 `minduty` も NVS に入ります。既存ファームの `MINDUTY` と
同じ意味で、モータが回り始める最小デューティです。個体差が大きいので、
`en 1` でドライバを起こしてから1台ずつ実測して設定してください。

```
> en 1
> minduty 0.12
> save
```

ドライブはスローディケイ固定にしてあります（既存ノードで検証済みの設定）。

ID 未設定（`id=0`）のノードは ESP-NOW を一切受け付けません。起動時に
シリアルへ警告が出ます。

---

## 較正係数の書き込み（再フラッシュ不要）

```powershell
python pc\unagi_test.py hall1 --node 3
python pc\hall_calib.py --hall data\hall1_node3.csv --ref data\ref_steps.csv --node 3
# → CAL[4] = {0.12, 0.02134, -1.2e-06, 3.4e-10}

python pc\push_cal.py --node 3 --coef 0.12 0.02134 -1.2e-6 3.4e-10
```

ノードは受信すると即 NVS に保存し、`CALACK` を返します。10台分を
フラッシュし直す必要はありません。

---

## 配線 (XIAO ESP32-C3)

既存の `hall_motor_node` と**同一の配線**です。基板の作り直しは不要です。

| 信号 | XIAO | GPIO | 備考 |
|---|---|---|---|
| DRV8833 AIN1 | D3 | 5 | |
| DRV8833 AIN2 | D4 | 6 | |
| enable / SLP | D2 | 4 | HIGH = 駆動可 |
| SS39ET S1 | A0 | 2 | **ADC1** |
| SS39ET S2 | A1 | 3 | **ADC1** |
| SYNC 出力 | D10 | 10 | SYNC-1 用（**新規追加**） |

**D10 だけが新しい配線です。** D2 は既存コードでイネーブルに使われているので、
SYNC パルスは D10 に逃がしました。D8 (GPIO8) と D9 (GPIO9) はブート
ストラッピングピンなので避けています。

**ADC2 はアナログ入力に使えません。** ESP32-C3 では WiFi 動作中に ADC2 が
無効になるため、ホール2本は ADC1 (GPIO0〜4) に載せる必要があります。
GPIO5 は ADC2 ですが、PWM 出力として使う分には問題ありません。

ホール出力は **2.5 V 以下**に収めてください（11 dB 減衰時の実用レンジ）。
SS39ET を 3.3 V で駆動すれば中点 1.65 V で収まります。

2本の信号線は必ずツイストペアにし、モータ配線から離してください。差動で
コモンモードは消えますが、配線が非対称だと消えません。

---

## 実施順序

```powershell
python pc\unagi_test.py mac                      # 10台の MAC 確認
python pc\unagi_test.py sync                     # 同期成立の確認
python pc\unagi_test.py rf0 --minutes 5          # RF-0
python pc\unagi_test.py sync1 --reps 20          # SYNC-1（要オシロ）
python pc\unagi_test.py drift1 --minutes 10      # DRIFT-1
python pc\unagi_test.py hall1 --node 1           # HALL-1（10台分）
python pc\unagi_test.py hall2 --node 1           # HALL-2（治具で固定）
python pc\unagi_test.py joint1 --node 1 --step 15
python pc\unagi_test.py swim --freq 2.0 --amp 20 --dphi 36 --seconds 10
```

---

## つまずきやすい点

**`-I../common` が効かない。** PlatformIO はプロジェクトルートからの相対パスを
解決します。`pio run` は `firmware/node/` の中で実行してください。うまく
いかない場合は `-I${PROJECT_DIR}/../common` に書き換えます。

**`missed > 0` が出る。** ADC のオーバーサンプリングが 1 ms 予算を食っています。
`OVERSAMPLE` を 4 → 2 に下げるか、`log_hz` を下げてください。REPORT の
`adc_us_max` が実測値です。400 µs を超えていたら要対策。

**RAM。** `UNAGI_MAX_SAMPLES` は 4000（32 kB）にしてあります。200 Hz なら
20 秒、1 kHz なら 4 秒分です。READY が返す空きRAMを見て増減してください。

**PWM と ADC のビート。** `PWM_HZ` は `CTRL_HZ` の整数倍（25 kHz = 25 × 1 kHz）
にしてあります。崩すと見かけの角度に低周波のうなりが乗り、HALL-2 で観測されます。

**制御ループが ISR ではなくタスクである理由。** `analogRead()` は ISR-safe では
ないので、タイマ ISR はセマフォを渡すだけにし、優先度 24 のタスクで ADC・PID・
ログを実行しています。