# UNAGI_Calibration (protocol v3 / RS-485)

ヘッド1台 + モータノード10台。給電は XT30 の数珠つなぎ、通信は RS-485 半二重バス（1 Mbps）。
PC はヘッドの WiFi (SoftAP) に TCP で繋ぐ。

> **ESP-NOW 版 (v2) からの変更点は [CHANGES_RS485.md](CHANGES_RS485.md)。**
> 古いファームを新しいノード基板に書くとモータが全開で回るので、必ず先に読んでください。

> **Hall 校正（hall_calib_gui.py）は2通りの方法で使えます。** 詳しくは [docs/CALWIFI.md](docs/CALWIFI.md) を見てください。
> - **ヘッド経由（おすすめ）：** 本番の head と node のファームのまま使えます。WiFi `UNAGI-HEAD` の 192.168.4.1:**5000** に接続します。電源の電圧・電流も記録されます。
> - **ノード単体：** ノードに `node_calwifi` を書き込み、WiFi `UNAGI-NODE-<id>` に接続します。

```
PC ──TCP 192.168.4.1:5555──▶ ヘッド (XIAO C3, INA226, ロードスイッチ, SP3485, 終端 120 Ω + バイアス)
                               │  RS-485 A/B 1 Mbps（ビーコン 20 Hz 常時、ヘッドが1台ずつ指名）
                               ├── ノード 1 (XIAO C3, DRV8833, SS39ET×2, SP3485)
                               ├── ...
                               └── ノード 10（終端ジャンパを入れる）
```

```
UNAGI_Calibration/
  platformio.ini          env:node / env:head
  include/unagi_protocol.h  メッセージ定義（ヘッド・ノード共通）
  include/unagi_bus.h       RS-485 のフレーム送受信（ヘッド・ノード共通）
  src/node/main.cpp       ノード（10台同一バイナリ。ID は NVS）
  src/head/main.cpp       ヘッド（PowerMeasurement の機能 + ゲートウェイ）
  pc/unagi.py             PC ツール（標準ライブラリのみ）
  data/                   CSV 出力先（自動作成）
```

---

## 1. VS Code で開く

1. **File → Open Folder** で `UNAGI_Calibration` フォルダそのものを開く
   （親フォルダや `.code-workspace` ではなく）。
2. 左の 🐜 PlatformIO → **PROJECT TASKS** に `node` と `head` が出れば OK。
3. 旧 `Calibration_procotol` は閉じておく（同じ COM ポートを取り合うため）。

## 2. 書き込み

| 対象 | 操作 |
|---|---|
| ヘッド | USB を挿す → `head ▸ General ▸ Upload` |
| ノード | USB を挿す → `node ▸ General ▸ Upload and Monitor` |

ノードは初回だけモニタで ID を設定（旧ファームで設定済みなら**そのまま残ります**）：

```
id 3
save
show        ← id=3、bus の rx_ok が増えている（head=受信中）ことを確認
```
モニタは `Ctrl+C` で閉じてから次の基板へ。**ソースは編集しない。**

## 3. 動作確認

PC の WiFi を `UNAGI-HEAD` / `unagi12345` に接続して：

```powershell
python pc\unagi.py check
```

ヘッド → 給電 → 10台の HELLO → 同期品質とホール値 の順に表示されます。
応答しないノードがあれば、そのノードを USB に挿してモニタで `show`：

| `show` の表示 | 意味 |
|---|---|
| `head=(未受信)` / `rx_ok=0` | ビーコンが届いていない（ヘッド未起動 / ケーブル / 終端） |
| `rx_ok=0` で `crc` か `uart_err` が増える | A と B が逆 |
| `id=0` | ID 未設定 → `id <n>` `save` |
| `sync: n=0` | ビーコン未受信。上と同じ |
| `nFAULT=LOW(異常)` | DRV8833 の過電流・過熱・低電圧 |

ノードを初めて動かすときは `pwm 0.3 500` と `hall` で、角度が **増える** 向きか確認。
減るなら `dir -1` → `save`。

## 4. 較正（ノードごと、対話式）

```powershell
python pc\unagi.py peek  --node 1 --watch   # 手で関節を動かして diff が変わるか（Ctrl+C で終了）
python pc\unagi.py calib --node 1           # -30..+30° を 5° ごとに固定 → Enter
```
`calib` は各角度でホール差分を 200 点平均 → 3 次多項式 → そのままノードの NVS へ書き込み。
再フラッシュ不要。その後：

```powershell
python pc\unagi.py setzero --node 1         # 0° 合わせ（任意）
python pc\unagi.py minduty --node 1 --value 0.12
```

## 5. 試験

```powershell
python pc\unagi.py openloop --nodes 1 --amp 0.4 --freq 1 --seconds 3   # まず開ループで動くか
python pc\unagi.py joint1   --node 1 --step 15                         # 閉ループ（較正後）
python pc\unagi.py sync                                                  # 同期品質
python pc\unagi.py sync1    --reps 20                                    # D10 パルスをオシロで
python pc\unagi.py swim     --freq 2 --amp 20 --dphi 36 --seconds 10
python pc\unagi.py halt                                                  # 非常停止
```
CSV 列: `t_s, cmd_deg, meas_deg, u, v1, v2, diff`（t_s は T からのヘッド時間）。
全コマンドは `python pc\unagi.py -h`。送受信を全部見たいときは `-v`。

## 6. ヘッドのコマンド（シリアルでも TCP でも同じ）

```
ver | help | nodes | bus | discover | power 0|1 [force] | keepload 0|1 | tele 0|1
param <id> <mode> <A> <f> <phi> <off> <kp> <ki> <kd> <aux> <dur_ms> <log_hz>
arm | go [delay_ms] | halt | dump <id> | peek <id> [n]
cal <id> c0 c1 c2 c3 | minduty <id> x | setzero <id>
```
PowerMeasurement の `ON OFF START STOP INFO CLEAR ZERO SELFTEST TARE SAVE GAIN OFFSET RATE AVG TRIP`
もそのまま使えるので、`Unagi_head_gui.py` も動きます（TCP は同時に1クライアントのみ）。

`bus` は RS-485 の受信カウンタ（CRC エラー・タイムアウト数）。

mode: 0 SWIM(閉ループ正弦) / 1 SYNC_TEST / 2 HALL_RAW / 3 PWM_SWEEP / 4 STEP / 5 OPENLOOP

---