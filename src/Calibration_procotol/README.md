# UNAGI_Calibration (protocol v2)

ヘッド1台 + モータノード10台。給電は電源ケーブル、通信は無線（ESP-NOW）。
PC はヘッドの WiFi (SoftAP) に TCP で繋ぐ。

```
PC ──TCP 192.168.4.1:5555──▶ ヘッド (XIAO C3, INA226, ロードスイッチ)
                               │  ESP-NOW ch 6（ビーコン 20 Hz 常時）
                               ├──▶ ノード 1 (XIAO C3, DRV8833, SS39ET×2)
                               ├──▶ ...
                               └──▶ ノード 10
```

```
UNAGI_Calibration/
  platformio.ini          env:node / env:head
  include/unagi_protocol.h
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
show        ← id=3, ch=6 が出ることを確認
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
| `head=(未受信)` | ヘッドのビーコンが届いていない（ヘッド未起動 / ch 違い / 距離） |
| `id=0` | ID 未設定 → `id <n>` `save` |
| `sync: n=0` | ビーコン未受信。上と同じ |

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
ver | help | nodes | discover | power 0|1 [force] | keepload 0|1 | tele 0|1
param <id> <mode> <A> <f> <phi> <off> <kp> <ki> <kd> <aux> <dur_ms> <log_hz>
arm | go [delay_ms] | halt | dump <id> | peek <id> [n]
cal <id> c0 c1 c2 c3 | minduty <id> x | setzero <id>
```
PowerMeasurement の `ON OFF START STOP INFO CLEAR ZERO SELFTEST TARE SAVE GAIN OFFSET RATE AVG TRIP`
もそのまま使えるので、`Unagi_head_gui.py` も動きます（TCP は同時に1クライアントのみ）。

mode: 0 SWIM(閉ループ正弦) / 1 SYNC_TEST / 2 HALL_RAW / 3 PWM_SWEEP / 4 STEP / 5 OPENLOOP

---

## 旧 Calibration_procotol が動かなかった理由

| # | 症状 | 原因 |
|---|---|---|
| 1 | READY/REPORT がヘッドに届かない | 起動時に全ノードが HELLO をブロードキャスト → 他ノードがそれを「ヘッド」と誤認して学習し、HELLO を投げ合い続けていた |
| 2 | `hall1` `joint1` `swim` が始まらない | ARM で trial を更新していなかったため、START の trial が常に不一致で無視されていた |
| 3 | `sync1` が1回しか動かない | 1回走ると同期状態が消え、毎回 30 秒の同期が必要だった |
| 4 | 角度がずれる | 較正は生の差分、走行時は ARM 時のバイアスを引いた差分に多項式を当てていた |
| 5 | ダンプが詰まる / 行が混ざる | ヘッドが ESP-NOW 受信コールバック（WiFi タスク）内で TCP に書いていた |
| 6 | 開始時に一瞬すべてが固まる | ノードが T まで最大 300 ms、最高優先度で空回りしていた（WiFi も止まる） |

v2 では、ヘッド発/ノード発を先頭バイトで区別し、同期は常時ビーコンの移動窓回帰、
受信処理はすべてキュー経由で loop() 側、に変えています。
