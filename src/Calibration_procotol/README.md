# UNAGI Calibration / Control Protocol

10関節の分散WiFi制御システム一式。PlatformIO 1プロジェクト2環境。

```
Calibration_procotol/
  platformio.ini            env:node / env:head / env:node_core3
  include/
    unagi_protocol.h        共有メッセージ定義
  src/
    node/main.cpp           モータノード ×10（同一バイナリ）
    head/main.cpp           ヘッド（SoftAP + TCP ↔ ESP-NOW）
  pc/
    unagi_test.py           受入試験ランナー
    hall_calib.py           SS39ET 較正の多項式フィット
    push_cal.py             較正係数を無線で NVS に書き込む
  data/                     CSV 出力先
  docs/
    SETUP.md                ★ まずこれを読む（既存プロジェクトの直し方）
    VSCODE_GUIDE.md         VS Code + PlatformIO の操作
    HARDWARE.md             配線・ピン割り当て・つまずきやすい点
    PROCEDURE.md            システム全体の設計と受入試験の手順書
```

---

## クイックスタート

### 1. 書き込み

VS Code の `PROJECT TASKS` から：

```
head > Upload                     ヘッド基板
node > Upload and Monitor         ノード1
  > id 1
  > save
  Ctrl+C
```

ノード2〜10も同じ手順。**ソースは一切編集しません。**
ノードIDは NVS に保存されます。

### 2. 接続確認

PC の WiFi を SSID `UNAGI` / パスワード `unagi12345` に接続してから：

```powershell
python pc\unagi_test.py mac
python pc\unagi_test.py sync
```

### 3. 較正（ノードごとに1回）

```powershell
python pc\unagi_test.py hall1 --node 1
python pc\hall_calib.py --hall data\hall1_node1.csv --ref data\ref_steps.csv --node 1
python pc\push_cal.py --node 1 --coef 0.12 0.02134 -1.2e-6 3.4e-10
```

再フラッシュ不要。ノードが受信して即 NVS に保存します。

### 4. 実験

```powershell
python pc\unagi_test.py sync1 --reps 20        # 位相同期精度（要オシロ）
python pc\unagi_test.py joint1 --node 1 --step 15
python pc\unagi_test.py swim --freq 2.0 --amp 20 --dphi 36 --seconds 10
```

---

## 配線 (XIAO ESP32-C3)

既存の `hall_motor_node` と同一。**D10 のみ新規**。

| 信号 | XIAO | GPIO |
|---|---|---|
| DRV8833 AIN1 | D3 | 5 |
| DRV8833 AIN2 | D4 | 6 |
| enable / SLP | D2 | 4 |
| SS39ET S1 | A0 | 2 |
| SS39ET S2 | A1 | 3 |
| SYNC 出力 | D10 | 10 |

詳細は `docs/HARDWARE.md`。

---

## 設計の要点

- ヘッド↔ノードは **ESP-NOW**（TCP/IP を使わない）。PC↔ヘッドのみ TCP。
- 時刻同期はビーコン10 Hz × 300点の**線形回帰**。オフセットとクロック比を
  同時推定するので、走行中の通信が不要になる。
- スタートは「今」ではなく**絶対時刻 T** を配信。取りこぼしても位相がずれない。
- 走行中、ノードは送信しない。受信は STOP のみ。
- ログは各ノードの RAM に溜め、走行後に ESP-NOW で回収。

根拠と受入基準は `docs/PROCEDURE.md`。
