# UNAGI 制御ソフトウェア（カメラフィードバック版）

Hallセンサを廃止し、各モータ上の AprilTag を俯瞰カメラで追跡して、**ラップトップ側だけで閉ループ制御**を行う構成。ヘッドとノードには制御則もセンサ処理もなく、「指令を中継する」「指令どおりにコイルを駆動する」だけ。

```
┌──────── Laptop ────────┐   WiFi UDP    ┌──── Head (XIAO C3) ────┐  RS-485 1 Mbaud  ┌─ Node ×10 ─┐
│ camera → AprilTag      │  duty[10]     │ UDP → RS-485 中継       │  broadcast       │ slot(id-1)  │
│ → 関節角               │ ───────────▶  │ watchdog 200 ms         │ ───────────────▶ │ → DRV8833   │
│ CPG + FF + PD          │ ◀───────────  │ INA226 / 過電流遮断     │  (一方向のみ)     │ watchdog    │
│ CSV ログ               │  telemetry    │ AON6407 モータ電源 SW   │                  │ 100 ms      │
└────────────────────────┘               └─────────────────────────┘                  └────────────┘
```

1カメラフレーム = 1制御ステップ。ヘッドはWiFi AP（`UNAGI-HEAD` / `192.168.4.1:4210`）。

## ファイル

| パス | 役割 |
|---|---|
| `firmware/` | **PlatformIO プロジェクト**（VS Code で `firmware` フォルダを開く） |
| `firmware/platformio.ini` | 環境 `head` / `node`、ボード `seeed_xiao_esp32c3` |
| `firmware/src/head/main.cpp` | ヘッド：UDP受信 → RS-485ブロードキャスト、ウォッチドッグ、INA226テレメトリ、過電流遮断 |
| `firmware/src/node/main.cpp` | ノード（10台とも**同一バイナリ**）：RS-485受信 → 自分のスロットのdutyでDRV8833駆動 |
| `laptop/unagi_control.py` | カメラ → AprilTag → 関節角 → CPG+FF+PD → UDP送信、CSVログ、キャリブレーション |
| `laptop/mock_head.py` | ハードなしでラップトップ側を試すための疑似ヘッド |

## 通信プロトコル

### Laptop → Head（UDP, little-endian, 30 B）
| オフセット | 型 | 内容 |
|---|---|---|
| 0 | `'U''N'` | マジック |
| 2 | u8 | type: 1=CMD, 2=STOP, 3=ARM, 4=DISARM, 5=PING |
| 3 | u8 | n（ノード数, CMD時のみ） |
| 4 | u16 | seq |
| 6 | u32 | ラップトップ時刻 [ms] |
| 10 | i16 × n | duty（‰, −1000…+1000） |

### Head → Laptop（UDP, 25 B、受信パケットごとに返信）
`'U''T'` / seq echo u16 / head_ms u32 / V f32 / I f32 / P f32 / E[J] f32 / flags u8
flags: 1=ARMED, 2=STREAMING, 4=TIMEOUT, 8=OVERCURRENT, 16=INA_FAIL

### Head → Nodes（RS-485, 26 B @ 10ノード ≈ 260 µs）
`A5 5A | type | seq | len | payload | CRC16(lo,hi)`
- type 0x01 CMD: payload = i16 duty × n。ノード id k はスロット k−1 を読む
- type 0x02 STOP: payload なし。アイドル中もヘッドが50 msごとに送信
- CRC16-CCITT-FALSE（type〜payload末尾）

送信者はヘッドだけなので衝突なし。ヘッドは DE を常時 HIGH、ノードは常時 LOW。**120 Ω終端は両端（ヘッドと最後のノード）**に。

## 安全系（3段）
1. **ノード**：有効フレームが100 ms来なければコイルOFF（コースト）
2. **ヘッド**：CMDが200 ms来なければSTOPをブロードキャスト。総電流 > `I_LIMIT_A`（15 A）でモータ電源（AON6407）を遮断。ARMされるまでCMDは無視
3. **ラップトップ**：duty ±800、参照角 ±25° にクランプ。終了時・Ctrl-C・スペースキーで STOP + DISARM

## 立ち上げ手順

### PlatformIO（VS Code）

- **`firmware` フォルダそのもの**を VS Code で開く（`unagi` ではなく。PlatformIO は開いたフォルダ直下の `platformio.ini` を探す）
- 下部ステータスバーの環境セレクタ（`Default (firmware)` と表示されている所）で `env:node` / `env:head` を選び、→（Upload）、🔌（Serial Monitor）
- 初回ビルドでプラットフォーム（pioarduino, core 3.x）を自動ダウンロード。数分かかる
- 標準の `platform = espressif32`（core 2.x）でもビルドできる（PWM部分は両対応）
- CLI の場合：
  ```
  pio run -e node -t upload --upload-port COM5
  pio device monitor -e node --port COM5
  pio run -e head -t upload
  ```
- 書き込めないとき：XIAO の **BOOT を押したまま RESET**（または押しながらUSB接続）でダウンロードモードに入れてから Upload

### 手順

1. **ピン確認**：`src/head/main.cpp` / `src/node/main.cpp` 冒頭の `PIN_*` を基板の回路図と照合（仮置き値です）。D0/D8/D9 はストラップピンなのでDRV8833入力には避ける
2. **ノード書き込み**：10台とも `env:node` を書き込み、シリアルモニタ（115200）で `id 1` … `id 10` と入力して Enter。ID未設定のノードはコイルを駆動しない（PlatformIO のモニタは入力をそのまま送るので、打った文字が表示されなくても Enter で送信される）
3. **ノード単体テスト**：USBシリアルで `t 500` → 0.5 秒だけ +500 で駆動。極性と動作確認
4. **ヘッド書き込み**：シリアルに `INA226=ok` が出ることを確認
5. **バス + コイルのベンチ試験**（カメラなし）：
   ```
   python unagi_control.py --open-loop --duration 10 --mode carangiform --freq 1.0 --amp 15
   ```
   各ノードで `?` を打つと `ok` が増え `crcErr` が増えないことを確認できる
6. **キャリブレーション**（ロボットを真っ直ぐにして水中で）：
   ```
   python unagi_control.py --calibrate
   ```
   ゼロ点と、関節ごとの静的ゲイン [deg/duty]（**符号=極性を含む**）を `unagi_calib.json` に保存
7. **閉ループ**：
   ```
   python unagi_control.py --mode carangiform --freq 1.5 --amp 20
   ```
   `g` で歩容開始/停止（振幅は滑らかにランプ）、`z` でゼロ点取り直し、スペースで非常停止、`q` で終了

ハードなしでのラップトップ側確認：`python mock_head.py` → 別ターミナルで `python unagi_control.py --open-loop --head 127.0.0.1`

## 関節角の定義（要確認）

タグ中心の**位置**から計算（タグのyawは使わない → どちらのセグメントに貼られていても結果が変わらない）。
点列 P0 = ヘッドタグ, P1…P10 = モータ1…10 のタグ, P11 = 尾タグ として、
**関節 i の角度 = (Pi − Pi−1) から (Pi+1 − Pi) への符号付き角 − ゼロ点**。

→ **関節10を閉ループにするには尾びれ付近にタグ（ID 11）が必要**。無ければ `TAIL_TAG_ID = None` にすれば関節10はフィードフォワードのみで動く。タグが一時的に見えない関節も自動でフィードフォワードのみになる（`TAG_HOLD_S` = 0.1 s までは最後の位置を保持）。

タグID・ファミリ（`tag36h11` 仮定）は `unagi_control.py` 冒頭の CONFIG で変更。

## 制御則

```
θ_ref, θ̇_ref = CPG（位相振動子チェーン, 隣接結合）を LATENCY_S だけ位相進め
cmd[deg]     = θ_ref + Kp·(θ_ref − θ) + Kd·(θ̇_ref − θ̇)      （θ が見えない関節は第1項のみ）
duty         = cmd / g_i                                     （g_i = キャリブで得た deg/duty）
```

- **遅延補償**：カメラ露光＋検出＋WiFi＋バスで20〜30 ms程度。2 Hzで25 msは **18° の位相遅れ**なので、参照を `LATENCY_S` だけ先読みしている。実測値（ログの指令と応答の相互相関）に合わせて調整
- **Kp, Kd**：まず `KP=0` でFFのみ → 少しずつ上げる。カメラ遅延があるので高ゲインは発振しやすい。Kd は角速度の差分ノイズに敏感なので小さく
- 静的ゲイン g は準静的な値なので、1.5〜3 Hz では振幅が不足/過剰になる。周期運動なので、次の段階として**周期ごとの反復学習制御（ILC）でFF波形を補正**するのが相性がよい（前の周期の誤差 e_k(φ) を使い u_{k+1}(φ) = u_k(φ) + L·e_k(φ+Δ)）
- 歩容プリセット（`GAITS`）は振幅包絡 A(s)=a0+(1−a0)s^p と体長あたりの波数だけの初期値。ベイズ最適化で探索する対象もここ（freq, amp, a0, p, body_waves）

## ログ

`laptop/logs/unagi_<mode>_<日時>.csv`：時刻, dt, seq, 歩容ON/OFF, 各関節の ref / θ / duty, ヘッドの V / I / P / 積算エネルギー / flags。
タグ位置（x, y）の生データも必要なら `JointEstimator.points` を同じ行に追記する。
