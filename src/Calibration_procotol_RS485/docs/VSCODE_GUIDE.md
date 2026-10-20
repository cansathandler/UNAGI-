# VS Code + PlatformIO 操作手順（詳細版）

CLI コマンドは全部 GUI のボタンで代替できます。以下は VS Code 前提の手順です。

---

## 1. プロジェクトを開く

PlatformIO は **`platformio.ini` があるフォルダ**をプロジェクトのルートと見なします。
リポジトリ全体を開くと `platformio.ini` が見つからず、PlatformIO のツールバーが
出てきません。ここが最初につまずく点です。

VS Code で：

1. 左端のアリのアイコン（PlatformIO）をクリック
2. `PIO Home` → `Open Project`
3. **`UNAGI-\src\Calibration_procotol\unagi\firmware\node`** を選ぶ

ノードとヘッドは別プロジェクトなので、両方使うなら
`File` → `Add Folder to Workspace` で `firmware\head` も足してください。
ウィンドウ下の青いステータスバーと、左サイドバーの `PROJECT TASKS` に
両方が並びます。

---

## 2. ビルドと書き込み（ボタンの場所）

左サイドバーの `PROJECT TASKS` を開くと、こういう木構造になっています。

```
node
 ├── General
 │    ├── Build          ← ビルドだけ
 │    ├── Upload         ← ビルド + 書き込み
 │    ├── Monitor        ← シリアルモニタ
 │    └── Upload and Monitor   ← これが一番便利
 └── Platform
      └── Erase Flash    ← NVS を消したいとき
```

`platformio.ini` に環境を2つ書いてあるので、実際には

```
node          ← env:node        (Arduino core 2.x、既定)
node_core3    ← env:node_core3  (Arduino core 3.x)
```

の2つが並びます。**普段は `node` の方だけ使ってください。**
`node_core3` は core 3.x を試したいときだけです。

ウィンドウ最下部の青いステータスバーにも同じものがあります：
`✓`(Build) `→`(Upload) `🗑`(Clean) `🔌`(Monitor)。
ただしステータスバーは「今アクティブな環境」に対して動くので、
環境を選び違えないよう `PROJECT TASKS` から実行する方が確実です。

---

## 3. 「10台すべてに同じバイナリ」の意味

ここが分かりにくかったと思うので、具体的に書きます。

**やらないこと**：ノードごとに `main.cpp` を書き換えて10回ビルドする。

**やること**：

```
ノード1 を USB で挿す
  → node > Upload and Monitor
  → モニタに "★ ID 未設定です" と出る
  → > id 1
  → > save
  → USB を抜く

ノード2 を USB で挿す
  → node > Upload and Monitor     ← main.cpp は一切触らない
  → > id 2
  → > save
  → USB を抜く

... ノード10 まで同じ
```

書き込み自体は10回必要です（10個の基板それぞれのフラッシュに書くので当然です）。
**違うのは、間でソースを編集しなくていい**という点だけです。以前のコードは
`#define NODE_ID 1` が固定だったので、10回とも数字を書き換えて10回ビルドし直す
必要がありました。今は NVS に入るので、ビルドは1回で済みます。

較正係数 `CAL[4]` も同じ理由で NVS に入れました。HALL-1 の結果が出るたびに
10台フラッシュし直すのは現実的ではないからです。

---

## 4. シリアルモニタでコマンドを打つ

`Monitor` を開くと、PlatformIO は `pio device monitor` を VS Code の
ターミナルパネルで走らせます。**そのターミナルにそのまま打ち込めます。**

```
--- Terminal on COM7 | 115200 8-N-1
UNAGI node  id=0  mac=8C:AA:B5:12:34:56
★ ID 未設定です。 'id <1-10>' のあと 'save' を実行してください。

id 1            ← ここに打って Enter
id = 1 (save で保存)
save
NVS に保存しました
show
id=1  mac=8C:AA:B5:12:34:56  fw=00010004
CAL = {0, 0.02, 0, 0}  minduty=0.000
state=0  sigma=1000000000.0us  b=1.000000000  freeheap=198432
hall v1=2047 v2=2051 diff=-4 deg=-0.08
```

`>` は私が書いた例のプロンプト記号で、実際には表示されません。打つのは
`id 1` の部分だけです。

**モニタを閉じるときは Ctrl+C** です。モニタが COM ポートを掴んだままだと
次の Upload が失敗します（`could not open port` と出ます）。

### 何も表示されないとき

XIAO ESP32-C3 はネイティブUSB（USB CDC）なので、`platformio.ini` に

```ini
-DARDUINO_USB_MODE=1
-DARDUINO_USB_CDC_ON_BOOT=1
```

が無いと `Serial` の出力が UART0 (GPIO20/21) に行ってしまい、USB では何も
見えません。これは入れてあります。それでも出ない場合は、基板の RESET を
一度押してみてください。

### コマンドを打っても反応しないとき

`monitor_eol = LF` を設定してあります。これが無いと Enter が `\r` だけを
送り、ファーム側の `readStringUntil('\n')` が1秒待ってからタイムアウト
するので、反応が1秒遅れたり効かなかったりします。

---

## 5. Windows の COM ポート

10台を順に挿すと COM 番号が毎回変わります。基本は自動検出に任せて、
**USB には1台だけ挿した状態**で作業してください。

どのポートか確認したいときは `PROJECT TASKS` の下の方、
`Miscellaneous` → `Devices` で一覧が出ます。

固定したい場合は `platformio.ini` に追記できますが、10台を回すなら
自動検出のままの方が楽です。

```ini
upload_port  = COM7
monitor_port = COM7
```

---

## 6. NVS を消したいとき

ID や較正係数を全部リセットしたい場合は
`PROJECT TASKS` → `Platform` → `Erase Flash` です。
そのあともう一度 Upload が必要になります。

---

## 7. `-I$PROJECT_DIR/../common` について

共有ヘッダ `unagi_protocol.h` は `firmware/common/` に1つだけ置いて、
node と head の両方から参照しています。`$PROJECT_DIR` は PlatformIO が
プロジェクトフォルダの絶対パスに展開してくれるので、VS Code がどこから
起動されても通ります。

`unagi_protocol.h: No such file or directory` というエラーが出たら、
`firmware/` 以下のフォルダ構成が崩れています。

```
firmware/
  common/unagi_protocol.h
  node/platformio.ini
  node/src/main.cpp
  head/platformio.ini
  head/src/main.cpp
```

この形を保ってください。

---

## 8. 全体の流れ

```
[1回だけ]
  firmware\head  を開く → Upload          ヘッド基板に書き込み
  firmware\node  を開く → Upload          ノード1に書き込み
                          Monitor → id 1 → save
                          （ノード2〜10も同じ、ソースは触らない）

[較正: ノードごとに1回]
  python pc\unagi_test.py hall1 --node 1
  python pc\hall_calib.py --hall data\hall1_node1.csv --ref data\ref_steps.csv --node 1
  python pc\push_cal.py --node 1 --coef <出力された4つの数字>
                                          ← 再フラッシュ不要

[毎回の実験]
  PCのWiFiを SSID "UNAGI" に接続
  python pc\unagi_test.py sync
  python pc\unagi_test.py swim --freq 2.0 --amp 20
```
