# 適用手順 — 既存の Calibration_procotol プロジェクトを直す

以前の説明で `firmware\node` というフォルダ構成を前提にしてしまいましたが、
すでに PlatformIO プロジェクトが1つできているので、**それをそのまま使う**方が
簡単です。1プロジェクトの中に `node` と `head` の2環境を置く形に直しました。

---

## 1. まず消すもの

`Calibration_procotol\src\` から以下を**削除**してください。

| ファイル | 理由 |
|---|---|
| `node.ino` | `main.cpp` と重複。`setup()` と `loop()` が2つになりリンクエラー |
| `platformio.ini` | `src\` に置いても読まれません（読まれるのはルートの方） |
| `README.md` | ビルド対象フォルダに置く必要なし |
| `hall_calib.py` | 同上 |
| `push_cal.py` | 同上 |
| `unagi_test.py` | 同上 |
| `main.cpp` | 新しい `src\node\main.cpp` に置き換わります |
| `unagi_protocol.h` | 新しい `include\` に移動します |

結果として `src\` は一度空になります。

念のため `.pio` フォルダも消してください（古いビルド結果が残っています）。
VS Code なら `PROJECT TASKS` → `General` → `Clean All` でも同じです。

---

## 2. 置くもの

このフォルダの中身を `Calibration_procotol\` に上書きコピーしてください。

```
Calibration_procotol\
  platformio.ini              ← 上書き
  include\
    unagi_protocol.h
  src\
    node\
      main.cpp                ← モータノード
    head\
      main.cpp                ← ヘッド
  pc\
    unagi_test.py
    hall_calib.py
    push_cal.py
```

`lib\`、`test\`、`.vscode\`、`UNAGI-.code-workspace` はそのままで構いません。

---

## 3. なぜこの形なのか

`platformio.ini` に `build_src_filter` を書いています。

```ini
[env:node]
build_src_filter = +<node/>

[env:head]
build_src_filter = +<head/>
```

PlatformIO は既定で `src\` 以下の `.c` `.cpp` `.ino` を**全部**コンパイルします。
ノードとヘッドの `main.cpp` を両方置くと `setup()` と `loop()` が重複するので、
環境ごとにどちらか一方だけを見るよう指定しています。

共有ヘッダを `include\` に置いたのは、PlatformIO が `include\` を自動的に
インクルードパスに入れるからです。これで `-I` の指定が要らなくなり、
node と head の両方から `#include "unagi_protocol.h"` が通ります。

---

## 4. VS Code での操作

プロジェクトを開き直す必要はありません。`Calibration_procotol` を開いたままで、
左サイドバーの `PROJECT TASKS` がこうなります。

```
node
 ├── General
 │    ├── Build
 │    ├── Upload
 │    ├── Monitor
 │    └── Upload and Monitor      ← これを使う
 └── Platform
      └── Erase Flash

head
 ├── General
 │    ├── Build
 │    ├── Upload
 │    └── ...

node_core3
 └── ...
```

**環境が3つ並ぶので、必ず `node` か `head` の下のボタンを押してください。**
ウィンドウ最下部の青いステータスバーのボタンは「今アクティブな環境」に対して
動くため、環境を取り違えやすいです。`PROJECT TASKS` から実行する方が確実です。

もし `PROJECT TASKS` が更新されない場合は、コマンドパレット
(`Ctrl+Shift+P`) → `PlatformIO: Rebuild IntelliSense Index` を実行するか、
VS Code を開き直してください。

---

## 5. 書き込みの流れ

```
[ヘッド基板を USB で挿す]
  head > Upload

[ノード1 を USB で挿す]
  node > Upload and Monitor
    モニタに「★ ID 未設定です」と出る
    id 1        ← 打って Enter
    save
    Ctrl+C でモニタを閉じる
  USB を抜く

[ノード2 を挿す]
  node > Upload and Monitor      ← ソースは一切触らない
    id 2
    save
  ...ノード10 まで同じ
```

書き込み操作自体は10回必要です（10個の基板のフラッシュに書くので当然です）。
**間でソースを編集しなくていい**のがポイントです。

モニタは Ctrl+C で必ず閉じてください。COM ポートを掴んだままだと次の Upload が
`could not open port` で失敗します。

---

## 6. Python スクリプトの実行

`pc\` に移したので、パスが変わります。

```powershell
cd C:\Users\kary\OneDrive - empa.ch\1_program\UNAGI-\src\Calibration_procotol
python pc\unagi_test.py mac
```

PC の WiFi を SSID `UNAGI`（パスワード `unagi12345`）に接続してから実行して
ください。ヘッドのシリアルに `head ready ip=192.168.4.1` が出ていれば正常です。

CSV は実行したフォルダの下の `data\` に出ます。

---

## 7. ビルドが通らないとき

**`unagi_protocol.h: No such file or directory`**
→ `include\unagi_protocol.h` が無い。`src\` に残っていないか確認。

**`multiple definition of 'setup'`**
→ `src\` に `node.ino` か古い `main.cpp` が残っている。

**`.pio` 関連の妙なエラー**
→ `Clean All` を実行するか、`.pio` フォルダを手で消す。

**モニタに何も出ない**
→ `platformio.ini` の `-DARDUINO_USB_CDC_ON_BOOT=1` が効いているか確認。
   一度 Upload し直してから基板の RESET を押す。
