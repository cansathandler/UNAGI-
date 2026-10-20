#!/usr/bin/env python3
"""
unagi.py — UNAGI ヘッド (TCP 192.168.4.1:5555) 経由でノード10台を動かす PC ツール  (protocol v3 / RS-485)

PC の WiFi を SSID "UNAGI-HEAD" / pass "unagi12345" に繋いでから実行。
依存ライブラリなし（標準ライブラリのみ）。

まずこれ:
    python pc/unagi.py check                      # ヘッド・給電・10台の応答・同期・ホール値を一覧

較正（1台ずつ・対話式）:
    python pc/unagi.py peek   --node 1 --watch    # 配線確認: 関節を手で動かして diff が変わるか
    python pc/unagi.py calib  --node 1            # -30..+30° を 5° ごとに固定 → 多項式 → NVS へ
    python pc/unagi.py setzero --node 1           # 今の姿勢を 0° にする
    python pc/unagi.py minduty --node 1 --value 0.12

受入試験:
    python pc/unagi.py sync                       # 同期品質 (sigma, skew)
    python pc/unagi.py sync1   --reps 20          # SYNC-1: 全ノード D10 パルス（オシロ）
    python pc/unagi.py drift1  --minutes 10
    python pc/unagi.py hall1   --node 1 --seconds 60
    python pc/unagi.py hall2   --node 1 --duty 0.6
    python pc/unagi.py openloop --nodes all --amp 0.4 --freq 1 --dphi 36 --seconds 5
    python pc/unagi.py joint1  --node 1 --step 15
    python pc/unagi.py swim    --freq 2 --amp 20 --dphi 36 --seconds 10

その他:
    python pc/unagi.py power on|off
    python pc/unagi.py nodes
    python pc/unagi.py halt
    python pc/unagi.py raw "ver"
    python pc/unagi.py fit --hall data/hall1_node1.csv --ref data/ref_steps.csv --node 1
"""
import argparse, csv, math, socket, sys, time
from datetime import datetime
from pathlib import Path

HOST, PORT = "192.168.4.1", 5555
N_NODES = 10
MAX_SAMPLES = 4000
OUT = Path(__file__).resolve().parent.parent / "data"

MODE = dict(swim=0, sync=1, hall_raw=2, sweep=3, step=4, openloop=5)
WHY = {0: "ok", 1: "ビーコン未受信", 2: "同期不良", 3: "param 未受信", 4: "走行中"}
ABORT = {0: "", 1: "STOP", 2: "角度超過", 3: "ログ満杯"}
STATE = {0: "IDLE", 1: "ARMED", 2: "WAIT", 3: "RUN", 4: "DONE"}


# =====================================================================
class Head:
    def __init__(self, host=HOST, port=PORT, verbose=False):
        try:
            self.s = socket.create_connection((host, port), timeout=5.0)
        except OSError as e:
            sys.exit(f"ヘッド {host}:{port} に接続できません ({e})\n"
                     f"  → PC の WiFi が UNAGI-HEAD に繋がっているか確認。GUI を開いていたら閉じる"
                     f"（ヘッドは TCP クライアント1台のみ）。")
        self.buf = b""
        self.verbose = verbose
        self.send("tele 0")        # 電力テレメトリ(JSON 20 Hz)を止める。GUI で START すれば再開
        self.send("keepload 1")    # このスクリプトが切断してもノード給電を落とさない
        self.drain(0.3)

    def send(self, cmd):
        if self.verbose:
            print(f"  → {cmd}")
        self.s.sendall((cmd + "\n").encode())

    def lines(self, seconds):
        t_end = time.time() + seconds
        while True:
            while b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                ln = ln.decode(errors="replace").strip()
                if ln:
                    if self.verbose and not ln.startswith("DUMP,"):
                        print(f"  ← {ln}")
                    yield ln
            rem = t_end - time.time()
            if rem <= 0:
                return
            self.s.settimeout(max(0.02, rem))
            try:
                d = self.s.recv(65536)
            except socket.timeout:
                continue
            if not d:
                raise ConnectionError("ヘッドとの接続が切れました")
            self.buf += d

    def drain(self, seconds):
        return list(self.lines(seconds))

    def wait(self, prefix, seconds, pred=None):
        for ln in self.lines(seconds):
            if ln.startswith(prefix):
                f = ln.split(",")
                if pred is None or pred(f):
                    return f
            elif ln.startswith("ERR"):
                print(f"  ★ {ln}")
        return None

    def collect(self, prefix, seconds, ids=None, pred=None):
        """prefix で始まる行を id ごとに集める。ids が全部揃ったら早く返る"""
        got = {}
        for ln in self.lines(seconds):
            if ln.startswith(prefix):
                f = ln.split(",")
                if pred and not pred(f):
                    continue
                got[int(f[1])] = f
                if ids and all(i in got for i in ids):
                    break
            elif ln.startswith(("ERR", "WARN")):
                print(f"  ★ {ln}")
        return got

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


# =====================================================================
def parse_nodes(s):
    if s in (None, "all"):
        return list(range(1, N_NODES + 1))
    return [int(x) for x in str(s).split(",") if x.strip()]


def power(h, on, boot_s=3.0):
    h.send(f"power {1 if on else 0}")
    f = h.wait("OK,power", 2.0)
    if f is None:
        h.send("power 1 force" if on else "power 0")
        f = h.wait("OK,power", 2.0)
    if on and f:
        print(f"  ノード給電 ON。起動待ち {boot_s:.0f} 秒...", flush=True)
        h.drain(boot_s)
    return f is not None


def discover(h, want=None, quiet=False):
    h.send("discover")
    got = h.collect("HELLO", 1.5, ids=want)
    if not quiet:
        print(f"  {'id':>3}  {'mac':17}  {'fw':8} {'st':5}  CAL (c0 c1 c2 c3)            minduty")
        for i in sorted(got):
            f = got[i]
            tag = "  ← ID 未設定（USB で id <n> → save）" if i == 0 else ""
            print(f"  {i:>3}  {f[2]:17}  {f[3]:8} {STATE.get(int(f[5]), f[5]):5}  "
                  f"{float(f[6]):+.3g} {float(f[7]):.4g} {float(f[8]):.3g} {float(f[9]):.3g}   {f[10]}{tag}")
        miss = [i for i in (want or []) if i not in got]
        if miss:
            print(f"  応答なし: {miss}")
    return got


def peek(h, node, n=64):
    h.send(f"peek {node} {n}")
    f = h.wait("PEEK", 1.0 + n / 1000.0, pred=lambda f: int(f[1]) == node)
    if f is None:
        return None
    k = ["id", "state", "n", "v1", "v2", "diff", "diff_sd", "deg", "sigma", "ppm", "sync_n"]
    d = dict(zip(k, f[1:]))
    for x in k[2:]:
        d[x] = float(d[x])
    d["state"] = STATE.get(int(d["state"]), d["state"])
    return d


def send_params(h, ids, make_line, tries=3):
    """make_line(id) -> 'param ...'。PACK の CRC がヘッド計算値と一致するまで再送"""
    ok = []
    for i in ids:
        for _ in range(tries):
            h.send(make_line(i))
            f = h.wait("OK,param", 1.0, pred=lambda f: int(f[2]) == i)
            if f is None:
                break
            crc = f[3]
            a = h.wait("PACK", 0.5, pred=lambda f: int(f[1]) == i)
            if a and a[2] == crc:
                ok.append(i)
                break
        if i not in ok:
            print(f"  ★ ノード {i}: param が届きません")
    return ok


def arm(h, ids):
    h.send("arm")
    f = h.wait("OK,arm", 1.0)
    if f is None:
        return None, {}
    trial = int(f[2])
    rows = h.collect("READY", 2.0, ids=ids, pred=lambda f: int(f[2]) == trial)
    return trial, rows


def print_ready(ids, rows):
    print(f"  {'id':>3} {'ready':>5} {'sigma[us]':>9} {'skew[ppm]':>9} {'n':>4} {'heap[kB]':>8}  v1/v2")
    bad = []
    for i in ids:
        f = rows.get(i)
        if f is None:
            print(f"  {i:>3}   ---  応答なし"); bad.append(i); continue
        rdy, why = int(f[3]), int(f[4])
        note = "" if rdy else f"  ← {WHY.get(why, why)}"
        print(f"  {i:>3} {rdy:>5} {float(f[7]):>9.1f} {float(f[8]):>9.2f} {int(f[6]):>4} "
              f"{int(f[9]) // 1024:>8}  {f[10]}/{f[11]}{note}")
        if not rdy:
            bad.append(i)
    return bad


def dump(h, node):
    h.send(f"dump {node}")
    rows, missing, n_total = [], 0, None
    t_end = time.time() + 60
    for ln in h.lines(60):
        if ln.startswith("DUMP,"):
            p = ln.split(",")
            if int(p[1]) != node:
                continue
            n = int(p[3])
            v = p[4:4 + n * 6]
            for k in range(n):
                t_us, cmd, meas, u, v1, v2 = (int(x) for x in v[k * 6:(k + 1) * 6])
                rows.append([t_us / 1e6, cmd / 100.0, meas / 100.0, u / 10000.0, v1, v2, v1 - v2])
        elif ln.startswith("DUMPMISS"):
            missing += 1
        elif ln.startswith("DUMPEND"):
            p = ln.split(",")
            if int(p[1]) == node:
                n_total = int(p[2])
                break
        elif ln.startswith("ERR"):
            print(f"  ★ {ln}")
            break
        if time.time() > t_end:
            break
    return rows, n_total, missing


def save_csv(rows, name, header):
    OUT.mkdir(exist_ok=True)
    path = OUT / name
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"  → {path}  ({len(rows)} 行)")
    return path


def fit_log_hz(seconds, log_hz):
    if seconds * log_hz <= MAX_SAMPLES:
        return log_hz
    for hz in (1000, 500, 250, 200, 125, 100, 50, 40, 25, 20, 10, 5, 2, 1):
        if seconds * hz <= MAX_SAMPLES:
            print(f"  ※ ログ容量 {MAX_SAMPLES} 点に収めるため log_hz を {log_hz} → {hz} に下げます")
            return hz
    return 1


# =====================================================================
def run_trial(h, ids, make_param, seconds, tag, force=False, go_delay_ms=300):
    """params → arm → go → REPORT → dump。成功したら {id: rows} を返す"""
    got = discover(h, want=ids, quiet=True)
    ids_ok = [i for i in ids if i in got]
    if len(ids_ok) < len(ids):
        print(f"  ★ 応答しないノード: {[i for i in ids if i not in got]}")
        if not force or not ids_ok:
            return None
        ids = ids_ok

    if len(send_params(h, ids, make_param)) < len(ids) and not force:
        return None

    trial, rows = arm(h, ids)
    bad = print_ready(ids, rows)
    if bad:
        if not force:
            print(f"\n  READY でないノード {bad}。中止（--force で強行）")
            return None
        ids = [i for i in ids if i not in bad]

    h.send(f"go {go_delay_ms}")
    if h.wait("OK,go", 1.0) is None:
        print("  ★ go に応答なし"); return None
    print(f"  trial {trial} 開始 ({seconds:.1f} s)", flush=True)
    reports = h.collect("REPORT", seconds + go_delay_ms / 1000 + 3.0, ids=ids,
                        pred=lambda f: int(f[2]) == trial)

    print(f"\n  {'id':>3} {'n':>6} {'missed':>6} {'jit[us]':>7} {'loop[us]':>8} "
          f"{'rms[deg]':>8} {'start_err':>9}  abort")
    exclude = False
    for i in ids:
        f = reports.get(i)
        if f is None:
            print(f"  {i:>3}  REPORT なし"); exclude = True; continue
        missed, ab = int(f[5]), int(f[9])
        print(f"  {i:>3} {int(f[3]):>6} {missed:>6} {int(f[6]):>7} {int(f[8]):>8} "
              f"{float(f[7]):>8.3f} {int(f[10]):>7}us  {ABORT.get(ab, ab)}")
        exclude |= missed > 0 or ab != 0
    if exclude:
        print("  ★ 除外基準に該当（missed > 0 / abort / REPORT 欠落）。解析に使わないこと。")

    print("\n  ログ回収中...", flush=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = {}
    for i in ids:
        if i not in reports or int(reports[i][3]) == 0:
            continue
        rows, n_total, miss = dump(h, i)
        if miss or n_total is None or len(rows) != n_total:
            print(f"  ★ ノード {i}: {len(rows)}/{n_total} 点、欠落ブロック {miss}")
        save_csv(rows, f"{tag}_{stamp}_t{trial}_node{i}.csv",
                 ["t_s", "cmd_deg", "meas_deg", "u", "v1", "v2", "diff"])
        out[i] = rows
    return out


# =====================================================================
#  commands
# =====================================================================
def cmd_check(h, a):
    print("[1] ヘッド")
    h.send("ver")
    f = h.wait("OK,ver", 2.0)
    if f is None:
        print("  ★ 'ver' に応答なし。ヘッドに新しいファームが書かれていません。")
        print("     PlatformIO: PROJECT TASKS → head → Upload")
        return
    print("  " + ",".join(f[2:]))

    print("\n[2] ノード給電")
    power(h, True)
    h.send("ver")
    f = h.wait("OK,ver", 2.0)
    vbus = next((float(x[5:]) for x in f if x.startswith("vbus=")), None) if f else None
    print(f"  VBUS = {vbus} V")
    if vbus is not None and vbus < 1.0:
        print("  ★ スイッチを閉じても VBUS が 0 V。電源/配線を確認。USB 給電のノードだけでも試験は続けられます。")

    print("\n[3] ノード応答 (discover)")
    got = discover(h, want=list(range(1, N_NODES + 1)))
    if not got:
        print("  ★ 1台も応答しません。確認:")
        print("     - ノードにも新しいファームを書いたか (PROJECT TASKS → node → Upload)")
        print("     - ノードのシリアルで 'show' → bus の rx_ok が増えているか (head=受信中)")
        print("     - rx_ok=0 で crc/uart_err が増えるなら A/B が逆。何も増えないならケーブル/終端/電源")
        print("     - ヘッドで 'bus' → crc とタイムアウトの数を確認")
        return

    print("\n[4] 同期とホール値 (peek)")
    print(f"  {'id':>3} {'state':>5} {'v1':>7} {'v2':>7} {'diff':>8} {'sd':>6} {'deg':>7} "
          f"{'sigma[us]':>9} {'skew[ppm]':>9} {'n':>4}")
    for i in sorted(k for k in got if k >= 1):
        d = peek(h, i, 32)
        if d is None:
            print(f"  {i:>3}  応答なし"); continue
        warn = ""
        if d["sync_n"] < 40:
            warn = "  ← 同期点不足（起動直後なら数秒待つ）"
        elif d["sigma"] > 500:
            warn = "  ← 同期不良"
        elif min(d["v1"], d["v2"]) < 50 or max(d["v1"], d["v2"]) > 4000:
            warn = "  ← ホール値が端に張り付き（配線/電源）"
        print(f"  {i:>3} {d['state']:>5} {d['v1']:>7.1f} {d['v2']:>7.1f} {d['diff']:>8.2f} "
              f"{d['diff_sd']:>6.2f} {d['deg']:>7.2f} {d['sigma']:>9.1f} {d['ppm']:>9.2f} "
              f"{int(d['sync_n']):>4}{warn}")


def cmd_nodes(h, a):
    discover(h, want=list(range(1, N_NODES + 1)))


def cmd_power(h, a):
    power(h, a.state == "on")
    if a.state == "on":
        discover(h, want=list(range(1, N_NODES + 1)))


def cmd_peek(h, a):
    while True:
        d = peek(h, a.node, a.n)
        if d is None:
            print("  応答なし")
        else:
            print(f"  v1={d['v1']:7.1f}  v2={d['v2']:7.1f}  diff={d['diff']:8.2f} ±{d['diff_sd']:.2f}  "
                  f"deg={d['deg']:7.2f}   sync: n={int(d['sync_n'])} sigma={d['sigma']:.0f}us", flush=True)
        if not a.watch:
            break
        time.sleep(0.2)


def cmd_sync(h, a):
    ids = parse_nodes(a.nodes)
    print("  同期は常時ビーコン(20 Hz)で維持されています。現在の品質:")
    print(f"  {'id':>3} {'sigma[us]':>9} {'skew[ppm]':>9} {'n':>4}")
    for i in ids:
        d = peek(h, i, 1)
        if d is None:
            print(f"  {i:>3}  応答なし"); continue
        flag = "" if (d["sigma"] < 500 and d["sync_n"] >= 40) else "  ← NG"
        print(f"  {i:>3} {d['sigma']:>9.1f} {d['ppm']:>9.2f} {int(d['sync_n']):>4}{flag}")


def cmd_sync1(h, a):
    ids = parse_nodes(a.nodes)
    print("SYNC-1: 全ノードが T の瞬間に D10 に 2 ms パルスを出します。10ch をオシロで観測。")
    print("  オシロ: Single / 立ち上がり / 200 us/div 程度")
    ok = send_params(h, ids, lambda i: f"param {i} {MODE['sync']} 0 0 0 0 0 0 0 0 10 1000")
    if len(ok) < len(ids) and not a.force:
        return
    errs = []
    for k in range(a.reps):
        input(f"  [{k + 1}/{a.reps}] オシロを Single で待機させて Enter: ")
        trial, rows = arm(h, ok)
        bad = [i for i in ok if i not in rows or rows[i][3] != "1"]
        if bad:
            print_ready(ok, rows); print("  NG ノードあり。スキップ"); continue
        h.send("go 300")
        rep = h.collect("REPORT", 2.0, ids=ok, pred=lambda f: int(f[2]) == trial)
        e = {i: int(rep[i][10]) for i in rep}
        errs.append(e)
        print("    ノード側の開始誤差 [us]: " + " ".join(f"{i}:{e[i]}" for i in sorted(e)))
    print("\n  合格基準: オシロ上のピークトゥピーク ≤ 200 us。最悪値を記録して Methods へ。")


def cmd_drift1(h, a):
    ids = parse_nodes(a.nodes)
    r1 = {i: peek(h, i, 1) for i in ids}
    print(f"  1回目取得。{a.minutes} 分待機...", flush=True)
    t_end = time.time() + a.minutes * 60
    while time.time() < t_end:
        h.drain(min(10.0, max(0.1, t_end - time.time())))
    r2 = {i: peek(h, i, 1) for i in ids}
    print(f"\n  {'id':>3} {'ppm(1)':>9} {'ppm(2)':>9} {'delta':>8}")
    worst = 0.0
    for i in ids:
        if r1[i] and r2[i]:
            d = r2[i]["ppm"] - r1[i]["ppm"]
            worst = max(worst, abs(d))
            print(f"  {i:>3} {r1[i]['ppm']:>9.2f} {r2[i]['ppm']:>9.2f} {d:>8.2f}")
    print(f"\n  最悪変動 {worst:.2f} ppm  (合格基準 < 2 ppm)")


# ---------------------------------------------------------------- 較正
def polyfit(x, y, order):
    """最小二乗 (numpy 不要)。x はスケールして条件数を抑える。昇冪 [c0..c3] を返す"""
    s = max(abs(v) for v in x) or 1.0
    xs = [v / s for v in x]
    m = order + 1
    A = [[sum(xi ** (r + c) for xi in xs) for c in range(m)] for r in range(m)]
    b = [sum(yi * xi ** r for xi, yi in zip(xs, y)) for r in range(m)]
    for i in range(m):                                   # ガウス消去
        p = max(range(i, m), key=lambda r: abs(A[r][i]))
        A[i], A[p] = A[p], A[i]; b[i], b[p] = b[p], b[i]
        for r in range(i + 1, m):
            f = A[r][i] / A[i][i]
            for c in range(i, m):
                A[r][c] -= f * A[i][c]
            b[r] -= f * b[i]
    coef = [0.0] * m
    for i in reversed(range(m)):
        coef[i] = (b[i] - sum(A[i][c] * coef[c] for c in range(i + 1, m))) / A[i][i]
    coef = [c / s ** k for k, c in enumerate(coef)]
    return coef + [0.0] * (4 - m)


def polyval(c, x):
    return c[0] + x * (c[1] + x * (c[2] + x * c[3]))


def report_fit(X, Y, C, order):
    pred = [polyval(C, x) for x in X]
    res = [y - p for y, p in zip(Y, pred)]
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    lin = polyfit(X, Y, 1)
    span = max(Y) - min(Y)
    nonlin = 100 * max(abs(y - polyval(lin, x)) for x, y in zip(X, Y)) / span if span else 0
    print(f"\n  点数 {len(X)}   範囲 {min(Y):+.1f} .. {max(Y):+.1f} deg")
    if lin[1]:
        print(f"  感度 (1次)   : {lin[1]:.5f} deg/count  ({1 / abs(lin[1]):.1f} count/deg)")
    print(f"  非線形性     : {nonlin:.2f} % FS")
    print(f"  {order}次 RMS     : {rms:.3f} deg   最大残差 {max(abs(r) for r in res):.3f} deg")
    if lin[1] and 1 / abs(lin[1]) < 20:
        print("  ★ 20 count/deg 未満。磁石位置 / SS39ET の電源電圧を見直す")
    if nonlin > 3 and order < 3:
        print("  ★ 非線形性 > 3 %。--order 3 を推奨")
    print(f"\n  {'ref[deg]':>9} {'diff':>9} {'fit[deg]':>9} {'resid':>7}")
    for x, y, p in sorted(zip(X, Y, pred), key=lambda t: t[1]):
        print(f"  {y:>9.2f} {x:>9.2f} {p:>9.2f} {y - p:>7.3f}")
    return pred, res


def push_cal(h, node, C):
    h.send(f"cal {node} {C[0]:.9g} {C[1]:.9g} {C[2]:.9g} {C[3]:.9g}")
    f = h.wait("CALACK", 2.0, pred=lambda f: int(f[1]) == node)
    if f is None:
        print("  ★ CALACK が返りません"); return False
    print(f"  ← ノード {node} が NVS に保存: {f[2:6]}")
    return True


def cmd_calib(h, a):
    angles = [-a.range + k * a.step for k in range(int(round(2 * a.range / a.step)) + 1)]
    print(f"較正: ノード {a.node}  角度 {angles[0]:+.0f} .. {angles[-1]:+.0f} deg ({a.step:g} deg 刻み)")
    print("  各角度に関節を固定して Enter。実際の角度が違うときは数値を入力。")
    print("  s = スキップ, q = ここで終了してフィット\n")
    d = peek(h, a.node, 8)
    if d is None:
        print("  ★ ノードが応答しません"); return
    X, Y = [], []
    for ang in angles:
        r = input(f"  {ang:+6.1f} deg に固定 → Enter: ").strip().lower()
        if r == "q":
            break
        if r == "s":
            continue
        ref = float(r) if r else ang
        d = peek(h, a.node, a.n)
        if d is None:
            print("    応答なし。もう一度"); continue
        X.append(d["diff"]); Y.append(ref)
        print(f"    diff = {d['diff']:.2f} ± {d['diff_sd']:.2f}  (v1 {d['v1']:.0f}, v2 {d['v2']:.0f})")
    order = min(a.order, len(X) - 2)
    if order < 1:
        print("  点が足りません"); return
    C = polyfit(X, Y, order)
    pred, res = report_fit(X, Y, C, order)
    print(f"\n  CAL = {C[0]:.6g} {C[1]:.6g} {C[2]:.6g} {C[3]:.6g}")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_csv([[x, y, p, r] for x, y, p, r in zip(X, Y, pred, res)],
             f"calib_{stamp}_node{a.node}.csv", ["diff", "ref_deg", "fit_deg", "resid_deg"])
    if a.push or input("\n  ノードに書き込みますか？ [y/N]: ").strip().lower() == "y":
        if push_cal(h, a.node, C):
            d = peek(h, a.node, 64)
            if d:
                print(f"  確認: 今の角度 = {d['deg']:.2f} deg")


def cmd_fit(a):
    """HALL-1 の CSV と基準角から較正（オフライン）。--mode steps / track"""
    def load(path, cols):
        with open(path) as f:
            return [[float(r[c]) for c in cols] for r in csv.DictReader(f)]
    hall = load(a.hall, ["t_s", "diff"])
    ref = load(a.ref, ["t_s", "angle_deg"])
    X, Y = [], []
    if a.mode == "steps":
        for tr, ang in ref:
            w = sorted(d for t, d in hall if abs(t - tr) < a.window / 2)
            if len(w) < 3:
                print(f"  警告: t={tr:.2f}s 付近のサンプル {len(w)} 点"); continue
            X.append(w[len(w) // 2]); Y.append(ang)
    else:
        rt = [r[0] for r in ref]
        for t, d in hall:
            if rt[0] <= t <= rt[-1]:
                k = max(1, next(i for i, v in enumerate(rt) if v >= t))
                t0, a0 = ref[k - 1]; t1, a1 = ref[k]
                X.append(d); Y.append(a0 + (a1 - a0) * (t - t0) / ((t1 - t0) or 1))
    order = min(a.order, len(X) - 2)
    if order < 1:
        sys.exit("点が足りません")
    C = polyfit(X, Y, order)
    report_fit(X, Y, C, order)
    print(f"\n  python pc/unagi.py pushcal --node {a.node} --coef {C[0]:.6g} {C[1]:.6g} {C[2]:.6g} {C[3]:.6g}")


def cmd_pushcal(h, a):
    push_cal(h, a.node, a.coef)


def cmd_setzero(h, a):
    input(f"  ノード {a.node} の関節を 0 deg に合わせて Enter: ")
    h.send(f"setzero {a.node}")
    f = h.wait("CALACK", 2.0, pred=lambda f: int(f[1]) == a.node)
    print(f"  ← {f}" if f else "  ★ 応答なし")


def cmd_minduty(h, a):
    h.send(f"minduty {a.node} {a.value}")
    f = h.wait("CALACK", 2.0, pred=lambda f: int(f[1]) == a.node)
    print(f"  ← minduty = {f[6]}" if f else "  ★ 応答なし")


# ---------------------------------------------------------------- 走行系
def param_line(i, mode, A=0, f=0, phi=0, off=0, kp=0, ki=0, kd=0, aux=0, seconds=1.0, log_hz=200):
    return (f"param {i} {mode} {A:g} {f:g} {phi:g} {off:g} {kp:g} {ki:g} {kd:g} {aux:g} "
            f"{int(seconds * 1000)} {log_hz}")


def cmd_hall1(h, a):
    print(f"HALL-1: ノード {a.node}。モータは駆動しません。記録中に関節を動かしてください。")
    hz = fit_log_hz(a.seconds, a.log_hz)
    input("  Enter で開始: ")
    run_trial(h, [a.node], lambda i: param_line(i, MODE["hall_raw"], seconds=a.seconds, log_hz=hz),
              a.seconds, "hall1", a.force)


def cmd_hall2(h, a):
    print(f"HALL-2: ノード {a.node}。★ 関節を治具で固定してから。PWM を 0→+{a.duty}→−{a.duty}→0 で掃引")
    hz = fit_log_hz(a.seconds, a.log_hz)
    input("  固定できたら Enter: ")
    out = run_trial(h, [a.node], lambda i: param_line(i, MODE["sweep"], aux=a.duty, seconds=a.seconds, log_hz=hz),
                    a.seconds, "hall2", a.force)
    if out and a.node in out and out[a.node]:
        deg = [r[2] for r in out[a.node]]
        print(f"\n  見かけの角度変動 p-p = {max(deg) - min(deg):.2f} deg（固定しているので理想は 0）")


def cmd_openloop(h, a):
    ids = parse_nodes(a.nodes)
    hz = fit_log_hz(a.seconds, a.log_hz)
    print(f"OPENLOOP: nodes {ids}  u = {a.amp} sin(2π·{a.freq}t − φ)  dphi={a.dphi} deg/joint")
    run_trial(h, ids, lambda i: param_line(i, MODE["openloop"], A=a.amp, f=a.freq, phi=a.dphi * (i - 1),
                                           seconds=a.seconds, log_hz=hz), a.seconds, "openloop", a.force)


def cmd_joint1(h, a):
    hz = fit_log_hz(a.seconds, a.log_hz)
    print(f"JOINT-1: ノード {a.node} ステップ {a.step:+.1f} deg  kp={a.kp} ki={a.ki} kd={a.kd}")
    run_trial(h, [a.node], lambda i: param_line(i, MODE["step"], aux=a.step, kp=a.kp, ki=a.ki, kd=a.kd,
                                                seconds=a.seconds, log_hz=hz), a.seconds, "joint1", a.force)


def cmd_swim(h, a):
    ids = parse_nodes(a.nodes)
    hz = fit_log_hz(a.seconds, a.log_hz)
    print(f"SWIM: f={a.freq} Hz  A={a.amp} deg  dphi={a.dphi} deg/joint  nodes={ids}")
    run_trial(h, ids, lambda i: param_line(i, MODE["swim"], A=a.amp, f=a.freq, phi=a.dphi * (i - 1),
                                           kp=a.kp, ki=a.ki, kd=a.kd, seconds=a.seconds, log_hz=hz),
              a.seconds, f"swim_f{a.freq}_a{a.amp}", a.force)


def cmd_halt(h, a):
    h.send("halt"); print(h.wait("OK,halt", 1.0))


def cmd_raw(h, a):
    h.send(a.cmd)
    for ln in h.lines(a.seconds):
        print(" ", ln)


# =====================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=HOST)
    ap.add_argument("-v", "--verbose", action="store_true", help="送受信をすべて表示")
    sp = ap.add_subparsers(dest="cmd", required=True)

    def P(name, fn, **kw):
        p = sp.add_parser(name, **kw); p.set_defaults(fn=fn); return p

    P("check", cmd_check)
    P("nodes", cmd_nodes)
    p = P("power", cmd_power); p.add_argument("state", choices=["on", "off"])
    p = P("peek", cmd_peek); p.add_argument("--node", type=int, required=True)
    p.add_argument("--n", type=int, default=64); p.add_argument("--watch", action="store_true")
    p = P("sync", cmd_sync); p.add_argument("--nodes", default="all")
    p = P("sync1", cmd_sync1); p.add_argument("--reps", type=int, default=20)
    p.add_argument("--nodes", default="all"); p.add_argument("--force", action="store_true")
    p = P("drift1", cmd_drift1); p.add_argument("--minutes", type=float, default=10)
    p.add_argument("--nodes", default="all")

    p = P("calib", cmd_calib); p.add_argument("--node", type=int, required=True)
    p.add_argument("--range", type=float, default=30); p.add_argument("--step", type=float, default=5)
    p.add_argument("--order", type=int, default=3, choices=[1, 2, 3])
    p.add_argument("--n", type=int, default=200, help="1点あたりの平均サンプル数")
    p.add_argument("--push", action="store_true", help="確認なしで書き込む")
    p = P("pushcal", cmd_pushcal); p.add_argument("--node", type=int, required=True)
    p.add_argument("--coef", type=float, nargs=4, required=True)
    p = P("setzero", cmd_setzero); p.add_argument("--node", type=int, required=True)
    p = P("minduty", cmd_minduty); p.add_argument("--node", type=int, required=True)
    p.add_argument("--value", type=float, required=True)
    p = sp.add_parser("fit"); p.set_defaults(fn=None)
    p.add_argument("--hall", required=True); p.add_argument("--ref", required=True)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--order", type=int, default=3, choices=[1, 2, 3])
    p.add_argument("--mode", choices=["steps", "track"], default="steps")
    p.add_argument("--window", type=float, default=0.4)

    def run_args(p, seconds, log_hz):
        p.add_argument("--seconds", type=float, default=seconds)
        p.add_argument("--log-hz", dest="log_hz", type=int, default=log_hz)
        p.add_argument("--force", action="store_true", help="READY でないノードを除いて強行")

    p = P("hall1", cmd_hall1); p.add_argument("--node", type=int, required=True); run_args(p, 60, 50)
    p = P("hall2", cmd_hall2); p.add_argument("--node", type=int, required=True)
    p.add_argument("--duty", type=float, default=0.6); run_args(p, 4, 1000)
    p = P("openloop", cmd_openloop); p.add_argument("--nodes", default="all")
    p.add_argument("--amp", type=float, default=0.4); p.add_argument("--freq", type=float, default=1.0)
    p.add_argument("--dphi", type=float, default=36.0); run_args(p, 5, 200)
    for name, fn, secs, hz in (("joint1", cmd_joint1, 2, 1000), ("swim", cmd_swim, 10, 200)):
        p = P(name, fn)
        p.add_argument("--kp", type=float, default=0.03)
        p.add_argument("--ki", type=float, default=0.0)
        p.add_argument("--kd", type=float, default=0.0005)
        run_args(p, secs, hz)
        if name == "joint1":
            p.add_argument("--node", type=int, required=True); p.add_argument("--step", type=float, default=15)
        else:
            p.add_argument("--nodes", default="all"); p.add_argument("--freq", type=float, default=2.0)
            p.add_argument("--amp", type=float, default=20.0); p.add_argument("--dphi", type=float, default=36.0)
    P("halt", cmd_halt)
    p = P("raw", cmd_raw); p.add_argument("cmd"); p.add_argument("--seconds", type=float, default=2.0)

    a = ap.parse_args()
    if a.cmd == "fit":
        cmd_fit(a); return
    h = Head(a.host, verbose=a.verbose)
    try:
        a.fn(h, a)
    except KeyboardInterrupt:
        print("\n  中断 → halt")
        h.send("halt"); h.drain(0.3)
    finally:
        h.close()


if __name__ == "__main__":
    main()
