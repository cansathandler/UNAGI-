#!/usr/bin/env python3
"""
unagi_test.py — .md の受入試験を実行する PC 側スクリプト

使い方:
    python unagi_test.py mac
    python unagi_test.py sync
    python unagi_test.py sync1     --reps 20      # SYNC-1 (要オシロ)
    python unagi_test.py drift1    --minutes 10   # DRIFT-1
    python unagi_test.py hall1     --node 1       # HALL-1 較正データ取得
    python unagi_test.py hall2     --node 1       # HALL-2 PWM クロストーク
    python unagi_test.py joint1    --node 1 --step 15
    python unagi_test.py swim      --freq 2.0 --amp 20
"""
import argparse, socket, time, csv, sys, statistics
from pathlib import Path

HOST, PORT = "192.168.4.1", 5555   # ヘッド v31: SSID "UNAGI-HEAD", port 5555
N_NODES = 10
OUT = Path("data"); OUT.mkdir(exist_ok=True)


class Head:
    def __init__(self, host=HOST, port=PORT, timeout=5.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.buf = b""
        # ヘッドの JSON テレメトリ (20 Hz) を止めてから使う。
        # GUI 側で再開したいときは "START" を送る。
        self.s.sendall(b"STOP\n")
        time.sleep(0.3)
        try:
            self.s.recv(65536)
        except socket.timeout:
            pass

    def send(self, cmd):
        self.s.sendall((cmd + "\n").encode())

    def lines(self, seconds):
        """seconds 秒間に届いた行を yield する"""
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.s.settimeout(max(0.05, t_end - time.time()))
            try:
                d = self.s.recv(8192)
            except socket.timeout:
                continue
            if not d:
                break
            self.buf += d
            while b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                yield ln.decode(errors="replace").strip()

    def collect(self, prefix, seconds, want=None):
        out = []
        for ln in self.lines(seconds):
            if ln.startswith(prefix):
                out.append(ln.split(","))
                if want and len(out) >= want:
                    break
        return out

    def close(self):
        self.s.close()


# ---------------------------------------------------------------- 同期
def do_sync(h, n_points=300, verbose=True):
    """同期フェーズを走らせ、全ノードの ready 状態を返す"""
    # ノード電源がヘッド制御なら投入して起動を待つ
    h.send("power 1")
    boot_ms = 0
    for ln in h.lines(1.5):
        if ln.startswith("OK,power"):
            p = ln.split(",")
            if p[2] != "disabled":
                boot_ms = int(p[3])
            break
    if boot_ms:
        if verbose:
            print(f"  ノード電源 ON、起動待ち {boot_ms/1000:.1f} 秒...", flush=True)
        time.sleep(boot_ms / 1000.0)

    h.send(f"sync {n_points}")
    wait = n_points / 10.0 + 3.0
    if verbose:
        print(f"  ビーコン {n_points} 点 ({wait:.0f} 秒)...", flush=True)
    list(h.lines(wait))
    h.send("beacon_off")
    h.send("arm")
    rows = h.collect("READY", 3.0, want=N_NODES)
    res = {}
    for r in rows:
        # READY,id,ready,sigma_us,ppm,crc,ram,b1,b2
        res[int(r[1])] = dict(ready=int(r[2]), sigma=float(r[3]), ppm=float(r[4]),
                              crc=int(r[5]), ram=int(r[6]),
                              bias1=int(r[7]), bias2=int(r[8]))
    return res


def print_ready(res):
    print(f"  {'id':>3} {'ready':>6} {'sigma[us]':>10} {'skew[ppm]':>10} {'RAM[kB]':>9}  bias1/2")
    bad = []
    for i in range(1, N_NODES + 1):
        r = res.get(i)
        if r is None:
            print(f"  {i:>3}   ---  (応答なし)")
            bad.append(i); continue
        flag = "" if (r["ready"] and r["sigma"] < 500) else "  ← NG"
        print(f"  {i:>3} {r['ready']:>6} {r['sigma']:>10.1f} {r['ppm']:>10.2f} "
              f"{r['ram']//1024:>9} {r['bias1']:>6}/{r['bias2']:<6}{flag}")
        if not r["ready"] or r["sigma"] >= 500:
            bad.append(i)
    return bad


# ---------------------------------------------------------------- RF-0
def rf0(h, minutes=5):
    """ビーコン受信率。ノードは READY 時に受信点数を返すので、
    要求点数と実際に集まった点数の比で見る。"""
    print("RF-0: ブロードキャスト受信率")
    n_points = int(minutes * 60 * 10)
    res = do_sync(h, n_points)
    print_ready(res)
    print("\n  sigma が極端に大きいノードは取りこぼしが多い。")
    print("  受信率そのものはノード側 Serial の n_sync/want_sync を確認すること。")


# ---------------------------------------------------------------- SYNC-1
def sync1(h, reps=20):
    """全ノードが t_start で GPIO をトグルする。オシロで立ち上がりのばらつきを測る。"""
    print("SYNC-1: 位相同期精度  (10ch をオシロで観測してください)")
    res = do_sync(h)
    bad = print_ready(res)
    if bad:
        print(f"\n  NG ノード {bad} があります。中止。")
        return
    print(f"\n  {reps} 回トリガします。オシロは Single / 立ち上がり / 500us/div 推奨。")
    input("  準備ができたら Enter: ")
    for k in range(reps):
        for i in range(1, N_NODES + 1):
            # mode=1 (MODE_GPIO_SYNC), duration/log は使わない
            h.send(f"param {i} 0 0 0 0 0 0 0 100 200 1 0")
        time.sleep(0.3)
        h.send("arm"); list(h.lines(1.0))
        h.send("start 200")
        list(h.lines(0.6))
        print(f"    trigger {k+1}/{reps}")
        time.sleep(1.0)
    print("\n  合格基準: ピークトゥピーク 200 us 以下")
    print("  実測値を手で記録し、最悪値を Methods に記載してください。")


# ---------------------------------------------------------------- DRIFT-1
def drift1(h, minutes=10):
    """同じ同期を時間を置いて2回行い、クロック比 b の安定性を見る。"""
    print(f"DRIFT-1: クロック比の安定性 ({minutes} 分)")
    r1 = do_sync(h); print_ready(r1)
    print(f"\n  {minutes} 分待機...")
    time.sleep(minutes * 60)
    r2 = do_sync(h); print_ready(r2)
    print(f"\n  {'id':>3} {'ppm(1)':>9} {'ppm(2)':>9} {'delta':>9}")
    worst = 0.0
    for i in range(1, N_NODES + 1):
        if i in r1 and i in r2:
            d = r2[i]["ppm"] - r1[i]["ppm"]
            worst = max(worst, abs(d))
            print(f"  {i:>3} {r1[i]['ppm']:>9.2f} {r2[i]['ppm']:>9.2f} {d:>9.2f}")
    print(f"\n  最悪変動 {worst:.2f} ppm   (合格基準 < 2 ppm)")


# ---------------------------------------------------------------- ダンプ
def dump(h, node, n_expected, timeout=30.0):
    """1ノードのログを全ブロック回収して list[list[int]] で返す"""
    n_blocks = (n_expected + 27) // 28
    got = {}
    for attempt in range(3):
        for b in range(n_blocks):
            if b in got:
                continue
            h.send(f"dump {node} {b}")
            time.sleep(0.012)
            for ln in h.lines(0.05):
                if not ln.startswith("DUMP"):
                    continue
                p = ln.split(",")
                bi, n = int(p[2]), int(p[3])
                vals = [int(x) for x in p[4:4 + n * 4]]
                got[bi] = [vals[k*4:(k+1)*4] for k in range(n)]
        if len(got) >= n_blocks:
            break
        print(f"    再送 {n_blocks - len(got)} ブロック")
    rows = []
    for b in sorted(got):
        rows.extend(got[b])
    return rows


def save(rows, path, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)
    print(f"  → {path}  ({len(rows)} rows)")


# ---------------------------------------------------------------- HALL-1
def hall1(h, node=1, seconds=8, log_hz=200):
    """モータ停止のままホール生値を記録。ゴニオメータ等で関節角を手で振ること。"""
    print(f"HALL-1: ノード {node} のホール較正データ取得")
    print("  モータは駆動しません。記録中に関節を -30° → +30° までゆっくり動かし、")
    print("  基準角度計の読みを別途メモしてください。")
    do_sync(h)
    dur = int(seconds * 1000)
    h.send(f"param {node} 0 0 0 0 0 0 0 {dur} {log_hz} 2 0")
    time.sleep(0.3)
    h.send("arm"); list(h.lines(1.0))
    input("  Enter で記録開始（合図後すぐ関節を動かす）: ")
    h.send("start 300")
    for k in range(seconds):
        print(f"    {k+1}/{seconds} s", flush=True); time.sleep(1.0)
    rep = h.collect("REPORT", 3.0, want=1)
    n = int(rep[0][2]) if rep else seconds * log_hz
    rows = dump(h, node, n)
    # MODE_HALL_RAW: [0]=cmd(0), [1]=deg*100, [2]=v2, [3]=v1
    out = [[i / log_hz, r[3], r[2], r[3] - r[2], r[1] / 100.0] for i, r in enumerate(rows)]
    save(out, OUT / f"hall1_node{node}.csv", ["t_s", "v1", "v2", "diff", "deg_uncal"])
    print("\n  次: 基準角度と diff の対応を hall_calib.py に渡して多項式を求める。")


# ---------------------------------------------------------------- HALL-2
def hall2(h, node=1, seconds=5, max_duty=0.8, log_hz=1000):
    """関節を機械的に固定して PWM を掃引。見かけの角度変化＝クロストーク。"""
    print(f"HALL-2: ノード {node} の PWM クロストーク")
    print("  ★ 関節を治具で機械的に固定してから実行してください。")
    input("  固定できたら Enter: ")
    do_sync(h)
    dur = int(seconds * 1000)
    h.send(f"param {node} 0 0 0 0 0 0 0 {dur} {log_hz} 3 {max_duty}")
    time.sleep(0.3)
    h.send("arm"); list(h.lines(1.0))
    h.send("start 300")
    time.sleep(seconds + 1.0)
    rep = h.collect("REPORT", 3.0, want=1)
    n = int(rep[0][2]) if rep else seconds * log_hz
    rows = dump(h, node, n)
    out = [[i / log_hz, r[2] / 10000.0, r[1] / 100.0, r[3]] for i, r in enumerate(rows)]
    save(out, OUT / f"hall2_node{node}.csv", ["t_s", "duty", "deg", "common"])
    degs = [r[2] for r in out]
    if degs:
        print(f"\n  見かけの角度変動: {min(degs):+.2f} .. {max(degs):+.2f} deg "
              f"(p-p {max(degs)-min(degs):.2f})")
        print("  合格基準: p-p が測定分解能の 2 倍以内。")
        print("  超える場合は差動配線のツイスト、ADC サンプルの PWM 同期を見直す。")


# ---------------------------------------------------------------- JOINT-1
def joint1(h, node=1, step=15.0, kp=0.03, ki=0.0, kd=0.0005, seconds=2, log_hz=1000):
    print(f"JOINT-1: ノード {node} ステップ応答 {step:+.1f} deg")
    do_sync(h)
    dur = int(seconds * 1000)
    h.send(f"param {node} 0 0 0 0 {kp} {ki} {kd} {dur} {log_hz} 4 {step}")
    time.sleep(0.3)
    h.send("arm"); list(h.lines(1.0))
    h.send("start 300")
    time.sleep(seconds + 1.0)
    rep = h.collect("REPORT", 3.0, want=1)
    if rep:
        r = rep[0]
        print(f"  n={r[2]} missed={r[4]} max_jitter={r[5]}us "
              f"rms_err={r[6]}deg adc_max={r[7]}us")
        if int(r[4]) > 0:
            print("  ★ missed > 0。OVERSAMPLE を下げるか log_hz を下げること。")
        if float(r[7]) > 400:
            print("  ★ ADC が 400us 超。1ms 予算を圧迫している。")
    n = int(rep[0][2]) if rep else seconds * log_hz
    rows = dump(h, node, n)
    out = [[i / log_hz, r[0] / 100.0, r[1] / 100.0, r[2] / 10000.0] for i, r in enumerate(rows)]
    save(out, OUT / f"joint1_node{node}_{step:+.0f}.csv", ["t_s", "cmd_deg", "meas_deg", "u"])


# ---------------------------------------------------------------- 遊泳
def swim(h, freq=2.0, amp=20.0, dphi=36.0, seconds=10,
         kp=0.03, ki=0.0, kd=0.0005, log_hz=200):
    print(f"SWIM: f={freq} Hz  A={amp} deg  dphi={dphi} deg/joint")
    res = do_sync(h)
    bad = print_ready(res)
    if bad:
        print(f"\n  NG ノード {bad}。中止。")
        return
    dur = int(seconds * 1000)
    for i in range(1, N_NODES + 1):
        phi = dphi * (i - 1)
        h.send(f"param {i} {amp} {freq} {phi} 0 {kp} {ki} {kd} {dur} {log_hz} 0 0")
        time.sleep(0.02)
    time.sleep(0.5)
    h.send("arm")
    rows = h.collect("READY", 3.0, want=N_NODES)
    if len(rows) < N_NODES:
        print(f"  READY が {len(rows)}/{N_NODES}。中止。")
        return
    h.send("start 300")
    time.sleep(seconds + 1.5)

    reports = h.collect("REPORT", 5.0, want=N_NODES)
    print(f"\n  {'id':>3} {'n':>6} {'missed':>7} {'jit[us]':>8} {'rms[deg]':>9} {'adc[us]':>8}")
    ok = True
    for r in sorted(reports, key=lambda x: int(x[1])):
        print(f"  {int(r[1]):>3} {int(r[2]):>6} {int(r[4]):>7} {int(r[5]):>8} "
              f"{float(r[6]):>9.3f} {float(r[7]):>8.0f}")
        if int(r[4]) > 0:
            ok = False
    if len(reports) < N_NODES:
        print(f"  ★ REPORT が {len(reports)}/{N_NODES}。この試行は除外。")
        ok = False
    if not ok:
        print("\n  ★ 除外基準に該当。解析に使わないこと。")

    print("\n  ログ回収中...")
    for i in range(1, N_NODES + 1):
        rep = [r for r in reports if int(r[1]) == i]
        if not rep:
            continue
        n = int(rep[0][2])
        rows = dump(h, i, n)
        out = [[k / log_hz, r[0] / 100.0, r[1] / 100.0, r[2] / 10000.0, r[3]]
               for k, r in enumerate(rows)]
        save(out, OUT / f"swim_f{freq}_a{amp}_node{i}.csv",
             ["t_s", "cmd_deg", "meas_deg", "u", "common"])


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["ping", "mac", "sync", "rf0", "sync1", "drift1",
                                     "hall1", "hall2", "joint1", "swim"])
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--reps", type=int, default=20)
    ap.add_argument("--minutes", type=int, default=10)
    ap.add_argument("--step", type=float, default=15.0)
    ap.add_argument("--freq", type=float, default=2.0)
    ap.add_argument("--amp", type=float, default=20.0)
    ap.add_argument("--dphi", type=float, default=36.0)
    ap.add_argument("--seconds", type=int, default=10)
    ap.add_argument("--host", default=HOST)
    a = ap.parse_args()

    h = Head(a.host)
    try:
        if a.test == "ping":
            # まずヘッドのファームが新しいか確認する
            h.send("ver")
            ver = None
            for ln in h.lines(2.0):
                print(" ", ln)
                if ln.startswith("OK,ver"):
                    ver = ln
            if ver is None:
                print("\n  ★ ヘッドが 'ver' に応答しません。古いファームが載っています。")
                print("     PROJECT TASKS → head → Upload で書き直してください。")
                print("     （矢印ボタンは default_envs=node なので node しか書きません）")
                return
            print("\nブロードキャストを1発送ります。")
            h.send("power 1")
            list(h.lines(3.0))
            h.send("ping")
            got = False
            for ln in h.lines(4.0):
                print(" ", ln)
                if ln.startswith("HELLO") or ln.startswith("MAC"):
                    got = True
            if not got:
                print("\n  ノードからの応答なし。確認事項:")
                print("   1) ノードのファームも書き直したか")
                print("      → HELLO への応答は今回追加した機能です")
                print("   2) ノードに電源が来ているか（USB給電で試す）")
                print("   3) ヘッドとノードの UNAGI_CHANNEL が一致しているか")
        elif a.test == "mac":
            h.send("mac")
            for ln in h.lines(3.0):
                print(" ", ln)
        elif a.test == "sync":
            print_ready(do_sync(h))
        elif a.test == "rf0":
            rf0(h, a.minutes)
        elif a.test == "sync1":
            sync1(h, a.reps)
        elif a.test == "drift1":
            drift1(h, a.minutes)
        elif a.test == "hall1":
            hall1(h, a.node)
        elif a.test == "hall2":
            hall2(h, a.node)
        elif a.test == "joint1":
            joint1(h, a.node, a.step)
        elif a.test == "swim":
            swim(h, a.freq, a.amp, a.dphi, a.seconds)
    finally:
        h.close()


if __name__ == "__main__":
    main()
