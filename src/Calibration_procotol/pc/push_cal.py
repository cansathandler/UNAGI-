#!/usr/bin/env python3
"""
push_cal.py — hall_calib.py が出した較正係数を ESP-NOW 経由でノードの NVS に書き込む

再フラッシュは不要です。ノード側は受信すると即 NVS に保存して CALACK を返します。

使い方:
    python push_cal.py --node 3 --coef 0.12 0.02134 -1.2e-6 3.4e-10
    python push_cal.py --node 3 --from data/cal_node3_coef.txt
    python push_cal.py --verify --node 3
"""
import argparse, socket, time, sys

HOST, PORT = "192.168.4.1", 5555


def connect(host=HOST):
    s = socket.create_connection((host, PORT), timeout=5.0)
    s.settimeout(3.0)
    return s


def lines(s, seconds):
    buf = b""
    t_end = time.time() + seconds
    while time.time() < t_end:
        s.settimeout(max(0.05, t_end - time.time()))
        try:
            d = s.recv(4096)
        except socket.timeout:
            continue
        if not d:
            break
        buf += d
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            yield ln.decode(errors="replace").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", type=int, required=True)
    ap.add_argument("--coef", type=float, nargs=4)
    ap.add_argument("--from", dest="src")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--host", default=HOST)
    a = ap.parse_args()

    if a.verify:
        c = [0, 0, 0, 0]
    elif a.coef:
        c = a.coef
    elif a.src:
        c = [float(x) for x in open(a.src).read().split()][:4]
    else:
        sys.exit("--coef か --from を指定してください")

    s = connect(a.host)
    if not a.verify:
        cmd = f"cal {a.node} {c[0]:.9g} {c[1]:.9g} {c[2]:.9g} {c[3]:.9g}"
        print(f"→ {cmd}")
        s.sendall((cmd + "\n").encode())

        ok = False
        for ln in lines(s, 3.0):
            if ln.startswith("CALACK"):
                p = ln.split(",")
                if int(p[1]) == a.node:
                    got = [float(x) for x in p[2:6]]
                    print(f"← ノード {a.node} が NVS に保存: {got}")
                    dev = max(abs(g - w) / (abs(w) + 1e-12) for g, w in zip(got, c))
                    if dev > 1e-4:
                        print(f"  ★ 送った値と差があります (相対 {dev:.2e})。"
                              f"float32 の丸めを超えていれば要確認。")
                    ok = True
                    break
        if not ok:
            print("  ★ CALACK が返りません。ノードの ID 設定と電源を確認してください。")
    else:
        print("ノードのシリアルで 'show' を実行して CAL を確認してください。")
    s.close()


if __name__ == "__main__":
    main()
