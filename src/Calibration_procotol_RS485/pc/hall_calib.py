#!/usr/bin/env python3
"""
hall_calib.py — SS39ET 差動出力から関節角への較正

前提:
  HALL-1 で hall1_nodeN.csv (t_s, v1, v2, diff, deg_uncal) を取得済み。
  同時に基準角度を取ること。方法は2通り:

  (A) ステップ較正 (推奨・簡単)
      -30° から +30° まで 5° ごとにゴニオメータで固定し、各点で 1 秒静止。
      ref_steps.csv に  t_s,angle_deg  を書く（各静止区間の中央時刻）。

  (B) 連続較正 (カメラ併用)
      AprilTag から求めた角度時系列を ref_track.csv に t_s,angle_deg で保存。

使い方:
    python hall_calib.py --hall data/hall1_node1.csv --ref data/ref_steps.csv --node 1
    python hall_calib.py --hall data/hall1_node1.csv --ref data/ref_track.csv --node 1 --order 3
"""
import argparse, csv, sys
import numpy as np


def load(path, cols):
    with open(path) as f:
        r = csv.DictReader(f)
        rows = [[float(row[c]) for c in cols] for row in r]
    return np.array(rows)


def match_steps(hall, ref, window=0.4):
    """基準点の時刻まわり ±window/2 秒の diff を中央値でとる"""
    t, d = hall[:, 0], hall[:, 3]
    X, Y = [], []
    for tr, ang in ref:
        m = np.abs(t - tr) < window / 2
        if m.sum() < 3:
            print(f"  警告: t={tr:.2f}s 付近にサンプルが {m.sum()} 点しかない")
            continue
        X.append(np.median(d[m]))
        Y.append(ang)
    return np.array(X), np.array(Y)


def match_track(hall, ref):
    """基準時系列をホール時刻に線形補間"""
    t, d = hall[:, 0], hall[:, 3]
    ang = np.interp(t, ref[:, 0], ref[:, 1], left=np.nan, right=np.nan)
    m = ~np.isnan(ang)
    return d[m], ang[m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hall", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--node", type=int, required=True)
    ap.add_argument("--order", type=int, default=3, choices=[1, 2, 3])
    ap.add_argument("--mode", choices=["steps", "track"], default="steps")
    a = ap.parse_args()

    hall = load(a.hall, ["t_s", "v1", "v2", "diff"])
    ref = load(a.ref, ["t_s", "angle_deg"])

    X, Y = match_steps(hall, ref) if a.mode == "steps" else match_track(hall, ref)
    if len(X) < a.order + 2:
        sys.exit(f"点が足りません ({len(X)} 点)")

    # theta = C0 + C1*d + C2*d^2 + C3*d^3
    coef = np.polyfit(X, Y, a.order)[::-1]          # 昇冪に並べ替え
    C = np.zeros(4); C[:len(coef)] = coef

    pred = np.polyval(C[::-1], X)
    res = Y - pred
    rms = float(np.sqrt(np.mean(res ** 2)))
    span = float(Y.max() - Y.min())

    # 1次だけで当てた場合と比較して非線形性の寄与を見る
    lin = np.polyfit(X, Y, 1)
    res_lin = Y - np.polyval(lin, X)
    nonlin_pct = 100.0 * np.max(np.abs(res_lin)) / span

    print(f"\nノード {a.node}   点数 {len(X)}   範囲 {Y.min():+.1f} .. {Y.max():+.1f} deg")
    print(f"  感度 (1次)     : {lin[0]:.5f} deg/count  ({1/lin[0]:.2f} count/deg)")
    print(f"  非線形性       : {nonlin_pct:.2f} % FS")
    print(f"  {a.order} 次フィット RMS : {rms:.3f} deg")
    print(f"  最大残差       : {np.max(np.abs(res)):.3f} deg")

    if 1 / abs(lin[0]) < 20:
        print("\n  ★ 分解能が 20 count/deg 未満です。")
        print("    磁石を近づける、SS39ET の電源電圧を上げる（比例出力なので感度が上がる）、")
        print("    または差動増幅を挟むことを検討してください。")
    if nonlin_pct > 3:
        print("\n  ★ 非線形性が 3 % を超えています。--order 3 を使うか、")
        print("    磁石とセンサの配置を見直してください。")
    if rms > 0.5:
        print("\n  ★ RMS が 0.5 deg 超。REC-1 の合格基準 (2 deg) に対して余裕がありません。")

    print(f"\nnode.ino に貼り付け:")
    print(f"float CAL[4] = {{{C[0]:.6g}f, {C[1]:.6g}f, {C[2]:.6g}f, {C[3]:.6g}f}};")

    # 残差テーブル
    print(f"\n  {'ref[deg]':>9} {'diff':>8} {'fit[deg]':>9} {'resid':>8}")
    idx = np.argsort(Y)
    for i in idx[:: max(1, len(idx) // 20)]:
        print(f"  {Y[i]:>9.2f} {X[i]:>8.1f} {pred[i]:>9.2f} {res[i]:>8.3f}")

    np.savetxt(f"data/cal_node{a.node}.csv",
               np.column_stack([X, Y, pred, res]),
               delimiter=",", header="diff,ref_deg,fit_deg,resid_deg", comments="")
    print(f"\n  → data/cal_node{a.node}.csv")


if __name__ == "__main__":
    main()
