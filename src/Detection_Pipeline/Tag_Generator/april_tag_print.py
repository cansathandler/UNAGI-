"""
Print-ready AprilTags (tag36h11) at an exact physical size.

The black square of each tag is exactly `size_mm` wide. A white quiet zone (2 cells)
is drawn around it, and a dashed cut line around that. Output:
  - an A4 PDF  (print at 100 % / "Actual size", NOT "Fit to page")
  - one PNG per tag with the DPI stored in the file (for Word/Inkscape import at true size)

Usage:
    python april_tag_print.py                       # ids 0 1, 15 mm
    python april_tag_print.py --ids 0 1 --size 15
    python april_tag_print.py --ids 0 1 2 3 --size 12 --copies 2
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

CELLS = 8          # tag36h11 in OpenCV: 6x6 data + 1-cell black border each side
QUIET_CELLS = 2    # white margin around the black square, in cells
A4_MM = (210.0, 297.0)


def tag_bitmap(tag_id, px_per_cell=40):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(d, int(tag_id), CELLS * px_per_cell)
    q = QUIET_CELLS * px_per_cell
    return cv2.copyMakeBorder(img, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def save_png(tag_id, size_mm, out_dir):
    """PNG whose DPI makes the black square exactly size_mm when placed at 100 %."""
    ppc = 40
    img = tag_bitmap(tag_id, ppc)
    dpi = CELLS * ppc / (size_mm / 25.4)
    p = out_dir / f"tag36h11_id{tag_id}_{size_mm:g}mm.png"
    Image.fromarray(img).save(p, dpi=(dpi, dpi))
    return p


def save_pdf(ids, size_mm, copies, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure
    from matplotlib.patches import Rectangle

    W, H = A4_MM
    fig = Figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")

    cell = size_mm / CELLS
    outer = size_mm + 2 * QUIET_CELLS * cell          # tag + quiet zone
    pitch_x, pitch_y = outer + 8, outer + 12
    cols = max(1, int((W - 30) // pitch_x))
    items = [i for i in ids for _ in range(copies)]
    for n, tid in enumerate(items):
        r, c = divmod(n, cols)
        x0, y0 = 15 + c * pitch_x, 25 + r * pitch_y     # top-left of the quiet zone
        img = tag_bitmap(tid)
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="nearest",
                  extent=(x0, x0 + outer, y0 + outer, y0))
        ax.add_patch(Rectangle((x0, y0), outer, outer, fill=False, lw=0.3, ec="0.6", ls="--"))
        ax.text(x0, y0 + outer + 4, f"id {tid}  ({size_mm:g} mm)", fontsize=7)
    ax.text(15, 12, f"AprilTag tag36h11 - black square = {size_mm:g} mm. "
                    "Print at 100 % (Actual size). Cut on the dashed line.", fontsize=8)
    # scale check
    ax.plot([15, 65], [H - 15, H - 15], color="k", lw=1)
    ax.plot([15, 15], [H - 17, H - 13], color="k", lw=1)
    ax.plot([65, 65], [H - 17, H - 13], color="k", lw=1)
    ax.text(15, H - 18, "50 mm - measure this to confirm the print scale", fontsize=7)
    name = f"tags36h11_{'_'.join(map(str, ids))}_{size_mm:g}mm.pdf"
    p = out_dir / name
    fig.savefig(p)
    return p


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--size", type=float, default=15.0, help="black square edge in mm")
    ap.add_argument("--copies", type=int, default=1)
    a = ap.parse_args()
    out = Path.cwd() / "tags"
    out.mkdir(exist_ok=True)
    print("PDF:", save_pdf(a.ids, a.size, a.copies, out))
    for i in a.ids:
        print("PNG:", save_png(i, a.size, out))