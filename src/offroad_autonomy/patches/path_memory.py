"""Draw the part of the path the camera cannot currently see as remembered.

The BEV grid plans from 1 m ahead (planning.grid.start_m), but the hood mask
means the nearest ground the camera actually sees is 11.3 m away, and 1 m
projects to row 777 of a 413-row image. So the dashboard draws ten metres of
path over the bonnet, in the same solid colour as the part it is looking at.

That part is not invented - the grid remembers 12-16 m of travel at these
speeds, which is where it comes from - but drawn solid it claims to be an
observation. The dashboard already distinguishes held paths from fresh ones by
dashing them, so this follows the same idea with opacity: points inside the
valid ROI keep the existing treatment, points behind the ego mask are drawn
faint, with no glow and no highlight.

Set DASH_PATH_MEMORY=0 to draw the path the old way.

Patches the THROWAWAY copy of Bryan's repo, chained from semantic_shim.py the
way fix_class.py is, because the dashboard is pip-installed from that copy
before any runner starts.
"""
import sys
from pathlib import Path

repo = Path(sys.argv[sys.argv.index("--repo") + 1])
f = repo / "src" / "offroad_autonomy" / "visualization" / "dashboard.py"
if not f.exists():
    print(f"  path_memory: {f} not found; not patched")
    raise SystemExit(0)

t = f.read_text()
if "_unseen_run" in t:
    print("  path memory already applied")
    raise SystemExit(0)

# The solid branch, drawn when the plan is fresh. Split it so the stretch the
# camera cannot see is drawn faint instead.
old = """            else:
                glow = display.copy()
                cv2.polylines(
                    glow, [pts_i], False, self._colors["PATH_GLOW"], 12, lineType=cv2.LINE_AA
                )
                cv2.addWeighted(glow, 0.22, display, 0.78, 0, display)
                cv2.polylines(display, [pts_i], False, core, 4, lineType=cv2.LINE_AA)
                cv2.polylines(
                    display,
                    [pts_i],
                    False,
                    self._colors["PATH_HIGHLIGHT"],
                    1,
                    lineType=cv2.LINE_AA,
                )"""

new = """            else:
                # Which points sit on ground the camera can currently see. The
                # ego mask is the only thing that hides near ground, so a point
                # behind it, or off the bottom of the frame, is remembered
                # rather than observed.
                import os as _os

                seen = np.ones(len(pts_i), dtype=bool)
                if _os.environ.get("DASH_PATH_MEMORY", "1") != "0" and ego is not None:
                    rows = np.clip(np.round(pts[:, 1]).astype(int), 0, dst_h - 1)
                    cols = np.clip(np.round(pts[:, 0]).astype(int), 0, dst_w - 1)
                    if ego.shape[:2] == (dst_h, dst_w):
                        seen &= ~ego[rows, cols]
                    seen &= (pts[:, 1] >= 0) & (pts[:, 1] <= dst_h - 1)

                def _unseen_run(points):
                    \"\"\"Faint, no glow, no highlight: drawn from memory.\"\"\"
                    if len(points) < 2:
                        return
                    faint = display.copy()
                    cv2.polylines(faint, [points], False, core, 3, lineType=cv2.LINE_AA)
                    cv2.addWeighted(faint, 0.45, display, 0.55, 0, display)

                if seen.all():
                    runs = [(pts_i, True)]
                else:
                    runs = []
                    start = 0
                    for i in range(1, len(seen) + 1):
                        if i == len(seen) or seen[i] != seen[start]:
                            # One point of overlap, or the two stretches meet
                            # with a visible gap between them.
                            end = min(i + 1, len(seen))
                            runs.append((pts_i[start:end], bool(seen[start])))
                            start = i

                for points, visible in runs:
                    if not visible:
                        _unseen_run(points)
                        continue
                    if len(points) < 2:
                        continue
                    glow = display.copy()
                    cv2.polylines(
                        glow, [points], False, self._colors["PATH_GLOW"], 12,
                        lineType=cv2.LINE_AA
                    )
                    cv2.addWeighted(glow, 0.22, display, 0.78, 0, display)
                    cv2.polylines(display, [points], False, core, 4, lineType=cv2.LINE_AA)
                    cv2.polylines(
                        display,
                        [points],
                        False,
                        self._colors["PATH_HIGHLIGHT"],
                        1,
                        lineType=cv2.LINE_AA,
                    )"""

if t.count(old) != 1:
    print(f"  WARNING: {t.count(old)} matches for the solid-path anchor; not patched")
    raise SystemExit(0)

t = t.replace(old, new, 1)
f.write_text(t)
import ast
ast.parse(t)
print("  dashboard: path outside the valid ROI drawn as remembered")
