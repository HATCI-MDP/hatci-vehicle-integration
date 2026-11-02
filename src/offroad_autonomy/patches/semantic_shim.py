"""Teach offroad_autonomy's RoadSegmenter to consume a task='semantic' model.

Applied to the harness's throwaway copy of orfd-lane-detection inside the
container, never to the repo on disk. Run it between copying the repo in and
pip installing it.

Why a shim and not a change to RoadSegmenter
--------------------------------------------
`RoadSegmenter` reads `result.masks` and `result.boxes.conf`, which is the
`task='segment'` contract. A semantic model returns neither: its output is
`result.semantic_mask`, a full-resolution class-index map, and it has no
per-detection score at all. Teaching the real class both shapes would put a
benchmark concern into a pipeline that does not otherwise use semantic models,
so the adaptation lives here.

The confidence problem
----------------------
`PerceptionGate` rejects a frame when the best detection confidence falls below
`planning.gate.confidence_threshold`. A semantic model has nothing to put there.
This shim substitutes the road pixel fraction within the valid ROI, which is a
different quantity on a different scale: it is bounded by how much of the frame
is road, so a correct segmentation of a narrow trail scores low. Any run using
it must carry its own gate threshold, and its confidences must never be compared
against a YOLOE model's in the same table.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# The chained scripts sit beside this one. Resolving them relative to this
# file rather than to an absolute /bench/scripts means the shim reproduces the
# patched tree anywhere, not only inside the container. Inside the image the
# siblings are in /bench/scripts too, so this resolves to the same files.
_HERE = Path(__file__).resolve().parent

SHIM = '''

# --- benchmark shim: task='semantic' support -------------------------------
# Added by bench-ros/scripts/semantic_shim.py against a throwaway copy of this
# repo. Not part of offroad_autonomy.
def _bench_load_model(weights):
    """Load weights without forcing task='segment'.

    RoadSegmenter hardcodes task='segment', which is right for YOLOE but makes
    ultralytics run the segment post-processor over a semantic model's output
    and fail with "index 1 is out of bounds". Letting the file's own metadata
    pick the task loads both correctly; the explicit task is kept as a fallback
    for weights that carry none.
    """
    from ultralytics import YOLO as _YOLO
    try:
        m = _YOLO(weights)
        if getattr(m, "task", None) in ("segment", "semantic"):
            return m
    except Exception:
        pass
    return _YOLO(weights, task="segment")


def _bench_semantic_mask(result):
    """Return a boolean road mask from a task='semantic' result, or None.

    ultralytics returns `semantic_mask` as a result wrapper whose `.data` is a
    CUDA tensor, so it needs taking off the device before numpy sees it.
    """
    import numpy as _np
    sem = getattr(result, "semantic_mask", None)
    if sem is None:
        return None
    data = getattr(sem, "data", sem)
    for attr in ("detach", "cpu"):
        if hasattr(data, attr):
            data = getattr(data, attr)()
    if hasattr(data, "numpy"):
        data = data.numpy()
    arr = _np.squeeze(_np.asarray(data))
    names = getattr(result, "names", {}) or {}
    road = [i for i, n in names.items()
            if "road" in str(n).lower() or "travers" in str(n).lower()]
    return _np.isin(arr, road) if road else (arr > 0)
'''

PATCH_EXTRACT = """
        # --- benchmark shim ---------------------------------------------
        _sem = _bench_semantic_mask(result)
        if _sem is not None:
            import cv2 as _cv2
            _m = _sem.astype(bool)
            if _m.shape != (h, w):
                _m = _cv2.resize(_m.astype("uint8"), (w, h),
                                 interpolation=_cv2.INTER_NEAREST).astype(bool)
            # One instance, and confidence 1.0 rather than a fabricated score.
            # A semantic model has no detection confidence; its "is there road"
            # signal is the mask itself, so the gate's min_road_pixels and
            # min_mask_area checks do the gating and the confidence check is
            # left to pass. Treat its confidence column as meaningless.
            return [_m], [1.0]
        # --- end benchmark shim -----------------------------------------
"""


def _fix_road_class(repo: Path) -> None:
    """Chained so every runner gets it: the class selector must check what the
    model predicts, not only what it declares. See scripts/fix_class.py."""
    import subprocess, sys as _sys
    subprocess.run([_sys.executable, str(_HERE / "fix_class.py"), "--repo", str(repo)],
                   check=True)


def _path_memory(repo: Path) -> None:
    """Chained likewise: the dashboard draws ten metres of path over the
    bonnet in the same solid colour as the road it can actually see.
    See scripts/path_memory.py."""
    import subprocess, sys as _sys
    subprocess.run([_sys.executable, str(_HERE / "path_memory.py"), "--repo", str(repo)],
                   check=True)


def apply(repo: Path) -> int:
    _fix_road_class(repo)
    _path_memory(repo)
    target = repo / "src" / "offroad_autonomy" / "perception" / "road_segmenter.py"
    if not target.exists():
        print(f"  road_segmenter.py not found under {repo}")
        return 1
    text = target.read_text()
    if "_bench_semantic_mask" in text:
        print("  shim already applied")
        return 0

    # The repo may now carry this properly, via the feat/semantic-segmentation-
    # adapter branch. The shim's patch anchor no longer exists there, so it
    # would fail to apply and print a misleading error. Stand down instead: the
    # adapter does the same job and does it better.
    if "_semantic_mask" in text and "semantic_mask" in text:
        print("  repo already supports task='semantic'; shim not needed")
        return 0

    text = text + SHIM

    # Patch _extract_masks, not predict(): it is the single place .masks is
    # read, and returning (instances, confidences) from there leaves the ROI
    # clipping, confidence weighting and road_fraction downstream untouched.
    anchor = "        if result is None or result.masks is None:"
    idx = text.find(anchor)
    if idx < 0:
        print("  could not find _extract_masks' guard to patch")
        return 1
    text = text[:idx] + PATCH_EXTRACT + text[idx:]

    # And the load, which hardcodes the task.
    before = text
    text = text.replace('YOLO(weights, task="segment")', "_bench_load_model(weights)", 1)
    if text == before:
        print("  warning: could not repoint the YOLO(...) load; semantic engines "
              "will fail in the segment post-processor")
    else:
        print("  repointed the model load through _bench_load_model")

    target.write_text(text)
    print(f"  patched {target.relative_to(repo)}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default="/tmp/bryan")
    a = ap.parse_args()
    sys.exit(apply(Path(a.repo)))
