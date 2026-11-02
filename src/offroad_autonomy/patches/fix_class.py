"""Pick the road class by what the model PREDICTS, not only by what it names.

The 2026-10-14 fine-tunes declare three classes - background, Final-Demo,
traversable_road - but never emit traversable_road: `Final-Demo` is the road,
the project name having become the label. The existing selector matches on the
name "road"/"travers", picks the declared-but-absent class, and returns an
empty mask. Measured: 100% empty paths against 4.8% for the previous model.

So intersect the name match with the classes actually present in this
prediction, and fall back to every non-background index when they disjoint.
That keeps the old two-class model working, fixes these, and degrades to the
existing fallback for anything else.
"""
import sys
from pathlib import Path

repo = Path(sys.argv[sys.argv.index("--repo") + 1])
f = repo / "src" / "offroad_autonomy" / "perception" / "road_segmenter.py"
t = f.read_text()
if "present_road" in t:
    print("  road class selection already fixed"); raise SystemExit(0)

old = """        if road:
            return np.isin(indices, road)"""
if t.count(old) != 1:
    print(f"  WARNING: {t.count(old)} matches for the class-selection anchor; not patched")
    raise SystemExit(0)
new = """        present = set(np.unique(indices).tolist())
        present_road = [i for i in road if i in present]
        if present_road:
            return np.isin(indices, present_road)
        if road:
            # Named a road class that this prediction never contains. The
            # 2026-10 fine-tunes do exactly this: traversable_road is declared
            # and never emitted, while Final-Demo carries the road. Trusting
            # the name alone returns an empty mask on every frame.
            logger.debug("Road-like classes %s absent from the prediction (present: "
                         "%s); taking every non-background index", road, sorted(present))"""
t = t.replace(old, new, 1)
f.write_text(t)
import ast; ast.parse(t)
print("  road_segmenter: road class now intersected with predicted classes")
