"""Draw the latency tile as "n/a" when capture-to-display is unmeasurable.

Capture-to-display is now minus the capture stamp, which only means
something when the stamp is on the same clock. Under bag replay it is the
bag's own recorded time, so the subtraction returns the age of the
RECORDING: the dashboard drew about 2.5 billion ms, which is 29 days.

Dropping the sample and sending zero would be worse, not better. A tile
reading "0 / 0 ms  Within Budget" is a plausible screenshot, and plausible
and wrong is the dangerous combination. So 6_visualizer_node sends NaN when
it has nothing measurable, and this renders that as "n/a" with a warning
band instead of inventing a number.

Patches the THROWAWAY copy of the repo, chained from semantic_shim.py the
way path_memory.py is, because the dashboard is pip-installed from that
copy before any runner starts. Idempotent: exits 0 when already applied.
"""

import sys
from pathlib import Path

repo = Path(sys.argv[sys.argv.index("--repo") + 1])
f = repo / "src" / "offroad_autonomy" / "visualization" / "dashboard.py"
if not f.exists():
    print(f"  latency_na: {f} not found; not patched")
    raise SystemExit(0)

t = f.read_text()
if "Stamps Not On This Clock" in t:
    print("  latency n/a already applied")
    raise SystemExit(0)

OLD = '        latency_level = bands.latency_level(telemetry.latency_p95_ms, thresholds.latency_budget_ms)\n        latency_note = f"Over {thresholds.latency_budget_ms:.0f} Ms Budget"\n        if latency_level == bands.GOOD:\n            latency_note = f"Within {thresholds.latency_budget_ms:.0f} Ms Budget"\n        self._draw_health_tile(\n            canvas,\n            layout.latency,\n            f"{telemetry.latency_ms:.0f} / {telemetry.latency_p95_ms:.0f}",\n            "ms",\n            latency_level,\n            latency_note,\n            telemetry.latency_p95_ms\n            / (thresholds.latency_budget_ms * thresholds.latency_bar_scale),\n            1.0 / thresholds.latency_bar_scale,\n        )\n'

NEW = '        latency_level = bands.latency_level(telemetry.latency_p95_ms, thresholds.latency_budget_ms)\n        latency_note = f"Over {thresholds.latency_budget_ms:.0f} Ms Budget"\n        if latency_level == bands.GOOD:\n            latency_note = f"Within {thresholds.latency_budget_ms:.0f} Ms Budget"\n        latency_value = f"{telemetry.latency_ms:.0f} / {telemetry.latency_p95_ms:.0f}"\n        latency_unit = "ms"\n        latency_fraction = telemetry.latency_p95_ms / (\n            thresholds.latency_budget_ms * thresholds.latency_bar_scale\n        )\n        # Capture-to-display can only be measured when the capture stamp is on\n        # this clock. Under bag replay it is the recording\'s own time, so there\n        # is nothing to measure and the caller passes NaN to say so.\n        #\n        # Saying "n/a" is the whole point. Left alone this tile drew the age of\n        # the RECORDING, about 2.5 billion ms. The tempting fix is to send zero\n        # instead, and that is worse: "0 / 0 ms  Within Budget" is a plausible\n        # screenshot of a number nobody measured.\n        if not math.isfinite(telemetry.latency_p95_ms):\n            latency_value = "n/a"\n            latency_unit = ""\n            latency_level = bands.WARN\n            latency_note = "Stamps Not On This Clock"\n            latency_fraction = 0.0\n        self._draw_health_tile(\n            canvas,\n            layout.latency,\n            latency_value,\n            latency_unit,\n            latency_level,\n            latency_note,\n            latency_fraction,\n            1.0 / thresholds.latency_bar_scale,\n        )\n'

if OLD not in t:
    print("  latency_na: anchor not found; dashboard.py has moved on, not patched")
    raise SystemExit(0)

f.write_text(t.replace(OLD, NEW))
print("  latency tile reports n/a when stamps are off this clock")
