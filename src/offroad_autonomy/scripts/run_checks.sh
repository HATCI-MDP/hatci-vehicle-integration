#!/usr/bin/env bash
#
# Run The Repository Checks Inside The Vehicle Image
#
# The checks in CLAUDE.md need ruff and pytest. The vehicle image ships pytest
# but deliberately not ruff, because ruff is a development tool that would
# never run on the vehicle and the image is large already. So ruff is installed
# into a throwaway container on each run and discarded with it. The image is
# never modified.
#
# The repository is mounted read-only, so a check cannot alter the tree it is
# checking. pytest's cache is disabled for the same reason.
#
# Usage:
#   scripts/run_checks.sh                     # uses hatci-vehicle:draft7
#   IMAGE=hatci-vehicle:draft8 scripts/run_checks.sh
set -euo pipefail

IMAGE="${IMAGE:-hatci-vehicle:draft7}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Read the pin from pyproject.toml rather than repeating it here, because a
# version that drifts from the declared one silently checks against different
# rules than CI does.
RUFF_SPEC="$(sed -n 's/^[[:space:]]*"\(ruff==[^"]*\)".*/\1/p' "$REPO/pyproject.toml" | head -1)"
if [ -z "$RUFF_SPEC" ]; then
    echo "run_checks: no ruff pin found in pyproject.toml" >&2
    exit 1
fi

echo "Repository: $REPO"
echo "Image:      $IMAGE"
echo "Ruff:       $RUFF_SPEC"
echo

docker run --rm --entrypoint bash \
    -v "$REPO":/work:ro \
    -e "RUFF_SPEC=$RUFF_SPEC" \
    "$IMAGE" -lc '
        set -uo pipefail
        export PYTHONPATH=/work/src:${PYTHONPATH:-}
        export RUFF_CACHE_DIR=/tmp/ruffcache
        export PATH=/tmp/ruffbin/bin:$PATH
        cd /work
        status=0

        echo "===== Installing $RUFF_SPEC ====="
        if pip3 install --quiet --no-cache-dir --target=/tmp/ruffbin "$RUFF_SPEC"; then
            ruff --version
        else
            echo "SKIPPED: ruff could not be installed (no network?)" >&2
            status=1
        fi
        echo

        if command -v ruff >/dev/null 2>&1; then
            echo "===== ruff check src tests scripts ====="
            ruff check src tests scripts || status=1
            echo
            echo "===== ruff format --check src tests scripts ====="
            ruff format --check src tests scripts || status=1
            echo
        fi

        echo "===== pytest ====="
        # No --cov: pytest-cov is not in the vehicle image, and installing it
        # per run buys nothing the suite itself does not already report.
        if python3 -c "import pytest_cov" 2>/dev/null; then
            python3 -m pytest tests -q -p no:cacheprovider --cov=offroad_autonomy || status=1
        else
            echo "(pytest-cov absent, running without coverage)"
            python3 -m pytest tests -q -p no:cacheprovider || status=1
        fi
        echo

        echo "===== scripts/sbend_sim.py ====="
        python3 scripts/sbend_sim.py || status=1

        exit $status
    '
