#!/bin/bash
# ============================================================
#  AGC/play.sh — Air-Ground Convoy one-shot launcher
#  Starts CarlaAir (if not running), waits for ports, then
#  plays the convoy scenario in a pygame window.
#
#  Usage (from inside AGC/):
#      ./play.sh                      # default 43 s run
#      ./play.sh --duration 60        # custom duration
#      ./play.sh --truck-speed 6.0    # slower truck
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CARLAAIR_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---------- activate carlaAir conda env (python 3.10 + carla + airsim) ----------
CONDA_BASE=""
for p in "$HOME/miniconda3" "$HOME/anaconda3"; do
    [ -f "$p/etc/profile.d/conda.sh" ] && CONDA_BASE="$p" && break
done
if [ -z "$CONDA_BASE" ]; then
    echo "ERROR: could not find miniconda/anaconda install."
    exit 1
fi
source "${CONDA_BASE}/etc/profile.d/conda.sh"
if ! conda env list | grep -qw "carlaAir"; then
    echo "ERROR: conda env 'carlaAir' not found."
    echo "       Run:  bash ${CARLAAIR_ROOT}/env_setup/setup_env.sh"
    exit 1
fi
conda activate carlaAir

# ---------- start CarlaAir if 2000 isn't listening ----------
if ! ss -ltn 2>/dev/null | grep -q ':2000 '; then
    echo "[AGC] CarlaAir not running — launching (Epic, Town10HD) ..."
    if [ ! -x "${CARLAAIR_ROOT}/CarlaAir.sh" ]; then
        echo "ERROR: CarlaAir.sh not found at ${CARLAAIR_ROOT}"
        echo "       AGC/ must live inside a CarlaAir v0.1.7 install."
        exit 1
    fi
    "${CARLAAIR_ROOT}/CarlaAir.sh" Town10HD --quality Epic --res 1280x720 \
        > /tmp/carlaair_agc.log 2>&1 &
    disown

    echo -n "[AGC] waiting for ports 2000 + 41451 "
    READY=0
    for _ in $(seq 1 60); do
        if ss -ltn 2>/dev/null | grep -q ':2000 ' \
           && ss -ltn 2>/dev/null | grep -q ':41451 '; then
            READY=1; break
        fi
        echo -n "."
        sleep 2
    done
    echo ""
    if [ "$READY" -ne 1 ]; then
        echo "ERROR: CarlaAir did not open ports within 120 s."
        echo "       Log: /tmp/carlaair_agc.log"
        exit 1
    fi
    sleep 3   # let AirSim drone stabilise before we spawn the truck
else
    echo "[AGC] CarlaAir already running — reusing."
fi

# ---------- run the scenario ----------
echo "[AGC] starting scenario ..."
exec python3 "${SCRIPT_DIR}/air_ground_convoy.py" "$@"
