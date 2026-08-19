#!/bin/bash
# run_controller_comparison.sh
# Runs all implemented controller configs, extracts position CSVs.
# Usage: bash run_controller_comparison.sh [--plot]
#
# Run from: ~/workspace/carla_platooning/examples/joinAtBack/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
EXTRACT_SCRIPT="$LOG_DIR/extract_real_positions.py"
PLOT_SCRIPT="$LOG_DIR/plot_controller_comparison.py"

OPP_RUN="opp_run"
OPP_ARGS="-m -u Cmdenv \
  -n ..:../../src:../../../carlanetpp/src:../../../omnetpp-6.0.3/v2v/inet4.5/examples:../../../omnetpp-6.0.3/v2v/inet4.5/showcases:../../../omnetpp-6.0.3/v2v/inet4.5/src:../../../omnetpp-6.0.3/v2v/inet4.5/tests/validation:../../../omnetpp-6.0.3/v2v/inet4.5/tests/networks:../../../omnetpp-6.0.3/v2v/inet4.5/tutorials:../../../veins/examples/veins:../../../veins/src/veins:../../../plexe/examples/platooning:../../../plexe/src/plexe \
  -x 'inet.common.selfdoc;inet.linklayer.configurator.gatescheduling.z3;inet.emulation;inet.showcases.visualizer.osg;inet.examples.emulation;inet.showcases.emulation;inet.transportlayer.tcp_lwip;inet.applications.voipstream;inet.visualizer.osg;inet.examples.voipstream' \
  --image-path=../../../omnetpp-6.0.3/v2v/inet4.5/images:../../../veins/images:../../../plexe/images \
  -l ../../src/carla -l ../../../carlanetpp/src/carlanet \
  -l ../../../omnetpp-6.0.3/v2v/inet4.5/src/INET -l ../../../veins/src/veins \
  -l ../../../plexe/src/plexe omnetpp.ini"

# Controllers to run (implemented ones only)
CONFIGS=("CC" "ACC_03" "ACC_12" "CACC")

# Controllers not yet implemented (will show placeholder in plot)
NOT_IMPLEMENTED=("PLOEG" "FLATBED" "CONSENSUS")

mkdir -p "$LOG_DIR"

cd "$SCRIPT_DIR"

echo "========================================"
echo "  Controller Comparison Run"
echo "========================================"

for CONFIG in "${CONFIGS[@]}"; do
    LOG="omnetpp_ControllerComparison_${CONFIG}.log"
    CSV="$LOG_DIR/positions_${CONFIG}.csv"

    echo ""
    echo "--- Running $CONFIG ---"

    if [ -f "$LOG" ]; then
        echo "  Log exists, skipping simulation (delete to re-run: rm $LOG)"
    else
        echo "  Simulating..."
        $OPP_RUN $OPP_ARGS -c "$CONFIG" > "$LOG" 2>&1
        echo "  Done. Log: $LOG"
    fi

    echo "  Extracting positions..."
    python3 "$EXTRACT_SCRIPT" "$LOG" \
        --sample-period 0.5 \
        --target-gap 5.0 \
        --gap-tolerance 1.0 \
        -o "$CSV"
    echo "  CSV: $CSV"
done

echo ""
echo "========================================"
echo "  All simulations complete"
echo "========================================"
echo ""

if [[ "$1" == "--plot" ]]; then
    echo "Generating comparison plot..."
    python3 "$EXTRACT_SCRIPT" "$LOG" \
        --sample-period 0.5 \
        --target-gap 5.0 \
        --gap-tolerance 1.0 \
        -o "$CSV_DIR/real_positions_sinusoidal${CONFIG}.csv"
fi