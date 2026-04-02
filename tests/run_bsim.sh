#!/bin/bash
# BabbleSim test setup for nRF RPC UART

#set -e

# Confirm currently in directory tests/external
if [ "$(basename "$PWD")" != "tests" ]; then
    echo "Error: This script must be run from the tests directory."
    exit 1
fi

# Set BabbleSim paths
export BSIM_OUT_PATH=external/tools/bsim
export BSIM_COMPONENTS_PATH=${BSIM_OUT_PATH}/components
export LD_LIBRARY_PATH=${BSIM_OUT_PATH}/lib:${LD_LIBRARY_PATH}

# Simulation ID provided as arg or exit
SIM_ID="$1"
if [ -z "$SIM_ID" ]; then
    echo "Error: Simulation ID must be provided as the first argument."
    exit 1
fi

pkill -f "bs_2G4_phy_v1 -s=${SIM_ID}" 2>/dev/null || true
pkill -f "zephyr_rpc_server_app -s=${SIM_ID}" 2>/dev/null || true
pkill -f "cgm_peripheral_sample -s=${SIM_ID}" 2>/dev/null || true

sleep 0.5
pkill -9 "bs_2G4_phy_v1 -s=${SIM_ID}" 2>/dev/null || true
pkill -9 "zephyr_rpc_server_app -s=${SIM_ID}" 2>/dev/null || true
pkill -9 "cgm_peripheral_sample -s=${SIM_ID}" 2>/dev/null || true

# Clean up old lock files
rm -rf /tmp/bs_${USER}/${SIM_ID} 2>/dev/null

echo "Starting BabbleSim PHY simulator..."
cd ${BSIM_OUT_PATH}/bin
./bs_2G4_phy_v1 -s=${SIM_ID} -D=2 -sim_length=86400e6 &

# (TODO) May need to make this architecture agnostic.
echo "Starting nRF RPC server with BabbleSim..."

echo ""
echo "=== BabbleSim Running ===" 
echo "Simulation ID: ${SIM_ID}"
echo "Simulation length: 86400 seconds (24 hours simulated, ~39 seconds real time at 2200x speed)"
echo ""
echo "To test RX, run in another terminal:"
echo "  socat UNIX-LISTEN:/tmp/nrf_rpc_server.sock,fork /dev/pts/XX,raw,echo=0"
echo "  printf '\\x04\\x00\\xff\\x00\\xff\\x00\\x62\\x74\\x5f\\x72\\x70\\x63' | socat - UNIX-CONNECT:/tmp/nrf_rpc_server.sock"
echo ""
echo "Starting device (Press Ctrl+C to stop)..."
echo ""

# Run in foreground to see all output
./zephyr_rpc_server_app -s=${SIM_ID} -d=0 -uart0_pty -uart_pty_pollT=1000 &
ZEPHYR_PID=$!

rm -f cgm_peripheral_sample.log
./cgm_peripheral_sample -s=${SIM_ID} -d=1 > cgm_peripheral_sample.log 2>&1 &

# Wait for the zephyr process to exit (keeps the script alive)
wait $ZEPHYR_PID
