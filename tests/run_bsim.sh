#!/bin/bash
# BabbleSim test setup for nRF RPC UART

set -e

# Confirm currently in directory tests/external
if [ "$(basename "$PWD")" != "tests" ]; then
    echo "Error: This script must be run from the tests directory."
    exit 1
fi

# Set BabbleSim paths
export BSIM_OUT_PATH=external/tools/bsim
export BSIM_COMPONENTS_PATH=${BSIM_OUT_PATH}/components
export LD_LIBRARY_PATH=${BSIM_OUT_PATH}/lib:${LD_LIBRARY_PATH}

# Simulation ID (use same for all devices in the simulation)
SIM_ID="nrf_rpc_test"

# Device number
DEVICE_NUM=0

# Clean up old lock files
rm -rf /tmp/bs_${USER}/${SIM_ID} 2>/dev/null

echo "Starting BabbleSim PHY simulator..."
cd ${BSIM_OUT_PATH}/bin
./bs_2G4_phy_v1 -s=${SIM_ID} -D=2 -sim_length=86400e6 &
PHY_PID=$!

# Wait for PHY to start
sleep 1

# Start time monitor to advance simulation time (suppress output)
echo "Starting time monitor device..."
./bs_device_time_monitor -s=${SIM_ID} -d=1 -interval=10000000 >/dev/null 2>&1 &
MONITOR_PID=$!

sleep 0.5

# (TODO) May need to make this architecture agnostic.
echo "Starting nRF RPC server with BabbleSim..."
cd ../../../build/zephyr_server_app/server/zephyr 

echo ""
echo "=== BabbleSim Running ===" 
echo "PHY PID: ${PHY_PID}"
echo "Monitor PID: ${MONITOR_PID}"
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
./zephyr.exe -s=${SIM_ID} -d=${DEVICE_NUM} -uart0_pty -uart_pty_pollT=1000
