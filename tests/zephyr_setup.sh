#!/bin/bash
# Script to setup zephyr submodule and build the server app executable.
set -e 

# ANSI color codes
RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
CYAN=$'\033[0;36m'
BOLD=$'\033[1m'
NC=$'\033[0m' # No Color

# Logging functions
log_info() {
    echo "${CYAN}${BOLD}$1${NC}"
}

log_error() {
    echo "${RED}${BOLD}$1${NC}"
}

log_success() {
    echo "${GREEN}${BOLD}$1${NC}"
}

# Ensure this script is only run in the tests directory (otherwise exit).
if [ "$(basename "$PWD")" != "tests" ]; then
    log_error "Error: This script must be run from the tests directory."
    exit 1
fi

git submodule update --init external/nrf

# All these commands are run in the external dir.
cd external

# Ensure machine has python.
if ! command -v python3 &> /dev/null; then
    log_error "Error: python3 is not installed."
    exit 1
fi

# Setup venv for installing west.
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

source .venv/bin/activate
pip3 install west

######################################################################
# (Note/todo) The following west commands scare me a bit since we are 
# not pinned to a specific version and at the mercy of what 
# west does here. Probably okay for now though.
#####################################################################

# Check if west is already initialized (i.e. if .west/ directory exists). If not, initialize west.
if [ -d ".west" ]; then
    log_info " Previous west setup, resetting now..."
    rm -rf .west
fi

west init -l nrf # This command is particularly evil and will use up a few GB of disk space :(

log_info "Updating west Babble Simulator..."
west config manifest.group-filter -- +babblesim
west update 

pip3 install -r nrf/scripts/requirements.txt
pip3 install -r zephyr/scripts/requirements.txt

# Build Babble Simulator. 
log_info "Building Babble Simulator..."
make -C tools/bsim everything -j 4 

log_success "Babble Simulator build complete."


# Pin zephyr to 3.2.1
# git -C zephyr checkout v3.2.1

# Build nrf sdk rcp example. 
log_info "Building Zephyr nrf_rpc protocol_serialization server example..."
ls


# confirm we are on branch bsim-test
current_branch=$(git -C nrf rev-parse --abrev-ref HEAD)
if [ "$current_branch" != "bsim-test" ]; then
    git -C nrf checkout bsim-test
fi


echo $PWD

west build -b nrf52_bsim -p always --build-dir build/zephyr_server_app nrf/samples/nrf_rpc/protocols_serialization/server -S ble

