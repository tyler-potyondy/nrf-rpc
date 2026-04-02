#!/bin/bash
# Script to setup zephyr submodule and build the server app executable.
set -e 

# Parse flags
AUTO_YES=false
for arg in "$@"; do
    case $arg in
        -y|--yes) AUTO_YES=true ;;
    esac
done

# Add warning and confirmation before running the script since it will delete the external repo and 
# reinstall everything from scratch.
if [ "$AUTO_YES" = false ]; then
    echo "WARNING: zephyr_setup will delete and install a new clean zephyr setup. Please confirm you want to proceed (y/n)"
    read -r response
    if [[ "$response" != "y" ]]; then
        echo "Aborting zephyr setup."
        exit 0
    fi
fi


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

# Delete everything including .gitignore files (but keep the top-level .gitignore)
log_info "Cleaning up existing external directory..."
find external -mindepth 2 -delete 2>/dev/null || true
find external -mindepth 1 -maxdepth 1 -not -name '.gitignore' -delete 2>/dev/null || true

log_info "Setting up zephyr submodule and building server app executable..."
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
    log_info "Creating venv"
    python3 -m venv .venv
fi

source .venv/bin/activate
pip install west
log_success "Entered venv"

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

pip install -r nrf/scripts/requirements.txt
pip install -r zephyr/scripts/requirements.txt

# Build Babble Simulator. 
log_info "Building Babble Simulator..."
make -C tools/bsim everything -j 4 

log_success "Babble Simulator build complete."


# Pin zephyr to 3.2.1
# git -C zephyr checkout v3.2.1

# Build nrf sdk rcp example. 
log_info "Building Zephyr nrf_rpc protocol_serialization server example..."
ls


# confirm we are on branch cgm-bsim
current_branch=$(git -C nrf rev-parse --abbrev-ref HEAD)
if [ "$current_branch" != "cgm-bsim" ]; then
    git -C nrf fetch origin cgm-bsim
    git -C nrf checkout -B cgm-bsim FETCH_HEAD
fi


echo $PWD

# Build ble rpc server.
west build -b nrf52_bsim -p always --build-dir build/zephyr_server_app nrf/samples/nrf_rpc/protocols_serialization/server -S ble

# Build the cgm peripheral sample.
west build -b nrf52_bsim -p always --build-dir build/cgm_peripheral_sample nrf/samples/bluetooth/peripheral_cgms

# Confirm we are in the external directory.
if [ "$(basename "$PWD")" != "external" ]; then
    log_error "Error: This script must be run from the external directory."
    exit 1
fi


# Copy all build artifacts to external/tools/bsim/bin
cp -r build/zephyr_server_app/server/zephyr/zephyr.exe tools/bsim/bin/zephyr_rpc_server_app
cp -r build/cgm_peripheral_sample/peripheral_cgms/zephyr/zephyr.exe tools/bsim/bin/cgm_peripheral_sample

log_success "Build artifacts copied to tools/bsim/bin/"

