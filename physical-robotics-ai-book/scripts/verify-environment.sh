#!/bin/bash
# Physical AI Environment Verification Script
# Tests: Ubuntu 22.04, ROS 2 Humble, Isaac Sim, ROS 2 topics
# Usage: ./verify-environment.sh

set -e

echo "=========================================="
echo "  Physical AI Environment Verification"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

# Helper functions
check_pass() {
    echo -e "${GREEN}✅ PASS${NC}: $1"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}❌ FAIL${NC}: $1"
    echo "  → $2"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}⚠️  WARN${NC}: $1"
    echo "  → $2"
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [1/7] System Requirements"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" = "22.04" ]; then
        check_pass "Ubuntu 22.04 LTS detected"
    else
        check_fail "Ubuntu version is $VERSION_ID" "Requires Ubuntu 22.04 LTS"
    fi
else
    check_fail "Cannot detect Ubuntu version" "Check /etc/os-release"
fi

# Check CPU cores
CPU_CORES=$(nproc)
if [ "$CPU_CORES" -ge 6 ]; then
    check_pass "CPU cores: $CPU_CORES (≥6 required)"
else
    check_fail "CPU cores: $CPU_CORES" "Minimum 6 cores recommended"
fi

# Check RAM
TOTAL_RAM_GB=$(free -g | awk '/^Mem:/{print $2}')
if [ "$TOTAL_RAM_GB" -ge 15 ]; then
    check_pass "RAM: ${TOTAL_RAM_GB} GB (≥16 GB required)"
else
    check_fail "RAM: ${TOTAL_RAM_GB} GB" "Minimum 16 GB recommended"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [2/7] NVIDIA GPU"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if command -v nvidia-smi &> /dev/null; then
    DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    GPU_MEMORY=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1)

    check_pass "NVIDIA driver $DRIVER_VERSION detected"
    check_pass "GPU: $GPU_NAME"

    if [ "$GPU_MEMORY" -ge 11000 ]; then
        check_pass "GPU memory: $((GPU_MEMORY / 1024)) GB (≥12 GB required)"
    else
        check_warn "GPU memory: $((GPU_MEMORY / 1024)) GB" "12 GB+ recommended for Isaac Sim"
    fi
else
    check_fail "nvidia-smi not found" "Install NVIDIA drivers (version 525+)"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [3/7] ROS 2 Humble"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ "$ROS_DISTRO" = "humble" ]; then
    check_pass "ROS 2 Humble environment sourced"
else
    check_fail "ROS_DISTRO=$ROS_DISTRO" "Source ROS 2 Humble: source /opt/ros/humble/setup.bash"
fi

if command -v ros2 &> /dev/null; then
    ROS2_VERSION=$(ros2 --version 2>&1 | head -1)
    check_pass "ros2 command available: $ROS2_VERSION"
else
    check_fail "ros2 command not found" "Install ROS 2 Humble"
fi

# Check key ROS 2 packages
ROS2_PKGS=$(ros2 pkg list 2>/dev/null | wc -l)
if [ "$ROS2_PKGS" -ge 400 ]; then
    check_pass "ROS 2 packages: $ROS2_PKGS (≥400 expected)"
else
    check_warn "ROS 2 packages: $ROS2_PKGS" "Expected 400+ packages"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [4/7] ROS 2 Development Tools"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if command -v colcon &> /dev/null; then
    check_pass "colcon build tool installed"
else
    check_fail "colcon not found" "Install: sudo apt install python3-colcon-common-extensions"
fi

if command -v rosdep &> /dev/null; then
    check_pass "rosdep dependency manager installed"
else
    check_fail "rosdep not found" "Install: sudo apt install python3-rosdep"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [5/7] Isaac Sim"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2024.1.1"
if [ -d "$ISAAC_SIM_PATH" ]; then
    check_pass "Isaac Sim 2024.1.1 found at $ISAAC_SIM_PATH"

    if [ -x "$ISAAC_SIM_PATH/isaac-sim.sh" ]; then
        check_pass "Isaac Sim launch script is executable"
    else
        check_fail "Isaac Sim launch script not executable" "chmod +x $ISAAC_SIM_PATH/isaac-sim.sh"
    fi
else
    check_warn "Isaac Sim not found" "Install via Omniverse Launcher (optional for Module 01)"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [6/7] ROS 2 Workspace"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ROS2_WS="$HOME/ros2_ws"
if [ -d "$ROS2_WS/src" ]; then
    check_pass "ROS 2 workspace found at $ROS2_WS"

    if [ -d "$ROS2_WS/install" ]; then
        check_pass "ROS 2 workspace built (install/ directory exists)"
    else
        check_warn "ROS 2 workspace not built" "Run: cd $ROS2_WS && colcon build"
    fi
else
    check_warn "ROS 2 workspace not found" "Create: mkdir -p $ROS2_WS/src"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " [7/7] ROS 2 Demo Test (Talker/Listener)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo "Starting ROS 2 talker node (5 seconds)..."
timeout 5s ros2 run demo_nodes_cpp talker > /tmp/ros2_talker.log 2>&1 &
TALKER_PID=$!
sleep 1

if ps -p $TALKER_PID > /dev/null 2>&1; then
    check_pass "ROS 2 talker node running"
    kill $TALKER_PID 2>/dev/null || true
else
    check_fail "ROS 2 talker node failed to start" "Check ROS 2 installation"
fi

echo ""
echo "=========================================="
echo "  Verification Summary"
echo "=========================================="
echo ""
echo -e "${GREEN}Passed: $PASS_COUNT${NC}"
echo -e "${RED}Failed: $FAIL_COUNT${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}✅ ALL CHECKS PASSED - Environment Ready!${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Test Isaac Sim: Launch Carter demo"
    echo "  2. Clone book repository"
    echo "  3. Start Module 02: Your First Humanoid"
    exit 0
else
    echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${RED}❌ SOME CHECKS FAILED${NC}"
    echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "Please fix the failed checks above before proceeding."
    echo "See troubleshooting guide in documentation."
    exit 1
fi
