#!/usr/bin/env bash
#
# e-Yantra StrataCobot 2026-27 -- task dependency installer
#
#   ./requirements.sh              install what is missing
#   ./requirements.sh --check      report what is missing, install nothing
#
# Needs Ubuntu 24.04 with ROS 2 Jazzy (ros-jazzy-desktop-full) already installed:

set -euo pipefail

ROS_DISTRO_REQUIRED="jazzy"
UBUNTU_VERSION_REQUIRED="24.04"


ROS_PACKAGES_ADDED=(
  ros-jazzy-ros2-control
  ros-jazzy-ros2-controllers
  ros-jazzy-gz-ros2-control
  ros-jazzy-controller-manager
  ros-jazzy-controller-manager-msgs
  ros-jazzy-tf-transformations
  ros-jazzy-rqt-tf-tree
  ros-jazzy-depth-image-proc
)

ROS_PACKAGES_EXPECTED=(
  ros-jazzy-ros-gz-sim
  ros-jazzy-ros-gz-bridge
  ros-jazzy-ros-gz-interfaces
  ros-jazzy-gz-sim-vendor
  ros-jazzy-xacro
  ros-jazzy-kdl-parser
  ros-jazzy-orocos-kdl-vendor
  ros-jazzy-urdf
  ros-jazzy-rviz2
  ros-jazzy-robot-state-publisher
  ros-jazzy-tf2-ros
  ros-jazzy-tf2-geometry-msgs
  ros-jazzy-ament-index-cpp
  ros-jazzy-ament-index-python
  ros-jazzy-rclpy
  ros-jazzy-cv-bridge
  ros-jazzy-image-geometry
  ros-jazzy-sensor-msgs-py
)

TOOLS_PACKAGES=(
  ros-jazzy-teleop-twist-keyboard
  ros-jazzy-rqt-gui
  ros-jazzy-rqt-gui-py
  ros-jazzy-rqt-common-plugins
  ros-jazzy-rqt-graph
  ros-jazzy-rqt-image-view
  ros-jazzy-rqt-plot
  ros-jazzy-rqt-topic
  ros-jazzy-rqt-reconfigure
  ros-jazzy-rqt-bag
  ros-jazzy-tf2-tools
  ros-jazzy-ros2bag
  ros-jazzy-rosbag2
  ros-jazzy-rosbag2-cpp
  ros-jazzy-rosbag2-py
  ros-jazzy-rosbag2-storage-mcap
)

PYTHON_PACKAGES=(
  python3-numpy
  python3-scipy
  python3-opencv
  python3-transforms3d
  python3-matplotlib
  python3-yaml
  python3-pip
  
)

SYSTEM_PACKAGES=(
  libfcl-dev
  libassimp-dev
  libssl3t64
)

# The Gazebo stack this release was built and tested with (26 Sep 2026: gz-sim 8.15.0,
# sdformat 14.9.0, gz-common 5.9.0, gz-physics 7.8.0, gz-transport 13.6.0, ros-gz 1.0.24).
# An older version is upgraded; a newer one is reported, since it has not been tested.
GAZEBO_TESTED_VERSIONS=(
  "ros-jazzy-gz-sim-vendor 0.0.13"
  "ros-jazzy-sdformat-vendor 0.0.14"
  "ros-jazzy-gz-common-vendor 0.0.10"
  "ros-jazzy-gz-physics-vendor 0.0.9"
  "ros-jazzy-gz-transport-vendor 0.0.9"
  "ros-jazzy-gz-msgs-vendor 0.0.8"
  "ros-jazzy-gz-math-vendor 0.0.11"
  "ros-jazzy-gz-sensors-vendor 0.0.6"
  "ros-jazzy-gz-rendering-vendor 0.0.7"
  "ros-jazzy-gz-ros2-control 1.2.20"
  "ros-jazzy-ros-gz-sim 1.0.24"
  "ros-jazzy-ros-gz-bridge 1.0.24"
  "ros-jazzy-ros-gz-interfaces 1.0.24"
)

BUILD_PACKAGES=(
  build-essential
  cmake
  git
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
)

if [ -t 1 ]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_CYAN=$'\033[36m'
else
  C_RESET=""; C_BOLD=""; C_DIM=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_CYAN=""
fi

section() { printf '\n%s%s%s\n' "$C_BOLD" "$1" "$C_RESET"; }
ok()      { printf '  %s[ ok ]%s %s\n' "$C_GREEN" "$C_RESET" "$1"; }
missing() { printf '  %s[miss]%s %s\n' "$C_YELLOW" "$C_RESET" "$1"; }
fail()    { printf '  %s[fail]%s %s\n' "$C_RED" "$C_RESET" "$1"; }
info()    { printf '  %s%s%s\n' "$C_DIM" "$1" "$C_RESET"; }
die()     { printf '\n%serror:%s %s\n\n' "$C_RED" "$C_RESET" "$1" >&2; exit 1; }

rule() {
  printf '%s%s%s\n' "$C_CYAN" "==================================================================" "$C_RESET"
}

CHECK_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    *) die "unrecognised argument '$1'. Usage: ./requirements.sh [--check]" ;;
  esac
  shift
done

printf '\n'
rule
printf '  %se-Yantra StrataCobot 2026-27  |  task dependency installer%s\n' "$C_BOLD" "$C_RESET"
rule

section "Checking the system"

[ -r /etc/os-release ] || die "cannot read /etc/os-release; this is not a supported system."

. /etc/os-release

if [ "${ID:-}" != "ubuntu" ]; then
  fail "this is ${PRETTY_NAME:-an unknown distribution}"
  die "This task requires Ubuntu ${UBUNTU_VERSION_REQUIRED}. Other distributions are not supported."
fi

if [ "${VERSION_ID:-}" != "$UBUNTU_VERSION_REQUIRED" ]; then
  fail "Ubuntu ${VERSION_ID:-unknown} detected, but ${UBUNTU_VERSION_REQUIRED} is required"
  info "ROS 2 Jazzy and Gazebo Harmonic are only packaged for 24.04. Installing on"
  info "another release will not give you a working environment."
  die "unsupported Ubuntu release."
fi
ok "Ubuntu $VERSION_ID ($VERSION_CODENAME)"

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ -f "$REPO_ROOT/eyrc-sc-evaluator" ]; then
  if [ -x "$REPO_ROOT/eyrc-sc-evaluator" ]; then
    ok "the evaluator is present"
  else
    missing "the evaluator is not executable; fixing it"
    chmod +x "$REPO_ROOT/eyrc-sc-evaluator" 2>/dev/null &&
      ok "eyrc-sc-evaluator is now executable" ||
      fail "could not make eyrc-sc-evaluator executable"
  fi
else
  fail "eyrc-sc-evaluator is missing from $REPO_ROOT"
  info "Re-clone the repository; the evaluator is what a task is submitted with."
fi

ARCH=$(dpkg --print-architecture)
if [ "$ARCH" != "amd64" ]; then
  missing "architecture is $ARCH, not amd64"
  info "The prebuilt binaries in ur_description are amd64 only, so the simulation will"
  info "not run on this machine. The task requires a 64-bit x86 processor."
else
  ok "architecture $ARCH"
fi

apt_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

apt_available() {
  local candidate
  candidate=$(apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/ {print $2}')
  [ -n "$candidate" ] && [ "$candidate" != "(none)" ]
}

section "ROS 2 Jazzy prerequisites"

PREREQ_FAILED=0

if apt-cache policy 2>/dev/null | grep -q "packages.ros.org" ||
   grep -Rqs "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/; then
  ok "the ROS 2 apt repository is configured"
else
  fail "the ROS 2 apt repository is not configured"
  PREREQ_FAILED=1
fi

if [ -d "/opt/ros/$ROS_DISTRO_REQUIRED" ]; then
  ok "/opt/ros/$ROS_DISTRO_REQUIRED exists"
else
  fail "/opt/ros/$ROS_DISTRO_REQUIRED does not exist -- ROS 2 Jazzy is not installed"
  PREREQ_FAILED=1
fi

if apt_installed ros-jazzy-desktop-full; then
  ok "ros-jazzy-desktop-full is installed"
else
  fail "ros-jazzy-desktop-full is NOT installed"
  for variant in ros-jazzy-desktop ros-jazzy-ros-base ros-jazzy-simulation; do
    if apt_installed "$variant"; then
      info "found $variant instead -- that is not enough for this task"
    fi
  done
  PREREQ_FAILED=1
fi

if apt_installed ros-jazzy-rmw-fastrtps-cpp || apt_installed ros-jazzy-rmw-cyclonedds-cpp; then
  ok "an RMW implementation is installed"
else
  fail "no RMW implementation found"
  PREREQ_FAILED=1
fi

if [ "$PREREQ_FAILED" -eq 1 ]; then
  printf '\n'
  rule
  printf '  %sInstall ROS 2 Jazzy first%s\n' "$C_BOLD" "$C_RESET"
  rule
  cat <<EOF

  This script installs the task's dependencies on top of a working ROS 2 Jazzy. It does
  not install ROS itself, and it does not touch your apt sources -- that is part of the
  official installation, and doing it here would only hide a broken ROS install.

  When \`ros2 pkg list\` works in a new terminal, run this script again.

EOF
  exit 1
fi

APT_UPDATED=0
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  command -v sudo >/dev/null 2>&1 || die "sudo is not installed, and this is not running as root."
  SUDO="sudo"
fi

apt_update_once() {
  [ "$APT_UPDATED" -eq 1 ] && return 0
  info "refreshing the package lists..."
  $SUDO apt-get update -qq || die "apt-get update failed. Check your network connection."
  APT_UPDATED=1
}

if [ "$CHECK_ONLY" -eq 0 ]; then
  if [ -n "$SUDO" ]; then
    info "apt needs root; you may be asked for your password."
    $SUDO -v || die "could not obtain sudo privileges."
  fi
  apt_update_once
fi

TO_INSTALL=()
UNAVAILABLE=()

survey_group() {
  local label="$1"; shift
  section "$label"
  local pkg
  for pkg in "$@"; do
    if apt_installed "$pkg"; then
      ok "$pkg"
    elif apt_available "$pkg"; then
      missing "$pkg"
      TO_INSTALL+=("$pkg")
    else
      fail "$pkg -- not found in any configured apt repository"
      UNAVAILABLE+=("$pkg")
    fi
  done
}

survey_group "ros2_control and helpers" "${ROS_PACKAGES_ADDED[@]}"
survey_group "Gazebo, description and ROS client packages" "${ROS_PACKAGES_EXPECTED[@]}"
survey_group "Tools" "${TOOLS_PACKAGES[@]}"
survey_group "Python modules" "${PYTHON_PACKAGES[@]}"
survey_group "Shared libraries for the prebuilt binaries" "${SYSTEM_PACKAGES[@]}"
survey_group "Build tools" "${BUILD_PACKAGES[@]}"

section "Gazebo versions"
TO_UPGRADE=()
for entry in "${GAZEBO_TESTED_VERSIONS[@]}"; do
  pkg=${entry% *}
  want=${entry#* }
  have=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || true)
  have=${have%%-*}
  if [ -z "$have" ]; then
    continue   # not installed yet: the groups above install it, at the current version
  elif dpkg --compare-versions "$have" lt "$want"; then
    missing "$pkg $have, older than the tested $want"
    TO_UPGRADE+=("$pkg")
  elif dpkg --compare-versions "$have" gt "$want"; then
    ok "$pkg $have (newer than the tested $want -- report anything odd on the forum)"
  else
    ok "$pkg $have"
  fi
done
if [ "${#TO_UPGRADE[@]}" -gt 0 ]; then
  if [ "$CHECK_ONLY" -eq 1 ]; then
    info "would upgrade: ${TO_UPGRADE[*]}"
  else
    $SUDO apt-get install -y --only-upgrade "${TO_UPGRADE[@]}" ||
      die "the Gazebo upgrade failed. Run 'sudo apt update && sudo apt upgrade' and run this again."
    ok "Gazebo packages upgraded to the tested versions"
  fi
fi

section "Summary"

if [ "${#UNAVAILABLE[@]}" -gt 0 ]; then
  fail "${#UNAVAILABLE[@]} package(s) could not be found in any configured repository:"
  for pkg in "${UNAVAILABLE[@]}"; do
    info "  $pkg"
  done
  info "Your apt lists may be stale. Try:  sudo apt update  and run this again."
fi

if [ "${#TO_INSTALL[@]}" -eq 0 ]; then
  ok "nothing to install -- every task dependency is already present"
else
  printf '  %d package(s) to install:\n' "${#TO_INSTALL[@]}"
  for pkg in "${TO_INSTALL[@]}"; do
    info "  $pkg"
  done

  if [ "$CHECK_ONLY" -eq 1 ]; then
    printf '\n  would run: %s apt-get install -y %s\n' "$SUDO" "${TO_INSTALL[*]}"
    printf '  %sRun without --check to install them.%s\n\n' "$C_DIM" "$C_RESET"
    exit 0
  fi

  section "Installing"
  if ! $SUDO apt-get install -y "${TO_INSTALL[@]}"; then
    die "apt-get install failed. Scroll up for the package that caused it; if it is a held or broken package, 'sudo apt --fix-broken install' usually clears it."
  fi
  ok "packages installed"
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
  printf '\n'
  exit 0
fi

section "rosdep"
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  ok "rosdep is initialised"
else
  info "initialising rosdep..."
  $SUDO rosdep init >/dev/null 2>&1 || info "rosdep init reported an error; continuing"
fi

if rosdep update --rosdistro="$ROS_DISTRO_REQUIRED" >/dev/null 2>&1; then
  ok "rosdep database updated"
else
  info "rosdep update failed (usually a network hiccup); this is not fatal"
fi

printf '\n'
ok "all task dependencies are installed"
printf '\n'
