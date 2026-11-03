#!/usr/bin/env bash
#
# Install everything needed to build, flash and test the SD-SPI emulator:
#   1. system packages (compiler, cmake, ninja, python, ...)
#   2. ESP-IDF with the esp32 and esp32s3 toolchains
#   3. serial port access (dialout / uucp group)
#
#   tools/setup.sh [--yes] [--dry-run] [--no-system] [--idf-dir DIR] [--idf-version TAG]
#
# Safe to re-run: steps that are already done are skipped.

set -euo pipefail

IDF_VERSION=${IDF_VERSION:-v5.5.5}
IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
IDF_TARGETS="esp32,esp32s3"
ASSUME_YES=0
DRY_RUN=0
DO_SYSTEM=1

usage() {
    sed -n '3,11p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        -y|--yes) ASSUME_YES=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --no-system) DO_SYSTEM=0 ;;
        --idf-dir) IDF_DIR=$2; shift ;;
        --idf-version) IDF_VERSION=$2; shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown option: $1"; usage 1 ;;
    esac
    shift
done

if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'; else B=; G=; Y=; R=; N=; fi
step() { echo; echo "${B}==> $*${N}"; }
ok()   { echo "${G}ok${N}  $*"; }
warn() { echo "${Y}!!${N}  $*"; }
die()  { echo "${R}error:${N} $*" >&2; exit 1; }
run()  { echo "+ $*"; [ $DRY_RUN = 1 ] || "$@"; }

confirm() {
    [ $ASSUME_YES = 1 ] && return 0
    [ $DRY_RUN = 1 ] && return 0
    read -r -p "$1 [Y/n] " a
    [ -z "$a" ] || [[ "$a" =~ ^[Yy] ]]
}

[ "$(uname -s)" = Linux ] || die "this script supports Linux only (see README for macOS/Windows)"
[ "$(id -u)" != 0 ] || die "run as your normal user, not root (sudo is used where needed)"

SUDO=sudo
command -v sudo >/dev/null 2>&1 || SUDO=

# ---------------------------------------------------------------------------
# 1. system packages
# ---------------------------------------------------------------------------
if [ $DO_SYSTEM = 1 ]; then
    step "System packages"
    if command -v apt-get >/dev/null 2>&1; then
        PKGS=(git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache
              libffi-dev libssl-dev dfu-util libusb-1.0-0 build-essential)
        echo "Debian/Ubuntu: ${PKGS[*]}"
        if confirm "Install with apt-get?"; then
            run $SUDO apt-get update
            run $SUDO apt-get install -y "${PKGS[@]}"
        fi
    elif command -v dnf >/dev/null 2>&1; then
        PKGS=(git wget flex bison gperf python3 python3-pip cmake ninja-build ccache dfu-util libusbx gcc make)
        echo "Fedora: ${PKGS[*]}"
        if confirm "Install with dnf?"; then
            run $SUDO dnf install -y "${PKGS[@]}"
        fi
    elif command -v pacman >/dev/null 2>&1; then
        PKGS=(gcc git make flex bison gperf python cmake ninja ccache dfu-util libusb)
        echo "Arch: ${PKGS[*]}"
        if confirm "Install with pacman?"; then
            run $SUDO pacman -S --needed --noconfirm "${PKGS[@]}"
        fi
    else
        warn "unknown package manager: install git, gcc, make, cmake, ninja, python3 (+venv), flex, bison, gperf, ccache, libusb yourself"
    fi
fi

# ---------------------------------------------------------------------------
# 2. ESP-IDF
# ---------------------------------------------------------------------------
step "ESP-IDF $IDF_VERSION in $IDF_DIR"
if [ -d "$IDF_DIR/.git" ]; then
    have=$(git -C "$IDF_DIR" describe --tags --always 2>/dev/null || echo unknown)
    if [ "$have" = "$IDF_VERSION" ]; then
        ok "already checked out ($have)"
    else
        warn "found $have, expected $IDF_VERSION"
        if confirm "Switch $IDF_DIR to $IDF_VERSION?"; then
            run git -C "$IDF_DIR" fetch --depth 1 origin tag "$IDF_VERSION"
            run git -C "$IDF_DIR" checkout "$IDF_VERSION"
            run git -C "$IDF_DIR" submodule update --init --recursive --depth 1
        fi
    fi
else
    echo "about 2 GB download and disk space (repository + toolchains)"
    if confirm "Clone ESP-IDF $IDF_VERSION?"; then
        run mkdir -p "$(dirname "$IDF_DIR")"
        run git clone -b "$IDF_VERSION" --recursive --depth 1 --shallow-submodules \
            https://github.com/espressif/esp-idf.git "$IDF_DIR"
    fi
fi

step "ESP-IDF tools for $IDF_TARGETS"
if [ -x "$IDF_DIR/install.sh" ] || [ $DRY_RUN = 1 ]; then
    run "$IDF_DIR/install.sh" "$IDF_TARGETS"
else
    warn "$IDF_DIR/install.sh not found, skipping"
fi

# ---------------------------------------------------------------------------
# 3. serial port access
# ---------------------------------------------------------------------------
step "Serial port access"
GROUP=dialout
getent group dialout >/dev/null 2>&1 || GROUP=uucp      # Arch
if id -nG "$USER" | tr ' ' '\n' | grep -qx "$GROUP"; then
    ok "$USER is in $GROUP"
elif getent group "$GROUP" >/dev/null 2>&1; then
    if confirm "Add $USER to the $GROUP group (needed to open /dev/ttyUSB*, /dev/ttyACM*)?"; then
        run $SUDO usermod -aG "$GROUP" "$USER"
        warn "log out and back in (or reboot) for the group change to take effect"
    fi
else
    warn "no dialout/uucp group found; make sure you can open the serial ports"
fi

# ---------------------------------------------------------------------------
# check
# ---------------------------------------------------------------------------
step "Check"
if [ $DRY_RUN = 1 ]; then
    ok "dry run, nothing changed"
    exit 0
fi
if [ -f "$IDF_DIR/export.sh" ] && ( . "$IDF_DIR/export.sh" >/dev/null 2>&1 && idf.py --version ); then
    ok "ESP-IDF works"
else
    warn "could not run idf.py from $IDF_DIR/export.sh"
fi
for t in gcc make python3; do
    command -v $t >/dev/null 2>&1 && ok "$t" || warn "$t missing"
done

cat <<EOF

${B}Done.${N} Next:
  1. wire the two boards (README, "Hardware")
  2. plug both into USB (the S3 through its UART port)
  3. make test
EOF
[ "$IDF_DIR" = "$HOME/esp/esp-idf" ] || echo "  (non-default ESP-IDF location: make test IDF_EXPORT=$IDF_DIR/export.sh)"
