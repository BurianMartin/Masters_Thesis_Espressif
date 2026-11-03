#!/usr/bin/env bash
#
# Full test pipeline: laptop tests, firmware builds, board detection,
# flashing, and hardware tests against the stock ESP-IDF host.
#
#   tools/run_tests.sh [all|sim|build|hw]
#
# Environment (all optional):
#   CARD_PORT   serial port of the ESP32-S3 card   (auto-detected)
#   HOST_PORT   serial port of the classic ESP32   (auto-detected)
#   IDF_EXPORT  path to ESP-IDF's export.sh         (default: $IDF_PATH/export.sh, ~/esp/esp-idf/export.sh)
#
# Every stage logs to test-logs/<timestamp>/ and the run ends with a summary.
# Exit code: 0 if nothing failed.

set -u
MODE=${1:-all}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
LOGDIR="$ROOT/test-logs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOGDIR"
cd "$ROOT"

if [ -t 1 ]; then
    C_OK=$'\e[32m'; C_BAD=$'\e[31m'; C_SKIP=$'\e[33m'; C_DIM=$'\e[2m'; C_B=$'\e[1m'; C_0=$'\e[0m'
else
    C_OK=; C_BAD=; C_SKIP=; C_DIM=; C_B=; C_0=
fi

NAMES=(); RESULTS=(); NOTES=()
record() { NAMES+=("$1"); RESULTS+=("$2"); NOTES+=("${3:-}"); }

# stage "name" command... : run, log, record PASS/FAIL
stage() {
    local name=$1; shift
    local log="$LOGDIR/$(echo "$name" | tr -c 'A-Za-z0-9\n' '_').log"
    printf '%-46s ' "$name"
    if "$@" >"$log" 2>&1; then
        echo "${C_OK}PASS${C_0}"
        record "$name" PASS "$log"
        return 0
    fi
    echo "${C_BAD}FAIL${C_0}  ${C_DIM}$log${C_0}"
    tail -n 12 "$log" | sed "s/^/    ${C_DIM}/; s/\$/${C_0}/"
    record "$name" FAIL "$log"
    return 1
}

skip() {
    printf '%-46s %sSKIP%s  %s\n' "$1" "$C_SKIP" "$C_0" "$2"
    record "$1" SKIP "$2"
}

section() { echo; echo "${C_B}== $1${C_0}"; }

# ---------------------------------------------------------------------------
# ESP-IDF environment
# ---------------------------------------------------------------------------
idf_env() {
    command -v idf.py >/dev/null 2>&1 && return 0
    local f
    for f in "${IDF_EXPORT:-}" "${IDF_PATH:+$IDF_PATH/export.sh}" "$HOME/esp/esp-idf/export.sh"; do
        if [ -n "$f" ] && [ -f "$f" ]; then
            # shellcheck disable=SC1090
            . "$f" >"$LOGDIR/idf_export.log" 2>&1
            command -v idf.py >/dev/null 2>&1 && return 0
        fi
    done
    return 1
}

# ---------------------------------------------------------------------------
# stages
# ---------------------------------------------------------------------------
run_sim() {
    make -C test/sim clean >/dev/null && make -C test/sim test
}

build_app() {   # dir target
    local dir=$1 target=$2
    if [ ! -f "$dir/sdkconfig" ] || ! grep -q "CONFIG_IDF_TARGET=\"$target\"" "$dir/sdkconfig"; then
        idf.py -C "$dir" set-target "$target" || return 1
    fi
    idf.py -C "$dir" build
}

chip_on_port() {   # prints ESP32, ESP32-S3, ... or nothing
    python -m esptool --port "$1" --before default_reset --after hard_reset chip_id 2>&1 |
        sed -n 's/.*Detecting chip type[. ]*\(ESP32[-A-Z0-9]*\).*/\1/p' | head -n1
}

detect_boards() {
    local p chip
    : > "$LOGDIR/detect.log"
    for p in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$p" ] || continue
        if [ ! -w "$p" ]; then
            echo "$p: no write permission (add yourself to the dialout group, then log in again)" >> "$LOGDIR/detect.log"
            continue
        fi
        chip=$(chip_on_port "$p")
        echo "$p: ${chip:-unknown}" >> "$LOGDIR/detect.log"
        case "$chip" in
            ESP32-S3) [ -z "${CARD_PORT:-}" ] && CARD_PORT=$p ;;
            ESP32)    [ -z "${HOST_PORT:-}" ] && HOST_PORT=$p ;;
        esac
    done
}

flash_app() {   # dir port
    idf.py -C "$1" -p "$2" flash
}

hw_host_info() {
    python tools/sdemu.py host --host "$HOST_PORT" info
}

hw_scenarios() {
    python tools/sdemu.py run --card "$CARD_PORT" --host "$HOST_PORT" \
        --summary "$LOGDIR/scenarios.txt" tools/scenarios/*.yaml
}

hw_card_status() {
    python tools/sdemu.py card --card "$CARD_PORT" status
}

# ---------------------------------------------------------------------------
# pipeline
# ---------------------------------------------------------------------------
echo "${C_B}SD-SPI emulator test run${C_0}  ($(date '+%Y-%m-%d %H:%M'), logs in ${LOGDIR#$ROOT/})"

SIM_OK=1; BUILD_OK=1; ENV_OK=1

if [ "$MODE" = all ] || [ "$MODE" = sim ]; then
    section "Laptop tests (engine against simulated ESP-IDF host)"
    stage "sim: protocol test bench" run_sim || SIM_OK=0
    summary_line=$(grep -h "test groups passed" "$LOGDIR"/sim*.log 2>/dev/null | tail -n1)
    [ -n "$summary_line" ] && NOTES[${#NOTES[@]}-1]="$summary_line"
fi

if [ "$MODE" = all ] || [ "$MODE" = build ] || [ "$MODE" = hw ]; then
    section "Firmware"
    if idf_env; then
        record "ESP-IDF environment" PASS "$(idf.py --version 2>/dev/null | head -n1)"
        printf '%-46s %sPASS%s  %s\n' "ESP-IDF environment" "$C_OK" "$C_0" "${NOTES[${#NOTES[@]}-1]}"
        stage "build: card firmware (ESP32-S3)" build_app apps/card esp32s3 || BUILD_OK=0
        stage "build: host firmware (ESP32)" build_app apps/host esp32 || BUILD_OK=0
    else
        ENV_OK=0; BUILD_OK=0
        printf '%-46s %sFAIL%s  %s\n' "ESP-IDF environment" "$C_BAD" "$C_0" \
            "idf.py not found: install ESP-IDF v5.5 or set IDF_EXPORT=/path/to/export.sh"
        record "ESP-IDF environment" FAIL "idf.py not found (set IDF_EXPORT)"
    fi
fi

if [ "$MODE" = all ] || [ "$MODE" = hw ]; then
    section "Hardware"
    if [ $ENV_OK = 0 ]; then
        skip "boards" "no ESP-IDF environment"
    elif [ $BUILD_OK = 0 ]; then
        skip "boards" "firmware did not build"
    else
        detect_boards
        sed 's/^/    /' "$LOGDIR/detect.log"
        if [ -n "${CARD_PORT:-}" ]; then
            record "detect: ESP32-S3 card" PASS "$CARD_PORT"
            printf '%-46s %sPASS%s  %s\n' "detect: ESP32-S3 card" "$C_OK" "$C_0" "$CARD_PORT"
        else
            record "detect: ESP32-S3 card" FAIL "not found (or set CARD_PORT=)"
            printf '%-46s %sFAIL%s  %s\n' "detect: ESP32-S3 card" "$C_BAD" "$C_0" "not found (or set CARD_PORT=)"
        fi
        if [ -n "${HOST_PORT:-}" ]; then
            record "detect: ESP32 host" PASS "$HOST_PORT"
            printf '%-46s %sPASS%s  %s\n' "detect: ESP32 host" "$C_OK" "$C_0" "$HOST_PORT"
        else
            record "detect: ESP32 host" FAIL "not found (or set HOST_PORT=)"
            printf '%-46s %sFAIL%s  %s\n' "detect: ESP32 host" "$C_BAD" "$C_0" "not found (or set HOST_PORT=)"
        fi

        if [ -n "${CARD_PORT:-}" ] && [ -n "${HOST_PORT:-}" ]; then
            FLASH_OK=1
            stage "flash: card -> $CARD_PORT" flash_app apps/card "$CARD_PORT" || FLASH_OK=0
            stage "flash: host -> $HOST_PORT" flash_app apps/host "$HOST_PORT" || FLASH_OK=0
            if [ $FLASH_OK = 1 ]; then
                if stage "hw: host initializes the emulated card" hw_host_info; then
                    info=$(grep -o 'name=[^ ]* sectors=[0-9]*' "${NOTES[${#NOTES[@]}-1]}" | head -n1)
                    NOTES[${#NOTES[@]}-1]="$info"
                    stage "hw: fault scenarios" hw_scenarios
                    if [ -f "$LOGDIR/scenarios.txt" ]; then
                        while read -r res name; do
                            [ "$res" = PASS ] && c=$C_OK || c=$C_BAD
                            printf '    %-42s %s%s%s\n' "$name" "$c" "$res" "$C_0"
                            record "  scenario: $name" "$res"
                        done < "$LOGDIR/scenarios.txt"
                    fi
                else
                    echo "    ${C_DIM}card not initialized: check the 5 wires (SCLK 18-12, MOSI 23-11, MISO 19-13, CS 5-10, GND)${C_0}"
                    skip "hw: fault scenarios" "host could not initialize the card"
                fi
                stage "hw: card diagnostics" hw_card_status
                diag="$LOGDIR/hw__card_diagnostics.log"
                if [ -f "$diag" ]; then
                    grep -E '^(cycles|transport|pcnt)' "$diag" | sed 's/^/    /'
                fi
            else
                skip "hw: tests" "flashing failed"
            fi
        else
            skip "hw: flash and tests" "both boards are needed"
        fi
    fi
fi

# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
section "Summary"
pass=0; fail=0; skipped=0
for i in "${!NAMES[@]}"; do
    case "${RESULTS[$i]}" in
        PASS) c=$C_OK; pass=$((pass + 1)) ;;
        FAIL) c=$C_BAD; fail=$((fail + 1)) ;;
        *)    c=$C_SKIP; skipped=$((skipped + 1)) ;;
    esac
    note=${NOTES[$i]}
    [[ "$note" == "$LOGDIR"* ]] && note=""
    printf '  %s%-4s%s  %-44s %s\n' "$c" "${RESULTS[$i]}" "$C_0" "${NAMES[$i]}" "$note"
done
echo
echo "${C_B}$pass passed, $fail failed, $skipped skipped${C_0}   logs: ${LOGDIR#$ROOT/}"
{
    for i in "${!NAMES[@]}"; do echo "${RESULTS[$i]} ${NAMES[$i]}"; done
} > "$LOGDIR/summary.txt"
[ $fail -eq 0 ]
