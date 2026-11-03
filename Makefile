# SD-SPI emulator
#
#   make setup     install system packages, ESP-IDF and serial port access
#   make test      everything: laptop tests, build both firmwares, detect the
#                  two boards, flash them, run the hardware tests, print a summary
#   make sim       laptop tests only (no ESP-IDF, no boards)
#   make build     both firmware builds only
#   make hwtest    build, flash and hardware tests (no laptop tests)
#   make clean
#
# Optional: make test CARD_PORT=/dev/ttyACM0 HOST_PORT=/dev/ttyUSB0 IDF_EXPORT=~/esp/esp-idf/export.sh

export CARD_PORT HOST_PORT IDF_EXPORT

.PHONY: help setup test sim build hwtest clean
.DEFAULT_GOAL := help

help:
	@sed -n '3,10p' Makefile | sed 's/^# \{0,1\}//'

setup:
	@tools/setup.sh

test:
	@tools/run_tests.sh all

sim:
	@tools/run_tests.sh sim

build:
	@tools/run_tests.sh build

hwtest:
	@tools/run_tests.sh hw

clean:
	$(MAKE) -C test/sim clean
	rm -rf apps/card/build apps/host/build test-logs
