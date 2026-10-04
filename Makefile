# display-firmware-lab -- host-testable display firmware core mechanisms
#
# Everything here is pure C99 + libm.  Nothing is flashed anywhere.
#
# Targets:
#   make        build the simulator (results go to results/)
#   make test   build and run the self-checking test suite
#   make sim    run the end-to-end simulation and write results/
#   make clean  remove build products
#
# NOTE: this Makefile deliberately does NOT create directories -- the
# simulator creates results/ itself (see ensure_dir() in main_sim.c), because
# some Windows make builds have no working `mkdir -p`.

CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -ffp-contract=off
LDLIBS  ?= -lm

# -ffp-contract=off is not optional: without it clang (and newer gcc) may fuse
# a*b+c into an FMA, which changes the last bits of the EDID timing maths and
# the backlight gamma LUT, and the floating point assertions then fail on
# macOS only.  See README "pitfalls".
CFLAGS  += -ffp-contract=off

ifeq ($(OS),Windows_NT)
  EXE_SIM  := dfw_sim.exe
  EXE_TEST := dfw_tests.exe
else
  EXE_SIM  := dfw_sim
  EXE_TEST := dfw_tests
endif

SRC_COMMON := src/edid.c src/ddc_ci.c src/source_mux.c src/osd.c src/backlight.c

.PHONY: all test sim clean

all: $(EXE_SIM)

$(EXE_SIM): $(SRC_COMMON) src/main_sim.c src/dfw.h
	$(CC) $(CFLAGS) -Isrc -o $@ $(SRC_COMMON) src/main_sim.c $(LDLIBS)

$(EXE_TEST): $(SRC_COMMON) test/test_dfw.c src/dfw.h
	$(CC) $(CFLAGS) -Isrc -o $@ $(SRC_COMMON) test/test_dfw.c $(LDLIBS)

test: $(EXE_TEST)
	./$(EXE_TEST)

sim: $(EXE_SIM)
	./$(EXE_SIM) results

clean:
	-rm -f $(EXE_SIM) $(EXE_TEST)
	-rm -f *.o src/*.o test/*.o
