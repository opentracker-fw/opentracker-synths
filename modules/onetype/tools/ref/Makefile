# The 1-Type reference (host only): renders the engine test's cases through
# Oxo Synth2's EngineCore and writes ../../tests/onetype_ref.inc.
#   make OXO=<oxo_synth2 checkout at commit 6bb4435>
#   make OXO=<...> tables     the engine's tables (stdout)
CXX   ?= g++
BUILD ?= build
INC   := ../../tests/onetype_ref.inc

all: $(INC)

$(BUILD)/ref1type: ref1type.cpp ../../tests/onetype_features.h ../../src/onetype_params.h ../sy1/sy1_map.h
	@test -n "$(OXO)" || { echo "set OXO=<oxo_synth2 checkout at 6bb4435>"; exit 1; }
	@mkdir -p $(BUILD)
	$(CXX) -std=c++20 -O2 -Wall -Wextra -Wno-missing-field-initializers -I$(OXO) ref1type.cpp -o $@

$(INC): $(BUILD)/ref1type
	$(BUILD)/ref1type > $@.tmp && mv $@.tmp $@

tables: $(BUILD)/ref1type
	$(BUILD)/ref1type tables

.PHONY: all tables
