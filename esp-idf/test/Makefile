# Host-side tests for SUPE's pure core.
#
#   make          build, run, and regenerate golden.txt
#   make clean
#
# No ESP-IDF, no RadioLib, no FreeRTOS: src/supe.{h,cpp} is deliberately free of
# all three so it can be tested here. If this build starts needing an IDF header,
# something has leaked into the core and belongs in lora.cpp instead.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

all: run engine

supe_core_test: supe_core_test.cpp ../src/supe.cpp ../src/supe.h
	$(CXX) $(CXXFLAGS) -o $@ supe_core_test.cpp ../src/supe.cpp -lm

supe_engine_test: supe_engine_test.cpp ../src/supe.cpp ../src/supe_engine.cpp \
                  ../src/lora_queue.cpp ../src/supe.h ../src/supe_engine.h \
                  ../src/lora_queue.h
	$(CXX) $(CXXFLAGS) -o $@ supe_engine_test.cpp ../src/supe.cpp \
	    ../src/supe_engine.cpp ../src/lora_queue.cpp -lm

run: supe_core_test
	./supe_core_test golden.txt supe-ladder-vectors.txt

engine: supe_engine_test
	./supe_engine_test

clean:
	rm -f supe_core_test supe_engine_test

.PHONY: all run clean
