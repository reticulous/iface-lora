# Host-side tests for SUPE's pure core.
#
#   make          build, run, and regenerate golden.txt
#   make clean
#
# No ESP-IDF, no RadioLib, no FreeRTOS: src/supe.{h,cpp} and src/lora_rfcal.{h,cpp}
# are deliberately free of all three so they can be tested here. If this build
# starts needing an IDF header, something has leaked into a core and belongs in
# lora.cpp or lora_fem.cpp instead.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

all: run engine rfcal

supe_core_test: supe_core_test.cpp ../src/supe.cpp ../src/supe.h
	$(CXX) $(CXXFLAGS) -o $@ supe_core_test.cpp ../src/supe.cpp -lm

supe_engine_test: supe_engine_test.cpp ../src/supe.cpp ../src/supe_engine.cpp \
                  ../src/lora_queue.cpp ../src/supe.h ../src/supe_engine.h \
                  ../src/lora_queue.h
	$(CXX) $(CXXFLAGS) -o $@ supe_engine_test.cpp ../src/supe.cpp \
	    ../src/supe_engine.cpp ../src/lora_queue.cpp -lm

rfcal_test: rfcal_test.cpp ../src/lora_rfcal.cpp ../src/lora_rfcal.h
	$(CXX) $(CXXFLAGS) -o $@ rfcal_test.cpp ../src/lora_rfcal.cpp

run: supe_core_test
	./supe_core_test golden.txt supe-ladder-vectors.txt

engine: supe_engine_test
	./supe_engine_test

rfcal: rfcal_test
	./rfcal_test

clean:
	rm -f supe_core_test supe_engine_test rfcal_test

.PHONY: all run engine rfcal clean
