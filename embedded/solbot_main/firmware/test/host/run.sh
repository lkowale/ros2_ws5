#!/bin/sh
# Host unit tests for actuator + power logic (needs ArduinoJson from `pio run` libdeps).
set -e
cd "$(dirname "$0")"
AJ=../../.pio/libdeps/solbot_main/ArduinoJson/src
g++ -std=c++17 -O1 -Wall -Wno-unused-function -I. -I"$AJ" -o /tmp/solbot_logic_test \
    test_logic.cpp ../../src/actuator.cpp ../../src/power.cpp
/tmp/solbot_logic_test
