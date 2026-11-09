#!/bin/sh
# Runs the IS11 logic tests and the motion simulation on a PC (gcc), no
# Pico needed.  The firmware itself is still built with build_make/.
#   sh tools/host_test/run.sh
set -e
cd "$(dirname "$0")"
A=../../app_program
CF="-std=gnu11 -Wall -Wno-unused-function -DTM_WIFI_CYW43=1 -Iinc -I$A -I$A/subsystem1"
gcc $CF -DAPP_TEST=1 -o hosttest main.c stubs.c \
  $A/subsystem1/telemetry_codec.c $A/subsystem1/subsystem1_test.c \
  $A/subsystem2/motion_ctrl.c $A/subsystem2/subsystem2_test.c \
  $A/subsystem3/line_sense.c $A/subsystem3/subsystem3_test.c \
  $A/subsystem4/hump.c $A/subsystem4/subsystem4_test.c \
  $A/subsystem5/scan_profile.c $A/subsystem5/ctrl_logic.c \
  $A/subsystem5/drv_ultrasonic.c $A/subsystem5/subsystem5_test.c
./hosttest | grep -E "FAIL|SUMMARY|info"
gcc $CF -w -o simmotion simmotion.c $A/subsystem2/task_motion.c \
  $A/subsystem2/motion_ctrl.c $A/subsystem2/motion_api.c -lm
./simmotion
python3 ../is11_mqtt.py selftest
rm -f hosttest simmotion
# Sensor bench (app_program/sensor_bench.c) against a scripted user:
# "good" hardware must pass every check, "bad" must catch 6 faults
gcc $CF -w -DAPP_TEST=1 -Dapp_log=sim_log -o benchsim benchsim.c $A/sensor_bench.c $A/subsystem4/hump.c -lm
./benchsim good | tail -1
./benchsim bad | tail -1
rm -f benchsim
