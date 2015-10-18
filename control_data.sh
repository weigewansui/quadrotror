#! /bin/bash

cd bin

./Vicon_state&
VICON_PID=$!

# 
# ./control_test &
# CONTROL_PID=$!

./aci_recvar &
RECEIVE=$!

cd ../

lcm-logger &
LCM_LOGGER_PID=$!

read KILL

kill $VICON_PID
kill $RECEIVE
kill $LCM_LOGGER_PID
# kill $CONTROL_PID