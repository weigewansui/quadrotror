#! /bin/bash

cd bin
./aci_recvar &
RECEIVE=$!

#./bin/logger & #file will be created as root directory
# ./logger &
# C_LOGGER_PID=$! 

cd ../

lcm-logger &
LCM_LOGGER_PID=$!

read KILL

kill $RECEIVE
kill $LCM_LOGGER_PID
# kill $C_LOGGER_PID
