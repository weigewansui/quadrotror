#! /bin/bash

cd bin
./logger &
STATE_LOGGER_PID=$!

./transmit_log &
TRANSMIT_LOG_PID=$!

cd ../

read KILL

kill $STATE_LOGGER_PID
kill $TRANSMIT_LOG_PID