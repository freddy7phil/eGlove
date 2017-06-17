#!/bin/sh -e

./sendReadings 1 &

PID=$!
python SPP-loopback_eglove.py

if ! kill $PID > /dev/null 2>&1; then
    echo "Closing program..." >&2
fi