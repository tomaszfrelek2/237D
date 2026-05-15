#!/bin/bash

DATA_PORT="/dev/ttyXRUSB1"
OUT_FILE="$HOME/radar/radar_test/raw_test.bin"
CAPTURE_SECONDS=10
MAGIC_WORD="0201040306050807"

echo "=== Radar Read Test ==="

if [ ! -e "$DATA_PORT" ]; then
    echo "ERROR: Data port not found: $DATA_PORT"
    echo "Check with: ls /dev/ttyXRUSB*"
    exit 1
fi

echo "Capturing radar data for $CAPTURE_SECONDS seconds..."
sudo timeout "$CAPTURE_SECONDS" cat "$DATA_PORT" > "$OUT_FILE"

echo
echo "Capture complete."
echo "Output file:"
ls -lh "$OUT_FILE"

echo
echo "First 20 lines of hexdump:"
hexdump -C "$OUT_FILE" | head -20

echo
echo "Searching for TI magic word:"
MATCHES=$(xxd -p "$OUT_FILE" | tr -d '\n' | grep -o -b "$MAGIC_WORD" | head)

if [ -z "$MATCHES" ]; then
    echo "No magic word found."
    echo "Data may not be valid TI radar packets, or radar may not be configured yet."
else
    echo "$MATCHES"
    echo
    echo "SUCCESS: TI radar packets found."
fi