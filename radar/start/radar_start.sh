#!/bin/bash

CONFIG_PORT="/dev/ttyXRUSB0"
DATA_PORT="/dev/ttyXRUSB1"
CFG_FILE="/home/ubuntu/radar/radar_driver/2000039 (TI) xwr1843aop firmware-v03_06_01_00-LTS-1/profile_3d_aop.cfg"

echo "Waiting for radar ports..."

for i in {1..30}; do
    if [ -e "$CONFIG_PORT" ] && [ -e "$DATA_PORT" ]; then
        echo "Radar ports found."
        break
    fi
    sleep 1
done

if [ ! -e "$CONFIG_PORT" ]; then
    echo "Config port not found: $CONFIG_PORT"
    exit 1
fi

if [ ! -e "$DATA_PORT" ]; then
    echo "Data port not found: $DATA_PORT"
    exit 1
fi

if [ ! -f "$CFG_FILE" ]; then
    echo "Config file not found: $CFG_FILE"
    exit 1
fi

echo "Setting serial speeds..."
stty -F "$CONFIG_PORT" 115200 raw -echo -ixon -ixoff -crtscts
stty -F "$DATA_PORT" 921600 raw -echo -ixon -ixoff -crtscts

echo "Sending radar config..."
while IFS= read -r line; do
    echo "$line"
    printf "%s\r\n" "$line" > "$CONFIG_PORT"
    sleep 0.1
done < "$CFG_FILE"

echo "Radar config sent."