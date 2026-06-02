# Radar Setup and Data Parsing Guide

## Overview

For this project, we use a DesignCore TI mmWave radar, specifically the RS-1843AOPU / RS-6843AOPU style radar module.

The radar is used to detect objects in front of the scooter and output point cloud data that can later be translated into ROS2 messages. Unlike a normal sensor that outputs readable values, the radar streams raw binary data. This binary stream is split into radar frames using the TI mmWave magic word.

The radar connects to the RubikPi over USB. After the XR USB serial driver is installed, the radar appears as two serial ports:

```bash
/dev/ttyXRUSB0
/dev/ttyXRUSB1
```

In this setup:

```text
/dev/ttyXRUSB0 = configuration port
/dev/ttyXRUSB1 = data stream port
```

---

## Features

- Loads the XR USB serial driver for the radar
- Sends the radar configuration file to the config port
- Reads the radar binary stream from the data port
- Checks for the TI mmWave magic word
- Parses radar frames and TLV sections
- Extracts detected point data such as position, range, velocity, SNR, and noise
- Saves radar points for later visualization or ROS2 integration

---

## File Overview

### 1. `start/`

Contains startup scripts for initializing the radar.

These scripts are used to set up the radar serial ports and send the configuration file.

### 2. `radar_test/`

Contains the main radar testing and parsing code.

This includes scripts for reading raw radar data, parsing packets, collecting detected points, and saving data.

### 3. `RadarVisualizer/`

Contains code for plotting radar point cloud data.

This is used to check whether detected radar points make sense visually.

---

## Prerequisites

Install Python dependencies if needed:

```bash
pip install pyserial numpy pandas matplotlib
```

If `pip` is blocked because of the system Python environment, use a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
pip install pyserial numpy pandas matplotlib
```

---

## Radar Setup Procedure

## 1. Connect the Radar

Plug the radar into the RubikPi using USB.

Check that the device is detected:

```bash
lsusb
```

---

## 2. Load the XR USB Serial Driver

The radar requires the XR USB serial driver so it can appear as serial devices on the RubikPi.

If the driver is not already loaded automatically, load it with:

```bash
sudo modprobe xr_usb_serial_common
```

Verify that the serial ports appear:

```bash
ls /dev/ttyXRUSB*
```

Expected output:

```bash
/dev/ttyXRUSB0
/dev/ttyXRUSB1
```

---

## 3. Start and Configure the Radar

Once the radar ports are available, run the startup script:
```bash
sh start.sh
```

This script handles the radar initialization process automatically.

It does the following:
- Waits for the radar serial ports to appear
- Verifies that the config port and data port exist
- Checks that the radar configuration file exists
- Sets the serial speeds for both ports
- Sends the radar configuration file to the radar
- 
After the config file is sent, the radar streams binary data from:
/dev/ttyXRUSB1

## 4. Read the Raw Radar Stream


To confirm that the radar is streaming data, capture a short sample from the data port:

```bash
sudo timeout 5 cat /dev/ttyXRUSB1 > radar_raw.bin
```

Check that data was captured:

```bash
ls -lh radar_raw.bin
```

If the file size is greater than zero, the radar is streaming data.

---

## 5. Verify the Magic Word

TI mmWave radar frames start with a magic word:

```text
02 01 04 03 06 05 08 07
```

This byte sequence marks the beginning of each radar frame.

The radar stream is structured like this:

```text
[magic word][packet header][TLV data]
[magic word][packet header][TLV data]
[magic word][packet header][TLV data]
```

Finding the magic word confirms that the radar is sending valid frame packets.

---

## Parser Overview

The parser converts the raw radar byte stream into useful object data.

The parser process is:

```text
Read bytes from /dev/ttyXRUSB1
   ↓
Search for the magic word
   ↓
Find the start of a radar frame
   ↓
Read the packet header
   ↓
Use the packet length to extract a full frame
   ↓
Read each TLV section
   ↓
Extract detected point data
   ↓
Print or save the points
```

Each radar packet contains:

```text
magic word
packet header
TLV sections
```

The packet header contains information such as:

```text
frame number
packet length
number of detected objects
number of TLVs
```

The TLV sections contain the actual radar output. For this project, the important TLV is the detected point cloud.

The parser extracts values such as:

```text
x
y
z
range
velocity
SNR
noise
```

The parser also converts the raw coordinates into easier scooter-relative values:

```text
forward = distance in front of the radar
side    = left/right offset
height  = vertical offset
range   = total distance from the radar
```

Example output:

```text
Frame 120 | points=3

Point 0 | raw=(0.42, 1.85, 0.10) m | forward=1.85 m, side=0.42 m, height=0.10 m, range=1.90 m, v=-0.21 m/s, snr=18, noise=7
```

---

## Parsed Point Format

Each detected point is stored in a format similar to:

```python
{
    "id": int,
    "raw_x": float,
    "raw_y": float,
    "raw_z": float,
    "forward": float,
    "side": float,
    "height": float,
    "range": float,
    "velocity": float,
    "snr": int,
    "noise": int
}
```

---

## Radar Configuration Details

The current configuration has approximately:

```text
Carrier frequency:        77 GHz
Ramp slope:               100 MHz/us
ADC samples:              256
ADC sampling rate:        7.2 Msps
Max distance:             8.64 m
Range resolution:         0.042 m
Number of TX antennas:    3
Number of RX antennas:    4
```

The most important values are:

```text
Max range:          about 8.64 meters
Range resolution:   about 4.2 centimeters
```

---

## Useful Commands

Check USB devices:

```bash
lsusb
```

Check radar serial ports:

```bash
ls /dev/ttyXRUSB*
```

Check driver messages:

```bash
dmesg | grep -i xr
```

Load the driver:

```bash
sudo modprobe xr_usb_serial_common
```

Send the config:

```bash
while IFS= read -r line; do
    echo "$line" | sudo tee /dev/ttyXRUSB0 >/dev/null
    sleep 0.05
done < profile_3d_aop.cfg
```

Capture raw data:

```bash
sudo timeout 5 cat /dev/ttyXRUSB1 > radar_raw.bin
```

Run the parser:

```bash
python3 parser.py
```

Run data collection:

```bash
python3 dataCollecter.py
```

---

## Current Status

The radar driver has been installed on the RubikPi, and the radar is recognized as two XR USB serial ports.

The radar configuration file can be sent through the config port, and the radar streams raw binary data through the data port. The raw stream has been checked for the TI mmWave magic word, confirming that valid radar packets are being received.

A parser has been created to separate radar frames, read packet headers, parse TLV sections, and extract detected radar points.

The next step is to clean up the parser output and convert it into a ROS2 node so the radar data can be used by the rest of the scooter system.
