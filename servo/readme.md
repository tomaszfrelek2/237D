# ST3025 Servo Driver & Readout Utility

A lightweight, multi-threaded Python interface to communicate with Waveshare/Feetech **ST3025 serial bus servos**. The driver implements low-level packet construction (checksum logic, register reading/writing) and offers an asynchronous, event-driven polling loop to stream live telemetry without blocking your main application thread.

## Features

- **Low-level Serial Protocol:** Handles custom packet packing, packet structure validation, bit-shifting for multi-byte values (Position, Speed, Load, Current), and directional/negative flag handling.
- **Non-Blocking Telemetry:** Utilizes a background daemon thread (`threading.Thread`) to continuously poll servo telemetry at a custom interval.
- **Event-Driven Callbacks:** Automatically routes incoming servo telemetry to a user-defined callback function.
- **Torque Management:** Easily toggle between active holding/control mode and limp "read-only" manual positioning mode.

---

## File Overview

### 1. `controller.py`
Contains the core `ST3025_Controller` class. It manages serial port lifecycles, parses packet responses, and handles background polling logic.

### 2. `readout.py`
An example implementation script. It handles command-line configuration, pings the hardware, sets the desired torque state, and starts the background callback loop.

---

## Prerequisites & Installation

Ensure you have `pyserial` installed:

```bash
pip install pyserial
```

If you are running on Linux, ensure your user has permissions to access the serial interface (e.g., adding your user to the `dialout` group):

```bash
sudo usermod -aG dialout $USER
```
*(Log out and back in for changes to take effect).*

---

## Usage Example

The readout script accepts an integer argument (`1` or `0`) to enable or disable motor torque on startup.

### Running with Torque Disabled (Read-Only Mode)
Perfect for manually moving the servo by hand while streaming position data in real-time.

```bash
python readout.py 0
```

### Running with Torque Enabled (Control Mode)
Stiffens the motor and holds its current position.

```bash
python readout.py 1
```

---

## API Summary

### `ST3025_Controller` Methods

| Method | Description |
| :--- | :--- |
| `ping(servo_id)` | Returns `True` if the servo responds to an existence check. |
| `set_torque(servo_id, enable)` | `True` stiffens the motor. `False` sets it limp for manual positioning. |
| `get_feedback(servo_id)` | Fetches a raw 12-byte telemetry chunk and parses it into a clean dictionary. |
| `start_polling(servo_id, callback, interval)` | Spawns a background thread that invokes `callback(data)` every `interval` seconds. |
| `stop_polling()` | Cleanly terminates the background polling loop. |
| `close()` | Safely stops polling and terminates the serial socket link. |

### Telemetry Dictionary Format

The callback function receives a dictionary containing the following keys:

```python
{
    "Position": int,      # Current pulse position
    "Speed": int,         # Directed rotational velocity (signed)
    "Load": int,          # Directional load metric (signed)
    "Voltage": float,     # Operating voltage (V)
    "Temperature": int,   # Internal temperature (Celsius)
    "Move": int,          # Movement flag status
    "Current": int        # Active current draw
}
```
