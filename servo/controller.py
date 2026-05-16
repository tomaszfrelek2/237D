import serial
import time
import threading

class ST3025_Controller:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.1):
        self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
        self._is_polling = False
        self._polling_thread = None

    def close(self):
        self.stop_polling()
        self.serial_port.close()

    def _write_packet(self, servo_id, instruction, parameters):
        length = len(parameters) + 2
        checksum_sum = servo_id + length + instruction + sum(parameters)
        checksum = (~checksum_sum) & 0xFF
        packet = [0xFF, 0xFF, servo_id, length, instruction] + parameters + [checksum]
        self.serial_port.write(bytearray(packet))

    def _read_packet(self, expected_data_len):
        total_bytes = expected_data_len + 6
        response = self.serial_port.read(total_bytes)
        
        if len(response) == total_bytes and response[0] == 0xFF and response[1] == 0xFF:
            error = response[4]
            data = list(response[5:-1])
            return error, data
            
        return None, None
    def set_torque(self, servo_id, enable):
        """
        Enables or disables the motor torque.
        enable=True : Servo stiffens, holds position, and accepts movement commands.
        enable=False: "Read-only" mode. Motor is limp and can be turned by hand.
        """
        # Register 0x28 is Torque Enable. Value 1 = On, 0 = Off.
        torque_val = 1 if enable else 0
        
        self.serial_port.reset_input_buffer()
        
        # Instruction 0x03 is WRITE. Parameters: [Register Address, Data Byte(s)]
        self._write_packet(servo_id, 0x03, [0x28, torque_val])
        
        # Read the standard 6-byte acknowledgment packet to clear the buffer
        error, data = self._read_packet(0)
        
        # Return True if there was no hardware error byte returned
        return error == 0
        
    def ping(self, servo_id):
        self.serial_port.reset_input_buffer()
        self._write_packet(servo_id, 0x01, []) 
        error, data = self._read_packet(0)
        return error is not None

    def get_feedback(self, servo_id):
        self.serial_port.reset_input_buffer()
        self._write_packet(servo_id, 0x02, [0x38, 12])
        error, data = self._read_packet(12)
        
        if data and len(data) == 12:
            position = data[0] | (data[1] << 8)
            
            speed_raw = data[2] | (data[3] << 8)
            speed = -(speed_raw & 0x7FFF) if (speed_raw & 0x8000) else speed_raw
            
            load_raw = data[4] | (data[5] << 8)
            load = -(load_raw & 0x03FF) if (load_raw & 0x0400) else (load_raw & 0x03FF)
            
            voltage = data[6] / 10.0
            temperature = data[7]
            move = data[9]
            current = data[10] | (data[11] << 8)
            
            return {
                "Position": position,
                "Speed": speed,
                "Load": load,
                "Voltage": voltage,
                "Temperature": temperature,
                "Move": move,
                "Current": current
            }
        return None

    # --- New Event-Driven Methods ---

    def start_polling(self, servo_id, callback_function, interval=0.1):
        """Starts a background thread that continuously reads data and triggers a callback."""
        if self._is_polling:
            return

        self._is_polling = True
        self._polling_thread = threading.Thread(
            target=self._poll_loop, 
            args=(servo_id, callback_function, interval),
            daemon=True # Daemon threads automatically close when the main program exits
        )
        self._polling_thread.start()

    def stop_polling(self):
        """Stops the background polling thread."""
        self._is_polling = False
        if self._polling_thread:
            self._polling_thread.join(timeout=1.0)

    def _poll_loop(self, servo_id, callback_function, interval):
        """The internal loop running on the background thread."""
        while self._is_polling:
            data = self.get_feedback(servo_id)
            if data:
                # Trigger the user's function and pass the data to it
                callback_function(data)
            time.sleep(interval)
