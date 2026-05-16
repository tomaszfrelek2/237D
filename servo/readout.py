import serial
import time
from controller import ST3025_Controller
from sys import argv

def on_new_servo_data(data):
    """This function is automatically called every time new data arrives."""
    print(f"[EVENT] New Data -> Pos: {data['Position']} | Temp: {data['Temperature']}C | Load: {data['Load']}")

if __name__ == '__main__':
    enable_torque = int(argv[1])
    if enable_torque:
        torque_info = "torque is enabled"
    else:
        torque_info = "torque is disabled"
    SERVO_ID = 1
    USB_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110532-if00'
    
    try:
        servo = ST3025_Controller(port=USB_PORT, baudrate=1000000)
    except serial.SerialException as e:
        print(f"Failed to open port: {e}")
        exit(1)

    if not servo.ping(SERVO_ID):
        print("Servo not responding.")
        servo.close()
        exit(1)

    print("Starting background data stream...")
    
    # 1. Start asking the servo for data 10 times a second (0.1s interval)
    print("Disabling motor torque...")
    servo.set_torque(SERVO_ID, enable=enable_torque)
    # 2. Every time it gets an answer, run the 'on_new_servo_data' function
    servo.start_polling(SERVO_ID, on_new_servo_data, interval=0.1)

    try:
        # Your main program is now completely free!
        # You can run a web server, read a camera, or do math here without being blocked.
        while(True):

            print(torque_info)
            time.sleep(1) 
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Cleanly shut down the background thread and close the port
        servo.close()
        print("Disconnected.")
