# GoPro MAX2 Setup and Streaming Guide

For the purpose of this project we use the GoPro Max2 camera. 

The GoPro Max2 is a 360 degree camera that shoots True 8K spherical video using dual lenses. The raw USB stream contains two fisheye lenses stitched together — not a normal flat video feed. This feed needs to be processed separately by OpenCV to separate the front and back frames.

The standard GoPro camera does not expose itself as a UVC cameras over USB on Linux. Instead the GoPro comes up as a network interface and the data stream from it via UDP. 

In order to communicate with the GoPro, we follow the following steps:

# Camera Setup Procedure
## 1. Connect the GoPro
Plug the GoPro to the RubikPi using a USB-C cable 


## 2. Verify Device Detection
- Run `lsusb` to confirm if the GoPro is detected by the RubikPi. You should see the GoPro listed as one of the devices. 
- Find the IP address of the GoPro using `ip a`
- Run `nmap -sn 172.28.114.0/24` to identify the GoPro.
- To enable live video stream on the GoPro run:
```bash
# Wake up the GoPro & enable USB control
curl -X GET "http://172.28.114.51:8080/gopro/camera/control/wired_usb?p=1"

# Start the preview/live stream
curl -X GET "http://172.28.114.51:8080/gopro/camera/stream/start"
```
Alternatively run `sh cam_setup.sh`

## 3. Test the Device
- Run `python3 cam_record.py` to test the GoPro Max2. You will find the recordings stored in `/recordings`