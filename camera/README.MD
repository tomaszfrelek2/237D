For the purpose of this project we use the GoProMAX2 camera. 

The standard GoPro camera does not expose itself as a UVC cameras over USB on Linux. Instead the GoPro comes up as a network interface and the data stream from it via UDP. 

In order to communicate with the GoPro, we follow the following steps:

- Plug the GoPro to the RubikPi using a USB-C cable 
- Run `lsusb` to confirm if the GoPro is detected by the RubikPi