# CSE237D - Embedded Systems Design Project (Spring 2026)
## Autonomous Scooter Hardware Project

**Tomasz Frelek** · **Ryan Chon** · **Gayathri Rajesh**

## Setting up the RUBIK Pi
RUBIK Pi 3 is a powerful, affordable, and flexible Edge AI platform, powered by the Qualcomm Dragonwing™ QCS6490, delivering 12 TOPS of efficient on-device AI performance. More details about the specifications of the RUBIK Pi can be found at <a href="https://www.thundercomm.com/rubik-pi-3/en/docs/rubik-pi-3-user-manual/1.1.0/quick-start/">https://www.thundercomm.com/rubik-pi-3/en/docs/rubik-pi-3-user-manual/1.1.0/quick-start/</a>.

### Installing Ubuntu 24.04 on the RUBIK Pi
- Visit the Qualcomm Software Centre webportal <a href="https://softwarecenter.qualcomm.com/catalog/item/Qualcomm_Launcher">https://softwarecenter.qualcomm.com/catalog/item/Qualcomm_Launcher</a>
- Choose the OS type and architecture type of your machine and download the latest version of the launcher
- Install the launcher from the downloaded folder
- In the launcher choose Rubik Pi as the development kit and Ubuntu Server as the target Operating System.
- Switch the device to EDL mode. Find instructions to switch to EDL mode <a href="https://www.thundercomm.com/rubik-pi-3/en/docs/rubik-pi-3-user-manual/1.0.0-u/set-up-your-device#enter-into-edl-mode">here</a>. Once the device is in EDL mode, it will automatically be detected. Click the flash button to begin updating the Renesas USB firmware.
- After the OS is sucessfully flashed, a confirmation message will appear on the screen. 
- Connect the RUBIK Pi to your system via a MicroUSB cable and configure the Wi-Fi.

## Setting up the Pi in AP mode
Rubik Pi allows to run STA and AP mode at the same time. You’ll need to first create a virtual interface for the AP:
```bash
sudo iw dev wlan0 interface add ap0 type __ap
```
Then you can set up AP mode:
```bash
sudo nmcli connection add type wifi ifname ap0 con-name ScooterMan ssid ScooterMan mode ap
```
Check the connection using
```bash
nmcli connection show --active
```

## Setting up the virtual environment
To install various packages on the RUBIK Pi, create a virtual environment and install all the packages inside this environment.
```bash
python3 -m venv cam_env
```

To activate this environment
```bash
source cam_env/bin/activate
```

## Hardware Specifications
- Camera: GoPro Max2 
- Radar: DesignCore mmWave radar
- Steering System: Waveshare ST3025 Serial Bus Servo 
- Microcontroller: RUBIK Pi 3 

