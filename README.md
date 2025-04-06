TUDU Blind Display - Foot Acceleration and Angle
This Arduino sketch is part of a final year project at Technological University Dublin. It reads foot Acceleration and Angle data in real-time using an Arduino Nano 33 IoT equipped with an IMU, OLED display, SD card, and BLE (Bluetooth Low Energy) communication.

Features
IMU Data Logging: Captures acceleration and calculates the angle y of the foot using a Kalman filter.
Real-Time Display: Shows live Y-axis angle on a 128x64 OLED screen.
Data Storage: Logs time-stamped angle and vertical acceleration to a CSV file on an SD card.
Bluetooth Streaming: Sends data over BLE using UART protocol for wireless analysis.
TU Dublin Splash Screen: Displays a custom startup animation with a sparkling "TU DUBLIN" logo.
Auto File Naming: Automatically creates new CSV log files (data0.csv, data1.csv, etc.).

Technologies & Libraries Used
ArduinoBLE.h – for Bluetooth Low Energy communication
Arduino_LSM6DS3.h – IMU sensor access
Adafruit_GFX.h + Adafruit_SSD1306.h – OLED display control
SPI.h + SD.h – SD card logging
Custom Kalman Filter – for smooth angle estimation from raw sensor data

Output Format (CSV)
Each entry contains:
text
Copy
Edit
Time(ms),Angle_Y(degrees),Acceleration_Y(m/s^2)

BLE Output Format
Sends values in this format over Bluetooth:
text
Copy
Edit
<angleY>,<accelerationY>
e.g., 12.56,9.81

Getting Started
Upload the sketch to your Arduino Nano 33 IoT

Connect:
OLED VCC → Pin 9
SD Card CS → Pin 10
Power up the device
Open Serial Monitor or connect to a BLE app
CSV logs are saved automatically to the SD card
