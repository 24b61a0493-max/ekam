---
PublishDate: 2026-01-07
Title: Autonomous Rail Patrol Robot for Track Health Analysis and Targeted Maintenance
Excerpt: A compact self-driving robot that patrols railway tracks, detects flaws instantly, and repairs them before they turn into disasters.
image: myosa-cover-image.jpg
Tags: 
  - #TrackBot
  - #SmartRailwayAutomation
  - #SafeRails 
  - #MYOSA4_0
Tagline: "Sensorially guarding every inch of the track."
Acknowledgements: 
  We express our sincere gratitude to our Faculty Mentor, Dr. Raman Kumar sir, for his guidance and
  technical support throughout the project. We thank the Department of Electronics and
  Communication Engineering and the management of NMREC for providing essential resources and a
  supportive environment.
  "->We also appreciate the IEEE MYOSA Event 4.0 organizers for promoting innovation and practical
  learning. Finally, we thank our team members for their dedication and teamwork, and we are
  motivated to further improve our project after being selected for the first round of the finals..
---
## Image

<p align="center">
  <img src="/myosa-cover-image.jpg" width="800" />
</p>



# Overview
The Autonomous Rail Patrol Robot for Track Health Analysis and Targeted Maintenance is developed
to improve railway safety by automating the inspection of railway tracks. At present, track inspection
is mostly done manually, which can be slow, risky, and prone to mistakes. This project introduces an
autonomous robot that moves along railway tracks, continuously monitoring track conditions and
detecting faults at an early stage. By identifying problems before they become serious, the system
helps prevent accidents and reduces maintenance delays.

# Key Features
1. Intelligent Defect Detection & Classification
2. Real-Time Alerts & Dashboard Visualization
3. Scalable IoT-Enabled Railway Safety System
4. Autonomous defect detection, classification, and response system

# Demo/Examples
Images
<p align="center">
  <img src="/myosa-rover-components-view.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-top-view.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-front-view.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-back-view.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-side-view.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-crack-detection-dashboard.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-dashboard.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-dashboard-.alerts.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-dashboard-IR.jpg" width="800">
</p>

<p align="center">
  <img src="/myosa-dashboard-log.jpg" width="800">
</p>

<p align="center">
  <video width="800" controls>
    <source src="/myosa-robot-demo.mp4" type="video/mp4">
  </video>
  <br>
  <em>Autonomous rail patrol robot performing real-time track inspection and defect detection</em>
</p>


# Features (Detailed) 

## 1.Intelligent Defect Detection & Classification
The system uses intelligent algorithms to continuously process real-time data collected from multiple onboard sensors, including track geometry sensors, vibration sensors, and rail condition sensors. These algorithms filter noise, extract meaningful parameters, and analyze patterns to accurately identify track defects at an early stage. Once a defect is detected, the system classifies it as either minor or major based on severity, risk level, and predefined safety thresholds. This early and accurate classification helps prevent small issues from developing into dangerous failures.

## 2. Real-Time Alerts & Dashboard Visualization
For major faults that pose a high safety risk, the system instantly generates alerts and sends them to railway maintenance teams along with accurate location details obtained through GPS and track mapping. At the same time, all sensor data and defect information are transmitted to a real-time web-based dashboard. This dashboard allows railway authorities to visualize current track conditions, monitor detected defects, and analyze historical data trends. By enabling predictive and preventive maintenance, the system supports data-driven decision-making and enhances long-term railway infrastructure reliability.

## 3. Scalable IoT-Enabled Railway Safety System
The system is designed as a scalable IoT-enabled railway safety solution using wireless communication and a modular architecture. Multiple autonomous inspection robots can be deployed across large railway networks, each continuously collecting and transmitting data to a centralized dashboard. This unified monitoring platform allows railway authorities to track real-time conditions, manage defects efficiently, and plan maintenance activities. The scalable design enables easy expansion by adding more robots, ensuring cost-effective, reliable, and efficient infrastructure management across extensive rail networks.

## 4. Autonomous defect detection, classification, and response system
The robot intelligently identifies track defects, distinguishes between minor and major issues, performs automatic on-site repairs for minor faults using a robotic arm, and instantly alerts maintenance teams with real-time dashboard visualization for major faults and predictive maintenance analysis.

# Usage Instructions
1. Place the robot on the track and power on the ESP32 and sensors.
2. Robot moves autonomously, collecting data with IR sensors, MPU6050, and GPS.
3. Live data is displayed on the dashboard with defect alerts and location.
4. Minor repairs (tightening bolts, welding cracks) are done automatically by the robotic arm.
5.  For major defects, the robot stops and sends an alert to authorities.
# Code used
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "MPU6050.h"
#include <TinyGPSPlus.h"

/* ================= PIN CONFIG ================= */
// Sensors
#define IR_SENSOR 34

// Motors
#define MOTOR_L1 14
#define MOTOR_L2 27
#define MOTOR_R1 26
#define MOTOR_R2 25

// Robotic Arm (2 Servos)
#define SERVO_BASE 13
#define SERVO_ARM 12

// Safety & Status
#define BUZZER 33
#define STATUS_LED 2        // ESP32 onboard LED
#define EMERGENCY_BTN 32    // Manual emergency stop

// GPS
#define GPS_RX 16
#define GPS_TX 17

/* ================= WIFI ================= */
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverURL = "http://YOUR_PC_IP:5000/data";

/* ================= OBJECTS ================= */
Servo servoBase, servoArm;
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* ================= SAFETY FLAGS ================= */
bool emergencyStop = false;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(9600);

  pinMode(IR_SENSOR, INPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);

  pinMode(MOTOR_L1, OUTPUT);
  pinMode(MOTOR_L2, OUTPUT);
  pinMode(MOTOR_R1, OUTPUT);
  pinMode(MOTOR_R2, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  servoBase.attach(SERVO_BASE);
  servoArm.attach(SERVO_ARM);
  servoBase.write(0);
  servoArm.write(0);

  Wire.begin();
  mpu.initialize();

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(500);
  }

  digitalWrite(STATUS_LED, HIGH); // System ready
}

/* ================= MOVEMENT ================= */
void moveForward() {
  digitalWrite(MOTOR_L1, HIGH);
  digitalWrite(MOTOR_L2, LOW);
  digitalWrite(MOTOR_R1, HIGH);
  digitalWrite(MOTOR_R2, LOW);
}

void stopRobot() {
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, LOW);
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, LOW);
}

/* ================= ROBOTIC ARM ================= */
void fixMinorDefect() {
  servoBase.write(90);
  delay(800);
  servoArm.write(90);
  delay(1500);
  servoArm.write(0);
  delay(800);
  servoBase.write(0);
}

/* ================= DASHBOARD DATA ================= */
void sendData(String defect, String severity, float lat, float lon) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"defect\":\"" + defect + "\",";
    payload += "\"severity\":\"" + severity + "\",";
    payload += "\"latitude\":" + String(lat, 6) + ",";
    payload += "\"longitude\":" + String(lon, 6);
    payload += "}";

    http.POST(payload);
    http.end();
  }
}

/* ================= EMERGENCY HANDLER ================= */
void handleEmergencyStop() {
  stopRobot();
  digitalWrite(BUZZER, HIGH);
  digitalWrite(STATUS_LED, LOW);
  sendData("Manual Emergency Stop", "Critical", 0.0, 0.0);
  while (1);   // MYOSA-safe halt
}

/* ================= MAIN LOOP ================= */
void loop() {

  // Manual emergency stop (MYOSA safety)
  if (digitalRead(EMERGENCY_BTN) == LOW) {
    handleEmergencyStop();
  }

  // GPS Read
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  float latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  float longitude = gps.location.isValid() ? gps.location.lng() : 0.0;

  moveForward();

  int irValue = digitalRead(IR_SENSOR);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float vibration = abs(ax) + abs(ay);

  // -------- MINOR DEFECT --------
  if (irValue == LOW && vibration < 15000) {
    stopRobot();
    sendData("Minor crack / Loose fastener", "Low", latitude, longitude);
    fixMinorDefect();
    delay(1500);
  }

  // -------- MAJOR DEFECT (FAIL-SAFE) --------
  else if (vibration >= 15000) {
    stopRobot();
    digitalWrite(BUZZER, HIGH);
    sendData("Major crack / Track damage", "High", latitude, longitude);
    while (1);   // MYOSA-compliant safe stop
  }

  delay(400);
# Tech stack
## 1. MYOSA Motherboard – ESP32
The ESP32 MYOSA motherboard is the brain of your robot. It controls all the sensors and actuators, processes the data, and communicates with the dashboard via Wi-Fi. All other components connect to it, making it the central control unit.

## 2. Sensors
**Gyroscope**  
The gyroscope detects the robot’s rotation or tilt. It helps the robot know which direction it is turning or if it is leaning, which is important for balance and navigation on tracks.  

**Accelerometer**  
The accelerometer measures the robot’s acceleration and movement. It tells the ESP32 how fast the robot is moving or if it suddenly stops, which can help detect shocks or collisions.  

**Temperature Sensor**  
The temperature sensor measures the environment or motor temperature. It can alert if the robot or motors are overheating to prevent damage.  

**Pressure Sensor**  
The pressure sensor measures atmospheric pressure. This is useful for detecting weather conditions or elevation changes in the track area.  

**Altitude Sensor**  
The altitude sensor calculates the robot’s height above sea level. Combined with the pressure sensor, it helps track elevation changes in the railway tracks.  

## 3. Display – OLED
The OLED display is a small screen connected to the ESP32. It shows real-time data like sensor readings (temperature, pressure, GPS location) or robot status, allowing quick monitoring without a computer.

## 4. How They Work Together
All these MYOSA sensors send data to the ESP32 motherboard:  
- Gyroscope + accelerometer help with movement and tilt detection.  
- Temperature ensures the robot doesn’t overheat.  
- Pressure + altitude track environmental conditions and height.  
- The OLED display shows this data in real-time.  
The ESP32 can also send all this information over Wi-Fi to a dashboard for live monitoring.

## 3. Motors and Servo
**Servo Motor (MYOSA Component):** A small motor controlled by the ESP32 with PWM signals. It does precise movements, like pushing switches, controlling small mechanical parts, or making adjustments on the track.  

**12V DC Motor + Motor Driver:** This is the main motor that moves the robot. The motor driver protects the ESP32 from high current and allows control over speed and direction.

## 4. Power Supply
The robot uses separate power lines:  
- ESP32 & sensors: 3.3V–5V  
- DC Motor: 12V  
This separation ensures safety and prevents the microcontroller from being damaged by the motor’s high power.

## 5. Software
The robot is programmed using Arduino IDE or PlatformIO. MYOSA components have libraries and functions that make coding easy:  
- Servo.h for servo control  
- TinyGPS++ for GPS data  
- PWM control for DC motor  
- Reading IR sensors with digitalRead() or interrupts  
- Sending data to a web dashboard

## 6. Communication
The ESP32 can send live data to a dashboard using Wi-Fi. This allows you to monitor:  
- The robot’s location (GPS)  
- Obstacle detection (IR sensors)  
- Motor and servo actions  
Protocols like HTTP or MQTT help the ESP32 communicate with the dashboard.

## 7. Dashboard
The dashboard shows real-time robot status:  
- Frontend: Built with React.js or Next.js  
- Backend: Node.js, Flask, or Firebase to store and process data  
- Map: GPS location displayed with Google Maps API or Leaflet.js

# Requirements/Installation
## 1. Hardware Requirements
To build your MYOSA robot, the main hardware requirement is the ESP32 MYOSA motherboard, which acts as the brain of the system. You also need IR sensors to detect lines or obstacles, and a GPS module to track the robot’s location. For movement, a 12V DC motor with a motor driver is required, and a servo motor is used for precise mechanical tasks like adjusting levers or switches. To monitor orientation and motion, you need a gyroscope and accelerometer. Environmental sensing requires a temperature sensor, pressure sensor, and altitude sensor. Finally, an OLED display is needed to show real-time sensor data, and a suitable power supply is required—3.3–5V for the ESP32 and sensors, and 12V for the DC motor.

## 2. Software Requirements
For programming the ESP32, you need the Arduino IDE or PlatformIO. Specific libraries are also required for the MYOSA components, such as Servo.h for servo control, TinyGPS++ for GPS data, Adafruit_SSD1306 for the OLED display, and Adafruit_MPU6050 or BMP280 for the gyroscope, accelerometer, pressure, and altitude sensors. Optional libraries like MQTT or HTTP clients are needed if you want to send real-time data to a dashboard over Wi-Fi.

## 3. Miscellaneous Requirements
You also need connecting wires, breadboard or PCB for connections, and basic tools to assemble the robot. A USB cable is required to upload the program to the ESP32. Proper wiring and insulation are important to prevent short circuits and ensure the robot works reliably.

# Installation Steps
## Step 1: Connect the Sensors and Actuators
- Connect the IR sensors to the ESP32’s GPIO pins.  
- Connect the GPS module to the ESP32 using UART (TX/RX pins).  
- Connect the servo motor to a PWM pin on the ESP32.  
- Connect the 12V DC motor to a motor driver, then connect the motor driver to the ESP32.  
- Connect the gyroscope, accelerometer, pressure, and altitude sensors to the ESP32 using the I2C interface.  
- Connect the OLED display to the ESP32 via I2C.

## Step 2: Install Arduino IDE and Libraries
- Download and install the Arduino IDE on your computer.  
- Add ESP32 board support in the Arduino IDE.  
- Install all required libraries for your MYOSA components, including:  
  - Servo.h for servo control  
  - TinyGPS++ for GPS module  
  - Adafruit_SSD1306 for OLED display  
  - Adafruit_MPU6050 or BMP280 for gyroscope, accelerometer, pressure, and altitude sensors  
  - Optional: MQTT or HTTP libraries for Wi-Fi communication

## Step 3: Upload the Program to ESP32
- Connect the ESP32 to your computer using a USB cable.  
- Open your code in the Arduino IDE and select the correct ESP32 board and COM port.  
- Upload the program to the ESP32.

## Step 4: Power On and Test the Robot
- Turn on the robot and check that all sensors give correct readings.  
- Verify that the motors and servo respond properly.  
- Check the OLED display to ensure live data is shown.  
- Test Wi-Fi communication to confirm that data is being sent to the dashboard.

## Step 5: Ready for Operation
Once all components are tested and working correctly, the robot is ready for:  
- Autonomous navigation  
- Track monitoring  
- Environmental sensing (temperature, pressure, altitude, tilt, and motion)

# Environment / Setup Requirements
- Laptop or PC – To run the dashboard and Python scripts for data processing.  

- Stable Wi-Fi Connection – For wireless data transmission between ESP32 and dashboard.




