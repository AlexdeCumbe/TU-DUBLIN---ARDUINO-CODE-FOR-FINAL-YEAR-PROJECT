#include <Wire.h>
#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_LSM6DS3.h>
#include <SPI.h>
#include <SD.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define SD_CS_PIN 10  
File dataFile;
int fileIndex = 0;
String fileName;
unsigned long startTime;

#define OLED_POWER_PIN 9  // Connect this pin to OLED VCC

BLEService BLEUartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic BLEUartTX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 20);
BLECharacteristic BLEUartRX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);

class KalmanFilter {
public:
    KalmanFilter() {
        Q_angle = 0.01;
        Q_bias = 0.002;
        R_measure = 0.03;
        angle = 0;
        bias = 0;
        P[0][0] = 1;
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 1;
    }
    float getAngle(float newAngle, float newRate, float dt) {
        rate = newRate - bias;
        angle += dt * rate;
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
        return angle;
    }
private:
    float Q_angle, Q_bias, R_measure, angle, bias, rate;
    float P[2][2];
};

KalmanFilter kalman;
float offset = 0;

void showSplashScreen() {
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(48, 10);
    display.println("TU");
    display.setCursor(10, 40);
    display.println("DUBLIN");
    display.display();

    unsigned long sparkleStartTime = millis();  // Start time for sparkles
    while (millis() - sparkleStartTime < 5000) {  // Display for 5 seconds
        int x = random(5, 120);
        int y = random(5, 60);
        display.drawPixel(x, y, WHITE);  // Draw a sparkling pixel
        display.display();
        delay(50);  // Delay between each spark for visibility

        // Keep the "TU DUBLIN" text on screen continuously
        display.setTextSize(3);
        display.setTextColor(WHITE);
        display.setCursor(48, 10);
        display.println("TU");
        display.setCursor(10, 40);
        display.println("DUBLIN");
        display.display();
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    pinMode(OLED_POWER_PIN, OUTPUT);
    digitalWrite(OLED_POWER_PIN, HIGH);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
    } else {
        showSplashScreen();  // Show TU DUBLIN splash for 5 sec
    }

    if (!IMU.begin()) {
        Serial.println("IMU initialization failed!");
    }

    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
    } else {
        BLE.setLocalName("Nano33_BLE");
        BLE.setAdvertisedService(BLEUartService);
        BLEUartService.addCharacteristic(BLEUartTX);
        BLEUartService.addCharacteristic(BLEUartRX);
        BLE.addService(BLEUartService);
        BLE.advertise();
        Serial.println("BLE UART Ready!");
    }

    Serial.print("Initializing SD Card... ");
    delay(1000);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("FAILED!");
        return;
    }
    Serial.println("Success!");

    while (SD.exists("data" + String(fileIndex) + ".csv") && fileIndex < 99) {
        fileIndex++;
    }
    fileName = "data" + String(fileIndex) + ".csv";
    
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Time(ms),Angle_Y(degrees),Acceleration_Y(m/s^2)");
        dataFile.close();
        Serial.println("New file created: " + fileName);
    } else {
        Serial.println("⚠️ ERROR: Could not create file!");
    }

    calibrateOffset();
    startTime = millis();
}

void calibrateOffset() {
    float ax, ay, az, angleY, sum = 0.0;
    for (int i = 0; i < 100; i++) {
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(ax, ay, az);
            angleY = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
            sum += angleY;
        }
        delay(10);
    }
    offset = sum / 100;
    Serial.print("Offset Calibration Complete. Offset: ");
    Serial.println(offset);
}

void logData(unsigned long timestamp, float angleY, float ay) {
    dataFile = SD.open(fileName, FILE_WRITE);
    if (!dataFile) {
        Serial.println("⚠️ ERROR: Unable to write to SD card!");
        return;
    }

    unsigned long elapsedTime = timestamp - startTime;
    Serial.print("Time (ms): ");
    Serial.println(elapsedTime);

    dataFile.print(elapsedTime);
    dataFile.print(",");
    dataFile.print(angleY, 2);
    dataFile.print(",");
    dataFile.println(ay, 2);
    dataFile.close();
}

void loop() {
    BLE.poll();
    float ax, ay, az, gx, gy, gz, angleY;
    unsigned long now = millis();
    float dt = (now - startTime) / 1000.0;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        angleY = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
        angleY = kalman.getAngle(angleY, gx, dt) - offset;

        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(20, 0);
        display.print("Angle Y:");
        display.setCursor(30, 20);
        display.print(angleY, 2);
        display.setCursor(20, 40);
        display.print("degrees");
        display.display();

        String bleData = String(angleY, 2) + "," + String(ay, 2) + "\n";
        BLEUartRX.writeValue(bleData.c_str());
        Serial.println(bleData);

        logData(now, angleY, ay);
    }
    delay(10);
}
