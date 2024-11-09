#include "TinyGPS++.h"
#include "QMC5883LCompass.h"
#include "Wire.h"
#include <Arduino.h>

TinyGPSPlus gps;
QMC5883LCompass compass;
int ch5 = 32;  // Pin D32 for PWM input

struct SensorData {
  float latitude;
  float longitude;
  float speed_ms;
  int azimuth;
  float voltage;
  int TAG;
} sensorData;

unsigned long previousMillis = 0;
const long interval = 1000;

// Flag to check if data has been sent
bool dataSent = false;

void sendDataOverUART() {
  String data = "Lat: " + String(sensorData.latitude, 6) + 
                ", Lng: " + String(sensorData.longitude, 6) + 
                ", Speed(m/s): " + String(sensorData.speed_ms, 2) + 
                ", Azimuth: " + String(sensorData.azimuth) +
                ", TAG: " + String(sensorData.TAG);

  Serial.println(data);
  Serial.flush();
  Serial2.println("Data sent over UART: ");
  Serial2.println(data);
}

int readInput(int m) {
  int cur_state = 0;

  if (m >= 98 && m <= 196) {
    cur_state = -1; 
  } else if (m >= 85 && m <= 98) {
    cur_state = 0;
  } else if (m < 80) {
    cur_state = 1;
  }
  
  return cur_state;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 5, 4);
  Serial1.begin(38400, SERIAL_8N1, 18, 19); // Assuming GPS is connected to Serial1 (pins 18 and 19)
  Wire.begin(21, 22);
  compass.init();
  compass.setSmoothing(10, true);
  pinMode(ch5, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  int mapPWM = pulseIn(ch5, HIGH); // Measure the duration of the high pulse on pin D32
  
  // Continuously read data from GPS

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Check if GPS data is valid before using it
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()) {
      sensorData.latitude = gps.location.lat();
      sensorData.longitude = gps.location.lng();
      sensorData.speed_ms = gps.speed.mps();
    }

    compass.read();
    sensorData.azimuth = compass.getAzimuth();
    sensorData.TAG = readInput(mapPWM);  // Determine TAG based on PWM value

    // Check if the PWM value exceeds 2045 and data has not been sent
    if (mapPWM > 2045 && !dataSent) {
      sendDataOverUART(); // Send GPS and sensor data over UART
      dataSent = true;    // Set the flag to indicate data has been sent
    } else if (mapPWM <= 2045) {
      dataSent = false;   // Reset the flag if PWM value drops below 2045
    }
  }
}
