#include "TinyGPS++.h"
#include "QMC5883LCompass.h"
#include "MPU6050.h"
#include "Wire.h"

// GPS and Compass initialization
TinyGPSPlus gps;
QMC5883LCompass compass;
// MPU6050 mpu;

// Pin for voltage sensor
const int voltagePin = 34;  // ESP32 analog pin (use any ADC-capable pin)
const float referenceVoltage = 3.3;  // Reference voltage for the sensor (ESP32 uses 3.3V)
const float voltageDividerRatio = 5.0;  // Voltage divider ratio (e.g., 5:1)

// Structure to hold sensor data
struct SensorData {
  float latitude;
  float longitude;
  float speed_ms;
  int azimuth;
  float voltage;
//  float accel_x;
//  float accel_y;
} sensorData;

unsigned long previousMillis = 0;  // For storing the previous time
const long interval = 1000;        // Interval for sending data in milliseconds (1 second)

// Function to send data over UART (using Serial2 with remapped pins)
void sendDataOverUART() {
  // Prepare the data string
  String data = "Lat: " + String(sensorData.latitude, 6) + 
                ", Lng: " + String(sensorData.longitude, 6) + 
                ", Speed(m/s): " + String(sensorData.speed_ms, 2) + 
                ", Azimuth: " + String(sensorData.azimuth);
                // ", Voltage: " + String(sensorData.voltage, 2);
                //", Accel X: " + String(sensorData.accel_x, 2) + 
                //", Accel Y: " + String(sensorData.accel_y, 2);

  // Send data via Serial2 (using remapped TX and RX pins)
  Serial2.println(data);
  Serial2.flush();
  Serial.println("Data sent over UART: ");
  Serial.println(data);
}

// Function to read voltage from analog pin
float readVoltage() {
  int sensorValue = analogRead(voltagePin);  // Read analog value
  float voltage = (sensorValue * referenceVoltage) / 4095.0;  // Convert to voltage for ESP32 (12-bit ADC)
  voltage *= voltageDividerRatio;  // Apply voltage divider ratio
  return voltage;
}

void setup() {
  Serial.begin(115200);     // Serial monitor (connected to USB)

  // UART2 for external communication (e.g., sending data)
  Serial2.begin(9600, SERIAL_8N1, 5, 4);  // Use GPIO17 as TX, GPIO16 as RX for Serial2

  // UART1 for GPS module
  Serial1.begin(38400, SERIAL_8N1, 18, 19);  // Use GPIO18 as RX and GPIO19 as TX for GPS

  // I2C for compass and MPU6050 (ESP32 I2C defaults to GPIO21 SDA and GPIO22 SCL)
  Wire.begin(21, 22);  // Initialize I2C (SDA, SCL)

  // Initialize compass and MPU6050
  compass.init();
  compass.setSmoothing(10, true);

  /*mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connected");
  }*/
}

void loop() {
  // Check if it's time to send data (interval timing control)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time data was sent
    previousMillis = currentMillis;

    // Read data from GPS module
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }

    // Update GPS data
    if (gps.location.isUpdated()) {
      sensorData.latitude = gps.location.lat();
      sensorData.longitude = gps.location.lng();
      sensorData.speed_ms = gps.speed.mps();
    }

    // Read compass data
    compass.read();
    sensorData.azimuth = compass.getAzimuth();

    // Read voltage data
    sensorData.voltage = readVoltage();

    // Read accelerometer data (MPU6050)
    int16_t ax, ay, az;
//    mpu.getAcceleration(&ax, &ay, &az);
//    sensorData.accel_x = ax / 16384.0;  // Convert to g (assuming default sensitivity of ±2g)
//    sensorData.accel_y = ay / 16384.0;

    // Send data over UART
    sendDataOverUART();
  }
}
