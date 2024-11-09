#include <Arduino.h> 
#include <MobaTools.h>
#include "makros.h"
#include "debug.h"
#include "RCReceive.h"

bool dataSent = false;
String cur_state;
int cur_map;
int stepperInput, mapPWM, statePWM;
float stepperOut, Setpoint1 = 0, Setpoint2 = 0, sensorOut1, sensorOut2, sensor1, sensor2;
const int serTX3 = 14;
const int serRX3 = 15;
const int stepPin = 7;
const int dirPin = 6;
const int motorStepPin2 = 40;
const int motorDirPin2 = 39;
const int thrusterSpeed[5] = {95, 110, 120, 130, 140};
const int ch1 = 3;
const int ch2 = 2;
const int ch3 = -1;
const int ch4 = -1;
const int ch5 = 21;
const int ch6 = 20;
RCReceive rc1, rc2, rc4, rc5, rc6;
int chRaw[7], chOut[7];
const int encoderPin1 = A0;
const int encoderPin2 = A1;

MoToServo esc;
const int escPin = A7;

const int minThrottle = 624;
const int maxThrottle = 2000;
int throttleValue = 0;

float angleData, escData;
const int stepsPerRevolution = 1600;  // Sesuaikan dengan pengaturan driver DM542

MoToStepper myStepper1(stepsPerRevolution, STEPDIR); 
MoToStepper myStepper2(stepsPerRevolution, STEPDIR);
int steps1, steps2;
int currentAngle = 0;


/* ############################  ATERLEVEN 2024  ############################## */

void setup() {
  Serial.begin(115200);   //
  Serial3.begin(115200);  //
  Serial1.begin(115200);  // 
  
  myStepper1.attach(stepPin, dirPin);
  myStepper1.setSpeed(400);
  myStepper1.setRampLen(100);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  
  myStepper2.attach(motorStepPin2, motorDirPin2);
  myStepper2.setSpeed(400);
  myStepper2.setRampLen(100);


  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);


  delay(1000);
  esc.attach(escPin, 1000, 2000);
  rc1.attachInt(ch1);
  rc2.attachInt(ch2);
  rc5.attachInt(ch5);
  rc6.attachInt(ch6);
}

void loop() {
  moveStepperToAngle();
  if (rc1.hasNP() && !rc1.hasError()) {
    chOut[1] = rc1.getValue();
  }
  if (rc2.hasNP() && !rc2.hasError()) {
    chOut[2] = rc2.getValue();
  }
  if (rc5.hasNP() && !rc5.hasError()) {
    mapPWM = rc5.getValue();
  }
  if (rc6.hasNP() && !rc6.hasError()) {
    statePWM = rc6.getValue();
  }
  readPWM();
  
}

void forwardESP(){
  // Proses data dari Serial1 (misalnya GPS)
  if (Serial1.available()) {
    String gpsData = Serial1.readStringUntil('\n');
    int gdSize = gpsData.length();
    gpsData[gdSize-1] = ',';
    gpsData = gpsData + " TAG: " + String(cur_map);
    // Serial1.println(gpsData);  // Menampilkan data GPS
  }  
}

void moveStepperToAngle() {
  sensor2 = analogRead(encoderPin2);
  sensor1 = analogRead(encoderPin1);
  sensorOut1 = map(sensor1, 455, 964, -90, 90);
  sensorOut2 = map(sensor2, 355, 869, -90, 90);
  if (Setpoint1 != sensorOut1 || Setpoint2 != sensorOut2) {  // Jika sudut baru berbeda dari sudut saat ini
    if (sensorOut1 >= -90 && sensorOut1 <= 90 || sensorOut2 >= -90 && sensorOut2 <= 90) {  // Cek apakah sudut berada dalam rentang yang diinginkan
      
      // Konversi sudut ke langkah (dari sudut saat ini ke sudut baru)
      steps1 = map(sensorOut1, -90, 90, -stepsPerRevolution / 4, stepsPerRevolution / 4)
            - map(Setpoint1, -90, 90, -stepsPerRevolution / 4, stepsPerRevolution / 4);
      steps2 = map(sensorOut2, -90, 90, -stepsPerRevolution / 4, stepsPerRevolution / 4)
            - map(Setpoint2, -90, 90, -stepsPerRevolution / 4, stepsPerRevolution / 4);

      // Putar kedua motor ke sudut baru
      myStepper1.move(steps1); 
      myStepper2.move(steps2);
      
      while (myStepper1.moving() || myStepper2.moving()) {
        // Tunggu sampai kedua motor selesai bergerak
      }

      sensorOut1 = Setpoint1;  // Update sudut saat ini
      sensorOut2 = Setpoint2;  // Update sudut saat ini
      // Serial.print("Motor berputar ke sudut: ");
      // Serial.println(newAngle);
      // Serial.println("Masukkan sudut berikutnya: ");
    } else {
      // Serial.println("Sudut di luar rentang. Masukkan sudut antara -90 dan 90.");
    }
  } else {
    // Jika sudut baru sama dengan sudut saat ini, tidak ada perubahan
    // Serial.println("Sudut yang dimasukkan sama dengan sudut saat ini. Tidak ada pergerakan.");
  }
  //Serial.println("sensor1 : " + String(sensor1) + "\tsensor2 : " + String(sensor2));
}

int roundToNearestStepperAngle(float rawAngle) {
  const int validAngles[] = {-90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 20, 25, 30, 35, 40, 45, 55, 60, 65, 70, 75, 80, 85, 90};
  int closestAngle = validAngles[0];
  int minDifference = abs(rawAngle - validAngles[0]);

  for (int i = 1; i < sizeof(validAngles) / sizeof(validAngles[0]); i++) {
    int difference = abs(rawAngle - validAngles[i]);
    if (difference < minDifference) {
      minDifference = difference;
      closestAngle = validAngles[i];
    }
  }

  return closestAngle;
}

void readPWM() {

  if (statePWM > 160) {
    cur_state = "AUTONOMOUS";
    autonomous();
  } else if (100 <= statePWM && statePWM <= 140) {
    cur_state = "NEUTRAL";
    Setpoint1 = 0;
    Setpoint2 = 0;
    esc.write(91);
  } else if (statePWM < 80) {
    cur_state = "RC";
    rc();
  }

  if (mapPWM > 1400) {
   cur_map = -1;
  } else if (1000 <= mapPWM && mapPWM <= 1400) {
    cur_map = 0;
    dataSent = false;
  } else if (mapPWM < 1000 && !dataSent) {
    cur_map = 1;
    forwardESP();
    dataSent = true;
  }

//   Serial.print("STATE : ");
//   Serial.print(cur_state);
//   Serial.print("\t");
//   Serial.print("MAPPING : ");
//   Serial.print(cur_map);
}




void processStepperInput() {
  stepperInput = chOut[1];
  stepperOut = map(stepperInput, 17, 243, -90, 90);

  Setpoint1 = stepperOut;
  Setpoint2 = stepperOut;
}

void processThrusterInput() {
  int thrusterInput = chOut[2];
  escData = map(thrusterInput, 8, 232, 0, 186);
  Serial.println(thrusterInput);
  esc.write(escData);
}



String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 2 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void rc() {
  processStepperInput();
  processThrusterInput();
}

void autonomous() {  
// Proses data dari Serial2 (misalnya dari NUC)
  Serial.println(Serial.available());
  if (Serial3.available() > 0) {
    String rawData = Serial3.readStringUntil('\n');

    // Memisahkan rawData menjadi angleData dan directionData
    escData = getValue(rawData, ',', 0).toFloat();  // Ambil data arah
    angleData = getValue(rawData, ',', 1).toFloat();  // Ambil data sudut dan ubah ke float
    String detectData = getValue(rawData, ',', 2);  // Ambil data detect
    int paksa = getValue(rawData, ',' , 3).toInt();
    String lintasan = getValue(rawData, ',', 4);
  //  Serial.print("Raw Angle: ");
  //  Serial.print(angleData);
  //  Serial.print("\tDirection: ");
  //  Serial.print(directionData);
  //  Serial.print("\tStepper Angle: ");
  //  Serial.println(stepperAngle);
  //  Serial.print("\tDetect: ");
  //  Serial.print(detectData);
  Serial.println(rawData);

    if (paksa == 0){
      if (detectData == "True") {
        Setpoint1 = angleData;
        Setpoint2 = angleData;
        if (escData > 0) {
          escData = map(escData, 1, 100, 100, 192);
          esc.write(escData);
        } else if (escData < 0) {
          escData = map(escData, -1, -100, 80, 0);
          esc.write(escData);
        } else if (escData == 0) {
          esc.write(91);
        }
      } else if (detectData == "False") {
        Setpoint1 = 0;
        Setpoint2 = 0;
        esc.write(91);
      }
    }
    elif (paksa == 1){
      esc.write(escData);
      delay(7000);
      if (lintasan == "A"){
        Setpoint1 = 90;
        Setpoint2 = 90;
        esc.write(escData);
        delay(5000);
      }
      if (lintasan == "B"){
        Setpoint1 = -90;
        Setpoint2 = -90;
        esc.write(escData);
        delay(5000);
      }
    }
  }
}
