#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <math.h>
#include <stdbool.h>
#include "Fusion.h"


float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;

int analogPin1 = A0, analogPin2= A1, analogPin3 = A2, analogPin4 = A3,
    analogPin5 = A4, analogPin6 = A5, analogPin7 = A6,analogPin8 = A7; //Analog Pins


int a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0, a7 = 0, a8 = 0; // variable to store the value read

float percent = 0, sum = 0; //Pressure Calculation
float w1 = 0, w2 = 0, w3 = 0, w4 = 0, w5 = 0, w6 = 0, w7 = 0, w8 = 0;

// Position
float x_pos = 0, y_pos = 0, z_pos = 0;  // Position along x,y,z-axis in meters
unsigned long prev_time = 0;

#define SAMPLE_PERIOD (104)

void setup() {
  Serial.begin(1000000);           //  setup serial 9600
    pinMode(LED_BUILTIN, OUTPUT);

  // this sketch will wait until something connects to serial!
  // this could be 'serial monitor', 'serial plotter' or 'processing.org P3D client' (see ./processing/RollPitchYaw3d.pde file)
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  calibrateIMU(250, 250);

  lastTime = micros();


}
/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}

void loop() {
  
  
  //   pressureCal();
  //   printPressure();
  
  if (readIMU()) {
    long currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    doAngleCalculations();
    // printAngleCalculations();
    // Position();
    // printPosition();
    
  }
  delay(1);
}

void pressureCal(){

  a1 = analogRead(analogPin1);  a2 = analogRead(analogPin2);  a3 = analogRead(analogPin3);  a4 = analogRead(analogPin4);
  a5 = analogRead(analogPin5);  a6 = analogRead(analogPin6);  a7 = analogRead(analogPin7);  a8 = analogRead(analogPin8);

  a1 = 1023 - a1;  a2 = 1023 - a2;  a3 = 1023 - a3;  a4 = 1023 - a4;  
  a5 = 1023 - a5;  a6 = 1023 - a6;  a7 = 1023 - a7;  a8 = 1023 - a8;
  percent = 1;
  w1 = a1*percent;  w2 = a2*percent;  w3 = a3*percent;  w4 = a4*percent;
  w5 = a5*percent;  w6 = a6*percent;  w7 = a7*percent;  w8 = a8*percent;

  sum = (w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8);
  percent = 100 / sum;
  if(sum < 100){
    percent = 0;
  }
  w1 = w1*percent;  w2 = w2*percent;  w3 = w3*percent;  w4 = w4*percent;
  w5 = w5*percent;  w6 = w6*percent;  w7 = w7*percent;  w8 = w8*percent;

}
void printPressure(){
  // Serial.print("Analogue Readings:"); 
  // Serial.print(w1);
  // Serial.print(",\t");
  // Serial.print(w2);
  // Serial.print(",\t");
  // Serial.print(w3);
  // Serial.print(",\t");
  // Serial.print(w4);
  // Serial.print(",\t");
  // Serial.print(w5);
  // Serial.print(",\t");
  // Serial.print(w6);
  // Serial.print(",\t");
  // Serial.print(w7);
  // Serial.print(",\t");
  // Serial.println(w8);

  Serial.print("Percent Readings:"); 
  Serial.print(w1);
  Serial.print(",\t");
  Serial.print(w2);
  Serial.print(",\t");
  Serial.print(w3);
  Serial.print(",\t");
  Serial.print(w4);
  Serial.print(",\t");
  Serial.print(w5);
  Serial.print(",\t");
  Serial.print(w6);
  Serial.print(",\t");
  Serial.print(w7);
  Serial.print(",\t");
  Serial.println(w8);
}

void doAngleCalculations() {

  FusionAhrs ahrs;
  FusionAhrsInitialise(&ahrs);
  const FusionVector gyroscope = {gyroX, gyroY, gyroZ}; // replace this with actual gyroscope data in degrees/s
  const FusionVector accelerometer = {accelX, accelY, accelZ}; // replace this with actual accelerometer data in g
  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  Serial.print("Roll: ");
  Serial.print(euler.angle.roll);
  Serial.print(", \t Pitch: ");
  Serial.print(euler.angle.pitch);
  Serial.print(", \t Yaw: ");
  Serial.println(euler.angle.yaw);


}
// void printAngleCalculations() {
//   Serial.print("Roll: ");
//   Serial.print(euler.angle.roll);
//   Serial.print(", \t Pitch: ");
//   Serial.print(euler.angle.pitch);
//   Serial.print(", \t Yaw: ");
//   Serial.println(euler.angle.yaw);
// }
