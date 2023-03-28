#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <math.h>
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
bool readPreassure(){
  a1 = analogRead(analogPin1);  a2 = analogRead(analogPin2);  a3 = analogRead(analogPin3);  a4 = analogRead(analogPin4);
  a5 = analogRead(analogPin5);  a6 = analogRead(analogPin6);  a7 = analogRead(analogPin7);  a8 = analogRead(analogPin8);
}



void loop() {
  
  if (readPreassure()) {
    // pressureCal();
    // printPressure();
  }
  
  if (readIMU()) {
    long currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    doAngleCalculations();
    // printAngleCalculations();
    Position();
    printPosition();
    
  }
}

void pressureCal(){

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
  Serial.print("Analogue Readings:"); 
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
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;

  //Pedometer: Step Counter

}
void printAngleCalculations() {
  Serial.print("Roll: ");
  Serial.print(complementaryRoll);
  Serial.print(", \t Pitch: ");
  Serial.print(complementaryPitch);
  Serial.print(", \t Yaw: ");
  Serial.println(complementaryYaw);
}


void Position() {
  unsigned long curr_time = millis();
  float dt = (curr_time - prev_time) / 1000.0;  // Time interval in seconds
  prev_time = curr_time;

    // Integrate accelerometer readings to obtain velocity
    float x_vel = accelX * dt;
    float y_vel = accelY * dt;
    float z_vel = accelZ * dt;

    // Integrate velocity readings to obtain position
    x_pos += x_vel * dt;
    y_pos += y_vel * dt;
    z_pos += z_vel * dt;
    // Compensate for gyro drift
}


void printPosition(){
    Serial.print("X position: ");
    Serial.print(x_pos);
    Serial.print(" m, Y position: ");
    Serial.print(y_pos);
    Serial.print(" m, Z position: ");
    Serial.print(z_pos);
    Serial.println(" m,: ");
}
