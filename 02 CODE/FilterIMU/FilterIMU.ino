#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <math.h>
// #include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  Serial.begin(1000000);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!IMU.begin()) {
  Serial.println("Failed to initialize IMU!");
  while (1);

  // lastTime = micros();

}



  // // start the IMU and filter
  // CurieIMU.begin();
  // CurieIMU.setGyroRate(25);
  // CurieIMU.setAccelerometerRate(25);
  // filter.begin(25);

  // // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}
  float aix, aiy, aiz;
  float gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;

bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(aix, aiy, aiz);
    IMU.readGyroscope(gix, giy, giz);
    return true;
  }
  return false;
}

void loop() {

  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    // CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

      if (readIMU()) {
        // update the filter, which computes orientation
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
        
        // filter.updateIMU(gix, giy, giz, aix, aiy, aiz);

        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        Serial.print("Orientation: ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(roll);

        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
    

    
  }

    // // convert from raw data to gravity and degrees/second units
    // ax = convertRawAcceleration(aix);
    // ay = convertRawAcceleration(aiy);
    // az = convertRawAcceleration(aiz);
    // gx = convertRawGyro(gix);
    // gy = convertRawGyro(giy);
    // gz = convertRawGyro(giz);


  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
