#include <Arduino_LSM6DS3.h>

#define INTERVAL_MS 10  // Sampling interval in milliseconds

float accelX = 0, accelY = 0, accelZ = 0;  // Accelerometer reading in m/s^2
float gyroX = 0, gyroY = 0, gyroZ = 0; // Gyroscope reading in degrees/s

float x_pos = 0, y_pos = 0, z_pos = 0;  // Position along x,y,z-axis in meters
unsigned long prev_time = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM6DS3!");
    while (1);
  }
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
    gyroX -= accelX * 180.0 / PI;
    gyroY -= accelY * 180.0 / PI;
    gyroZ -= accelZ * 180.0 / PI;

    Serial.print("X position: ");
    Serial.print(x_pos);
    Serial.print(" m, Y position: ");
    Serial.print(y_pos);
    Serial.print(" m, Z position: ");
    Serial.print(z_pos);
    Serial.print(" m, X gyro: ");
    Serial.print(gyroX);
    Serial.print(" deg/s, Y gyro: ");
    Serial.print(gyroY);
    Serial.print(" deg/s, Z gyro: ");
    Serial.print(gyroZ);
    Serial.println(" deg/s");
  }
