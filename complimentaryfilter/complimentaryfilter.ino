//attempt to make a complimentary filter
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Angle variables
float accAngleX, accAngleY; // Accelerometer angles
float gyroAngleX, gyroAngleY; // Gyroscope angles
float compAngleX, compAngleY; // Complementary filter angles
float gyroX, gyroY; // Gyroscope rates
unsigned long prevTime;
float dt; // Time interval

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) delay(100);
  }
  Serial.println("MPU6050 Found!");

  // Set ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100); // Short delay for sensor stabilization

  // Get first readings for initialization
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize angles using accelerometer
  accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  accAngleY = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

  // Start with accelerometer values to prevent jumps
  compAngleX = accAngleX;
  compAngleY = accAngleY;

  prevTime = millis(); // Initialize timer
}

void loop() {
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert gyro data from rad/s to deg/s
  gyroX = g.gyro.x * 180 / PI;
  gyroY = g.gyro.y * 180 / PI;

  // Compute time elapsed (dt)
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Compute accelerometer angles (roll & pitch)
  accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  accAngleY = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

  // Integrate gyro data to get angles
  gyroAngleX += gyroX * dt;
  gyroAngleY += gyroY * dt;

  // Complementary Filter
  compAngleX = 0.98 * (compAngleX + gyroX * dt) + 0.02 * accAngleX;
  compAngleY = 0.98 * (compAngleY + gyroY * dt) + 0.02 * accAngleY;

  // Serial Plotter Output (CSV-style)
  Serial.print(compAngleX);
  Serial.print(",");
  Serial.println(compAngleY);

  delay(10); // Small delay to stabilize output
}
