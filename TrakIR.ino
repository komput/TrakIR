#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

MPU9250 accelgyro;

const int Version=1;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
}

void loop()
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  
  Serial.print("a/g/m:t");
  Serial.print(ax); Serial.print("t");
  Serial.print(ay); Serial.print("t");
  Serial.print(az); Serial.print("t");
  Serial.print(gx); Serial.print("t");
  Serial.print(gy); Serial.print("t");
  Serial.print(gz); Serial.print("t");
  Serial.print(mx); Serial.print("t");
  Serial.print(my); Serial.print("t");
  Serial.println(mz);
}

