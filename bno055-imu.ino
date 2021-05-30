#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup() {
  Serial.begin(9600);
  Serial.println("BNO055 IMU");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
}

void loop() {
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  Serial.print("X: ");
  Serial.print(linAccel.x(), 5);
  Serial.print(" Y: ");
  Serial.print(linAccel.y());
  Serial.print(" Z: ");
  Serial.print(linAccel.z());
  Serial.println("");

  delay(BNO055_SAMPLERATE_DELAY_MS);

}