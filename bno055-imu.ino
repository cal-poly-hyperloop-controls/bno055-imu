#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Time when reading is sampled from IMU [ms]
uint32_t  currentTime, lastTime = 0;

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
  // Save sample time
  currentTime = millis();

  // Grab linear acceleration data from IMU
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  double xLinAccel = linAccel.x();
  double yLinAccel = linAccel.y();
  double zLinAccel = linAccel.z();

  // Print the obtained linear accel data
  Serial.print("X accel: ");
  Serial.print(xLinAccel, 5);
  Serial.print(" Y accel: ");
  Serial.print(yLinAccel, 5);
  Serial.print(" Z accel: ");
  Serial.print(zLinAccel, 5);
  Serial.print("\t\t");

  // Print and calculate velocity from accel: v = a*t, but don't calculate
  // velocity for the first sample (aka can't find velocity with only one 
  // accel data point)
  if (lastTime > 0) {
    uint32_t deltaTime = currentTime - lastTime; 

    Serial.print("X vel: ");
    Serial.print(xLinAccel * deltaTime, 5);
    Serial.print(" Y vel: ");
    Serial.print(yLinAccel * deltaTime, 5);
    Serial.print(" Z vel: ");
    Serial.print(zLinAccel * deltaTime, 5);
    Serial.print("\t\t");
  }
  
  
  Serial.println("");

  lastTime = currentTime;

  // Wait a bit before reading the data again
  delay(BNO055_SAMPLERATE_DELAY_MS);

}