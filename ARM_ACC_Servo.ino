#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
MPU6050 mpu2;
Servo servo0;
Servo servo1;
#define ADDRESS (0x69)




// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;


void setup() 
{
  Serial.begin(115200);


  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, ADDRESS))
  {
    Serial.println("Could not find a valid MPU6050 1 sensor, check wiring!");
    delay(500);
  }

  delay(100);
  
 
   servo0.attach(4); // port 1
   servo1.attach(5);
   servo0.write(90);
   servo1.write(90);
  
  mpu.calibrateGyro();
  mpu.setThreshold(3);

}

//North = negative pitch
// south = positive pitch
// east = positive roll
// west = negative roll

void acc1()
{
  timer = millis();

  // Read normalized values
  Vector valuesAcc = mpu.readNormalizeAccel();

  // Calculate Pitch, Roll and Yaw
  roll = -(atan2(valuesAcc.XAxis, sqrt(valuesAcc.YAxis*valuesAcc.YAxis + valuesAcc.ZAxis*valuesAcc.ZAxis))*180.0)/M_PI;
  pitch = (atan2(valuesAcc.YAxis, valuesAcc.ZAxis)*180.0)/M_PI;
  roll = map(roll, -180, 180, 20, 160);
  pitch = map(pitch, -180, 180, 20, 160);
  servo0.write(roll);
  delay(100);
  servo1.write(pitch);
  Serial.print("pitch 1: ");
  Serial.print(pitch);
  Serial.println(" ");
  Serial.print("roll 1: ");
  Serial.print(roll);
  Serial.println(" ");  
}


void loop()
{
  acc1();
  delay(200);
}
