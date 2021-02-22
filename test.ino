#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions
byte text[4] = {0, 0, 0, 0};


const int MPU=0x68; //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ; //16-bit integers
double pitch,roll;

const int MPU2=0x69; //I2C address of the MPU-6050

RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.

void setup() {

radio.begin();                  //Starting the Wireless communication
radio.openWritingPipe(address); //Setting the address where we will send the data
radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
radio.stopListening();          //This sets the module as transmitter


// MPU Se
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(9600); //serial communication at 9600 bauds


// MPU 2 Se
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU2); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
}

void acc1()
{
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU,6,true); //request 14 registers in total  


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
    double x = AcX;
    double y = AcY;
    double z = AcZ;
   
    roll = atan(x/sqrt((y*y) + (z*z))) * (180.0/3.14);
    pitch = atan(y/sqrt((x*x) + (z*z))) * (180.0/3.14) ; 

    roll = map(roll, -50, 50, 20, 160);
    pitch = map(pitch, -60, 60, 20, 160);
    roll = constrain(roll, 20, 160);
    pitch = constrain(pitch, 20, 160);
    text[0] = pitch;
    text[1] = roll;
 
}


void acc2()
{
    Wire.beginTransmission(MPU2); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU2,6,true); //request 14 registers in total  


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
    double x = AcX;
    double y = AcY;
    double z = AcZ;

    pitch = atan(x/sqrt((y*y) + (z*z))) * (180.0/3.14);
    roll = atan(y/sqrt((x*x) + (z*z))) * (180.0/3.14) ; 
        Serial.print("ppp    ");
    Serial.println(pitch);
    Serial.print("rrrrr  ");
    Serial.println(pitch);
    roll = map(roll, -50, 50, 20, 160);
    pitch = map(pitch, -60, 60, 20, 160);
    roll = constrain(roll, 20, 160);
    pitch = constrain(pitch, 20, 160);

    text[2] = pitch;
    text[3] = roll;

}

void loop()
{
  while(true)
  {
  acc1();
  delay(10);
  acc2();

  radio.write(&text, sizeof(text));                  //Sending the message to receiver

  delay(100);  
  }


}
