/* CREDITS
 * nRF24L01 library
 * Author: Charles-Henri Hallard
 * http://nRF24.github.io/RF24
 * 
 * Transmitter setup example taken from:
 * https://create.arduino.cc/projecthub/muhammad-aqib/nrf24l01-interfacing-with-arduino-wireless-communication-0c13d4
 * Author:Muhammad Aqib Dutt
 * 
 * 
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <math.h>


byte degreesAngle[4] = {0, 0, 0, 0};
volatile int flag_count = 0;

const int MPU=0x68;
const int MPU2=0x69; 

RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "00001";     

void setup() {

  radio.begin();                 
  radio.openWritingPipe(address); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.stopListening();          


// MPU 1 Setup
    Wire.begin(); 
    Wire.beginTransmission(MPU); 
    Wire.write(0x6B); 
    Wire.write(0);   
    Wire.endTransmission(true); 
    Serial.begin(9600); 


// MPU 2 Setup
//    Wire.begin(); 
    Wire.beginTransmission(MPU2); 
    Wire.write(0x6B); 
    Wire.write(0);   
    Wire.endTransmission(true);

     //  //http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  
  // TIMER 0 for interrupt frequency 1000 Hz:
  cli(); // stop interrupts
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR0A = 249; // = 16000000 / (64 * 1000) - 1 (must be <256)
  // turn on CTC mode
  TCCR0B |= (1 << WGM01);
  // Set CS02, CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei(); // allow interrupts 
  delay(1000);
}

ISR(TIMER0_COMPA_vect)
{

  ++flag_count;
  TCNT0 = 0;
}

void PitchTask(int task){
  if(task%2 == 0)
  {
    Wire.beginTransmission(MPU); 
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU,6,true);   
  }
  else
  {
    Wire.beginTransmission(MPU2); 
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true);  
  }

    //read accelerometer data
    int16_t AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    int16_t AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    int16_t AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
    double x = AcX;
    double y = AcY;
    double z = AcZ;

    double pitch = atan(y/sqrt((x*x) + (z*z))) * (180.0/3.14) ; 
    pitch = map(pitch, -60, 60, 20, 160);
    pitch = constrain(pitch, 20, 160);
    degreesAngle[task] = pitch;
}
 
void RollTask(int task){
  if(task%2 == 0)
  {
    Wire.beginTransmission(MPU); 
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU,6,true);   
  }
  else
  {
    Wire.beginTransmission(MPU2); 
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true);  
  }

    //read accelerometer data
    int16_t AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    int16_t AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    int16_t AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
    double x = AcX;
    double y = AcY;
    double z = AcZ;

    double roll = atan(x/sqrt((y*y) + (z*z))) * (180.0/3.14); 
    roll = map(roll, -60, 60, 20, 160);
    roll = constrain(roll, 20, 160);
    degreesAngle[task] = roll;
}

void (*tasks[])(int) = {PitchTask, PitchTask, RollTask, RollTask};

int taskCount = 4;
void genericOS()
{

  while(true)
  {
    for(int t = 0; t<taskCount; ++t)
    {
        (*tasks[t])(t);          
    }
    radio.write(&degreesAngle, sizeof(degreesAngle));
    
    while(flag_count < 10){} // intervals of 10ms
//    cli(); // stop interrupts    
    flag_count = 0;
    TCNT0 = 0;
//    sei(); // allow interrupts
  }
}

void loop()
{
  genericOS();

}
