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

const int selectPins[4] = {5, 4, 3, 2}; // S0~2, S1~3, S2~4
const int zInput = A0; // Connect common (Z) to 5 (PWM-capable)

const float VCC = 5; // Measured voltage of Ardunio 5V line
const float R_DIV = 47000.0; // Measured resistance of 47k resistor

#define TOTAL_FLEX_TASKS 5
#define TASK_COUNT 9

const float FlexValues[5][3] = {{920.0, 1025.0, 0}, {910.0, 1015.0,0}, {850.0, 1015.0, 1}, {900.0, 1020.0, 0}, {880.0, 1015.0, 0}}; 


byte degreesAngle[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
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
// MUX setUp
  // Set up the select pins, as outputs
  for (int i=0; i<4; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }
  pinMode(zInput, INPUT); // Set up Z as an input


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

void selectMuxPin(byte pin)
{
  if (pin > 15) 
    return; // Exit if pin is out of scope
  for (int i=0; i<4; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }

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
    int16_t AcX=Wire.read()<<8|Wire.read(); 
    int16_t AcY=Wire.read()<<8|Wire.read(); 
    int16_t AcZ=Wire.read()<<8|Wire.read(); 
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
    int16_t AcX=Wire.read()<<8|Wire.read();  
    int16_t AcY=Wire.read()<<8|Wire.read(); 
    int16_t AcZ=Wire.read()<<8|Wire.read();
    double x = AcX;
    double y = AcY;
    double z = AcZ;

    double roll = atan(x/sqrt((y*y) + (z*z))) * (180.0/3.14); 
    roll = map(roll, -60, 60, 20, 160);
    roll = constrain(roll, 20, 160);
    degreesAngle[task] = roll;
}

struct flexTask{
  int taskNumber;     // stores the task
  float STRAIGHT_RESISTANCE;
  float BEND_RESISTANCE;
  int set_degrees;    // Stores  what degree to set the motor to
  //static const int RECORD_SIZE = 10;  // the bigger this number is the more stray inputs are removed at the cost of delay
  //bool record_sets[RECORD_SIZE];    // used to average the value for the degree    
  //int record_index = 0;   
  int flexADC;
  int increasing = 0;
  void setVal(float Straight, float Bent, int inc)
  {
    STRAIGHT_RESISTANCE = Straight;
    BEND_RESISTANCE = Bent;
    increasing = inc;
  }

  void getCase()
  {

      byte t = (byte)taskNumber - 4;
      selectMuxPin(t);
      flexADC = analogRead(A0);     

    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
    // Serial.println("original: " + String(flexADC));
    //  Serial.println("Resistance: " + String(flexR) + " ohms");

    if(increasing == 0)
    {
      set_degrees = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 180);    
    }
    else
    {
      set_degrees = map(flexR,  BEND_RESISTANCE, STRAIGHT_RESISTANCE, 0, 180);
    }
    set_degrees = constrain(set_degrees, 20, 160);
    //  Serial.println("Bend: " + String(set_degrees) + " degrees"); 
    degreesAngle[taskNumber] = set_degrees; //setting to 180 or 0 may change the cone
  }
};

struct flexTask flexPerform[TOTAL_FLEX_TASKS];

void FlexSensorTask(int task){
  flexPerform[task - 4].taskNumber = task;
  flexPerform[task - 4].setVal(FlexValues[task - 4][0], FlexValues[task - 4][1], FlexValues[task - 4][2]);
  
 // flexPerform[task].flexADC = analogRead(FlexPorts[task]);

  flexPerform[task - 4].getCase();

}

void (*tasks[])(int) = {PitchTask, PitchTask, RollTask, RollTask, FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask};

int taskCount = 9;
void genericOS()
{

  while(true)
  {
    for(int t = 0; t<taskCount; ++t)
    {
        (*tasks[t])(t);          
    }
    Serial.println(degreesAngle[0]);
     Serial.println(degreesAngle[1]);
     Serial.println(degreesAngle[2]);
    Serial.println(degreesAngle[3]);
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
