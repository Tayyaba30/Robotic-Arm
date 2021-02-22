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

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const Servo SERVO_PORTS[] = {servo0, servo1, servo2, servo3, servo4};

//static const int RECORD_SIZE = 5;  // the bigger this number is the more stray inputs are removed at the cost of delay
const int MOTOR_COUNT = 4;
//int record_sets[RECORD_SIZE][MOTOR_COUNT];    // used to average the value for the degree    
//int record_index = 0;   

void setup() {

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);   
  radio.setPALevel(RF24_PA_MIN);       
  radio.startListening();              

   servo0.attach(2, 1000, 2000);
   servo1.attach(3, 1000, 2000);
   servo2.attach(4, 1000, 2000);
   servo3.attach(5, 1000, 2000); 
//   servo4.attach(7, 1000, 2000); 

    servo0.write(90);
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);

//    for(int i = 0; i < MOTOR_COUNT; ++i)
//    {
//      for(int j = 0; j < RECORD_SIZE; ++j)
//      {
//        record_sets[j][i] = 90;
//      }
//    }    

    delay(1000);
}



void loop()
{
  if (radio.available())              //Looking for the data.
  {
    byte text[MOTOR_COUNT];                 //Saving the incoming data
    radio.read(&text, sizeof(text));    //Reading the data
    
    for(int i = 0; i < MOTOR_COUNT; ++i)
    {
      Serial.print("tele ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(text[i]);
    }

//    for(int i = 0; i< MOTOR_COUNT; ++i)
//    {
//      if(int(text[i]) != 90)
//      {
//        record_sets[record_index][i] = text[i];
//      }
//      else
//      {
//        Serial.println("is 90");
//      }
//    }
//    record_index = (record_index + 1 ) % RECORD_SIZE;
//    
//
//    int avg[MOTOR_COUNT] = {0,0};
//    for(int i = 0; i < MOTOR_COUNT; ++i)
//    {
//      for(int j = 0; j < RECORD_SIZE; ++j)
//      {
//        avg[i] += record_sets[j][i];
//      }
//    }    
//
//    for(int i = 0; i < MOTOR_COUNT; ++i)
//    {
//      SERVO_PORTS[i].write(avg[i]/RECORD_SIZE);
//    }
//

    for(int i = 0; i < MOTOR_COUNT; ++i)
    {
      SERVO_PORTS[i].write(text[i]);
    }
  
//    for(int i = 0; i < MOTOR_COUNT; ++i)
//    {
//      Serial.print("avg ");
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.println(text[i]);
//    }
  }


  
  delay(100);
}
