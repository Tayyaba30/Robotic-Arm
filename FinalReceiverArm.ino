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
  

    delay(1000);
}



void loop()
{
  if (radio.available())              //Looking for the data.
  {
    byte text[9];                 //Saving the incoming data
    radio.read(&text, sizeof(text));    //Reading the data
    
    Serial.println(text[1]);
    Serial.println(text[2]);

    for(int i = 0; i < 4; ++i)
    {
      SERVO_PORTS[i].write(text[i]);
    }
  
  }


  
  delay(100);
}
