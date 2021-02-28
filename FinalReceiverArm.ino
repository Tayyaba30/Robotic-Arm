//45 with first value
//6, wit j seconf
//7 = thirs
//8 = last


#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

Servo servo0;  // pitch mpu 1
Servo servo1;  // pitch mpu 2
Servo servo2;  // roll mpu 1
Servo servo3;  // roll mpu 2
//                      pitch 1, pitch 2, roll 1, roll2
Servo SERVO_PORTS[4] = {servo0, servo1, servo2, servo3};
const int MOTOR_COUNT = 4;
byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver

//data:  4 5 6 7
//ports: 6 5 7 4 

   servo0.attach(6); //pitch1
   servo1.attach(5); //pitch2
   servo2.attach(7); //roll1
   servo3.attach(4); //roll2

    servo0.write(90);
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    
    delay(100);
}



void loop()
{
  if (radio.available())              //Looking for the data.
  {
                     //Saving the incoming data
    radio.read(&data, sizeof(data));    //Reading the data


    for(int i = 0; i < MOTOR_COUNT; ++i) 
    {
      SERVO_PORTS[i].write(data[i+4]);
    }
  }
}
