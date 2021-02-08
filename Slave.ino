#include <Servo.h>
// Include the required Wire library for I2C<br>
#include <Wire.h>
int LED = 12;
volatile int angles[] = {-1, -1, -1, -1, -1};
volatile int old_angles[] = {-1, -1, -1, -1, -1};

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
const Servo SERVO_PORTS[] = {servo0, servo1, servo2, servo3, servo4};

const int SERVOS_IN_USE = 5;


void setup() {
  Serial.begin(9600);
  
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);

  // Start the I2C Bus as Slave on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  //setup servos
  servo0.attach(3, 1000, 2000); // port 3
  servo1.attach(4, 1000, 2000); // port 4
  servo2.attach(5, 1000, 2000); // port 5
  servo3.attach(6, 1000, 2000); // port 4
  servo4.attach(7, 1000, 2000); // port 5
  
}
void receiveEvent(int bytes) {
  for (int i = 0; i < SERVOS_IN_USE; ++i)
  {
    angles[i] = Wire.read();
  } 
  

  
}
void loop() {
  for(int i = 0; i < SERVOS_IN_USE; ++i)
  {
//    if (abs(old_angles[i] - angles[i]) > 2)
//    {
      SERVO_PORTS[i].write(angles[i]); // move servoI
      old_angles[i] = angles[i];
//    }
    Serial.println("" + String(i) + ": " + String(angles[i]) + ", " + String(old_angles[i]));
  }
  delay(200);
  
}
