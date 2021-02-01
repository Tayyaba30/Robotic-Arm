#include <Servo.h>
#include <RFM69.h>         
#include <RFM69_ATC.h>     
#include <SPI.h>          
#include <LowPower.h>  
 
 
#define NETWORKID     0 
#define RECEIVER      1    //unique ID of the gateway/receiver
#define SENDER        2
#define NODEID        RECEIVER 
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
 
//#define ENABLE_ATC      //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI  
 
#define SERIAL_BAUD   115200
 
//#ifdef ENABLE_ATC
//  RFM69_ATC radio;
//#else
  RFM69 radio;
//#endif
 
char Receive[62];
int ReceiveLength = 62;
 
// servo motors
 
Servo servo0;  // finger 1
Servo servo1;  // finger 2
Servo servo2;  // finger 3
Servo servo3;  // finger 4
Servo servo4;  // finger 5
Servo servo5;  // wrist pitch angle
Servo servo6;  // wrist roll angle
 
const Servo SERVO_PORTS[] = {servo0, servo1, servo2, servo3, servo4, servo5, servo6};
 
 
volatile int flag_count = 0;
const int TOTAL_MOTOR_TASKS = 7; //count of motor/button pairs
 
void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
    radio.encrypt(ENCRYPTKEY);
    
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.flush();
  //flex sensors setup
 
   servo0.attach(3, 1000, 2000); // port 0
   servo1.attach(4, 1000, 2000); // port 1
   servo2.attach(5, 1000, 2000); // port 2
   servo3.attach(6, 1000, 2000); // port 3
   servo4.attach(7, 1000, 2000); // port 4
 
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
 
}
 
ISR(TIMER0_COMPA_vect)
{
//  Serial.print("flag set");
  ++flag_count;
  TCNT0 = 0;
}
 
void Blinking(int Blink, Servo s)
{
 
  s.write(Blink);
}
 
struct MotorTask{
  int TaskNumber; // stores the task
  Servo servo;        // Stores the servo that we want to blink
  int setState;   // Stores what angle to set the motor   
  float degree; 
 
  void getCase() // no need to call
  {
    Serial.print("motor task ");
    Serial.print(TaskNumber);
 
  }
};
struct MotorTask Perform[TOTAL_MOTOR_TASKS];
void ServoTask(int task)  
{
 
  Perform[task].TaskNumber = task;    // set the task number
  Perform[task].servo = SERVO_PORTS[task]; //set which servo we are interacting with
  // Perform[task].getCase();
  getMessage(task);
  Blinking(Perform[task].degree, Perform[task].servo);
}
 
 
//OS information Part2
void (*tasks[])(int) = {ServoTask,  ServoTask,  ServoTask,  ServoTask,  ServoTask, ServoTask, ServoTask};
int taskCount = 7;
 
void getMessage(int task)
{
    if (radio.receiveDone()) // Got one!
  {
    Serial.print("received from node ");
 
    for ( byte i = 0; i < radio.DATALEN; i++)
      Receive[i] = char(radio.DATA[i]);
    sscanf(Receive, "%f", &Perform[task].degree);
  }
  else{
    Serial.println("Not received");
  }
}
void genericOS()
{
 
  while(true)
  {
    for(int t = 0; t<taskCount; ++t)
    {        
      (*tasks[t])(t);
    }
   
    while(flag_count < 10){} // intervals of 10ms
//    cli(); // stop interrupts
    flag_count = 0;
    TCNT0 = 0;
//    sei(); // allow interrupts
  }
}
 
void loop() {
  Serial.println("OS Begin");
  genericOS();
 
}
