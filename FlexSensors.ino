#include <Servo.h>



// flex sensor 1: straight = 2814.09(968), Bent 180 degree = 4545.45 (937)
// flex sensor 2: straight = 6315.0, Bent 180 Degree = 0  *************   Faulty **************
// flex sensor 3: straight = 4315.09, Bent 180 Degree = 687.21
// flex sensor 4: straight = 5306.09, Bent 180 Degree = 2488.0
// flex sensor 5: straight = 5725.0, Bent 180 Degree = 3085.37

const float VCC = 5; // Measured voltage of Ardunio 5V line
const float R_DIV = 49528.0; // Measured resistance of 3.3k resistor

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const Servo SERVO_PORTS[] = {servo0, servo1, servo2, servo3, servo4};

const int FlexSensor1 = A0, FlexSensor2 = A1, FlexSensor3 = A2, FlexSensor4 = A3, FlexSensor5 = A5;  



const int FlexPorts[] = {FlexSensor1, FlexSensor2, FlexSensor3, FlexSensor4, FlexSensor5};
const float FlexValues[5][2] = {{2814.09, 4545.45}, {4315.09, 687.21}, {5306.09, 2488.0}, {}, {}};    // *****  NEED TO COMPLETE **************

//.1sec/60 Degrees
// if less time is given than needed then will turn to do the new instruction
// vibrates when trying to reach the maximum angles (0, 180)
// has been recorded to exceed 160 degree cone, cause is unknown
// may stutter when running this OS code, 300 ms delays, on loop
// has been recorded to move the cone, but will recognize where 90 degrees is later






//OS information Part1
int taskScale[] = {10,10,10,10,10,
                  1,1,1,1,1}; //button at 10ms scale, lights at 100ms scale or 1000ms scale
bool taskActive[] = {true,true,true,true,true,
                  true, true, true, true, true}; //buttons always active, lights must be set active
//int taskOffset[] = {0,0,0,0}; // button can be pressed at any time, lights must comply
//volatile bool flag_raised = false;
volatile int flag_count = 0;
const int TOTAL_MOTOR_TASKS = 5; //count of motor/button pairs



void setup() {

   pinMode(FlexSensor1, INPUT);
   pinMode(FlexSensor2, INPUT);
   pinMode(FlexSensor3, INPUT);
   pinMode(FlexSensor4, INPUT);
   pinMode(FlexSensor5, INPUT);



    Serial.begin(9600);

   servo0.attach(2, 1000, 2000); // port 0
   servo1.attach(3, 1000, 2000); // port 1
   servo2.attach(4, 1000, 2000); // port 2
   servo3.attach(5, 1000, 2000); // port 3
   servo4.attach(11, 1000, 2000); // port 4

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

void moveMotor(int Blink, Servo s)
{

  s.write(Blink);
}


struct MotorTask{
  int TaskNumber; // stores the task
  Servo servo;        // Stores the servo that we want to blink
  int setState;   // Stores what angle to set the motor    

};



struct MotorTask Perform[TOTAL_MOTOR_TASKS];



void motorTask(int task)  
{
    
  Perform[task].TaskNumber = task;    // set the task number
  Perform[task].servo = SERVO_PORTS[task]; //set which servo we are interacting with
  // Perform[task].getCase();
  moveMotor(Perform[task].setState, Perform[task].servo);
}



//********************************************************************************

struct flexTask{
  int taskNumber;     // stores the task
  float STRAIGHT_RESISTANCE;
  float BEND_RESISTANCE;
  int set_degrees;    // Stores  what degree to set the motor to
  static const int RECORD_SIZE = 10;  // the bigger this number is the more stray inputs are removed at the cost of delay
  bool record_sets[RECORD_SIZE];    // used to average the value for the degree    
  int record_index = 0;   
  int flexADC;
  void setVal(float Straight, float Bent)
  {
    STRAIGHT_RESISTANCE = Straight;
    BEND_RESISTANCE = Bent;
  }


  void getCase()
  {
    //int flexADC = analogRead(FlexPorts[taskNumber]); // 
    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
   // Serial.println("original: " + String(flexADC));
  //  Serial.println("Resistance: " + String(flexR) + " ohms");

    set_degrees = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 180);
    set_degrees = constrain(set_degrees, 20, 160);
  //  Serial.println("Bend: " + String(set_degrees) + " degrees");
    Perform[taskNumber].setState = set_degrees; //setting to 180 or 0 may change the cone
  }
};

struct flexTask flexPerform[TOTAL_MOTOR_TASKS];

//*********************************************************************************



void FlexSensorTask(int task){
  flexPerform[task].taskNumber = task;
  flexPerform[task].setVal(FlexValues[task][0], FlexValues[task][1]);
  
  flexPerform[task].flexADC = analogRead(FlexPorts[task]);

  flexPerform[task].getCase();


}



//OS information Part2
void (*tasks[])(int) = {FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask,
                          motorTask,  motorTask,  motorTask,  motorTask,  motorTask};
int taskLine[] = {    // only one case no loop
                  1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1
              };
int taskCount = 10;

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
