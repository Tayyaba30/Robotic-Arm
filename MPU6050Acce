#include <Servo.h>

#include <MPU6050.h>

MPU6050 mpu;



Servo servo0;
Servo servo1;


const Servo SERVO_PORTS[] = {servo0, servo1};



//.1sec/60 Degrees
// if less time is given than needed then will turn to do the new instruction
// vibrates when trying to reach the maximum angles (0, 180)
// has been recorded to exceed 160 degree cone, cause is unknown
// may stutter when running this OS code, 300 ms delays, on loop
// has been recorded to move the cone, but will recognize where 90 degrees is later






//OS information Part1

//volatile bool flag_raised = false;
volatile int flag_count = 0;
const int TOTAL_MOTOR_TASKS = 2; //count of motor/button pairs



void setup() {

   Serial.begin(9600);

   servo0.attach(2, 1000, 2000); // port 0
   servo0.write(90);
   servo1.attach(3, 1000, 2000); // port 1
   servo1.write(90);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor");
    delay(500);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);
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

  void getCase() // no need to call
  {
    Serial.print("motor task ");
    Serial.print(TaskNumber);

  }
};



struct MotorTask Perform[TOTAL_MOTOR_TASKS];



void motorTask(int task)  
{
    
  Perform[task].TaskNumber = task;    // set the task number
  Perform[task].servo = SERVO_PORTS[task]; //set which servo we are interacting with
  moveMotor(Perform[task].setState, Perform[task].servo);
}

void PitchTask(int task){
  Vector valuesAcc = mpu.readNormalizeAccel();
 
  // Calculating Pitch & Roll
  int pitchAngle = -(atan2(valuesAcc.XAxis, sqrt(valuesAcc.YAxis*valuesAcc.YAxis + valuesAcc.ZAxis*valuesAcc.ZAxis))*180.0)/M_PI;
  Perform[task].setState = pitchAngle;
}
 
void RollTask(int task){
   Vector valuesAcc = mpu.readNormalizeAccel();
  // Calculating Roll
   int rollAngle = (atan2(valuesAcc.YAxis, valuesAcc.ZAxis)*180.0)/M_PI;
   Perform[task].setState = rollAngle;

}



//OS information Part2
void (*tasks[])(int) = {PitchTask, RollTask,
                          motorTask,  motorTask};

int taskCount = 4;

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
