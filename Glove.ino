
#include <RFM69.h>         
#include <RFM69_ATC.h>     
#include <SPI.h>          
#include <LowPower.h>  

// Accelerometer setup

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;


//// radio transmitter setup
//#define NETWORKID     0 
//#define RECEIVER      1    //unique ID of the gateway/receiver
//#define SENDER        2
//#define NODEID        SENDER 
//#define FREQUENCY     RF69_915MHZ
//#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//
////#define ENABLE_ATC      //comment out this line to disable AUTO TRANSMISSION CONTROL
//
//
//#define ATC_RSSI  
//
//#define SERIAL_BAUD   115200
//
//#ifdef ENABLE_ATC
//  RFM69_ATC radio;
//#else
//  RFM69 radio;
//#endif

char Send[62];
int SendLength = 62;

//OS information Part1
// flex sensor 1: straight = 2814.09(968), Bent 180 degree = 4545.45 (937)
// flex sensor 2: straight = 4545.08, Bent 180 Degree = 538.34  
// flex sensor 3: straight = 4315.09, Bent 180 Degree = 687.21
// flex sensor 4: straight = 5306.09, Bent 180 Degree = 2488.0
// flex sensor 5: straight = 5725.0, Bent 180 Degree = 3085.37


volatile int flag_count = 0;
const int TOTAL_MOTOR_TASKS = 5; //count of motor/button pairs
const int TOTAL_ACC_TASKS = 2; // 2 for now, but we will use four in the future

// flex sensors 

const int FlexSensor1 = A0, FlexSensor2 = A1, FlexSensor3 = A2, FlexSensor4 = A3, FlexSensor5 = A4;  

const int FlexPorts[] = {FlexSensor1, FlexSensor2, FlexSensor3, FlexSensor4, FlexSensor5};
const float FlexValues[5][3] = {{2814.09, 4545.45, 1}, {4545.08, 538.34, 0}, {4315.09, 687.21, 0}, {5306.09, 5725.0, 0}, {5725.0, 3085.37, 0}};    // *****  NEED TO COMPLETE **************

const float VCC = 5; // Measured voltage of Ardunio 5V line
const float R_DIV = 49528.0; // Measured resistance of 3.3k resistor

void setup() {
  Serial.begin(9600);
//  radio.initialize(FREQUENCY,NODEID,NETWORKID);
//  #ifdef IS_RFM69HW_HCW
//  radio.setHighPower(); //must include this only for RFM69HW/HCW!
//  #endif
//    radio.encrypt(ENCRYPTKEY);
//    
//  #ifdef ENABLE_ATC
//    radio.enableAutoPower(ATC_RSSI);
//  #endif
//  char buff[50];
//  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
//  Serial.println(buff);
//  Serial.flush();
  //flex sensors setup
  Wire.begin(); 
  
   pinMode(FlexSensor1, INPUT);
   pinMode(FlexSensor2, INPUT);
   pinMode(FlexSensor3, INPUT);
   pinMode(FlexSensor4, INPUT);
   pinMode(FlexSensor5, INPUT);

   // accelerometer setup

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

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

struct AccTask{
  int taskNumber;
  int rollAngle;  // roll angle taken from the accelerometer
  int pitchAngle; // pitch angle taken from the accelerometer

  bool PitchSet = false;
  bool RollSet = false;
  void getPitch()
  {
    Vector valuesAcc = mpu.readNormalizeAccel();
    PitchSet = true;
    RollSet = false;

  // Calculating Pitch & Roll
    pitchAngle = -(atan2(valuesAcc.XAxis, sqrt(valuesAcc.YAxis*valuesAcc.YAxis + valuesAcc.ZAxis*valuesAcc.ZAxis))*180.0)/M_PI;
  }
  void getRoll()
  {
    Vector valuesAcc = mpu.readNormalizeAccel();
  // Calculating Roll
    rollAngle = (atan2(valuesAcc.YAxis, valuesAcc.ZAxis)*180.0)/M_PI;
    PitchSet = false;
    RollSet = true;
  }
};

struct flexTask{
//  int number;         // stores the value that we will use to decide the case.
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


  void getValues()
  {
    int flexADC = analogRead(FlexPorts[taskNumber]); // 
    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
   // Serial.println("original: " + String(flexADC));
  //  Serial.println("Resistance: " + String(flexR) + " ohms");

    set_degrees = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 180);
    set_degrees = constrain(set_degrees, 0, 180);
 // Serial.println("Bend: " + String(set_degrees) + " degrees");
  }
};

struct flexTask flexPerform[TOTAL_MOTOR_TASKS];



void FlexSensorTask(int task){
  flexPerform[task].taskNumber = task;
  flexPerform[task].setVal(FlexValues[task][0], FlexValues[task][1]);
  
  flexPerform[task].getValues();  // read values from the flex sensor

}


struct AccTask AccelerometerPerform[TOTAL_ACC_TASKS];


void PitchTask(int task){
  int ActualTask = task - TOTAL_MOTOR_TASKS;  // we will be using 2 accelerometers: one for wrist and the other for elbow.
  AccelerometerPerform[ActualTask].taskNumber = ActualTask;
  AccelerometerPerform[ActualTask].getPitch(); 
}

void RollTask(int task){
  int ActualTask = task - TOTAL_MOTOR_TASKS;  // we will be using 2 accelerometers: one for wrist and the other for elbow.
  AccelerometerPerform[ActualTask].taskNumber = ActualTask;
  AccelerometerPerform[ActualTask].getRoll(); 
}


//OS information Part2
void (*tasks[])(int) = {FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask, FlexSensorTask, PitchTask, RollTask};
int taskLine[] = {    // only one case no loop
                  1, 1, 1, 1, 1,1, 1  // we have seven tasks now
              };
int taskCount = 7;


void genericOS()
{

  while(true)
  {
    for(int t = 0; t<taskCount; ++t)
    {

        (*tasks[t])(t); 
        if(t <= 5)  // flex tasks
        {
          sprintf(Send, "%d", flexPerform[t].set_degrees);
        }
        else   // accelerometer tasks
        {
          if(AccelerometerPerform[t - TOTAL_MOTOR_TASKS].PitchSet == false)
            sprintf(Send, "%d", AccelerometerPerform[t - TOTAL_MOTOR_TASKS].rollAngle);
          else
            sprintf(Send, "%d", AccelerometerPerform[t - TOTAL_MOTOR_TASKS].pitchAngle);                  
        }
        Wire.beginTransmission(9); // transmit to device #9
        Wire.write(Send);              // sends x 
        Wire.endTransmission(); 
//
//        
//        Serial.println("Sending");
//        radio.send(SENDER, Send, SendLength); 
//        Serial.println("Signal sent");
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
