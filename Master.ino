// Include the required Wire library for I2C<br>
#include<Wire.h>
//#include <MPU6050.h>

//MPU6050 mpu;


//float pitch = 0;
//float roll = 0;

volatile char *values[36];



volatile int flag_count = 0;
const int FlexSensor1 = A0, FlexSensor2 = A1, FlexSensor3 = A2, FlexSensor4 = A3, FlexSensor5 = A4; 
const int FlexPorts[] = {FlexSensor1, FlexSensor2, FlexSensor3, FlexSensor4, FlexSensor5};
const float FlexValues[3][3] = {{5000, 3000, 1}, {3700, 2500, 1}, {3500, 2000, 1}};    // *****  NEED TO COMPLETE **************






int totalTasks = 3;

const float VCC = 5; // Measured voltage of Ardunio 5V line
const float R_DIV = 47000.0; // Measured resistance of 3.3k resistor

int flexADC= 0;

void setup() {
  // Start the I2C Bus as Master
  Serial.begin(9600);
  Wire.begin(); 
   pinMode(FlexSensor1, INPUT);
   pinMode(FlexSensor2, INPUT);
   pinMode(FlexSensor3, INPUT);


 //Serial.println("Initialize MPU6050");

 // while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
 // {
 //   Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
 //   delay(500);
 // }



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
//  Serial.print("flag set");
  ++flag_count;
  TCNT0 = 0;
}

struct tasks{
  int setState;
  
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
    //Serial.println("original: " + String(flexADC));
    //Serial.println("Resistance: " + String(flexR) + " ohms");

    set_degrees = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 20, 160);
    set_degrees = constrain(set_degrees, 20, 160);
 // Serial.println("Bend: " + String(set_degrees) + " degrees");
 
  }
};

struct flexTask flexPerform[3];



void FlexSensorTask(int task){
  flexPerform[task].taskNumber = task;
  flexPerform[task].setVal(FlexValues[task][0], FlexValues[task][1]);
  
  flexPerform[task].getValues();  // read values from the flex sensor

}


//struct tasks Perform[2];
//void PitchTask(int task){
//  Vector valuesAcc = mpu.readNormalizeAccel();
 
//  // Calculating Pitch & Roll
//  int pitchAngle = -(atan2(valuesAcc.XAxis, sqrt(valuesAcc.YAxis*valuesAcc.YAxis + valuesAcc.ZAxis*valuesAcc.ZAxis))*180.0)/M_PI;
//  Perform[task].setState = pitchAngle;
//}


//void RollTask(int task){
//   Vector valuesAcc = mpu.readNormalizeAccel();
//  // Calculating Roll
//   int rollAngle = (atan2(valuesAcc.YAxis, valuesAcc.ZAxis)*180.0)/M_PI;
//   Perform[task].setState = rollAngle;

//}



void (*tasks[])(int) = {FlexSensorTask, FlexSensorTask, FlexSensorTask};
int taskLine[] = {    // only one case no loop
                  1, 1, 1 // we have seven tasks now
              };
int taskCount = 3;


void genericOS()
{

  while(true)
  {
    String val = "";
    byte vaa [3];
    char *toSend[36];
    for(int t = 0; t<taskCount; ++t)
    {
        (*tasks[t])(t);

         vaa[t] = flexPerform[t].set_degrees;

 
          
        

 // stop transmitting    

    }
        Wire.beginTransmission(9); // transmit to device #9
        Wire.write(vaa, 3);
        Wire.endTransmission();  
         delay(200); 
  
    while(flag_count < 10){} // intervals of 10ms
//    cli(); // stop interrupts    
    flag_count = 0;
    TCNT0 = 0;
//    sei(); // allow interrupts
  }
}

void loop() {
  //Serial.println("OS Begin");
  genericOS();

}
