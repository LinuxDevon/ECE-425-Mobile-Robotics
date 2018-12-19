/*
 * This file was used to calibrate our robot Moravec
 * 
 * Devon Adair and Hunter LaMantia
 * 12/16/2018
 */

/*
 * Includes
 */
#include <NewPing.h> //include sonar library

/*
 * Defines and Declarations
 */
const int rtStepPin = 46; //right stepper motor step pin
const int rtDirPin = 53;  // right stepper motor direction pin
const int ltStepPin = 44; //left stepper motor step pin
const int ltDirPin = 49;  //left stepper motor direction pin

//define sensor pins
#define irFront   A8    //front IR analog pin
#define irRear    A9    //back IR analog pin
#define irRight   A10   //right IR analog pin
#define irLeft    A11   //left IR analog pin
#define snrLeft   8   //front left sonar 
#define snrRight  9  //front right sonar 

#define baud_rate     9600  //set serial communication baud rate
#define ping_interval 1500//interval between sonar pulses

NewPing sonarLt(snrLeft, snrLeft);//create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);//create an instance of the right sonar

// Sensor readings
int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int srLeftAvg;   //variable to hold average of left sonar current reading
int srRightAvg;  //variable to hold average or right sonar current reading

int rightVal, leftVal, srRightInches, srLeftInches;

/*
 * Function Declarations
 */
void readIR(void);
void readSonar(void);

void setup(void) {
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output

  //Set up serial communication
  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Sensor Data and Calibration......");
}

void loop(void) {
//  readIR();
  readSonar();
}


void readIR(void) {
  long value = 0;
  long inches = 0;
  
  for (int i = 0; i < 9; i++) {
    value = value + analogRead(irRight);
  }
  value = value / 10;
  
//  inches = (1111/(value+16))-1; //front
//  inches = (1111/(value+20))-1; //rear
//  inches = (285714/(value+2257))-103; //left
  inches = (285714/(value+2600))-90; //right

  Serial.print("IR Inches: ");
  Serial.println(inches);
}

void readSonar(void) {
  long valueLeft = 0;
  long valueRight = 0;
  long inchesLeft = 0, inchesRight = 0;

  for(int i = 0; i < 99; i++) {
    pinMode(snrLeft, OUTPUT);
    digitalWrite(snrLeft, LOW);
    delayMicroseconds(2);
    digitalWrite(snrLeft, HIGH);
    delayMicroseconds(5);
    digitalWrite(snrLeft, LOW);
    pinMode(snrLeft, INPUT);
    valueLeft = valueLeft + pulseIn(snrLeft, HIGH);
  }

  for(int i = 0; i < 99; i++) {
    pinMode(snrRight, OUTPUT);
    digitalWrite(snrRight, LOW);
    delayMicroseconds(2);
    digitalWrite(snrRight, HIGH);
    delayMicroseconds(5);
    digitalWrite(snrRight, LOW);
    pinMode(snrRight, INPUT);
    valueRight = valueRight + pulseIn(snrRight, HIGH);
  }

  valueRight = valueRight / 100;
  valueLeft = valueLeft / 100;

  inchesRight = (-568181/(valueRight-11136))-50; //right
  inchesLeft = (-479616/(valueLeft-9520))-50; //left
  
  Serial.print("Sonar Left: ");
  Serial.print(inchesLeft);
  Serial.print("    Sonar Right: ");
  Serial.println(inchesRight);
}
