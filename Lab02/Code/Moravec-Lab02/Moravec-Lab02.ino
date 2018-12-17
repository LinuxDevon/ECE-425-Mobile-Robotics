/*RobotTimerInterrupt.ino
  Author: Carlotta. A. Berry
  Date: December 17, 2016
  This program will test using a timer interrupt to update the IR and sonar data
  in order  to create an obstacle avoidance beavhior on the robot.

  Hardware Connections:
  Stepper Enable Pin 48
  Right Stepper Step Pin 46
  Right Stepper Direction Pin 53
  Left Stepper Step Pin 44
  Left Stepper Direction Pin 49

  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 46 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 44 - left stepper motor step pin
  digital pin 49 - left stepper motor direction pin

  Hardware Connections:
  Front IR    A8
  Back IR     A9
  Right IR    A10
  Left IR     A11
  Left Sonar  A12
  Right Sonar A13
  Pushbutton  A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>

//define stepper motor pin numbers
const int rtStepPin = 46; //right stepper motor step pin
const int rtDirPin = 53;  // right stepper motor direction pin
const int ltStepPin = 44; //left stepper motor step pin
const int ltDirPin = 49;  //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

//define stepper motor constants
#define stepperEnable 48    //stepper enable pin on stepStick
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define one_rotation  400//stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  800 //stepper motor 2 rotations
#define three_rotation 1200 //stepper rotation 3 rotations
#define max_accel     10000//maximum robot acceleration
#define robot_spd     500 //set robot speed
#define max_spd       500//maximum robot speed

#define irFront   A8    //front IR analog pin
#define irRear    A9    //back IR analog pin
#define irRight   A10   //right IR analog pin
#define irLeft    A11   //left IR analog pin
#define snrLeft   8   //front left sonar 
#define snrRight  9  //front right sonar 
#define button    A15    //pushbutton 

NewPing sonarLt(snrLeft, snrLeft);//create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);//create an instance of the right sonar

#define irThresh    400 // The IR threshold for presence of an obstacle
#define snrThresh   7  // The sonar threshold for presence of an obstacle
#define minThresh   0   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move

int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int srLeftAvg;   //variable to hold average of left sonar current reading
int srRightAvg;  //variable to hold average or right sonar current reading

#define baud_rate     9600  //set serial communication baud rate
#define ping_interval 1500//interval between sonar pulses
#define TIME          500   //pause time
#define timer_int     125000 //timer interupt interval in microseconds


//sonar Interrupt variables
volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
volatile uint8_t test_state = 0;

//flag byte to hold sensor data
byte flag = 0;    // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

//state byte to hold robot motion and state data
byte state = 0;   //state to hold robot states and motor motion

//bit definitions for robot motion and state byte
#define movingR   0  // Moving Right Motor in progress flag
#define movingL   1  // Moving Left Motor in progress flag
#define fwd       2
#define rev       3
#define collide   4
#define runAway   5
#define wander    6

// USER DEFINES //
#define LEFT 0
#define RIGHT 1
#define Pi 3.14159265358979

#define REST_DELAY 500        // half second delay
#define ONE_SECOND 1000       // one second delay
#define FULL_SPIN 360       // 360 degrees
#define TICKS_FOR_FULL_WHEEL_SPIN 800
#define INCHES_FOR_FULL_WHEEL_SPIN 10.5
#define RIGHT_ANGLE 90        // 90 degrees
#define FULL_CIRCLE_TICKS_COUNT 1900 // number of ticks to make a full spin

#define RED_LED 14
#define GREEN_LED 16
#define YELLOW_LED 15

bool isObstacle;
int rightVal, leftVal, srRightInches, srLeftInches;

void setup() {
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);//set right motor speed
  stepperLeft.setSpeed(robot_spd);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  
  isObstacle = false; // assume no obstacle starting
   
  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(obsRoutine);  // attaches updateSensors() as a timer overflow interrupt

  //Set up serial communication
  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
  delay(2500); //seconds before the robot moves
}

void loop() {
  forward(10, 6);
  delay(5000);
  reverse(24, 6);
  delay(5000);
}

//obstacle avoidance routine based upon timer interrupt

void obsRoutine() {
  updateSensors();
//  if (((srRightAvg < snrThresh && srRightAvg > minThresh)
//       || (srLeftAvg < snrThresh && srLeftAvg > minThresh)) 
//       || (irFrontAvg > irThresh)     // check front ir
//       || (irRearAvg > irThresh)) {   // check rear ir

  if ((irFrontAvg > irThresh)     // check front ir
       || (irRearAvg > irThresh)) {   // check rear ir
//    Serial.println("obstacle detected: stop Robot");
    //    Serial.print("f:\t"); Serial.print(irFrontAvg); Serial.print("\t");
    //    Serial.print("b:\t"); Serial.print(irRearAvg); Serial.print("\t");
    //    Serial.print("l:\t"); Serial.print(irLeftAvg); Serial.print("\t");
    //    Serial.print("r:\t"); Serial.print(irRightAvg); Serial.print("\t");
    //    Serial.print("lt snr:\t"); Serial.print(srLeftAvg); Serial.print("\t");
    //    Serial.print("rt snr:\t"); Serial.print(srRightAvg); Serial.println("\t");
    isObstacle = true;
    stop();//stop the robot
  }
  else {
    bitSet(state, movingR);//set right motor moving
    bitSet(state, movingL);//set left motor moving
//    Serial.println("no obstacle detected");
    isObstacle = false;
  }
}


/* Motion Commands */

/*
  Description: 
    Moves the robot forward in a straight line.

  Input: 
    inches - the number of inches to move the robot.
    inputSpeed - linear speed (in/s)
  
  Return: nothing
*/
void forward(long inches, long inputSpeed) {
  long distance = inches * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;
  long tickSpeed = inputSpeed * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;

  // reset
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  
  stepperRight.moveTo(distance);//move distance
  stepperLeft.moveTo(distance);//move distance
  stepperRight.setSpeed(tickSpeed);//set speed
  stepperLeft.setSpeed(tickSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
}

/*
  Description: 
    Moves the robot backwards in a straight line.

  Input: 
    inches - the number of inches to move the robot.
    inputSpeed - linear speed (in/s)
  
  Return: nothing
*/
void reverse(long inches, long inputSpeed) {
  long distance = inches * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;
  long tickSpeed = inputSpeed * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;

  // reset
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  
  stepperRight.moveTo(-distance);     //move distance backwards
  stepperLeft.moveTo(-distance);      //move distance backwards
  stepperRight.setSpeed(tickSpeed);//set speed
  stepperLeft.setSpeed(tickSpeed);//set speed
  stepperRight.runSpeedToPosition();  //move right motor
  stepperLeft.runSpeedToPosition();   //move left motor
  runToStop();              //run until the robot reaches the target
}


void turnRight(int rot) {
  long positions[2]; // Array of desired stepper positions
  Serial.print("reverse\t");
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() - two_rotation; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() - two_rotation; //left motor absolute position
  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in
  positions[0] = stepperRight.currentPosition() - rot; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
*/
void updateSensors() {
  //  Serial.print("updateSensors\t");
  //  Serial.println(test_state);
  test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  updateIR();     //update IR readings and update flag variable and state machine
  updateSonar();  //update Sonar readings and update flag variable and state machine
}

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/

void updateIR() {
  irFrontAvg = analogRead(irFront);
  irRearAvg = analogRead(irRear);
  irLeftAvg = analogRead(irLeft);
  irRightAvg = analogRead(irRight);
  //  print IR data
//      Serial.println("frontIR\tbackIR\tleftIR\trightIR");
//      Serial.print(irFrontAvg); Serial.print("\t");
//      Serial.print(irRearAvg); Serial.print("\t");
//      Serial.print(irLeftAvg); Serial.print("\t");
//      Serial.println(irRightAvg);
}


/*
  This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar() {
  
//  for (int i = 0; i < 5; i++) {
//    //Activate the right sonar
//    pinMode(snrRight, OUTPUT); //set the PING pin as an output
//    digitalWrite(snrRight, LOW); //set the PING pin low first
//    delayMicroseconds(2);//wait 2 us
//    digitalWrite(snrRight, HIGH); //trigger sonar by a 2 us HIGH PULSE
//    delayMicroseconds(5);//wait 5 us
//    pinMode(snrRight, INPUT);//set pin as input with duration as reception time
//    rightVal = rightVal + pulseIn(snrRight, HIGH); //measures how long the pin is high
//    //Activate the left sonar
//    pinMode(snrLeft, OUTPUT); //set the PING pin as an output
//    digitalWrite(snrLeft, LOW); //set the PING pin low first
//    delayMicroseconds(2);
//    digitalWrite(snrLeft, HIGH); //trigger sonar by a 2 us HIGH PULSE
//    delayMicroseconds(5);
//    digitalWrite(snrLeft, LOW); //set pin low first again
//    pinMode(snrLeft, INPUT);//set pin as input with duration as reception time
//    leftVal = leftVal + pulseIn(snrLeft, HIGH);
//  }
//  rightVal = rightVal / 5; //averages right values
//  leftVal = leftVal / 5;   //averages left values
//  srRightInches = .0069 * rightVal; //converts right values to inches
//  srLeftInches = .007 * leftVal;    //converts left values to inches
//
//  srRightAvg = rightVal;
//  srLeftAvg = leftVal;
  // print
//    Serial.print("lt snr:\t");
//    Serial.print(leftVal);
//    Serial.print(" in\t");
//    Serial.print("rt snr:\t");
//    Serial.print(rightVal);
//    Serial.println(" in");

}

//runToStop() is a function to run the individual motors without blocking
void runToStop() {
  int runNow = 1;
  bool reload = false;
  long leftDistance = stepperLeft.targetPosition();
  long rightDistance = stepperRight.targetPosition();

  long distanceLeftToGo = leftDistance;
  long distanceRightToGo = rightDistance;
  
  float leftSpeed = stepperLeft.speed();
  float rightSpeed = stepperRight.speed();
    
  stepperRight.setMaxSpeed(rightSpeed);
  stepperLeft.setMaxSpeed(leftSpeed);
  while (runNow) {
//    Serial.println(isObstacle);
    if (!isObstacle && reload) {
      stepperRight.setMaxSpeed(rightSpeed);
      stepperLeft.setMaxSpeed(leftSpeed);
      stepperRight.move(distanceRightToGo);
      stepperLeft.move(distanceLeftToGo);
      reload = false;
    } else {
      distanceLeftToGo = stepperLeft.distanceToGo();
      distanceRightToGo = stepperRight.distanceToGo();
      reload = true;
      stepperRight.stop();
      stepperLeft.stop();
    }
    
    if (!stepperRight.run() && !isObstacle) {
      bitClear(state, movingR);  // clear bit for right motor moving
    }
    if (!stepperLeft.run() && !isObstacle) {
      bitClear(state, movingL);   // clear bit for left motor moving
    }
    if ((state & 0b11) == 0 ) runNow = 0;
  }
}
