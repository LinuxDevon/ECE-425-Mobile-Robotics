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
#define button    A15    //pushbutton 

///////////// NEW SONAR CLASSES FOR TIMER 2 INTERRUPT/////////////////
//define sonar sensor connections
#define snrLeft   8   //front left sonar 
#define snrRight  9  //front right sonar 
#define SONAR_NUM     2         // Number of sensors.
#define MAX_DISTANCE 200        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 125        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define FIRST_PING_START 50     // First ping starts at this time in ms, gives time for the Arduino to chill before starting.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(snrLeft, snrLeft, MAX_DISTANCE),//create an instance of the left sonar
  NewPing(snrRight, snrRight, MAX_DISTANCE),//create an instance of the right sonar
};
////////////////////////////////////////////////////////////////////

#define irThresh    6 // The IR threshold for presence of an obstacle
#define snrThresh   6  // The sonar threshold for presence of an obstacle
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
//#define timer_int     250000

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

#define DETECT_DIST 6

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
bool freezeAtObstacle;

enum States {
  goToGoalState,
  avoidState,
  wanderState
};

States State;
#define StartX 36
#define StartY 36

int Xpos;
int Ypos;


void setup() {
  //multipler sonar on timer 2 setup
  pingTimer[0] = millis() + FIRST_PING_START;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++)               // Set the starting time for each sensor.
  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
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
  freezeAtObstacle = true; //robot will freeze at obstacles by default
  State = goToGoalState; // start state is go to goal
  Xpos = StartX;
  Ypos = StartY;
   
  //Timer Interrupt Set Up
//  Timer1.initialize(timer_int);         // initialize timer1, and set a 1/2 second period
//  Timer1.attachInterrupt(obsRoutine);  // attaches updateSensors() as a timer overflow interrupt

  //Set up serial communication
  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
  delay(2500); //seconds before the robot moves

  // LED SETUP
  // set as outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
}

void loop() {
//  forward(12,12);
//  delay(5000);
//  reverse(12,12);
//  delay(5000);
freezeAtObstacle = false;
//  randomWander();
//  goToGoal(36,36);
//  shyKid(); 
//  freezeAtObstacle = true;
//  delay (1000);

  goToGoalWithAvoid();
//  spin(RIGHT, 45, 45);
//  spin(LEFT, 180, 90);
  
}


void obsRoutine() {
  updateSensors();
  
  if ((irFrontAvg < irThresh)     // check front ir
       || (irRearAvg < irThresh)) {   // check rear ir;
    if(freezeAtObstacle == true) {
      isObstacle = true;
//      freezeAtObstacle = false;
      State = avoidState;
      stop();//stop the robot
    }
  }
  else {
    bitSet(state, movingR);//set right motor moving
    bitSet(state, movingL);//set left motor moving
    isObstacle = false;
  }
}

/*
 * 
 */
void goToGoalWithAvoid(void) {
    freezeAtObstacle = true; // stop robot if detected to change states
//    Serial.println(State);
    switch (State) {
      case goToGoalState:
        goToGoal(Xpos, Ypos);
        break;
      case avoidState:
        avoidObstacle();
        break;
      case wanderState:
        randomWander();
        break;
    }
  
}

void avoidObstacle(void) {
  freezeAtObstacle = false;
  isObstacle = false;
  long numberOfSpins = 0;
  long smallTurn = 10; // ten degrees
  bool avoided = false;
  int inchesMoved = 0;

  int turnExtra = 1;

  float turnVectorX = 0;
  float turnVectorY = 0;
  long angleToTurnBack = 0;

  // snap where the robot stopped at
  int StopRight = stepperRight.currentPosition();
  int StopLeft = stepperLeft.currentPosition();

  int TargetRight = stepperRight.targetPosition();
  int TargetLeft = stepperLeft.targetPosition();

  // reset position
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);

  while(!avoided) {
    // only going to have a problem if it is in front of the robot
    // turn right until the robot doesn't see the obstacle
//    while(irFrontAvg < DETECT_DIST || irLeftAvg > DETECT_DIST - 3) {
////      Serial.println("Turning");
//      // spin right a little till conditions are met;
//      freezeAtObstacle = false;
//      spin(RIGHT, smallTurn, 45); 
//  
//      // add up each small turn steps
////      StopRight += stepperRight.currentPosition();
////      StopLeft += stepperLeft.currentPosition();
//    }
    spin(RIGHT, 90, 45);
    delay(500);
    while (irLeftAvg < DETECT_DIST){
      //keep moving forward till obstacle is completely gone
      forward(1, 6);

      // add up each small turn steps
      StopRight += stepperRight.currentPosition();
      StopLeft += stepperLeft.currentPosition();
    }
    // spin the same directions before avoiding
    delay(500);
    spin(LEFT, 90, 45);
//    if(numberOfSpins != 0) {
//      angleToTurnBack = smallTurn*numberOfSpins;
//      turnVectorX = inchesMoved*cos(angleToTurnBack);
//      turnVectorY = inchesMoved*sin(angleToTurnBack);
//    }

    // move forward to check the left sensor to verify we aren't in a dead spot
    delay(500);
    forward(3, 6);
    if (irLeftAvg > DETECT_DIST) {  // left sensor clear so good to make the last movement
      StopRight += stepperRight.currentPosition();
      StopLeft += stepperLeft.currentPosition();
      avoided = true;
    }
//    inchesMoved += 5;
//    numberOfSpins = 0;

  }

  // recalculate the new position
//  spin(LEFT, angleToTurnBack, 45);  // face the correct way now just x position matters
  Ypos = 0;

  Xpos = Xpos - (turnVectorY + inchesMoved + (TargetRight * INCHES_FOR_FULL_WHEEL_SPIN/TICKS_FOR_FULL_WHEEL_SPIN)); 

  Serial.println("Finished Avoiding");
  Serial.println(Xpos);
  Serial.println(Ypos);
  // set state to go to goal since object is avoided
  State = goToGoalState;
  
}

void shyKid() {
  freezeAtObstacle = false;
  
  int forward = 0;
  int backward = 0;
  int left = 0;
  int right = 0;
  int leftD = 0;
  int rightD = 0;

  digitalWrite(YELLOW_LED, HIGH);  // turn on the yellow led for this function

  // check the to see which sensors are tripped within 6 inches
  if (irRearAvg < DETECT_DIST) {
    forward = 10000/irRearAvg;
  }

  if (irFrontAvg < DETECT_DIST) {
    backward = 10000/irFrontAvg;
  }

  if (irRightAvg < DETECT_DIST) {
    left = 10000/irRightAvg;
  }

  if (irLeftAvg < DETECT_DIST) {
    right = 10000/irLeftAvg;
  }

  // Handle if Left and Right are tripped
  if(irRightAvg < DETECT_DIST && irLeftAvg < DETECT_DIST && irRearAvg > DETECT_DIST && irFrontAvg > DETECT_DIST) {
    left = 0;
    right = 0;
    forward = 800;

  // Handle if Front and Rear are tripped
  } else if(irRightAvg > DETECT_DIST && irLeftAvg > DETECT_DIST && irRearAvg < DETECT_DIST && irFrontAvg < DETECT_DIST) {
    spin(1,90,45);
    left = 0;
    right = 0;
    forward = 0;
    backward = 0;

  // handle if Front, Rear or one side is tripped
  } else if(irRightAvg < DETECT_DIST && irLeftAvg < DETECT_DIST && (irRearAvg < DETECT_DIST || irFrontAvg < DETECT_DIST)) {
    if(backward > forward) {
      spin(1,180,45);
      forward = 0;
      backward = 0;
    }
    left = 0;
    right = 0;
  } else if(irRightAvg > DETECT_DIST && irLeftAvg < DETECT_DIST && irRearAvg < DETECT_DIST && irFrontAvg < DETECT_DIST) {
    spin(1,90,45);
    left = 0;
    right = 0;
    forward = 0;
    backward = 0;
  } else if(irRightAvg < DETECT_DIST && irLeftAvg > DETECT_DIST && irRearAvg < DETECT_DIST && irFrontAvg < DETECT_DIST) {
    spin(1,-90,45);
    left = 0;
    right = 0;
    forward = 0;
    backward = 0;
  }
  
  int rightWSpeed = forward - backward + left - right;
  int leftWSpeed = forward - backward - left + right;
//  Serial.println(rightWSpeed);
//  Serial.println(leftWSpeed);

  // if any of the sensors are tripped set the distance to a quarter revolution
  if (irLeftAvg < DETECT_DIST || irRightAvg < DETECT_DIST || irFrontAvg < DETECT_DIST || irRearAvg < DETECT_DIST) {
    rightD = 200;
    leftD = 200;
  }
  
  if(rightWSpeed < 0) {
    rightD = -rightD;
  }

  
  if(leftWSpeed < 0) {
    leftD = -leftD;
  }

  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(rightD);//move distance
  stepperLeft.moveTo(leftD);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  
  digitalWrite(YELLOW_LED, LOW);  // turn off the yellow led
}

void checkShyKid() {
  while(irRearAvg < DETECT_DIST || irFrontAvg < DETECT_DIST || irRightAvg < DETECT_DIST || irLeftAvg < DETECT_DIST) {
    shyKid();
  }
}

void randomWander() {
   checkShyKid();

   digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
  
  long rightDSign = random(1,1000);
  if(rightDSign % 2 == 1) {
    rightDSign = -1;
  } else {
    rightDSign = 1;
  }
  
  long leftDSign = random(1,1000);
  if(leftDSign % 2 == 1) {
    leftDSign = -1;
  } else {
    leftDSign = 1;
  }
  
  long rightSSign = random(1,1000);
  if(rightSSign % 2 == 1) {
    rightSSign = -1;
  } else {
    rightSSign = 1;
  }
  
  long leftSSign = random(1,1000);
  if(leftSSign % 2 == 1) {
    leftSSign = -1;
  } else {
    leftSSign = 1;
  }
  
  long rightDistance = rightDSign * random(400,1200);
  long leftDistance = leftDSign * random(400,1200);
  long rightSpeed = rightSSign * random(400,1200);
  long leftSpeed = leftSSign * random(400,1200);
  
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(rightDistance);//move distance
  stepperLeft.moveTo(leftDistance);//move distance
  stepperRight.setSpeed(rightSpeed);//set speed
  stepperLeft.setSpeed(leftSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target

  digitalWrite(GREEN_LED, LOW);  // turn off the green led
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
//  checkShyKid();

  digitalWrite(RED_LED, HIGH);  // turn on the red led for this function
  
  long distance = inches * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;
  long tickSpeed = inputSpeed * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;

  // reset
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);

  stepperRight.setMaxSpeed(tickSpeed);//set right motor speed
  stepperLeft.setMaxSpeed(tickSpeed);//set left motor speed
  stepperRight.moveTo(distance);//move distance
  stepperLeft.moveTo(distance);//move distance
  stepperRight.setSpeed(tickSpeed);//set speed
  stepperLeft.setSpeed(tickSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target

  digitalWrite(RED_LED, LOW);  // turn off the red led
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

/*
  Description: 
    Spin is similar to pivot but instead turns the wheels at the same speed in opposite
    directions.

  Input: 
    direction - It can be left or right where left = 0, right = 1
    angle - the angle in degrees to turn
    inputSpeed - angular speed (degrees/s)

  Return: nothing
*/
void spin(int direction,long angle, long inputSpeed) {
  long ticks = (angle * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  long tickSpeed = (inputSpeed * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setMaxSpeed(tickSpeed);//set right motor speed
  stepperLeft.setMaxSpeed(tickSpeed);//set left motor speed
  
  if(direction == LEFT) {
    stepperRight.moveTo(ticks);//move distance
    stepperLeft.moveTo(-ticks);//move distance
  } else if(direction == RIGHT) {
    stepperRight.moveTo(-ticks);//move distance
    stepperLeft.moveTo(ticks);//move distance
  }

  stepperRight.setSpeed(tickSpeed);//set right motor speed
  stepperLeft.setSpeed(tickSpeed);//set left motor speed
  
  stepperRight.runSpeedToPosition();//set right motor speed
  stepperLeft.runSpeedToPosition();//set left motor speed
  runToStop();
}

/*
  Description: 
    Pivot keeps one wheel stationary and the other wheel spins until the desired angle.

  Input: 
    direction - It can be left or right where left = 0, right = 1
    angle - the angle in degrees to turn
    inputSpeed - angular speed (degrees/s)
  
  Return: nothing
*/
void pivot(int direction, long angle, long inputSpeed) {
//  checkShyKid();
  
  long ticks = (angle * 2 * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;//FULL_CIRCLE_TICKS_COUNT was set for spin(), so we multiply it by 2 to work here
  long tickSpeed = (inputSpeed * 2 * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  if(direction == LEFT) {
    stepperRight.setSpeed(tickSpeed);//set right motor speed
    stepperRight.moveTo(ticks);//move distance
    stepperRight.runSpeedToPosition();//set right motor speed
  } else if(direction == RIGHT) {
    stepperLeft.setSpeed(tickSpeed);//set left motor speed
    stepperLeft.moveTo(ticks);//move distance
    stepperLeft.runSpeedToPosition();//set left motor speed
  }
  runToStop();
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  Description: 
    Moves the robot to face the given angle by calling pivot. 

  Input: 
    angle - the angle in degrees to turn. 
  
  Return: nothing
*/
void goToAngle(int angle) {
//  checkShyKid();
  
//  digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
 
  if(angle > 0) {
    spin(LEFT, angle, 90);//90 sets it to 90 degrees per second
  } else  if (angle < 0) {
    spin(RIGHT, -angle, 90);
  }

//  digitalWrite(GREEN_LED, LOW); // turn off leds
}

/*
  Description: 
    Calculates the angle and length to move the robot to the given position.
    It calls goToAngle based on the calculated angle then finally calls forward()
    to move the correct calculated distance.

  Input: 
    x - is the x position where positive x is in front of the robot and negative is backwards
    y - is the y position where positive y is to the left and negative is the right.
  
  Return: nothing
*/
void goToGoal(double x, double y) {
//  checkShyKid();

  digitalWrite(RED_LED, HIGH);  // turn on the green led for this function
  digitalWrite(GREEN_LED, HIGH);  // turn on the green and yellow led for this function
  digitalWrite(YELLOW_LED, HIGH); 
   
  int angle;
  if(x > 0 && y > 0) {        // correctly calculates if left front quadrant
    angle = atan2(y,x)*180/Pi;
  } else if(x > 0 && y < 0) {   // correctly calculates if right front quadrant
    angle = atan2(abs(y),x)*180/Pi;
    angle = -angle;
  } else if(x < 0 && y > 0) {   // correctly calculates if left rear quadrant
    angle = atan2(y,abs(x))*180/Pi;
    angle = 180 - angle;
  } else if(x < 0 && y < 0) {   // correctly calculates if right rear quadrant
    angle = atan2(abs(y),abs(x))*180/Pi;
    angle = angle + 180;
  } else if(x == 0 && y > 0) {    // directly right
    angle = RIGHT_ANGLE;
  } else if(x == 0 && y < 0) {    // directly left
    angle = -RIGHT_ANGLE;
  } else if(x > 0 && y == 0) {    // straight ahead
    angle = 0;
  } else if(x < 0 && y == 0) {    // directly behind
    angle = 180;
  } else {              // dont' move because both x,y are 0    
    angle = 0;
  }
//  Serial.println(angle);
  goToAngle(angle);   // turn to the calculated angle
  delay(500);
  long distance = sqrt(x*x + y*y);  // calculates the distance to travel at the given angle
  forward(distance, 12);//12 sets the speed to 12 inches per second
  delay(500);

  digitalWrite(GREEN_LED, LOW);  // turn off leds
  digitalWrite(YELLOW_LED, LOW);  
  digitalWrite(RED_LED, LOW);
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
//  updateSonar();  //update Sonar readings and update flag variable and state machine
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

  irFrontAvg = (1111/(irFrontAvg+16))-1;
  irRearAvg = (1111/(irRearAvg+20))-1;
  irLeftAvg = (285714/(irLeftAvg+2257))-103;
  irRightAvg = (286714/(irRightAvg+2600))-90;

  if(irFrontAvg <= 0) {
    irFrontAvg = 1;
  }

  if(irRearAvg <= 0) {
    irRearAvg = 1;
  }

  if(irLeftAvg <= 0) {
    irLeftAvg = 1;
  }

  if(irRightAvg <= 0) {
    irRightAvg = 1;
  }
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
  test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    //    Serial.print("\t\t\t");
    //    Serial.print(millis());
    //    Serial.print("\t");
    //    Serial.print(pingTimer[i]);
    //    Serial.print("\t");
    //    Serial.println(PING_INTERVAL);
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        //oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        if (cm[0] > 0)
          srLeftAvg = cm[0];
        if (cm[1] > 0)
          srRightAvg = cm[1];
//        Serial.print("lt snr:\t");
//        Serial.print(srLeftAvg);
//        Serial.print(" cm ");
//        Serial.print("\trt snr:\t");
//        Serial.print(srRightAvg);
//        Serial.println(" cm");
      }
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  srLeftAvg = (-479616/(srLeftAvg-9520))-50;
  srRightAvg = (-568181/(srRightAvg-11136))-50;
}

//This function writes to the sonar data if the ping is received
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//This function prints the sonar data once all sonars have been read
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(i);
    //Serial.print(" = ");
    //Serial.print(cm[i]);
    //Serial.print(" cm\t");
  }
  srLeftAvg = cm[0];
  srRightAvg = cm[1];
  //  Serial.print("Left Sonar = ");
  //  Serial.print(srLeftAvg);
  //  Serial.print("\t\tRight Sonar = ");
  //  Serial.print(srRightAvg);
  //  Serial.println();
}

//runToStop() is a function to run the individual motors without blocking
void runToStop() {
  int runNow = 1;
  long leftDistance = stepperLeft.targetPosition();
  long rightDistance = stepperRight.targetPosition();

  float leftSpeed = stepperLeft.speed();
  float rightSpeed = stepperRight.speed();

  int runLeft = 0;
  int runRight = 0;
    
//  stepperRight.setMaxSpeed(max_spd);
//  stepperLeft.setMaxSpeed(max_spd);
//      Serial.print("obstacle: ")

  while (runNow && (leftSpeed != 0 && rightSpeed !=0)) {
    if (!isObstacle) {
//      Serial.println("Setting");
      stepperRight.setMaxSpeed(rightSpeed);
      stepperLeft.setMaxSpeed(leftSpeed);
      stepperRight.moveTo(rightDistance);
      stepperLeft.moveTo(leftDistance);
//      stepperRight.setSpeed(rightSpeed);
//      stepperLeft.setSpeed(leftSpeed);
    } else {
//      Serial.println("HERE");
      runNow = 0;
      leftSpeed = 0;
      rightSpeed = 0;
//      stepperRight.stop();
//      stepperLeft.stop();
     
    }
    runRight = stepperRight.run();
    if (!runRight && !isObstacle) {
      bitClear(state, movingR);  // clear bit for right motor moving
    }
    runLeft = stepperLeft.run();
    if (!runLeft && !isObstacle) {
      bitClear(state, movingL);   // clear bit for left motor moving
    }

    if ((state & 0b11) == 0 ) runNow = 0;
//      Serial.print("isObs: ");
//    Serial.println(isObstacle);
  }
//  Serial.println("DONE");
//  Serial.println(isObstacle);
  
}
