/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  goToAngle - given an angle input use odomery to sturn the rob
  goToGoal - given an x and y position in feet, use the goToAngle() function and trigonometry to move to a goal positoin
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveSquare - given the side length in inches, move the robot in a square with the given side length
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME
  
  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 46 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 44 - left stepper motor step pin
  digital pin 49 - left stepper motor direction pin
  digital pin 14 - red LED in series with 220 ohm resistor
  digital pin 15 - green LED in series with 220 ohm resistor
  digital pin 16 - yellow LED in series with 220 ohm resistor
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <math.h>

//define pin numbers
const int rtStepPin = 46; //right stepper motor step pin
const int rtDirPin = 53;  // right stepper motor direction pin
const int ltStepPin = 44; //left stepper motor step pin
const int ltDirPin = 49;  //left stepper motor direction pin
const int stepTime = 500; //delay time between high and low on step pin
const int redLED = 14;  //state red LED
const int grnLED = 15;  //state green LED
const int ylwLED = 16;  //state yellow LED

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

// USER DEFINES
#define LEFT 0
#define RIGHT 1
#define Pi 3.15149265358979

#define REST_DELAY 500				// half second delay
#define ONE_SECOND 1000 			// one second delay
#define FULL_SPIN 360				// 360 degrees
#define TICKS_FOR_FULL_WHEEL_SPIN 800
#define INCHES_FOR_FULL_WHEEL_SPIN 10.5
#define RIGHT_ANGLE 90				// 90 degrees
#define FULL_CIRCLE_TICKS_COUNT 1900 // number of ticks to make a full spin

#define RED_LED 14
#define GREEN_LED 16
#define YELLOW_LED 15


void setup()
{
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(1000); //always wait 1 second before the robot moves
  Serial.begin(9600); //start serial communication at 9600 baud rate for debugging

  // LED SETUP
  // set as outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  // turn off at first
  // 0 - off , 1 - on
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
}

void loop()
{
  //uncomment each function one at a time to see what the code does

//800 ticks = 10.5 inches
//800 ticks = 135 degrees
//100 speed = 10.5in / 0.5s = 21in/s
//100 speed = in / s = rad/s (pivoting)
//100 speed = in / s = rad/s (spinning)
//100 speed = in / s = rad/s (turning)
  
//  move1();//call move back and forth function
//  move2();//call move back and forth function with AccelStepper library functions
//  move3();//call move back and forth function with MultiStepper library functions
//  stepperRight.move(800);
//  stepperLeft.move(800);
//  runToStop();
//  delay(500);
//  reverse(12);
//  delay(500);
//  spin(LEFT, 360);
//  delay(500);
//  spin(RIGHT, 360);
//  delay(500);
//  pivot(LEFT, 360);
//  delay(500);
//  pivot(RIGHT, 360);
//  delay(500);
  turn(LEFT, 360, 12);
//  delay(500);
//  turn(RIGHT, 360, 12);

//  moveSquare(12);
//  goToAngle(720);

  delay(100000);
}

// -- PREWRITTEN FUNCTIONS OF MOVEMENT -- //
/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

// -- BASIC MOVEMENT FUNCTIONS -- //
/*
	Description: 
		Pivot keeps one wheel stationary and the other wheel spins until the desired angle.

	Input: 
		direction - It can be left or right where left = 0, right = 1
		angle - the angle in degrees to turn
	
	Return: nothing
*/
void pivot(int direction, long angle) {
  long ticks = (angle * 3750)/360;
  float inputSpeed = 100;
  if(direction == LEFT) {
    stepperRight.setSpeed(inputSpeed);//set right motor speed
    stepperRight.move(ticks);//move distance
    stepperRight.runSpeedToPosition();//set right motor speed
  } else if(direction == RIGHT) {
    stepperLeft.setSpeed(inputSpeed);//set left motor speed
    stepperLeft.move(ticks);//move distance
    stepperLeft.runSpeedToPosition();//set left motor speed
  }
  runToStop();
}


/*
	Description: 
		Spin is similar to pivot but instead turns the wheels at the same speed in opposite
		directions.

	Input: 
		direction - It can be left or right where left = 0, right = 1
		angle - the angle in degrees to turn

	Return: nothing
*/
void spin(int direction,long angle) {
  long ticks = (angle * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  float inputSpeed = 100;
  if(direction == LEFT) {
    stepperRight.move(ticks);//move distance
    stepperLeft.move(-ticks);//move distance
  } else if(direction == RIGHT) {
    stepperRight.move(-ticks);//move distance
    stepperLeft.move(ticks);//move distance
  }
  stepperRight.setSpeed(inputSpeed);//set right motor speed
  stepperLeft.setSpeed(inputSpeed);//set left motor speed
  stepperRight.runSpeedToPosition();//set right motor speed
  stepperLeft.runSpeedToPosition();//set left motor speed
  runToStop();
}

/*
	Description: 
      This function turns the robot a given a direction

	Input: 
      direction - LEFT or RIGHT, 0 or 1, the direction to go
      angle - the amount to go around the "circle"
      diameter - the size of the "circle" to go around
	
	Return: nothing
*/
void turn(int direction, long angle, long diameter) {
  float circumferenceRatio = (diameter + 16.5) / diameter;
  long ticksInnerWheel = (PI * diameter * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN) * (angle/360);
  long ticksOuterWheel = ticksInnerWheel * circumferenceRatio;

  float slowSpeed = 457;  // ~ 6 in/s
  float fastSpeed = slowSpeed*circumferenceRatio;

  Serial.println(fastSpeed);
  Serial.println(circumferenceRatio);
  Serial.println(ticksInnerWheel);
  Serial.println(ticksOuterWheel);
  if(direction == LEFT) {
    stepperRight.move(ticksOuterWheel);//move distance
    stepperLeft.move(ticksInnerWheel);//move distance
    stepperRight.setMaxSpeed(fastSpeed);//set right motor speed
    stepperLeft.setMaxSpeed(slowSpeed);//set left motor speed
  } else if(direction == RIGHT) {
    stepperRight.move(ticksInnerWheel);//move distance
    stepperLeft.move(ticksOuterWheel);//move distance
    stepperRight.setMaxSpeed(slowSpeed);//set right motor speed
    stepperLeft.setMaxSpeed(fastSpeed);//set left motor speed
  }
  stepperRight.runSpeedToPosition();//set right motor speed
  stepperLeft.runSpeedToPosition();//set left motor speed
  runToStop();
}

/*
	Description: 
		Moves the robot forward in a straight line.

	Input: 
		inches - the number of inches to move the robot.
	
	Return: nothing
*/
void forward(long inches) {
  long distance = inches * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;
  stepperRight.move(distance);//move distance
  stepperLeft.move(distance);//move distance
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
}

/*
	Description: 
		Moves the robot backwards in a straight line.

	Input: 
		inches - the number of inches to move the robot.
	
	Return: nothing
*/
void reverse(long inches) {
  long distance = inches * TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;	
  stepperRight.move(-distance);			//move distance backwards
  stepperLeft.move(-distance);			//move distance backwards
  stepperRight.runSpeedToPosition();	//move right motor
  stepperLeft.runSpeedToPosition();		//move left motor
  runToStop();							//run until the robot reaches the target
}

/*
	Description: 
		Stops the robot from moving.

	Input: nothing
	
	Return: nothing
*/
void stop() {
  stepperRight.stop();	//stop right motor
  stepperLeft.stop();	//stop left motor
}


// -- SPECIAL MOVEMENT FUNCTIONS -- //

/*
	Description: 
		Moves the robot to face the given angle by calling pivot. 

	Input: 
		angle - the angle in degrees to turn. 
	
	Return: nothing
*/
void goToAngle(int angle) {
  digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
 
  if(angle > 0) {
    pivot(LEFT, angle);
  } else  if (angle < 0) {
    pivot(RIGHT, -angle);
  }

  digitalWrite(GREEN_LED, LOW); // turn off leds
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
  digitalWrite(GREEN_LED, HIGH);  // turn on the green and yellow led for this function
  digitalWrite(YELLOW_LED, HIGH); 
   
  int angle;
  if(x > 0 && y > 0) {				// correctly calculates if left front quadrant
    angle = atan2(y,x)*180/Pi;
  } else if(x > 0 && y < 0) {		// correctly calculates if right front quadrant
    angle = atan2(abs(y),x)*180/Pi;
    angle = -angle;
  } else if(x < 0 && y > 0) {		// correctly calculates if left rear quadrant
    angle = atan2(y,abs(x))*180/Pi;
    angle = 180 - angle;
  } else if(x < 0 && y < 0) {		// correctly calculates if right rear quadrant
    angle = atan2(abs(y),abs(x))*180/Pi;
    angle = angle + 180;
  } else if(x == 0 && y > 0) {		// directly right
    angle = RIGHT_ANGLE;
  } else if(x == 0 && y < 0) {		// directly left
    angle = -RIGHT_ANGLE;
  } else if(x > 0 && y == 0) {		// straight ahead
    angle = 0;
  } else if(x < 0 && y == 0) {		// directly behind
    angle = 180;
  } else {							// dont' move because both x,y are 0		
    angle = 0;
  }
  goToAngle(angle);		// turn to the calculated angle
  
  long distance = sqrt(x*x + y*y);	// calculates the distance to travel at the given angle
  forward(distance);

  digitalWrite(GREEN_LED, LOW);  // turn off leds
  digitalWrite(YELLOW_LED, LOW);  
}

/*
	Description: 
		Makes a square given the length of each side. It calls the goToGoal to make each side.

	Input: 
		side - the length of each side in inches
	
	Return: nothing
*/
void moveSquare(int side) {
  digitalWrite(GREEN_LED, HIGH);  // turn on green, red, yellow for this function
  digitalWrite(RED_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  
  goToGoal(abs(side),0);  // move forward
  delay(REST_DELAY);			// Delay after each to give some time so the momentum doesn't throw us off course
  goToGoal(0,side);       // turn
  delay(REST_DELAY);
  goToGoal(0,side);
  delay(REST_DELAY);
  goToGoal(0,side);
  delay(REST_DELAY);
  
  // depending on which way we make the square we need to turn one last time the correct way
  // to face the correct direction to make another sqaure
  if(side > 0) {							
    goToAngle(RIGHT_ANGLE);
  } else if (side < 0) {
    goToAngle(-RIGHT_ANGLE);
  }
  delay(REST_DELAY);

  digitalWrite(GREEN_LED, LOW);     // turn off the leds that were set
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
}
/*
	Description: 
		

	Input: 

	
	Return: nothing
*/
void moveCircle(int diam, int dir) {
  digitalWrite(RED_LED, HIGH);  // turn on the red led for this function
//  speedDiff=
//  turn(dir,360,speeddiff);
  digitalWrite(RED_LED, LOW);
}

/*
	Description: 
		The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  		twice with 2 different direcitons to create a figure 8 with circles of the given diameter.

	Input: 

	
	Return: nothing
*/
void moveFigure8(int diam) {
  digitalWrite(RED_LED, HIGH);  // turn on the red and yellow led for this function
  digitalWrite(YELLOW_LED, HIGH);  

  digitalWrite(RED_LED, LOW);  // turn off the leds set for this function
  digitalWrite(YELLOW_LED, LOW);
}
