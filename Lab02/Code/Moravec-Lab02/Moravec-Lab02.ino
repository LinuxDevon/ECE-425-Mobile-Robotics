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

  Authors: Devon Adair & Hunter LaMantia
  Date Created: 12/12/2018
  Description: Does basic movement functions
  forward(): moves robot forward
  reverse(): moves robot backward
  spin(): robot spins in place
  pivot(): robot pivots around one wheel
  turn(): robot moves in an arc
  stop(): stops robot movement
  moveCircle(); robot moves in a circle
  moveFigure8(): robot moves in a figure 8
  moveSquare(): robot moves in a square
  goToAngle(): robot turns to face a specific angle
  goToGoal(): robot turns toward goal and move towards it
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
#define Pi 3.14159265358979

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
//  Serial.begin(9600); //start serial communication at 9600 baud rate for debugging

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
    inputSpeed - angular speed (degrees/s)
	
	Return: nothing
*/
void pivot(int direction, long angle, long inputSpeed) {
  long ticks = (angle * 2 * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;//FULL_CIRCLE_TICKS_COUNT was set for spin(), so we multiply it by 2 to work here
  long tickSpeed = (inputSpeed * 2 * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  if(direction == LEFT) {
    stepperRight.setMaxSpeed(tickSpeed);//set right motor speed
    stepperRight.move(ticks);//move distance
    stepperRight.runSpeedToPosition();//set right motor speed
  } else if(direction == RIGHT) {
    stepperLeft.setMaxSpeed(tickSpeed);//set left motor speed
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
    inputSpeed - angular speed (degrees/s)

	Return: nothing
*/
void spin(int direction,long angle, long inputSpeed) {
  long ticks = (angle * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  long tickSpeed = (inputSpeed * FULL_CIRCLE_TICKS_COUNT)/FULL_SPIN;
  if(direction == LEFT) {
    stepperRight.move(ticks);//move distance
    stepperLeft.move(-ticks);//move distance
  } else if(direction == RIGHT) {
    stepperRight.move(-ticks);//move distance
    stepperLeft.move(ticks);//move distance
  }
  stepperRight.setMaxSpeed(tickSpeed);//set right motor speed
  stepperLeft.setMaxSpeed(tickSpeed);//set left motor speed
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
void turn(int direction, float angle, float diameter) {
  float numerator = diameter + 15.75;//calculated from geometry of robot
  float circumferenceRatio = numerator / diameter;//ratio calculated from geometry of robot
  float percentOfCircle = angle/360;
  float scaling = diameter/36;//calibrated via trial and error
  long innerCorrectionFactor = 400 * percentOfCircle * scaling;//400 from trial and error
  float inchesToTicks = TICKS_FOR_FULL_WHEEL_SPIN/INCHES_FOR_FULL_WHEEL_SPIN;
  float correctedTicksInnerWheel = percentOfCircle * PI * diameter * inchesToTicks + innerCorrectionFactor;
  float correctedTicksOuterWheel = correctedTicksInnerWheel * circumferenceRatio;

  float slowSpeed = 400;//speed that we found to make the robot behave well at a good range of diameters
  float fastSpeed = slowSpeed * circumferenceRatio;
  
  if(direction == LEFT) {
    stepperRight.move(correctedTicksOuterWheel);//move distance
    stepperLeft.move(correctedTicksInnerWheel);//move distance
    stepperRight.setMaxSpeed(fastSpeed);//set right motor speed
    stepperLeft.setMaxSpeed(slowSpeed);//set left motor speed
  } else if(direction == RIGHT) {
    stepperRight.move(correctedTicksInnerWheel);//move distance
    stepperLeft.move(correctedTicksOuterWheel);//move distance
    stepperRight.setMaxSpeed(slowSpeed);//set right motor speed
    stepperLeft.setMaxSpeed(fastSpeed);//set left motor speed
  }
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();//set left motor speed
  runToStop();
}

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
  stepperRight.move(distance);//move distance
  stepperLeft.move(distance);//move distance
  stepperRight.setMaxSpeed(tickSpeed);//set speed
  stepperLeft.setMaxSpeed(tickSpeed);//set speed
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
  stepperRight.move(-distance);			//move distance backwards
  stepperLeft.move(-distance);			//move distance backwards
  stepperRight.setMaxSpeed(tickSpeed);//set speed
  stepperLeft.setMaxSpeed(tickSpeed);//set speed
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
    pivot(LEFT, angle, 90);//90 sets it to 90 degrees per second
  } else  if (angle < 0) {
    pivot(RIGHT, -angle, 90);
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
  forward(distance, 12);//12 sets the speed to 12 inches per second

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
		calls turn() in order to move in a complete circle

	Input: 
    dir - direction (left or right)
    diameter - diameter of circle (inches)
	
	Return: nothing
*/
void moveCircle(int dir, int diameter) {
  digitalWrite(RED_LED, HIGH);  // turn on the red led for this function
  turn(dir, 360, diameter); // 360 makes it do a full circle
  digitalWrite(RED_LED, LOW);
}

/*
	Description: 
		The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  		twice with 2 different direcitons to create a figure 8 with circles of the given diameter.

	Input: 
    diam - diameter of the two circles
	
	Return: nothing
*/
void moveFigure8(int diam) {
  digitalWrite(RED_LED, HIGH);  // turn on the red and yellow led for this function
  digitalWrite(YELLOW_LED, HIGH);  
  moveCircle(LEFT,diam);
  moveCircle(RIGHT,diam);
  digitalWrite(RED_LED, LOW);  // turn off the leds set for this function
  digitalWrite(YELLOW_LED, LOW);
}
