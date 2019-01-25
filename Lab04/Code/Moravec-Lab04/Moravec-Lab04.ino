/*Moravec-Lab04.ino
  Author: Devon Adair and Hunter LaMantia
  Date: 1/16/19

  This lab was trying to follow walls using bang-bang control and a p/pd control.
  bang-bang was a more swingy follow method that was rough. The P control is a 
  proportianal controller and takes error into account to proportionally adjust based
  how far away from the wall the robot is. The P controller was smooth enough where we
  didn't need to have the Derivative portion.
  
  We modified wallBang() to support the following of wall roughly. The movement wasn't smooth and was
  jerky.
  We modified updateIR() to convert the raw values to inches to make it easier to tune thresholds.

  We didn't use sonar due to it not working last lab.

  wallP() was the function that we created for our proportional control. It is based on
  wallBang() but the distances it adjusts are proportional to the error or distance from the wall.
  There is a gain value that we tuned to smooth out the movement. It also handles corners, hallways
  and obstacles. When it looses a wall it tries three times then goes to random wander if it looses it.

  We modified runToStop() to disable the timer and enable when done. We found that it wasn't returing 
  correctly and didn't finish the movements we gave it correctly. This fixed the issues and allowed us
  to handle corners correctly.

  The last function we add was randomWander which we copied from last lab. It just picks random
  directions to go and executes.

  All other functions were given and unmodifed.

  Layer0 - our IR sensors recieving data and determinine if there is something there or not. It also
           involves avoid obstacle which needs to happen when following a wall or wandering.
  Layer1 - Follow Wall which is done with bang-bang or P that we created. This is when this is just one
           wall.
  Layer2 - Follow Center is when it detects a wall on either side which. This is done with minor adjustments
           that are proportional to each wall.
  Layer3 - This layer is random wander where if we loose a wall move around to find it or another wall.  
  
  Hardware Connections:
  Stepper Enable          Pin 48
  Right Stepper Step      Pin 46
  Right Stepper Direction Pin 53
  Left Stepper Step       Pin 44
  Left Stepper Direction  Pin 49

  Front IR    A8
  Back IR     A9
  Right IR    A10
  Left IR     A11
  Left Sonar  A12
  Right Sonar A13
  Button      A15
  Left Light  A1
  Right Light A0
*/

// -- INCLUDES --//
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>//include timer interrupt library

//define stepper motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin     46  //right stepper motor step pin
#define rtDirPin      53  // right stepper motor direction pin
#define ltStepPin     44  //left stepper motor step pin
#define ltDirPin      49  //left stepper motor direction pin

//define sensor pin numbers
#define irFront   A8    //front IR analog pin
#define irRear    A9    //back IR analog pin
#define irRight   A10   //right IR analog pin
#define irLeft    A11   //left IR analog pin
#define snrLeft   A12   //front left sonar 
#define snrRight  A13   //front right sonar 
#define button    A15   //pushbutton 
#define liLeft     A1    //left light sensor
#define liRight    A0    //right light sensor

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                //create instance to control multiple steppers at the same time
NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

//define stepper motor constants
#define stepperEnTrue false     //variable for enabling stepper motor
#define stepperEnFalse true     //variable for disabling stepper motor
#define test_led 13             //test led to test interrupt heartbeat
#define enableLED 13            //stepper enabled LED
#define robot_spd 250           //set robot speed
#define max_accel 10000         //maximum robot acceleration
#define max_spd 500            //maximum robot speed
#define eighth_rotation 100    //stepper quarter rotation
#define quarter_rotation 200    //stepper quarter rotation
#define half_rotation 400       //stepper half rotation
#define one_rotation  800       //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600      //stepper motor 2 rotations
#define three_rotation 2400     //stepper rotation 3 rotations
#define four_rotation 3200      //stepper rotation 4 rotations
#define five_rotation 4000      //stepper rotation 5 rotations
#define six_rotation 4800      //stepper rotation 6 rotations
#define eight_rotation 6400      //stepper rotation 8 rotations

// -- LED Pins -- //
#define RED_LED 14
#define GREEN_LED 16
#define YELLOW_LED 15

// -- True and False defined -- //
#define TRUE 0
#define FALSE 1

//define sensor constants and variables
#define irMin    4      // IR minimum threshold for wall 4 inches
#define irMax    6      // IR maximum threshold for wall 6 inches
#define liMin    0.25   // light minimum threshold 12 inches

int irFrontArray[5] = {0, 0, 0, 0, 0};//array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;                       //variable to hold average of current front IR reading
int irLeftAvg;                        //variable to hold average of current left IR reading
int irRearAvg;                        //variable to hold average of current rear IR reading
int irRightAvg;                       //variable to hold average of current right IR reading
int irIdx = 0;                        //index for 5 IR readings to take the average
int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right sonar readings
int srIdx = 0;                        //index for 5 sonar readings to take the average
int srLeft;                           //variable to hold average of left sonar current reading
int srRight;                          //variable to hold average or right sonar current reading
int srLeftAvg;                        //variable to holde left sonar data
int srRightAvg;                       //variable to hold right sonar data

//STATE MACHINE TIMER INTERRUPT VARIABLES
volatile boolean test_state;          //variable to hold test led state for timer interrupt
#define timer_int 500000              //1/2 second (500000 us) period for timer interrupt

//bit definitions for sensor data flag byte [rt_snr left_snr left_ir right_ir rear_ir front_ir]
volatile byte flag = 0;
#define obFront   0   // Front IR trip [used to detect front wall for corner]
#define obRear    1   // Rear IR trip
#define obRight   2   // Right IR trip
#define obLeft    3   // Left IR trip
#define obFLeft   4   // Left Sonar trip
#define obFRight  5   // Right Sonar trip
#define lit       6   // Light trip

//bit definitions for robot motion and state byte [follow_hallway follow_right follow_left wander avoid]
volatile byte state = 0;
#define avoid     0   //avoid behavior
#define wander    1   //wander behavior
#define fleft     2   //follow left wall behavior
#define fright    3   //follow right wall behavior
#define center    4   //follow hallway behavior
#define movingL   6   //robot left wheel moving
#define movingR   7   //robot right wheel moving

//define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0      //avoid obstacle layer
#define wLayer 1      //wander layer
#define fwLayer 2     //follow wall layer
#define fhLayer 3     //follow hallway layer

//define light sensing behaviors
#define none 0  //no behavior
#define aggr 1  //aggresive behavior
#define love 2  //love behavior
#define expl 3  //explorer behavior
#define fear 4  //fear behavior
#define dock 5  //dock behavior
int lightType = dock;

#define MIN_LIGHT_THRESHOLD_LEFT 600
#define MAX_LIGHT_THRESHOLD_LEFT 850

#define MIN_LIGHT_THRESHOLD_RIGHT 750
#define MAX_LIGHT_THRESHOLD_RIGHT 950

long lightCounter = 0;

//define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
//store previous error to calculate derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
//store derror = difference between left and right error (used for hall follow to center the robot)

int ls_curr;      //left sonar current reading
int li_curr;      //left ir current reading
float lli_curr;   //left light current reading
int rs_curr;      //right sonar current reading
int ri_curr;      //right ir current reading
float rli_curr;   //left light current reading

int ls_cerror;    //left sonar current error
int li_cerror;    //left ir current error
float lli_cerror; //left light current error
int rs_cerror;    //right sonar current error
int ri_cerror;    //right ir current error
float rli_cerror; //right light current error

int ls_perror;    //left sonar previous error
int li_perror;    //left ir previous error
float lli_perror; //left light previous error
int rs_perror;    //right sonar previous error
int ri_perror;    //right ir previous error
float rli_perror; //right light previous error

int ls_derror;    //left sonar delta error
int li_derror;    //left ir delta error
float lli_derror; //left light delta error
int rs_derror;    //right sonar delta error
int ri_derror;    //right ir delta error
float rli_derror; //right light delta error
int left_derror;  //difference between left front and back sensor, this may be useful for adjusting the turn angle
int right_derror; //difference between right front and back sensor, this may be useful for adjusting the turn angle

int derror;       //difference between left and right error to center robot in the hallway

int rightState;   // detects if the right wall was ever found
int leftState;    // dtects if the left wall was ever found

int counter = 3;  // count how many times we try to find the wall. 3 means we are in random wander to start

#define baud_rate 9600  //set serial communication baud rate

/*
 * Initialization code
 */
void setup()
{
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);                 //sets pin as output
  pinMode(rtDirPin, OUTPUT);                  //sets pin as output
  pinMode(ltStepPin, OUTPUT);                 //sets pin as output
  pinMode(ltDirPin, OUTPUT);                  //sets pin as output
  pinMode(stepperEnable, OUTPUT);             //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  stepperRight.setMaxSpeed(max_spd);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);    //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);           //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);           //set right motor speed
  stepperLeft.setSpeed(robot_spd);            //set left motor speed
  steppers.addStepper(stepperRight);          //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);           //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue); //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);              //turn on enable LED

  // timer 1 setup
  Timer1.initialize(timer_int);               //initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);      //attaches updateSensors() as a timer overflow interrupt

  // serial setup
  Serial.begin(baud_rate);                    //start serial communication in order to debug the software while coding
  delay(1500);                                //wait 1.5 seconds before robot moves

  // LED SETUP
  // set as outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  // start in wander state
  bitSet(flag, wander);
}

/*
 * Main program to continuously call
 */
void loop()
{
  wallP();            //wall following proportional control
}

/*
  Description: 
    Follows walls using a proportional control. This means that it turns proportional to the error
    which is the distnace minus the 4-6 inch band it is trying to stay in. There is a gain value 
    that is adjusted by us to smooth the robot.

  Yellow,Red LED        - within 4-6 inches
  Yellow LED            - too close 2-4 inches
  Green,Yellow,Red LED  - follow center
  Red LED               - too far > 6 inches away
  Red,Green LED         - Front wall found
  Red,Yellow LED        - right wall follow
  Green,Yellow LED      - left wall follow 
  Green LED             - random wander
  No LED                - wall lost
  
  Input: nothing
  
  Return: nothing
*/
void wallP() {
//  if (bitRead(state,lit) && lightType != none && (!bitRead(flag, obFront) && !bitRead(flag, obLeft) && !bitRead(flag, obRight))) {
  if (bitRead(state,lit) && lightType != none) {
//  if (lightType != none) {
    if(lightType == fear) {
      fearAction();
    } else if(lightType == aggr) {
      aggrAction();
    } else if(lightType == love) {
      loveAction();
    } else if(lightType == expl){
      explAction();
    } else if(lightType == dock) {
      dockAction();
    }
  } else {
  // gain values are independent to turn right and left seperatley if needed
  float Pg_right = 0.75;
  float Pg_left = Pg_right;
  int r_turn = abs(eighth_rotation*ri_cerror*Pg_right);
  int l_turn = abs(eighth_rotation*li_cerror*Pg_left);

  // right wall found
  if (bitRead(state, fright)) {
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    rightState = TRUE;  // set the state that there is a right wall when turing corners
    leftState = FALSE;
    if (bitRead(flag, obFront)) { //check for a front wall before moving
      //make left turn if wall found
      delay(200);
      reverse(eighth_rotation);  //back up
      delay(200);
      spin(half_rotation, 0);    //turn left
      delay(200);
    }
    if (ri_cerror == 0) { // robot within 4-6 inches
      forward(quarter_rotation);        //move robot forward
    }
    else {
      if (ri_cerror > 0 && rs_curr < irMin && rs_curr > 1) {  // too close 4 ~ 2 inches aways
        pivot(r_turn, 0);      //pivot left
        delay(200);
        pivot(r_turn, 1);     //pivot right to straighten up
        delay(200);
      }
      else if (ri_cerror > 0 && rs_curr <= 1) { // way to close 0~2 inches
        pivot(one_rotation, 1);   // pivot right 
        delay(200);
        reverse(eighth_rotation); // backup at the pivot angle
        delay(200);
        pivot(-one_rotation, 1);  // straigten back up pivot left
        delay(200);
      }
      else if (ri_cerror < 0 && rs_curr > irMax) {     //positive error means too far > 6 inches
        pivot(r_turn, 1);      //pivot right
        delay(200);
        pivot(r_turn, 0);   //pivot left to straighten up
        delay(200);
      }
    }
    digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
  }
  // found left wall
  else if (bitRead(state, fleft)  ) {
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    rightState = FALSE; 
    leftState = TRUE;   // indicate that a left wall was found to tell corners
    if (bitRead(flag, obFront)) { //check for a front wall before moving forward
      //make left turn if wall found
      delay(200);
      reverse(eighth_rotation);    //back up
      delay(200);
      spin(half_rotation, 1);      //turn right
      delay(200);
    }
    if (li_cerror == 0) {   // robot within 4-6 inches
      forward(quarter_rotation);      //move robot forward
    }
    else {
      if (li_cerror > 0 && ls_curr < irMin && ls_curr > 1) { // too close within 2~4 inches
        pivot(l_turn, 1);      //pivot right
        delay(200);
        pivot(l_turn, 0);     //pivot left
        delay(200);
      }
      else if (li_cerror > 0 && ls_curr <= 1) {   // way too close within 0~2 inches
        pivot(one_rotation, 0);     // pivot left  
        delay(200);
        reverse(eighth_rotation);   // backup at the pivoted angle
        delay(200);
        pivot(-one_rotation, 0);    // straighten back up by pivoting right
        delay(200);
      }
      else if (li_cerror < 0 && ls_curr > irMax)  { // too far >6 inches
        pivot(l_turn, 0);      //pivot left
        delay(200);
        pivot(l_turn, 1);   //pivot right
        delay(200);
      }
    }
    digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
  }
//   follow hallway
  else if (bitRead(state, center) ) {
    digitalWrite(RED_LED, HIGH);  // turn on the red led for this function
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    digitalWrite(YELLOW_LED, HIGH);  // turn on the yellow led for this function
    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) { // centered in the hallway
      forward(half_rotation);          //drive robot forward
    }
    else {
      //try to average the error between the left and right to center the robot
      if (derror > 0) {
        spin(eighth_rotation, 1);        //spin right, the left error is larger
        pivot(quarter_rotation, 0);       //pivot left to adjust forward
      }
      else
      {
        spin(eighth_rotation, 0);        //spin left the right error is larger
        pivot(quarter_rotation, 1);       //pivot right to adjust forward
      }
    }
    digitalWrite(RED_LED, LOW);  // turn off the red led for this function
    digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
    digitalWrite(YELLOW_LED, LOW);  // turn off the yellow led for this function

    // front wall found need to turn and follow the wall
  } else if (bitRead(flag, obFront) && !bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    reverse(eighth_rotation);       //back up
    delay(200);
    spin(half_rotation, 1);         //turn right
    digitalWrite(GREEN_LED, LOW);   // turn off the green led for this function
  } else  if (bitRead(state, wander)) { // wander, no walls found
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    digitalWrite(YELLOW_LED, HIGH);  // turn on the yellow led for this function
    rightState = FALSE;
    leftState = FALSE;
    randomWander();
    digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
    digitalWrite(YELLOW_LED, LOW);  // turn off the yellow led for this function
  } else if(!bitRead(state, fright) && rightState == TRUE) {   // try and turn to find the wall, outside corner
    counter++;    // increase the number of attempts to find wall
    delay(200);
    spin(half_rotation+50,1);   // spin right but a little bit extra (50) to account for error in testing
    delay(200);
    forward(one_rotation);
    forward(half_rotation);
    delay(200);
  } else if(!bitRead(state, fleft) && leftState == TRUE) { // try and turn to find the wall, outside corner
    counter++;    // increase the number of attempts to find wall
    delay(200);
    spin(half_rotation+50,0); // spin left but a little bit extra (50) to account for error in testing
    delay(200);
    forward(one_rotation);
    forward(half_rotation);
    delay(200);
  }
  }
}

/*
  Description: 
    Calls the sensors update function to update error and states based on the errors.

  Input: nothing
  
  Return: nothing
*/
void updateSensors() {
  test_state = !test_state;             //LED to test the heartbeat of the timer interrupt routine
  digitalWrite(test_led, test_state);   //flash the timer interrupt LED
  flag = 0;                             //clear all sensor flags
  state = 0;                            //clear all state flags
  updateIR();                           //update IR readings and update flag variable and state machine
  updateLight();
  updateError();                        //update sensor current, previous, change in error
  updateState();                        //update State Machine based upon sensor readings
}

/*
  Description: 
    Update the sensor data from the IR. Polling front, left, right sensors.
    Modified from original to use inches instead of raw values

  Input: nothing
  
  Return: nothing
*/
void updateIR() {
  int front = 0, back = 0, left = 0, right = 0;         //declare IR variables

  int i = 0;

  // sum 5 readings of each sensor
  for(i = 0; i < 5; i++) {
    front += analogRead(irFront);          //read front IR sensor
    back += analogRead(irRear);            //read back IR sensor
    left += analogRead(irLeft);            //read left IR sensor
    right += analogRead(irRight);          //read right IR sensor
  }

  // divide to average the sum of the 5 readings
  front /= 5;
  back /= 5;
  left /= 5;
  right /= 5;

  // equations to convert to inches
  front = (1111/(front+16))-1;
  back = (1111/(back+20))-1;
  left = (285714/(left+2257))-103;
  right = (286714/(right+2600))-90;

  // filters out negative numbers
  if(front <= 0) {
    front = 1;
  }
  if(back <= 0) {
    back = 1;
  }
  if(left <= 0) {
    left = 1;
  }
  if(right <= 0) {
    right = 1;
  }

  // set the current values to the inches calculated
  ls_curr = left;
  rs_curr = right;

  if (right < irMax + 6) {  // wall right found within 12 inches and need to get within 4~6 inches
    bitSet(flag, obRight);            //set the right obstacle
  }
  else
    bitClear(flag, obRight);          //clear the right obstacle

  if (left < irMax + 6) {   // wall left found within 12 inches and need to get within 4~6 inches
    bitSet(flag, obLeft);             //set the left obstacle
  }
  else
    bitClear(flag, obLeft);           //clear the left obstacle

  if (front < irMin + 2) {  // front wall found within 6 inches
    bitSet(flag, obFront);            //set the front obstacle
  }
  else
    bitClear(flag, obFront);          //clear the front obstacle

  ///////////////////////update variables
  ri_curr = right;             //log current sensor reading [right IR]
  if ((ri_curr > irMax) | (ri_curr < irMin))
    ri_cerror = irMax - ri_curr;  //calculate current error (too far positive, too close negative)
  else
    ri_cerror = 0;                  //set error to zero if robot is in dead band
  ri_derror = ri_cerror - ri_perror; //calculate change in error
  ri_perror = ri_cerror;            //log current error as previous error [left sonar]

  li_curr = left;                   //log current sensor reading [left sonar]
  if ((li_curr > irMax) | (li_curr < irMin))
    li_cerror = irMax - li_curr;   //calculate current error
  else
    li_cerror = 0;                  //error is zero if in deadband
  li_derror = li_cerror - li_perror; //calculate change in error
  li_perror = li_cerror;                //log reading as previous error
}

/*
  Description: 
    Update the sensor data from the light sensor. Polling left and right sensors.
    Modified from original to use inches instead of raw values

  Input: nothing
  
  Return: nothing
*/
void updateLight() {
  float left = 0, right = 0;         //declare light variables

  int i = 0;

  // sum 5 readings of each sensor
  for(i = 0; i < 5; i++) {
    left += analogRead(liLeft);            //read left light sensor
    right += analogRead(liRight);          //read right light sensor
  }

  // divide to average the sum of the 5 readings
  left /= 5;
  right /= 5;

  // equations to convert to inches
  left = (left-MIN_LIGHT_THRESHOLD_LEFT)/(MAX_LIGHT_THRESHOLD_LEFT-MIN_LIGHT_THRESHOLD_LEFT);
  right = (right-MIN_LIGHT_THRESHOLD_RIGHT)/(MAX_LIGHT_THRESHOLD_RIGHT-MIN_LIGHT_THRESHOLD_RIGHT);
  if(left > 1) {
    left = 1;
  }
  if(right > 1) {
    right = 1;
  }
  if(left < 0) {
    left = 0;
  }
  if(right < 0) {
    right = 0;
  }

  // set the current values to the inches calculated
  lli_curr = left;
  rli_curr = right;

  if (right > liMin || left > liMin) {  // light found within 12 inches and need to get within 4~6 inches
    bitSet(flag, lit);            //set the light
  }
  else
    bitClear(flag, lit);          //clear the light

  ///////////////////////update variables
  rli_curr = right;             //log current sensor reading [right IR]
//  if ((rli_curr > liMax) | (rli_curr < liMin))
//    rli_cerror = liMax - rli_curr;  //calculate current error (too far positive, too close negative)
//  else
//    rli_cerror = 0;                  //set error to zero if robot is in dead band
//  rli_derror = rli_cerror - rli_perror; //calculate change in error
//  rli_perror = rli_cerror;            //log current error as previous error [left sonar]

  lli_curr = left;                   //log current sensor reading [left sonar]
//  if ((lli_curr > liMax) | (lli_curr < liMin))
//    lli_cerror = liMax - lli_curr;   //calculate current error
//  else
//    li_cerror = 0;                  //error is zero if in deadband
//  lli_derror = lli_cerror - lli_perror; //calculate change in error
//  lli_perror = lli_cerror;                //log reading as previous error
}

/*
   This function will update all of the error constants to be used for P and PD control
   store previous error to calculate derror = curr_sensor-prev_sensor, side_derror = side front sensor - side back sensor
*/
void updateError() {
  left_derror = ls_cerror - li_cerror; //difference between left front and back sensor, use threshold for robot mostly parallel to wall
  right_derror = rs_cerror - ri_cerror; //difference between right front and back sensor, use threshold for robot mostly parallel to wall
  //derror = ls_cerror - rs_cerror;//use sonar data for difference error
  derror = li_cerror - ri_cerror; //use IR data for difference error
  //  Serial.print("left derror\t"); Serial.print(left_derror);
  //  Serial.print("\tright derror\t"); Serial.println(right_derror);
}

/*
  Description: 
    Given to us and used to see which state we need to bein

  Input: nothing
  
  Return: nothing
*/
void updateState() {
  if (!(flag) && counter >= 3) { //no sensors triggered
    //set random wander bit
    bitSet(state, wander);//set the wander state
    //clear all other bits
    bitClear(state, fright);//clear follow wall state
    bitClear(state, fleft);//clear follow wall state
    bitClear(state, center);//clear follow wall state
    bitClear(state, lit);//clear light state
  }
  else if (bitRead(flag, lit)) {
    bitSet(state, lit); //set light state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
    bitClear(state, fright);  //clear follow wall state
  }
  else if (bitRead(flag, obRight) && !bitRead(flag, obLeft) ) {
    counter = 0;
    bitSet(state, fright);    //set RIGHT WALL state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
    bitClear(state, lit);//clear light state
  }
  else if (bitRead(flag, obLeft) && !bitRead(flag, obRight) ) {
    counter = 0;
    bitSet(state, fleft);     //set left wall state
    //clear all other bits
    bitClear(state, fright);  //clear follow wall state
    bitClear(state, wander);  //clear wander state
    bitClear(state, center);  //clear follow wall state
    bitClear(state, lit);//clear light state
  }
  else if (bitRead(flag, obLeft) && bitRead(flag, obRight) ) {
    counter = 0;
    bitSet(state, center);      //set the hallway state
    //clear all other bits
    bitClear(state, fright);    //clear follow wall state
    bitClear(state, wander);    //clear wander state
    bitClear(state, fleft);     //clear follow wall state
    bitClear(state, lit);//clear light state
  }
}

/*
  Description: 
    Makes the robot flee from light by speeding up the motor closest to it

  Input: nothing
  
  Return: nothing
*/
void fearAction() {
  digitalWrite(YELLOW_LED, HIGH);   // turn on the yellow led for this function
  digitalWrite(RED_LED, HIGH);      // turn on the red led for this function
  digitalWrite(GREEN_LED, HIGH);    // turn on the green led for this function

  int rightWSpeed; // initializes the right wheel's speed
  int leftWSpeed;  // initializes the left wheel's speed
    
  if(rli_curr > lli_curr) {
    rightWSpeed = 200 + rli_curr * 1000;   // set the right wheel's speed
    leftWSpeed = 200;                     // set the left wheel's speed
  } else {
    rightWSpeed = 200;                    // set the right wheel's speed
    leftWSpeed = 200 + lli_curr * 1000;    // set the left wheel's speed
  }
  
  // puts above values into motor functions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(eighth_rotation);//move distance
  stepperLeft.moveTo(eighth_rotation);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  digitalWrite(YELLOW_LED, LOW);   // turn off the yellow led for this function
  digitalWrite(RED_LED, LOW);      // turn off the red led for this function
  digitalWrite(GREEN_LED, LOW);    // turn off the green led for this function
}

/*
  Description: 
    Makes the robot move towards the light by slowing down the motor closest to it

  Input: nothing
  
  Return: nothing
*/
void loveAction() {
  digitalWrite(YELLOW_LED, HIGH);   // turn on the yellow led for this function
  digitalWrite(RED_LED, HIGH);      // turn on the red led for this function

  int rightWSpeed; // initializes the right wheel's speed
  int leftWSpeed;  // initializes the left wheel's speed

  if(rli_curr > lli_curr) {
    rightWSpeed = 800 - rli_curr * 800;   // set the right wheel's speed
    leftWSpeed = 800;                     // set the left wheel's speed
  } else {
    rightWSpeed = 800;                    // set the right wheel's speed
    leftWSpeed = 800 - lli_curr * 800;    // set the left wheel's speed
  }

//  Serial.print("Right: ");
//  Serial.print(rli_curr);
//  Serial.print("Left: ");
//  Serial.println(lli_curr);
  
  // puts above values into motor functions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(eighth_rotation);//move distance
  stepperLeft.moveTo(eighth_rotation);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  digitalWrite(YELLOW_LED, LOW);   // turn off the yellow led for this function
  digitalWrite(RED_LED, LOW);      // turn off the red led for this function
}

/*
  Description: 
    Makes the robot move towards the light by speeding up the motor farthest to it

  Input: nothing
  
  Return: nothing
*/
void aggrAction() {
  digitalWrite(GREEN_LED, HIGH);    // turn on the green led for this function
  digitalWrite(YELLOW_LED, HIGH);   // turn on the yellow led for this function

  int rightWSpeed; // initializes the right wheel's speed
  int leftWSpeed;  // initializes the left wheel's speed
  
  if(lli_curr > rli_curr) {
    rightWSpeed = 200 + lli_curr * 1000;  // set the right wheel's speed
    leftWSpeed = 200;                     // set the left wheel's speed
  } else {
    rightWSpeed = 200;                    // set the right wheel's speed
    leftWSpeed = 200 + rli_curr * 1000;   // set the left wheel's speed
  }
  
  // puts above values into motor functions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(eighth_rotation);//move distance
  stepperLeft.moveTo(eighth_rotation);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  digitalWrite(GREEN_LED, LOW);     // turn off the green led for this function
  digitalWrite(YELLOW_LED, LOW);    // turn off the yellow led for this function
}

/*
  Description: 
    Makes the robot move away from the light by slowing down the motor farthest from it

  Input: nothing
  
  Return: nothing
*/
void explAction() {
  digitalWrite(GREEN_LED, HIGH);    // turn on the green led for this function
  digitalWrite(RED_LED, HIGH);      // turn on the red led for this function

  int rightWSpeed; // initializes the right wheel's speed
  int leftWSpeed;  // initializes the left wheel's speed

   if(lli_curr > rli_curr) {
    rightWSpeed = 800 - lli_curr * 800;   // set the right wheel's speed
    leftWSpeed = 800;                     // set the left wheel's speed
  } else {
    rightWSpeed = 800;                    // set the right wheel's speed
    leftWSpeed = 800 - rli_curr * 800;    // set the left wheel's speed
  }
  
  // puts above values into motor functions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(eighth_rotation);//move distance
  stepperLeft.moveTo(eighth_rotation);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  digitalWrite(GREEN_LED, LOW);    // turn off the green led for this function
  digitalWrite(RED_LED, LOW);      // turn off the red led for this function
}

/*
  Description: 
    Makes the robot move to a light and dock at it before moving back

  Input: nothing
  
  Return: nothing
*/
void dockAction() {
  digitalWrite(YELLOW_LED, HIGH);    // turn on the yellow led for this function

  int rightWSpeed; // initializes the right wheel's speed
  int leftWSpeed;  // initializes the left wheel's speed
  int i;

   if(rli_curr > lli_curr + 0.05) {
    leftWSpeed = 100 + rli_curr * 800;   // set the right wheel's speed
    rightWSpeed = 100;                     // set the left wheel's speed
  } else if (rli_curr < lli_curr - 0.05) {
    leftWSpeed = 100;                    // set the right wheel's speed
    rightWSpeed = 100 + rli_curr * 800;    // set the left wheel's speed
  } else if(rli_curr > 0.99 || lli_curr > 0.99){
    rightWSpeed = 600;                    // set the right wheel's speed
    leftWSpeed = 600;                     // set the left wheel's speed
    delay(1000);
    spin(one_rotation, 1);
    for(i = 0; i < lightCounter; i++) {
//      forward(eight_rotation);
      stepperRight.setCurrentPosition(0);
      stepperLeft.setCurrentPosition(0);
      stepperRight.moveTo(eighth_rotation);//move distance
      stepperLeft.moveTo(eighth_rotation);//move distance
      stepperRight.setSpeed(rightWSpeed);//set speed
      stepperLeft.setSpeed(leftWSpeed);//set speed
      stepperRight.runSpeedToPosition();//move right motor
      stepperLeft.runSpeedToPosition();//move left motor
    }
    lightCounter = 0;
    spin(one_rotation, 1);
  } else {
    rightWSpeed = 600;                    // set the right wheel's speed
    leftWSpeed = 600;                     // set the left wheel's speed
  }
  
  // puts above values into motor functions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(eighth_rotation);//move distance
  stepperLeft.moveTo(eighth_rotation);//move distance
  stepperRight.setSpeed(rightWSpeed);//set speed
  stepperLeft.setSpeed(leftWSpeed);//set speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  lightCounter++;
  
  digitalWrite(YELLOW_LED, LOW);    // turn off the yellow led for this function
}

/*
  Description: 
    Given to us and is used to move forward a given amount in terms of rotations that
    are predefined.

  Input:
    rot - amount of ticks for the motor. Predefined values above
  
  Return: nothing
*/
void forward(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*
  Description: 
    Given to us and is used to move backward a given amount in terms of rotations that
    are predefined.

  Input:
    rot - amount of ticks for the motor. Predefined values above
  
  Return: nothing
*/
void reverse(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() - rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() - rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*
  Description: 
    Given to us and is used to pivot a given amount in terms of rotations that
    are predefined.

  Input:
    rot - amount of ticks for the motor. Predefined values above
    dir - 0 is left, 1 or more is right
  
  Return: nothing
*/
void pivot(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//pivot right
    positions[0] = stepperRight.currentPosition();    //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot ; //left motor absolute position
  }
  else//pivot left
  {
    positions[0] = stepperRight.currentPosition() + rot ; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() ;     //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*
  Description: 
    Given to us and is used to spin a given amount in terms of rotations that
    are predefined.

  Input:
    rot - amount of ticks for the motor. Predefined values above
    dir - 0 is left, 1 or more is right
  
  Return: nothing
*/
void spin(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//spin right
    positions[0] = stepperRight.currentPosition() - rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  }
  else//spin left
  {
    positions[0] = stepperRight.currentPosition() + rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() - rot;  //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*
  Description: 
    moves random distances at random speeds

  Input: nothing
  
  Return: nothing
*/
void randomWander() {
   digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function

  // randomly decides the signs of the speed and distance values
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

  // randomly sets distances and speeds
  long rightDistance = rightDSign * random(400,1200);
  long leftDistance = leftDSign * random(400,1200);

  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + rightDistance;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + leftDistance;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target

  digitalWrite(GREEN_LED, LOW);  // turn off the green led
}

/*
  Description: 
    Stops both motors from spining

  Input: nothing
  
  Return: nothing
*/
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  Description: 
    spins the motors until the robot doesn't have any more ticks. It was modified to 
    stop the timer from interrupting to allow it to actually move as far as we tell it.

  Input: nothing
  
  Return: nothing
*/
void runToStop ( void ) {
  int runNow = 1;
  Timer1.stop();  // added in to stop interrupts to allow full movement

//  while (runNow && !bitRead(state, lit)) {
  while(runNow) {
//    updateLight();
    if (!stepperRight.run()) {
      bitClear(state, movingR);  // clear bit for right motor moving
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      bitClear(state, movingL);   // clear bit for left motor moving
      stepperLeft.stop();//stop left motor
    }//
    if (!bitRead(state, movingR) & !bitRead(state, movingL))
      runNow = 0;
  }
  Timer1.start(); // restart the timer to update sensors again
}
