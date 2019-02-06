/*Moravec-WallFollowing.ino
  Author: Devon Adair and Hunter LaMantia
  Date: 1/16/19



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
*/

// -- INCLUDES --//
#include <printf.h>
#include <RF24_config.h>
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>//include timer interrupt library

//sets up wireless transceiver
#include <SPI.h> //include serial peripheral interface library
#include <RF24.h> //include wireless transceiver library
#include <nRF24L01.h> //include wireless transceiver library
#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);
#define test_LED 16
#define team_channel 69   //transmitter and receiver on same channel between 1 & 125

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
#define max_spd 500             //maximum robot speed
#define eighth_rotation 100     //stepper quarter rotation
#define quarter_rotation 200    //stepper quarter rotation
#define half_rotation 400       //stepper half rotation
#define one_rotation  800       //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600      //stepper motor 2 rotations
#define three_rotation 2400     //stepper rotation 3 rotations
#define four_rotation 3200      //stepper rotation 4 rotations
#define five_rotation 4000      //stepper rotation 5 rotations
#define six_rotation 4800       //stepper rotation 6 rotations
#define eight_rotation 6400     //stepper rotation 8 rotations

// -- LED Pins -- //
#define RED_LED 14
#define GREEN_LED 16
#define YELLOW_LED 15

// -- True and False defined -- //
#define TRUE 0
#define FALSE 1

//define sensor constants and variables
#define irMin    4   // IR minimum threshold for wall 4 inches
#define irMax    6   // IR maximum threshold for wall 6 inches
#define topoMax  12  // IR maximum threshold for topological navvigation

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

//define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
//store previous error to calculate derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
//store derror = difference between left and right error (used for hall follow to center the robot)

int ls_curr;      //left sonar current reading
int li_curr;      //left ir current reading
int rs_curr;      //right sonar current reading
int ri_curr;      //right ir current reading

int ls_cerror;    //left sonar current error
int li_cerror;    //left ir current error
int rs_cerror;    //right sonar current error
int ri_cerror;    //right ir current error

int ls_perror;    //left sonar previous error
int li_perror;    //left ir previous error
int rs_perror;    //right sonar previous error
int ri_perror;    //right ir previous error

int ls_derror;    //left sonar delta error
int li_derror;    //left ir delta error
int rs_derror;    //right sonar delta error
int ri_derror;    //right ir current error
int left_derror;  //difference between left front and back sensor, this may be useful for adjusting the turn angle
int right_derror; //difference between right front and back sensor, this may be useful for adjusting the turn angle

int derror;       //difference between left and right error to center robot in the hallway

int rightState;   // detects if the right wall was ever found
int leftState;    // dtects if the left wall was ever found

int counter = 3;  // count how many times we try to find the wall. 3 means we are in random wander to start

int topo_check = 1; // counts current state if topological tracking is active

#define baud_rate 9600  //set serial communication baud rate

// O Map defines
#define OBSTACLE  99
#define GOAL      50
#define EMPTY     0

// T Map defines
#define N     B0001 
#define E     B0010
#define NE    B0011
#define S     B0100
#define NS    B0101
#define SE    B0110
#define NSE   B0111
#define W     B1000
#define NW    B1001
#define WE    B1010
#define NWE   B1011
#define SW    B1100
#define SNW   B1101
#define SWE   B1110
#define NSWE  B1111
byte tile;

// maps for the little robot
//volatile byte Tmap[4][4] = {{NW, SNW, N, NSWE},
//                            {W, NS, S, NE},
//                            {WE, SNW, NS, E},
//                            {SWE, SNW, NS, SE}};
//volatile byte Tmap[4][4] = {{NW, NSWE, NSWE, NE},
//                            {W, NS, NS, E},
//                            {WE, NSWE, NSWE, WE},
//                            {SWE, SNW, NS, SE}};

//volatile byte Tmap[4][4] = {{NWE, NSWE, NSWE, NWE},   // occupancy grid
//                            {W, NS, NS, E},
//                            {WE, NSWE, NSWE, WE},
//                            {SWE, NSWE, NSWE, SWE}};
volatile byte Tmap[4][4] = {{NWE, NWE, NWE, NWE},       // topological grid
                            {WE, W, E, WE},
                            {W, E, W, E},
                            {SWE, NSWE, NSWE, SWE}};
                                                        
volatile byte Omap[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0}};

// Character array for giving an input to the top function when
// we calculate the path
char directions[10];

#define NORTH 0
#define SOUTH 1
#define WEST  2
#define EAST  3
int startingDirection = NORTH;

// Set up constants for movement
#define FORWARD 1
#define BACKWARD -1
#define LEFT 0
#define RIGHT 1

const uint64_t pipes[2] = {0xE8E8F0F0E1LL, 0xE8E8F0F0A1LL}; //define the radio transmit pipe (5 Byte configurable)
//RF24 radio(CE_PIN, CSN_PIN);          //create radio object
uint8_t data[1];                      //variable to hold transmit data
uint8_t sendData[1];

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

  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);//open up reading pipe
  radio.startListening();//start listening for data;

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

//  CalcWavefront(3,0,2,1);
//  CalcWavefront(0,0,3,1);
//  CalcWavefront(0,1,2,1);
//  CalcWavefront(1,3,0,1);
  CalcWavefront(0,0,0,0);
  
  // start in wander state
  bitSet(flag, wander);

}

/*
 * Main program to continuously call
 */
void loop()
{
//  topo(directions);
//  topo("SLLLLT");
  delay(5);
  radio.startListening();
  while (radio.available()) {
//    Serial.println("test");
    radio.read(&data, sizeof(data));
//    Serial.println(data[0]);
    if(data[0] == 8) {
      Serial.println("forward");
      forward2(FORWARD);
    } else if(data[0] == 2) {
      Serial.println("backward");
      forward2(BACKWARD);
    } else if(data[0] == 4) {
      Serial.println("left");
      spin2(LEFT);
    } else if(data[0] == 6) {
      Serial.println("right");
      spin2(RIGHT);
    } else if(data[0] == 5) {
      localize(tile);
    }
  }
  delay(5);
  radio.stopListening();
  sendData[0] = 38;
  radio.write(sendData, sizeof(sendData));
}


///////////////////////////////////////////////////////////
// TOPOLOGICAL
///////////////////////////////////////////////////////////
/*
  Description: 
    Follows a set of instructions to follow paths. This means that it makes a specific
    sequence of turns at intersections and terminates.
  
  Input: nothing
  
  Return: nothing
*/

void topo(char *instr) {
  char topo_current = instr[topo_check]; // tracks current instruction
//  Serial.print(instr[0]);
//  Serial.print(instr[1]);
//  Serial.print(instr[2]);
//  Serial.print(instr[3]);
//  Serial.println(instr[4]):

  if (bitRead(state, center)) { // initiates wall following
    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) { // centered in the hallway
      forward(half_rotation);          //drive robot forward
    } else {
      //try to average the error between the left and right to center the robot
      if (derror > 0) {
        spin(eighth_rotation, 1);        //spin right, the left error is larger
        pivot(quarter_rotation, 0);       //pivot left to adjust forward
      } else {
        spin(eighth_rotation, 0);        //spin left the right error is larger
        pivot(quarter_rotation, 1);       //pivot right to adjust forward
      }
    }
  }
  // turns around corners if instructions call for it
  if(ri_curr > 12 || li_curr > 12) {
    if(topo_current == 'R') {
      spin(quarter_rotation+50,1);
      forward(half_rotation);
      spin(quarter_rotation+15,1);
      forward(one_rotation);
      forward(half_rotation);
      topo_check++;
      topo_current = instr[topo_check];
    } else if(topo_current == 'L') {
      spin(quarter_rotation+50,0);
      forward(half_rotation);
      spin(quarter_rotation+50,0);
      forward(one_rotation);
      forward(half_rotation);
      topo_check++;
      topo_current = instr[topo_check];
    } else if(topo_current == 'F') {
      forward(one_rotation);
      forward(half_rotation);
      topo_check++;
      topo_current = instr[topo_check];
    }
    // terminates program at after doing all turns
    if(topo_current == 'T' && bitRead(flag,obFront)) {
//      forward(one_rotation);
//      forward(half_rotation);
      exit(0);
    }  
  }
}


///////////////////////////////////////////////////////////
// PATH PLANNING
///////////////////////////////////////////////////////////
/*
  Description: 
    This takes the Tmap that is already populated and expands the omap to 9x9.
    When the omap is populated and does wavefront propogation on it. This 
    creates a path to the goal from any square on the map. Once that is calculated
    it plans the path and sets the directions[] to E, W, N, S directions.

    The directions[] is filled with:
    S - for start
    T - for Terminate
    F - for Forward for the T junctions
    L - for Left
    R - for Right

    All the directions are relative to the robot and not the grid. This is why 
    StartingDirection variable is important to know.
  
  Input:
    StartRow - starting row for the robot
    StartCol - starting column for the robot
    GoalRow - the row that the goal is in
    GoalCol - the column that the goal is in

    The coordinates should be given in terms of a 4x4 0 indexed.
  
  Return: nothing
*/
void CalcWavefront(int StartRow, int StartCol, int GoalRow, int GoalCol) {
  int Trow,Tcol,Orow,Ocol;          // rows and columns of the maps
  int i, j, currentVal, smallestIndex;  // indexing
  int directionIndex = 1;           // mark where we are in the direction array skipping the S
  bool notDone = true;              // check for doing the wavefront prop
  bool found = false;               // used to find check if we found empty boxes to fill in the wavefront algorithm
  bool foundGoal = false;           // check for finding the goal in the path finding
  int previousStep;                 // what was the last move in the path finding
  byte options[4];                  // store the direction values of left,right,up,down for path finding.

  makeOmapFromTmap();

  // make the goal in terms of 9x9 for omap
  GoalRow = (GoalRow * 2) + 1;
  GoalCol = (GoalCol * 2) + 1;
  StartRow = (StartRow * 2) + 1;
  StartCol = (StartCol * 2) + 1;

  // set the inital goal spot and values around the goal to
  // start the algorithm
  Omap[GoalRow][GoalCol] = GOAL;
  if(Omap[GoalRow+1][GoalCol] != OBSTACLE) {
    Omap[GoalRow+1][GoalCol] = 1;
  }
  if(Omap[GoalRow-1][GoalCol] != OBSTACLE) {
    Omap[GoalRow-1][GoalCol] = 1;
  }
  if(Omap[GoalRow][GoalCol+1] != OBSTACLE) {
    Omap[GoalRow][GoalCol+1] = 1;
  }
  if(Omap[GoalRow][GoalCol-1] != OBSTACLE) {
    Omap[GoalRow][GoalCol-1] = 1;
  }
  
  Orow = 0;
  Ocol = 0;

  printArray(); // print the array to check initial omap values
  
  // Do the wave...
  while(notDone) {
    if(Omap[Orow][Ocol] != 0 && Omap[Orow][Ocol] != OBSTACLE) { // check if there is a number
      currentVal = Omap[Orow][Ocol]+1;
      if((Orow + 1) <= 9) {
        if(Omap[Orow+1][Ocol] == 0 && Omap[Orow+1][Ocol] != GOAL) {  // above the box
          Omap[Orow+1][Ocol] = currentVal;
        }
      }
      if((Orow - 1) >= 0) {
        if(Omap[Orow-1][Ocol] == 0 && Omap[Orow-1][Ocol] != GOAL) {  // below the box
          Omap[Orow-1][Ocol] = currentVal;
        }
      }
      if((Ocol + 1) <= 9) {
        if(Omap[Orow][Ocol+1] == 0 && Omap[Orow][Ocol+1] != GOAL) {  // right to the box
          Omap[Orow][Ocol+1] = currentVal;
        }
      }
      if((Ocol - 1) >= 0) {
        if(Omap[Orow][Ocol-1] == 0 && Omap[Orow][Ocol-1] != GOAL) {  // left to the box
          Omap[Orow][Ocol-1] = currentVal;
        }
      }
    }

    // handle row and column incrementations
    if(Ocol == 8) {
      Ocol = 0;
      if (Orow == 8) {
        Orow = 0;
        // check if we have filled in every square
        for(i = 0 ; i < 9; i ++){
          for(j = 0; j < 9; j++) {
            if(Omap[i][j] == 0 && i != GoalRow && j != GoalCol){  // still have more to fill out
              found = true;
            }
          }
        }
        // if we found a box not filled clear the flag.
        if(found == true) {
          found = false;
        } else {  // no more boxes to fill
          notDone = false;
        }
      } else {
        Orow += 1;
      }
    } else {
      Ocol += 1;
    }
  }

  Ocol = StartCol;
  Orow = StartRow;

  Omap[GoalRow][GoalCol] = EMPTY; // clear the goal that was set to 50 for the algorithm

  printArray(); // final Omap that has the values of the paths
  
  directions[0] = 'S';
  previousStep = 0;
  // Ride the wave... (Path finding...)
  while(!foundGoal) {
    if(Omap[Orow][Ocol] == EMPTY) {
      foundGoal = true;
      break;
    }
    // check the boxes to the left, right, top, and bottom
    options[0] = Omap[Orow+1][Ocol];
    options[1] = Omap[Orow-1][Ocol];
    options[2] = Omap[Orow][Ocol+1];
    options[3] = Omap[Orow][Ocol-1];

    // find the smallest path
    smallestIndex = 0;
    for(i = 0; i < 4; i++) {
      if(options[i] <= options[smallestIndex]){
        smallestIndex = i;
      }
    }
    
//    Serial.print(Omap[Orow][Ocol]);
//    Serial.print(", ");
//    Serial.println(smallestIndex);

    // pick the direction that is the smallest
    switch(smallestIndex) {
      case 0: // South
        if(previousStep == 2) { // just turned right
          directions[directionIndex] = 'R';
          directionIndex++;
        }else if(previousStep == 3) {   // just turned left
          directions[directionIndex] = 'L';
          directionIndex++;
        }else if (options[3] != 99 || options[2] != 99) {   // check T junctions
          directions[directionIndex] = 'F';
          directionIndex++;
        }
        startingDirection = SOUTH;
        Orow += 1;
        break;
      case 1: // North
        if(previousStep == 2) {
          directions[directionIndex] = 'L'; 
          directionIndex++;
        }else if(previousStep == 3) { // just turned right
          directions[directionIndex] = 'R';
          directionIndex++;
        }else if (options[3] != 99 || options[2] != 99) { // check T junctions
          directions[directionIndex] = 'F';
          directionIndex++;
        }
        startingDirection = NORTH;
        Orow -= 1;
        break;
      case 2: // EAST
        if(previousStep != smallestIndex && startingDirection != EAST) {
          if(startingDirection == SOUTH) {
            directions[directionIndex] = 'L';
          } else {
            directions[directionIndex] = 'R';
          }
          directionIndex += 1;
        }else if (options[0] != 99 || options[1] != 99 ) { // check T junctions
          directions[directionIndex] = 'F';
          directionIndex++;
        }
        startingDirection = EAST;
        Ocol += 1;
        break;
      case 3: // WEST
        if(previousStep != smallestIndex && startingDirection != WEST) {
          if(startingDirection == SOUTH) {
            directions[directionIndex] = 'R';
          } else {
            directions[directionIndex] = 'L';
          }
          directionIndex += 1;
        }else if (options[0] != 99 || options[1] != 99) { // check T junctions
          directions[directionIndex] = 'F';
          directionIndex++;
        }
        startingDirection = WEST;
        Ocol -= 1;
        break;
       default:
        break;
    }
    previousStep = smallestIndex;
  }
  
  directions[directionIndex] = 'T';

  Serial.print("Total Steps: ");
  Serial.println(directionIndex);
  Serial.print(directions[0]);
  Serial.print(directions[1]);
  Serial.print(directions[2]);
  Serial.print(directions[3]);
  Serial.print(directions[4]);
  Serial.print(directions[5]);
  Serial.print(directions[6]);
  Serial.print(directions[7]);
  Serial.print(directions[8]);
  Serial.print(directions[9]);
}

void makeOmapFromTmap() {
  int Trow, Tcol, Orow, Ocol;
  
  // Make the o map based on the t map to add 99's
  for(Trow = 0; Trow < 4; Trow++) {  // rows
    for(Tcol = 0; Tcol < 4; Tcol++) {  // columns
      Orow = (Trow * 2) + 1;
      Ocol = (Tcol * 2) + 1;
      // NORTH
      if((Tmap[Trow][Tcol] & B0001) == B0001) { // NORTH
        Omap[Orow-1][Ocol] = OBSTACLE;
        Omap[Orow-1][Ocol+1] = OBSTACLE;
        Omap[Orow-1][Ocol-1] = OBSTACLE;
      }    
      if((Tmap[Trow][Tcol] & B0010) == B0010) {  // EAST
        Omap[Orow][Ocol+1] = OBSTACLE;
        Omap[Orow+1][Ocol+1] = OBSTACLE;
        Omap[Orow-1][Ocol+1] = OBSTACLE;
      }
      if((Tmap[Trow][Tcol] & B0100) == B0100) { // SOUTH
        Omap[Orow+1][Ocol] = OBSTACLE;
        Omap[Orow+1][Ocol+1] = OBSTACLE;
        Omap[Orow+1][Ocol-1] = OBSTACLE;
      }
      if((Tmap[Trow][Tcol] & B1000) == B1000) {  // WEST
        Omap[Orow][Ocol-1] = OBSTACLE;
        Omap[Orow+1][Ocol-1] = OBSTACLE;
        Omap[Orow-1][Ocol-1] = OBSTACLE;
      }
      if((Tmap[Trow][Tcol] & B1111) == B1111) {  // ALL
        Omap[Orow][Ocol] = OBSTACLE;
      }
    }
  } 
}

//Debugging
void printArray() {
  int col, row;
  Serial.println("=============================================");
  for(row = 0; row < 9; row++){
    Serial.print("[");
    for(col = 0; col < 9; col++) {
      if(Omap[row][col] < 10) {
        Serial.print("0");
        Serial.print(Omap[row][col]);
      } else {
        Serial.print(Omap[row][col]);
      }
      Serial.print(", ");
    }
    Serial.println("]");  
  }
  Serial.println("=============================================");
}
/*
  Description: 
    
  
  Input: nothing
  
  Return: nothing
*/
void FollowPath() {

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
  // gain values are independent to turn right and left seperatley if needed
  float Pg_right = 0.75;
  float Pg_left = Pg_right;
  int r_turn = abs(eighth_rotation*ri_cerror*Pg_right);
  int l_turn = abs(eighth_rotation*li_cerror*Pg_left);

  // right wall found
  if (bitRead(state, fright)) {
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
      digitalWrite(YELLOW_LED, HIGH);   // turn on the yellow led for this function
      digitalWrite(RED_LED, HIGH);      // turn on the red led for this function
      forward(quarter_rotation);        //move robot forward
      digitalWrite(YELLOW_LED, LOW);    // turn off the yellow led for this function
      digitalWrite(RED_LED, LOW);       // turn off the red led for this function
    }
    else {
      if (ri_cerror > 0 && rs_curr < irMin && rs_curr > 1) {  // too close 4 ~ 2 inches aways
        digitalWrite(YELLOW_LED, HIGH);
        pivot(r_turn, 0);      //pivot left
        delay(200);
        pivot(r_turn, 1);     //pivot right to straighten up
        delay(200);
        digitalWrite(YELLOW_LED, LOW);
      }
      else if (ri_cerror > 0 && rs_curr <= 1) { // way to close 0~2 inches
        digitalWrite(YELLOW_LED, HIGH);
        pivot(one_rotation, 1);   // pivot right 
        delay(200);
        reverse(eighth_rotation); // backup at the pivot angle
        delay(200);
        pivot(-one_rotation, 1);  // straigten back up pivot left
        delay(200);
        digitalWrite(YELLOW_LED, LOW);
      }
      else if (ri_cerror < 0 && rs_curr > irMax) {     //positive error means too far > 6 inches
        digitalWrite(RED_LED, HIGH);
        pivot(r_turn, 1);      //pivot right
        delay(200);
        pivot(r_turn, 0);   //pivot left to straighten up
        delay(200);
        digitalWrite(RED_LED, LOW);
      }
    }
  }
  // found left wall
  else if (bitRead(state, fleft)  ) {
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
      digitalWrite(YELLOW_LED, HIGH);  // turn on the yellow led for this function
      digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
      forward(quarter_rotation);      //move robot forward
      digitalWrite(YELLOW_LED, LOW);  // turn off the yellow led for this function
      digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
    }
    else {
      if (li_cerror > 0 && ls_curr < irMin && ls_curr > 1) { // too close within 2~4 inches
        digitalWrite(YELLOW_LED, HIGH);  // turn on the yellow led for this function
        pivot(l_turn, 1);      //pivot right
        delay(200);
        pivot(l_turn, 0);     //pivot left
        delay(200);
        digitalWrite(YELLOW_LED, LOW);  // turn on the yellow led for this function
      }
      else if (li_cerror > 0 && ls_curr <= 1) {   // way too close within 0~2 inches
        digitalWrite(YELLOW_LED, HIGH);
        pivot(one_rotation, 0);     // pivot left  
        delay(200);
        reverse(eighth_rotation);   // backup at the pivoted angle
        delay(200);
        pivot(-one_rotation, 0);    // straighten back up by pivoting right
        delay(200);
        digitalWrite(YELLOW_LED, LOW);
      }
      else if (li_cerror < 0 && ls_curr > irMax)  { // too far >6 inches
        digitalWrite(RED_LED, HIGH);  // turn on the red led for this function
        pivot(l_turn, 0);      //pivot left
        delay(200);
        pivot(l_turn, 1);   //pivot right
        delay(200);
        digitalWrite(RED_LED, LOW);  // turn off the red led for this function
      }
    }
  }
  // follow hallway
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
    digitalWrite(RED_LED, HIGH);    // turn on the red led for this function
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    reverse(eighth_rotation);       //back up
    delay(200);
    spin(half_rotation, 1);         //turn right
    digitalWrite(RED_LED, LOW);     // turn off the red led for this function
    digitalWrite(GREEN_LED, LOW);   // turn off the green led for this function
  } else  if (bitRead(state, wander)) { // wander, no walls found
    digitalWrite(GREEN_LED, HIGH);  // turn on the green led for this function
    rightState = FALSE;
    leftState = FALSE;
    randomWander();
    digitalWrite(GREEN_LED, LOW);  // turn off the green led for this function
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

  if (front < irMin + 6) {  // front wall found within 6 inches
    bitSet(flag, obFront);            //set the front obstacle
  }
  else
    bitClear(flag, obFront);          //clear the front obstacle

  if (back < irMin + 6) {  // back wall found within 6 inches
    bitSet(flag, obRear);            //set the rear obstacle
  }
  else
    bitClear(flag, obRear);          //clear the rear obstacle


  if(bitRead(flag, obFront) && !bitRead(flag, obRead) && !bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = N;
  } else if(!bitRead(flag, obFront) && !bitRead(flag, obRear) && !bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = E;
  } else if(bitRead(flag, obFront) && !bitRead(flag, obRear) && !bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = NE;
  } else if(!bitRead(flag, obFront) && bitRead(flag, obRear) && !bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = S;
  } else if(bitRead(flag, obFront) && bitRead(flag, obRear) && !bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = NS;
  } else if(!bitRead(flag, obFront) && bitRead(flag, obRear) && !bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = SE;
  } else if(bitRead(flag, obFront) && bitRead(flag, obRear) && !bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = NSE;
  } else if(!bitRead(flag, obFront) && !bitRead(flag, obRear) && bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = W;
  } else if(bitRead(flag, obFront) && !bitRead(flag, obRear) && bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = NW;
  } else if(!bitRead(flag, obFront) && !bitRead(flag, obRear) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = WE;
  } else if(bitRead(flag, obFront) && !bitRead(flag, obRear) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = NWE;
  } else if(!bitRead(flag, obFront) && bitRead(flag, obRear) && bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = SW;
  } else if(bitRead(flag, obFront) && bitRead(flag, obRear) && bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
    tile = SNW;
  } else if(!bitRead(flag, obFront) && bitRead(flag, obRear) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = SWE;
  } else if(bitRead(flag, obFront) && bitRead(flag, obRear) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
    tile = NSWE;
  }

  
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
  }
  else if (bitRead(flag, obRight) && !bitRead(flag, obLeft) ) {
    counter = 0;
    bitSet(state, fright);    //set RIGHT WALL state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && !bitRead(flag, obRight) ) {
    counter = 0;
    bitSet(state, fleft);     //set left wall state
    //clear all other bits
    bitClear(state, fright);  //clear follow wall state
    bitClear(state, wander);  //clear wander state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && bitRead(flag, obRight) ) {
    counter = 0;
    bitSet(state, center);      //set the hallway state
    //clear all other bits
    bitClear(state, fright);    //clear follow wall state
    bitClear(state, wander);    //clear wander state
    bitClear(state, fleft);     //clear follow wall state
  }
}

/*
  Description: 
    Used to move forward a predefined distance in a given direction

  Input:
    dir - direction to move
  
  Return: nothing
*/
void forward2(int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + 400 * dir;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + 400 * dir;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*
  Description: 
    Used to spin a predefined amount in a given direction

  Input:
    dir - direction to spin
  
  Return: nothing
*/
void spin2(int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//spin right
    positions[0] = stepperRight.currentPosition() - 200; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + 200; //left motor absolute position
  }
  else//spin left
  {
    positions[0] = stepperRight.currentPosition() + 200; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() - 200;  //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
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

  while (runNow) {
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
