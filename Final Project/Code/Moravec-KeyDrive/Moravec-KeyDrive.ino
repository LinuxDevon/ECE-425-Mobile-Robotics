//#include <PS2X_lib.h>

/*RobotReceiver.ino
  Authors: Carlotta Berry, Ricky Rung
  modified: 11/23/16
  This program will set up the laptop to use a nRF24L01 wireless transceiver to
  communicate wirelessly with a mobile robot
  the transmitter is an Arduino Mega connected to the laptop
  the receiver is on an Arduino Mega mounted on the robot

  https://www.arduino.cc/en/Hacking/PinMapping2560
  Arduino MEGA nRF24L01 connections
    CE  pin 7         CSN   pin 8
    MOSI  pin 51      MISO  pin 50
    SCK pin 52        VCC 3.3 V
    GND GND
*/

#include <SPI.h>      //include serial peripheral interface library
#include <RF24.h>     //include wireless transceiver library
#include <nRF24L01.h> //include wireless transceiver library
#include <printf.h>
#include <RF24_config.h>
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <TimerOne.h>//include timer interrupt library

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define test_LED 16
#define team_channel 69   //transmitter and receiver on same channel between 1 & 125

//define stepper motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin     46  //right stepper motor step pin
#define rtDirPin      53  // right stepper motor direction pin
#define ltStepPin     44  //left stepper motor step pin
#define ltDirPin      49  //left stepper motor direction pin

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

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                //create instance to control multiple steppers at the same time

// Set up constants for movement
#define FORWARD 1
#define BACKWARD -1
#define LEFT 0
#define RIGHT 1

//bit definitions for robot motion and state byte [follow_hallway follow_right follow_left wander avoid]
volatile byte state = 0;
#define avoid     0   //avoid behavior
#define wander    1   //wander behavior
#define fleft     2   //follow left wall behavior
#define fright    3   //follow right wall behavior
#define center    4   //follow hallway behavior
#define movingL   6   //robot left wheel moving
#define movingR   7   //robot right wheel moving

const uint64_t pipes[2] = {0xE8E8F0F0E1LL, 0xE8E8F0F0A1LL}; //define the radio transmit pipe (5 Byte configurable)
RF24 radio(CE_PIN, CSN_PIN);          //create radio object
uint8_t data[1];                      //variable to hold transmit data
uint8_t sendData[1];

void setup() {
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
//  radio.openWritingPipe(pipe);//open up writing pipe
  pinMode(test_LED, OUTPUT);//set LED pin as an output
}

void loop() {
  delay(5);
  radio.startListening();
  while (radio.available()) {
//    Serial.println("test");
    radio.read(&data, sizeof(data));
//    Serial.println(data[0]);
    if(data[0] == 8) {
      Serial.println("forward");
      forward(FORWARD);
    } else if(data[0] == 2) {
      Serial.println("backward");
      forward(BACKWARD);
    } else if(data[0] == 4) {
      Serial.println("left");
      spin(LEFT);
    } else if(data[0] == 6) {
      Serial.println("right");
      spin(RIGHT);
//    } else {
//      int num = 1;
//      radio.write(num, 1);
    }
  }
  delay(5);
  radio.stopListening();
  sendData[0] = 38;
  radio.write(sendData, sizeof(sendData));
}

/*
  Description: 
    Used to move forward a predefined distance in a given direction

  Input:
    dir - direction to move
  
  Return: nothing
*/
void forward(int dir) {
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
void spin(int dir) {
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
