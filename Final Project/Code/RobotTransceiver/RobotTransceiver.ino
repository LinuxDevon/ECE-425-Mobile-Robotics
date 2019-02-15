/*RobotTransceiver.ino
  Authors: Carlotta Berry, Ricky Rung
  modified: 2/15/19
  Modified by: Devon Adair and Hunter Lamantia
  This program will set up the laptop to use a nRF24L01 wireless transceiver to
  communicate wirelessly with a mobile robot
  the transmitter is an Arduino Mega connected to the laptop
  the receiver is on an Arduino Mega mounted on the robot

  The modification that were made now allow bidirectional communication.
  The robot sends the map to the arduino on the laptop row by row for a total of 9 rows (9x9).
  The arduino on the laptop send commands to the robot to drive it and control behavior.

  HARDWARE CONNECTIONS:

  https://www.arduino.cc/en/Hacking/PinMapping2560
  Arduino MEGA nRF24L01 connections
    CE  pin 7         CSN   pin 8
    MOSI  pin 51      MISO  pin 50
    SCK pin 52        VCC 3.3 V
    GND GND

    Arduino Uno nRF24L01 connections
     http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
     http://www.theengineeringprojects.com/2015/07/interfacing-arduino-nrf24l01.html
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>
#include <printf.h>
#include <RF24_config.h>


// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define test_LED 13
#define team_channel 69   //transmitter and receiver on same channel between 1 & 125

const uint64_t pipes[2] = {0xE8E8F0F0E1LL, 0xE8E8F0E07DLL}; //define the radio transmit pipe (5 Byte configurable)
RF24 radio(CE_PIN, CSN_PIN);          //create radio object
uint8_t data[1];                      //variable to hold transmit data

int i;                  // index of the row for the map
int rowCount = 0;       // count how many rows and after 9 print a new line to seperate the maps

uint8_t recieveInfo[9]; // the row buffer used to recieve a row at a time

void setup() {
  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openWritingPipe(pipes[0]);//open up writing pipe
  radio.openReadingPipe(1, pipes[1]);//open up reading pipe
}

// The code now looks for a input to serial console and writes it to the robot if there is an input
// otherwise it reads a map that is sent from the bot.
// the map is sent row by row which come in 9 numbers
void loop() {
  // Write commands to the robot if there is an input
  delay(5);
  radio.stopListening();
  if (Serial.available() > 0) { // if there is an input write
    data[0] = Serial.parseInt();  // read the input from the console
    Serial.println(data[0]);
    radio.write(data, sizeof(data));  // push it wirelessly
  }

  // start listening for the map
  delay(5);
  radio.startListening();
  if (radio.available()) {  // if it recieves a row print and read it
    radio.read(&recieveInfo, sizeof(recieveInfo));
    for(i = 0; i < 9; i++) {
      // print the array and make it look nice
      if(recieveInfo[i] < 10) {
        Serial.print(0);
      }
      Serial.print(recieveInfo[i]);
      Serial.print(", ");
    }
    rowCount++;
    Serial.println();
    if(rowCount == 9) {
      Serial.println();
      rowCount = 0;
    }
  }
}
