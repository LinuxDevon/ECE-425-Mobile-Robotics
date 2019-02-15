/*RobotTransceiver.ino
  Authors: Carlotta Berry, Ricky Rung
  modified: 11/23/16
  This program will set up the laptop to use a nRF24L01 wireless transceiver to
  communicate wirelessly with a mobile robot
  the transmitter is an Arduino Mega connected to the laptop
  the receiver is on an Arduino Mega mounted on the robot

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

int i, j;
int rowCount = 0;
bool waitForRow = false;

void setup() {
  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openWritingPipe(pipes[0]);//open up writing pipe
  radio.openReadingPipe(1, pipes[1]);//open up reading pipe
//  radio.startListening();//start listening for data;
}

uint8_t recieveInfo[9];

// code was referenced from here: https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
void loop() {
  //use serial monitor to send 0 and 1 to blink LED on digital pin 13 on robot microcontroller
  delay(5);
//  if(data[0] == 0) {
//    delay(1000);
//  }
  radio.stopListening();
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
    Serial.println(data[0]);
    radio.write(data, sizeof(data));
  }
//  Serial.print(data[0]);
  delay(5);
  radio.startListening();
  if(data[0] == 1) {
    for( i = 0; i < 9; i++) {
      while(!radio.available());
      radio.read(&recieveInfo, sizeof(recieveInfo));
      for (j = 0; j < 9; j++) {
        if(recieveInfo[i] < 10) {
           Serial.print(0);
        }
        Serial.print(recieveInfo[i]);
        Serial.print(", ");
      }
      Serial.println();
    }
    
  }
  if (radio.available()) {
    radio.read(&recieveInfo, sizeof(recieveInfo));
    for(i = 0; i < 9; i++) {
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