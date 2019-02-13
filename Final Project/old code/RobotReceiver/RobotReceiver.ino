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

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define test_LED 16
#define team_channel 69   //transmitter and receiver on same channel between 1 & 125

const uint64_t pipe = 0xE8E8F0F0E1LL; //define the radio transmit pipe (5 Byte configurable)
RF24 radio(CE_PIN, CSN_PIN);          //create radio object
uint8_t data[1];                      //variable to hold transmit data

void setup() {
  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openReadingPipe(1, pipe);//open up reading pipe
  radio.startListening();;//start listening for data;
  pinMode(test_LED, OUTPUT);//set LED pin as an output
}

void loop() {
  while (radio.available()) {
    radio.read(&data, sizeof(data));
    digitalWrite(test_LED, data[0]);
    Serial.println("change the light");
  }
}
