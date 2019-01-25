/*
 * Author: Devon Adair and Hunter LaMantia
 * Date: 1/21/2019
 * CM: 2202
 * 
 * Description:
 *    Test program to read the sensor data to get calibration
 *    factors for the main program.
 */

#define photoLeft   A1  // left photo resistor
#define photoRight  A0  // right photo resistor

#define baud_rate 9600  // serial communication baud rate

int leftSensor, rightSensor;
/*
 * Setup pins and serial communication
 */
 void setup(void)
 {

    Serial.begin(baud_rate); // start serial communication for printing
 }

/*
 * Main loop to call and update values
 */
 void loop(void)
 {
  int i;
  for(i = 0; i < 5; i++) {
    rightSensor += analogRead(photoRight);
    leftSensor += analogRead(photoLeft);
  }
  Serial.print("Left Sensor: ");
  Serial.print(leftSensor / 5);
  Serial.print("      Right Sensor: ");
  Serial.println(rightSensor / 5);

  rightSensor = 0;
  leftSensor = 0;
  delay(100);
 }

 /*
  * Read the right sensor
  * 
  * returns: analog value in volts
  */
int readRightPhotoSensor()
{
  return analogRead(photoRight);
}

 /*
  * Read the left sensor
  * 
  * returns: analog value in volts
  */
int readLeftPhotoSensor()
{
  return analogRead(photoLeft);
}
