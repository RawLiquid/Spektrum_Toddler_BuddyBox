/* YourDuinoStarter Example:RECEIVE nRF24L01 Joystick data to control Pan Tilt Servos Over Radio.
   QUESTIONS? terry@yourduino.com
 -WHAT IT DOES:
  -Receives Joystick Analog Values over a nRF24L01 Radio Link, using the Radiohead library.
  - Sends Joystick position to 2 servos, usually X,Y to pan-tilt arrangement
  - TODO! Send the Joystick push-down click to turn Laser on and off
 - SEE the comments after "//" on each line below
 - CONNECTIONS: nRF24L01 Modules See:
 http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 8
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED


   -V2.00 7/12/14 by Noah King
   Based on examples at http://www.airspayce.com/mikem/arduino/RadioHead/index.html
*/

/*-----( Import needed libraries )-----*/
// SEE http://arduino-info.wikispaces.com/Arduino-Libraries  !!
// NEED the SoftwareServo library installed
// http://playground.arduino.cc/uploads/ComponentLib/SoftwareServo.zip
#include <SoftwareServo.h>  // Regular Servo library creates timer conflict!

// NEED the RadioHead Library installed!
// http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.23.zip
#include <RHDatagram.h>
#include <RH_NRF24.h>

#include <SPI.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define CLIENT_ADDRESS     1
#define SERVER_ADDRESS     2

#define ServoHorizontalPIN 3   //Pin Numbers
#define ServoVerticalPIN   5
#define LaserPIN           6

#define ServoMIN_H  20  // Don't go to very end of servo travel
#define ServoMAX_H  150 // which may not be all the way from 0 to 180. 
#define ServoMIN_V  20  // Don't go to very end of servo travel
#define ServoMAX_V  150 // which may not be all the way from 0 to 180. 


/*-----( Declare objects )-----*/
SoftwareServo HorizontalServo;
SoftwareServo VerticalServo;  // create servo objects to control servos
SoftwareServo HorizontalServo2;
SoftwareServo VerticalServo2;  // create servo objects to control servos

// Create an instance of the radio driver
RH_NRF24 RadioDriver;

// Create an instance of a manager object to manage message delivery and receipt, using the driver declared above
RHDatagram RadioManager(RadioDriver, SERVER_ADDRESS);

/*-----( Declare Variables )-----*/
int HorizontalJoystickReceived[2]; // Variable to store received Joystick values
int HorizontalServoPosition[2];    // variable to store the servo position

int VerticalJoystickReceived[2];   // Variable to store received Joystick values
int VerticalServoPosition[2];      // variable to store the servo position

uint8_t ReturnMessage[] = "JoyStick Data Received";  // 28 MAX
// Predefine the message buffer here: Don't put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//--------------------------------( SETUP Runs ONCE )-----------------------------------------------------
void setup()
{
  pinMode(LaserPIN, OUTPUT);
  digitalWrite(LaserPIN, HIGH); // turn on Laser

  /*-----( Set up servos )-----*/
  HorizontalServo.attach(ServoHorizontalPIN);  // attaches the servo to the servo object
  VerticalServo.attach(ServoVerticalPIN);      // attaches the servo to the servo object


  // begin serial to display on Serial Monitor. Set Serial Monitor to 115200
  // See http://arduino-info.wikispaces.com/YourDuino-Serial-Monitor
  ////Serial.begin(115200);
RadioManager.init();
//  if (!RadioManager.init()) // Initialize radio. If NOT "1" received, it failed.
    ////Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
} // END Setup



//--------------------------------( LOOP runs continuously )-----------------------------------------------------
void loop()
{
  if (RadioManager.available())
  {
 // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (RadioManager.recvfrom(buf, &len, &from))
 //Serial Print the values of joystick
    {
      ////Serial.print("got request from : 0x");
      ////Serial.print(from, HEX);
      ////Serial.print(": X = ");
      ////Serial.print(buf[0]);
      ////Serial.print("| Y = ");
      ////Serial.print(buf[1]);
      ////Serial.print(": X = ");
      ////Serial.print(buf[2]);
      ////Serial.print(" Y = ");
      ////Serial.println(buf[3]);

      // Send a reply back to the originator client, check for error
//      if (!RadioManager.sendto(ReturnMessage, sizeof(ReturnMessage), from))
//        ////Serial.println("sendtoWait failed");
    }// end 'IF Received data Available
  }// end 'IF RadioManager Available
  
  {
    SoftwareServo::refresh();//refreshes servo to keep them updating
    HorizontalJoystickReceived[0]  = buf[1];  // Get the values received
    VerticalJoystickReceived[0]    = buf[0]; 
    HorizontalJoystickReceived[1]  = buf[3];  // Get the values received
    VerticalJoystickReceived[1]    = buf[2]; 

    // scale it to use it with the servo (value between MIN and MAX)
    HorizontalServoPosition[0]  = map(HorizontalJoystickReceived[0], 0, 255, ServoMIN_H , ServoMAX_H);
    VerticalServoPosition[0]    = map(VerticalJoystickReceived[0],   0, 255, ServoMIN_V , ServoMAX_V);
    HorizontalServoPosition[1]  = map(HorizontalJoystickReceived[1], 0, 255, ServoMIN_H , ServoMAX_H);
    VerticalServoPosition[1]    = map(VerticalJoystickReceived[1],   0, 255, ServoMIN_V , ServoMAX_V);
    ////Serial.print("H1 : ");
    //Serial.print(HorizontalServoPosition[0]);
    //Serial.print("  V1 : ");
    //Serial.print(VerticalServoPosition[0]);
    //Serial.print("H2 : ");
    //Serial.print(HorizontalServoPosition[1]);
    //Serial.print("  V2 : ");
    //Serial.println(VerticalServoPosition[1]);
    // tell servos to go to position
    HorizontalServo.write(HorizontalServoPosition[0]);
    VerticalServo.write(VerticalServoPosition[0]);
    delay(5);                      // wait for the servo to reach the position
  }
}// END Main LOOP
