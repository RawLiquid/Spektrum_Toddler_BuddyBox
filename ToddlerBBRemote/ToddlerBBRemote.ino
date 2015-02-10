/* YourDuinoStarter Example: TRANSMIT nRF24L01 Joystick data to Pan Tilt Over Radio.
   QUESTIONS? terry@yourduino.com
 - WHAT IT DOES: Reads Joystick Analog Values on A0, A1 and transmits
   them over a nRF24L01 Radio Link, using the Radiohead library.
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

      -
   Analog Joystick or two 10K potentiometers:
   GND to Arduino GND
   VCC to Arduino +5V
   X Pot to Arduino A5
   Y Pot to Arduino A4
   Click Button to pin 4

   -V2.00 7/12/14  by Noah King
   Based on examples at http://www.airspayce.com/mikem/arduino/RadioHead/index.html
*/

/*-----( Import needed libraries )-----*/
// SEE http://arduino-info.wikispaces.com/Arduino-Libraries  !!
// NEED the RadioHead Library installed!
// http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.23.zip
#include <RHDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>


/*-----( Declare Constants and Pin Numbers )-----*/
#define JoyStick_X_PIN     A5  //Pin Numbers
#define JoyStick_Y_PIN     A4
#define JoyStick2_X_PIN     A3  //Pin Numbers
#define JoyStick2_Y_PIN     A2
#define ClickPIN           4

#define CLIENT_ADDRESS 1      // For Radio Link
#define SERVER_ADDRESS 2


// Create an instance of the radio driver
RH_NRF24 RadioDriver;

// Create an instance of a manager object to manage message delivery and receipt, using the driver declared above
RHDatagram RadioManager(RadioDriver, CLIENT_ADDRESS);// sets the driver to NRF24 and the client adress to 1

/*-----( Declare Variables )-----*/
uint8_t joystick[8];  // 2 element array of unsigned 8-bit type, holding Joystick readings
uint8_t joysticklast[8];  // 2 element array of unsigned 8-bit type, holding Joystick readings

uint16_t joystickraw[4];  // 2 element array of unsigned 8-bit type, holding Joystick readings
uint16_t joystickLimits[4][2];

// Predefine the message buffer here: Don't put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];    // Actually: 28 bytes (32 minus 4 byte header)

void setup()  /****** SETUP: RUNS ONCE ******/
{

  // begin serial to display on Serial Monitor. Set Serial Monitor to 115200
  // See http://arduino-info.wikispaces.com/YourDuino-Serial-Monitor
  ////Serial.begin(115200);
uint16_t joycenter[4]= {0,0,0,0};
//Stick calibration
while(joycenter[0]==joystickraw[0] || joycenter[1]==joystickraw[1] || joycenter[2]==joystickraw[2] || joycenter[3]==joystickraw[3] || joystickLimits[0][1]-joystickLimits[0][0] < 500 || joystickLimits[1][1]-joystickLimits[1][0] < 500 || joystickLimits[2][1]-joystickLimits[2][0] < 500 || joystickLimits[3][1]-joystickLimits[3][0] < 500) {

  joystickraw[0] = analogRead(JoyStick_X_PIN);
  joystickraw[1] = analogRead(JoyStick_Y_PIN);
  joystickraw[2] = analogRead(JoyStick2_X_PIN);
  joystickraw[3] = analogRead(JoyStick2_Y_PIN);

  for(int x = 0; x < 4; x++ ) {
	  if(joystickraw[x]<joystickLimits[x][0]) {joystickLimits[x][0]=joystickraw[x];} //update Min limits for sticks
	  if(joystickraw[x]>joystickLimits[x][1]) {joystickLimits[x][1]=joystickraw[x];} //update Max limits for sticks
	  }
  if(joycenter[0]==0){ for(int j=0;j<4;j++) { joycenter[j]=joystickraw[j]; }; }
  }
		  
	  




  // NOTE: pinMode for Radio pins handled by RadioDriver
  if (!RadioManager.init())   // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
    //Serial.println("init failed");

  pinMode(ClickPIN, INPUT);  //Not really needed: pins default to INPUT
}



void loop() /****** LOOP: RUNS CONSTANTLY ******/
{
  //Read the joystick values, scale them to 8-bit type and store them in the joystick[] array.
  //Serial.println("Reading joystick values ");
  // Take the value of Joystick voltages which are 0 to 1023 (10 bit), and convert them to 0 to 255 (8 bit)
  joystickraw[0] = analogRead(JoyStick_X_PIN);
  joystickraw[1] = analogRead(JoyStick_Y_PIN);
  joystickraw[2] = analogRead(JoyStick2_X_PIN);
  joystickraw[3] = analogRead(JoyStick2_Y_PIN);

for(int x = 0; x < 4; x++ ) {
  if(joystickraw[x]<joystickLimits[x][0]) {joystickLimits[x][0]=joystickraw[x];}
  if(joystickraw[x]>joystickLimits[x][1]) {joystickLimits[x][1]=joystickraw[x];}
  uint8_t *joysticktemp;
  
  joysticktemp = convertFrom16To8(map(joystickraw[x], joystickLimits[x][0], joystickLimits[x][1], 0, 1023));
    joystick[x*2]=joysticktemp[0];
    joystick[(x*2)+1]=joysticktemp[1];
  }

  //Display the joystick values in the serial monitor.
  //Serial.print("x:");
  //Serial.print(joystickraw[0]);
  //Serial.print(" / ");
  //Serial.print(joystick[0]);
  //Serial.print("  y:");
  //Serial.print(joystickraw[1]);
  //Serial.print(" / ");
  //Serial.print(joystick[1]);

  //Serial.print("  x:");
  //Serial.print(joystickraw[2]);
  //Serial.print(" / ");
  //Serial.print(joystick[2]);
  //Serial.print("  y:");
  //Serial.print(joystickraw[3]);
  //Serial.print(" / ");
  //Serial.println(joystick[3]);

  //Serial.println("Sending Joystick data to nrf24_reliable_datagram_server");
  //Send a message containing Joystick data to manager_server
  if (joystick != joysticklast)
   {joysticklast==joystick;
    RadioManager.sendto(joystick, sizeof(joystick), SERVER_ADDRESS);


//     // Now wait for a reply from the server
//     uint8_t len = sizeof(buf);
//     uint8_t from;
// //    if (RadioManager.recvfromAckTimeout(buf, &len, 2000, &from))
// //    {
// //      //Serial.print("got reply from : 0x");
// //      //Serial.print(from, HEX);
// //      //Serial.print(": ");
// //      //Serial.println((char*)buf);
// //    }
// //    else
// //    {
// //      //Serial.println("No reply, is nrf24_datagram_server running?");
// //    }
   }
//  else
    //Serial.println("sendto failed");

//  delay(500);  // Wait a bit before next transmission
}


uint16_t convertFrom8To16(uint8_t dataFirst, uint8_t dataSecond) {
    uint16_t dataBoth = 0x0000;

    dataBoth = dataFirst;
    dataBoth = dataBoth << 8;
    dataBoth |= dataSecond;
    return dataBoth;
}

uint8_t *convertFrom16To8(uint16_t dataAll) {
    static uint8_t arrayData[2] = { 0x00, 0x00 };

    *(arrayData) = (dataAll >> 8) & 0x00FF;
    arrayData[1] = dataAll & 0x00FF;
    return arrayData;
}

