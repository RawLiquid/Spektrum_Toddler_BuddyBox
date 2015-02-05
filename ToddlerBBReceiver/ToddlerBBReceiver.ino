//#include <Adafruit_ssd1306syp.h>
//#define SDA_PIN 4
//#define SCL_PIN 5
//Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);


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
//#include <SoftwareServo.h>  // Regular Servo library creates timer conflict!
#define RH_ASK_ARDUINO_USE_TIMER2
//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 4  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////
int throttle,roll,pitch,yaw;
int rcMin = 500;
int rcMax = 1700;

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

// NEED the RadioHead Library installed!
// http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.23.zip
#include <RHDatagram.h>
#include <RH_NRF24.h>

#include <SPI.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define CLIENT_ADDRESS     1
#define SERVER_ADDRESS     2

//#define ServoHorizontalPIN 3   //Pin Numbers
//#define ServoVerticalPIN   5
#define LaserPIN           6

#define ServoMIN_H  20  // Don't go to very end of servo travel
#define ServoMAX_H  150 // which may not be all the way from 0 to 180. 
#define ServoMIN_V  20  // Don't go to very end of servo travel
#define ServoMAX_V  150 // which may not be all the way from 0 to 180. 


/*-----( Declare objects )-----*/
//SoftwareServo HorizontalServo;
//SoftwareServo VerticalServo;  // create servo objects to control servos
//SoftwareServo HorizontalServo2;
//SoftwareServo VerticalServo2;  // create servo objects to control servos

// Create an instance of the radio driver
RH_NRF24 RadioDriver;

// Create an instance of a manager object to manage message delivery and receipt, using the driver declared above
RHDatagram RadioManager(RadioDriver, SERVER_ADDRESS);

/*-----( Declare Variables )-----*/

//unsigned int HorizontalJoystickReceived[2]; // Variable to store received Joystick values
//unsigned int HorizontalServoPosition[2];    // variable to store the servo position

//unsigned int VerticalJoystickReceived[2];   // Variable to store received Joystick values
//unsigned int VerticalServoPosition[2];      // variable to store the servo position

uint8_t ReturnMessage[] = "JoyStick Data Received";  // 28 MAX
// Predefine the message buffer here: Don't put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//--------------------------------( SETUP Runs ONCE )-----------------------------------------------------
void setup()
{
//  pinMode(LaserPIN, OUTPUT);
//  digitalWrite(LaserPIN, HIGH); // turn on Laser
//delay(1000);
//display.initialize();
  /*-----( Set up servos )-----*/
//  HorizontalServo.attach(ServoHorizontalPIN);  // attaches the servo to the servo object
//  VerticalServo.attach(ServoVerticalPIN);      // attaches the servo to the servo object
 for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }


  // begin serial to display on Serial Monitor. Set Serial Monitor to 115200
  // See http://arduino-info.wikispaces.com/YourDuino-Serial-Monitor
  ////Serial.begin(115200);
RadioManager.init();
//  if (!RadioManager.init()) // Initialize radio. If NOT "1" received, it failed.
    ////Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

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
      //test
    }// end 'IF Received data Available
  }// end 'IF RadioManager Available
  
  {
//    SoftwareServo::refresh();//refreshes servo to keep them updating
      ppm[2] = convertFrom8To16(buf[4],buf[5])+1000; //map(analogRead(pitch),0,1024,pitchMax,pitchMin);
      ppm[1] = convertFrom8To16(buf[2],buf[3])+1000; //map(analogRead(roll),0,1024,rollMax,rollMin);
      ppm[0] = convertFrom8To16(buf[0],buf[1])+1000; //map(analogRead(throttle),0,1024,throttleMin,throttleMax);
      ppm[3] = convertFrom8To16(buf[6],buf[7])+1000;
 
//    HorizontalJoystickReceived[0]  = convertFrom8To16(buf[2],buf[3]);  // Get the values received
//    VerticalJoystickReceived[0]    = convertFrom8To16(buf[0],buf[1]); 
//    HorizontalJoystickReceived[1]  = convertFrom8To16(buf[4],buf[5]);  // Get the values received
//    VerticalJoystickReceived[1]    = convertFrom8To16(buf[6],buf[7]); 
//
//    // scale it to use it with the servo (value between MIN and MAX)
//    HorizontalServoPosition[0]  = map(HorizontalJoystickReceived[0], 0, 1023, ServoMIN_H , ServoMAX_H);
//    VerticalServoPosition[0]    = map(VerticalJoystickReceived[0],   0, 1023, ServoMIN_V , ServoMAX_V);
//    HorizontalServoPosition[1]  = map(HorizontalJoystickReceived[1], 0, 1023, ServoMIN_H , ServoMAX_H);
//    VerticalServoPosition[1]    = map(VerticalJoystickReceived[1],   0, 1023, ServoMIN_V , ServoMAX_V);
    ////Serial.print("H1 : ");
    //Serial.print(HorizontalServoPosition[0]);
    //Serial.print("  V1 : ");
    //Serial.print(VerticalServoPosition[0]);
    //Serial.print("H2 : ");
    //Serial.print(HorizontalServoPosition[1]);
    //Serial.print("  V2 : ");
    //Serial.println(VerticalServoPosition[1]);
    // tell servos to go to position
//    HorizontalServo.write(HorizontalServoPosition[0]);
//    VerticalServo.write(VerticalServoPosition[0]);
//    delay(5);                      // wait for the servo to reach the position
  }
}// END Main LOOP
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
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

