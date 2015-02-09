// ToddlerBuddyBox.ino

//#include "I2Cdev.h"

//#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#define throttlePin A6
#define pitchPin A5
#define rollPin A4
#define yawPin A7


#define PPM_PIN 5
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PRINT_RAW_DATA 1

// ================================================================
// ===                        VARIABLES                         ===
// ================================================================

bool blinkState = false;

// MPU control/status vars
//MPU6050 mpu;
//bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int throttle,roll,pitch,yaw;

//output vars
unsigned long timer = 0;
int rcMin = 625;
int rcMax = 1500;

int rollMin = 500;
int rollMax = 1700;
int pitchMin = 500;
int pitchMax = 1700;
int throttleMin = 500;
int throttleMax = 1500;
int yawMin = 500;
int yawMax = 1700;


long framelength = 21980;
int frameDelay = 400;

// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================


void pulseDataOut(int pin, int data[8]) {
  delayMicroseconds(framelength-(data[0]+data[1]+data[2]+data[3]+data[4]+data[5]+data[6]+data[7])-(frameDelay*9));
  digitalWrite(pin,LOW); 
  delayMicroseconds(frameDelay);
  for (int x=0;x<8;x++)
  {
	  digitalWrite(pin,HIGH);
	  delayMicroseconds(data[x]);
	  digitalWrite(pin,LOW);
	  delayMicroseconds(frameDelay);
 digitalWrite(pin,HIGH);
  }
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
 // join I2C bus (I2Cdev library doesn't do this automatically)
    //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        //Wire.begin();
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        //Fastwire::setup(400, true);
    //#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(PPM_PIN,OUTPUT);
 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
        pitch = analogRead(pitchPin);
        roll = analogRead(rollPin);
        throttle = analogRead(throttlePin);
        yaw = analogRead(yawPin);
        
  
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        if (digitalRead(A1)) { Serial.println("\t\t\t\tButton");};
    
    int channelData[8]={1000,1000,1000,1000,1000,1000,1000,1000};
    //every 20 milliseconds, send out the PPM data
      channelData[2] = map(pitch,1024,0,rcMin,rcMax); //map(analogRead(pitch),0,1024,pitchMax,pitchMin);
      channelData[1] = map(roll,0,1024,rcMin,rcMax); //map(analogRead(roll),0,1024,rollMax,rollMin);
      channelData[0] = map(throttle,1024,0,throttleMin,throttleMax); //map(analogRead(throttle),0,1024,throttleMin,throttleMax);
	  channelData[3] = map(yaw,0,1024,rcMin,rcMax);

        #if PRINT_RAW_DATA == 1
        Serial.print(roll); Serial.print("x");
        Serial.print(pitch); Serial.print("x");
        Serial.print(throttle); Serial.print("x");
        Serial.print(yaw); Serial.print("\n");
        #elif PRINT_RAW_DATA == 0
        Serial.print(channelData[0]); Serial.print("x");
        Serial.print(channelData[1]); Serial.print("x");
        Serial.print(channelData[2]); Serial.print("x");
        Serial.print(channelData[3]); Serial.print("\n");
        #endif

    if (timer + 20 < millis()) {
		timer=millis();
        pulseDataOut(PPM_PIN, channelData);
      }
}


