#define DEBUG 0
#define PPM_INPUT_25 25
#define PPM_OUTPUT_04 4


#include <PPM.h>
int modval=5;
float spektrumMin = 821;
float spektrumMax = 1621;
//float spektrumMin = 1000;
//float spektrumMax = 2000;


float spread= 500;
PPM myPPM; 	/* Instance our PPM class library	*/
float remoteLimits[5][2];
boolean calibrated = false;

void setup()
{
  myPPM.PPMInput(RISING, 1, PPM_INPUT_25); 		/* Instance pin 25 as RISING edge input pins	*/
  myPPM.PPMOutput(FALLING, 1, PPM_OUTPUT_04); 	/* Instance pin 4 as FALLING edge output pins	*/
  delay(2000);
  for (int z=1;z<5;z++){
    myPPM.dataWrite(PPM_OUTPUT_04, z,2000);
      remoteLimits[z][0]=1020;
      remoteLimits[z][1]=1990;
  }

  Serial.begin(115200);
  Serial.println("Ready");
}

void loop()
{
  // Input is AETR  2314
  // Output must be TAER (3124 )
  int i, num;

  /*	Receive input from Taranis DragonLink on PIN(03) FALLING Edge.  The
   *	DragonLink will emit 12 PPM channels.  Upon arrival, swap channels
   *	1 with channel 8, and leave all the others the same.
   *
   * 	Taranis Dragon-Link input         ==> INPUT:   PIN(05) FALLING	(FTM-00)
   *	INPUT:	PIN(05) FALLING (FTM-00)  ==> OUTPUT:  PIN(04) RISING	(FTM-01)
   */
  unsigned long started=millis();
  while (calibrated == false) {
    while (millis()-started<15000) {

      for (int x = 1; x < 5; x++) {
        float chanval = myPPM.dataRead(PPM_INPUT_25, x);
        if (chanval > 200 && chanval < remoteLimits[x][0]) {
          remoteLimits[x][0] = chanval; //update Min limits for sticks
        }
        if (chanval > remoteLimits[x][1]) {
          remoteLimits[x][1] = chanval; //update Min limits for sticks
        }
      }
    }
    if ((remoteLimits[4][1]) - remoteLimits[4][0] > spread && (remoteLimits[1][1]) - remoteLimits[1][0] > spread && (remoteLimits[2][1]) - remoteLimits[2][0] > spread && (remoteLimits[3][1]) - remoteLimits[3][0] > spread) {
      Serial.println("Calibrated");
      Serial.println("CH    1 	  	    2 		        3  		       4");
      for (int n=1; n<5; n++) {
        Serial.print(remoteLimits[n][0]);
        Serial.print(" - ");
        Serial.print(remoteLimits[n][1]);
        Serial.print("  ");
      }
      Serial.println();

      calibrated = true;
    }
  }
  if (myPPM.dataAvl(PPM_INPUT_25)) {
    for (i = 1; i < 5; i++) {
      float val = myPPM.dataRead(PPM_INPUT_25, i);
      if (val < remoteLimits[i][0]) {
        remoteLimits[i][0] = val;
      }
      if (val > remoteLimits[i][1]) {
        remoteLimits[i][1] = val;
      }
      float sendval;

#if DEBUG==1
      Serial.print(i);
      Serial.print(" - ");
      Serial.print(sendval);
      Serial.print(" ");
#endif
      switch (i) {
      case 1:
        sendval=mapf(val, remoteLimits[i][0], remoteLimits[i][1], spektrumMin, spektrumMax);
        myPPM.dataWrite(PPM_OUTPUT_04, 2, sendval);
        break;

      case 2:
        sendval=mapf(val, remoteLimits[i][0], remoteLimits[i][1], spektrumMax+2, spektrumMin-1);
        myPPM.dataWrite(PPM_OUTPUT_04, 3, sendval);
        break;

      case 3:
        sendval=mapf(val, remoteLimits[i][0], remoteLimits[i][1], spektrumMin, spektrumMax);
        //myPPM.dataWrite(PPM_OUTPUT_04, 1, (((spektrumMax - spektrumMin) / 2) + spektrumMin));
        myPPM.dataWrite(PPM_OUTPUT_04, 1, sendval);
        break;
      case 4:
        sendval=mapf(val, remoteLimits[i][0], remoteLimits[i][1], spektrumMax+2, spektrumMin-1);
        myPPM.dataWrite(PPM_OUTPUT_04, 4, sendval);
        break;
      default:
        myPPM.dataWrite(PPM_OUTPUT_04, i,val);
        Serial.print(i);
        Serial.print(" - ");
        Serial.println(val);
        break;
      }
#if DEBUG==1
      Serial.println();
#endif
    }
  }

  /* add main program code here */

}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void serialEvent() {
  char inChr=(char)Serial.read();
  Serial.println(inChr);
  if (inChr=='<') {
    spektrumMin=spektrumMin+modval;
  }
  if (inChr=='>') {
    spektrumMax=spektrumMax+modval;
  }
  if(inChr==',') {
    spektrumMin=spektrumMin-modval;
  }
  if (inChr=='.') {
    spektrumMax=spektrumMax-modval;
  }
  if(inChr=='[') {
    modval=modval-1;
  }
  if (inChr==']') {
    modval=modval+1;
  }
  if (inChr=='?') {
   Serial.print("In - ");
   for (int i = 1; i < 5; i++) {
      float val = myPPM.dataRead(PPM_INPUT_25, i);
      Serial.print("CH:");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(val);
      Serial.print("   ");
     }
     Serial.println();
     
   Serial.print("Out - ");
   for (int i = 1; i < 5; i++) {
      float val = myPPM.dataRead(PPM_INPUT_25, i);
      float sndval=mapf(val, remoteLimits[i][0], remoteLimits[i][1], spektrumMin, spektrumMax);
      Serial.print("CH:");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(sndval);
      Serial.print("   ");
     }
     Serial.println();

     
     
  }
  if (inChr=='v') {
          Serial.println("CH    1 	  	    2 		        3  		       4");
      for (int n=1; n<5; n++) {
        Serial.print(remoteLimits[n][0]);
        Serial.print(" - ");
        Serial.print(remoteLimits[n][1]);
        Serial.print("  ");
      }
      Serial.println();
  }

  Serial.print("Min=");
  Serial.print(spektrumMin);
  Serial.print(" Max=");
  Serial.println(spektrumMax);
}

