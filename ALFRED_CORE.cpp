// Do not remove the include below
#include "ALFRED_CORE.h"


Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x41);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define LED 13 // This is the L LED
#define one_sec 1000000
#define thirty_sec 30000000

// our servo # counter
uint8_t servonum = 4;

int power_servos = 31; // This is our pin to control the power to the servos
char serialData[32];
byte com = 0;
byte error = 0;
byte timerCounter = 0;
boolean connected;

void reset()
{
	digitalWrite(LED, LOW);
    connected = false;
}


boolean parseCommand(char* command, int* returnValues, byte returnNumber)
/**
 * This function makes int's out of the received serial data, the 2 first
 * characters are not counted as they consist of the command character and
 * a comma separating the first variable.
 *
 * @params command The whole serial data received as an address
 * @params returnValues The array where the return values go as an address
 * @params returnNumber The number of values to set inside the returnValues variable
 */
{
  // parsing state machine
  byte i = 1, j = 0, sign = 0, ch = 0, number;
  int temp = 0;
  while(i++)
  {
    switch(*(command + i))
    {
    case '\0':
    case ':':
      // set return value
      if(ch != 0)
      {
        returnValues[j++] = sign?-temp:temp;
        sign = 0;
        temp = 0;
        ch = 0;
      }
      else
      {
        return false;
      }
      break;
    case '-':
      sign = 1;
      break;
    default:
      // convert string to int
      number = *(command + i) - '0';
      if(number < 0 || number > 9)
      {
        return false;
      }
      temp = temp * 10 + number;
      ch++;
    }

    // enough return values have been set
    if(j == returnNumber)
    {
      return true;
    }
    // end of command reached
    else if(*(command + i) == '\0')
    {
      return false;
    }
  }
}

void timeout_handler()
{
    // no data has been passed since last time
    // interpret as communication failure
    reset();

}


//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialisation code here
	Serial.begin(9600);
	Serial.println("16 channel Servo test!");

	  pwmR.begin();
	  pwmR.setPWMFreq(50);  // Analogue servo's run at ~50 Hz updates

	  pwmL.begin();
	  pwmL.setPWMFreq(50);  // Analogue servo's run at ~50 Hz updates

	  pinMode(power_servos, OUTPUT);
	  digitalWrite(power_servos, LOW);

	  pinMode(LED, OUTPUT);
	  // the bluetooth dongle communicates at 115200 baud only
	  Serial1.begin(115200);
	  // set up timeout timer on timer2 in case of lost connection
	  // Initialise counter
	  Timer2.attachInterrupt(timeout_handler); //Initialise timed interrupt
	  Timer2.start(thirty_sec); // Call thirty second timed interrupt
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

	  if(Serial1.available() > 0)
	  {
		Timer2.stop();
		Timer2.detachInterrupt();
	    digitalWrite(LED, HIGH);
	    connected = true;

	    Serial1.readBytesUntil('\n', serialData, 31);
	    switch(serialData[0])
	    {
	    case 0:
	      Serial1.println(0);
	      break;
	    case 'd':
	      // set left and right motor speeds
	      int speed[2];
	      if(parseCommand(serialData, speed, 2))
	      {
	        //setSpeed(speed[0], speed[1]);
	        Serial1.println("New speed set");
	      }
	      else
	      {
	        Serial1.println("Error while setting new speed");
	      }
	      break;
	    case 'i':
	      // inform about robot
	      Serial1.println("ALFRED 1.0");
	      break;
	    case 'r':
	      // quickly stop
	      reset();
	      Serial1.println("Robot reset");
	      break;
	    default:
	      // inform user of non existing command
	      Serial1.println("Command not recognised");
	    }

	    // clear serialData array
	    memset(serialData, 0, sizeof(serialData));
	  }
	  else
	  {
		  Timer2.attachInterrupt(timeout_handler); //Initialise timed interrupt
		  Timer2.start(thirty_sec); // Call thirty second timed interrupt
	  }
	  // Drive each servo one at a time
	 /* Serial.println(servonum);

	  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
		  Serial.println(servonum);
		  Serial.println(pulselen);
		  pwmR.setPWM(servonum, 0, pulselen);
		  pwmL.setPWM(servonum, 0, pulselen);
	  }
	  delay(500);
	  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
		  Serial.println(servonum);
		  Serial.println(pulselen);
		  pwmR.setPWM(servonum, 0, pulselen);
		  pwmL.setPWM(servonum, 0, pulselen);
	  }
	  delay(500);

	  servonum ++;
	  if (servonum > 15) servonum = 4; */
}

