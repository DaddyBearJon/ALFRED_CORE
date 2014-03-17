// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _ALFRED_CORE_H_
#define _ALFRED_CORE_H_
#include "Arduino.h"
//add your includes for the project ALFRED_CORE here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ALFRED_CORE here

#include "Adafruit_PWMServoDriver.h"
#include <DueTimer.h>



//Do not add code below this line
#endif /* _ALFRED_CORE_H_ */
