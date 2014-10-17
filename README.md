minirobot
=========

Arduino Framework for educational robot for Kaite Birkel

By Chris Gerth - Fall 2014

Robot User API:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Motion Functions 
-Can control both direction and speed.
-At any time, speed needs to be set to something besides "Stop" for motion to occur
--ie, at the start, you can't just say "setDirectionFwd()" you have to also call "setSpeedFast()" or something like that.
-Default values: Direction = Fwd, Speed = Stop

void setDirectionFwd(void)
void setDirectionRev(void)
void setDirectionLeft(void)
void setDirectionRight(void)
void setDirectionSharpLeft(void)
void setDirectionSharpRight(void)

void setSpeedMax(void)
void setSpeedFast(void)
void setSpeedMedium(void)
void setSpeedSlow(void)
void setSpeedStop(void)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Output Functions
-Allow the robot to interface with the outside world
-A sample set is provided assuming output devices include red LED, green LED, tri-color LED ("BigLED"), and speaker
-All these are good for debugging (ie, robot should be turning right, so make led green, robot is stopped, so make led red, that sort of thing)

void turnRedLEDOn(void)
void turnRedLEDOff(void)
void turnGreenLEDOn(void)
void turnGreenLEDOff(void)

void setBigLEDColor(const str color)

- colors: 'Red' 'Blue' 'Green' 'Yellow' 'Aqua' 'Magenta' 'White' 'Off'
  
void playBeep1(void)
void playBeep2(void)
void playBeep3(void)
void playBeepCustom(int pitch_in_hz, int length)

void printMessage(const str message)
-prints an arbitrary message to the serial port

void printRobotStatus(void)
-prints block of debugging data to serial port:

"
---------------------------------------------
Seconds since program started: <num_sec>
Direction: <dir>
Speed: <speed>
Left Switch: <Pressed / Not Pressed>
Right Switch: <Pressed / Not Pressed>
Left Light Sensor: <number>%
Right Light Sensor: <number>%
Red LED: <on / off>
Green LED: <on / off>
Big LED: <some color>
---------------------------------------------
"


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Input Functions

boolean CurrentLeftSwitchValue(void) 
boolean CurrentRightSwitchValue(void)

-return true if pressed, false if not pressed
-debouncing performed internally


float CurrentLeftLightSensorVal(void)
float CurrentRightLightSensorVal(void)

-normalized to 0.00-100.00% of full range of analog input
-filtering and linearisation possible


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if/else/while/math/delay() - use arduino-native functions


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Configuration section - Mentor will be required to set this up to match hardware specifications:

//io pin numbers
#define LEFT_MOTOR_PIN <int>
#define RIGHT_MOTOR_PIN <int>
#define LEFT_SWITCH_PIN <int>
#define RIGHT_SWITCH_PIN <int>
#define LEFT_LIGHT_SENSOR_PIN <int>
#define RIGHT_LIGHT_SENSOR_PIN <int>
#define RED_LED_PIN <int>
#define GREEN_LED_PIN <int>
#define BIG_LED_RED_PIN <int>
#define BIG_LED_BLUE_PIN <int>
#define BIG_LED_GREEN_PIN <int>

//pin state inversion (so active-low electrical inputs or outputs map to the proper words/description in the api the kids see)
#define INVERT_LEFT_SWITCH <1 or 0>
#define INVERT_RIGHT_SWITCH <1 or 0>
#define INVERT_LEFT_LIGHT_SENSOR <1 or 0>
#define INVERT_RIGHT_LIGHT_SENSOR <1 or 0>
#define INVERT_BIG_LED <1 or 0>

//speed ranges - pct should be 0 - 100
#define SPEED_MAX <pct>
#define SPEED_FAST <pct>
#define SPEED_MEDIUM <pct>
#define SPEED_SLOW <pct>

//Define what built-in beeps should sound like
//might have to change depending on what speaker is like.
#define BEEP_1_PITCH_HZ <int>
#define BEEP_1_ON_LENGTH_MS <int>
#define BEEP_1_OFF_LENGTH_MS <int>
#define BEEP_1_REPEAT <int>
#define BEEP_2_PITCH_HZ <int>
#define BEEP_2_ON_LENGTH_MS <int>
#define BEEP_2_OFF_LENGTH_MS <int>
#define BEEP_2_REPEAT <int>
#define BEEP_3_PITCH_HZ <int>
#define BEEP_3_ON_LENGTH_MS <int>
#define BEEP_3_OFF_LENGTH_MS <int>
#define BEEP_3_REPEAT <int>
