/*
  minirobot
  A framework for basic, small robotics programming
  Fall 2014
  
  Architected by Chris Gerth
  
  Config file - Mentor will be required to set these values to match hardware specifications.
 */

/*********************************************************/
//io pin numbers
/*********************************************************/

//DO NOT USE pins 0 or 1, these are needded for serial communication

//Motors must be connected to PWM-capiable pins which do not get
//screwed up by the usage of "tone()".
//on an arduino UNO, these are 5,6,9,10
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 10


//light sensors must be connected to an analog input (A0-A6)
#define LEFT_LIGHT_SENSOR_PIN A0
#define RIGHT_LIGHT_SENSOR_PIN A1

//No restrictions on LED or input pins
#define LEFT_SWITCH_PIN 2
#define RIGHT_SWITCH_PIN 4
#define RED_LED_PIN 7
#define GREEN_LED_PIN 8
#define RGB_LED_RED_PIN 5
#define RGB_LED_BLUE_PIN 6
#define RGB_LED_GREEN_PIN 12
#define SPEAKER_PIN 3

/*********************************************************/
//io state inversions
/*********************************************************/
//pin state inversion (so active-low electrical inputs or outputs map to the proper words/description in the api the kids see)
#define INVERT_LEFT_SWITCH 0
#define INVERT_RIGHT_SWITCH 0
#define INVERT_LEFT_LIGHT_SENSOR 0
#define INVERT_RIGHT_LIGHT_SENSOR 0
#define INVERT_RGB_LED 0


/*********************************************************/
//User Constants
/*********************************************************/
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
