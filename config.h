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
//screwed up by the usage of "tone()". 5 and 6 can be screwy sometimes too.
//So on an arduino Uno, just use 9 and 10...
#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_PWM_PIN 6


//light sensors must be connected to an analog input (A0-A6)
#define LEFT_LIGHT_SENSOR_PIN A0
#define RIGHT_LIGHT_SENSOR_PIN A1

//No restrictions on other devices
#define LEFT_SWITCH_PIN 2
#define RIGHT_SWITCH_PIN 4
#define RED_LED_PIN 7
#define GREEN_LED_PIN 8
#define LEFT_MOTOR_DIR_PIN 9
#define RIGHT_MOTOR_DIR_PIN 10

/*********************************************************/
//io state inversions
/*********************************************************/
//pin state inversion (so active-low electrical inputs or outputs map to the proper words/description in the api the kids see)
#define INVERT_LEFT_SWITCH 0
#define INVERT_RIGHT_SWITCH 0
#define INVERT_LEFT_LIGHT_SENSOR 0
#define INVERT_RIGHT_LIGHT_SENSOR 0
#define INVERT_RED_LED 0
#define INVERT_GREEN_LED 0
#define INVERT_LEFT_MOTOR_DIR 0
#define INVERT_RIGHT_MOTOR_DIR 0


/*********************************************************/
//User Constants
/*********************************************************/
//speed ranges - pct should be 0 - 100
#define SPEED_MAX 100
#define SPEED_FAST 75
#define SPEED_MEDIUM 50
#define SPEED_SLOW 25

//Define what built-in beeps should sound like
//might have to change depending on what speaker is like.
#define BEEP_1_PITCH_HZ 1397
#define BEEP_1_ON_LENGTH_MS 250
#define BEEP_1_OFF_LENGTH_MS 250
#define BEEP_1_REPEAT 3
#define BEEP_2_PITCH_HZ 1760
#define BEEP_2_ON_LENGTH_MS 1000
#define BEEP_2_OFF_LENGTH_MS 0
#define BEEP_2_REPEAT 1
#define BEEP_3_PITCH_HZ 2093
#define BEEP_3_ON_LENGTH_MS 500
#define BEEP_3_OFF_LENGTH_MS 500
#define BEEP_3_REPEAT 2
