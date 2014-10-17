/*
  minirobot
  A framework for basic, small robotics programming
  Fall 2014
  
  Architected by Chris Gerth
  
  Overarching Idea - provide a simplified interface to drive a two-motor robot with 
  basic io for teaching purposes
  
  minirobot - main code file. This should never have to be edited
  It contains all the functions that do all the things in the background
  If you didn't write it, it happens here.
 
  */
  
/******************************************/
//       Included Constants 
/******************************************/
#include "config.h"

/******************************************/
//       Private Definitions
/******************************************/
#define FORWARD true //map motor directions to boolean values. just makes things easier to read.
#define REVERSE false
#define PWM_MAX 255 //range of PWM signals. By arduino specs, this must be an unsigned char, which has range 0-255
#define PWM_MIN 0


/******************************************/
//       User function declaration 
/******************************************/
void run_robot(void);

/******************************************/
//       Private, Internal data 
/******************************************/
unsigned char current_speed_pwm = 0;
boolean current_left_motor_dir = FORWARD;
boolean current_right_motor_dir = FORWARD;

/******************************************/
//       Arduino Setup Function 
/******************************************/
void setup() {                
  // set pin direction modes so the microprocessor knows 
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);    
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT); 
  pinMode(LEFT_SWITCH_PIN, INPUT); 
  pinMode(RIGHT_SWITCH_PIN, INPUT); 
  pinMode(RED_LED_PIN, OUTPUT); 
  pinMode(GREEN_LED_PIN, OUTPUT); 
  pinMode(SPEAKER_PIN , OUTPUT); 
}

/******************************************/
//       Arduino Loop Function 
/******************************************/
// This is the arduino-required function for non-setup things.
// We won't actually use it as a loop.
void loop() {

  //play tone sequence to indicate robot is starting
  tone(SPEAKER_PIN, NOTE_F6, 750);
  tone(SPEAKER_PIN, NOTE_C7, 1500);    
  
  //pause briefly before starting
  delay(1000); 
  
  //run user's function
  run_robot(); //The user's function! Yaaaay!
  
  //pause briefly at the end
  delay(1000); 
  
  //play tone sequence to indicate robot is finished
  tone(SPEAKER_PIN, NOTE_C7, 750);
  tone(SPEAKER_PIN, NOTE_F6, 1500);    
  
  while(1); //JK, don't run a loop here, just hang when the user's function finishes
}


/******************************************/
//       Private Helper functions 
/******************************************/
//convert a percentage (0.0->100.0) to a pwm value (PWM_MIN->PWM_MAX)
//ensure proper data typecasting
unsigned char convert_pct_to_pwm(float percent)
{
   return PWM_MIN + (unsigned char)((float)PWM_MAX*percent/100.0);
}

//set motor speed and directions with approprate inversions
void set_motor_vals(boolean right_dir, unsigned char right_speed, boolean left_dir, unsigned char left_speed)
{
  
  
}


//User API function implementations
void setDirectionFwd(void)
{
  
}
void setDirectionRev(void)
{
  
}
void setDirectionLeft(void)
{
  
}
void setDirectionRight(void)
{
  
}
void setDirectionSharpLeft(void)
{
  
}
void setDirectionSharpRight(void)
{
  
}

void setSpeedMax(void)
{
  
}
void setSpeedFast(void)
{
  
}
void setSpeedMedium(void)
{
  
}
void setSpeedSlow(void)
{
  
}
void setSpeedStop(void)
{
  
}

void turnRedLEDOn(void)
{
  
}
void turnRedLEDOff(void)
{
  
}
void turnGreenLEDOn(void)
{
  
}
void turnGreenLEDOff(void)
{
  
}

void setBigLEDColor(const char * color)
{
  
}

void playBeep1(void)
{
  
}
void playBeep2(void)
{
  
}
void playBeep3(void)
{
  
}
void playBeepCustom(int pitch_in_hz, int length)
{
  
}

void printMessage(const char * message)
{
  
}
void printRobotStatus(void)
{
  
}
