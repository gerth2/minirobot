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
#include "User_Code.h"


/******************************************/
//       Private Definitions
/******************************************/
#define FORWARD true //map motor directions to boolean values. just makes things easier to read.
#define REVERSE false
#define PWM_MAX 255 //range of PWM signals. By arduino specs, this must be an unsigned char, which has range 0-255
#define PWM_MIN 0


/******************************************/
//       Private, Internal data 
/******************************************/
float current_speed_pct = 0;
boolean current_left_motor_dir = FORWARD;
boolean current_right_motor_dir = FORWARD;
String current_dir_str = "NONE";
String current_speed_str = "STOP";

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
  
  Serial.begin(115200);
}

/******************************************/
//       Arduino Loop Function 
/******************************************/
// This is the arduino-required function for non-setup things.
// We won't actually use it as a loop.
void loop() {

  //play tone sequence to indicate robot is starting
  tone(SPEAKER_PIN, 1397 , 750);
  tone(SPEAKER_PIN, 2093, 1500);    
  
  //pause briefly before starting
  delay(1000); 
  
  //run user's function
  run_robot(); //The user's function! Yaaaay!
  
  //kill the motors in case the student forgot to
  set_motor_vals(FORWARD, 0.0, FORWARD, 0.0);

  //pause briefly at the end
  delay(1000); 
  
  //play tone sequence to indicate robot is finished
  tone(SPEAKER_PIN, 2093, 750);
  tone(SPEAKER_PIN, 1397 , 1500);    
  
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
void set_motor_vals(boolean right_dir, float right_speed, boolean left_dir, float left_speed)
{
  //set motor speeds with approprate type conversion
  analogWrite(RIGHT_MOTOR_PWM_PIN, convert_pct_to_pwm(right_speed));
  analogWrite(LEFT_MOTOR_PWM_PIN, convert_pct_to_pwm(left_speed));
  
  //set motor directions based on inversion 
  if(INVERT_RIGHT_MOTOR_DIR)
    digitalWrite(RIGHT_MOTOR_DIR_PIN, !right_dir);
  else
    digitalWrite(RIGHT_MOTOR_DIR_PIN, right_dir);
  
  if(INVERT_LEFT_MOTOR_DIR)
    digitalWrite(LEFT_MOTOR_DIR_PIN, !left_dir);
  else
    digitalWrite(LEFT_MOTOR_DIR_PIN, left_dir); 
}



/******************************************/
//       User API Implementation
/******************************************/
void setDirectionFwd(void)
{
  current_right_motor_dir = FORWARD;
  current_left_motor_dir = FORWARD;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct);
  current_dir_str = "FWD"; 
}
void setDirectionRev(void)
{
  current_right_motor_dir = REVERSE;
  current_left_motor_dir = REVERSE;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_dir_str = "REV"; 
}
void setDirectionLeft(void)
{
  current_right_motor_dir = FORWARD;
  current_left_motor_dir = FORWARD;  
  set_motor_vals(current_right_motor_dir, 0.0, current_left_motor_dir, current_speed_pct); 
  current_dir_str = "LEFT"; 
}
void setDirectionRight(void)
{
  current_right_motor_dir = FORWARD;
  current_left_motor_dir = FORWARD;  
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, 0.0); 
  current_dir_str = "RIGHT"; 
}
void setDirectionSharpLeft(void)
{
  current_right_motor_dir = FORWARD;
  current_left_motor_dir = REVERSE;  
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_dir_str = "SharpLEFT"; 
}
void setDirectionSharpRight(void)
{
  current_right_motor_dir = REVERSE;
  current_left_motor_dir = FORWARD;    
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_dir_str = "SharpRIGHT"; 
}

void setSpeedMax(void)
{
  current_speed_pct = SPEED_MAX;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_speed_str = "MAX";
}
void setSpeedFast(void)
{
  current_speed_pct = SPEED_FAST;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_speed_str = "FAST";
}
void setSpeedMedium(void)
{
  current_speed_pct = SPEED_MEDIUM;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_speed_str = "MEDIUM";
}
void setSpeedSlow(void)
{
  current_speed_pct = SPEED_SLOW;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_speed_str = "SLOW";
}
void setSpeedStop(void)
{
  current_speed_pct = 0;
  set_motor_vals(current_right_motor_dir, current_speed_pct, current_left_motor_dir, current_speed_pct); 
  current_speed_str = "STOP";
}

void turnRedLEDOn(void)
{
  if(INVERT_RED_LED)
    digitalWrite(RED_LED_PIN, LOW);
  else
    digitalWrite(RED_LED_PIN, HIGH);
}
void turnRedLEDOff(void)
{
  if(INVERT_RED_LED)
    digitalWrite(RED_LED_PIN, HIGH);
  else
    digitalWrite(RED_LED_PIN, LOW);
}
void turnGreenLEDOn(void)
{
  if(INVERT_GREEN_LED)
    digitalWrite(GREEN_LED_PIN, LOW);
  else
    digitalWrite(GREEN_LED_PIN, HIGH);
}
void turnGreenLEDOff(void)
{
  if(INVERT_GREEN_LED)
    digitalWrite(GREEN_LED_PIN, HIGH);
  else
    digitalWrite(GREEN_LED_PIN, LOW);
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
void playBeepCustom(int pitch_in_hz, int length_in_ms)
{
  tone(SPEAKER_PIN, pitch_in_hz, length_in_ms);
}

void printMessage(const char * message)
{
  Serial.println(message); 
}
void printRobotStatus(void)
{
  Serial.println("-------------------------------------");
  Serial.print("Runtime: ");
  Serial.print((float)millis()/1000.0);
  Serial.print("Direction: ");
  Serial.println(current_dir_str);
  Serial.print("Speed: ");
  Serial.println(current_speed_str);
  Serial.println("-------------------------------------");
  
  
  
}
