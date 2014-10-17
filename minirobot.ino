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

// the setup routine runs once when you press reset:
void setup() {                
  // 
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);    
  pinMode(led, OUTPUT);    
  pinMode(led, OUTPUT); 
  pinMode(led, OUTPUT); 
  pinMode(led, OUTPUT); 
  pinMode(led, OUTPUT); 
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  
  run_robot(); //The user's function! Yaaaay!
  
  while(1); //JK, don't run a loop here, just hang when the user's function finishes
}

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

void turnRedLEDOn(void)
void turnRedLEDOff(void)
void turnGreenLEDOn(void)
void turnGreenLEDOff(void)

void setBigLEDColor(const char * color)

void playBeep1(void)
void playBeep2(void)
void playBeep3(void)
void playBeepCustom(int pitch_in_hz, int length)

void printMessage(const char * message)
void printRobotStatus(void)
