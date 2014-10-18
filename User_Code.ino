/*
  minirobot
  A framework for basic, small robotics programming
  Fall 2014
  
  Architected by Chris Gerth
  
  User Code - this is the file you edit to define the behavior of the robot.
  The function you write here is called once. The robot just goes down the
  "list of instructions" you put here to figure out what to do. 
 */
 
#include "User_Code.h" 

void run_robot(void)
{
  /******************************************/
  /*       START WRITING CODE HERE!!!       */
  
 setDirectionFwd();
 
  while(1)
  {
    setSpeedMax();
    printRobotStatus();
    delay(250);
    
    setSpeedFast();
    printRobotStatus();
    delay(250);
    
    setSpeedMedium();
    printRobotStatus();
    delay(250);
    
    setSpeedSlow();
    printRobotStatus();
    delay(250);
    
    setSpeedStop();
    printRobotStatus();
    delay(250);
    
  }
  
  
  
  
  
  
  
  
  
  /*        STOP WRITING CODE HERE!!!       */
  /******************************************/
}
