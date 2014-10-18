/*
  minirobot
  A framework for basic, small robotics programming
  Fall 2014
  
  Architected by Chris Gerth
  
  

  User Code - this is the file you edit to define the behavior of the robot.
  The function you write here is called once. The robot just goes down the
  "list of instructions" you put here to figure out what to do. 
  
  By <PUT YOUR NAME HERE :D>
  
 */
 
#include "User_Code.h" 

void run_robot(void)
{
  /******************************************/
  /*       START WRITING CODE HERE!!!       */
  
  
  //sample code - do a dance!
  
  turnGreenLEDOn();
  turnRedLEDOn();
  printMessage("Let's Party!"); //ANNOUNCE YOURSELF!
  delay(2000);
  turnGreenLEDOff();
  turnRedLEDOff();
  delay(500);
 
 
  while(1) //DANCE FOREVER!!!
  {
    turnRedLEDOn();
    printRobotStatus();
    delay(1000);
    
    setDirectionRev();
    setSpeedFast();
    turnRedLEDOff();
    printRobotStatus();
    delay(1000);
 
    setDirectionFwd();
    setSpeedMedium();
    turnGreenLEDOn();
    printRobotStatus();
    delay(1000);
    
    if(currentLeftLightSensorValue() > 40.5) //MAKE IT FUNKY!
    {
      turnGreenLEDOff();
      turnRedLEDOn();
      setSpeedMax();
      printRobotStatus();
      delay(1000);
    }
    
    setDirectionSharpRight();
    turnGreenLEDOn();
    printRobotStatus();
    delay(500);
    
  }
  
  
  
  
  
  
  
  
  
  /*        STOP WRITING CODE HERE!!!       */
  /******************************************/
}
