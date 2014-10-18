minirobot
=========

Arduino Framework for educational robot for Kaite Birkel

By Chris Gerth - Fall 2014

Robot User API:
--------------

### Motion Functions 

These functions allow you to move your robot from place to place.

* Can control both direction and speed.
* At any time, speed needs to be set to something besides "Stop" for motion to occur
** ie, at the start, you can't just say "setDirectionFwd()" you have to also call "setSpeedFast()" or something like that.
* Default values: Direction = FWD, Speed = STOP
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
setDirectionFwd()
setDirectionRev()
setDirectionLeft()
setDirectionRight()
setDirectionSharpLeft()
setDirectionSharpRight()

setSpeedMax()
setSpeedFast()
setSpeedMedium()
setSpeedSlow()
setSpeedStop()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### Output Functions

These functions allow you to communicate with the outside world. There are three devices currently available:

* serial port (send text to computer) 
* red LED
* green LED.
  
All these are good for debugging (ie, robot should be turning right, so make led green, robot is stopped, so make led red, that sort of thing)

* Change the state of an LED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
turnRedLEDOn()
turnRedLEDOff()
turnGreenLEDOn()
turnGreenLEDOff()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Print an arbitrary message to the serial port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
printMessage("Your Message Here")
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Prints block of debugging data to serial port:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
printRobotStatus()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This will generate the following message on the PC:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Time Elapsed: <number> s
Direction: <dir>
Speed: <speed>
Left Switch: <RELEASED or PRESSED>
Right Switch: <RELEASED or PRESSED>
Left Light Sensor: <percent> %
Right Light Sensor: <percent> %
Red LED: <ON or OFF>
Green LED: <ON or OFF>

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### Input Functions

These functions allow you to capture input from the outside world

* Digital Switch inputs
** Return true if pressed, false if released
** Inputs are debounced for 20ms
*** This means that the value of the switch must remain stable for 20ms before that stable value is reported by this functions
*** This helps prevent incorrect readings due to the mechanical action of the switch or noise on the input
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
boolean currentLeftSwitchValue() 
boolean currentRightSwitchValue()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*Analog Light Sensor inputs
** Normalized to 0.00-100.00% of full range of analog input
** Low-Pass filtered with a cutoff of 17Hz
*** This should reduce noise in sensor reading (especially from 60Hz light sources)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float currentLeftLightSensorValue()
float currentRightLightSensorValue()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### Program Execution Control Functions
  
ex: if/else/while/math/delay()  

Use the built-in functions that the Arduino enviroment provides you with. 
They are documented at http://Arduino.cc/en/Reference/HomePage


### Configuration section 

Mentor will be required to set this up to match hardware specifications. The provided numbers are the defaults.

* IO Pin Numbers
** Define the purpose of each pin on the Arduino
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//DO NOT USE pins 0 or 1, these are needed for serial communication

//It would be best to only use 5 and 6 for the PWM outputs. Things get
// screwey with timers and interrupts if other pins are used. Only 5 and 6
// have been tested to work.
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
#define LEFT_MOTOR_DIR_PIN 9   //direction pins used to invert current direction through motor
#define RIGHT_MOTOR_DIR_PIN 10 // via switching h-bridge configuration.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* I/O Pin State Inversions
** Provided so active-low electrical inputs or outputs map to the proper words/description in the API.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define INVERT_LEFT_SWITCH 0
#define INVERT_RIGHT_SWITCH 0
#define INVERT_LEFT_LIGHT_SENSOR 0
#define INVERT_RIGHT_LIGHT_SENSOR 0
#define INVERT_RED_LED 0
#define INVERT_GREEN_LED 0
#define INVERT_LEFT_MOTOR_DIR 0
#define INVERT_RIGHT_MOTOR_DIR 0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* User Constants
**speed ranges in percent of full motor range - range should be 0 - 100
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define SPEED_MAX 100
#define SPEED_FAST 75
#define SPEED_MEDIUM 50
#define SPEED_SLOW 25
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~