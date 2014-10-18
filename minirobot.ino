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
#include "TimerOne.h"


/******************************************/
//       Private Definitions
/******************************************/
#define FORWARD true //map motor directions to boolean values. just makes things easier to read.
#define REVERSE false
#define PWM_MAX 255 //range of PWM signals. By arduino specs, this must be an unsigned char, which has range 0-255
#define PWM_MIN 0

#define SAMPLE_PERIOD_US 2000    //length of the input sample period in microseconds
                                 //2000us yeilds a sample rate of 500Hz

#define NUM_FILTER_TAPS 25   //buffer lengths
#define LEFT_SW_BUFFER_LEN 10
#define RIGHT_SW_BUFFER_LEN 10


/******************************************/
//       Private, Internal data 
/******************************************/
float current_speed_pct = 0;
boolean current_left_motor_dir = FORWARD;
boolean current_right_motor_dir = FORWARD;

//Strings to hold kid-friendly descriptions of what's going on
String current_dir_str = "FWD";
String current_speed_str = "STOP";
String current_left_sw_str = "RELEASED";
String current_right_sw_str = "RELEASED";
String current_red_led_state_str = "OFF";
String current_green_led_state_str = "OFF";

//volatile b/c these are changed inside the interrupt routine
volatile unsigned char left_filter_buff_start_idx = 0;
volatile unsigned char right_filter_buff_start_idx = 0;
volatile unsigned char left_sw_buff_start_idx = 0;
volatile unsigned char right_sw_buff_start_idx = 0;

//filter analog inputs with a FIR filter 
//Designed at http://t-filter.appspot.com/fir/index.html
//Assumed passband of 0-17Hz, gain 1
//Transition band 17-20Hz
//Stopband: 20-250Hz, gain 0
//Actual Passband Ripple: 12.10 dB
//Actual Stopband Ripple: -20.02 dB
//Assumed 25 taps and 500Hz sample frequency
const float filter_coef[NUM_FILTER_TAPS] = {
                                                0.0604206028416727,
                                                0.022765286202108867,
                                                0.026559407357829072,
                                                0.030298526984037823,
                                                0.0339463305236097,
                                                0.0374149200448067,
                                                0.04063324687425954,
                                                0.0435353609068285,
                                                0.04602927926463161,
                                                0.048021926552339445,
                                                0.04948579543591529,
                                                0.05048020312982948,
                                                0.05067537613278131,
                                                0.05048020312982948,
                                                0.04948579543591529,
                                                0.048021926552339445,
                                                0.04602927926463161,
                                                0.0435353609068285,
                                                0.04063324687425954,
                                                0.0374149200448067,
                                                0.0339463305236097,
                                                0.030298526984037823,
                                                0.026559407357829072,
                                                0.022765286202108867,
                                                0.0604206028416727
                                                                          };

//volatile b/c these are changed inside the interrupt routine
volatile unsigned int left_filter_buff[NUM_FILTER_TAPS];
volatile unsigned int right_filter_buff[NUM_FILTER_TAPS];
volatile boolean left_sw_buff[LEFT_SW_BUFFER_LEN];
volatile boolean right_sw_buff[RIGHT_SW_BUFFER_LEN];

//switch values (debounced)
boolean left_sw_debounced_val = false;
boolean right_sw_debounced_val = false;


/******************************************/
//       Arduino Setup Function 
/******************************************/
void setup() {   
  //Every time user hits the "Reset" button, the code re-initializes all the global variables (above)
  //and starts running code here.
  
  Serial.begin(115200); //Initalize serial port hardware. Must be done first so we can talk to the PC
  
  Serial.println("Started initalizing..."); 
  
  // set pin direction modes so the microprocessor knows to drive or read the pin voltages
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);    
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT); 
  pinMode(LEFT_SWITCH_PIN, INPUT); 
  pinMode(RIGHT_SWITCH_PIN, INPUT); 
  pinMode(RED_LED_PIN, OUTPUT); 
  pinMode(GREEN_LED_PIN, OUTPUT); 
  
  //initalize outputs to off state
  turnGreenLEDOff();
  turnRedLEDOff();
  
  //Set up input sampling and start sampling.
  Timer1.initialize(SAMPLE_PERIOD_US); //set Timer1 to trigger an interrupt every SAMPLE_PERIOD_US microseconds
  Timer1.attachInterrupt(isr_sample); //Set up the "isr_sample()" function to be called every time Timer1 triggers an interrupt
                                      //This will also enable interrupts to start input sampling at a regular rate.
  Serial.println("Done initalizing!"); 
  
}

/******************************************/
//       Arduino Loop Function 
/******************************************/
// This is the arduino-required function for non-setup things.
// We won't actually use it as a loop.
void loop() {
  //After the setup() function has completed, the arduino starts running code here.
  
  //pause for 1 second (1000 ms) before starting
  delay(1000);
  
  //run user's function
  Serial.println("Starting User Function..."); 
  
  run_robot(); //The user's function! Yaaaay!
  
  Serial.println("Finished User Function!"); 
  
  //kill the motors in case the student forgot to
  set_motor_vals(FORWARD, 0.0, FORWARD, 0.0);

  //pause briefly at the end
  delay(1000);
  
  while(1); //JK, don't run a loop here, just hang when the user's function finishes
}

/******************************************/
//       Interrupt Functions
/******************************************/
//samples all the inputs into the approprate buffers every time timer1 overflows
//this is an interrupt routine, so it must be as lightweight as possible
void isr_sample(void)
{
   //Sample each of the inputs, and write the result into a circular buffer 
   left_filter_buff[left_filter_buff_start_idx++] = analogRead(LEFT_LIGHT_SENSOR_PIN);
   if(left_filter_buff_start_idx == NUM_FILTER_TAPS)
     left_filter_buff_start_idx = 0;
     
   right_filter_buff[right_filter_buff_start_idx++] = analogRead(RIGHT_LIGHT_SENSOR_PIN);
   if(right_filter_buff_start_idx == NUM_FILTER_TAPS)
     right_filter_buff_start_idx = 0;

   left_sw_buff[left_sw_buff_start_idx++] = digitalRead(LEFT_SWITCH_PIN);
   if(left_sw_buff_start_idx == LEFT_SW_BUFFER_LEN)
     left_sw_buff_start_idx = 0;
   
   right_sw_buff[right_sw_buff_start_idx++] = digitalRead(RIGHT_SWITCH_PIN);
   if(right_sw_buff_start_idx == RIGHT_SW_BUFFER_LEN)
     right_sw_buff_start_idx = 0;
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

float get_filtered_and_scaled_analog_sensor_val(volatile unsigned int * buffer, volatile unsigned char buf_start_index)
{
  unsigned char i;
  float accumulator = 0;
  //perform FIR filter operation with circular buffer and pre-computed filter taps
  for(i = 0; i < NUM_FILTER_TAPS; i++)
  {
    accumulator += (float)buffer[(i + buf_start_index) % NUM_FILTER_TAPS] * filter_coef[i];
  }
  
  //scale from [0,1024] to a percent and return the value
  return constrain((accumulator * 100/1024), 0.0, 100.0);

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
  current_red_led_state_str = "ON";
  if(INVERT_RED_LED)
    digitalWrite(RED_LED_PIN, LOW);
  else
    digitalWrite(RED_LED_PIN, HIGH);
}
void turnRedLEDOff(void)
{
  current_red_led_state_str = "OFF";
  if(INVERT_RED_LED)
    digitalWrite(RED_LED_PIN, HIGH);
  else
    digitalWrite(RED_LED_PIN, LOW);
}
void turnGreenLEDOn(void)
{
  current_green_led_state_str = "ON";
  if(INVERT_GREEN_LED)
    digitalWrite(GREEN_LED_PIN, LOW);
  else
    digitalWrite(GREEN_LED_PIN, HIGH);
}
void turnGreenLEDOff(void)
{
  current_green_led_state_str = "OFF";
  if(INVERT_GREEN_LED)
    digitalWrite(GREEN_LED_PIN, HIGH);
  else
    digitalWrite(GREEN_LED_PIN, LOW);
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
  Serial.println(" s");
  Serial.print("Direction: ");
  Serial.println(current_dir_str);
  Serial.print("Speed: ");
  Serial.println(current_speed_str);
  Serial.print("Left Switch: ");
  CurrentLeftSwitchValue();
  Serial.println(current_left_sw_str);
  Serial.print("Right Switch: ");
  CurrentRightSwitchValue();
  Serial.println(current_right_sw_str);
  Serial.print("Left Light Sensor: ");
  Serial.print(CurrentLeftLightSensorVal());
  Serial.println(" %");
  Serial.print("Right Light Sensor: ");
  Serial.print(CurrentRightLightSensorVal());
  Serial.println(" %");
  Serial.print("Red LED: ");
  Serial.println(current_red_led_state_str);
  Serial.print("Green LED: ");
  Serial.println(current_green_led_state_str);
  Serial.println("-------------------------------------");
  Serial.println("");

}

float CurrentLeftLightSensorVal(void)
{
  return  get_filtered_and_scaled_analog_sensor_val(left_filter_buff, left_filter_buff_start_idx);
}
float CurrentRightLightSensorVal(void)
{
  return  get_filtered_and_scaled_analog_sensor_val(right_filter_buff, right_filter_buff_start_idx);
}

boolean CurrentLeftSwitchValue(void) 
{
   //debounce value based on what is in theh buffer
   int i;
   boolean temp_val = left_sw_buff[0]; //initialize to see what the first value in the buffer is
   boolean val_is_stable = true;
    
   //see if all the values in the buffer are
   for(i = 1; i < LEFT_SW_BUFFER_LEN; i++)
   {
     if(left_sw_buff[i] != temp_val) //if any of them are not the same,
     {
       val_is_stable = false; //the signal is not yet stable. Don't change the debunced value
       break;
     }
   }
   
   //if the buffer is stable, set the debounced val to the buffer's value (stable implies all buffer elements are equal)
   //apply proper inversion
   if(val_is_stable)
   {
     if(INVERT_LEFT_SWITCH == 1)
       left_sw_debounced_val = !temp_val;
     else
       left_sw_debounced_val = temp_val;
   }
   
   //provide a kid-friendly description of what is going on
   if(left_sw_debounced_val)
     current_left_sw_str = "PRESSED";
   else
     current_left_sw_str = "RELEASED";
     
   return left_sw_debounced_val;
}
boolean CurrentRightSwitchValue(void)
{
   //debounce value based on what is in theh buffer
   int i;
   boolean temp_val = right_sw_buff[0]; //initialize to see what the first value in the buffer is
   boolean val_is_stable = true;
    
   //see if all the values in the buffer are
   for(i = 1; i < RIGHT_SW_BUFFER_LEN; i++)
   {
     if(right_sw_buff[i] != temp_val) //if any of them are not the same,
     {
       val_is_stable = false; //the signal is not yet stable. Don't change the debunced value
       break;
     }
   }
   
   //if the buffer is stable, set the debounced val to the buffer's value (stable implies all buffer elements are equal)
   //apply proper inversion
   if(val_is_stable)
   {
     if(INVERT_RIGHT_SWITCH == 1)
       right_sw_debounced_val = !temp_val;
     else
       right_sw_debounced_val = temp_val;
   }
   
   //provide a kid-friendly description of what is going on
   if(right_sw_debounced_val)
     current_right_sw_str = "PRESSED";
   else
     current_right_sw_str = "RELEASED";
     
   return right_sw_debounced_val;
  
}
