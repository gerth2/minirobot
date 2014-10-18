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
#include "TimerOne.h" #timer-based interrupt library


/******************************************/
//       Private Definitions
/******************************************/
#define FORWARD true //map motor directions to boolean values. just makes things easier to read.
#define REVERSE false
#define PWM_MAX 255 //range of PWM signals. By arduino specs, this must be an unsigned char, which has range 0-255
#define PWM_MIN 0

#define SAMPLE_PERIOD_US 20000 //length of the input sample period in microseconds
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
String current_dir_str = "NONE";
String current_speed_str = "STOP";

//volatile b/c these are changed inside the interrupt routine
volatile unsigned char left_filter_buff_start_idx = 0;
volatile unsigned char right_filter_buff_start_idx = 0;
volatile unsigned char left_sw_buff_start_idx = 0;
volatile unsigned char right_sw_buff_start_idx = 0;

//filter analog inputs with a FIR filter 
//Designed at http://t-filter.appspot.com/fir/index.html
//Assumed passband of 0-30Hz, gain 1
//Stopband: 50-250Hz, gain 0
const float filter_coef[NUM_FILTER_TAPS] = {
                                                 0.040928427477075285,
                                                -0.03114113765147863,
                                                -0.03146978868376737,
                                                -0.032750560783785895,
                                                -0.029754341774297543,
                                                -0.018635003339078193,
                                                0.0020380773131874847,
                                                0.03169197530148918,
                                                0.06719096139954261,
                                                0.10354918750956267,
                                                0.13511794838959662,
                                                0.1565649896617666,
                                                0.164150546780493,
                                                0.1565649896617666,
                                                0.13511794838959662,
                                                0.10354918750956267,
                                                0.06719096139954261,
                                                0.03169197530148918,
                                                0.0020380773131874847,
                                                -0.018635003339078193,
                                                -0.029754341774297543,
                                                -0.032750560783785895,
                                                -0.03146978868376737,
                                                -0.03114113765147863,
                                                0.040928427477075285
                                                                          };

//volatile b/c these are changed inside the interrupt routine
volatile unsigned int left_filter_buff[NUM_FILTER_TAPS];
volatile unsigned int right_filter_buff[NUM_FILTER_TAPS];
volatile boolean left_sw_buff[LEFT_SW_BUFFER_LEN];
volatile boolean right_sw_buff[RIGHT_SW_BUFFER_LEN];

boolean left_sw_debounced_val = false;
boolean right_sw_debounced_val = false;


/******************************************/
//       Arduino Setup Function 
/******************************************/
void setup() {                
  // set pin direction modes so the microprocessor knows to drive or read the pin voltages
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);    
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT); 
  pinMode(LEFT_SWITCH_PIN, INPUT); 
  pinMode(RIGHT_SWITCH_PIN, INPUT); 
  pinMode(RED_LED_PIN, OUTPUT); 
  pinMode(GREEN_LED_PIN, OUTPUT); 
  pinMode(SPEAKER_PIN , OUTPUT); 
  
  Timer1.initialize(SAMPLE_PERIOD_US); //timer1 fires off at a pre-defined rate
  Timer1.attachInterrupt(isr_sample); //set the sample function to run every time timer1 fires off
  
  Serial.begin(115200); //Initalize serial port hardware
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
//       Interrupt Functions
/******************************************/
//samples all the inputs into the approprate buffers every time timer1 overflows
//this is an interrupt routine, so it must be as lightweight as possible
void isr_sample(void)
{
   //write samples into circular buffer.  
   left_filter_buff[left_filter_buff_start_idx++] = analogRead(LEFT_LIGHT_SENSOR_PIN);
   if(left_filter_buff_start_idx == NUM_FILTER_TAPS)
     left_filter_buff_start_idx = 0;
     
   right_filter_buff[right_filter_buff_start_idx++] = analogRead(RIGHT_LIGHT_SENSOR_PIN);
   if(right_filter_buff_start_idx == NUM_FILTER_TAPS)
     right_filter_buff_start_idx = 0;

   left_sw_buff[left_sw_buff_start_idx++] = digitalRead(LEFT_SWITCH_PIN);
   if(left_sw_buff_start_idx == LEFT_SW_BUFFER_LEN)
     left_filter_buff_start_idx = 0;
   
   right_sw_buff[right_sw_buff_start_idx++] = digitalRead(RIGHT_SWITCH_PIN);
   if(right_sw_buff_start_idx == RIGHT_SW_BUFFER_LEN)
     right_filter_buff_start_idx = 0;
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
  return accumulator * 100/1024; 

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
  Serial.print("Left Switch: ");
  Serial.println(CurrentLeftSwitchValue());
  Serial.print("Right Switch: ");
  Serial.println(CurrentRightSwitchValue());
  Serial.print("Left Light Sensor: ");
  Serial.println(CurrentLeftLightSensorVal());
  Serial.print("Right Light Sensor: ");
  Serial.println(CurrentRightLightSensorVal());
  Serial.println("-------------------------------------");

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
   if(val_is_stable)
     left_sw_debounced_val = temp_val;
     
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
   if(val_is_stable)
     right_sw_debounced_val = temp_val;
     
   return right_sw_debounced_val;
  
}
