/*
 * Servo Calibrate
 * 
 * Version 1.0
 * Date: 12/17/20
 * Author: Nikolai Tiong
 * 
 * The purpose of this sketch is to calibrate the individual servos on the robot hand.
 * Setup does an initial open and closing of the hand.
 * 
 * The main loop then closes and opens the hand while printing out the analogRead() for every tick of 
 * the PWM driver. The values printed for an open or close routine can be pasted into a text
 * file, saved as a .csv then imported into a spreadsheet. See accompanying document for more details.
 * 
 * The pulse (x axis) and a finger (y axis) can then be plotted on a scattergraph and a line of best fit used to
 * find the equation of the form currentPosition = analogRead * analogMultiplier - analogShifter.
 * Typical values should around 0.6 for analogMultiplier and -20 for analogShifter.
 * 
 * Use servo_readings to verify the calibration is acceptable.
 * 
 * To Do:
 *    Streamline the process: do the calibration in the sketch itself by sampling fewer values.
 */

#include <Wire.h>
// Adafruit PWM Servo Library
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

// Definitions and global variables
// Values for PWM
#define THUMB  0
#define INDEX  1                                                             
#define MIDDLE 2
#define RING   3
#define PINKY  4

// analogRead() pins
int pinkyPin = A4;  
int ringPin = A3;   
int middlePin = A2;   
int indexPin = A1;    
int thumbPin = A0; 
int currentThumbPos, currentIndexPos, currentMiddlePos, currentRingPos, currentPinkyPos;
int analogThumb, analogIndex, analogMiddle, analogRing, analogPinky;

// These are typical values for the servos to be in for open and closed
// They will need to be adjusted if a value doesn't open or close all the way
// Analog read 
// 290 open
// 770 closed

// PWM driver values
// 150 open
// 450 closed
#define SERVOMIN  150                                                        
#define SERVOMAX  440                                                       
#define USMIN  600                                                           
#define USMAX  2400                                                          
#define SERVO_FREQ 50 

// The values used to convert analogReads to PWM driver values. These are placeholders for the initial
// open and closing of the hand and are within 10% accuracy.
double analogMultiplier = 0.61;
int analogShifter = -22;


// Creating the pwm object that will controll the speed of each robot finger later on
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();                     

void setup() 
{
  Serial.begin(115200);
  pwm.begin();                                                               // start the pwm
  pwm.setOscillatorFrequency(27000000);                                      // with the defaulted frequency
  pwm.setPWMFreq(SERVO_FREQ);                                                // *Analog servos run at ~50 Hz updates 
  delay(1000);                                                                // make sure it is finished initializing before we start giving commands         

  // Setup closes and then opens the hand
  // Read the current position of each servo and calculate roughly what value to send to the PWM driver
  uint16_t pastIndexPos = analogRead(indexPin);
  // move finger to this position
  currentThumbPos = analogRead(thumbPin)*analogMultiplier+analogShifter;
  int thumbTicks = currentThumbPos - SERVOMAX;
  currentIndexPos = analogRead(indexPin)*analogMultiplier+analogShifter;
  int indexTicks = currentIndexPos - SERVOMAX;
  currentMiddlePos = analogRead(middlePin)*analogMultiplier+analogShifter;
  int middleTicks = currentMiddlePos - SERVOMAX;
  currentRingPos = analogRead(ringPin)*analogMultiplier+analogShifter;
  int ringTicks = currentRingPos - SERVOMAX;
  currentPinkyPos = analogRead(pinkyPin)*analogMultiplier+analogShifter;
  int pinkyTicks = currentPinkyPos - SERVOMAX;

  delay(1000);
  // Close all fingers
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
  {
     if (thumbTicks <0){
        int movement = SERVOMAX + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, movement); 
        thumbTicks++;
     }
     if (indexTicks <0){
        int movement = SERVOMAX + indexTicks + 1;
        pwm.setPWM(INDEX, 0, movement); 
        indexTicks++;
     }
     if (middleTicks <0){
        int movement = SERVOMAX + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, movement); 
        middleTicks++;
     }
     if (ringTicks <0){
        int movement = SERVOMAX + ringTicks + 1;
        pwm.setPWM(RING, 0, movement);
        ringTicks++;
     }
     if (pinkyTicks <0){
        int movement = SERVOMAX + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, movement);
        pinkyTicks++;
     }
     delay(3);  
  }
  delay(1000);
                                                                
  // Open the hand
  indexTicks = currentIndexPos - SERVOMIN;
  ringTicks = currentRingPos - SERVOMIN;
  thumbTicks = currentThumbPos - SERVOMIN;
  middleTicks = currentMiddlePos - SERVOMIN;
  pinkyTicks = currentPinkyPos - SERVOMIN;
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) 
  {
     if (thumbTicks > 0){
        int thumbMovement = SERVOMIN + thumbTicks - 1;
        pwm.setPWM(THUMB, 0, thumbMovement);
        thumbTicks--;
     }
     if (indexTicks >0){
        int indexMovement = SERVOMIN + indexTicks - 1;
        pwm.setPWM(INDEX, 0, indexMovement);
        indexTicks--;
     }     
     if (middleTicks > 0){
        int middleMovement = SERVOMIN + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement);
        middleTicks--;
     }      
     if (ringTicks >0){
        int ringMovement = SERVOMIN + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement);
        ringTicks--;
     }
     if (pinkyTicks >0){
        int pinkyMovement = SERVOMIN + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement);
        pinkyTicks--;
     }
     delay(3);  
  }
}


// The main loop does an analogRead(), prints out the position, then closes the hand. For every pulse
// of the PWM driver, each servo is read again with analogRead() and printed on serial output. It then opens
// the hand and does the same thing.

void loop() 
{
  
   currentThumbPos = analogRead(thumbPin);
   currentIndexPos = analogRead(indexPin);
   currentMiddlePos = analogRead(middlePin);
   currentRingPos = analogRead(ringPin);
   currentPinkyPos = analogRead(pinkyPin);
   Serial.print(currentThumbPos);
   Serial.print(",");
   Serial.print(currentIndexPos);
   Serial.print(",");
   Serial.print(currentMiddlePos);
   Serial.print(",");
   Serial.print(currentRingPos);
   Serial.print(",");
   Serial.println(currentPinkyPos);
   delay(1000);
   // Close all fingers
   int thumbTicks = SERVOMIN - SERVOMAX;
   int indexTicks = SERVOMIN - SERVOMAX;
   int middleTicks = SERVOMIN - SERVOMAX;
   int ringTicks = SERVOMIN - SERVOMAX;
   int pinkyTicks = SERVOMIN - SERVOMAX;
   for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)      // for each step in the servo range 
  {
     if (thumbTicks <0){
        int movement = SERVOMAX + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, movement);
        thumbTicks++;
     }
     if (indexTicks <0){
        int movement = SERVOMAX + indexTicks + 1;
        pwm.setPWM(INDEX, 0, movement);
        indexTicks++;
     }
     if (middleTicks <0){
        int movement = SERVOMAX + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, movement);
        middleTicks++;
     }
     if (ringTicks <0){
        int movement = SERVOMAX + ringTicks + 1;
        pwm.setPWM(RING, 0, movement);
        ringTicks++;
     }
     if (pinkyTicks <0){
        int movement = SERVOMAX + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, movement);
        pinkyTicks++;
     }
     analogThumb = analogRead(thumbPin);
     analogIndex = analogRead(indexPin);
     analogMiddle = analogRead(middlePin);
     analogRing = analogRead(ringPin);
     analogPinky = analogRead(pinkyPin);
     Serial.print(pulselen);
     Serial.print(",");
     Serial.print(analogThumb);
     Serial.print(",");
     Serial.print(analogIndex);
     Serial.print(",");
     Serial.print(analogMiddle);
     Serial.print(",");
     Serial.print(analogRing);
     Serial.print(",");
     Serial.println(analogPinky);
     delay(3);  
  }
  delay(5000);                                                              // this delay adjusts the pause between fingers
  // Open all fingers
  indexTicks = SERVOMAX - SERVOMIN;
  ringTicks = SERVOMAX - SERVOMIN;
  thumbTicks = SERVOMAX - SERVOMIN;
  middleTicks = SERVOMAX - SERVOMIN;
  pinkyTicks = SERVOMAX - SERVOMIN;
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
     if (thumbTicks > 0){
        int thumbMovement = SERVOMIN + thumbTicks - 1;
        pwm.setPWM(THUMB, 0, thumbMovement); 
        thumbTicks--;
     }
     if (indexTicks >0){
        int indexMovement = SERVOMIN + indexTicks - 1;
        pwm.setPWM(INDEX, 0, indexMovement); 
        indexTicks--;
     }     
     if (middleTicks > 0){
        int middleMovement = SERVOMIN + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement);
        middleTicks--;
     }      
     if (ringTicks >0){
        int ringMovement = SERVOMIN + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement);
        ringTicks--;
     }
     if (pinkyTicks >0){
        int pinkyMovement = SERVOMIN + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement);
        pinkyTicks--;
     }
     analogThumb = analogRead(thumbPin);
     analogIndex = analogRead(indexPin);
     analogMiddle = analogRead(middlePin);
     analogRing = analogRead(ringPin);
     analogPinky = analogRead(pinkyPin);
     Serial.print(pulselen);
     Serial.print(",");
     Serial.print(analogThumb);
     Serial.print(",");
     Serial.print(analogIndex);
     Serial.print(",");
     Serial.print(analogMiddle);
     Serial.print(",");
     Serial.print(analogRing);
     Serial.print(",");
     Serial.println(analogPinky);
     delay(3);  
  }
   
   delay(5000);
}
