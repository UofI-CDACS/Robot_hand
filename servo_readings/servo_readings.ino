/*
 * Servo Readings
 * 
 * Version 1.0
 * Date 12/17/20
 * Author: Nikolai Tiong
 * 
 * The purpose of this sketch is to complement Servo Calibrate by printing out the calibrated
 * positions of each finger of the robot hand.
 * 
 * It is identical to Servo Calibrate apart from having individual multipliers and shifts
 * for each finger.
 * 
 * Adjust each value according to what the line of best fit is and use the same process as in
 * Servo Calibrate: copy the serial output for an open/close routine and import into a .csv
 * Open in a spreadsheet program and plot pulse against each finger position. 
 * Verify that the finger position overlays the pulse. The ideal result is that the plots 
 * are indistinguishable.
 * 
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

// Change these to the calibrated values for each servo
double analogThumbMultiplier = 0.6;
int analogThumbShifter = -22;
double analogIndexMultiplier = 0.59;
int analogIndexShifter = -20;
double analogMiddleMultiplier = 0.6;
int analogMiddleShifter = -22;
double analogRingMultiplier = 0.59;
int analogRingShifter = -21;
double analogPinkyMultiplier = 0.61;
int analogPinkyShifter = -21;

// Creating the pwm object that will controll the speed of each robot finger later on
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  

void setup() 
{
  Serial.begin(115200);
  pwm.begin();                                                               // start the pwm
  pwm.setOscillatorFrequency(27000000);                                      // with the defaulted frequency
  pwm.setPWMFreq(SERVO_FREQ);                                                // *Analog servos run at ~50 Hz updates 
  delay(1000);                                                                // make sure it is finished initializing before we start giving commands         

  currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
  int thumbTicks = currentThumbPos - SERVOMAX;
  currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
  int indexTicks = currentIndexPos - SERVOMAX;
  currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
  int middleTicks = currentMiddlePos - SERVOMAX;
  currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
  int ringTicks = currentRingPos - SERVOMAX;
  currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
  int pinkyTicks = currentPinkyPos - SERVOMAX;

  delay(1000);
  // Close all fingers
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
     if (thumbTicks <0){
        int movement = SERVOMAX + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, movement); 
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks++;
     }
     if (indexTicks <0){
        int movement = SERVOMAX + indexTicks + 1;
        pwm.setPWM(INDEX, 0, movement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks++;
     }
     if (middleTicks <0){
        int movement = SERVOMAX + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, movement); 
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks++;
     }
     if (ringTicks <0){
        int movement = SERVOMAX + ringTicks + 1;
        pwm.setPWM(RING, 0, movement); 
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks++;
     }
     if (pinkyTicks <0){
        int movement = SERVOMAX + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, movement); 
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks++;
     }
     delay(3);  
  }
  delay(1000);                                                             
  // Open all fingers
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
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks--;
     }
     if (indexTicks >0){
        int indexMovement = SERVOMIN + indexTicks - 1;
        pwm.setPWM(INDEX, 0, indexMovement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks--;
     }     
     if (middleTicks > 0){
        int middleMovement = SERVOMIN + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement); 
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks--;
     }      
     if (ringTicks >0){
        int ringMovement = SERVOMIN + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement); 
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks--;
     }
     if (pinkyTicks >0){
        int pinkyMovement = SERVOMIN + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement); 
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks--;
     }
     delay(3);  
  }
}


void loop() 
{
   currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
   currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
   currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
   currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
   currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
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
   int thumbTicks = currentThumbPos - SERVOMAX;
   int indexTicks = currentIndexPos - SERVOMAX;
   int middleTicks = currentMiddlePos - SERVOMAX;
   int ringTicks = currentRingPos - SERVOMAX;
   int pinkyTicks = currentPinkyPos - SERVOMAX;
   for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)     
  {
     if (thumbTicks <0){
        int movement = SERVOMAX + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, movement); 
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks++;
     }
     if (indexTicks <0){
        int movement = SERVOMAX + indexTicks + 1;
        pwm.setPWM(INDEX, 0, movement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks++;
     }
     if (middleTicks <0){
        int movement = SERVOMAX + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, movement); 
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks++;
     }
     if (ringTicks <0){
        int movement = SERVOMAX + ringTicks + 1;
        pwm.setPWM(RING, 0, movement); 
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks++;
     }
     if (pinkyTicks <0){
        int movement = SERVOMAX + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, movement); 
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks++;
     }
     Serial.print(pulselen);
     Serial.print(",");
     Serial.print(currentThumbPos);
     Serial.print(",");
     Serial.print(currentIndexPos);
     Serial.print(",");
     Serial.print(currentMiddlePos);
     Serial.print(",");
     Serial.print(currentRingPos);
     Serial.print(",");
     Serial.println(currentPinkyPos);
     delay(3);  
  }
  delay(5000);                                                            
  // Open all fingers
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
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks--;
     }
     if (indexTicks >0){
        int indexMovement = SERVOMIN + indexTicks - 1;
        pwm.setPWM(INDEX, 0, indexMovement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks--;
     }     
     if (middleTicks > 0){
        int middleMovement = SERVOMIN + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement); 
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks--;
     }      
     if (ringTicks >0){
        int ringMovement = SERVOMIN + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement); 
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks--;
     }
     if (pinkyTicks >0){
        int pinkyMovement = SERVOMIN + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement); 
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks--;
     }
     Serial.print(pulselen);
     Serial.print(",");
     Serial.print(currentThumbPos);
     Serial.print(",");
     Serial.print(currentIndexPos);
     Serial.print(",");
     Serial.print(currentMiddlePos);
     Serial.print(",");
     Serial.print(currentRingPos);
     Serial.print(",");
     Serial.println(currentPinkyPos);
     delay(3);  
  }
   
   delay(5000);
}
