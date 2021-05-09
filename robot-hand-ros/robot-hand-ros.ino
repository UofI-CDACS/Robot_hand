/******************************************************************************
 * Robot Hand Ros
 * 
 * Version 1.0
 * Date: 12/17/20
 * Author: Nikolai Tiong
 * 
 * Robot Hand Ros is the sketch that runs on the Adafruit Feather M0 attached 
 * to the robot hand. It has an Adafruit ethernet shield mounted on top and is
 * connected to an Adafruit PWM controller via the I2C ports. Analog pins 0-4
 * are connected to the servos controlling the fingers and are set to thumb,
 * index, middle, ring, pinky respectively. The ethernet shield uses SPI.
 * 
 * This program is in 2 sections: controlling the servo motors and the ROS
 * nodes sending and receiving messages about the servos.
 * There are two ROS nodes, a subscriber - robot_hand_command which receives
 * messages from the Python script robot_hand_command.py. It receives a
 * message of the form "aaa bbb ccc ddd eee" which correspond to 3 digit
 * numbers for each finger. These numbers tell where to move the servos.
 * The current operating range is 150-440 where 150 is open and 440 is closed.
 * If the servos are replaced with different models, these numbers may change
 * or even be reversed.
 * 
 * The second ROS node is a subscriber - robot_hand_position which constructs
 * a message out of the current position of each servo and sends it to the 
 * Python script robot_hand_position.py. 
 * 
 * Controlling the servos is done by first making sure the values sent are
 * in the valid range: if a number is below 150, it will be set to 150, if
 * above 440, it's set to 440. Going beyond these values will probably damage
 * the servos.
 *
 * Once that is done the direction to move is calculated based on where the 
 * servos are at currently. A positive value means that finger needs to open
 * while a negative closes the finger. This value is used to also determine
 * how many PWM pulses are needed to get to the destination. All 5 fingers are
 * moved in one loop to ensure all move at the same time. There is a delay() 
 * in between each looping to control the speed of movement. Currently this is
 * fixed but should be modified in the future to allow the speed to be
 * controlled in ROS.
 * 
 * Upon startup, the hand goes through a close and open routine. 
 * The sketch doesn't use Serial output when connected to the power box on the
 * FANUC arm. There are commented out serial statements that can be used when 
 * connecting the hand directly to a PC for diagnostics or development on this
 * sketch.
 *
 * How to use:
 *    Make sure the robot hand is powered on and connected to the network that
 *    the system running ROS is on - ping the IP address of the hand.
 *    Open a terminal window on the system running ROS
 *    Type "roscore"
 *    Open another terminal window and type:
 *       "rosrun rosserial_python serial_node.py tcp"
 *    Open another terminal window and navigate to the folder this script is in
 *    and type "python robot_hand_command.py"
 *    The topic robot_hand_command will be published and messages should appear
 *    on the rosrun window.
 *    Open another terminal window, navigate to the folder containing
 *    robot_hand_position.py and type "python robot_hand_position"
 *    The position of the fingers will be reported here after every command
 *    is finished processing movement.
 * 
 * To Do:
 *    Modify the message so that a speed parameter can be sent for each finger.
 *    This will require modifying the finger movement loop as well.
 *    
 *    Eliminate the repetitiveness of the program by writing a separate 
 *    function for the movement and sending the finger pin and direction.
 * 
 *****************************************************************************/


/* Hand Stuff
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

// Creating the pwm object that will controll the speed of each robot finger later on
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();                     

// analogRead() pins
int pinkyPin = A4;  
int ringPin = A3;   
int middlePin = A2;   
int indexPin = A1;    
int thumbPin = A0; 
int currentThumbPos, currentIndexPos, currentMiddlePos, currentRingPos, currentPinkyPos;
int analogThumb, analogIndex, analogMiddle, analogRing, analogPinky;

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

// These are typical values for the servos to be in for open and closed
// They will need to be adjusted if a value doesn't open or close all the way
// Analog read 
// 290 open
// 770 closed

// PWM driver values
// 150 open
// 450 closed
#define SERVOMIN  150                                                        // *This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  440                                                        // *This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600                                                           // *This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400                                                          // *This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50                                                        // *Analog servos run at ~50 Hz updates


 /* Ros Stuff
  *  
  */
#include <Ethernet.h>


// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/String.h>

// Set the shield settings
// The ethernet shield doesn't typically have a built-in MAC address so set it
// to whatever you want, as long as it doesn't share one with another device
// on the network.
byte mac[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
IPAddress ip(129, 101, 98, 241);

// Server IP address - ROS node 
IPAddress server(129,101,98,243);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("robot_hand_position", &str_msg);

bool publishPosition = false;

void robotCommandParser(const std_msgs::String& msg)                                     // This function will parse the messages published to the /robot_hand_command topic and send any key commands on to the hand
{
  char newTransmission[21] = "";                                                          // create a new transmission string whenever a mew message comes in
  strncpy(newTransmission, msg.data, 21);                                                 // copy the incoming message data to this new string
  nh.loginfo("new message is:");                                                         // Serial monitor debugging must be formatted like this, Serial.print will not work when running rosserial
  nh.loginfo(msg.data); 
  //Serial.println(msg.data);
  long int fingers[5];
  long int thumb, index, middle, ring, pinky;
  char * pEnd;
  fingers[0] = strtol (newTransmission, &pEnd, 10);
  fingers[1] = strtol (pEnd, &pEnd, 10);
  fingers[2] = strtol (pEnd, &pEnd, 10);
  fingers[3] = strtol (pEnd, &pEnd, 10);
  fingers[4] = strtol (pEnd, NULL, 10);
  // Set values out of movement range to maximum or minimum value
  for (int i = 0; i< 5; i++){
     if (fingers[i] > SERVOMAX){
        fingers[i] = SERVOMAX;
//        Serial.print("Changed ");
//        Serial.print(fingers[i]);
//        Serial.println(i);
     }
     else if (fingers[i] < SERVOMIN){
        fingers[i] = SERVOMIN;
//        Serial.print("Changed ");
//        Serial.print(fingers[i]);
//        Serial.println(i);
     }
  }

  currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
  int thumbTicks = currentThumbPos - fingers[0];
  currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
  int indexTicks = currentIndexPos - fingers[1];
  currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
  int middleTicks = currentMiddlePos - fingers[2];
  currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
  int ringTicks = currentRingPos - fingers[3];
  currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
  int pinkyTicks = currentPinkyPos - fingers[4];
//  Serial.print("indexTicks ");
//  Serial.println(indexTicks);
  for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--){
     // Thumb
     if (thumbTicks >0){
        int thumbMovement = fingers[0] + thumbTicks - 1;
        pwm.setPWM(THUMB, 0, thumbMovement);
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks--;  
     }
     else if (thumbTicks < 0){
        int thumbMovement = fingers[0] + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, thumbMovement);
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        thumbTicks++; 
     }
     // Index Finger
     if (indexTicks >0){
        int indexMovement = fingers[1] + indexTicks - 1;
        pwm.setPWM(INDEX, 0, indexMovement);
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks--;  
     }
     else if (indexTicks < 0){
        int indexMovement = fingers[1] + indexTicks + 1;
        pwm.setPWM(INDEX, 0, indexMovement);
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        indexTicks++; 
     }

     // Middle Finger
     if (middleTicks >0){
        int middleMovement = fingers[2] + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement);
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks--;  
     }
     else if (middleTicks < 0){
        int middleMovement = fingers[2] + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, middleMovement);
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        middleTicks++; 
     }
     // Ring Finger
     if (ringTicks >0){
        int ringMovement = fingers[3] + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement);
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks--;  
     }
     else if (ringTicks < 0){
        int ringMovement = fingers[3] + ringTicks + 1;
        pwm.setPWM(RING, 0, ringMovement);
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        ringTicks++; 
     }
     // Pinky Finger
     if (pinkyTicks >0){
        int pinkyMovement = fingers[4] + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement);
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks--;  
     }
     else if (pinkyTicks < 0){
        int pinkyMovement = fingers[4] + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, pinkyMovement);
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        pinkyTicks++; 
     }
     delay(3);
  }
  publishPosition = true;
}

// create a subscriber, that looks for messages on /robot_hand_command and
// processes them with the robotCommandParser function
ros::Subscriber<std_msgs::String> sub("robot_hand_command", &robotCommandParser);      


void setup()
{
  // Disables the LoRa chip so that another SPI device can be used, like the Ethernet Featherwing
  //pinMode(8, INPUT_PULLUP);
  Ethernet.init(10);  // ESP32 with Adafruit Featherwing Ethernet
  // Use serial to monitor the process
  //Serial.begin(115200);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  // Allow some time for the Ethernet Shield to be initialized
  delay(1000);
  //Serial.println("");
  //Serial.println("Ethernet connected");
  //Serial.println("IP address: ");
  //Serial.println(Ethernet.localIP());

  nh.getHardware()->setConnection(server, serverPort);
  //Serial.println("Connect to server");
  nh.initNode();
  //Serial.println("Init Node");
  nh.subscribe(sub);  
  //Serial.println("Subscribed to Robot Hand Command");
  pwm.begin();                                                               // start the pwm
  //Serial.println("Begin PWM");
  pwm.setOscillatorFrequency(27000000);                                      // with the defaulted frequency
  pwm.setPWMFreq(SERVO_FREQ);                                                // *Analog servos run at ~50 Hz updates 
  delay(1000);

  // Calculate the current position
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
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)      // for each step in the servo range 
  {
     if (thumbTicks <0){
        //Serial.println("close thumb");
        int movement = SERVOMAX + thumbTicks + 1;
        pwm.setPWM(THUMB, 0, movement); 
        //Serial.print("thumb movement ");
        //Serial.print(movement);
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        //Serial.print("currentPos ");
        //Serial.println(currentThumbPos);
        thumbTicks++;
     }
     if (indexTicks <0){
        //Serial.println("close index finger");
        int movement = SERVOMAX + indexTicks + 1;
        //Serial.print("index movement ");
        //Serial.print(movement);
        pwm.setPWM(INDEX, 0, movement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
       // Serial.print(" currentPos ");
        //Serial.println(currentIndexPos);
        indexTicks++;
     }
     if (middleTicks <0){
       // Serial.println("close middle finger");
        int movement = SERVOMAX + middleTicks + 1;
        pwm.setPWM(MIDDLE, 0, movement); 
        //Serial.print("middle movement ");
        //Serial.print(movement);
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        //Serial.print("currentPos ");
        //Serial.println(currentMiddlePos);
        middleTicks++;
     }
     if (ringTicks <0){
        //Serial.println("close ring finger");
        int movement = SERVOMAX + ringTicks + 1;
        pwm.setPWM(RING, 0, movement); 
       // Serial.print("ring movement ");
        //Serial.print(movement);
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
        //Serial.print("currentPos ");
        //Serial.println(currentRingPos);
        ringTicks++;
     }
     if (pinkyTicks <0){
        //Serial.println("close pinky finger");
        int movement = SERVOMAX + pinkyTicks + 1;
        pwm.setPWM(PINKY, 0, movement); 
       // Serial.print("pinky movement ");
       // Serial.print(movement);
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
     //   Serial.print("currentPos ");
        //Serial.println(currentPinkyPos);
        pinkyTicks++;
     }
     delay(3);  
  }
  delay(1000);                                                              // this delay adjusts the pause between fingers
  // Open all fingers
  indexTicks = currentIndexPos - SERVOMIN;
  ringTicks = currentRingPos - SERVOMIN;
  thumbTicks = currentThumbPos - SERVOMIN;
  middleTicks = currentMiddlePos - SERVOMIN;
  pinkyTicks = currentPinkyPos - SERVOMIN;
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)      // for each step in the servo range 
  {
     if (thumbTicks > 0){
        //Serial.println("open thumb finger");
        int thumbMovement = SERVOMIN + thumbTicks - 1;
        pwm.setPWM(THUMB, 0, thumbMovement); 
        currentThumbPos = analogRead(thumbPin)*analogThumbMultiplier+analogThumbShifter;
        //Serial.print("currentthumbPos ");
        //Serial.println(currentThumbPos);
        thumbTicks--;
     }
     if (indexTicks >0){
        //Serial.println("open index finger");
        int indexMovement = SERVOMIN + indexTicks - 1;
        //Serial.print("movement ");
       // Serial.print(indexMovement);
        pwm.setPWM(INDEX, 0, indexMovement); 
        currentIndexPos = analogRead(indexPin)*analogIndexMultiplier+analogIndexShifter;
        //Serial.print(" currentindexPos ");
        //Serial.println(currentIndexPos);
        indexTicks--;
     }     
     if (middleTicks > 0){
        //Serial.println("open middle finger");
        int middleMovement = SERVOMIN + middleTicks - 1;
        pwm.setPWM(MIDDLE, 0, middleMovement); 
        currentMiddlePos = analogRead(middlePin)*analogMiddleMultiplier+analogMiddleShifter;
        //Serial.print("currentmiddlePos ");
        //Serial.println(currentMiddlePos);
        middleTicks--;
     }      
     if (ringTicks >0){
        //Serial.println("open ring finger");
        int ringMovement = SERVOMIN + ringTicks - 1;
        pwm.setPWM(RING, 0, ringMovement); 
        currentRingPos = analogRead(ringPin)*analogRingMultiplier+analogRingShifter;
       // Serial.print("currentringPos ");
        //Serial.println(currentRingPos);
        ringTicks--;
     }
     if (pinkyTicks >0){
        //Serial.println("open pinky finger");
        int pinkyMovement = SERVOMIN + pinkyTicks - 1;
        pwm.setPWM(PINKY, 0, pinkyMovement); 
        currentPinkyPos = analogRead(pinkyPin)*analogPinkyMultiplier+analogPinkyShifter;
        //Serial.print("currentpinkyPos ");
        //Serial.println(currentPinkyPos);
        pinkyTicks--;
     }
     delay(3);  
  }
  // Advertise robot_hand_position
  nh.advertise(chatter);
}

void loop()
{
  // Converts the integer values into a string format usable by the ROS
  // Publisher. There's got to be a better way of doing this.
  if (publishPosition){
     char handPosition[21] = ""; 
     char thumbPos[3] = "";
     char indexPos[3] = "";
     char middlePos[3] = "";
     char ringPos[3] = "";
     char pinkyPos[3] = "";
     itoa(currentThumbPos,thumbPos,10);
     itoa(currentIndexPos,indexPos,10);
     itoa(currentMiddlePos,middlePos,10);
     itoa(currentRingPos,ringPos,10);
     itoa(currentPinkyPos,pinkyPos,10);
     strcat(handPosition,thumbPos);
     strcat(handPosition," ");
     strcat(handPosition,indexPos);
     strcat(handPosition," ");
     strcat(handPosition,middlePos);
     strcat(handPosition," ");
     strcat(handPosition,ringPos);
     strcat(handPosition," ");
     strcat(handPosition,pinkyPos);
     str_msg.data = (const char*)handPosition;
     chatter.publish( &str_msg );
     publishPosition = false;
  }
  nh.spinOnce();
  delay(1000);
}
