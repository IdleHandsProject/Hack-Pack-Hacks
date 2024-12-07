#include <Arduino.h>
#include <Servo.h>

#include "DFRobot_DF2301Q.h"

//I2C communication
DFRobot_DF2301Q_I2C DF2301Q;



//////////////////////////////////////////////////
//  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
//this is where we store global variables!
Servo yawServo;    //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo;  //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo;   //names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal;  //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal;

int pitchMoveSpeed = 8;  //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90;   //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90;   //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90;  //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90;  //value to stop the roll motor - keep this at 90

int yawPrecision = 150;   // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 250;  // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 175;  // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 10;   // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax


//////////////////////////////////////////////////
//  S E T U P  //
//////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);  // initializes the Serial communication between the computer and the microcontroller

  yawServo.attach(10);    //attach YAW servo to pin 10
  pitchServo.attach(11);  //attach PITCH servo to pin 11
  rollServo.attach(12);   //attach ROLL servo to pin 12

  homeServos();  //set servo motors to home position
  while (!(DF2301Q.begin())) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  /**
   * @brief Set voice volume
   * @param voc - Volume value(1~7)
   */
  DF2301Q.setVolume(6);

  /**
   * @brief Set mute mode
   * @param mode - Mute mode; set value 1: mute, 0: unmute
   */
  DF2301Q.setMuteMode(0);

  /**
   * @brief Set wake-up duration
   * @param wakeTime - Wake-up duration (0-255)
   */
  DF2301Q.setWakeTime(15);

  /**
   * @brief Get wake-up duration
   * @return The currently-set wake-up period
   */
  uint8_t wakeTime = 0;
  wakeTime = DF2301Q.getWakeTime();
  Serial.print("wakeTime = ");
  Serial.println(wakeTime);

  /**
   * @brief Play the corresponding reply audio according to the command word ID
   * @param CMDID - Command word ID
   * @note Can enter wake-up state through ID-1 in I2C mode
   */
  DF2301Q.playByCMDID(1);   // Wake-up command
  DF2301Q.playByCMDID(23);  // Common word ID
}

////////////////////////////////////////////////
//  L O O P  //
////////////////////////////////////////////////
void shakeHeadYes(int moves = 3);

void loop() {

  /**
   * @brief Get the ID corresponding to the command word 
   * @return Return the obtained command word ID, returning 0 means no valid ID is obtained
   */
  uint8_t CMDID = 0;
  CMDID = DF2301Q.getCMDID();
  if (0 != CMDID) {
    Serial.print("CMDID = ");
    Serial.println(CMDID);

    switch (CMDID) {  //this is where the commands are handled
      case 1:
        shakeHeadYes(3);
        delay(1000);
        break;

      case 6:  //pitch up
        upMove(2);
        break;

      case 5:  //pitch down
        downMove(2);
        break;

      case 7:  //fast counterclockwise rotation
        leftMove(1);
        break;

      case 8:  //fast clockwise rotation
        rightMove(1);
        break;

      case 9:  //firing routine
        fire();
        //Serial.println("FIRE");
        break;
      case 10:
        fireAll();
        delay(50);
        break;

      //// ADD MORE FUN CASES HERE //////
      default:
        break;


    }
  }
  delay(1000);
}


void shakeHeadYes(int moves = 3) {
  Serial.println("YES");
  int startAngle = pitchServoVal;  // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 20;  // Angle for nodding motion

  for (int i = 0; i < moves; i++) {  // Repeat nodding motion three times
    // Nod up
    for (int angle = startAngle; angle <= nodAngle; angle++) {
      pitchServo.write(angle);
      delay(7);  // Adjust delay for smoother motion
    }
    delay(50);  // Pause at nodding position
    // Nod down
    for (int angle = nodAngle; angle >= startAngle; angle--) {
      pitchServo.write(angle);
      delay(7);  // Adjust delay for smoother motion
    }
    delay(50);  // Pause at starting position
  }
}

void shakeHeadNo(int moves = 3) {
  Serial.println("NO");
  int startAngle = pitchServoVal;  // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 60;  // Angle for nodding motion

  for (int i = 0; i < moves; i++) {  // Repeat nodding motion three times
    // rotate right, stop, then rotate left, stop
    yawServo.write(140);
    delay(190);  // Adjust delay for smoother motion
    yawServo.write(yawStopSpeed);
    delay(50);
    yawServo.write(40);
    delay(190);  // Adjust delay for smoother motion
    yawServo.write(yawStopSpeed);
    delay(50);  // Pause at starting position
  }
}

void leftMove(int moves) {
  for (int i = 0; i < moves; i++) {
    yawServo.write(yawStopSpeed + yawMoveSpeed);  // adding the servo speed = 180 (full counterclockwise rotation speed)
    delay(yawPrecision);                          // stay rotating for a certain number of milliseconds
    yawServo.write(yawStopSpeed);                 // stop rotating
    delay(5);                                     //delay for smoothness
    Serial.println("LEFT");
  }
}

void rightMove(int moves) {  // function to move right
  for (int i = 0; i < moves; i++) {
    yawServo.write(yawStopSpeed - yawMoveSpeed);  //subtracting the servo speed = 0 (full clockwise rotation speed)
    delay(yawPrecision);
    yawServo.write(yawStopSpeed);
    delay(5);
    Serial.println("RIGHT");
  }
}

void upMove(int moves) {
  for (int i = 0; i < moves; i++) {
    if (pitchServoVal > pitchMin) {                    //make sure the servo is within rotation limits (greater than 10 degrees by default)
      pitchServoVal = pitchServoVal - pitchMoveSpeed;  //decrement the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println("UP");
    }
  }
}

void downMove(int moves) {
  for (int i = 0; i < moves; i++) {
    if (pitchServoVal < pitchMax) {                    //make sure the servo is within rotation limits (less than 175 degrees by default)
      pitchServoVal = pitchServoVal + pitchMoveSpeed;  //increment the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println("DOWN");
    }
  }
}

/**
 * fire does xyz
 */
void fire() {                                      //function for firing a single dart
  rollServo.write(rollStopSpeed + rollMoveSpeed);  //start rotating the servo
  delay(rollPrecision);                            //time for approximately 60 degrees of rotation
  rollServo.write(rollStopSpeed);                  //stop rotating the servo
  delay(15);                                        //delay for smoothness
  Serial.println("FIRING");
}

void fireAll() {                                   //function to fire all 6 darts at once
  rollServo.write(rollStopSpeed + rollMoveSpeed);  //start rotating the servo
  delay(rollPrecision * 6);                        //time for 360 degrees of rotation
  rollServo.write(rollStopSpeed);                  //stop rotating the servo
  delay(5);                                        // delay for smoothness
  Serial.println("FIRING ALL");
}

void homeServos() {
  yawServo.write(yawStopSpeed);  //setup YAW servo to be STOPPED (90)
  delay(20);
  rollServo.write(rollStopSpeed);  //setup ROLL servo to be STOPPED (90)
  delay(100);
  pitchServo.write(100);  //set PITCH servo to 100 degree position
  delay(100);
  pitchServoVal = 100;  // store the pitch servo value
  Serial.println("HOMING");
}