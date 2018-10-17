//************************************************************************************
// Robot Console
// Copyright (c) 2018 Chi K. Lee
//************************************************************************************

#include "Config.h"
#include "Command.h"



//-----------------------------------------------------------
// Adafruit PWM controller library
// PCA9685 16-channel PWM & Servo driver
//-----------------------------------------------------------
#ifdef WITH_SERVOS
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();;
#endif

//-----------------------------------------------------------
// Robot console
//-----------------------------------------------------------
Command c(&pwm);
String tempStr="";

//-----------------------------------------------------------
// main routines
//-----------------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE);
  Serial.println(String(STR_BOOT_MSG));
#ifdef WITH_SERVOS  
  ::pwm.begin();
  ::pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
#endif
  delay(10);
  c.sendOK();
}

void loop() {
 if (!c.isInProgress()) {// process next command in command buffer if there is no command in progress
    if (c.commandPending()) { // buffer is not empty    
      c.processFirstCmd();
    }
  } else { // some command is in progress
    //TODO
  }
  if (Serial.available()) {    
    if (!c.isFull()) { // command buffer is not full
      processSerialIn();
    } 
  }
//  delay(5);
}

void processSerialIn() {
  int ch;
  ch=Serial.read();
  switch(ch) {
    case 'R':
    case ';':
//            tempStr+='\n';
            c.appendCommand(tempStr);
            tempStr=(char)ch;
            break;
    case '\n':
            if (tempStr.length() > MAX_LINE_SIZE) {
              Serial.println("Command is too long (>"+String(MAX_LINE_SIZE)+")");
            } else if (tempStr.length()>0) {
              c.appendCommand(tempStr);
            }
            tempStr="";
            break;
    default:
            tempStr+=(char)ch;           
  }
}

