//************************************************************************************
// Robot Console
// Copyright (c) 2018 Chi K. Lee
//************************************************************************************

#include "Config.h"
#include "Command.h"

Command c;
String tempStr="";

void setup() {
  Serial.begin(BAUDRATE);
  printBootMessage();
  c.sendOK();
}

void loop() {
 int ch;
 if (!c.isInProgress()) {// process next command in command buffer if there is no command in progress
    if (!c.isEmpty()) { // buffer is not empty
      c.processFirstCmd();
    }
  } else { // some command is in progress
    //TODO
  }
  if (Serial.available()) {    
    if (!c.isFull()) { // command buffer is not full
      ch=Serial.read();
      switch(ch) {
      case 'R':
      case ';':
      case '\n':
              if (ch != '\n') { tempStr+='\n'; }
              if (tempStr.length() > MAX_LINE_SIZE) {
                Serial.println("Commad line is too long (>"+String(MAX_LINE_SIZE)+")");
              } else if (tempStr.length()==0) {
                Serial.println("Command line is too short (0 character)");    
              } else {
                c.appendCommand(tempStr);
              }
              tempStr="";
              if (ch != '\n') {
                tempStr+=(char)ch;
              }
              break;
      default:
              tempStr+=(char)ch;           
      }
    } else {  // command buffer is full. do nothing
    }
  }
}
  
void printBootMessage() {
  Serial.println("echo: Robot Console");
  Serial.println("echo: Version: " + String(STR_VERSION));
  Serial.println("echo: Author : " + String(STR_AUTHOR));
}
