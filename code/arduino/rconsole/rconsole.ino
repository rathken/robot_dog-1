//************************************************************************************
// Robot Console
// Copyright (c) 2018 Chi K. Lee
//************************************************************************************

#include "Config.h"
#include "Command.h"

//extern MyScheduler runner;
Command c;

void setup() {
  Serial.begin(BAUDRATE);
  printBootMessage();

  Serial.println("ok\n");
}

void loop() {
 int ch;
 if (!c.isInProgress()) {// process next command in command buffer if there is no command in progress
    //    Serial.println("Not busy, checking command buffer");
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
              if (ch != '\n') { c.appendChar('\n'); }
              if (c.readingCmd.length() > MAX_LINE_SIZE) {
                Serial.println("Commad line is too long (>"+String(MAX_LINE_SIZE)+")");
              } else if (c.readingCmd.length()==0) {
                Serial.println("Command line is too short (0 character)");    
              } else {
                c.appendCommand();
              }
              c.readingCmd="";
              if (ch != '\n') {
                c.appendChar(ch);
              }
              break;
      default:
              c.appendChar(ch);            
      }
    } else {  // command buffer is full. do nothing
    }
  }
  //runner.execute();
}
  
void printBootMessage() {
  Serial.println("echo: Robot Console");
  Serial.println("echo: Version: " + String(STR_VERSION));
  Serial.println("echo: Author : " + String(STR_AUTHOR));
}
