//************************************************************************************
// Robot Console
// Copyright (c) 2018 Chi K. Lee
//************************************************************************************
#include "Command.h"
#include "Config.h"
#include <Arduino.h>

Command::Command() {
  inProgress=false;
  cmd="";
  cmdIdx=0;
}

void Command::parseCmd(void) {
  int codeNum;
  if (cmd.charAt(0) == '?') {
    showCommands();
    return;
  }
  //=========================================================================================
  // R-code
  //=========================================================================================
  if ((cmd.charAt(0) == 'r')||(cmd.charAt(0) == 'R')) {
    codeNum=findString('r',0);
    switch(codeNum) {
    case 0: 
      inProgress=true;
      Serial.println("R0");
      inProgress=false;
      sendOK();
      break;
   case 999:
      inProgress=true;
      Serial.println("R999");
      showCommands();
      inProgress=false;
      sendOK();
      break;      
    default:
      Serial.println("Unknown code");
      sendOK();
      break;
    }
  }
}
void Command::showCommands(void) {
  Serial.println("List of commands:");
  Serial.println("R0 [motor number] [motor angle]      : go to position");
  Serial.println("R999                                 : show commands");
}

int Command::findString(char ch,int index) {
  return findStringProc(ch,index).toInt();
}

float Command::findStringFloat(char ch,int index) {
  float result;
  result=findStringProc(ch,index).toFloat();
  return result;
}

String Command::findStringProc(char ch, int index) {
  //find the string after the character 'ch'
  
  String temp,lc;
  lc=cmd.substring(index);
  lc.toLowerCase();
  
  int idxCh,idxFirstSpace,result;
  // Look for first appearence of character ch
  idxCh=lc.indexOf(ch);
  if (idxCh <0) { return "NOT FOUND";}  // not able to find ch
  
  idxFirstSpace=lc.indexOf(' ',idxCh+1); // look for first space after ch
  if (idxFirstSpace >0) {
    temp=cmd.substring(idxCh+1,idxFirstSpace);
  } else {
    temp=cmd.substring(idxCh+1);
  }
  return temp;
}

void Command::removeComments(void) {
  int idxCh;
  String temp;
  idxCh=cmd.indexOf(';');
  if (idxCh<0) { // unable to find ';'
    return;
  } else if (idxCh==0) {
    cmd="";
  } else {
    temp=cmd.substring(0,idxCh); // remove comments after ';'
    cmd=temp;
  }
}

void Command::removeFirstEntry(void) {
  if (cmdIdx>1) {
    for(int i=1;i<cmdIdx;i++) { // move the entries up
      strcpy(cmdBuffer[i-1],cmdBuffer[i]);
    }
  }
  cmdIdx--;
}

void Command::copyCmdBuffer(void) {
  cmd="";
  for (int i=0;i<strlen(cmdBuffer[0]);i++) {
    cmd+=(char) cmdBuffer[0][i];
  }
}

void Command::appendCommand(String &c) {
  c.toCharArray(cmdBuffer[cmdIdx],MAX_LINE_SIZE);
  cmdBuffer[cmdIdx][c.length()]=0x00;          // null character at the end
  cmdIdx++;
}

void Command::processFirstCmd(void) {
  copyCmdBuffer();
  removeComments();
  #ifdef DEBUG_MODE  
    Serial.println("Command : "+cmd);
  #endif
  parseCmd();
  cmd="";
  removeFirstEntry();
}

