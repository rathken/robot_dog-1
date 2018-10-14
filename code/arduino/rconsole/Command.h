//************************************************************************************
// Robot Console
// Copyright (c) 2018 Chi K. Lee
//************************************************************************************
#pragma once
#include <Arduino.h>
#include "Config.h"


class Command {
  private:
    volatile bool inProgress;
    volatile int cmdIdx; // index of cmdBuffer. New command will be written into this entry.

    String cmd;
    char cmdBuffer[NUM_ENTRY][(MAX_LINE_SIZE+1)];
    volatile unsigned long startTime;              // in milliseconds
    volatile unsigned long expectedEndTime;        // in milliseconds
    int findString(char ch, int index);
    float findStringFloat(char ch, int index);
    String findStringProc(char ch, int index);
    void copyCmdBuffer(void);
    void removeComments(void);
    void parseCmd(void);
    void removeFirstEntry(void);
  public:
    String readingCmd;                             // buffer storing command from serial port. 
    Command(); 
    void showCommands(void);    // R999
    bool isInProgress(void) {  return inProgress;};
    void sendOK(void) {  Serial.println("ok\n");};
    bool isEmpty(void) { return (cmdIdx==0);};
    bool isFull(void) { return (cmdIdx>=NUM_ENTRY);};
    volatile int getIndex(void) { return cmdIdx;};
    void appendChar(int ch) {  readingCmd+=(char) ch;};
    void appendCommand(void);
    void processFirstCmd(void);
};


