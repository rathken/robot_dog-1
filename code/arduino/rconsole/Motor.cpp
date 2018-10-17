#include "Motor.h"

using namespace RobotDog;

Motor::Motor(void) {
  maxPW=minPW=defaultPW=250;
  offset=0;
}

void Motor::setParams(uint16_t maxP,uint16_t minP,uint16_t defaultP,int8_t o) {
  setMaxPW(maxP);
  setMinPW(minP);
  setDefaultPW(defaultP);
  setOffset(o);
  setPW(defaultP+o);
}

void Motor::setPW(uint16_t pulsewidth) {
  lastPW=pw;
  if (pulsewidth>=maxPW) { pw=maxPW;}
  else if (pulsewidth <=minPW) {pw=minPW;}
  else {pw=pulsewidth;}
}

uint16_t Motor::changePW(int8_t delta) {
  setPW(pw+delta);
  return lastPW;
}

void Motor::revertPW(void) {
  pw=lastPW;
}
