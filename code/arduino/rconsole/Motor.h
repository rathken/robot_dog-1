#pragma once

#include <stdint.h>

namespace RobotDog {
  class Motor {
  public:
    uint16_t maxPW,minPW,defaultPW;
    uint16_t pw,lastPW;
    int8_t   offset;
    uint8_t  channel;
    
    Motor();
    void setParams(uint16_t maxP,uint16_t minP,uint16_t defaultP,int8_t o);
    void setPW(uint16_t pulsewidth);
    void setMaxPW(uint16_t m) { maxPW=m;}
    void setMinPW(uint16_t m) { minPW=m;}
    void setDefaultPW(uint16_t m) { defaultPW=m;}
    void setOffset(int8_t o) {offset=o;}
    void setChannel(uint8_t ch) { channel=ch;}
    uint16_t changePW(int8_t delta);
    void revertPW(void);

  };
};


  
