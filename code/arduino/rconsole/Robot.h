#pragma once
#include "Leg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//extern Adafruit_PWMServoDriver pwm;

namespace RobotDog {
  class Robot {
  private:
    Adafruit_PWMServoDriver *pwm;

    static const int num_motors = 12;
    static const int servo_min=100; // this is the 'minimum' pulse length count (out of 4096)
    static const int servo_max=400; // this is the 'maximum' pulse length count (out of 4096)
    static const int servo_avg=250;
    
    int offset[num_motors] = { 20, 10, 30,-40, 20,-60,-40, 10,-40, 40, 10, 40};
    int avg[num_motors] =    {servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg,servo_avg};
    static const int d = 10;
    int delta[num_motors]  = {-d,-d,0,d,d,0,d,d,0,-d,-d,0};

    Leg fl,fr,bl,br; // front left, front right, back left, back right
    Motor* motors[num_motors] = {
      &fr.m1, &fr.m2, &fr.m3,  
      &fl.m1, &fl.m2, &fl.m3,
      &bl.m1, &bl.m2, &bl.m3,
      &br.m1, &br.m2, &br.m3
    };  
  public:
    Robot();
    // methods
    void init(void);
    void moveMotor(Motor& m,uint16_t pulsewidth);
    void pos1(void);
    void pos2(void);
    void pos3(void);
    void moveMotors(void);
    void setPWM(Adafruit_PWMServoDriver *p) { pwm=p;}
    void moveMotors(const String &cmd);
  };
};




