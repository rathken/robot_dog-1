#include "Robot.h"

using namespace RobotDog;



Robot::Robot() {
}

void Robot::init() {
  for(uint8_t i=0;i<num_motors;i++) {
//    Serial.println(i);
    Motor *mtr=motors[i];
    mtr->channel=i;
    mtr->setParams(servo_max,servo_min,servo_avg,offset[i]);
//    moveMotor(*mtr,mtr->pw);
  }
  moveMotors();
}

void Robot::moveMotor(Motor& m,uint16_t pulsewidth) {
  m.setPW(pulsewidth);
  pwm->setPWM(m.channel,0,m.pw);
}

void Robot::pos1(void) {
  for(uint8_t i=0;i<num_motors;i++) {
    Motor *mtr=motors[i];
    mtr->setPW((mtr->defaultPW)+(mtr->offset)-delta[i]);
  }
  moveMotors();
}

void Robot::pos2(void) {
  for(uint8_t i=0;i<num_motors;i++) {
    Motor *mtr=motors[i];
    mtr->setPW((mtr->defaultPW)+(mtr->offset));
  }
  moveMotors();
}

void Robot::pos3(void) {
  for(uint8_t i=0;i<num_motors;i++) {
    Motor *mtr=motors[i];
    mtr->setPW((mtr->defaultPW)+(mtr->offset)+delta[i]);
  }
  moveMotors();
}
void Robot::moveMotors(void) {
  for(uint8_t i=0;i<num_motors;i++) {
    Motor *mtr=motors[i];
    moveMotor(*mtr,mtr->pw);
//    Serial.print(String(i)+" ( " +String(mtr->pw)+" ) , ");
  }
//  Serial.println("");
}

