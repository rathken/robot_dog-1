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

void Robot::moveMotors(const String &c) {
  // format : r4 3 270 5 120
  // meaning: motor 3 pwm = 270; motor 5 pwm = 120
  int idx;
  bool searchMotorNumber=true;
  String str="";
  int motorNumber;
  int pwmvalue;
  for (auto i=c.indexOf(' ');i< c.length(); i++) {
    int ch=c.charAt(i);
    if ((ch == ' ')||(i==c.length()-1)) {
      int j;
      if(i==c.length()-1) {
        str+=char(ch);
      }
      if (str.length()>0) {
          j=atoi(str.c_str());
          if (searchMotorNumber) {
            motorNumber=j;
          } else {
            pwmvalue=j;
            if ((motorNumber<num_motors)&&(motorNumber>=0)) {
              /*Serial.println("Motor number : "+String(motorNumber));
              Serial.println("PWM : "+String(pwmvalue));*/
              Motor *mtr=motors[motorNumber];
              mtr->setPW(pwmvalue);
            }
          }
          searchMotorNumber=!searchMotorNumber;
          str="";
      }
    } else {
      str+=char(ch);
    }
  }
  moveMotors();
}

