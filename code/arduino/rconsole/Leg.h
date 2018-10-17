#pragma once

#include <stdint.h>
#include "Motor.h"

namespace RobotDog {
  class Leg {
  public:
    Motor m1,m2,m3;
    Leg();
  };
};

