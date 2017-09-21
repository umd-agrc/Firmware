#ifndef BASIC_NAVIGATOR_H
#define BASIC_NAVIGATOR_H

#include <inttypes.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <uORB/topics/vehicle_local_position.h>

// Defines close enough to setpoint position in meters
#define POSITION_EPSILON 0.1
#define POSITION_ERROR_DEFAULT 0.5

namespace elka {
  class BasicNavigator;
}

class elka::BasicNavigator {
private:
  math::Vector<12> curr_state; // {x,y,z,vx,vy,vz,r,p,y,rr,pr,yr}

public:
  BasicNavigator();
  ~BasicNavigator();

  bool atSetpoint(math::Vector<3> *pos);
  bool atSetpoint(math::Vector<3> *pos, math::Vector<3> *vel_max);
};

#endif
