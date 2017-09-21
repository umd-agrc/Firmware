#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "basic_navigator.h"

elka::BasicNavigator::BasicNavigator() {

}

elka::BasicNavigator::~BasicNavigator() {

}

bool elka::BasicNavigator::atSetpoint(
    math::Vector<3> *pos) {
  //curr_state.slice(3); 
  return true;
}

bool elka::BasicNavigator::atSetpoint(
    math::Vector<3> *pos,
    math::Vector<3> *vel_max) {
  return true;
}
