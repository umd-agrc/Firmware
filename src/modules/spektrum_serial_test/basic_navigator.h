#ifndef BASIC_NAVIGATOR_H
#define BASIC_NAVIGATOR_H

#include <ctype.h>
#include <inttypes.h>
#include <stdint.h>
#include <vector>
#include <map>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vision_velocity.h>

#include "serial_defines.h"
#include "basic_estimator.h"

// Defines close enough to setpoint position in meters
#define POSITION_EPSILON 0.05
#define VELOCITY_EPSILON 0.01
#define ANGLE_EPSILON 0.02
#define ANGLE_RATE_EPSILON 0.02
#define POSITION_MAX 5
#define VELOCITY_MAX 3
#define ANGLE_MAX 21
#define ANGLE_RATE_MAX 21
#define POSITION_ERROR_DEFAULT 6
#define VELOCITY_ERROR_DEFAULT 0
#define ANGLE_ERROR_DEFAULT 0
#define POSITION_ERROR_THRES 0.1
#define VELOCITY_ERROR_THRES 0.1
#define ANGLE_ERROR_THRES 0.05
#define ANGLE_RATE_ERROR_THRES 0.05
// Define error threshold as norm({*_ERROR_THRES})
#define ERROR_THRES 1 

// Define error height close to ground in meters
#define LANDING_HEIGHT_EPSILON 0.1

#define POSITION_LEN 5
// Don't neet derivative values in setpoints.
// These can be derived from interpolating spline
#define SETPOINT_MAP_LEN 5 // x,y,z,yaw,yawrate

#define SETPOINT_X "x"
#define SETPOINT_Y "y"
#define SETPOINT_Z "z"
#define SETPOINT_YAW "l"

namespace elka {
  class BasicNavigator;
  struct PlanElement;
}

struct setpoint_s {
  hrt_abstime _dt,_start;
  std::map<char,math::Vector<SETPOINT_FUNCTION_ORDER>> _coefs;
  setpoint_s(
      hrt_abstime dt,
      float start[SETPOINT_MAP_LEN],
      float end[SETPOINT_MAP_LEN],
      uint16_t base_thrust) {
  }
};

// PlanElement includes type and 3D 
// Not every PlanElement includes 3D
// Types : Actions
//  calibrate : spin motors up and note effects
//    (useful for eventual neural network control)
//  check : nothing for now
//  takeoff : fly straight up to 1.5 m with level attitude
//  land : fly straight down to ground with level attitude
//  hover : remain at altitude with level attitude
struct elka::PlanElement {
  std::vector<math::Vector<POSITION_LEN>> _positions;
  hrt_abstime _dt,_start,_init_time;
  uint8_t _type; 
  bool _begun,_completed,_timeout;

  PlanElement(uint8_t t,hrt_abstime time)
    : _dt(time),_type(t),_begun(false),_completed(false)
		{_init_time=hrt_absolute_time();}
  PlanElement(uint8_t t,hrt_abstime time,
		std::vector<math::Vector<POSITION_LEN>> &v)
    : _positions(v),_dt(time),_type(t),_begun(false),
			_completed(false)
		{_init_time=hrt_absolute_time();}
  ~PlanElement(){}

  static uint8_t priority(const PlanElement *p) {
    switch (p->_type) {
    case PLAN_ELEMENT_CALIBRATE:
      return 1;
    case PLAN_ELEMENT_CHECK:
      return 2;
    case PLAN_ELEMENT_TAKEOFF:
      return 5;
    case PLAN_ELEMENT_LAND:
      return 5;
    case PLAN_ELEMENT_HOVER:
      return 5;
    default:
      return 255;
    }
  }

  // Update parameters based off of elapsed time
  void update() {
    if (!_begun) {
      _begun=true;
      _start=hrt_absolute_time();
    }

    if (_start+_dt<hrt_absolute_time()) {
      _timeout=true;
      _completed=true;
    }
  }

	void print_element() {
		PX4_INFO("Element type: %d\nTime: %" PRIu64 "+%" PRIu64"\n"
			"Num positions: %zu",
			_type,_start,_dt,_positions.size());
	}
};


class elka::BasicNavigator {
private:
  // Storing global frame NED error
  // For VISlam local , x(+) := board left
  //                    y(+) := board forward
  //                    z(+) := board down
  //                    yaw(+) :=TODO (assuming cw about z from forward)

  elka::BasicEstimator _est; // estimated state in elka inertial frame
  std::vector<pose_stamped_s> _setpoints; // stores current setpoint
  //std::vector<setpoint_s> _setpoints; // stores current setpoint
  pose_stamped_s _curr_err;
	// Store transformation from elka
	// to snapdragon
  math::Matrix<3,3> _elka_sf_r;
  math::Vector<3> _elka_sf_t;

public:
  BasicNavigator();
  ~BasicNavigator();

	//TODO make private and have getters
  bool _at_setpoint,_new_setpoint,_from_manual,_landed,_wait,_kill;

  // Set offset from sensor to fcu
  void set_fcu_offset(math::Vector<3> *r, math::Vector<3> *t);

  pose_stamped_s *get_pose();
  float get_pose(uint8_t n);
  pose_stamped_s *get_err();
  float get_err(uint8_t n);
  void next_setpoint();

  void set_prev_inert_sens(sensor_combined_s *s);
  void set_pose(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_pose(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_pose(pose_stamped_s *p);
  void set_err(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_err(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_err(pose_stamped_s *p);

  // Update poses and stores position/velocity in ELKA body frame
  // Corrects for Snapdragon<->ELKA offset
  // If current setpoint expired, sets next setpoint
  void update_pose(vehicle_local_position_s *p,
                   vehicle_attitude_s *a);
	// Clear setpoints 
  void add_setpoint(
      hrt_abstime t,
      uint8_t param_mask,
      math::Vector<STATE_LEN>*v,
      uint16_t base_thrust);
  uint8_t takeoff(float z,bool hold);
	// Generate hover setpoint
	// Hover at current {x,y,z,yaw} for default length of time
	//TODO maintain yaw
	uint8_t hover(bool hold);
	uint8_t land(bool hold);
  void reset_setpoints();
	bool at_setpoint();
  // Load positions into setpoints from a vector of positions
  // These positions are typically from a elka::PlanElement
  int8_t generate_setpoints(
      std::vector<math::Vector<POSITION_LEN>> p);
  void print_setpoints();
  void update_error(pose_stamped_s *curr_setpoint);
  //void update_error(setpoint_s *curr_setpoint);

  void copy_pose_error(
      vehicle_local_position_s *p,
      vehicle_attitude_s *a);
};

#endif
