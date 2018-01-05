#ifndef BASIC_NAVIGATOR_H
#define BASIC_NAVIGATOR_H

#include <ctype.h>
#include <inttypes.h>
#include <stdint.h>
#include <vector>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

#include "serial_defines.h"
#include "basic_estimator.h"

// Defines close enough to setpoint position in meters
#define POSITION_EPSILON 0.1
#define VELOCITY_EPSILON 0.01
#define ANGLE_EPSILON 0.01
#define POSITION_ERROR_DEFAULT 0
#define VELOCITY_ERROR_DEFAULT 0
#define ANGLE_ERROR_DEFAULT 0
#define POSITION_ERROR_THRES 0.2
#define VELOCITY_ERROR_THRES 0.4
#define ANGLE_ERROR_THRES 0.1
#define ANGLE_RATE_ERROR_THRES 0.1
// Define error threshold as norm({*_ERROR_THRES})
#define ERROR_THRES 1 

#define POSITION_LEN 5

namespace elka {
  class BasicNavigator;
  struct PlanElement;
}

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
  bool _begun,_completed;

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
  pose_stamped_s _curr_err;
  pose_stamped_s _prev_min_err;
	// Store transformation from elka
	// to snapdragon
  math::Matrix<3,3> _elka_sf_r;
  math::Vector<3> _elka_sf_t;

public:
  BasicNavigator();
  ~BasicNavigator();

	//TODO make private and have getters
  bool _at_setpoint,_new_setpoint,_from_manual;

  // Set offset from sensor to fcu
  void set_fcu_offset(math::Vector<3> *r, math::Vector<3> *t);

  pose_stamped_s *get_pose();
  float get_pose(uint8_t n);
  pose_stamped_s *get_err();
  float get_err(uint8_t n);
  pose_stamped_s *get_prev_min_err();
  float get_prev_min_err(uint8_t n);
  pose_stamped_s *get_next_setpoint();

  void set_prev_inert_sens(sensor_combined_s *s);
  void set_pose(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_pose(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_pose(pose_stamped_s *p);
  void set_err(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_err(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_err(pose_stamped_s *p);
  void set_prev_min_err(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_prev_min_err(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_prev_min_err(pose_stamped_s *p);

  // Update poses and stores position/velocity in ELKA body frame
  // Corrects for Snapdragon<->ELKA offset
  void update_pose(vehicle_local_position_s *p,
                   vehicle_attitude_s *a);
	// Clear setpoints 
  void add_setpoint(hrt_abstime t,math::Vector<STATE_LEN>*v);
	// Generate hover setpoint
	// Hover at current {x,y,z,yaw} for default length of time
	//TODO maintain yaw
	void hover();
  void reset_setpoints();
	bool at_setpoint();

  // Load positions into setpoints from a vector of positions
  // These positions are typically from a elka::PlanElement
  int8_t generate_setpoints(std::vector<math::Vector<POSITION_LEN>> p);

  void copy_pose_error(
      vehicle_local_position_s *p,
      vehicle_attitude_s *a);
};

#endif