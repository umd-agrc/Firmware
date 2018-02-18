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
#define POSITION_EPSILON 0.15
#define VELOCITY_EPSILON 0.15
#define ANGLE_EPSILON 0.07
#define ANGLE_RATE_EPSILON 0.05
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
#define SETPOINT_MAP_LEN 4 // x,y,z,yaw

#define SETPOINT_X 'x'
#define SETPOINT_Y 'y'
#define SETPOINT_Z 'z'
#define SETPOINT_YAW 'l'

namespace elka {
  class BasicNavigator;
  struct PlanElement;
}

static math::Matrix<6,6> spline_coefs_mat_;
static bool spline_coefs_init_=false;

struct setpoint_s {
  // _coefs vector is function order+1 for all polynomial terms
  // including constant term
  std::map<char,math::Vector<SETPOINT_FUNCTION_ORDER+1>> _coefs;
  hrt_abstime _dt,_dt_tot,_start;
  uint16_t _base_thrust;
  bool _init,_timeout,_hold,_land;
  setpoint_s(
      hrt_abstime dt,
      math::Vector<SETPOINT_MAP_LEN> &start_pos,
      math::Vector<SETPOINT_MAP_LEN> &start_vel,
      math::Vector<SETPOINT_MAP_LEN> &start_acc,
      math::Vector<SETPOINT_MAP_LEN> &end_pos,
      math::Vector<SETPOINT_MAP_LEN> &end_vel,
      math::Vector<SETPOINT_MAP_LEN> &end_acc,
      uint16_t base_thrust,
      uint8_t param_mask)
    : _dt_tot(dt),_base_thrust(base_thrust) {
    _init=false;
    _timeout=false;
    _hold=param_mask&SETPOINT_PARAM_HOLD;
    _land=param_mask&SETPOINT_PARAM_LAND;

    if (!spline_coefs_init_) {
      float spline_coefs_arr[6][6] =
        {{-1,5,-10,10,-5,1},
         {5,-20,30,-20,5,0},
         {-10,30,-30,10,0,0},
         {10,-20,10,0,0,0},
         {-5,5,0,0,0,0},
         {1,0,0,0,0,0}};
      spline_coefs_mat_=math::Matrix<6,6>(spline_coefs_arr);
      spline_coefs_init_=true;
    }

    math::Matrix<6,SETPOINT_MAP_LEN> p;
    p.zero();
    p.set_row(0,start_pos);
    p.set_row(1,start_vel*0.2+start_pos);
    p.set_row(2,start_acc*0.05+p.get_row(1)*2-p.get_row(0));
    p.set_row(5,end_pos);
    p.set_row(4,p.get_row(5)-end_vel*0.2);
    p.set_row(3,end_acc*0.05+p.get_row(4)*2-p.get_row(5));
    
    _coefs[SETPOINT_X]=spline_coefs_mat_*p.get_col(0);
    _coefs[SETPOINT_Y]=spline_coefs_mat_*p.get_col(1);
    _coefs[SETPOINT_Z]=spline_coefs_mat_*p.get_col(2);
    _coefs[SETPOINT_YAW]=spline_coefs_mat_*p.get_col(3);
  }
  void update() {
    if (!_init) {
      _init=true;
      _start=hrt_absolute_time();
    }
    _dt=hrt_absolute_time()-_start;
    if (!_hold &&
        _dt>=_dt_tot) {
      _timeout=true;
    }
  }
  // Evaluate spline w/_coefs
  // Mark _timeout if setpoint has timed out
  // Thus, **must** check for timeout after calling eval()
  math::Vector<POSITION_LEN> eval() {
    math::Vector<POSITION_LEN> s;
    update();
    if (_timeout) return s;
    s(0)=_coefs[SETPOINT_X](0)*pow(_dt,5)+
         _coefs[SETPOINT_X](1)*pow(_dt,4)+
         _coefs[SETPOINT_X](2)*pow(_dt,3)+
         _coefs[SETPOINT_X](3)*pow(_dt,2)+
         _coefs[SETPOINT_X](4)*pow(_dt,1)+
         _coefs[SETPOINT_X](5);
    s(1)=_coefs[SETPOINT_Y](0)*pow(_dt,5)+
         _coefs[SETPOINT_Y](1)*pow(_dt,4)+
         _coefs[SETPOINT_Y](2)*pow(_dt,3)+
         _coefs[SETPOINT_Y](3)*pow(_dt,2)+
         _coefs[SETPOINT_Y](4)*pow(_dt,1)+
         _coefs[SETPOINT_Y](5);
    s(2)=_coefs[SETPOINT_Z](0)*pow(_dt,5)+
         _coefs[SETPOINT_Z](1)*pow(_dt,4)+
         _coefs[SETPOINT_Z](2)*pow(_dt,3)+
         _coefs[SETPOINT_Z](3)*pow(_dt,2)+
         _coefs[SETPOINT_Z](4)*pow(_dt,1)+
         _coefs[SETPOINT_Z](5);
    s(3)=_coefs[SETPOINT_YAW](0)*pow(_dt,5)+
         _coefs[SETPOINT_YAW](1)*pow(_dt,4)+
         _coefs[SETPOINT_YAW](2)*pow(_dt,3)+
         _coefs[SETPOINT_YAW](3)*pow(_dt,2)+
         _coefs[SETPOINT_YAW](4)*pow(_dt,1)+
         _coefs[SETPOINT_YAW](5);
    s(4)=5*_coefs[SETPOINT_YAW](0)*pow(_dt,4)+
         4*_coefs[SETPOINT_YAW](1)*pow(_dt,3)+
         3*_coefs[SETPOINT_YAW](2)*pow(_dt,2)+
         2*_coefs[SETPOINT_YAW](3)*pow(_dt,1)+
         1*_coefs[SETPOINT_YAW](4);
    return s;
  }
};

// Derivative function
// For endpoints use backward derivative formula:
//    f'(x)=(f(x)-f(x-h))/h
// For midpoints use forward & backward derivative formula:
//    f'(x)=(f(x+h1)-f(x-h2))/(h1+h2)
// Note that this does not assume equal spacing between points
/*
std::vector<math::Vector<N>> deriv(
    std::vector<math::Vector<N>> v,hrt_abstime dt) {
  std::vector<math::Vector<N>> dv;
  uint8_t i=0,sz=v.size();
  float dv_pt[N];
  for (auto it=v.begin(); it!=v.end(); it++) {
    for (uint8_t j=0; j<N; j++) {
      if (i==0) {
        dv_pt[j]=(*(it+1)(j)-*(it)(j))/dt;
      } else if (i==N-1) {
        dv_pt[j]=(*(it)(j)-*(it-1)(j))/dt;
      } else {
        dv_pt[j]=(*(it+1)(j)-*(it-1)(j))/dt;
      }
    }
    i++;
  }
}
*/

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
  math::Vector<12> _body_pose_error;

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

  math::Vector<12> get_body_pose_error();
  uint16_t get_base_thrust() {
    return _curr_err.get_base_thrust();
  } 

  // Update poses and stores position/velocity in ELKA body frame
  // Corrects for Snapdragon<->ELKA offset
  // If current setpoint expired, sets next setpoint
  void update_pose(vehicle_local_position_s *p,
                   vehicle_attitude_s *a,
                   vision_velocity_s *v);
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
