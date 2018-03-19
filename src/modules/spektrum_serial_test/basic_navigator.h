#ifndef BASIC_NAVIGATOR_H
#define BASIC_NAVIGATOR_H

#include <ctype.h>
#include <inttypes.h>
#include <stdint.h>
#include <vector>
#include <map>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <uORB/topics/plan_element_params.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vision_velocity.h>

#include "serial_defines.h"
#include "basic_estimator.h"

// Defines close enough to setpoint position in meters
#define FLOAT_ERROR_EPSILON 0.000001
#define POSITION_EPSILON 0.04
#define ALTITUDE_EPSILON 0.04
#define VELOCITY_EPSILON 0.04
#define ANGLE_EPSILON 0.07
#define ANGLE_RATE_EPSILON 0.05
#define POSITION_MAX 5
#define ALTITUDE_MAX 2
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

#define POSITION_LEN 5 // {x,y,z,yaw,yawrate}
#define SETPOINT_POSE_LEN 8 // {x,y,z,vx,vy,vz,yaw,yawrate}

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
  // _coefs stores each setpoint parameter's associated interpolating
  // spline
  std::map<char,math::Vector<SETPOINT_FUNCTION_ORDER+1>> _coefs;
  hrt_abstime _dt,_dt_tot,_start;
  uint16_t _base_thrust;
  bool _init,_timeout,_hold,_land;
  setpoint_s() {}
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

  math::Vector<SETPOINT_POSE_LEN> eval_at(hrt_abstime dt) {
    math::Vector<SETPOINT_POSE_LEN> s;
    // x
    s(0)=_coefs[SETPOINT_X](0)*pow(dt,5)+
         _coefs[SETPOINT_X](1)*pow(dt,4)+
         _coefs[SETPOINT_X](2)*pow(dt,3)+
         _coefs[SETPOINT_X](3)*pow(dt,2)+
         _coefs[SETPOINT_X](4)*pow(dt,1)+
         _coefs[SETPOINT_X](5);
    // y
    s(1)=_coefs[SETPOINT_Y](0)*pow(dt,5)+
         _coefs[SETPOINT_Y](1)*pow(dt,4)+
         _coefs[SETPOINT_Y](2)*pow(dt,3)+
         _coefs[SETPOINT_Y](3)*pow(dt,2)+
         _coefs[SETPOINT_Y](4)*pow(dt,1)+
         _coefs[SETPOINT_Y](5);
    // z
    s(2)=_coefs[SETPOINT_Z](0)*pow(dt,5)+
         _coefs[SETPOINT_Z](1)*pow(dt,4)+
         _coefs[SETPOINT_Z](2)*pow(dt,3)+
         _coefs[SETPOINT_Z](3)*pow(dt,2)+
         _coefs[SETPOINT_Z](4)*pow(dt,1)+
         _coefs[SETPOINT_Z](5);
    // vx
    s(3)=5*_coefs[SETPOINT_X](0)*pow(dt,4)+
         4*_coefs[SETPOINT_X](1)*pow(dt,3)+
         3*_coefs[SETPOINT_X](2)*pow(dt,2)+
         2*_coefs[SETPOINT_X](3)*pow(dt,1)+
         1*_coefs[SETPOINT_X](4);
    // vy
    s(4)=5*_coefs[SETPOINT_Y](0)*pow(dt,4)+
         4*_coefs[SETPOINT_Y](1)*pow(dt,3)+
         3*_coefs[SETPOINT_Y](2)*pow(dt,2)+
         2*_coefs[SETPOINT_Y](3)*pow(dt,1)+
         1*_coefs[SETPOINT_Y](4);
    // vz
    s(5)=5*_coefs[SETPOINT_Z](0)*pow(dt,4)+
         4*_coefs[SETPOINT_Z](1)*pow(dt,3)+
         3*_coefs[SETPOINT_Z](2)*pow(dt,2)+
         2*_coefs[SETPOINT_Z](3)*pow(dt,1)+
         1*_coefs[SETPOINT_Z](4);
    // yaw
    s(6)=_coefs[SETPOINT_YAW](0)*pow(dt,5)+
         _coefs[SETPOINT_YAW](1)*pow(dt,4)+
         _coefs[SETPOINT_YAW](2)*pow(dt,3)+
         _coefs[SETPOINT_YAW](3)*pow(dt,2)+
         _coefs[SETPOINT_YAW](4)*pow(dt,1)+
         _coefs[SETPOINT_YAW](5);
    // yawrate
    s(7)=5*_coefs[SETPOINT_YAW](0)*pow(dt,4)+
         4*_coefs[SETPOINT_YAW](1)*pow(dt,3)+
         3*_coefs[SETPOINT_YAW](2)*pow(dt,2)+
         2*_coefs[SETPOINT_YAW](3)*pow(dt,1)+
         1*_coefs[SETPOINT_YAW](4);
    return s;

  }

  // Evaluate spline w/_coefs
  // Mark _timeout if setpoint has timed out
  // Thus, **must** check for timeout after calling eval()
  math::Vector<SETPOINT_POSE_LEN> eval() {
    math::Vector<SETPOINT_POSE_LEN> s;
    update();
    if (_timeout) return s;
    else return eval_at(_dt);
  }

  math::Vector<SETPOINT_POSE_LEN> get_start() {
    math::Vector<SETPOINT_POSE_LEN> s;
    s(0) = _coefs[SETPOINT_X](5);
    s(1) = _coefs[SETPOINT_Y](5);
    s(2) = _coefs[SETPOINT_Z](5);
    s(3) = _coefs[SETPOINT_X](4);
    s(4) = _coefs[SETPOINT_Y](4);
    s(5) = _coefs[SETPOINT_Z](4);
    s(6) = _coefs[SETPOINT_YAW](5);
    s(7) = _coefs[SETPOINT_YAW](4);
    return s;
  }

  math::Vector<SETPOINT_POSE_LEN> get_end() {
    return eval_at(_dt_tot);
  }

  void set_pose_error(pose_stamped_s* p, pose_stamped_s* e) {
    math::Vector<3>eul_err;
    math::Vector<SETPOINT_POSE_LEN> setpoint = eval();

    // Check for timeout
    if (_timeout) {
      e->pose.zero();
      return;
    }

    e->pose(0)=setpoint(0)-p->pose(0); // x
    e->pose(1)=setpoint(1)-p->pose(1); // y
    e->pose(2)=setpoint(2)-p->pose(2); // z
    e->pose(3)=setpoint(3)-p->pose(3); // vx
    e->pose(4)=setpoint(4)-p->pose(4); // vy
    e->pose(5)=setpoint(5)-p->pose(5); // vz
    e->pose(12)=setpoint(7)-p->pose(12); // yawrate

    // euler angles
    eul_err=p->get_eul();
    eul_err(0) = 0; // roll
    eul_err(1) = 0; // pitch
    eul_err(2) = setpoint(6) - eul_err(2); // yaw
    e->set_eul(eul_err);
    e->base_thrust = _base_thrust;
  }
};

// Derivative function
// For endpoints use backward derivative formula:
//    f'(x)=(f(x)-f(x-h))/h
// For midpoints use forward & backward derivative formula:
//    f'(x)=(f(x+h1)-f(x-h2))/(h1+h2)
// Note that this does not assume equal spacing between points
template <unsigned int N>
void deriv(
    std::vector<math::Vector<N>>* v,
    std::vector<math::Vector<N>>* dv,
    hrt_abstime dt) {
  uint8_t i=0;
  float dv_pt[N];
  math::Vector<N> y2,y1;
  for (auto it=v->rend(); it!=v->rbegin(); it++) {
    if (i==0) {
      y2 = *(it+1); y1 = *it;
    } else if (i==N-1) {
      y2 = *it; y1 = *(it-1);
    } else {
      y2 = *(it+1); y1 = *(it-1);
      dt*=2;
    }
    for (uint8_t j=0; j<N; j++) {
      dv_pt[j]=(y2(j)-y1(j))/dt;
    }
    i++;
    dv->push_back(math::Vector<N>(dv_pt));
  }
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
  plan_element_params_s _params;
  hrt_abstime _dt, _start,_init_time;
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
  PlanElement(plan_element_params_s* p)
    : _type(p->type),_begun(false),_completed(false)
  {
    _params.type=p->type;
    _params.dt=(hrt_abstime)p->dt;
    _init_time=hrt_absolute_time();
    switch(_params.type) {
      case plan_element_params_s::TYPE_CALIBRATE:
      break;
    case plan_element_params_s::TYPE_CHECK:
      break;
    case plan_element_params_s::TYPE_TRAJECTORY:
      _params.trajectory_type = p->trajectory_type;
      _params.trajectory_direction = p->trajectory_direction;
      _params.trajectory_radius = p->trajectory_radius;
      _params.dpsi = p->dpsi;
       memcpy(_params.init_pos,p->init_pos,sizeof(_params.init_pos));
       memcpy(_params.final_pos,p->final_pos,sizeof(_params.final_pos));
       memcpy(_params.init_vel,p->init_vel,sizeof(_params.init_vel));
       memcpy(_params.final_vel,p->final_vel,sizeof(_params.final_vel));
       memcpy(_params.init_acc,p->init_acc,sizeof(_params.init_acc));
       memcpy(_params.final_acc,p->final_acc,sizeof(_params.final_acc));
    default:
      break;
    };
  }
  ~PlanElement(){}

  static uint8_t priority(const PlanElement *p) {
    switch (p->_type) {
    case PLAN_ELEMENT_CALIBRATE:
      return 1;
    case PLAN_ELEMENT_CHECK:
      return 2;
    case PLAN_ELEMENT_TRAJECTORY:
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

  static inline math::Vector<SETPOINT_MAP_LEN> set_setpoint_map_point(float x[3],float yaw) {
    math::Vector<SETPOINT_MAP_LEN> v;
    v(0)=x[0];
    v(1)=x[1];
    v(2)=x[2];
    v(3)=yaw;
    return v;
  }

  //TODO 
  static inline math::Vector<3> get_center(
      pose_stamped_s* p,
      uint8_t trajectory_type,
      uint8_t trajectory_dir) {
    if (trajectory_type == plan_element_params_s::TRAJ_TYPE_CIRCLE) {
    }
    return math::Vector<3>();
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
  //std::vector<pose_stamped_s> _setpoints; // stores current setpoint
  std::vector<setpoint_s> _setpoints; // stores current setpoint
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
  bool _pose_init=false;

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
  /*
  void add_setpoint(
      hrt_abstime t,
      uint8_t param_mask,
      math::Vector<STATE_LEN>*v,
      uint16_t base_thrust);
      */

  void add_setpoint(setpoint_s *s);
  void add_setpoint(
      hrt_abstime dt,
      math::Vector<SETPOINT_MAP_LEN> &start_pos,
      math::Vector<SETPOINT_MAP_LEN> &start_vel,
      math::Vector<SETPOINT_MAP_LEN> &start_acc,
      math::Vector<SETPOINT_MAP_LEN> &end_pos,
      math::Vector<SETPOINT_MAP_LEN> &end_vel,
      math::Vector<SETPOINT_MAP_LEN> &end_acc,
      uint16_t base_thrust,
      uint8_t param_mask);

  int8_t trajectory(plan_element_params_s* params);
  void safe_landing();
  /*
  uint8_t takeoff(float z,bool hold);
	// Generate hover setpoint
	// Hover at current {x,y,z,yaw} for default length of time
	//TODO maintain yaw
	uint8_t hover(bool hold);
	uint8_t land(bool hold);
  */
  void reset_setpoints();
	bool at_setpoint();
  // Load positions into setpoints from a vector of positions
  // These positions are typically from a elka::PlanElement
  int8_t generate_setpoints(
      std::vector<math::Vector<SETPOINT_MAP_LEN>> p);
  /*
  int8_t generate_setpoints(
      std::vector<math::Vector<POSITION_LEN>> p);
      */
  void print_setpoints();
  void update_error(pose_stamped_s *curr_setpoint);
  void update_error(setpoint_s *curr_setpoint);

  void copy_pose_error(
      vehicle_local_position_s *p,
      vehicle_attitude_s *a);
};

#endif
