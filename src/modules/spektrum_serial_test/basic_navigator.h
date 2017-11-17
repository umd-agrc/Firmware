#ifndef BASIC_NAVIGATOR_H
#define BASIC_NAVIGATOR_H

#include <inttypes.h>
#include <stdint.h>
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

namespace elka {
  class BasicNavigator;
}

class elka::BasicNavigator {
private:
  // Storing global frame NED error
  // For VISlam local , x(+) := board left
  //                    y(+) := board forward
  //                    z(+) := board down
  //                    yaw(+) :=TODO (assuming cw about z from forward)

  elka::BasicEstimator _est; // estimated state in elka inertial frame
  pose_stamped_s _setpoint; // stores current setpoint
  pose_stamped_s _curr_err;
  pose_stamped_s _prev_min_err;
  pose_stamped_s _prev_sens;
  bool _new_setpoint;
  math::Matrix<3,3> _elka_sf_r;
  math::Vector<3> _elka_sf_t;

public:
  BasicNavigator();
  ~BasicNavigator();

  // Set offset from sensor to fcu
  void set_fcu_offset(math::Vector<3> *r, math::Vector<3> *t);

  bool atSetpoint();
  pose_stamped_s *get_pose();
  float get_pose(uint8_t n);
  pose_stamped_s *get_err();
  float get_err(uint8_t n);
  pose_stamped_s *get_prev_min_err();
  float get_prev_min_err(uint8_t n);
  pose_stamped_s *get_setpoint();
  float get_setpoint(uint8_t n);

  void set_pose(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_pose(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_pose(pose_stamped_s *p);
  void set_err(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_err(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_err(pose_stamped_s *p);
  void set_setpoint(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_setpoint(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_setpoint(pose_stamped_s *p);
  void set_prev_min_err(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_prev_min_err(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_prev_min_err(pose_stamped_s *p);
  void set_new_setpoint(bool b);

  // Update poses and stores position/velocity in ELKA body frame
  // Corrects for Snapdragon<->ELKA offset
  void update_pose(vehicle_local_position_s *p,
                   vehicle_attitude_s *a);
  void reset_setpoint();

  // Two cases here:
  // 1) Error exceeds error threshold.
  //    In this case, reset curr_err to 0,
  //                  reset prev_min_err to *_ERROR_DEFAULT
  //                  reset setpoint to curr_pos
  // 2) Error does not exceed error threshold.
  //    In this case, update curr_pos by curr_sens,
  //                  update curr_err by curr_sens,
  //                  if curr_err < prev_min_error
  //                  then update prev_min_err = curr_err
  // Return _new_setpoint 
  bool check_setpoint();

  void copy_pose_error(
      vehicle_local_position_s *p,
      vehicle_attitude_s *a);
};

#endif
