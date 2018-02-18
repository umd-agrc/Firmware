#ifndef BASIC_ESTIMATOR_H
#define BASIC_ESTIMATOR_H

#include <cstring>

#include <px4_defines.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <lib/mathlib/math/Quaternion.hpp>
#include <lib/mathlib/math/Matrix.hpp>
#include <uORB/topics/sensor_combined.h>

#include "serial_defines.h"

namespace elka {
  class BasicEstimator;
}

// Using coordinate system BACKWARD-LEFT-DOWN as positive
//  YAW RIGHT as positive

#define STATE_LEN 13
#define STATE_LEN_EKF 15
#define STATE_LEN_EKF_ODOM 9 
#define STATE_LEN_EKF_BIAS 6 

#define EKF_STATE_P1 0
#define EKF_STATE_P2 1
#define EKF_STATE_P3 2
#define EKF_STATE_Q1 3
#define EKF_STATE_Q2 4
#define EKF_STATE_Q3 5
#define EKF_STATE_PD1 6
#define EKF_STATE_PD2 7
#define EKF_STATE_PD3 8
#define EKF_STATE_BG1 9
#define EKF_STATE_BG2 10
#define EKF_STATE_BG3 11
#define EKF_STATE_AG1 12
#define EKF_STATE_AG2 13
#define EKF_STATE_AG3 14

#define SECT_POS 0
#define SECT_VEL 1
#define SECT_ANG 2
#define SECT_ANG_RATE 3

#define FILT_NONE 0
#define FILT_STATE 1
#define FILT_DERIV 2

#define DT_MIN 0.02f

#define BAD_NUM(x) (isnan(abs(x)) || isinf(abs(x)))

#define SIGMOID(x) 2*(exp(x)/(exp(x)+1)-.5) // scales between [-1,1]

struct sensor_stamped_s {
  uint8_t type;
  hrt_abstime ts; // timestamp
  math::Vector<3> dat; // data
  math::Vector<3> bias; // bias
  math::Vector<3> noise; // sensor
  //TODO add noise models as array of
  //     function pointers
  math::Vector<3> offset_t;
  math::Matrix<3,3> offset_r;

  sensor_stamped_s() {}

  sensor_stamped_s(hrt_abstime t, float d[3],
                   math::Vector<3> trans,
                   math::Matrix<3,3> rot,
                   uint8_t sens_type) {
    type=sens_type;
    ts=t;
    offset_t=trans;
    offset_r=rot;
    dat(0)=d[0];
    dat(1)=d[1];
    dat(2)=d[2];
    convert_frame();
  }

  void set_type(uint8_t sens_type) {
    type=sens_type;
  }

  //TODO use noise models (usually gwn)
  void update_bias() {

  }

  void set_offset(math::Vector<3> t, math::Matrix<3,3> r) {
    offset_t = t;
    offset_r = r;
  }

  //TODO convert data received into correct reference frame
  //     using offset_t and offset_r
  void convert_frame() {

  }

  void set_data(hrt_abstime t,float s[3]) {
    ts=t;
    update_bias();
    dat(0) = s[0];
    dat(1) = s[1];
    dat(2) = s[2];
    convert_frame();
  }

  void set_data(hrt_abstime t,math::Vector<3> s) {
    ts=t;
    update_bias();
    dat(0) = s(0);
    dat(1) = s(1);
    dat(2) = s(2);
    convert_frame();
  }

  const math::Vector<3> *get_data() {
    return &dat;
  }

  const math::Vector<3> *get_bias() {
    return &bias;
  }
};

// Inertial frame pose
// Currently storing body axis angle rates as pose 10,11,12
// Since other pose members are wrt inertial frame, these should
// also be stored wrt inertial frame
struct pose_stamped_s {
  math::Vector<STATE_LEN>pose;
  // If using pose_stamped_s as error, keep inertial rotation as well
  // as rotation error
  math::Quaternion q; // Actual rotation
  math::Vector<3> eul,body_pos,body_vel;
  math::Matrix<3,3> rot;
  math::Vector<3> offset_t;
  math::Matrix<3,3> offset_r;
  hrt_abstime init_time,t[STATE_LEN];
  uint16_t base_thrust=0;
  bool error_pose=false,initialized,timeout,hold,land;

  pose_stamped_s() {
    initialized=false;
    memset(t,0,sizeof(t));
  }
  pose_stamped_s(hrt_abstime tau,
                 math::Vector<STATE_LEN> *v) {
    initialized=false;
    set_pose(tau,v);
  }
  //Use this initializer if using pose as setpoint
  pose_stamped_s(hrt_abstime tau,
      uint8_t param_mask,
      math::Vector<STATE_LEN> *v,
      uint16_t thrust) {
    initialized=false;
    timeout=false;
    hold=param_mask&SETPOINT_PARAM_HOLD;
    land=param_mask&SETPOINT_PARAM_LAND;
    set_pose(tau,v);
    base_thrust=thrust;
  }
  pose_stamped_s(pose_stamped_s *p) {
    initialized=false;
    set_pose(p);
  }
  // Update pose parameters
  // Useful for setpoints
  void update() {
    if (!initialized) {
      initialized=true;
      init_time=hrt_absolute_time();
    }
    if (!hold &&
        init_time+t[0]
        <hrt_absolute_time()) {
      timeout=true;
    }
  }
  // Offset rot in Euler angles
  // Offset trans in meters
  void set_offset(math::Vector<3> *offset_rot,
                  math::Vector<3> *offset_trans) {
    offset_r.from_euler((*offset_rot)(0),
                        (*offset_rot)(1),
                        (*offset_rot)(2));
    offset_t=*offset_trans;
  }
  void set_pose(hrt_abstime tau[STATE_LEN],
                math::Vector<STATE_LEN> *v) {
    for (uint8_t i=0; i < STATE_LEN; i++)
      t[i]=tau[i];
    pose=*v;
  }
  void set_pose(hrt_abstime tau, math::Vector<STATE_LEN> *v) {
    for (uint8_t i=0; i < STATE_LEN; i++)
      t[i]=tau;
    pose=*v;
  }
  void set_pose(pose_stamped_s *p) {
    for (uint8_t i=0; i < STATE_LEN; i++)
      t[i]=p->t[i];
    pose=p->pose;
  }
  void set_pose(uint8_t n, hrt_abstime tau, float f) {
    if (n<STATE_LEN) {
      t[n]=tau;
      pose(n)=f;
    }
  }
  float pos_norm() {
    return sqrtf(pose(1)*pose(1)+pose(2)*pose(2)+pose(3)*pose(3));
  }
  // Transform a vector by a rotation and a translation
  static math::Vector<3>transform(
      math::Matrix<3,3> *r,
      math::Vector<3> *t,
      math::Vector<3> *s) {
    return (*r)*(*s)+*t;
  }
  // Offset transform for position vector
  void offset_transform() {
    math::Vector<3>tmp = {pose(0),pose(1),pose(2)};
    tmp = transform(
            &offset_r,
            &offset_t,
            &tmp);
    pose(0)=tmp(0);
    pose(1)=tmp(1);
    pose(2)=tmp(2);
  }
  void update_rot() {
    // Update rotation with current direction quaternion
    q(0)=pose(6);
    q(1)=pose(7);
    q(2)=pose(8);
    q(3)=pose(9);
    //TODO clean up
    eul=q.to_euler();
    rot.from_euler(eul(0),eul(1),eul(2));
    rot=rot;

    // Update body position & velocity
    body_pos(0)=pose(0);
    body_pos(1)=pose(1);
    body_pos(2)=pose(2);
    body_pos=rot*body_pos;
    body_vel(0)=pose(3);
    body_vel(1)=pose(4);
    body_vel(2)=pose(5);
    body_vel=rot*body_vel;
  }
  math::Matrix<3,3>get_rot(){
    update_rot();
    return rot;
  }
  math::Vector<3>get_eul() {
    update_rot();
    return eul;
  }
  void set_eul(float r,float p,float y) {
    eul={r,p,y};
    q.from_euler(r,p,y);
    pose(6)=q(0);
    pose(7)=q(1);
    pose(8)=q(2);
    pose(9)=q(3);
  }
  void set_eul(math::Vector<3> v) {
    set_eul(v(0),v(1),v(2));
  }
  // For now just do yaw correction
  math::Vector<3>get_body_pose(uint8_t sect) {
    update_rot();
    // Collect correct section, then perform
    // offset transformation if necessary.
    // For now, only perform offset rotation,
    // bc get_body() only being used with error state vector
    if (sect==SECT_POS) {
      return body_pos;
    } else if (sect==SECT_VEL) {
      return body_vel;
    } else if (sect==SECT_ANG) {
      return eul;
    } else if (sect==SECT_ANG_RATE) {
      return math::Vector<3>({pose(10),pose(11),pose(12)});
    } else {
      return math::Vector<3>();
    }
  }

  math::Vector<12>get_body_pose() {
    math::Vector<12> s;
    update_rot();
    s(0)=body_pos(0);
    s(1)=body_pos(1);
    s(2)=body_pos(2);
    s(3)=body_vel(0);
    s(4)=body_vel(1);
    s(5)=body_vel(2);
    s(6)=eul(0);
    s(7)=eul(1);
    s(8)=eul(2);
    s(9)=pose(10);
    s(10)=pose(11);
    s(11)=pose(12);
    return s;
  }

  uint16_t get_base_thrust() {
    return base_thrust;
  }
};

class elka::BasicEstimator {
private:
  sensor_stamped_s _prev_acc;
  sensor_stamped_s _prev_gyro;
  sensor_stamped_s _prev_cam;

  pose_stamped_s _curr_pose;
  pose_stamped_s _prev_pose;
  pose_stamped_s _prev_filt_pose;

public:
  BasicEstimator();
  ~BasicEstimator();

  void set_prev_inert_sens(sensor_combined_s *s);
  pose_stamped_s *get_pose();
  float get_pose(uint8_t n);
  void set_pose(hrt_abstime t,math::Vector<STATE_LEN>*v);
  void set_pose(hrt_abstime t[STATE_LEN],math::Vector<STATE_LEN>*v);
  void set_pose(pose_stamped_s *p);
  void set_pose(uint8_t n, hrt_abstime t, float f);
  void update_prev_pose();
  void update_prev_pose(uint8_t n);

  /* xdot = alpha*xdot_filt_t-1 + (1-alpha)*xdot_raw_t
   * where:
   *    xdot_raw_t = 1/(del_t)*(x_curr-x_prev)
   *    alpha scales between 0,1
   *      Higher alpha -> higher phase lag, smoother estimate
   *      Lower alpha  -> lower phase lag, estimates mirror raw data
   */
  void low_pass_filt(uint8_t *filter_states);
  void ekf();
};

#endif
