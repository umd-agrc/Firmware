#ifndef BASIC_ESTIMATOR_H
#define BASIC_ESTIMATOR_H

#include <cstring>

#include <px4_defines.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <lib/mathlib/math/Quaternion.hpp>
#include <lib/mathlib/math/Matrix.hpp>

#include "serial_defines.h"

namespace elka {
  class BasicEstimator;
}

#define STATE_LEN 13

#define SECT_POS 0
#define SECT_VEL 1
#define SECT_ANG 2
#define SECT_ANG_RATE 3

#define SEC_TO_MICROSEC 10e6

#define FILT_NONE 0
#define FILT_STATE 1
#define FILT_DERIV 2

#define DT_MIN 0.02f

#define BAD_NUM(x) (isnan(abs(x)) || isinf(abs(x)))

struct pose_stamped_s {
  hrt_abstime t[STATE_LEN];
  math::Vector<STATE_LEN>pose;
  math::Quaternion q;
  math::Vector<3> eul;
  math::Matrix<3,3> rot;
  math::Vector<3> offset_t;
  math::Matrix<3,3> offset_r;

  pose_stamped_s() {
    memset(t,0,sizeof(t));
  }
  pose_stamped_s(hrt_abstime tau,
                 math::Vector<STATE_LEN> *v) {
    set_pose(tau,v);
  }
  pose_stamped_s(pose_stamped_s *p) {
    set_pose(p);
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
  float norm(bool planar_pos) {
    if (planar_pos)
      return sqrtf(pose(1)*pose(1)+pose(2)*pose(2));
    else
    return pose.length();
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
    q(0)=pose(6);
    q(1)=pose(7);
    q(2)=pose(8);
    q(3)=pose(9);
    eul=q.to_euler();
    //TODO Update to 3-axis rot eventually
    //rot.from_euler(eul(0),eul(1),eul(2));
    rot.from_euler(eul(0),eul(1),eul(2));
    rot=rot.inversed();
  }
  math::Matrix<3,3>get_rot(){
    update_rot();
    return rot;
  }
  math::Vector<3>get_eul() {
    update_rot();
    return eul;
  }
  // For now just do yaw correction
  math::Vector<3>get_body(uint8_t sect) {
    update_rot();
    math::Vector<3> s;

    // Collect correct section, then perform
    // offset transformation if necessary.
    // For now, only perform offset rotation,
    // bc get_body() only being used with error state vector
    if (sect==SECT_POS) {
      s(0)=pose(0);
      s(1)=pose(1);
      s(2)=pose(2);
      //FIXME is this rotation correct?
      s=rot*s;
    } else if (sect==SECT_VEL) {
      s(0)=pose(3);
      s(1)=pose(4);
      s(2)=pose(5);
      //FIXME is this rotation correct?
      s=rot*s;
    } else if (sect==SECT_ANG) {
      s(0)=eul(0);
      s(1)=eul(1);
      s(2)=eul(2);
    } else if (sect==SECT_ANG_RATE) {
      s(0)=pose(10);
      s(1)=pose(11);
      s(2)=pose(12);
    }

    return s;
  }
};

class elka::BasicEstimator {
private:
  pose_stamped_s _curr_pose;
  pose_stamped_s _prev_pose;
  pose_stamped_s _prev_filt_pose;

public:
  BasicEstimator();
  ~BasicEstimator();

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
};

#endif
