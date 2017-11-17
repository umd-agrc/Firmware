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
  reset_setpoint();
  // Rotation from sf to elka
  math::Vector<3> offset_r = {0,0,M_PI_2_F};
  // Translation from sf to elka
  math::Vector<3> offset_t = {-0.13,0,-0.05};
  set_fcu_offset(&offset_r,&offset_t);
}

elka::BasicNavigator::~BasicNavigator() {
}

void elka::BasicNavigator::set_fcu_offset(
    math::Vector<3> *r, math::Vector<3> *t) {
  // Get rotation from elka to sf
  _elka_sf_r.from_euler((*r)(0),
                        (*r)(1),
                        (*r)(2));
  _elka_sf_r=_elka_sf_r.inversed();
  // Get translation from elka to sf
  _elka_sf_t=-(*t);

  // Set offsets for estimator
  _est.get_pose()->set_offset(r,t);
  _setpoint.set_offset(r,t);
  _curr_err.set_offset(r,t);
}

bool elka::BasicNavigator::atSetpoint() {
  return true;
}

pose_stamped_s *elka::BasicNavigator::get_pose() {
  return _est.get_pose();
}

float elka::BasicNavigator::get_pose(uint8_t n){
  return _est.get_pose(n);
}

pose_stamped_s *elka::BasicNavigator::get_err() {
  return &_curr_err;
}

float elka::BasicNavigator::get_err(uint8_t n){
  if (n > STATE_LEN) {
    PX4_ERR("Invalid err index %d",n);
    return PX4_ERROR;
  } else
    return _curr_err.pose(n);
}

pose_stamped_s *elka::BasicNavigator::get_prev_min_err() {
  return &_prev_min_err;
}

float elka::BasicNavigator::get_prev_min_err(uint8_t n){
  if (n > STATE_LEN) {
    PX4_ERR("Invalid prev min err index %d",n);
    return PX4_ERROR;
  } else
    return _prev_min_err.pose(n);
}

pose_stamped_s *elka::BasicNavigator::get_setpoint() {
  return &_setpoint;
}

float elka::BasicNavigator::get_setpoint(uint8_t n){
  if (n > STATE_LEN) {
    PX4_ERR("Invalid err index %d",n);
    return PX4_ERROR;
  } else
    return _setpoint.pose(n);
}

void elka::BasicNavigator::set_pose(hrt_abstime t,
                                    math::Vector<STATE_LEN>*v) {
  _est.set_pose(t,v);  
}

void elka::BasicNavigator::set_pose(hrt_abstime t[STATE_LEN],
                                    math::Vector<STATE_LEN>*v) {
  _est.set_pose(t,v);  
}

void elka::BasicNavigator::set_pose(pose_stamped_s *p) {
  _est.set_pose(p);  
}

void elka::BasicNavigator::set_err(hrt_abstime t,
                                   math::Vector<STATE_LEN>*v) {
  _curr_err.set_pose(t,v);
}

void elka::BasicNavigator::set_err(hrt_abstime t[STATE_LEN],
                                   math::Vector<STATE_LEN>*v) {
  _curr_err.set_pose(t,v);
}

void elka::BasicNavigator::set_err(pose_stamped_s *p) {
  _curr_err.set_pose(p);
}

void elka::BasicNavigator::set_prev_min_err(
    hrt_abstime t,
    math::Vector<STATE_LEN>*v) {
  _prev_min_err.set_pose(t,v);
}

void elka::BasicNavigator::set_prev_min_err(
    hrt_abstime t[STATE_LEN],
    math::Vector<STATE_LEN>*v) {
  _prev_min_err.set_pose(t,v);
}

void elka::BasicNavigator::set_prev_min_err(pose_stamped_s *p) {
  _prev_min_err.set_pose(p);
}

void elka::BasicNavigator::set_setpoint(
    hrt_abstime t,
    math::Vector<STATE_LEN>*v) {
  _setpoint.set_pose(t,v);
}

void elka::BasicNavigator::set_setpoint(
    hrt_abstime t[STATE_LEN],
    math::Vector<STATE_LEN>*v) {
  _setpoint.set_pose(t,v);
}

void elka::BasicNavigator::set_setpoint(pose_stamped_s *p) {
  _setpoint.set_pose(p);
}

void elka::BasicNavigator::set_new_setpoint(bool b) {
  _new_setpoint=b;
}

void elka::BasicNavigator::update_pose(vehicle_local_position_s *p,
                                       vehicle_attitude_s *a) {
  hrt_abstime t_p=p->timestamp,
              t_a=a->timestamp;
  float *q=a->q; // yaw cw about down axis
  float x=p->x,
        y=p->y,
        z=p->z,
        vx=p->vx,
        vy=p->vy,
        vz=p->vz,
        rs=a->rollspeed,
        ps=a->pitchspeed,
        ys=a->yawspeed;
  // Low pass derivative filter on vx,vy,vz
  static uint8_t filt_states[STATE_LEN]=
      {FILT_NONE,FILT_NONE,FILT_NONE,
       FILT_DERIV,FILT_DERIV,FILT_DERIV,
       FILT_NONE,FILT_NONE,FILT_NONE,FILT_NONE,
       FILT_NONE,FILT_NONE,FILT_NONE};

  static math::Vector<3>inert_pos, sf_elka_t_curr;
  static math::Matrix<3,3>sf_rot;

  _est.update_prev_pose();

  // Update angles and angle rates
  _est.set_pose(6,t_a,*q);
  _est.set_pose(7,t_a,*(q+1));
  _est.set_pose(8,t_a,*(q+2));
  _est.set_pose(9,t_a,*(q+3));
  _est.set_pose(10,t_a,rs);
  _est.set_pose(11,t_a,ps);
  _est.set_pose(12,t_a,ys);

  // Update inertial snapdragon pose to align with ELKA inertial frame
  // In Elka inertial frame, use elka origin, forward-right-down coords
  // Set x to sensed y and y to sensed -x b/c initial yaw is pi/2 rad
  // for VISLAM
  inert_pos(0)=y+_elka_sf_t(0);
  inert_pos(1)=-x+_elka_sf_t(1);
  inert_pos(2)=z+_elka_sf_t(2);

  // Get rotation 
  sf_rot=_est.get_pose()->get_rot();

  // Get body frame translation offset from sf->elka
  sf_elka_t_curr=-_elka_sf_r*sf_rot*_elka_sf_t;

  inert_pos += sf_elka_t_curr;

#if defined(ELKA_DEBUG) && defined(DEBUG_TRANSFORM)
  /*
  math::Vector<3>eul=_est.get_pose()->get_eul();
  PX4_INFO("euler angles: %3.3f,%3.3f,%3.3f",
           eul(0),eul(1),eul(2));
  PX4_INFO("sf_elka_t_curr: %3.3f,%3.3f,%3.3f",
           sf_elka_t_curr(0),sf_elka_t_curr(1),sf_elka_t_curr(2));
  PX4_INFO("inert_pos: %3.3f,%3.3f,%3.3f",
           inert_pos(0),inert_pos(1),inert_pos(2));
           */
#endif

  // Set estimator position
  _est.set_pose(0,t_p,inert_pos(0));
  _est.set_pose(1,t_p,inert_pos(1));
  _est.set_pose(2,t_p,inert_pos(2));

  // Low pass filter to set estimator velocity
  _est.low_pass_filt(filt_states);

  //FIXME Inefficient way to do this.
  //      Should happen all at once.
  // Set Elka inertial error
  // In this case, we don't set angle error, just angle so that
  // we can retrieve body position from absolute angles
  _curr_err.pose=_est.get_pose()->pose-_setpoint.pose;
  _curr_err.pose(6)=_est.get_pose(6);
  _curr_err.pose(7)=_est.get_pose(7);
  _curr_err.pose(8)=_est.get_pose(8);
  _curr_err.pose(9)=_est.get_pose(9);

#if defined(ELKA_DEBUG) && defined(DEBUG_TRANSFORM)
    PX4_INFO("curr_err: %3.3f,%3.3f,%3.3f",
           _curr_err.pose(0),_curr_err.pose(1),_curr_err.pose(2));
#endif

  // Update prev sensor/SLAM reading
  _prev_sens.set_pose(0,t_p,x);
  _prev_sens.set_pose(1,t_p,y);
  _prev_sens.set_pose(2,t_p,z);
  _prev_sens.set_pose(3,t_p,vx);
  _prev_sens.set_pose(4,t_p,vy);
  _prev_sens.set_pose(5,t_p,vz);
  _prev_sens.set_pose(6,t_a,*q);
  _prev_sens.set_pose(7,t_a,*(q+1));
  _prev_sens.set_pose(8,t_a,*(q+2));
  _prev_sens.set_pose(9,t_a,*(q+3));
  _prev_sens.set_pose(10,t_a,rs);
  _prev_sens.set_pose(11,t_a,ps);
  _prev_sens.set_pose(12,t_a,ys);
}

void elka::BasicNavigator::reset_setpoint() {
  math::Vector<STATE_LEN> tmp=math::Vector<STATE_LEN>();
  hrt_abstime t=hrt_absolute_time();
  _curr_err.set_pose(t,&tmp);
  set_setpoint(t,&get_pose()->pose);
  _setpoint.set_pose(3,t,0);
  _setpoint.set_pose(4,t,0);
  _setpoint.set_pose(5,t,0);
  _setpoint.set_pose(10,t,0);
  _setpoint.set_pose(11,t,0);
  _setpoint.set_pose(12,t,0);
  _prev_min_err.set_pose(0,t,POSITION_ERROR_THRES);
  _prev_min_err.set_pose(1,t,POSITION_ERROR_THRES);
  _prev_min_err.set_pose(2,t,POSITION_ERROR_THRES);
  _prev_min_err.set_pose(3,t,VELOCITY_ERROR_THRES);
  _prev_min_err.set_pose(4,t,VELOCITY_ERROR_THRES);
  _prev_min_err.set_pose(5,t,VELOCITY_ERROR_THRES);
  _prev_min_err.set_pose(6,t,ANGLE_ERROR_THRES);
  _prev_min_err.set_pose(7,t,ANGLE_ERROR_THRES);
  _prev_min_err.set_pose(8,t,ANGLE_ERROR_THRES);
  _prev_min_err.set_pose(9,t,ANGLE_ERROR_THRES);
  _prev_min_err.set_pose(10,t,ANGLE_RATE_ERROR_THRES);
  _prev_min_err.set_pose(11,t,ANGLE_RATE_ERROR_THRES);
  _prev_min_err.set_pose(12,t,ANGLE_RATE_ERROR_THRES);
  _new_setpoint=true;
}

bool elka::BasicNavigator::check_setpoint() {
  // Retrieve norm of planar position error as norm error
  float norm_curr_err = _curr_err.norm(true),
        norm_prev_min_err = _prev_min_err.norm(true);
  if (_new_setpoint ||
      (norm_curr_err >
       10*norm_prev_min_err &&
       norm_curr_err > ERROR_THRES)) {
    reset_setpoint();
  } else if (norm_curr_err < norm_prev_min_err) {
    _prev_min_err.set_pose(&_curr_err);
  }

  return _new_setpoint;
}

// TODO would be nice to parameterize copy to allow
// for copy of a chosen pose (eg error, setpoint, etc)
void elka::BasicNavigator::copy_pose_error(
    vehicle_local_position_s *p,
    vehicle_attitude_s *a) {
  math::Vector<3> vtmp;
  math::Quaternion qtmp;
  vtmp=_curr_err.get_body(SECT_POS);  
  p->x = vtmp(0); 
  p->y = vtmp(1); 
  p->z = vtmp(2); 

  vtmp=_curr_err.get_body(SECT_VEL);  
  p->vx = vtmp(0); 
  p->vy = vtmp(1); 
  p->vz = vtmp(2); 

  vtmp=_curr_err.get_body(SECT_ANG);  
  qtmp.from_euler(vtmp(0),vtmp(1),vtmp(2));
  a->q[0] = qtmp(0); 
  a->q[1] = qtmp(1); 
  a->q[2] = qtmp(2); 
  a->q[3] = qtmp(3); 

  vtmp=_curr_err.get_body(SECT_ANG_RATE);
  a->rollspeed=vtmp(0);
  a->pitchspeed=vtmp(1);
  a->yawspeed=vtmp(2);
}
