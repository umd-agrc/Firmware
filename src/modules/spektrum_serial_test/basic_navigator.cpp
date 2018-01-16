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
  reset_setpoints();
  // Rotation from sf to elka
  math::Vector<3> offset_r = {0,0,M_PI_2_F};
  // Translation from sf to elka
  math::Vector<3> offset_t = {-0.13,0,-0.05};
  set_fcu_offset(&offset_r,&offset_t);
  _at_setpoint=false;_new_setpoint=false;
  _wait=false;_landed=false;_from_manual=false;_kill=false;
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
  _curr_err.set_offset(r,t);
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

void elka::BasicNavigator::next_setpoint() {
  if (!_setpoints.empty()) {
    _setpoints.front().update();
    // Handle case setpoint timeout
    if (_setpoints.front().timeout) {
      _setpoints.erase(_setpoints.begin());
      usleep(5000);
      if (!_setpoints.empty()) {
        _setpoints.front().update();
      }
    }
    update_error(&_setpoints.front());
    bool b=at_setpoint();
    // Handle case at setpoint 
    // TODO clean this up
    if (b) {
      if (_setpoints.front().land)
        _landed=true;
      else _landed = false;
      if (!_setpoints.front().hold) {
        _setpoints.erase(_setpoints.begin());
        if (!_setpoints.empty()) {
          _setpoints.front().update();
          update_error(&_setpoints.front());
        }
      }
    } else _landed=false;
  }
}

void elka::BasicNavigator::set_prev_inert_sens(
    sensor_combined_s *s) {
  _est.set_prev_inert_sens(s);
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

void elka::BasicNavigator::update_pose(vehicle_local_position_s *p,
                                       vehicle_attitude_s *a) {
  hrt_abstime t_p=p->timestamp,
              t_a=a->timestamp;
  float *q=a->q; // yaw cw about down axis
  float x=p->x,
        y=p->y,
        z=p->z;
        //vx=p->vx,
        //vy=p->vy,
        //vz=p->vz,
        //rs=a->rollspeed,
        //ps=a->pitchspeed,
        //ys=a->yawspeed;
  // Low pass derivative filter on vx,vy,vz
  static uint8_t filt_states[STATE_LEN]=
      {FILT_NONE,FILT_NONE,FILT_NONE,
       FILT_DERIV,FILT_DERIV,FILT_DERIV,
       FILT_NONE,FILT_NONE,FILT_NONE,FILT_NONE,
       FILT_DERIV,FILT_DERIV,FILT_DERIV};

  static math::Vector<3>inert_pos, sf_elka_t_curr;
  static math::Matrix<3,3>sf_rot;

  _est.update_prev_pose();

  // Update angles and angle rates
  _est.set_pose(6,t_a,*q);
  _est.set_pose(7,t_a,*(q+1));
  _est.set_pose(8,t_a,*(q+2));
  _est.set_pose(9,t_a,*(q+3));

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

  // Set Elka inertial error and next setpoint
  // In this case, we don't set angle error
  // Set absolute angle so that we can retrieve body position
  // from absolute angles
  next_setpoint();

#if defined(ELKA_DEBUG) && defined(DEBUG_TRANSFORM)
    PX4_INFO("curr_err: %3.3f,%3.3f,%3.3f",
           _curr_err.pose(0),_curr_err.pose(1),_curr_err.pose(2));
#endif
}

void elka::BasicNavigator::add_setpoint(
	hrt_abstime dt,uint8_t param_mask,math::Vector<STATE_LEN>*v) {
	math::Vector<3> offset_rot,
									offset_trans;
	offset_rot=
		_elka_sf_r.inversed().to_euler();
	offset_trans=-_elka_sf_t;
	pose_stamped_s p(dt,param_mask,v);
  p.set_offset(&offset_rot,
							 &offset_trans);
	_setpoints.push_back(p);
}

uint8_t elka::BasicNavigator::takeoff(float z,bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
  float eps;
  uint8_t param_mask=0;
	t=TAKEOFF_SETPOINT_DEFAULT_LEN;
	tmp=get_pose()->pose;
  tmp(3)=0;
	tmp(4)=0;
	tmp(5)=0;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
  float dz=0;
  tmp(2)=dz;
  eps=fabs(z-dz);
  if (eps<POSITION_EPSILON)
    if (hold) param_mask|=SETPOINT_PARAM_HOLD;
  add_setpoint(t,param_mask,&tmp);
  while (eps>POSITION_EPSILON) {
    dz=dz+(z-dz)/2;
    eps=fabs(z-dz);
    tmp(2)=dz;
    if (eps<POSITION_EPSILON)
      if (hold) param_mask|=SETPOINT_PARAM_HOLD;
    add_setpoint(t,param_mask,&tmp);
  }
  return ELKA_SUCCESS;
}
uint8_t elka::BasicNavigator::hover(bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
  uint8_t param_mask=0;
  if (hold) param_mask|=SETPOINT_PARAM_HOLD;
	t=SETPOINT_DEFAULT_LEN;
  // For hover, keep current x,y,z
	tmp=get_pose()->pose;
	tmp(3)=0;
	tmp(4)=0;
	tmp(5)=0;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
  add_setpoint(t,param_mask,&tmp);
  return ELKA_SUCCESS;
}

uint8_t elka::BasicNavigator::land(bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
  uint8_t param_mask=0;
	t=LANDING_SETPOINT_DEFAULT_LEN;
	tmp=get_pose()->pose;
  tmp(3)=0;
	tmp(4)=0;
	tmp(5)=0;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
  float z=tmp(2);
  // Check if landed. Can land lower than LAND_DEFAULT_HEIGHT,
  // but not higher
  if (z-LAND_DEFAULT_HEIGHT<LANDING_HEIGHT_EPSILON) {
    if (hold) param_mask|=SETPOINT_PARAM_LAND|SETPOINT_PARAM_HOLD;
  }
  add_setpoint(t,param_mask,&tmp);
  while (z-LAND_DEFAULT_HEIGHT>LANDING_HEIGHT_EPSILON) {
    z-=(z-LAND_DEFAULT_HEIGHT)/1.5;
    tmp(2)=z;
    if (z-LAND_DEFAULT_HEIGHT<LANDING_HEIGHT_EPSILON) {
      if (hold) param_mask|=SETPOINT_PARAM_LAND|SETPOINT_PARAM_HOLD;
    }
    add_setpoint(t,param_mask,&tmp);
  }
  return ELKA_SUCCESS;
}

void elka::BasicNavigator::reset_setpoints() {
  math::Vector<STATE_LEN> tmp=math::Vector<STATE_LEN>();
  hrt_abstime t=hrt_absolute_time();
  _curr_err.set_pose(t,&tmp);
	_setpoints.clear();
	
  _at_setpoint=false;
  _new_setpoint=true;
}

bool elka::BasicNavigator::at_setpoint() {
  // Retrieve norm of position error as norm error
  float norm_curr_err = _curr_err.pos_norm();
  if (norm_curr_err < POSITION_ERROR_THRES)
    _at_setpoint = true;
  else _at_setpoint = false;

  return _at_setpoint;
}

void elka::BasicNavigator::update_error(pose_stamped_s *curr_setpoint) {
  if (curr_setpoint) {
    _curr_err.pose=_est.get_pose()->pose-curr_setpoint->pose;
    _curr_err.pose(6)=_est.get_pose(6);
    _curr_err.pose(7)=_est.get_pose(7);
    _curr_err.pose(8)=_est.get_pose(8);
    _curr_err.pose(9)=_est.get_pose(9);
  } else {
    _curr_err.pose.zero();
    //TODO move to next_setpoint() function
    _wait=true;
  }
}

int8_t elka::BasicNavigator::generate_setpoints(
    std::vector<math::Vector<POSITION_LEN>> p) {
  float s[STATE_LEN];
  math::Vector<STATE_LEN> v;
  hrt_abstime dt=SETPOINT_DEFAULT_LEN;
  uint8_t param_mask=0;
  for (auto it=p.begin(); it!=p.end(); it++) {
    s[0]=(*it)(0);
    s[1]=(*it)(1);
    s[2]=(*it)(2);
    v=math::Vector<STATE_LEN>(s);
    add_setpoint(dt,param_mask,&v);
  }
  return ELKA_SUCCESS;
}

// TODO update to include yaw and yaw rate
void elka::BasicNavigator::print_setpoints() {
  if (_setpoints.empty())
    PX4_INFO("No setpoints");
  else
    PX4_INFO("Setpoints:");
  uint16_t i=0;
  for (auto it=_setpoints.begin();it!=_setpoints.end();it++) {
    PX4_INFO("%d: %f %f %f,params: %d %d dt %" PRIu64 "",
        i,it->pose(0),it->pose(1),it->pose(2),
        it->land,it->hold,it->t[0]);
    i++;
  }
}

// TODO would be nice to parameterize copy to allow
// for copy of a chosen pose (eg error, setpoint, etc)
void elka::BasicNavigator::copy_pose_error(
    vehicle_local_position_s *p,
    vehicle_attitude_s *a) {
  math::Vector<3> vtmp;
  math::Quaternion qtmp;
  vtmp=_curr_err.get_body_pose(SECT_POS);  
  p->x = vtmp(0); 
  p->y = vtmp(1); 
  p->z = vtmp(2); 

  vtmp=_curr_err.get_body_pose(SECT_VEL);  
  p->vx = vtmp(0); 
  p->vy = vtmp(1); 
  p->vz = vtmp(2); 

  a->q[0] = _curr_err.pose(6); 
  a->q[1] = _curr_err.pose(7); 
  a->q[2] = _curr_err.pose(8); 
  a->q[3] = _curr_err.pose(9); 

  vtmp=_curr_err.get_body_pose(SECT_ANG_RATE);
  a->rollspeed=vtmp(0);
  a->pitchspeed=vtmp(1);
  a->yawspeed=vtmp(2);
}
