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
  math::Vector<3> offset_t = {-0.094,0,-0.04};
  set_fcu_offset(&offset_r,&offset_t);
  _at_setpoint=false;_new_setpoint=false;
  _wait=false;_landed=false;_from_manual=false;_kill=false;
  // Set _curr_err member as error pose type
  _curr_err.error_pose=true;

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
      if (_setpoints.front().land) _landed=true;
      else _landed = false;
      if (!_setpoints.front().hold) {
        _setpoints.erase(_setpoints.begin());
        usleep(5000);
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

math::Vector<12> elka::BasicNavigator::get_body_pose_error() {
  return _curr_err.get_body_pose();
}

void elka::BasicNavigator::update_pose(vehicle_local_position_s *p,
                                       vehicle_attitude_s *a,
                                       vision_velocity_s *v) {
  hrt_abstime t_p=p->timestamp,
              t_a=a->timestamp,
              t_v=v->timestamp;
  float x=p->x,
        y=p->y,
        z=p->z,
        vx=v->vx,
        vy=v->vy,
        vz=v->vz;
  math::Vector<3>inert_pos,inert_vel,
    body_axis_rate_curr,
    inertial_angle_rate_curr,
    sf_elka_t_curr,
    sf_elka_radii,
    sf_elka_inert_rot,
    eul;
  math::Matrix<3,3>sf_rot;
  math::Quaternion q;
  if (!_nav_init) _nav_init=true;
  _est.update_prev_pose();

  // Update angles and angle rates
  // Update angles first so that transformation from body-axis to
  // euler-angle rates can be performed wrt most recent orientation
  //TODO make this neater
  q=math::Quaternion(a->q);
  eul=q.to_euler();
  sf_rot.from_euler(eul(0),eul(1),eul(2));
  eul=(_elka_sf_r*sf_rot).to_euler();
  q.from_euler(eul(0),eul(1),eul(2));
  _est.set_pose(6,t_a,q(0));
  _est.set_pose(7,t_a,q(1));
  _est.set_pose(8,t_a,q(2));
  _est.set_pose(9,t_a,q(3));

  body_axis_rate_curr={v->rollspeed,-v->pitchspeed,-v->yawspeed};
  _est.set_pose(10,t_v,body_axis_rate_curr(0));
  _est.set_pose(11,t_v,body_axis_rate_curr(1));
  _est.set_pose(12,t_v,body_axis_rate_curr(2));

  // Update inertial snapdragon pose to align with ELKA inertial frame
  // In Elka inertial frame, use elka origin, forward-right-down coords
  // Set x to sensed y and y to sensed -x b/c initial yaw is pi/2 rad
  // for VISLAM
  inert_pos(0)=y+_elka_sf_t(0);
  inert_pos(1)=-x+_elka_sf_t(1);
  inert_pos(2)=z+_elka_sf_t(2);

  inert_vel(0)=vx;
  inert_vel(1)=-vy;
  inert_vel(2)=-vz;

  // Get rotation 
  sf_rot=_est.get_pose()->get_rot().inversed();

  // Get inertial frame frame translation offset from sf->elka
  sf_elka_t_curr=-_elka_sf_r*sf_rot*_elka_sf_t;

  // Transform position and velocity to ELKA-centered
  inert_pos += sf_elka_t_curr;

  // Compute inertial axis radii of sf_elka offset:
  //  radius xy
  //  radius yz
  //  radius zx 
  sf_elka_radii={
    sqrtf(pow(sf_elka_t_curr(0),2)+pow(sf_elka_t_curr(1),2)),
    sqrtf(pow(sf_elka_t_curr(1),2)+pow(sf_elka_t_curr(2),2)),
    sqrtf(pow(sf_elka_t_curr(0),2)+pow(sf_elka_t_curr(2),2))
  };

  // FIXME domain error if both inputs to atan2 are 0
  // Compute inertial axis rotation angles:
  //  theta xy, (q1 back->left)
  //  theta yz, (q1 left->up)
  //  theta zx, (q1 up->back)
  sf_elka_inert_rot={
      atan2(sf_elka_t_curr(1),sf_elka_t_curr(0)),
      atan2(sf_elka_t_curr(2),sf_elka_t_curr(1)),
      atan2(sf_elka_t_curr(0),sf_elka_t_curr(2))
  };

  // Get inertial frame angle rates
  inertial_angle_rate_curr=
    -_elka_sf_r*sf_rot*body_axis_rate_curr;

  // x = -r_xy*yaw(cw)-r_zx*pitch(back)
  inert_vel(0) -= 
    -sf_elka_radii(0)*sin(sf_elka_inert_rot(0))
      *inertial_angle_rate_curr(2)
    +sf_elka_radii(2)*cos(sf_elka_inert_rot(2))
      *inertial_angle_rate_curr(1);
  // y += r_yz*roll(right) - r_xy*yaw(cw)
  inert_vel(1) -= 
    -sf_elka_radii(0)*cos(sf_elka_inert_rot(0))
      *inertial_angle_rate_curr(2)
    -sf_elka_radii(1)*sin(sf_elka_inert_rot(1))
      *inertial_angle_rate_curr(0);
  // z += r_xz*pitch(back) - r_yz*roll(right)
  inert_vel(2) -= 
    sf_elka_radii(1)*cos(sf_elka_inert_rot(1))
      *inertial_angle_rate_curr(0)
    -sf_elka_radii(2)*sin(sf_elka_inert_rot(2))
      *inertial_angle_rate_curr(1);

#if defined(ELKA_DEBUG) && defined(DEBUG_TRANSFORM)
  math::Vector<3>euler=_est.get_pose()->get_eul();
  PX4_INFO("euler angles: %3.3f,%3.3f,%3.3f",
      euler(0),euler(1),euler(2));
  /*
  PX4_INFO("body axis rates: %3.3f,%3.3f,%3.3f",
           body_axis_rate_curr(0),body_axis_rate_curr(1),
           body_axis_rate_curr(2));
  PX4_INFO("sf_elka_t_curr: %3.3f,%3.3f,%3.3f",
           sf_elka_t_curr(0),sf_elka_t_curr(1),sf_elka_t_curr(2));
  PX4_INFO("inert_pos: %3.3f,%3.3f,%3.3f",
           inert_pos(0),inert_pos(1),inert_pos(2));
  PX4_INFO("inert ang: %3.3f,%3.3f,%3.3f",
      inertial_angle_rate_curr(0),inertial_angle_rate_curr(1),
      inertial_angle_rate_curr(2));
  PX4_INFO("inert_vel: %3.3f,%3.3f,%3.3f",
           inert_vel(0)*100,inert_vel(1)*100,inert_vel(2)*100);
           */
#endif

  // Set estimator position
  _est.set_pose(0,t_p,inert_pos(0));
  _est.set_pose(1,t_p,inert_pos(1));
  _est.set_pose(2,t_p,inert_pos(2));
  _est.set_pose(3,t_p,inert_vel(0));
  _est.set_pose(4,t_p,inert_vel(1));
  _est.set_pose(5,t_p,inert_vel(2));

#if defined(ELKA_DEBUG) && defined(DEBUG_VISION_VELOCITY)
  /*
        PX4_INFO("lin vel: %f %f %f \t ang vel: %f %f %f",
            v->vx,v->vy,v->vz,
            v->rollspeed,v->pitchspeed,
            v->yawspeed);
            */
  PX4_INFO("ang vel: %f %f %f",
      v->rollspeed*100,v->pitchspeed*100,v->yawspeed*100);
  /*
  PX4_INFO("lin vel: %f %f %f",
      inert_vel(0),inert_vel(1),inert_vel(2));
      */
#endif

  // Set Elka inertial error and next setpoint
  // In this case, we don't set angle error
  // Set absolute angle so that we can retrieve body position
  // from absolute angles
  next_setpoint();
}

void elka::BasicNavigator::add_setpoint(
	hrt_abstime dt,
  uint8_t param_mask,
  math::Vector<STATE_LEN>*v,
  uint16_t base_thrust) {
	math::Vector<3> offset_rot,
									offset_trans;
	offset_rot=
		_elka_sf_r.inversed().to_euler();
	offset_trans=-_elka_sf_t;
	pose_stamped_s p(dt,param_mask,v,base_thrust);
  p.set_offset(&offset_rot,
							 &offset_trans);
	_setpoints.push_back(p);
}

int8_t elka::BasicNavigator::trajectory(PlanElement::plan_element_params* params) {
  math::Vector<SETPOINT_MAP_LEN> start_pos, start_vel, start_acc,
                                 end_pos, end_vel, end_acc;
  uint16_t base_thrust = HOVER_DEFAULT_THRUST;
  uint8_t param_mask=0;
  hrt_abstime dt = params->_dt;
  pose_stamped_s* p = get_pose();
  math::Vector<3> eul = p->get_eul();
  math::Vector<3> center; // useful for circles
  uint8_t num_setpoints; // useful for setpoint chains
  switch(params->_trajectory_type) {
  case PlanElement::TrajectoryTypes::Takeoff:
    start_pos(0) = p->pose(0);
    start_pos(1) = p->pose(1);
    start_pos(2) = p->pose(2);
    start_pos(3) = eul(2);
    start_vel.zero();
    start_vel(3) = VERTICAL_DEFAULT_SPEED;
    start_acc.zero();
    end_pos(0) = p->pose(0);
    end_pos(1) = p->pose(1);
    end_pos(2) = p->pose(2) + HOVER_DEFAULT_HEIGHT;
    end_pos(3) = eul(2);
    end_vel.zero();
    end_acc.zero();
    add_setpoint(dt,
      start_pos, start_vel, start_acc,
      end_pos, end_vel, end_acc,
      base_thrust, param_mask);
    break;
  case PlanElement::TrajectoryTypes::Land:
    start_pos(0) = p->pose(0);
    start_pos(1) = p->pose(1);
    start_pos(2) = p->pose(2);
    start_pos(3) = eul(2);
    start_vel.zero();
    start_vel(3) = -1.0*VERTICAL_DEFAULT_SPEED;
    start_acc.zero();
    end_pos(0) = p->pose(0);
    end_pos(1) = p->pose(1);
    end_pos(2) = LAND_DEFAULT_HEIGHT;
    end_pos(3) = eul(2);
    end_vel.zero();
    end_acc.zero();
    param_mask|=SETPOINT_PARAM_LAND;
    base_thrust = LAND_DEFAULT_THRUST;
    add_setpoint(dt,
      start_pos, start_vel, start_acc,
      end_pos, end_vel, end_acc,
      base_thrust, param_mask);
    break;
  case PlanElement::TrajectoryTypes::Hover:
    start_pos(0) = p->pose(0);
    start_pos(1) = p->pose(1);
    start_pos(2) = p->pose(2);
    start_pos(3) = eul(2);
    start_vel.zero();
    start_acc.zero();
    end_pos(0) = p->pose(0);
    end_pos(1) = p->pose(1);
    end_pos(2) = p->pose(2);
    end_pos(3) = eul(2);
    end_vel.zero();
    end_acc.zero();
    add_setpoint(dt,
      start_pos, start_vel, start_acc,
      end_pos, end_vel, end_acc,
      base_thrust, param_mask);
    break;
  case PlanElement::TrajectoryTypes::Line:
    start_pos = PlanElement::set_setpoint_map_point(params->_init_pos,eul(2));
    start_vel = PlanElement::set_setpoint_map_point(params->_init_vel,0);
    start_acc = PlanElement::set_setpoint_map_point(params->_init_acc,0);
    end_pos = PlanElement::set_setpoint_map_point(params->_final_pos,eul(2)+params->_dpsi);
    end_vel = PlanElement::set_setpoint_map_point(params->_final_vel,0);
    end_acc = PlanElement::set_setpoint_map_point(params->_final_acc,0);
    add_setpoint(dt,
      start_pos, start_vel, start_acc,
      end_pos, end_vel, end_acc,
      base_thrust, param_mask);
    break;
  case PlanElement::TrajectoryTypes::Circle:
    num_setpoints = 21;
    start_vel.zero();
    start_acc.zero();
    end_vel.zero();
    end_acc.zero();

    start_pos(0) = p->pose(0);
    start_pos(1) = p->pose(1);
    start_pos(2) = p->pose(2);
    end_pos(0) = p->pose(0);
    end_pos(1) = p->pose(1);
    end_pos(2) = p->pose(2);

    //TODO get_center()
    center = PlanElement::get_center(
        p,
        params->_trajectory_type,
        params->_trajectory_direction);

    //FIXME verify x forward, y backward
    //FIXME verify that first setpoint equals current position
    //FIXME include all direction possibilities
    for (uint8_t i=0; i<num_setpoints; i++) {
      switch (params->_trajectory_direction){
      case PlanElement::TrajectoryDirection::Fwd:
        start_pos = end_pos;
        end_pos(0) = center(0) - params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(1) - params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        break;
      case PlanElement::TrajectoryDirection::Back:
        end_pos(0) = center(0) + params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(1) + params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        break;
      case PlanElement::TrajectoryDirection::Left:
        end_pos(0) = center(0) + params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(1) - params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        break;
      case PlanElement::TrajectoryDirection::Right:
        end_pos(0) = center(0) - params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(1) + params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        break;
      case PlanElement::TrajectoryDirection::Up: 
        end_pos(0) = center(0) - params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(2) - params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        break;
      case PlanElement::TrajectoryDirection::Down:
        end_pos(0) = center(0) + params->_trajectory_radius*sin(i/num_setpoints*2*M_2_PI);
        end_pos(1) = center(2) + params->_trajectory_radius*cos(i/num_setpoints*2*M_2_PI);
        break;
      default:
        break;
      };

      // Set yaw
      end_pos(3) = eul(2) + params->_dpsi*(i/num_setpoints);
      
      //TODO add setpoint
    }
    break;
  case PlanElement::TrajectoryTypes::Square:
    break;
  case PlanElement::TrajectoryTypes::Spiral:
    break;
  case PlanElement::TrajectoryTypes::Spin:
    break;
  default:
    break;
  };

  return ELKA_SUCCESS;
}

void elka::BasicNavigator::add_setpoint(
      hrt_abstime dt,
      math::Vector<SETPOINT_MAP_LEN> &start_pos,
      math::Vector<SETPOINT_MAP_LEN> &start_vel,
      math::Vector<SETPOINT_MAP_LEN> &start_acc,
      math::Vector<SETPOINT_MAP_LEN> &end_pos,
      math::Vector<SETPOINT_MAP_LEN> &end_vel,
      math::Vector<SETPOINT_MAP_LEN> &end_acc,
      uint16_t base_thrust,
      uint8_t param_mask) {
  /*
	offset_rot=
		_elka_sf_r.inversed().to_euler();
	offset_trans=-_elka_sf_t;
	pose_stamped_s p(dt,param_mask,v,base_thrust);
  p.set_offset(&offset_rot,
							 &offset_trans);
               */

  setpoint_s s(
      dt,
      start_pos,
      start_vel,
      start_acc,
      end_pos,
      end_vel,
      end_acc,
      base_thrust,
      param_mask);
  _setpoints_new.push_back(s);
}

void elka::BasicNavigator::add_setpoint(setpoint_s *s) {
  /*
	offset_rot=
		_elka_sf_r.inversed().to_euler();
	offset_trans=-_elka_sf_t;
	pose_stamped_s p(dt,param_mask,v,base_thrust);
  p.set_offset(&offset_rot,
							 &offset_trans);
               */

  _setpoints_new.push_back(*s);
}

uint8_t elka::BasicNavigator::takeoff(float z,bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
  float eps;
  uint8_t param_mask=0;
	t=2*TAKEOFF_SETPOINT_DEFAULT_LEN;
	tmp=get_pose()->pose;
	tmp(3)=0;
	tmp(4)=0;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
  float dz=0;
  tmp(2)=dz;
  tmp(5)=VERTICAL_DEFAULT_SPEED*((z-dz)/z);
  
#if defined(ELKA_DEBUG) && defined(DEBUG_GAINS)
  add_setpoint(t,param_mask,&tmp,TEST_DEFAULT_THRUST);
#else
  float thrust_percentage_heuristic;
  float thrust_percentage_heuristic_init=0.6;
  float thrust_percentage_heuristic_max=0.8;
  uint8_t warmup=4;
  for (int i=1; i<=warmup; i++) {
    thrust_percentage_heuristic=thrust_percentage_heuristic_init+
        (i/warmup)*(thrust_percentage_heuristic_max - thrust_percentage_heuristic_init);
    add_setpoint(0.5*TAKEOFF_SETPOINT_DEFAULT_LEN,
        param_mask,&tmp,thrust_percentage_heuristic*HOVER_DEFAULT_THRUST);
  }

  eps=fabs(z-dz);
  if (eps<POSITION_EPSILON) {
    if (hold) param_mask|=SETPOINT_PARAM_HOLD;
    tmp(5)=0;
  }

  add_setpoint(t,param_mask,&tmp,HOVER_DEFAULT_THRUST);
#endif

  while (eps>POSITION_EPSILON) {
    dz=dz+(z-dz)/2;
    eps=fabs(z-dz);
    tmp(2)=dz;
    tmp(5)=VERTICAL_DEFAULT_SPEED*((z-dz)/z);
    if (eps<POSITION_EPSILON) {
      if (hold) param_mask|=SETPOINT_PARAM_HOLD;
      tmp(5)=0;
    }
#if defined(ELKA_DEBUG) && defined(DEBUG_GAINS)
    add_setpoint(t,param_mask,&tmp,TEST_DEFAULT_THRUST);
#else
    add_setpoint(t,param_mask,&tmp,HOVER_DEFAULT_THRUST);
#endif
  }
  return ELKA_SUCCESS;
}

uint8_t elka::BasicNavigator::hover(bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
  uint8_t param_mask=0;
  if (hold) param_mask|=SETPOINT_PARAM_HOLD;

#if defined(ELKA_DEBUG) && defined(DEBUG_HOVER_HOLD)
	t=1000*SETPOINT_DEFAULT_LEN;
#else
  t=100*SETPOINT_DEFAULT_LEN;
#endif
  // For hover, keep current x,y,z
	tmp=get_pose()->pose;
	tmp(3)=0;
	tmp(4)=0;
	tmp(5)=0;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
#if defined(ELKA_DEBUG) && defined(DEBUG_GAINS)
  add_setpoint(t,param_mask,&tmp,TEST_DEFAULT_THRUST);
#else
  add_setpoint(t,param_mask,&tmp,HOVER_DEFAULT_THRUST);
#endif
  return ELKA_SUCCESS;
}

uint8_t elka::BasicNavigator::land(bool hold) {
  math::Vector<STATE_LEN> tmp;
	hrt_abstime t;
#if defined(ELKA_DEBUG) && defined(DEBUG_GAINS)
  uint16_t base_thrust=TEST_DEFAULT_THRUST;
#else
  uint16_t base_thrust=LAND_DEFAULT_THRUST;
#endif
  uint8_t param_mask=0;
	t=LANDING_SETPOINT_DEFAULT_LEN;
	tmp=get_pose()->pose;
	tmp(3)=0;
	tmp(4)=0;
  tmp(5)=-VERTICAL_DEFAULT_SPEED;
	tmp(10)=0;
	tmp(11)=0;
	tmp(12)=0;
  float z=tmp(2),init_z=tmp(2);
  tmp(5)=-VERTICAL_DEFAULT_SPEED*(z/init_z);
  // Check if landed. Can land lower than LAND_DEFAULT_HEIGHT,
  // but not higher
  if (fabs(z-LAND_DEFAULT_HEIGHT)<LANDING_HEIGHT_EPSILON) {
    if (hold) param_mask|=SETPOINT_PARAM_LAND|SETPOINT_PARAM_HOLD;
    else param_mask|=SETPOINT_PARAM_LAND;
    tmp(5)=0;
    base_thrust=0;
  }
  add_setpoint(t,param_mask,&tmp,base_thrust);
  while (fabs(z-LAND_DEFAULT_HEIGHT)>LANDING_HEIGHT_EPSILON) {
    z-=(z-LAND_DEFAULT_HEIGHT)/1.5;
    tmp(2)=z;
    tmp(5)=-VERTICAL_DEFAULT_SPEED*(z/init_z);
    if (fabs(z-LAND_DEFAULT_HEIGHT)<LANDING_HEIGHT_EPSILON) {
      if (hold) param_mask|=SETPOINT_PARAM_LAND|SETPOINT_PARAM_HOLD;
      else param_mask|=SETPOINT_PARAM_LAND;
      tmp(5)=0;
      base_thrust=0;
    }
    add_setpoint(t,param_mask,&tmp,base_thrust);
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

//FIXME
bool elka::BasicNavigator::at_setpoint() {
  // Retrieve norm of position error as norm error
  float norm_curr_err = _curr_err.pos_norm();
  if (norm_curr_err < POSITION_ERROR_THRES)
    _at_setpoint = true;
  else _at_setpoint = false;

  return _at_setpoint;
}

//FIXME Should we still be correcting angle error like this?
//      Velocity error should scale with position error
void elka::BasicNavigator::update_error(pose_stamped_s *curr_setpoint) {
  math::Vector<3>eul_err;
  if (curr_setpoint) {
    _curr_err.pose=curr_setpoint->pose-_est.get_pose()->pose;
    // Avoid quaternion error when current pose is at +/-PI
    // Subtract Euler angles
    eul_err=curr_setpoint->get_eul()-_est.get_pose()->get_eul();
    _curr_err.set_eul(eul_err);
    _curr_err.base_thrust=curr_setpoint->base_thrust;
  } else {
    _curr_err.pose.zero();
    //TODO move to next_setpoint() function
    _wait=true;
  }
#if defined(ELKA_DEBUG) && defined(DEBUG_TRANSFORM_ERROR)
  math::Vector<3> err_ang=_curr_err.get_body_pose(SECT_ANG);
  PX4_INFO("err yaw: %3.3f",err_ang(2));
  /*
  PX4_INFO("pose ang: %3.3f,%3.3f,%3.3f",
      pose_ang(0),pose_ang(1),pose_ang(2));
  PX4_INFO("sp ang: %3.3f,%3.3f,%3.3f",
      sp_ang(0),sp_ang(1),sp_ang(2));
      */
  /*
  PX4_INFO("curr_err_q: %3.3f,%3.3f,%3.3f,%3.3f",
      _curr_err.pose(6),_curr_err.pose(7),_curr_err.pose(8),
      _curr_err.pose(9));
      */
  /*
  PX4_INFO("curr_err_pos: %3.3f,%3.3f,%3.3f",
      _curr_err.pose(0),_curr_err.pose(1),_curr_err.pose(2));
   */ 
#endif
}

//void elka::BasicNavigator::update_error(setpoint_s *curr_setpoint) {
//  TODO
//}

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
    add_setpoint(dt,param_mask,&v,HOVER_DEFAULT_THRUST);
  }
  return ELKA_SUCCESS;
}
 //TODO Handle cases when landing or when gaining altitude
 //     Must change base_thrust (currently HOVER_DEFAULT_THRUST)
int8_t elka::BasicNavigator::generate_setpoints_new(
    std::vector<math::Vector<SETPOINT_MAP_LEN>> p) {
  std::vector<math::Vector<SETPOINT_MAP_LEN>> v,a;
  setpoint_s s;
  hrt_abstime dt=SETPOINT_DEFAULT_LEN;
  uint8_t param_mask=0;
  //TODO implement deriv()
  //TODO set final component to zero
  deriv(&p,&v,dt);
  deriv(&v,&a,dt);
  auto it_p=p.begin(),
       it_v=v.begin(),
       it_a=a.begin(); 
  while (it_p!=p.end()) {
    s=setpoint_s(dt,*it_p,*it_v,*it_a,
      *(it_p+1),*(it_v+1),*(it_a+1),HOVER_DEFAULT_THRUST,
      param_mask);
    add_setpoint(&s);
    it_p++;
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
