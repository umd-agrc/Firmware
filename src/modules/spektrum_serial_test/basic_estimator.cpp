#include "basic_estimator.h"

elka::BasicEstimator::BasicEstimator() {
  math::Vector<STATE_LEN> tmp=math::Vector<STATE_LEN>();
  _curr_pose.set_pose(hrt_absolute_time(),&tmp);
  _prev_pose.set_pose(hrt_absolute_time(),&tmp);
  _prev_filt_pose.set_pose(hrt_absolute_time(),&tmp);
}

elka::BasicEstimator::~BasicEstimator() {

}

pose_stamped_s *elka::BasicEstimator::get_pose() {
  return &_curr_pose;
}

float elka::BasicEstimator::get_pose(uint8_t n){
  if (n > STATE_LEN) {
    PX4_ERR("Invalid pose index %d",n);
    return PX4_ERROR;
  } else
    return _curr_pose.pose(n);
}

void elka::BasicEstimator::set_pose(hrt_abstime t,math::Vector<STATE_LEN>*v) {
  update_prev_pose();
  _curr_pose.set_pose(t,v);
}

void elka::BasicEstimator::set_pose(
    hrt_abstime t[STATE_LEN],
    math::Vector<STATE_LEN>*v) {
  update_prev_pose();
  _curr_pose.set_pose(t,v);
}

void elka::BasicEstimator::set_pose(pose_stamped_s *p) {
  update_prev_pose();
  _curr_pose.set_pose(p);
}

void elka::BasicEstimator::set_pose(uint8_t n,
                                    hrt_abstime t,
                                    float f) {
  if (n > STATE_LEN)
    PX4_ERR("Invalid pose index %d",n);
  else {
    update_prev_pose(n);
    _curr_pose.set_pose(n,t,f);
  }
}

void elka::BasicEstimator::update_prev_pose() {
  _prev_pose.set_pose(&_curr_pose);
}

void elka::BasicEstimator::update_prev_pose(uint8_t n) {
  _prev_pose.set_pose(n,
                      _curr_pose.t[n],
                      _curr_pose.pose(n));
}

//TODO
void elka::BasicEstimator::low_pass_filt(
    uint8_t *filter_states) {
  float alpha=0.07;
  hrt_abstime t;
  float dt=0;
  for (uint8_t i=0; i < STATE_LEN; i++) {
    uint8_t curr_state=*filter_states;
    float s=0.0,s_samp=0.0;
    if (curr_state==FILT_STATE) {
      t=_curr_pose.t[i];
      s=alpha*_curr_pose.pose(i)+(1-alpha)*_prev_filt_pose.pose(i);
      
      set_pose(i,t,s);  
      _prev_filt_pose.set_pose(i,t,s);
      // Typically, _curr_pose updates before next call.
      // It is useful when calling BasicEstimator::get_pose().
      // _prev_filt_pose should stay the same for the next call
      // Set time for pose state as mean between curr pose and prev
      // pose state times
    } else if (curr_state==FILT_DERIV) {
      if (i==3 || i==4 || i==5) {
        dt=((float)(_curr_pose.t[i-3]-_prev_filt_pose.t[i]))*1e-6;
        if (dt < DT_MIN) dt=DT_MIN;
        t=_curr_pose.t[i-3];
        s_samp=(_curr_pose.pose(i-3)-_prev_pose.pose(i-3))/
               (dt);
        s=alpha*s_samp+(1-alpha)*_prev_filt_pose.pose(i);
#if defined(ELKA_DEBUG) && defined(DEBUG_FILTER)
        PX4_INFO("dt: %f, s_samp: %f, s: %f",
            dt,s_samp,s);
#endif

      } else if (i==10 || i==11 || i==12) {
        //TODO derivative of quaternion
      }

      set_pose(i,t,s);  
      _prev_filt_pose.set_pose(i,t,s); // Used for next call
    }

    filter_states++;
  }
}
