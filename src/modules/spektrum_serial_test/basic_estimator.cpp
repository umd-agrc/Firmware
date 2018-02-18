#include "basic_estimator.h"

elka::BasicEstimator::BasicEstimator() {
  math::Vector<STATE_LEN> tmp=math::Vector<STATE_LEN>();
  _curr_pose.set_pose(hrt_absolute_time(),&tmp);
  _prev_pose.set_pose(hrt_absolute_time(),&tmp);
  _prev_filt_pose.set_pose(hrt_absolute_time(),&tmp);

  // Set up sensors
  _prev_acc.set_type(SENSOR_ACCEL);
  _prev_gyro.set_type(SENSOR_GYRO);
  _prev_cam.set_type(SENSOR_CAM);
  //TODO set offsets for imu, gyro, camera
  _prev_acc.set_offset(math::Vector<3>(), math::Matrix<3,3>());
  _prev_gyro.set_offset(math::Vector<3>(), math::Matrix<3,3>());
  _prev_cam.set_offset(math::Vector<3>(), math::Matrix<3,3>());
}

elka::BasicEstimator::~BasicEstimator() {

}

void elka::BasicEstimator::set_prev_inert_sens(sensor_combined_s *s) {
  _prev_acc.set_data(
      s->timestamp+s->accelerometer_timestamp_relative,
      {s->accelerometer_m_s2[0],
       s->accelerometer_m_s2[1],
       s->accelerometer_m_s2[2]});
  _prev_acc.set_data(
      s->timestamp,
      {s->gyro_rad[0],
       s->gyro_rad[1],
       s->gyro_rad[2]});
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

void elka::BasicEstimator::low_pass_filt(
    uint8_t *filter_states) {
  float alpha=0.08;
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
        s_samp=(_curr_pose.pose(i-3)-_prev_pose.pose(i-3))/dt;
      } else if (i==10 || i==11 || i==12) {
        // Get current and previous euler angles 
        math::Vector<3> v_curr=_curr_pose.get_body_pose(SECT_ANG),
                        v_prev=_prev_pose.get_body_pose(SECT_ANG);

        // Assume all quaternion properties updated at the same time
        dt=((float)(_curr_pose.t[6]-_prev_pose.t[i]))*1e-6;
        s_samp=(v_curr(i-10)-v_prev(i-10))/dt;
      }

      s=alpha*s_samp+(1-alpha)*_prev_filt_pose.pose(i);
#if defined(ELKA_DEBUG) && defined(DEBUG_FILTER)
      PX4_INFO("dt: %f, s_samp: %f, s: %f",
          dt,s_samp,s);
#endif

      set_pose(i,t,s);  
      _prev_filt_pose.set_pose(i,t,s); // Used for next call
    }

    filter_states++;
  }
}

void elka::BasicEstimator::ekf() {
  // Using state matrix x:
  //    p=posiiton_cam
  //    q=rotation_cam
  //    pd=velocity_cam
  //    b_g=bias_gyro
  //    b_a=bias_accelerometer

  // Using nonlinear stochastic equation dx=f(x,u,n):
  //    pd
  //    G(q)^-1*(w_m)
  //    g+R(q)*(a_m-b_a-n_a)
  //    n_g
  //    n_a
  //
  //    With w_m = gyro measurements
  //         a_m = accel measurements
  //         n_g = noise gyro
  //         n_a = noise accel
  //         g = gravitational constant
  //         G = conversion matrix from body->inertial angle rates
  //         R = body->inertial rotation matrix

  // Define f(mu_{t-1},u_t,0)
  static math::Vector<STATE_LEN_EKF> prev_mea_update,
                                     mu_bar_t,
                                     mu_t,
                                     b;
  static math::Matrix<STATE_LEN_EKF,STATE_LEN_EKF_ODOM> A,
                                                        F;
  static math::Matrix<STATE_LEN_EKF,STATE_LEN_EKF_BIAS> U,
                                                        V;

  // Define convenience vars for jacobians A,U,F,V,b
  // A=df/dx|(mu_{t-1},u_t,0)
  A(1,4)=1;
  A(2,5)=1;
  A(3,6)=1;

  // Define model and sensor covariance matrices Q,R
  
  // Update mean bar{mu}_t
  
  // Update bar{Sigma}_t

  // Update _prev_filt_pose for use later to update
  // prev_mean_update
  _prev_filt_pose.set_pose(&_curr_pose);
  
  // Define convenience jacobians vars for jacobians C,W

  // Define h(bar{mu}_t,0)

  // Define K_t
  
  // Update mu_t and update prev_mean_update

  //prev_mean_update = mu_t-_prev_filt_pose;

  // Update Sigma_t

  // Update curr_pose to be value of mu_t
}
