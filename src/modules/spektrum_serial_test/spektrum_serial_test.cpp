#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

#include "basic_navigator.h"
#include "basic_uart.h"
#include "spektrum_serial_test.h"

extern "C" { __EXPORT int spektrum_serial_test_main(int argc, char *argv[]); }

typedef struct gains gains_s;
//static void set_gains();

static int daemon_task;
static volatile bool thread_should_exit;
static volatile bool thread_running;
int _serial_state;
hrt_abstime _prev_spektrum_update, _old_msg_duration;
struct hrt_call	_serial_state_call;
static int _serial_state_call_interval;
gains_s _gains;
bool _new_setpoint;

void usage() {
  PX4_WARN("usage: spektrum_serial_test <start | stop | status>");
}

int spektrum_serial_test_main(int argc, char *argv[]) {
	if (argc < 2) {
		PX4_WARN("Missing action.");
    usage();
    return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
    if (!thread_running) {
      char thread_name[256];
      sprintf(thread_name,"spektrum_serial_test");

      thread_should_exit = false;
      daemon_task = px4_task_spawn_cmd(
        thread_name,
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        1000,
        spektrum_test_loop,
        NULL);

      unsigned constexpr max_wait_us = 1000000;
      unsigned constexpr max_wait_steps = 2000;
      unsigned j;

      for (j=0; j < max_wait_steps; j++) {
        usleep(max_wait_us / max_wait_steps);
        if (thread_running) {
          break;
        }
      }
      return !(j < max_wait_steps);
    } else {
      PX4_INFO("spektrum serial test is already running.");
    }


  } else if (!strcmp(argv[1], "stop")) {
    if (!thread_running) {
      PX4_WARN("spektrum serial test already stopped");
      return PX4_OK;
    }

    thread_should_exit = true;

    while(thread_running) {
      usleep(200000);
      PX4_WARN(".");
    }

    PX4_WARN("terminated.");

    return PX4_OK;
  } else if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      PX4_INFO("spektrum serial test is running");
    } else {
      PX4_INFO("spektrum serial test is not running");
    }

    return PX4_OK;

  } else {
		PX4_WARN("Action not supported");
  }


  return PX4_OK;
}

int spektrum_test_loop(int argc, char **argv) {
  _serial_state = SERIAL_STATE_NONE;
  char snd_arr[MAX_SERIAL_MSG_LEN];
  int snd_arr_len = 0,
      spektrum_fd = -1;

  // Open file for UART
  spektrum_fd = spektrum_serial_open(); 

  // Define poll_return for defined file descriptors
  int poll_ret;

  struct input_rc_s input_rc;
  memset(&input_rc, 0, sizeof(input_rc));
  vehicle_local_position_s estimator_pos,
                           vision_pos;
  vehicle_attitude_s estimator_att,
                     vision_att;

  memset(&estimator_pos, 0, sizeof(estimator_pos));
  memset(&estimator_att, 0, sizeof(estimator_att));
  memset(&vision_pos, 0, sizeof(vision_pos));
  memset(&vision_att,0,sizeof(vision_att));

  // Subscribe to elka msg, elka msg ack, and input_rc (TODO only if necessary)
  // Vision position omes from MAVLink SLAM pose estimate
  // Local position comes from LPE BlockLocalPositionEstimator
  int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
  int vision_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_position));
  int vision_att_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_attitude));

// If debugging, must not be receiving pose from built-in
// LPE, EKF, etc filters. Instead, debugging mode sends back
// local_position_ned via MAVLink on the
// vehicle_{attitude|local_position} channels
#if defined(ELKA_DEBUG) && defined(DEBUG_POSE)
  orb_advert_t local_pos_pub, local_att_pub;
  local_pos_pub=orb_advertise(ORB_ID(vehicle_local_position),
                              &estimator_pos);
  local_att_pub=orb_advertise(ORB_ID(vehicle_attitude),
                              &estimator_att);
#else
  int estimator_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
  int estimator_att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
  orb_set_interval(estimator_pos_sub_fd, 10);
  orb_set_interval(estimator_att_sub_fd, 10);
#endif

  // Set update rate to 100Hz
  orb_set_interval(input_rc_sub_fd, 10);
  orb_set_interval(vision_pos_sub_fd, 10);
  orb_set_interval(vision_att_sub_fd, 10);

  // Create estimator
  elka::BasicNavigator nav = elka::BasicNavigator();

  uint8_t len_fds;
#if defined(ELKA_DEBUG) && defined(DEBUG_POSE)
  px4_pollfd_struct_t fds[] = {
    {.fd = input_rc_sub_fd, .events = POLLIN},
    {.fd = vision_pos_sub_fd, .events = POLLIN},
    {.fd = vision_att_sub_fd, .events = POLLIN},
  };
  len_fds=3;
#else
  px4_pollfd_struct_t fds[] = {
    {.fd = input_rc_sub_fd, .events = POLLIN},
    {.fd = estimator_pos_sub_fd, .events = POLLIN},
    {.fd = estimator_att_sub_fd, .events = POLLIN},
    {.fd = vision_pos_sub_fd, .events = POLLIN},
    {.fd = vision_att_sub_fd, .events = POLLIN},
  };
  len_fds=5;
#endif

  // Set old message duration to 1/10 s
  set_old_msg_duration(100000);
  _prev_spektrum_update = hrt_absolute_time();
  // Serial state call interval in hz
  _serial_state_call_interval = 1000;
  hrt_call_every(&_serial_state_call, 0,
           (_serial_state_call_interval),
           (hrt_callout)&serial_state_timeout_check,
           nullptr);

  _new_setpoint = true;
  uint8_t msg_type;

  uint8_t using_vision=3; // So far, vision gets precedence

  uint16_t spektrum_thrust = 0;

  int error_counter = 0;

  thread_running = true;

  while (!thread_should_exit) {
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 500);

    // Handle the poll result
    if (poll_ret == 0) {
      // None of our providers is giving us data
      PX4_ERR("Got no data");
    } else if (poll_ret < 0) {
      // Should be an emergency
      if (error_counter < 10 || error_counter % 50 == 0) {
        // Use a counter to prevent flooding and slowing us down
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
      }

      error_counter++;
    } else {

      if (fds[0].revents & POLLIN) { // input_rc
        orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &input_rc);

        // Thrust correction for ELKA
        input_rc.values[SPEKTRUM_THRUST_CHANNEL] =
          input_rc.values[SPEKTRUM_THRUST_CHANNEL] < RAW_THRUST_BASELINE ?
          0 : input_rc.values[SPEKTRUM_THRUST_CHANNEL]-RAW_THRUST_BASELINE;
        spektrum_thrust = input_rc.values[SPEKTRUM_THRUST_CHANNEL];
        msg_set_serial_state(MSG_TYPE_SPEKTRUM, &input_rc);

#if defined(ELKA_DEBUG) && defined(DEBUG_SPEKTRUM)
        PX4_INFO("channel values\n0:\t%" PRIu16 "\n1:\t%" PRIu16 "\n2:\t"
"%" PRIu16 "\n3:\t%" PRIu16 "\n4:\t%" PRIu16 "\n5:\t%" PRIu16 "\n6:\t"
"%" PRIu16 "\n7:\t%" PRIu16 "\n8:\t%" PRIu16 "\n9:\t%" PRIu16 "\n10:\t"
"%" PRIu16 "",
            input_rc.values[0],input_rc.values[1],input_rc.values[2],
            input_rc.values[3],input_rc.values[4],input_rc.values[5],
            input_rc.values[6],input_rc.values[7],input_rc.values[8],
            input_rc.values[9],input_rc.values[10],input_rc.values[11]);
#endif

        if (_serial_state == SERIAL_STATE_KILL) {
          msg_type = MSG_TYPE_KILL;
        } else if (_serial_state == SERIAL_STATE_SPEKTRUM) {
          msg_type = MSG_TYPE_SPEKTRUM;
        }

        if (check_state(msg_type) &&
            (snd_arr_len = pack_spektrum_cmd(snd_arr,
                                            msg_type,
                                            &input_rc))
             != PX4_ERROR) {
          if (spektrum_serial_write(spektrum_fd,
                                    snd_arr, snd_arr_len) !=
              spektrum_fd) {
        PX4_WARN("Spektrum serial write failed for msg with timestamp: \
%" PRIu64 "",
            input_rc.timestamp);
          } else {
            // Must send new setpoint after successful spektrum write
            nav.set_new_setpoint(true);
            usleep(20000);
          }
        }
      }

#ifndef ELKA_DEBUG
      if (fds[1].revents & POLLIN) { // estimator_pos 
        orb_copy(ORB_ID(vehicle_local_position),
                 estimator_pos_sub_fd,
                 &estimator_pos);
        using_vision&=2; // Bitmask 0 as first bit
      }
      if (fds[2].revents & POLLIN) { // estimator_att
        orb_copy(ORB_ID(vehicle_attitude),
                 estimator_att_sub_fd,
                 &estimator_att);
        using_vision&=1; // Bitmask 0 as second bit
      }
#endif
      if (fds[len_fds-2].revents & POLLIN) { // vision_pos 
        orb_copy(ORB_ID(vehicle_vision_position),
                 vision_pos_sub_fd,
                 &vision_pos);
        using_vision|=1; // Bitmask 1 as first bit
      }
      if (fds[len_fds-1].revents & POLLIN) { // vision_att 
        orb_copy(ORB_ID(vehicle_vision_attitude),
                 vision_att_sub_fd,
                 &vision_att);
        using_vision|=2; // Bitmask 1 as second bit
      }

      if (using_vision & 3) { // vision_att and vision_pos
        nav.update_pose(&vision_pos,&vision_att);
      } else if (using_vision & 2) { // vision_att and estimator_pos
        nav.update_pose(&estimator_pos,&vision_att);
      } else if (using_vision & 1) { // estimator_att and vision_pos
        nav.update_pose(&vision_pos,&estimator_att);
      } else { // estimator_att and estimator_pos
        nav.update_pose(&estimator_pos,&estimator_att);
      }

#if defined(ELKA_DEBUG) && defined(DEBUG_POSE)
      nav.copy_pose_error(&estimator_pos,
                          &estimator_att);
      orb_publish(ORB_ID(vehicle_local_position),
                  local_pos_pub,
                  &estimator_pos);
      orb_publish(ORB_ID(vehicle_attitude),
                  local_att_pub,
                  &estimator_att); 

      static math::Vector<3> pos,ang,vel;
      pos=nav.get_pose()->get_body(SECT_POS);
      vel=nav.get_pose()->get_body(SECT_VEL);
      ang=nav.get_pose()->get_body(SECT_ANG);
      PX4_INFO("loc_x: %f, loc_y: %f,\
loc_vx: %f, loc_vy: %f, loc_yaw: %f",
               pos(0),pos(1),
               vel(0),vel(1),
               ang(2));
#endif

      if (nav.check_setpoint())
        msg_type = MSG_TYPE_SETPOINT;
      else if (!using_vision)
        msg_type = MSG_TYPE_LOCAL_POS;
      else
        msg_type = MSG_TYPE_VISION_POS;

      //FIXME should pos and att should align with using_vision specs
      msg_set_serial_state(msg_type,
                           &vision_pos,
                           &vision_att);
      if (check_state(msg_type) &&
          (pack_position_estimate(snd_arr,
                                  msg_type,
                                  nav.get_err())
           != PX4_ERROR)) {
        // Add on spektrum thrust to serial message
        if (append_serial_msg(snd_arr,
                              MSG_TYPE_THRUST,
                              spektrum_thrust,
                              sizeof(spektrum_thrust)) ==
            PX4_ERROR) {
          PX4_WARN("Could not append to serial message.");
        }
        snd_arr_len = *snd_arr+1;

        if (spektrum_serial_write(spektrum_fd,
                                  snd_arr, snd_arr_len) !=
            spektrum_fd) {
          PX4_WARN("Spektrum serial write failed for msg with timestamp: \
%" PRIu64 "",
              vision_pos.timestamp);
        } else {
          // Don't send new setpoint after successful setpoint write
          nav.set_new_setpoint(false);
          usleep(20000);
        }
      }
    }
  }

  spektrum_fd = spektrum_serial_close(spektrum_fd); 

  thread_running = false;

  return PX4_OK;
}

int pack_position_estimate(char *snd_arr,
                           uint8_t pos_type,
                           pose_stamped_s *curr_err) {
  int num_bytes = 0;
  int32_t xe,ye,ze=0,vxe,vye,vze=0;
  bool continue_packing = true;

  static math::Vector<3> pos_e;
  static math::Vector<3> vel_e;

  pos_e=curr_err->get_body(SECT_POS);
  vel_e=curr_err->get_body(SECT_VEL);

  // Sending body frame error
  xe = (int32_t)(pos_e(1)*10000);
  ye = (int32_t)(-pos_e(0)*10000);
  ze = (int32_t)(pos_e(2)*10000);
  vxe = (int32_t)(vel_e(1)*10000);
  vye = (int32_t)(-vel_e(0)*10000);
  vze = (int32_t)(vel_e(2)*10000);

#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL)
  PX4_INFO("xe: %" PRIi32 " ye: %" PRIi32 " ze: %" PRIi32 "\n\
vxe: %" PRIi32 " vye: %" PRIi32 " vze: %" PRIi32 "",
    xe,ye,ze,
    vxe,vye,vze);
#endif

  while (num_bytes < MAX_SERIAL_MSG_LEN &&
         continue_packing) {
    if (num_bytes < 2 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 1)) {
      // Pack length:
      *snd_arr = 20;
      snd_arr++; num_bytes++;
      *snd_arr = 19;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 5 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 3)) {
      // Pack header defined as follows:
      // state, 255, 255
      //FIXME distinguish between vision_pos and local_pos
      *snd_arr = pos_type;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 21 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 12)) {
      *snd_arr = get_byte_n((int64_t)xe,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)xe,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)xe,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)xe,1);
      snd_arr++; num_bytes++;

      *snd_arr = get_byte_n((int64_t)ye,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ye,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ye,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ye,1);
      snd_arr++; num_bytes++;

      /*
      *snd_arr = get_byte_n((int64_t)ze,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ze,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ze,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)ze,1);
      snd_arr++; num_bytes++;
      */

      *snd_arr = get_byte_n((int64_t)vxe,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vxe,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vxe,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vxe,1);
      snd_arr++; num_bytes++;

      *snd_arr = get_byte_n((int64_t)vye,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vye,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vye,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vye,1);
      snd_arr++; num_bytes++;
      
      /*
      *snd_arr = get_byte_n((int64_t)vze,4);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vze,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vze,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n((int64_t)vze,1);
      snd_arr++; num_bytes++;
      */

      continue_packing = false;
    }
    // Add the following if you intend to send gains
    /* else if (num_bytes < 19 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >=4)) {
      
      *snd_arr = (_gains.pos_kp >> 8) & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (_gains.pos_kp) & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (_gains.pos_kd >> 8) & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (_gains.pos_kd) & 0xff;
      snd_arr++; num_bytes++;
                           
      continue_packing = false;
      
    }*/
    else {
      PX4_ERR("position estimate could not be packed. Message too long.");
      continue_packing = false;
      return PX4_ERROR;
    }
  }

  return num_bytes;
}

int pack_position_estimate(char *snd_arr,
                           uint8_t pos_type, 
                           pose_stamped_s *curr_err,
                           uint16_t thrust) {
  int num_bytes = pack_position_estimate(snd_arr,pos_type,curr_err);
  if (num_bytes + 2 < MAX_SERIAL_MSG_LEN) {
    *snd_arr += 2;
    *(snd_arr+1) += 2;
    *(snd_arr+num_bytes) = get_byte_n((uint64_t)thrust,2);
    *(snd_arr+num_bytes+1) = get_byte_n((uint64_t)thrust,1);
    //*(snd_arr+num_bytes) = 0;
    //*(snd_arr+num_bytes+1) = 1;
    num_bytes+=2;
    return num_bytes;
  } else {
    return PX4_ERROR;
  }
}

int pack_spektrum_cmd(char *snd_arr,
                      uint8_t msg_type,
                      input_rc_s *input_rc) {
  if (msg_type == MSG_TYPE_KILL)
    return pack_kill_msg(snd_arr, input_rc);
  else if (msg_type == MSG_TYPE_SPEKTRUM)
    return pack_input_rc_joysticks(snd_arr, input_rc);
  else
    return PX4_ERROR;
}

int pack_kill_msg(char *snd_arr, input_rc_s *input_rc) {
  int num_bytes = 0;
  bool continue_packing = true;

  while (num_bytes < MAX_SERIAL_MSG_LEN &&
         continue_packing) {
    if (num_bytes < 2 &&
          (MAX_SERIAL_MSG_LEN - num_bytes >= 1)) {
        // Pack length:
        *snd_arr = 12;
        snd_arr++; num_bytes++;
        *snd_arr = 11;
        snd_arr++; num_bytes++;
    } else if (num_bytes < 5 &&
          (MAX_SERIAL_MSG_LEN - num_bytes >= 4)) {
        *snd_arr = MSG_TYPE_KILL;
        snd_arr++; num_bytes++;
        *snd_arr = 255;
        snd_arr++; num_bytes++;
        *snd_arr = 255;
        snd_arr++; num_bytes++;

    } else if ((num_bytes < 9 +
                            2*NUM_JOYSTICK_CHANNELS) &&
      (MAX_SERIAL_MSG_LEN - num_bytes >= 2*NUM_JOYSTICK_CHANNELS)) {
      // up to 36B channel pwms ranging from 1000-2000
      for (uint8_t i=0; i < NUM_JOYSTICK_CHANNELS; i++){
        *snd_arr = (input_rc->values[i] >> 8) & 0xff;
        snd_arr++; num_bytes++;
        *snd_arr = input_rc->values[i] & 0xff;
        snd_arr++; num_bytes++;
        PX4_INFO("channel %d value: %d", i, input_rc->values[i]);
        //PX4_INFO("next byte: %du",
        //    (*(snd_arr-1) << 8) | (*(snd_arr-2)));
      }

      continue_packing = false;
    } else {
      PX4_ERR("position estimate could not be packed. Message too long.");
      continue_packing = false;
      return PX4_ERROR;
    }
  }

  return num_bytes;
}

int pack_input_rc_joysticks(char *snd_arr, input_rc_s *spektrum) {
  int num_bytes = 0;
  bool continue_packing = true;

  while (num_bytes < MAX_SERIAL_MSG_LEN &&
         continue_packing) {

    if (num_bytes < 2 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 1)) {
      // Pack length:
      *snd_arr = 12;
      snd_arr++; num_bytes++;
      *snd_arr = 11;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 5 &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 4)) {
      // Pack header defined as follows:
      //   msg type, 255, 255 
      *snd_arr = MSG_TYPE_SPEKTRUM;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
    } else if ((num_bytes < 5+
                            2*NUM_JOYSTICK_CHANNELS) &&
        (MAX_SERIAL_MSG_LEN - num_bytes >= 2*NUM_JOYSTICK_CHANNELS)) {
      // up to 36B channel pwms ranging from 1000-2000
      for (uint8_t i=0; i < NUM_JOYSTICK_CHANNELS; i++){
        *snd_arr = (spektrum->values[i] >> 8) & 0xff;
        snd_arr++; num_bytes++;
        *snd_arr = spektrum->values[i] & 0xff;
        snd_arr++; num_bytes++;
        PX4_INFO("channel %d value: %d", i, spektrum->values[i]);
        //PX4_INFO("next byte: %du",
        //    (*(snd_arr-1) << 8) | (*(snd_arr-2)));
      }

      continue_packing = false;
    } else {
      PX4_ERR("input_rc could not be packed. Message too long.");
      continue_packing = false;
      return PX4_ERROR;
    }
  }

  return num_bytes;
}

void write_serial_header(
    char *snd_arr,
    uint16_t len,
    uint8_t msg_type) {
  *(snd_arr) = len;
  *(snd_arr+1) = msg_type;
  *(snd_arr+2) = SERIAL_HEADER_ASSURANCE_BYTE;
  *(snd_arr+3) = SERIAL_HEADER_ASSURANCE_BYTE;
}

int append_serial_msg(char *snd_arr,
                      uint8_t msg_type,
                      uint16_t bytes,
                      uint16_t len) {
  uint16_t curr_global_len = *(snd_arr)+1,
           new_msg_len = len+SERIAL_HEADER_LEN+1;

  if (!(curr_global_len+new_msg_len < MAX_SERIAL_MSG_LEN)) {
    PX4_DEBUG("Not enough room in packet to append serial message");
    return PX4_ERROR;
  }

  *(snd_arr+curr_global_len) = new_msg_len;

  write_serial_header(
      &(*(snd_arr+curr_global_len)),
      new_msg_len-1,
      msg_type);

  for (uint8_t i=SERIAL_HEADER_LEN+1; i<new_msg_len; i++)
    *(snd_arr+curr_global_len+i) =
      get_byte_n((uint64_t)bytes,new_msg_len-i);
  
  // Set new global length
  *(snd_arr) = curr_global_len-1+new_msg_len;
  // Return new total packet length
  return curr_global_len+new_msg_len;
}

void serial_state_timeout_check(void *arg) {
  static hrt_abstime timeout_duration;
  timeout_duration = hrt_abstime(100000);
  if (hrt_absolute_time() - _prev_spektrum_update > timeout_duration) {
    _serial_state = SERIAL_STATE_NONE;
  }
  /*
  static int i=0;
  i++;
  if (!(i%100))
    PX4_INFO("serial state: %d", _serial_state);
    */
}

void msg_set_serial_state(int msg_type, input_rc_s *input_rc) {
  if (!old_msg(input_rc->timestamp)) {
    if (input_rc->values[SPEKTRUM_KILL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_KILL;
    } else if (input_rc->values[SPEKTRUM_CONTROL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_SPEKTRUM;
    } else {
      // Must set to none if none of the switches are flipped
      // so that _serial_state can switch to lower-priority state
      // in the event that controller inputs continue streaming
      _serial_state = SERIAL_STATE_NONE;
    }
    _prev_spektrum_update = hrt_absolute_time();
  }
}

void msg_set_serial_state(int msg_type,
                          vehicle_local_position_s *pos,
                          vehicle_attitude_s *att) {
  if (!(old_msg(pos->timestamp) || old_msg(att->timestamp))) {
    if (_serial_state != SERIAL_STATE_KILL &&
        _serial_state != SERIAL_STATE_SPEKTRUM) {
      if (msg_type == MSG_TYPE_LOCAL_POS) {
        _serial_state = SERIAL_STATE_LOCAL_POS;
      } else if (msg_type == MSG_TYPE_VISION_POS &&
                 _serial_state != SERIAL_STATE_LOCAL_POS) {
        _serial_state = SERIAL_STATE_VISION_POS;
      } else if (msg_type == MSG_TYPE_SETPOINT) {
        _serial_state = SERIAL_STATE_SETPOINT;
      } else {
        _serial_state = SERIAL_STATE_NONE;
      }
    }
    _prev_spektrum_update = hrt_absolute_time();
  }
}

inline bool check_state(int msg_type) {
  switch(msg_type) {
    case MSG_TYPE_KILL:
      return _serial_state ==SERIAL_STATE_KILL;
    case MSG_TYPE_SPEKTRUM:
      return _serial_state == SERIAL_STATE_SPEKTRUM;
    case MSG_TYPE_LOCAL_POS: 
      return _serial_state == SERIAL_STATE_LOCAL_POS;
    case MSG_TYPE_VISION_POS: 
      return _serial_state == SERIAL_STATE_VISION_POS;
    case MSG_TYPE_SETPOINT:
      return _serial_state == SERIAL_STATE_SETPOINT;
    default:
      return false;
  }
}

inline bool old_msg(hrt_abstime msg_time) {
  return (hrt_absolute_time() - msg_time) > _old_msg_duration;
}

void set_old_msg_duration(hrt_abstime time) {
  _old_msg_duration = time;
}

/*
static void set_gains() {
  _gains.pos_kp = 0;
  _gains.pos_kd = 0;
}
*/
