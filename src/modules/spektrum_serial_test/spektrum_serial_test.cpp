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

#include "spektrum_serial_test.h"

extern "C" { __EXPORT int spektrum_serial_test_main(int argc, char *argv[]); }

static int daemon_task;
static volatile bool thread_should_exit;
static volatile bool thread_running;
int _serial_state;
hrt_abstime _prev_spektrum_update, _old_msg_duration;
struct hrt_call	_serial_state_call;
static int _serial_state_call_interval;
gains_s _gains[3];

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
        800,
        spektrum_test_loop,
        &argv[2]);

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

//TODO update serial framework to use new messenger/manager 
//     framework
int spektrum_test_loop(int argc, char *argv[]) {
  thread_running = true;

  _serial_state = SERIAL_STATE_NONE;

  // Set up message manager
  elka::BasicMessageMgr *msg_mgr=elka::BasicMessageMgr::instance();
  int elka_msgr_d=msg_mgr->add_messenger(
      MESSENGER_SERIAL,(void *)"/dev/tty-4");
  // Create controller and get pointer copy of navigator
  elka::BasicController *ctl=elka::BasicController::instance();
  ctl->add_messenger(elka_msgr_d);
  if (ctl->parse_plan_file(argv[0])) {
    PX4_ERR("Failed to parse plan file %s",argv[0]);
    return PX4_ERROR;
  }
  if (ctl->start()!=ELKA_SUCCESS) {
    PX4_ERR("Failed to start controller");
    return PX4_ERROR;
  }
  elka::BasicNavigator *nav=ctl->get_navigator();

  // Define poll_return for defined file descriptors
  int poll_ret;

  input_rc_s input_rc;
  vehicle_local_position_s vision_pos;
  vehicle_attitude_s vision_att;
  elka_msg_s elka_out;
  elka_packet_s elka_pkt;

  memset(&input_rc,0,sizeof(input_rc));
  memset(&vision_pos,0,sizeof(vision_pos));
  memset(&vision_att,0,sizeof(vision_att));
  memset(&elka_out,0,sizeof(elka_out));
  memset(&elka_pkt,0,sizeof(elka_pkt));

  // Subscribe to elka msg, elka msg ack, and input_rc (TODO only if necessary)
  // Vision position omes from MAVLink SLAM pose estimate
  // Local position comes from LPE BlockLocalPositionEstimator
  int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
  int vision_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_position));
  int vision_att_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_attitude));

  // Set update rate to 100Hz
  orb_set_interval(input_rc_sub_fd, 10);
  orb_set_interval(vision_pos_sub_fd, 10);
  orb_set_interval(vision_att_sub_fd, 10);

  uint8_t len_fds;
  px4_pollfd_struct_t fds[] = {
    {.fd = input_rc_sub_fd, .events = POLLIN},
    {.fd = vision_pos_sub_fd, .events = POLLIN},
    {.fd = vision_att_sub_fd, .events = POLLIN},
  };
  len_fds=3;


  // Set old message duration to 1/10 s
  set_old_msg_duration(100000);
  _prev_spektrum_update = hrt_absolute_time();
  // Serial state call interval in hz
  _serial_state_call_interval = 1000;
  hrt_call_every(&_serial_state_call, 0,
           (_serial_state_call_interval),
           (hrt_callout)&serial_state_timeout_check,
           nullptr);

  uint8_t msg_type;

  uint16_t spektrum_thrust = 0;

  int error_counter = 0;

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
        thread_should_exit=true;
      }

      error_counter++;
    } else {


#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL_WRITE)
      if ((pack_test_msg(
              &elka_pkt))
           != PX4_ERROR) {
        //pack_test_msg(&elka_pkt);
        msg_mgr->send(elka_msgr_d,&elka_pkt);
        usleep(10000);
      }
#endif

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
            (pack_spektrum_cmd(
                &elka_pkt,
                msg_type,
                &input_rc))
             != PX4_ERROR) {
          msg_mgr->send(elka_msgr_d,&elka_pkt);
          // Must send new setpoint after successful write to ELKA
          // from RC ctrlr
          nav->reset_setpoints();
          usleep(20000);
          }
        }

      if (fds[1].revents & POLLIN) { // vision_pos 
        orb_copy(ORB_ID(vehicle_vision_position),
                 vision_pos_sub_fd,
                 &vision_pos);
      }
      if (fds[2].revents & POLLIN) { // vision_att 
        orb_copy(ORB_ID(vehicle_vision_attitude),
                 vision_att_sub_fd,
                 &vision_att);
      }

      nav->update_pose(&vision_pos,&vision_att);

      if (nav->_from_manual) {
        msg_type = MSG_TYPE_SETPOINT;
				nav->hover();
      } else
        msg_type = MSG_TYPE_VISION_POS;

      msg_set_serial_state(msg_type,
                           &vision_pos,
                           &vision_att);
      if (check_state(msg_type) &&
          (pack_position_estimate(&elka_pkt,
                                  nav->get_err())
           != PX4_ERROR)) {
        msg_mgr->send(elka_msgr_d,&elka_pkt);
        usleep(20000);
      }
    }

    // Parse packets received and reset packet to send
    //TODO this should be threaded
    msg_mgr->parse_packets();
    elka_pkt.num_msgs=0;
    elka_pkt.len=0;
  }

  ctl->exit();
  msg_mgr->remove_messenger(elka_msgr_d);

  thread_running = false;

  return PX4_OK;
}

int pack_position_estimate(elka_packet_s *snd,
                           pose_stamped_s *curr_err) {
  // Error of form [xe,ye,ze,vxe,vye,vze,yawe,vyawe]
  float e[8];
  uint8_t data_len=32,data[32];

  static math::Vector<12> body_pose_e;

  body_pose_e=curr_err->get_body_pose();

  // Sending body frame error in mm/rad
  e[0] = (float)(body_pose_e(1)*100);
  e[1] = (float)(-body_pose_e(0)*100);
  e[2] = (float)(body_pose_e(2)*100);
  e[3] = (float)(body_pose_e(4)*100);
  e[4] = (float)(-body_pose_e(3)*100);
  e[5] = (float)(body_pose_e(5)*100);
  e[6] = (float)body_pose_e(8);
  e[7] = (float)body_pose_e(11);

#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL)
  PX4_INFO("xe: %f ye: %f ze: %f\n\
vxe: %f vye: %f vze: %f\n\
yawe: %f vyawe: %f",
    e[0],e[1],e[2],e[3],e[4],e[5],e[6],e[7]);
#endif

  serialize(&(data[0]),&e,8*sizeof(float));

  return append_pkt(
          snd,
          MSG_TYPE_VISION_POS,
          data_len,
          data);
}

int pack_spektrum_cmd(elka_packet_s *snd,
                      uint8_t msg_type,
                      input_rc_s *input_rc) {
  if (msg_type == MSG_TYPE_KILL)
    return pack_kill_msg(snd, input_rc);
  else if (msg_type == MSG_TYPE_SPEKTRUM)
    return pack_input_rc_joysticks(snd, input_rc);
  else
    return PX4_ERROR;
}

int pack_kill_msg(elka_packet_s *snd, input_rc_s *input_rc) {
  uint8_t data_len=8,data[8];

  serialize(&(snd->data[0]),&(input_rc->values[0]),2);
  serialize(&(snd->data[2]),&(input_rc->values[1]),2);
  serialize(&(snd->data[4]),&(input_rc->values[2]),2);
  serialize(&(snd->data[6]),&(input_rc->values[3]),2);
  return append_pkt(
          snd,
          MSG_TYPE_KILL,
          data_len,
          data);
}

int pack_input_rc_joysticks(elka_packet_s *snd, input_rc_s *input_rc) {
  uint8_t data_len=8,data[8];

  serialize(&(snd->data[0]),&(input_rc->values[0]),2);
  serialize(&(snd->data[2]),&(input_rc->values[1]),2);
  serialize(&(snd->data[4]),&(input_rc->values[2]),2);
  serialize(&(snd->data[6]),&(input_rc->values[3]),2);
  return append_pkt(
          snd,
          MSG_TYPE_KILL,
          data_len,
          data);
}

int pack_test_msg(elka_packet_s *snd) {
  uint8_t data_len=6,data[6]={0,1,0,1,0,1};
  return append_pkt(
          snd,
          MSG_TYPE_TEST,
          data_len,
          data);
  }

void serial_state_timeout_check(void *arg) {
  static hrt_abstime timeout_duration;
  timeout_duration = hrt_abstime(100000);
  if (hrt_absolute_time() - _prev_spektrum_update > timeout_duration) {
    _serial_state = SERIAL_STATE_NONE;
  }

}

void msg_set_serial_state(int msg_type, input_rc_s *input_rc) {
  if (!old_msg(input_rc->timestamp)) {
    if (input_rc->values[SPEKTRUM_KILL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_KILL;
    } else if (input_rc->values[SPEKTRUM_CONTROL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_SPEKTRUM;
    } else {
      // Must set to none if none of the switches are flipped
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