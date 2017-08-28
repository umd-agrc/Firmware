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
#include <uORB/topics/vehicle_local_position.h>

#include "basic_uart.h"
#include "spektrum_serial_test.h"

extern "C" { __EXPORT int spektrum_serial_test_main(int argc, char *argv[]); }

static int daemon_task;
static volatile bool thread_should_exit;
static volatile bool thread_running;
int _serial_state;
hrt_abstime _prev_spektrum_update, _old_msg_duration;
struct hrt_call	_serial_state_call;
static int _serial_state_call_interval;

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
  char snd_arr[MAX_MSG_LEN];
  int snd_arr_len = 0,
      spektrum_fd = -1;

  // Open file for UART
  spektrum_fd = spektrum_serial_open(); 

  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to elka msg, elka msg ack, and input_rc (TODO only if necessary)
  // Vision position omes from MAVLink SLAM pose estimate
  // Local position comes from LPE BlockLocalPositionEstimator
  int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
  int estimator_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
  int vision_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_position));

  // Set update rate to 100Hz
  orb_set_interval(input_rc_sub_fd, 10);
  orb_set_interval(estimator_pos_sub_fd, 10);
  orb_set_interval(vision_pos_sub_fd, 10);

  struct input_rc_s input_rc;
  memset(&input_rc, 0, sizeof(input_rc));
  vehicle_local_position_s estimator_pos, vision_pos;
  memset(&estimator_pos, 0, sizeof(estimator_pos));
  memset(&vision_pos, 0, sizeof(vision_pos));

  px4_pollfd_struct_t fds[] = {
    {.fd = input_rc_sub_fd, .events = POLLIN},
    {.fd = estimator_pos_sub_fd, .events = POLLIN},
    {.fd = vision_pos_sub_fd, .events = POLLIN},
  };

  // Set old message duration to 1/10 s
  set_old_msg_duration(100000);
  _serial_state_call_interval = 1;
  hrt_call_every(&_serial_state_call, 0,
           (_serial_state_call_interval * 1000),
           (hrt_callout)&serial_state_timeout_check,
           nullptr);

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
        msg_set_serial_state(MSG_TYPE_INPUT_RC, &input_rc);

        /*
        PX4_INFO("channel values\n1:\t%d\n2:\t%d\n3:\t%d\n4:\t%d",
            input_rc.values[0],input_rc.values[1],input_rc.values[2],
            input_rc.values[3]);
        */

        if (_serial_state == SERIAL_STATE_KILL) {
          snd_arr_len = pack_kill_msg(snd_arr, input_rc);
        } else if (_serial_state == SERIAL_STATE_SPEKTRUM) {
          snd_arr_len = pack_input_rc_joysticks(snd_arr, input_rc);
        }

        if (check_state(MSG_TYPE_RC) &&
            (spektrum_serial_write(spektrum_fd,
                                  snd_arr, snd_arr_len) !=
             spektrum_fd)) {
          PX4_WARN("Spektrum serial write failed for msg with timestamp: %lu",
              input_rc.timestamp_last_signal);
        } else {
          usleep(20000);
        }
      }

      if (fds[1].revents & POLLIN) { // estimator_pos 
        orb_copy(ORB_ID(vehicle_local_position),
                 estimator_pos_sub_fd,
                 &estimator_pos);
        msg_set_serial_state(MSG_TYPE_LOCAL_POS, &estimator_pos);

        if (check_state(MSG_TYPE_LOCAL_POS) &&
            (snd_arr_len = pack_position_estimate(snd_arr,
                                                  &estimator_pos))
             != PX4_ERROR) {
          if (spektrum_serial_write(spektrum_fd,
                                    snd_arr, snd_arr_len) !=
              spektrum_fd) {
            PX4_WARN("Spektrum serial write failed for msg with timestamp: %lu",
                estimator_pos.timestamp);
          } else {
            usleep(20000);
          }
        }
      }

      if (fds[2].revents & POLLIN) { // vision_pos 
        orb_copy(ORB_ID(vehicle_vision_position),
                 vision_pos_sub_fd,
                 &vision_pos);
        msg_set_serial_state(MSG_TYPE_VISION_POS, &vision_pos);
        if (check_state(MSG_TYPE_VISION_POS) &&
            (snd_arr_len = pack_position_estimate(snd_arr,
                                                  &vision_pos))
             != PX4_ERROR) {
          if (spektrum_serial_write(spektrum_fd,
                                    snd_arr, snd_arr_len) !=
              spektrum_fd) {
            PX4_WARN("Spektrum serial write failed for msg with timestamp: %lu",
                vision_pos.timestamp);
          } else {
            usleep(20000);
          }
        }
      }
    }
  }

  spektrum_fd = spektrum_serial_close(spektrum_fd); 

  thread_running = false;

  return PX4_OK;
}

int pack_position_estimate(char *snd_arr,
                           vehicle_local_position_s *pos) {
  int num_bytes = 0;
  uint32_t x,y,z;
  bool continue_packing = true;

  x = (uint32_t)(pos->x*10000 + POSITION_OFFSET);
  y = (uint32_t)(pos->y*10000 + POSITION_OFFSET);
  z = (uint32_t)(pos->z*10000 + POSITION_OFFSET);

  while (num_bytes < MAX_MSG_LEN &&
         continue_packing) {
    if (num_bytes < 1 &&
        (MAX_MSG_LEN - num_bytes >= 1)) {
      // Pack length:
      *snd_arr = 15;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 4 &&
        (MAX_MSG_LEN - num_bytes >= 3)) {
      // Pack header defined as follows:
      // state, 255, 255
      //FIXME distinguish between vision_pos and local_pos
      *snd_arr = SERIAL_STATE_LOCAL_POS;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 15 &&
        (MAX_MSG_LEN - num_bytes >= 6)) {
      *snd_arr = get_byte_n(x,1);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(x,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(x,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(x,4);
      snd_arr++; num_bytes++;

      *snd_arr = get_byte_n(y,1);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(y,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(y,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(y,4);
      snd_arr++; num_bytes++;

      *snd_arr = get_byte_n(z,1);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(z,2);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(z,3);
      snd_arr++; num_bytes++;
      *snd_arr = get_byte_n(z,4);
      snd_arr++; num_bytes++;
      continue_packing = false;
    } else {
      PX4_ERR("position estimate could not be packed. Message too long.");
      continue_packing = false;
      return PX4_ERROR;
    }
  }

  return num_bytes;
}


int pack_kill_msg(char *snd_arr, input_rc_s input_rc) {
  int num_bytes = 0;
  bool continue_packing = true;

  while (num_bytes < MAX_MSG_LEN &&
         continue_packing) {
    if (num_bytes < 1 &&
          (MAX_MSG_LEN - num_bytes >= 1)) {
        // Pack length:
        *snd_arr = 3;
        snd_arr++; num_bytes++;
      }
      else if (num_bytes < 4 &&
          (MAX_MSG_LEN - num_bytes >= 4)) {
        // Pack header defined as follows:
        //    0, 255, 0, 255
        *snd_arr = SERIAL_STATE_KILL;
        snd_arr++; num_bytes++;
        *snd_arr = 255;
        snd_arr++; num_bytes++;
        *snd_arr = 255;
        snd_arr++; num_bytes++;

      continue_packing = false;
    } else {
      PX4_ERR("position estimate could not be packed. Message too long.");
      continue_packing = false;
      return PX4_ERROR;
    }
  }

  return num_bytes;
}

int pack_input_rc_joysticks(char *snd_arr, input_rc_s spektrum) {
  int num_bytes = 0;
  bool continue_packing = true;

  while (num_bytes < MAX_MSG_LEN &&
         continue_packing) {

    if (num_bytes < 1 &&
        (MAX_MSG_LEN - num_bytes >= 1)) {
      // Pack length:
      *snd_arr = 11;
      snd_arr++; num_bytes++;
    }
    else if (num_bytes < 4 &&
        (MAX_MSG_LEN - num_bytes >= 4)) {
      // Pack header defined as follows:
      //    0, 255, 0, 255
      *snd_arr = SERIAL_STATE_SPEKTRUM;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 4+2*NUM_JOYSTICK_CHANNELS &&
        (MAX_MSG_LEN - num_bytes >= 2*NUM_JOYSTICK_CHANNELS)) {
      // up to 36B channel pwms ranging from 1000-2000
      for (uint8_t i=0; i < NUM_JOYSTICK_CHANNELS; i++){
        *snd_arr = (spektrum.values[i] >> 8) & 0xff;
        snd_arr++; num_bytes++;
        *snd_arr = spektrum.values[i] & 0xff;
        snd_arr++; num_bytes++;
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

// Multi-byte elements are packed LSB first
int pack_input_rc(char *snd_arr, input_rc_s spektrum) {
  int num_bytes = 1;
  bool continue_packing = true;

  /*
  *snd_arr = 0;
  snd_arr++; num_bytes++;
  *snd_arr = 255;
  snd_arr++; num_bytes++;
  *snd_arr = 0;
  snd_arr++; num_bytes++;
  *snd_arr = 255;
  snd_arr++; num_bytes++;
  continue_packing = false;
  */
  while (num_bytes < MAX_MSG_LEN &&
         continue_packing) {
    // Pack header defined as follows:
    //    0, 255, 0, 255
    if (num_bytes < 4 &&
        (MAX_MSG_LEN - num_bytes >= 4)) {
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 0;
      snd_arr++; num_bytes++;
      *snd_arr = 255;
      snd_arr++; num_bytes++;
      *snd_arr = 0;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 12 &&
        (MAX_MSG_LEN - num_bytes >= 8)) {
      // Pack timestamp by splitting into 8 bytes
      *snd_arr = spektrum.timestamp_last_signal & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 8) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 16) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 24) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 32) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 40) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 48) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.timestamp_last_signal >> 56) & 0xff; 
      snd_arr++; num_bytes++;
    } else if (num_bytes < 16 &&
          (MAX_MSG_LEN - num_bytes >= 4)) {
      // 4B channel count
      *snd_arr = spektrum.channel_count & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.channel_count >> 8) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.channel_count >> 16) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.channel_count >> 24) & 0xff; 
      snd_arr++; num_bytes++;
    } else if (num_bytes < 20 &&
        (MAX_MSG_LEN - num_bytes >= 4)) {
      // 4B receive signal strength indicator (RSSI)
      *snd_arr = spektrum.rssi & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rssi >> 8) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rssi >> 16) & 0xff; 
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rssi >> 24) & 0xff; 
      snd_arr++; num_bytes++;
    } else if (num_bytes < 21 &&
      // 2B rc_failsafe
        (MAX_MSG_LEN - num_bytes >= 1)) {
      *snd_arr = spektrum.rc_failsafe & 0xff;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 22 &&
        (MAX_MSG_LEN - num_bytes >= 1)) {
      // 2B rc lost this frame
      *snd_arr = spektrum.rc_lost & 0xff;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 24 &&
        (MAX_MSG_LEN - num_bytes >= 2)) {
      // 2B rc lost frame count
      *snd_arr = spektrum.rc_lost_frame_count & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rc_lost_frame_count >> 8) & 0xff;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 26 &&
        (MAX_MSG_LEN - num_bytes >= 2)) {
      // 2B rc total frame count
      *snd_arr = spektrum.rc_total_frame_count & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rc_total_frame_count >> 8) & 0xff;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 28 &&
        (MAX_MSG_LEN - num_bytes >= 2)) {
      // 2B rc ppm frame length
      *snd_arr = spektrum.rc_ppm_frame_length & 0xff;
      snd_arr++; num_bytes++;
      *snd_arr = (spektrum.rc_ppm_frame_length >> 8) & 0xff;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 29 &&
        (MAX_MSG_LEN - num_bytes >= 2)) {
      // 1B input sources
      *snd_arr = spektrum.input_source;
      snd_arr++; num_bytes++;
    } else if (num_bytes < 29+2*spektrum.channel_count &&
        (MAX_MSG_LEN - num_bytes >= 2*spektrum.channel_count)) {
      // up to 36B channel pwms ranging from 1000-2000
      for (int i=0; i < spektrum.channel_count; i++){
        *snd_arr = spektrum.values[i] & 0xff;
        snd_arr++; num_bytes++;
        *snd_arr = (spektrum.values[i] >> 8) & 0xff;
        snd_arr++; num_bytes++;
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

void serial_state_timeout_check(void *arg) {
  static hrt_abstime timeout_duration;
  timeout_duration = _old_msg_duration;
  if (hrt_abstime() - _prev_spektrum_update > timeout_duration) {
    _serial_state = SERIAL_STATE_NONE;
  }
}

void msg_set_serial_state(int msg_type, input_rc_s *input_rc) {
  bool state_set = false;
  if (!old_msg(input_rc->timestamp)) {
    if (input_rc->values[SPEKTRUM_KILL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_KILL;
      state_set = true;
    } else if (input_rc->values[SPEKTRUM_CONTROL_CHANNEL] > 1500) {
      _serial_state = SERIAL_STATE_SPEKTRUM;
      state_set = true;
    }
    _prev_spektrum_update = hrt_absolute_time();
  }
}

void msg_set_serial_state(int msg_type, vehicle_local_position_s *pos) {
  if (!old_msg(pos->timestamp)) {
    if (_serial_state != SERIAL_STATE_KILL ||
        _serial_state != SERIAL_STATE_SPEKTRUM) {
      if (msg_type == MSG_TYPE_LOCAL_POS) {
        _serial_state = SERIAL_STATE_LOCAL_POS;
      } else if (msg_type == MSG_TYPE_VISION_POS &&
                 _serial_state != SERIAL_STATE_LOCAL_POS) {
        _serial_state = SERIAL_STATE_VISION_POS;
      }
    }
    _prev_spektrum_update = hrt_absolute_time();
  }
}

inline bool check_state(int msg_type) {
  switch(msg_type) {
    case MSG_TYPE_RC:
      return _serial_state == SERIAL_STATE_KILL ||
             _serial_state == SERIAL_STATE_SPEKTRUM;
    case MSG_TYPE_LOCAL_POS: 
      return _serial_state == SERIAL_STATE_LOCAL_POS;
    case MSG_TYPE_VISION_POS: 
      return _serial_state == SERIAL_STATE_VISION_POS;
    default:
      return false;
  }
}

inline bool old_msg(hrt_abstime msg_time) {
  return (hrt_abstime() - msg_time) < _old_msg_duration;
}

void set_old_msg_duration(hrt_abstime time) {
  _old_msg_duration = time;
}
