#ifndef SPEKTRUM_SERIAL_TEST_H
#define SPEKTRUM_SERIAL_TEST_H

#include <inttypes.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <lib/mathlib/math/Quaternion.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>

#include "serial_defines.h"
#include "basic_navigator.h"

struct gains_s {
  float kp, kd;
};

void usage();
int spektrum_test_loop(int argc, char **argv);

// Returns array length on success, PX4_ERROR on failure
int pack_position_estimate(char *snd_arr,
                           uint8_t pos_type, 
                           pose_stamped_s *curr_err);
int pack_position_estimate(char *snd_arr,
                           uint8_t pos_type, 
                           pose_stamped_s *curr_err,
                           uint16_t thrust);
int pack_spektrum_cmd(char *snd_arr,
                      uint8_t msg_type,
                      input_rc_s *input_rc);
int pack_kill_msg(char *snd_arr,
                  input_rc_s *input_rc);
int pack_input_rc_joysticks(char *snd_arr,
                            input_rc_s *spektrum);
/*
int pack_input_rc(char *snd_arr,
                  input_rc_s *spektrum);
*/


void write_serial_header(
    char *snd_arr,
    uint16_t len,
    uint8_t msg_type);

int append_serial_msg(
    char *snd_arr,
    uint8_t msg_type,
    uint16_t bytes,
    uint16_t len);

void serial_state_timeout_check(void *arg);
void msg_set_serial_state(int msg_type, input_rc_s *input_rc);
void msg_set_serial_state(int msg_type,
                          vehicle_local_position_s *pos,
                          vehicle_attitude_s *att);
inline bool check_state(int msg_type);

inline bool old_msg(hrt_abstime msg_time);
// microseconds
void set_old_msg_duration(hrt_abstime time);
#endif
