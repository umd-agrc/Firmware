#ifndef SPEKTRUM_SERIAL_TEST_H
#define SPEKTRUM_SERIAL_TEST_H

#include <inttypes.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <uORB/topics/vehicle_local_position.h>

#define NUM_JOYSTICK_CHANNELS 4
#define NUM_GAIN_CHANNELS 2
// Channel for kill switch
#define SPEKTRUM_KILL_CHANNEL 5
// Channel for switch to resume control with Spektrum RC
#define SPEKTRUM_CONTROL_CHANNEL 4
// Useful for pilot to send thrust command
#define SPEKTRUM_THRUST_CHANNEL 2

#define POSITION_OFFSET UINT32_MAX/2

#define SERIAL_HEADER_MSG_LEN 0
#define SERIAL_HEADER_MSG_TYPE 1
#define SERIAL_HEADER_DATA_OFFSET 4

// Distance offset of a local message from the global header
#define SERIAL_HEADER_LOCAL_MSG_OFFSET 1

#define SERIAL_HEADER_LEN 3
#define SERIAL_HEADER_ASSURANCE_BYTE 255

#define SERIAL_STATE_NONE 0
#define SERIAL_STATE_KILL 1
#define SERIAL_STATE_SPEKTRUM 2
#define SERIAL_STATE_VISION_POS 3
#define SERIAL_STATE_LOCAL_POS 4
#define SERIAL_STATE_SETPOINT 5
#define SERIAL_STATE_THRUST 6

#define MSG_TYPE_NONE 0
#define MSG_TYPE_KILL 1
#define MSG_TYPE_SPEKTRUM 2
#define MSG_TYPE_VISION_POS 3
#define MSG_TYPE_LOCAL_POS 4
#define MSG_TYPE_SETPOINT 5
#define MSG_TYPE_THRUST 6

#define RAW_ROLL_BASELINE 1500
#define RAW_PITCH_BASELINE 1500
// 916 here is minimum thrust stick value
#define RAW_THRUST_BASELINE 916 
#define RAW_YAW_BASELINE 1500

struct gains {
  int16_t pos_kp, pos_kd;
};

void usage();
int spektrum_test_loop(int argc, char **argv);

// Returns array length on success, PX4_ERROR on failure
int pack_position_estimate(char *snd_arr,
                           uint8_t pos_type, 
                           vehicle_local_position_s *pos);
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
void msg_set_serial_state(int msg_type, vehicle_local_position_s *pos);
inline bool check_state(int msg_type);

inline bool old_msg(hrt_abstime msg_time);
// microseconds
void set_old_msg_duration(hrt_abstime time);

void check_pos(
    vehicle_local_position_s *curr_pos,
    math::Vector<3>*prev_pos_error,
    math::Vector<3>*setpoint);

#endif
