#ifndef SPEKTRUM_SERIAL_TEST_H
#define SPEKTRUM_SERIAL_TEST_H

#include <inttypes.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>


#define NUM_JOYSTICK_CHANNELS 4
// Channel for kill switch
#define SPEKTRUM_KILL_CHANNEL 5
// Channel for switch to resume control with Spektrum RC
#define SPEKTRUM_CONTROL_CHANNEL 6

#define POSITION_OFFSET UINT32_MAX/2

#define SERIAL_STATE_NONE 0
#define SERIAL_STATE_KILL 1
#define SERIAL_STATE_SPEKTRUM 2
#define SERIAL_STATE_LOCAL_POS 3
#define SERIAL_STATE_VISION_POS 4

#define MSG_TYPE_NONE 0
#define MSG_TYPE_RC 1
#define MSG_TYPE_INPUT_RC 2
#define MSG_TYPE_VISION_POS 3
#define MSG_TYPE_LOCAL_POS 4

void usage();
int spektrum_test_loop(int argc, char **argv);

// Returns array length on success, PX4_ERROR on failure
int pack_position_estimate(char *snd_arr,
                           vehicle_local_position_s *pos);
int pack_kill_msg(char *snd_arr, input_rc_s input_rc);
int pack_input_rc_joysticks(char *snd_arr, input_rc_s spektrum);
int pack_input_rc(char *snd_arr, input_rc_s spektrum);

void serial_state_timeout_check(void *arg);
void msg_set_serial_state(int msg_type, input_rc_s *input_rc);
void msg_set_serial_state(int msg_type, vehicle_local_position_s *pos);
inline bool check_state(int msg_type);

inline bool old_msg(hrt_abstime msg_time);
// microseconds
void set_old_msg_duration(hrt_abstime time);

#endif
