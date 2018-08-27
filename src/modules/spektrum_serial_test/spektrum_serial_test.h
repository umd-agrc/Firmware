#ifndef SPEKTRUM_SERIAL_TEST_H
#define SPEKTRUM_SERIAL_TEST_H

#include <inttypes.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Vector.hpp>
#include <lib/mathlib/math/Quaternion.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vision_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_packet.h>

#include "serial_defines.h"
#include "basic_comm.h"
#include "basic_control.h"
#include "basic_navigator.h"

void usage();
int spektrum_test_loop(int argc, char **argv);

// Position estimate varies following sign convention as such:
// x: Back/Forward (+/-)
// y: Left/Right (+/-)
// z: Down/Up (+/-)
// Yaw left/Yaw right(-/+)
int pack_position_estimate(elka_packet_s *snd,
                           elka::BasicNavigator *nav);
int pack_spektrum_cmd(elka_packet_s *snd,
                      uint8_t msg_type,
                      input_rc_s *input_rc);
int pack_kill_msg(elka_packet_s *snd,
                  input_rc_s *input_rc);
// Spektrum rc varies as such:
// Thrust: [0 1000] (down up)
// Roll: [2000 1000] (left right)
// Pitch: [1000 2000] (back forward)
// Yaw: [2000 1000] (left right) TODO rearrange motors?
int pack_input_rc_joysticks(elka_packet_s *snd,
                            input_rc_s *spektrum);
int pack_test_msg(elka_packet_s *snd);


void serial_state_timeout_check(void *arg);
void msg_set_serial_state(int msg_type, hrt_abstime t);
void msg_set_serial_state(int msg_type, input_rc_s *input_rc);
void msg_set_serial_state(int msg_type,
                          vehicle_local_position_s *pos,
                          vehicle_attitude_s *att);
void msg_set_serial_state(int msg_type,
                          vision_velocity_s *vel);
inline bool check_state(int msg_type);

inline bool old_msg(hrt_abstime msg_time);
// microseconds
void set_old_msg_duration(hrt_abstime time);
#endif
