#pragma once

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

#define ELKA_DEBUG 1
#define DEBUG_SERIAL 1
#define DEBUG_SERIAL_WRITE 1
//#define DEBUG_POSE 1
//#define DEBUG_FILTER 1
//#define DEBUG_SPEKTRUM 1
//#define DEBUG_TRANSFORM 1

