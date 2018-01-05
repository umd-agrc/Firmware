#pragma once

#include <ctype.h>
#include <stddef.h>

// Skip over whitespace
// Stop if *p is whitespace or NUL ('\0')
#define SKIP(p) while (*p && isspace(*p)) {p++;}

// Test whether *p is not NUL and not whitespace
#define WANT(p) *p && !isspace(*p)

#define ELKA_DIR "/dev/fs/.elka/"
#define FLIGHT_PLAN_DIR "flight_plan/"
#define NN_DIR "nn/"

#define NUM_JOYSTICK_CHANNELS 4
#define NUM_GAIN_CHANNELS 2
// Channel for kill switch
#define SPEKTRUM_KILL_CHANNEL 5
// Channel for switch to resume control with Spektrum RC
#define SPEKTRUM_CONTROL_CHANNEL 4
// Useful for pilot to send thrust command
#define SPEKTRUM_THRUST_CHANNEL 2

#define HRT_ABSTIME_TO_SEC 1e6

#define ELKA_MSG_PACKET_LEN 0
#define ELKA_MSG_LEN 0
#define ELKA_MSG_TYPE 1
#define ELKA_MSG_DATA_OFFSET 4

// Distance offset of a local message from the global header
#define ELKA_MSG_LOCAL_MSG_OFFSET 1

#define ELKA_MSG_HEADER_LEN 3
#define ELKA_MSG_HEADER_ASSURANCE_BYTE 255

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
#define MSG_TYPE_GAINS 7
#define MSG_TYPE_TEST 8
#define MSG_TYPE_ERROR 8

#define RAW_ROLL_BASELINE 1500
#define RAW_PITCH_BASELINE 1500
// 916 here is minimum thrust stick value
#define RAW_THRUST_BASELINE 916 
#define RAW_YAW_BASELINE 1500

#define ELKA_DEBUG 1
#define DEBUG_SERIAL 1
#define DEBUG_SERIAL_WRITE 1
#define DEBUG_CONTROLLER 1
//#define DEBUG_SERIAL_READ 1
//#define DEBUG_MGR_PARSE 1
//#define DEBUG_POSE 1
//#define DEBUG_FILTER 1
//#define DEBUG_SPEKTRUM 1
//#define DEBUG_TRANSFORM 1

#define SENSOR_NONE 0
#define SENSOR_ACCEL 1
#define SENSOR_GYRO 2
#define SENSOR_MAG 3
#define SENSOR_BARO 4
#define SENSOR_CAM 5

// Define max message lengths
#define MAX_SERIAL_MSG_LEN 256
#define MAX_ELKA_MSG_LEN 55
#define MAX_ELKA_PACKET_LEN 55

#define ELKA_SUCCESS 0
#define SERIAL_ERROR -2
#define MSG_ERROR -3
#define PKT_ERROR -4
#define MSG_MGR_ERROR -5
#define PARSE_ERROR -6

// Generic max array length if desire to keep
// array length smallish
#define MAX_ARR_LEN 128

// Flight plan defines
#define PLAN_ELEMENT_CALIBRATE 0
#define PLAN_ELEMENT_CHECK 1
#define PLAN_ELEMENT_TAKEOFF 2
#define PLAN_ELEMENT_LAND 3
#define PLAN_ELEMENT_HOVER 4

#define HOVER_DEFAULT_HEIGHT 1.5f
#define PLAN_ELEMENT_DEFAULT_LEN 3.0f*HRT_ABSTIME_TO_SEC // Seconds
