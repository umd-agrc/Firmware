#pragma once

#include <ctype.h>
#include <stddef.h>

// Skip over whitespace
// Stop if *p is whitespace or NUL ('\0')
#define SKIP(p) while (*p && isspace(*p)) {p++;}

// Test whether *p is not NUL and not whitespace
#define WANT(p) *p && !isspace(*p)

#define ELKA_DIR "/home/linaro/.elka/"
#define FLIGHT_PLAN_DIR "flight_plan/"

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
#define SERIAL_STATE_POSE 3
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
//FIXME ELKA chip side change msg_type back to uint8 from int16
#define MSG_TYPE_ERROR 9 
#define MSG_TYPE_MOTOR_INPUTS 10
#define MSG_TYPE_PLAN_ELEMENT 11

#define RAW_ROLL_BASELINE 1500
#define RAW_PITCH_BASELINE 1500
// 916 here is minimum thrust stick value
#define RAW_THRUST_BASELINE 916 
#define RAW_YAW_BASELINE 1500

#define ELKA_DEBUG 1
//#define DEBUG_SERIAL 1
//#define DEBUG_SERIAL_WRITE 1
//#define DEBUG_NAVIGATOR 1
//#define DEBUG_SERIAL_TX 1
//#define DEBUG_CONTROLLER 1
//#define DEBUG_SERIAL_READ 1
//#define DEBUG_MGR_PARSE 1
#define DEBUG_POSE 1
//#define DEBUG_VISION_VELOCITY 1
//#define DEBUG_FILTER 1
//#define DEBUG_SPEKTRUM 1
//#define DEBUG_TRANSFORM 1
//#define DEBUG_TRANSFORM_ERROR 1
//#define DEBUG_SETPOINTS 1
#define DEBUG_HOVER_HOLD 1
//#define DEBUG_GAINS 1

#define LOG_POSE_ERROR 1
#define LOG_TRAJECTORY 1

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
#define PLAN_ELEMENT_NONE 0
#define PLAN_ELEMENT_CALIBRATE 1
#define PLAN_ELEMENT_CHECK 2
#define PLAN_ELEMENT_TAKEOFF 3
#define PLAN_ELEMENT_LAND 4
#define PLAN_ELEMENT_HOVER 5
#define PLAN_ELEMENT_TRAJECTORY 6

#define TEST_DEFAULT_THRUST 480
#define SPOOL_DEFAULT_THRUST 380
//#define HOVER_DEFAULT_THRUST 630 // pwm
//#define LAND_DEFAULT_THRUST 600 // pwm
#define TAKEOFF_DEFAULT_THRUST 590 // pwm
#define TAKEOFF_DTHRUST 8 // pwm
#define HOVER_DEFAULT_THRUST 590 // pwm
#define LAND_DEFAULT_THRUST 575 // pwm
#define HOVER_DEFAULT_HEIGHT (-1.0f) // m
#define HOVER_MIN_HEIGHT (-0.7f) // m
#define HOVER_START_HEIGHT (-0.4f) //m
#define LAND_DEFAULT_HEIGHT (-0.1f) // m
#define VERTICAL_DEFAULT_SPEED (-0.3f) // m/s
#define VERTICAL_MAX_SPEED (-3.0f) // m/s
#define LAND_DEFAULT_SPEED (-0.3f) // m/s
#define LAND_MAX_SPEED (-1.0f) // m/s
#define HORIZONTAL_DEFAULT_SPEED 0.5f // m/s
#define HORIZONTAL_MAX_SPEED 3.0f // m/s
#define PLAN_ELEMENT_DEFAULT_LEN 5.0f*HRT_ABSTIME_TO_SEC // Seconds
#define SETPOINT_DEFAULT_LEN 5.0f*HRT_ABSTIME_TO_SEC // Seconds
#define LANDING_SETPOINT_DEFAULT_LEN 1.0f*HRT_ABSTIME_TO_SEC // Seconds
#define TAKEOFF_SETPOINT_DEFAULT_LEN 0.5f*HRT_ABSTIME_TO_SEC // Seconds
#define SPOOL_DEFAULT_LEN 2.5f*HRT_ABSTIME_TO_SEC
#define SETPOINT_FUNCTION_ORDER 5

#define SETPOINT_PARAM_HOLD 0x01
#define SETPOINT_PARAM_HOLD_UNTIL_TIME 0x02
#define SETPOINT_PARAM_LAND 0x04
#define SETPOINT_PARAM_TAKEOFF 0x08

#define OLD_SETPOINT 0
#define NEW_SETPOINT 1
