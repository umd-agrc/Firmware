#ifndef ELKA_H
#define ELKA_H

#include <ctime>
#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#if defined(__PX4_QURT) || defined(__PX4_POSIX)

//#include <drivers/bootloaders/include/random.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_time.h>

#endif

//FIXME copy random stuff from <drivers/bootloaders/include/random.h>
//      because of linking error
void util_srand(uint16_t seed);
uint16_t util_random(uint16_t min, uint16_t max);

#define MAX_ELKA_DEVS 2
#define MAX_SERIAL_PORTS 6

// Max device name length in characters
#define MAX_NAME_LEN 80
#define MSG_ID_LEN sizeof(uint32_t)
#define MSG_NUM_LEN 0xffff // Max possible message number + 1
#define RECENT_ACKS_LEN 0xff // Length of _recent_acks array
#define MAX_MSG_LEN 0xff

// Define buffer types for TX/RX data
#define NO_BUF 0
#define ARRAY 3
#define PRIORITY_QUEUE 4

// Define threshold for expired message
//(this assumes hrt_abstime is an integer)
extern const hrt_abstime msg_threshold;

// Define device id type
typedef uint16_t dev_id_t;

// Define device properties type
typedef uint8_t dev_prop_t;

// Define msg_id type
typedef uint64_t msg_id_t;

// Print flags for printf statements
#define PRDIT PRIu16
#define PRDPT PRIu8
#define PRMIT PRIu64

// Define msg id struct
struct elka_msg_id_s {
  dev_id_t snd_id, rcv_id;
  uint8_t snd_params, type, length;
};

// Define send params struct
struct snd_params_s {
  uint8_t port_num, port_type, proc_side;
};

// Define max id number
#define ID_MAX UINT16_MAX

// Define id bitwise values
#define PORT_NUM 0xe0
#define PORT_TYPE 0x1c
#define PROC_SIDE 0x03

// Define port types
#define PORT_NONE 0x00
#define PORT_UART 0x01

// Message format thru UART:
// 8B msg_id, 2B msg_num, <(MAX_MSG_LEN)B data

// Define sender/receiver id values
#define POSIX_SIDE 0x00
#define QURT_SIDE 0x01
#define ELKA_SIDE 0x02 

// Sender params format (uint8_t)
// 3b port_num , 3b port_type , 2b proc_side
/// port_num = (id & PORT_NUM) >> 5
/// port_type = (id & PORT_TYPE) >> 2
/// proc_side = (id & PROC_SIDE)

// Define elka_msg bitwise values
#define SENDER_ID           0xffff000000000000
#define RECEIVER_ID         0xffff00000000
#define SENDER_PARAMS       0xff000000
// TODO room in msg_id for more stuff
//      RECEIVER_PARAMS is now filler
#define RECEIVER_PARAMS     0xff0000
#define MESSAGE_TYPE        0xff00
#define MESSAGE_LENGTH      0xff
// EXPECTING_ACK is a part of MESSAGE_TYPE spec
#define ID_EXPECTING_ACK 0x0100
#define TYPE_EXPECTING_ACK 0x01

// Message is broadcast if receiver id is equal to BROADCAST_MSG_ID
#define BROADCAST_MSG_ID (dev_id_t)0x00

// Define message types for elka_msg, elka_msg_ack, and elka_ctl
// Odd message types expect ack
// Even message types do not expect ack
// ROUTING msg format
//    ROUTE_DEV_PROPS
//      {msg_header, requesting response (T/F),
//       dev props len, dev props}
//      This message can be for initial network building as a
//      broadcast or a response
//    ROUTE_REQUEST_HB:
//      {msg_header, req}
//      1b request heartbeat
//    ROUTE_HB:
//      {msg_header, heartbeat}
//      1b heartbeat
//    ROUTE_CHANGED:
//      {msg_header, requesting response (T/F),
//       num deleted, deleted dev id,
//       num changed/added,
//       {changed/added dev id, dev route len, dev route,
//                              dev props len, dev props}_i,...}
//    ROUTE_TABLE:
//      {msg_header, requesting response (T/F), num devices,
//       {dev id, dev route len, dev route,
//                dev props len, dev props}_i,...}
#define MSG_NULL 0x00
// Message incorrect
// Possible modes of failure are:
//    Message sent to wrong port
//    Message of wrong format
#define MSG_FAILED 0x02
// Send command to motors
#define MSG_MOTOR_CMD 0x04
// Ack message
#define MSG_ACK 0x06
// Build routing table
#define MSG_ROUTE_DEV_PROPS 0x08
#define MSG_ROUTE_REQUEST_HB 0x0a
#define MSG_ROUTE_HB 0x0c
//TODO change PORT_CTL to HW_CTL and
//            ELKA_CTL to SW_CTL
// Control Snapdragon Flight ports
#define MSG_PORT_CTL 0x03
// Control PX4 ELKA device
#define MSG_ELKA_CTL 0x05
#define MSG_ROUTE_CHANGED 0x07
#define MSG_ROUTE_TABLE 0x09

#define MSG_ACK_LENGTH 1
#define MSG_STATE_LENGTH 1

//TODO If get rid of sender params
//     then can add msg_num as a part
//     of msg_id
// msg_id (64b)
// 16b sender id , 16b receiver id ,
// 8b sender params , 8b filler (TODO)
// 8b msg_type , 8b length
/// snd_id = (msg_id & SENDER_ID) >> 48
/// rcv_id = (msg_id & RECEIVER_ID) >> 32
/// snd_params = (msg_id & SENDER_PARAMS) >> 24
/// filler = (msg_id & RECEIVER_PARAMS) >> 16
/// type = (msg_id & MESSAGE_TYPE) >> 8
//// expecting_ack = (msg_id & ID_EXPECTING ACK) >> 8
//// expecting_ack = (msg_type & TYPE_EXPECTING_ACK)
/// length = (msg_id & MESSAGE_LENGTH)
//// length of MSG_ACK is 1

// Max msgs retries for msgs expecting acks
#define MAX_NUM_RETRIES 21

// <ELKA_|PORT_>CTL msg format
// 8b action
// State STATE_NULL means port not set up
// State STATE_START means setup and transition to STATE_RESUME
// State STATE_STOP means tear down
// State STATE_PAUSE means temporarily stop sending messages
// State STATE_RESUME means send messages
#define STATE_NULL 0x00
#define STATE_START 0x01
#define STATE_STOP 0x02
#define STATE_PAUSE 0x03
#define STATE_RESUME 0x04

// Define device properties.
// These are properties from one device to another device.
// Eg: A device is connected to another device by
//     hardware 
//     A device has permission to use the motors connected 
//     to another device
#define DEV_PROP_POSIX_SIDE (dev_prop_t)0x00
#define DEV_PROP_QURT_SIDE (dev_prop_t)0x01
#define DEV_PROP_ELKA_SIDE (dev_prop_t)0x02 
#define DEV_PROP_HW_CONNECTED (dev_prop_t)0x03
#define DEV_PROP_WIRELESS_CONNECTED (dev_prop_t)0x04
#define DEV_PROP_GROUND_STATION (dev_prop_t)0x05
#define DEV_PROP_FLIGHT_CONTROLLER (dev_prop_t)0x06
#define DEV_PROP_PERFORM_LOCALIZATION (dev_prop_t)0x07
#define DEV_PROP_SENSE_LOCATION (dev_prop_t)0x08
#define DEV_PROP_SPIN_MOTORS (dev_prop_t)(dev_prop_t)0x09
#define DEV_PROP_HAS_MOTORS (dev_prop_t)0x0a
#define DEV_PROP_USE_CAMERA (dev_prop_t)0x0b
#define DEV_PROP_HAS_CAMERA (dev_prop_t)0x0c
#define DEV_PROP_TRANSMISSION_CTL (dev_prop_t)0x0d

// Call this in Device instantiation to get device id
void get_dev_id_t(dev_id_t *d);

// Check dev_id_t equivalence
// @return -1 if d1 < d2
//         1 if d1 > d2
//         0 if d1 == d2
inline int8_t cmp_dev_id_t(dev_id_t d1, dev_id_t d2) {
  if (d1 < d2) return -1;
  else if (d1 > d2) return 1;
  else return 0;
}

// Used for sorting routing table
// Routing table is sorted so that closest devices
// are near the front
struct dev_id_tCmp {
  bool operator()(const dev_id_t &d1, const dev_id_t &d2) const {
    return d1 < d2;
  }
};

// Check dev_prop_t equivalence
// @return -1 if d1 < d2
//         1 if d1 > d2
//         0 if d1 == d2
inline int8_t cmp_dev_prop_t(dev_prop_t d1, dev_prop_t d2) {
  if (d1 < d2) return -1;
  else if (d1 > d2) return 1;
  else return 0;
}

struct dev_prop_tCmp {
  bool operator()(const dev_prop_t &d1, const dev_prop_t &d2) {
    return d1 < d2;
  }
};

// Call this in Device instantiation to get send params
// Set snd_params
void get_snd_params(uint8_t *snd_params,
    uint8_t port_num, uint8_t port_type,
    uint8_t snd_side);

// Get snd ids from snd_params_struct
void get_snd_params(uint8_t *snd_params,
    struct snd_params_s &snd_params_struct);

// Get snd ids from msg_id
void get_snd_params(uint8_t *snd_params,
    msg_id_t msg_id);

// Get snd attrs from snd_params
void get_snd_params_attr(uint8_t *port_num,
    uint8_t *port_type,
    uint8_t *proc_side,
    uint8_t params);

// Get snd attrs from snd_params
void get_snd_params_attr(
    struct snd_params_s *params_struct,
    uint8_t params);

// Call this before sending elka_msg or elka_msg_ack to set msg_id field
void get_elka_msg_id(msg_id_t *msg_id,
    dev_id_t snd_id, dev_id_t rcv_id,
    uint8_t snd_params, uint8_t msg_type, uint8_t length);

void get_elka_msg_id(msg_id_t *msg_id,
    struct elka_msg_id_s &msg_id_struct);

void get_elka_msg_id_attr(struct elka_msg_id_s *msg_id_struct,
    msg_id_t msg_id);

void get_elka_msg_id_attr(
    dev_id_t *snd_id, dev_id_t *rcv_id,
    uint8_t *snd_params, uint8_t *msg_type, uint8_t *length,
    msg_id_t msg_id);

// Return true if message is broadcast message
inline bool broadcast_msg(msg_id_t &msg_id) {
  dev_id_t rcv_id;

  get_elka_msg_id_attr(NULL,&rcv_id,NULL,NULL,NULL,
      msg_id);

  return !cmp_dev_id_t(rcv_id,BROADCAST_MSG_ID);
}

inline bool broadcast_msg(dev_id_t &rcv_id) {
  return !cmp_dev_id_t(rcv_id,BROADCAST_MSG_ID);
}

inline bool initial_msg(msg_id_t &msg_id) {
  dev_id_t snd_id, rcv_id;

  get_elka_msg_id_attr(&snd_id,&rcv_id,NULL,NULL,NULL,
      msg_id);

  return !( cmp_dev_id_t(snd_id,(dev_id_t)0) || 
            cmp_dev_id_t(rcv_id,(dev_id_t)0) );
}

// Check ELKA ack against known msg_id and msg_num
// @param elka_msg_ack ack message
// @param msg_id id of sent message
// @param msg_num
// @return message result if message correct
//         elka_msg_ack_s::ACK_FAILED if message incorrect
//         elka_msg_ack_s::ACK_NULL if ack not for u
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    msg_id_t &msg_id, uint16_t &msg_num, uint8_t num_retries);

// Check ELKA ack against known msg_id and msg_num
// @param elka_msg_ack ack message
// @param msg_id id of sent message
// @param msg_num
// @return message result if message correct
//         elka_msg_ack_s::ACK_FAILED if message incorrect
//         elka_msg_ack_s::ACK_NULL if ack not for u
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    struct elka_msg_id_s &msg_id, uint16_t &msg_num, uint8_t num_retries);

// Set elka state in elka_msg. May push this to a buffer after.
// @param elka_snd message to be set
// @param state state to set in message
inline void set_state_msg(elka_msg_s &elka_snd, uint8_t state,
    dev_id_t snd_id, dev_id_t rcv_id,
    uint8_t snd_params, uint8_t msg_t, uint8_t msg_len) {
  get_elka_msg_id(&elka_snd.msg_id,
      snd_id, rcv_id, snd_params, msg_t, msg_len);
  elka_snd.data[0] = state;
}

// Serialization of elka_msg and elka_msg_ack
// Serialize multi-byte elements with low byte first
void serialize_elka_msg(uint8_t *ret, elka_msg_s &elka_msg);
void serialize_elka_msg_ack(uint8_t *ret, elka_msg_ack_s &elka_ack);

// Elka msg and Elka ack print methods
void print_elka_msg_id(msg_id_t &msg_id);
void print_elka_msg(elka_msg_s &elka_msg);
void print_elka_msg_ack(elka_msg_ack_s &elka_msg);
void print_elka_routing_msg(elka_msg_s &elka_msg);

// Buffer print convenience methods
void print_array(uint8_t *buf, uint8_t len);
void print_elka_serial_array(uint8_t *buf);
void print_uint8_array(uint8_t *buf, uint8_t len);
void print_char_array(char *buf, uint8_t len);

// Circular buffer methods
// cb_max_size is max buffer size before wrapping
void print_cb(uint16_t *cb, uint16_t cb_end, uint16_t cb_len,
    uint16_t cb_max_size);

int cb_bin_search(uint16_t el, uint16_t *cb,
    uint16_t cb_end, uint16_t cb_len, uint16_t cb_max_size);

void cb_push(uint16_t el, uint16_t *cb,
    uint16_t &cb_end, uint16_t &cb_len, uint16_t cb_max_size);

void cb_insertion_sort(uint16_t *cb, uint16_t cb_end,
    uint16_t cb_len, uint16_t cb_max_size);

uint16_t get_nxt_idx(uint16_t *cb, uint16_t cb_end,
    uint16_t cb_len, uint16_t cb_max_size, uint16_t i);

#endif
