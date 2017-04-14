#ifndef ELKA_H
#define ELKA_H

#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#include <drivers/drv_hrt.h>

#define MAX_ELKA_DEVS 2
#define MAX_SERIAL_PORTS 6

// Max device name length in characters
#define MAX_NAME_LEN 80

#define MAX_MSG_LEN 254
#define ACK_LEN 1

// Define buffer types for TX/RX data
#define NO_BUF 0
#define CHAR_ARRAY 1
#define UINT8_ARRAY 2
#define RINGBUF 3
#define PRIORITY_QUEUE 4

//#define (this assumes hrt_abstime is an integer)
extern const hrt_abstime msg_threshold;

// Define msg id struct
struct elka_msg_id_s {
  uint8_t snd_id, rcv_id, type, length;
};

// Define send/receive id struct
struct snd_rcv_id_s {
  uint8_t port_num, port_type, proc_side;
};

// Define id bitwise values
#define PORT_NUM 0xe0
#define PORT_TYPE 0x1c
#define PROC_SIDE 0x03

// Define port types
#define PORT_NONE 0x00
#define PORT_UART 0x01

// Define sender/receiver id values
#define POSIX_SIDE 0x00
#define QURT_SIDE 0x01
//FIXME add ELKA_SIDE into sender/receiver id
//      => proc_side needs to be 2b
//      => port_num can be 3b
//      => PORT_NUM, PORT_TYPE, and PROC_SIDE macros must change
#define ELKA_SIDE 0x02 

// Sender/receiver id format (uint8_t)
// 3b port_num , 3b port_type , 2b proc_side
/// port_num = (id & PORT_NUM) >> 5
/// port_type = (id & PORT_TYPE) >> 2
/// proc_side = (id & PROC_SIDE)

// Define elka_msg bitwise values
#define SENDER_ID 0xff000000
#define RECEIVER_ID 0x00ff0000
#define MESSAGE_TYPE 0x0000ff00
#define MESSAGE_LENGTH 0x000000ff
// EXPECTING_ACK is a part of MESSAGE_TYPE spec
#define ID_EXPECTING_ACK 0x0100
#define TYPE_EXPECTING_ACK 0x01

// Define message types for elka_msg, elka_msg_ack, and elka_ctl
// Odd message types expect ack
// Even message types do not expect ack
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
// Control Snapdragon Flight ports
#define MSG_PORT_CTL 0x01
// Control PX4 ELKA device
#define MSG_ELKA_CTL 0x03
#define MSG_ACK_LENGTH 1

// msg_id
// 8b sender id , 8b receiver id , 8b msg_type , 8b length
/// sender_id = (msg_id & SENDER_ID) >> 24
/// receiver_id = (msg_id & RECEIVER_ID) >> 16
/// type = (msg_id & MESSAGE_TYPE) >> 8
//// expecting_ack = (msg_id & ID_EXPECTING ACK) >> 8
//// expecting_ack = (msg_type & TYPE_EXPECTING_ACK)
/// length = (msg_id & MESSAGE_LENGTH)
//// length of MSG_ACK is 1

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

// Call this in Device instantiation to get send/receive id
// Set snd_id and rcv_id
void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t port_num, uint8_t port_type,
    uint8_t snd_side, uint8_t rcv_side);

// Get snd/rcv ids from snd_rcv_id_struct
void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    struct snd_rcv_id_s &snd_id_struct,
    struct snd_rcv_id_s &rcv_id_struct);

// Get snd/rcv ids from msg_id
void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint32_t msg_id);

// Get snd/rcv attrs from snd_id or rcv_id
void get_snd_rcv_id_attr(uint8_t *port_num, uint8_t *port_type,
    uint8_t *proc_side,
    uint8_t id);

// Get snd/rcv attrs from snd_rcv
void get_snd_rcv_id_attr(struct snd_rcv_id_s *id_struct, uint8_t id);

// Call this before sending elka_msg or elka_msg_ack to set msg_id field
void get_elka_msg_id(uint32_t *msg_id,
    uint8_t snd_id, uint8_t rcv_id,
    uint8_t msg_type, uint8_t length);

void get_elka_msg_id(uint32_t *msg_id, struct elka_msg_id_s &msg_id_struct);

void get_elka_msg_id_attr(struct elka_msg_id_s *msg_id_struct, uint32_t msg_id);

void get_elka_msg_id_attr(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t *msg_type, uint8_t *length,
    uint32_t msg_id);

// Check ELKA ack against known msg_id and msg_num
// @param elka_msg_ack ack message
// @param msg_id id of sent message
// @param msg_num
// @return message result if message correct
//         elka_msg_ack_s::ACK_FAILED if message incorrect
//         elka_msg_ack_s::ACK_NULL if ack not for u
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    uint32_t &msg_id, uint16_t &msg_num);

// Check ELKA ack against known msg_id and msg_num
// @param elka_msg_ack ack message
// @param msg_id id of sent message
// @param msg_num
// @return message result if message correct
//         elka_msg_ack_s::ACK_FAILED if message incorrect
//         elka_msg_ack_s::ACK_NULL if ack not for u
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    struct elka_msg_id_s &msg_id, uint16_t &msg_num);

// Buffer print convenience methods
void print_char_array(char *buf);
void print_uint8_array(uint8_t *buf);
//TODO
//void print_ringbuf(pringbuf_t buf);

#endif
