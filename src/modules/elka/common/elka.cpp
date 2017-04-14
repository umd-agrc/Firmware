#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_time.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "elka.h"

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t port_num, uint8_t port_type, 
    uint8_t snd_side, uint8_t rcv_side) {
  if (snd_id)
    *snd_id = (port_num << 5) | (port_type << 2) | (snd_side);
  if (rcv_id)
    *rcv_id = (port_num << 5) | (port_type << 2) | (rcv_side);
}

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    struct snd_rcv_id_s &snd_id_struct,
    struct snd_rcv_id_s &rcv_id_struct) {
  if (snd_id)
    *snd_id = (snd_id_struct.port_num << 5) |
              (snd_id_struct.port_type << 2) |
              (snd_id_struct.proc_side);

  if (rcv_id)
    *rcv_id = (rcv_id_struct.port_num << 5) |
              (rcv_id_struct.port_type << 2) |
              (rcv_id_struct.proc_side);
}

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint32_t msg_id) {
  if (snd_id && rcv_id)
    get_elka_msg_id_attr(snd_id, rcv_id, NULL, NULL, msg_id);
  else if (snd_id)
    get_elka_msg_id_attr(snd_id, NULL, NULL, NULL, msg_id);
  else if (rcv_id)
    get_elka_msg_id_attr(NULL, rcv_id, NULL, NULL, msg_id);
}

void get_snd_rcv_id_attr( uint8_t *port_num, uint8_t *port_type,
    uint8_t *proc_side,
    uint8_t id) {
  if (port_num)
    *port_num = (id & PORT_NUM) >> 5;
  if (port_type)
    *port_type = (id & PORT_TYPE) >> 2;
  if (proc_side)
    *proc_side = (id & PROC_SIDE);
}

void get_snd_rcv_id_attr(struct snd_rcv_id_s *id_struct, uint8_t id) {
  if (id_struct) {
    id_struct->port_num = (id & PORT_NUM) >> 5;
    id_struct->port_type = (id & PORT_TYPE) >> 2;
    id_struct->proc_side = (id & PROC_SIDE);
  }    
}

// Call this before sending elka_msg or elka_msg_ack to set msg_id field
void get_elka_msg_id(uint32_t *msg_id,
    uint8_t snd_id, uint8_t rcv_id,
    uint8_t msg_type, uint8_t length) {
  if (msg_id) {
    *msg_id = (snd_id << 24) | (rcv_id << 16) | (msg_type << 8) | (length);
  }
}

void get_elka_msg_id(uint32_t *msg_id, struct elka_msg_id_s &msg_id_struct) {
  if (msg_id) {
    *msg_id = (msg_id_struct.snd_id << 24) |
              (msg_id_struct.rcv_id << 16) |
              (msg_id_struct.type << 8) |
              (msg_id_struct.length);
  }
}

void get_elka_msg_id_attr(struct elka_msg_id_s *msg_id_struct, uint32_t msg_id) {
  if (msg_id_struct && msg_id) {
    msg_id_struct->snd_id = (msg_id & SENDER_ID) >> 24;
    msg_id_struct->rcv_id = (msg_id & RECEIVER_ID) >> 16;
    msg_id_struct->type = (msg_id & MESSAGE_TYPE) >> 8;
    msg_id_struct->length = (msg_id & MESSAGE_LENGTH);
  }
}

void get_elka_msg_id_attr(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t *msg_type, uint8_t *length,
    uint32_t msg_id) {
  if (snd_id)
    *snd_id = (msg_id & SENDER_ID) >> 24;

  if (rcv_id)
    *rcv_id = (msg_id & RECEIVER_ID) >> 16;

  if (msg_type)
    *msg_type = (msg_id & MESSAGE_TYPE) >> 8;

  if (length)
    *length = (msg_id & MESSAGE_LENGTH);
}

// Check ELKA ack against known msg_id and msg_num
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    uint32_t &msg_id, uint16_t &msg_num) {
  uint8_t ack_snd_id, ack_rcv_id, ack_msg_type, ack_msg_len,
          self_snd_id, self_rcv_id, self_msg_type, self_msg_len;
  get_elka_msg_id_attr(&ack_snd_id,
                       &ack_rcv_id,
                       &ack_msg_type,
                       &ack_msg_len,
                       elka_msg_ack.msg_id);

  get_elka_msg_id_attr(&self_snd_id,
                       &self_rcv_id,
                       &self_msg_type,
                       &self_msg_len,
                       msg_id);

  // Check that message is for u
  if ( (ack_snd_id == self_rcv_id) &&
          (ack_rcv_id == self_snd_id) ) { // message for u
    // Check that message parameters match 
    if ( !( (ack_msg_type == self_msg_type) &&
            (ack_msg_len == self_msg_len) &&
            (elka_msg_ack.msg_num == msg_num) ) ) {
      PX4_ERR("Ack message specified incorrectly");
      return elka_msg_ack_s::ACK_FAILED;
    } else {
      return elka_msg_ack.result;
    }
  } else { // msg not for u
    return elka_msg_ack_s::ACK_NULL;
  }
}

// Check ELKA ack against known msg_id and msg_num
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    struct elka_msg_id_s &msg_id, uint16_t &msg_num) {
  uint8_t ack_snd_id, ack_rcv_id, ack_msg_type, ack_msg_len;

  get_elka_msg_id_attr(&ack_snd_id,
                       &ack_rcv_id,
                       &ack_msg_type,
                       &ack_msg_len,
                       elka_msg_ack.msg_id);
  
  // Check that message is for u
  if ( (ack_snd_id == msg_id.rcv_id) &&
          (ack_rcv_id == msg_id.snd_id) ) { // message for u
    // Check that message parameters match 
    if ( !( (ack_msg_type == msg_id.type) &&
            (ack_msg_len == msg_id.length) &&
            (elka_msg_ack.msg_num == msg_num) ) ) {
      PX4_ERR("Ack message specified incorrectly");
      return elka_msg_ack_s::ACK_FAILED;
    } else {
      return elka_msg_ack.result;
    }
  } else { // msg not for u
    return elka_msg_ack_s::ACK_NULL;
  }
}


void print_char_array(char *buf) {
  char to_print[MAX_MSG_LEN+1];
  memcpy(to_print,buf,*(buf+1)+2);
  to_print[*(buf+1)+3] = 0;
  PX4_INFO("char array: %s",buf);
}

void print_uint8_array(uint8_t *buf) {
  char to_print[4*MAX_MSG_LEN+1],
       a_char[5]; // 3 chars max for a uint8 number and empty space
  uint8_t len = *(buf+1);
  uint8_t i=0;

  memset(to_print,0,4*MAX_MSG_LEN+1);

  while (i++ < len+2) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  PX4_INFO("uint8 array: %s",to_print);
}


