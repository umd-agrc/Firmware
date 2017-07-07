#if defined(__PX4_QURT) || defined(__PX4_POSIX)

#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_time.h>

#endif

#include <stdlib.h>
#include <string>
#include <cstring>

#include "elka.h"

//FIXME copy random stuff from <drivers/bootloaders/include/random.h>
//      because of linking error
#define RAND_MAX_32 ((1U << 31) - 1)
static uint32_t rseed;

void util_srand(uint16_t seed) {
  rseed = seed;
}

uint16_t util_random(uint16_t min, uint16_t max) {
  uint16_t rand = ( rseed =
            (rseed * 214013 + 2531011) & RAND_MAX_32 ) >> 16;
  return rand % (max - min) + min;
}

const hrt_abstime msg_threshold = 500000;

//TODO seed in a way that doesn't depend on
// PX4 hrt_abstime implementation
void get_dev_id_t(dev_id_t *d) {
  util_srand(hrt_absolute_time());
  *d = util_random(1, ID_MAX);
}

void get_snd_params(uint8_t *snd_params,
    uint8_t port_num, uint8_t port_type, 
    uint8_t snd_side) {
  if (snd_params)
    *snd_params = (port_num << 5) | (port_type << 2) | (snd_side);
}

void get_snd_params(uint8_t *snd_params,
    struct snd_params_s &snd_params_struct) {
  if (snd_params)
    *snd_params = (snd_params_struct.port_num << 5) |
              (snd_params_struct.port_type << 2) |
              (snd_params_struct.proc_side);
}

void get_snd_params(uint8_t *snd_params, 
    msg_id_t msg_id) {
  if (snd_params)
    get_elka_msg_id_attr(
        NULL, NULL, snd_params, NULL, NULL,
        msg_id);
}

void get_snd_params_attr(
    uint8_t *port_num,
    uint8_t *port_type,
    uint8_t *proc_side,
    uint8_t params) {
  if (port_num)
    *port_num = (params & PORT_NUM) >> 5;
  if (port_type)
    *port_type = (params & PORT_TYPE) >> 2;
  if (proc_side)
    *proc_side = (params & PROC_SIDE);
}

void get_snd_params_attr(
    struct snd_params_s *params_struct,
    uint8_t params) {
  if (params_struct) {
    params_struct->port_num = (params & PORT_NUM) >> 5;
    params_struct->port_type = (params & PORT_TYPE) >> 2;
    params_struct->proc_side = (params & PROC_SIDE);
  }    
}

// Call this before sending elka_msg or elka_msg_ack to set msg_id field
void get_elka_msg_id(msg_id_t *msg_id,
    dev_id_t snd_id, dev_id_t rcv_id,
    uint8_t snd_params, uint8_t msg_type, uint8_t length) {
  if (msg_id) {
    *msg_id = ((uint64_t)snd_id << 48) |
              ((uint64_t)rcv_id << 32) |
              ((uint64_t)snd_params << 24) | 
              ((uint64_t)msg_type << 8) | (length);
  }
}

void get_elka_msg_id(msg_id_t *msg_id, struct elka_msg_id_s &msg_id_struct) {
  if (msg_id) {
    *msg_id = ((uint64_t)msg_id_struct.snd_id << 48) |
              ((uint64_t)msg_id_struct.rcv_id << 32) |
              ((uint64_t)msg_id_struct.snd_params << 24) |
              ((uint64_t)msg_id_struct.type << 8) |
              (msg_id_struct.length);
  }
}

void get_elka_msg_id_attr(
    struct elka_msg_id_s *msg_id_struct,
    msg_id_t msg_id) {
  if (msg_id_struct) {
    msg_id_struct->snd_id = (msg_id & SENDER_ID) >> 48;
    msg_id_struct->rcv_id = (msg_id & RECEIVER_ID) >> 32;
    msg_id_struct->snd_params = (msg_id & SENDER_PARAMS) >> 24;
    msg_id_struct->type = (msg_id & MESSAGE_TYPE) >> 8;
    msg_id_struct->length = (msg_id & MESSAGE_LENGTH);
  }
}

void get_elka_msg_id_attr(
    dev_id_t *snd_id, dev_id_t *rcv_id,
    uint8_t *snd_params, uint8_t *msg_type, uint8_t *length,
    msg_id_t msg_id) {
  if (snd_id)
    *snd_id = (msg_id & SENDER_ID) >> 48;

  if (rcv_id)
    *rcv_id = (msg_id & RECEIVER_ID) >> 32;

  if (snd_params)
    *snd_params = (msg_id & SENDER_PARAMS) >> 24;

  if (msg_type)
    *msg_type = (msg_id & MESSAGE_TYPE) >> 8;

  if (length)
    *length = (msg_id & MESSAGE_LENGTH);
}

//FIXME separate 
// Check ELKA ack against known msg_id and msg_num
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    msg_id_t &msg_id, uint16_t &msg_num, uint8_t num_retries) {
  dev_id_t ack_snd_id, ack_rcv_id,
           self_snd_id, self_rcv_id;
  uint8_t ack_snd_params,
          ack_msg_type, ack_msg_len,
          self_snd_params,
          self_msg_type, self_msg_len;

  get_elka_msg_id_attr(&ack_snd_id,
                       &ack_rcv_id,
                       &ack_snd_params,
                       &ack_msg_type,
                       &ack_msg_len,
                       elka_msg_ack.msg_id);

  get_elka_msg_id_attr(&self_snd_id,
                       &self_rcv_id,
                       &self_snd_params,
                       &self_msg_type,
                       &self_msg_len,
                       msg_id);

  // Check that message is for u
  if ( (ack_snd_id == self_rcv_id) &&
          (ack_rcv_id == self_snd_id) ) { // message for u
    // Check that msg num matches and num retries of msg received are
    // less than or equal
    if ( !(elka_msg_ack.msg_num == msg_num &&
           elka_msg_ack.num_retries <= num_retries) ) {
      PX4_INFO("elka ack msg num: %d\nmsg num: %d",
          elka_msg_ack.msg_num, msg_num);
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
    struct elka_msg_id_s &msg_id, uint16_t &msg_num,
    uint8_t num_retries) {
  msg_id_t elka_msg_id;
  
  get_elka_msg_id(&elka_msg_id, msg_id);
  return check_elka_ack(elka_msg_ack, elka_msg_id,
                        msg_num, num_retries);
}

void serialize_elka_msg(uint8_t *ret, elka_msg_s &elka_msg) {
  uint8_t len;
  
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,&len,
      elka_msg.msg_id);
  *ret = elka_msg.msg_id & 0xff;
  *(ret+1) = (uint8_t)elka_msg.msg_id & 0xff00 >> 8;
  *(ret+2) = (uint8_t)elka_msg.msg_id & 0xff0000 >> 16;
  *(ret+3) = (uint8_t)elka_msg.msg_id & 0xff000000 >> 24;
  *(ret+4) = (uint8_t)elka_msg.msg_id & 0xff00000000 >> 32;
  *(ret+5) = (uint8_t)elka_msg.msg_id & 0xff0000000000 >> 40;
  *(ret+6) = (uint8_t)elka_msg.msg_id & 0xff000000000000 >> 48;
  *(ret+7) = (uint8_t)elka_msg.msg_id & 0xff00000000000000 >> 56;
  *(ret+8) = elka_msg.msg_num & 0xff;
  *(ret+9) = elka_msg.msg_num & 0xff00 >> 8;
  *(ret+10) = elka_msg.num_retries;
  memcpy((ret+11), elka_msg.data, len);
}

void serialize_elka_msg_ack(uint8_t *ret, elka_msg_ack_s &elka_msg) {
  *ret = elka_msg.msg_id & 0xff;
  *(ret+1) = (uint8_t)elka_msg.msg_id & 0xff00 >> 8;
  *(ret+2) = (uint8_t)elka_msg.msg_id & 0xff0000 >> 16;
  *(ret+3) = (uint8_t)elka_msg.msg_id & 0xff000000 >> 24;
  *(ret+4) = (uint8_t)elka_msg.msg_id & 0xff00000000 >> 32;
  *(ret+5) = (uint8_t)elka_msg.msg_id & 0xff0000000000 >> 40;
  *(ret+6) = (uint8_t)elka_msg.msg_id & 0xff000000000000 >> 48;
  *(ret+7) = (uint8_t)elka_msg.msg_id & 0xff00000000000000 >> 56;
  *(ret+8) = elka_msg.msg_num & 0xff;
  *(ret+9) = elka_msg.msg_num & 0xff00 >> 8;
  *(ret+10) = elka_msg.num_retries;
  *(ret+11) = elka_msg.result;
}

void print_elka_msg_id(msg_id_t &msg_id) {
  dev_id_t snd_id, rcv_id;
  uint8_t snd_params, msg_type, msg_len;

  get_elka_msg_id_attr(
      &snd_id, &rcv_id,
      &snd_params, &msg_type, &msg_len,
      msg_id);

  PX4_INFO("\tid: %" PRMIT "\n\tsnd_id: %" PRDIT "\t\
rcv_id: %" PRDIT "\n\tsnd_params: %d\tmsg_type: %d\tmsg_len: %d\n",
    msg_id,snd_id,rcv_id,snd_params,msg_type,msg_len);
}

void print_elka_msg(elka_msg_s &elka_msg) {
  PX4_INFO("-----ELKA msg-----");
  print_elka_msg_id(elka_msg.msg_id);
  PX4_INFO("\n");
}

void print_elka_msg_ack(elka_msg_ack_s &elka_msg) {
  PX4_INFO("-----ELKA msg ack-----");
  print_elka_msg_id(elka_msg.msg_id);
  PX4_INFO("\n");
}

void print_array(uint8_t *buf, uint8_t len) {
  char to_print[4*MAX_MSG_LEN+1],
       a_char[5]; // 3 chars max for a uint8 number and empty space
  uint8_t i=0;

  memset(to_print,0,4*MAX_MSG_LEN+1);

  while (i++ < len) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  PX4_INFO("array: %s",to_print);
}

void print_elka_serial_array(uint8_t *buf) {
  uint8_t len = *(buf+1);
  PX4_INFO("serial array");
  print_array(buf,len);
}

void print_uint8_array(uint8_t *buf, uint8_t len) {
  PX4_INFO("uint8 array");
  print_array(buf,len);
}

void print_char_array(char *buf, uint8_t len) {
  PX4_INFO("char array");
  print_array((uint8_t *)buf,len);
}

void print_cb(uint16_t *cb, uint16_t cb_end, uint16_t cb_len,
    uint16_t cb_max_size) {
  uint16_t nxt_idx;
  PX4_INFO("\nCircular buffer:\nLength: %d\tEnd: %d\n",
      cb_len, cb_end);
  for (uint16_t i=0; i < cb_max_size; i++) {
    PX4_INFO("%d ", *(cb+i));
  }
  PX4_INFO("\n");

  for (uint16_t i=0; i < cb_len; i++) {
    nxt_idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, i);
    PX4_INFO("%d ",
      *(cb + nxt_idx));
  }
  PX4_INFO("\n");
}

int cb_bin_search(uint16_t el, uint16_t *cb,
    uint16_t cb_end, uint16_t cb_len, uint16_t cb_max_size) {
  if (cb_len == 0) return -1;

  uint16_t mdpt_offset = cb_len % 2 ? cb_len/2 + 1 : cb_len/2;
  uint16_t search_idx = (cb_end >= mdpt_offset) ?
                        cb_end - mdpt_offset :
                        cb_max_size - (mdpt_offset - cb_end);
  uint16_t cb_el = *(cb+search_idx);

  if (el == cb_el)
    return search_idx;
  else {
    if (el > cb_el)
      return cb_bin_search(el, cb, cb_end, cb_len/2, cb_max_size);
    else
      return cb_bin_search(el, cb, search_idx, cb_len/2, cb_max_size);
  }
}

void cb_push(uint16_t el, uint16_t *cb,
    uint16_t &cb_end, uint16_t &cb_len, uint16_t cb_max_size) {
  *(cb + cb_end) = el;
  cb_end = (cb_end + 1) % cb_max_size;
  if (cb_len < cb_max_size) cb_len++;
}

void cb_insertion_sort(uint16_t *cb, uint16_t cb_end,
    uint16_t cb_len, uint16_t cb_max_size) {
  int j;
  uint16_t el, nxt_idx, idx;
  for (int i=1; i < cb_len; i++) {
    nxt_idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, i);
    el = *(cb+nxt_idx);
    j = i-1;
    nxt_idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, j);
    idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, j + 1);
    while (j >= 0 && *(cb+nxt_idx) > el) {
      *(cb+idx) = *(cb+nxt_idx);
      j -= 1;
      idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, j + 1);
      nxt_idx = get_nxt_idx(cb, cb_end, cb_len, cb_max_size, j);
    }
    *(cb+idx) = el;
  }
}

uint16_t get_nxt_idx(uint16_t *cb, uint16_t cb_end,
    uint16_t cb_len, uint16_t cb_max_size, uint16_t i) {
  return (cb_end - cb_len + i) < 0 ?
    cb_max_size - (cb_len - cb_end) + i : cb_end - cb_len + i;
}
