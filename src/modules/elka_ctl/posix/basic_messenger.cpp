#include <px4_log.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "basic_messenger.h"

int8_t serialize(uint8_t *dst, void *src, uint8_t len) {
  if (len < MAX_ELKA_MSG_LEN) {
    memcpy(dst,src,len);
    return ELKA_SUCCESS;
  } else return SERIAL_ERROR;
}

int8_t deserialize(void *dst, uint8_t *src, uint8_t len) {
  if (len < MAX_ELKA_MSG_LEN) {
    memcpy(dst,src,len);
    return ELKA_SUCCESS;
  } else return SERIAL_ERROR;
}

void pack_msg_header(uint8_t *msg,uint8_t msg_type,uint8_t len){
  *msg=len;
  *(msg+1)=msg_type;
  *(msg+2)=ELKA_MSG_HEADER_ASSURANCE_BYTE;
  *(msg+3)=ELKA_MSG_HEADER_ASSURANCE_BYTE;
}

void write_msg_header(
    elka_packet_s *snd,
    uint8_t idx,
    uint16_t data_len,
    uint8_t msg_type) {
  // Pack initial byte for packet len
  if (snd->num_msgs==0) {
    snd->len++;
    snd->data[0]=0;
    idx++;
  }

  // Update packet len 
  snd->len+=ELKA_MSG_HEADER_LEN+1;
  snd->data[0]+=data_len+ELKA_MSG_HEADER_LEN+1;

  // Pack msg len
  snd->data[idx]=data_len+ELKA_MSG_HEADER_LEN;
  snd->data[idx+1]=msg_type;
  snd->data[idx+2]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
  snd->data[idx+3]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
}
// TODO FIX THIS IS SO MESSY. This is necessary for elka_nn
void write_elka_msg_header(
    elka_msg_s *snd,
    uint16_t data_len,
    uint8_t msg_type) {
  // Write msg len 
	snd->len = data_len+ELKA_MSG_HEADER_LEN+1;
  snd->data[ELKA_MSG_LEN]=data_len+ELKA_MSG_HEADER_LEN+1;

  // Pack msg len
  snd->data[ELKA_MSG_TYPE]=msg_type;
  snd->data[ELKA_MSG_TYPE+1]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
  snd->data[ELKA_MSG_TYPE+2]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
}

//TODO
int append_pkt(elka_packet_s *snd,
               uint8_t msg_type,
               uint16_t data_len,
               void *data) {
  if (snd->len+data_len+ELKA_MSG_HEADER_LEN+1
      >MAX_ELKA_MSG_LEN){
    return PKT_ERROR;
  }

  write_msg_header(snd,snd->len,data_len,msg_type);
  serialize(&(snd->data[snd->len]),data,data_len);
  snd->len+=data_len;
  // Update num msgs
  snd->num_msgs++;
  // Return new total packet length
  return snd->len;
}

int8_t check_msg_header(uint8_t *m){
  if (*(m+2)!=ELKA_MSG_HEADER_ASSURANCE_BYTE &&
      *(m+3)!=ELKA_MSG_HEADER_ASSURANCE_BYTE)
    return MSG_ERROR;
  else
    return ELKA_SUCCESS;
}

void print_array(const uint8_t *buf, uint8_t len) {
  char to_print[7*MAX_SERIAL_MSG_LEN+1],
       a_char[7]; // 3 chars max for a uint8 number and empty space
  uint8_t i=0;

  memset(to_print,0,7*MAX_SERIAL_MSG_LEN+1);

  while (i++ < len) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  PX4_INFO("array: %s",to_print);
}

void print_char_array(const char *buf, uint8_t len) {
  PX4_INFO("char array");
  print_array((const uint8_t *)buf,len);
}

void print_elka_msg(elka_msg_s msg) {
  PX4_INFO("elka msg{ type: %d len: %d }",
      msg.type,msg.len);
  print_array(msg.data,msg.len);
}
