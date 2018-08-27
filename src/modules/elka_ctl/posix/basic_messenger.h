#pragma once

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_packet.h>

#include "serial_defines.h"

namespace elka {
  class BasicMessenger;
}

/* Serialize data into an array of max size MAX_ELKA_MSG_LEN
 * @param dst = destination
 * @param src = source
 * @return SUCCESS if successful else SERIAL_ERROR
 */
int8_t serialize(uint8_t *dst, void *src, uint8_t len);
int8_t deserialize(void *dst, uint8_t *src, uint8_t len);
inline bool file_exists(const char *s) {
  FILE *f;
  if ((f=fopen(s,"r"))!=NULL) {
    fclose(f);
    return true;
  } else return false;
}

// Trim to no leading/ending whitespace|unprintable chars
//TODO can still have unprintable chars in middle of string
inline char *trim_path(char *str) {
  char *end;
  // Trim leading space
  while(!isprint((unsigned char)*str)||
				isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str&&(!isprint((unsigned char)*end)||
										isspace((unsigned char)*end))) end--;

  // Write new null terminator
  *(end+1) = 0;
  return str;
}

int8_t check_msg_header(uint8_t *m);
void pack_msg_header(uint8_t *msg,uint8_t msg_type,uint8_t len);
void write_msg_header(
    elka_packet_s *snd,
    uint8_t idx,
    uint16_t data_len,
    uint8_t msg_type);
void write_elka_msg_header(
    elka_msg_s *snd,
    uint16_t data_len,
    uint8_t msg_type);
int append_pkt(elka_packet_s *snd,
               uint8_t msg_type,
               uint16_t data_len,
               void *data);

void print_array(const uint8_t *buf, uint8_t len);
void print_char_array(const char *buf, uint8_t len);
void print_elka_msg(elka_msg_s msg);

// BasicMessenger is the base type of a local messenger object
// BasicMessengers connect with remote messengers
// (TBD: maybe also other local messengers)
class elka::BasicMessenger {
private:
public:
  char _des[MAX_ARR_LEN];
  uint8_t _tx_buf[MAX_ELKA_PACKET_LEN],_rx_buf[MAX_ELKA_PACKET_LEN];
  bool _data_rdy;
  /* Probe for remote messengers
   * @param available = available remote messengers
   * @return ELKA_SUCCESS or error code
   */
  virtual int8_t probe(void *available){return MSG_ERROR;}
  /* Open messenger
   * @param des = descriptor of messenger to open
   * @return success or error code
   */
  virtual int8_t open(){return MSG_ERROR;}
  /* Close messenger
   * @return success or error code
   */
  virtual int8_t close(){return MSG_ERROR;}
  /* Send message to a destination
   * @param msg = message to send
   * @return SUCCESS if successful else error code
   */
  virtual int8_t send(elka_packet_s *pkt){return MSG_ERROR;}

  /* Receive message (read call or equivalent)
   * Not useful for all messengers, as some may have receive
   * callbacks
   * @return success or error code
   */
  virtual int8_t recv(){return MSG_ERROR;}
};
