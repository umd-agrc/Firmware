#pragma once

#include <uORB/topics/elka_msg.h>

#include "nn_defines.h"

void write_elka_msg_header(
    elka_msg_s *snd,
    uint16_t data_len,
    uint8_t msg_type);

int8_t serialize(uint8_t *dst, void *src, uint8_t len);
int8_t deserialize(void *dst, uint8_t *src, uint8_t len);

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

inline bool file_exists(const char *s) {
  FILE *f;
  if ((f=fopen(s,"r"))!=NULL) {
    fclose(f);
    return true;
  } else return false;
}

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


