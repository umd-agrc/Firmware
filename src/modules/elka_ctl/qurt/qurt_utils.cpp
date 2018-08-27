#include "qurt_utils.h"

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

