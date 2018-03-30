#pragma once
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <px4_log.h>
#include <elka_ctl/posix/serial_defines.h>

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

void print_array(const uint8_t *buf, uint8_t len);
void print_char_array(const char *buf, uint8_t len);

