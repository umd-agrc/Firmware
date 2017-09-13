#pragma once

#include <stdint.h>

#define MAX_NUM_SERIAL_DEVS 6
#define MAX_SERIAL_MSG_LEN 256
#define SERIAL_SIZE_OF_DATA_BUFFER 128

// Get byte of 64b integer
// High byte is higher number
inline char get_byte_n(uint64_t num, uint8_t n) {
  return (char)((num >> ((n-1)*8)) & 0xff);
}
inline char get_byte_n(int64_t num, uint8_t n) {
  return (char)((num >> ((n-1)*8)) & 0xff);
}

void spektrum_port_read_callback(void *context, char *buffer, size_t num_bytes);

int spektrum_set_interface_attribs(int fd, int baud, int parity);

void spektrum_set_blocking(int fd, int should_block);

void spektrum_print_array(const uint8_t *buf, uint8_t len);
void spektrum_print_char_array(const char *buf, uint8_t len);

int spektrum_serial_open(); 
int spektrum_serial_close(int fd); 
int spektrum_serial_read_callback(int fd); 
int spektrum_serial_read(int fd, char* rx_buffer); 
int spektrum_serial_write(int fd, const char* tx_buffer,
    uint8_t tx_buf_len); 
int spektrum_serial_read_write(int fd, char* rx_buffer,
    const char* tx_buffer, uint8_t tx_buf_len);
