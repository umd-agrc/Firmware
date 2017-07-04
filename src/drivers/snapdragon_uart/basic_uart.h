#pragma once

#include <elka/common/elka.h>
#include <elka/common/elka_comm.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include "snapdragon_uart.h"

//typedef struct buf buffer;

void elka_read_callback(void *context,
    char *buffer, size_t num_bytes);

//TODO elka_write_callback();

//void buffer_print_next_msg(buffer *buf);

//int memcpy_into(buffer *dst, char *src, size_t num_bytes);

int set_interface_attribs(int fd, int baud, int parity);

void set_blocking(int fd, int should_block);

// @return serial_fd for desired port_num
int serial_open(int port_num, elka::SerialBuffer *tx_sb,
    elka::SerialBuffer *rx_sb);
//int serial_open(int port_num, void *tx_buf, uint8_t tx_buf_type,
//    void *rx_buf, uint8_t rx_buf_type);

int serial_close(int fd, int port_num);

int assign_serial_read_callback(int fd, int port_num);

// FIXME Deprecated?
// Performs serial read directly (doesn't write to internal buffer)
int serial_read(int fd, int port_num, uint8_t *rx_buffer);

int serial_write(int fd, int port_num,
    uint8_t *tx_buffer, uint8_t tx_buffer_len);

int serial_read_write(int fd, int port_num,
    uint8_t *rx_buffer, uint8_t *tx_buffer, uint8_t tx_buffer_len);

//FIXME change name to uart_write_elka_msg or serial_write_elka_msg
int write_elka_msg(int fd, elka_msg_s &elka_msg);

int write_elka_msg(int fd, elka_msg_ack_s &elka_msg);
