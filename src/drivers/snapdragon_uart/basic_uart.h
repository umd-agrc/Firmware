#pragma once

void port_read_callback(void *context, char *buffer, size_t num_bytes);

int set_interface_attribs(int fd, int baud, int parity);

void set_blocking(int fd, int should_block);

// @return serial_fd for desired port_num
int serial_open(int port_num);

int serial_close(int fd);

int assign_serial_read_callback(int fd, int port_num);

int serial_read(int fd, int port_num, char *rx_buffer);

int serial_write(int fd, int port_num, char *tx_buffer);

int serial_read_write(int fd, int port_num,
    char *rx_buffer, char *tx_buffer);
