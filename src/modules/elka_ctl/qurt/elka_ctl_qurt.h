#include <inttypes.h>
#include <stdint.h>
#include <dev_fs_lib_serial.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <px4_log.h>
#include <px4_defines.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <uORB/topics/elka_packet.h>
#include <elka_ctl/posix/serial_defines.h>

#include "qurt_utils.h"

void usage();
int elka_ctl_loop(int argc, char **argv);

int elka_uart_set_interface_attribs(int fd, int baud, int parity);
void elka_uart_set_blocking(int fd, int should_block);
void elka_uart_set_params();
int8_t elka_uart_open();
int8_t elka_uart_close();
int8_t elka_uart_send(elka_packet_s *pkt);
//int8_t elka_uart_recv();
