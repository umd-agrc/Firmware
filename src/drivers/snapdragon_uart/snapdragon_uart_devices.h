#ifndef SNAPDRAGON_UART_DEVICES_H
#define SNAPDRAGON_UART_DEVICES_H

#include "snapdragon_uart.h"

extern "C" {
#include "basic_uart.h"
//TODO add ringbufs after linking correctly 
//#include "c-ringbuf/ringbuf.h"
}

namespace uart {
class DeviceNode;
}

class uart::DeviceNode {
public:
  DeviceNode(int port_num, char *dev_name);
  ~DeviceNode();

  // Start uart thread
  int init();

  // Serial methods ------------------------------
  // @return serial_fd for desired port_num
  int open();

  int close();

  int assign_read_callback();

  // TODO
  // Read from ringbuf
  int read(char *buf);

  // TODO
  // Write to ringbuf
  int write(char *buf);

  // TODO
  // Read and write to ringbufs
  int read_write(char *rx_buf, char *tx_buf);

  // TODO get rid of these. these are for debugging
  char _tx_buffer[MAX_MSG_LEN];
  char _rx_buffer[MAX_MSG_LEN];

private:
  // Data members
  int _port_num;
  int _serial_fd;
  char _dev_name[MAX_NAME_LEN];
  //TODO tx/rx ringbufs here
  // ringbuf_t _tx_buf;
  // ringbuf_t _rx_buf;

  int deinit();
};
#endif
