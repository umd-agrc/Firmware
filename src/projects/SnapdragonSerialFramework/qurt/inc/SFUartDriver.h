#pragma once

//#include <dev_fs_lib_serial.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <px4_app.h>
#include "uORB/topics/esc_status.h"
#include "uORB/topics/vehicle_command.h"

#define DEFAULT_ELKA_SERIAL_PATH "/dev/tty-2" // J13 module

#define SERIAL_DATA_BUFFER_LENGTH  128
#define SERIAL_WRITE_DELAY_IN_USECS (8000*10)

#define MAX_UART_DEVICE_NUM 6
#define NUM_UART_DEVICE_ENABLED 4

class SFUartDriver {

public:
  // Subscribe to RPCChannel using muorb/adsp/uORBFastRpcChannel

  SFUartDriver(char* port);
  SFUartDriver(void);
  ~SFUartDriver();

  int main();
  
  // Start access to port defined by dev_port;
  uint32_t start(void);

  // Stop access to port defined by dev_port;
  uint32_t stop(void);
 
  // Test access to port defined by dev_port;
  uint32_t test_port(void);

  // buf <- buffer to read to 
  // len <- length of buffer to read to 
  uint32_t uart_read(char* buf, int len);

  // msg <- message to send
  // len <- length of msg to write 
  uint32_t uart_write(char* msg, int len);

  // msg <- message to send
  // msg_len <- length of message to write
  // buf <- buffer to read to 
  // buf_len <- length of buffer to read to 
  uint32_t uart_write_read(
      char* msg, int msg_len, char* buf, int buf_len);

  static px4::AppState appState;
private:
  // Methods
  uint32_t health();

  // Members
  char dev_port[256];
  //TODO Work queue
  
  //struct vehicle_command_s m_vc;
}; // End SFUartDriver class
