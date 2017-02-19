#include <cstring>
#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>

#include "snapdragon_uart_devices.h"

//-----------------Public Methods----------------------
uart::DeviceNode::DeviceNode(int port_num, char *dev_name) {
  _port_num = port_num;
  strcpy(_dev_name, dev_name);
}

uart::DeviceNode::~DeviceNode() {
  if (deinit() != PX4_OK) {
    PX4_ERR("Unable to deinitialize elka device with port_num %d",
        _port_num);
  };
}

//-----------------Private Methods---------------------
// Set up thread loop to transfer messages in PX4 and serially
// with external devices
// @return PX4_OK on success, else PX4_ERROR
int uart::DeviceNode::init() {
  if (open() == PX4_OK) {
    PX4_INFO("Initialized Snapdragon UART device to communicate on port %s",
        _dev_name);
    return PX4_OK;
  } else {
    return PX4_ERROR;
  }
}

// Tear down thread loop
// @return PX4_OK on success, else PX4_ERROR
int uart::DeviceNode::deinit() {
  if (close() == PX4_OK)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::open() {
  if ((_serial_fd = serial_open(_port_num)) > 0 )
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::close() {
  if (!(_serial_fd = serial_close(_port_num)))
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::assign_read_callback() {
  if (assign_serial_read_callback(_serial_fd, _port_num) == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::read(char *buf) {
  if (serial_read(_serial_fd, _port_num, buf) == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::write(char *buf) {
  if (serial_write(_serial_fd, _port_num, buf) == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::read_write(char *rx_buf, char *tx_buf) {
  if (serial_read_write(_serial_fd, _port_num, rx_buf, tx_buf)
      == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}
