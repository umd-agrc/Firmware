#include <px4_defines.h>
#include <px4_log.h>

#include "elka_comm.h"

//-----------------SerialBuffer Methods---------------------

elka::SerialBuffer::SerialBuffer(uint8_t buf_type) {
  _type = buf_type;

  _msg_num=0;

  if (buf_type == CHAR_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else if (buf_type == UINT8_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else {
    PX4_ERR("Unsupported buffer type");
    errno = EINVAL;
  }
}

elka::SerialBuffer::~SerialBuffer() {
  if (_type == CHAR_ARRAY) {
    free(_buffer);
  } else if (_type == UINT8_ARRAY) {
    free(_buffer);
  } else {
    PX4_ERR("Unsupported buffer type for ELKA serial port");
    errno = EINVAL;
  }
}

//-----------------SerialBuffer Methods---------------------

//-----------------CommPort Methods---------------------

elka::CommPort::CommPort(uint8_t port_n, uint8_t port_t,
    uint8_t buf_t) {
  _port_num = port_n;

  _tx_buf = new SerialBuffer(buf_t);
  _rx_buf = new SerialBuffer(buf_t);

  _state = STATE_STOP;
}

elka::CommPort::~CommPort() {
}
//-----------------CommPort Methods---------------------
