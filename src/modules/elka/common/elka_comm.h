#ifndef ELKA_COMM_H
#define ELKA_COMM_H

#include "elka.h"

//FIXME debug. only useful for expecting_ack, which will change
#include <map>

namespace elka {
  struct SerialBuffer;
  struct CommPort;
}

struct elka::SerialBuffer {
  void *_buffer;
  uint16_t _msg_num;
  uint8_t _type;
  SerialBuffer(uint8_t buf_type);
  ~SerialBuffer();
};

// Encapsulates necessary aspects of communication to either PX4 ELKA or
// hardware ELKA
struct elka::CommPort {
  struct elka::SerialBuffer *_tx_buf;
  struct elka::SerialBuffer *_rx_buf;
  // map of <msg_num,num_retries> key-value pairs
  // for msgs waiting to receive ack
  std::map<uint16_t,uint8_t> _expecting_ack;
  uint8_t _port_num;
  uint8_t _rcv_id;
  uint8_t _snd_id;
  uint8_t _state;
  CommPort(uint8_t port_num, uint8_t buf_type, uint8_t proc_side);
  // virtual destructor so that derived class destructor is called.
  virtual ~CommPort(); 
  virtual bool start_port() = 0;
  virtual bool stop_port() = 0;
  virtual bool pause_port() = 0;
  virtual bool resume_port() = 0;
};

#endif
