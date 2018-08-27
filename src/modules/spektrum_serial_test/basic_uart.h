#pragma once

#include <stdint.h>
#include <map>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_packet.h>

#include "serial_defines.h"
#include "basic_messenger.h"

#define MAX_NUM_SERIAL_DEVS 6
#define SERIAL_SIZE_OF_DATA_BUFFER 128

int spektrum_set_interface_attribs(int fd, int baud, int parity);

void spektrum_set_blocking(int fd, int should_block);

namespace elka {
  class SnapdragonSerialMessenger;
}

// Snapdragon's serial messenger.
// Possible to send multiple messages in one packet.
class elka::SnapdragonSerialMessenger : public BasicMessenger {
private:
  int _fd;
  static void read_cb_helper(void *context,
      char *buffer,size_t num_bytes);
  void read_callback(char *buffer, size_t num_bytes);
public:
  SnapdragonSerialMessenger(char *des);
  int8_t probe(void *available) override;
  int8_t open() override;
  int8_t close() override;
  int8_t send(elka_packet_s *pkt) override;
  int8_t recv() override;
};
