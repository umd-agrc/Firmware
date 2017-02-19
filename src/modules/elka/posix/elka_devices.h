#ifndef ELKA_DEVICES_H
#define ELKA_DEVICES_H

#include "elka.h"

namespace elka {
class DeviceNode;
}

class elka::DeviceNode {
public:
  DeviceNode(int port_num, char *dev_name);
  ~DeviceNode();

  int init();
private:
  // Data members
  int _port_num;
  char _dev_name[MAX_NAME_LEN];

  // Class methods
  int thread_loop();
  int deinit();
};
#endif
