#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <cstring>

#include "elka_devices.h"

//-----------------Public Methods----------------------
elka::DeviceNode::DeviceNode(int port_num, char *dev_name) {
  _port_num = port_num;
  strcpy(_dev_name, dev_name);
}

elka::DeviceNode::~DeviceNode() {
  if (!(deinit() == PX4_OK)) {
    PX4_ERR("Unable to deinitialize elka device with port_num %d",
        _port_num);
  };
}

//-----------------Private Methods---------------------
int elka::DeviceNode::init() {
  return PX4_OK;
}


int elka::DeviceNode::thread_loop() {
  return PX4_OK;
}

int elka::DeviceNode::deinit() {
  return PX4_OK;
}
