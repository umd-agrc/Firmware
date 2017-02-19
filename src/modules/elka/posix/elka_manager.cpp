#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include "elka_manager.h"

//============== Static initialization ================
elka::Manager *elka::Manager::_instance = nullptr;

//-----------------------------------------------------
bool elka::Manager::initialize() {
  if (_instance == nullptr) {
    _instance = new elka::Manager();
  }

  return _instance != nullptr;
}

//-----------------------------------------------------
elka::DeviceNode *elka::Manager::get_elka_dev(
    int port_num, char *dev_name) {
  if (!_elka_devs[port_num]) {
    _elka_devs[port_num] = new DeviceNode(port_num, dev_name);

    if (_elka_devs[port_num] ) {
      int ret = _elka_devs[port_num]->init();

      if (ret != PX4_OK) {
        PX4_ERR("Initialization of DeviceNode failed (%i)", ret);
        errno = -ret;
        delete _elka_devs[port_num];
        _elka_devs[port_num] = nullptr;
      }
    } else {
      PX4_ERR("Failed to allocate DeviceNode");
      errno = ENOMEM;
    }
  }

  return _elka_devs[port_num];
}

bool elka::Manager::print_statistics(bool reset) {
  return false;
}

//-----------------------------------------------------
elka::Manager::Manager() {
  for (int i=0; i < MAX_ELKA_DEVS; i++) {
    _elka_devs[i] = nullptr;
  }
}

//-----------------------------------------------------
elka::Manager::~Manager() {
  for (int i=0; i < MAX_ELKA_DEVS; i++) {
    if (_elka_devs[i]) {
      delete _elka_devs[i];
    }
  }
}

