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
elka::PX4Port *elka::Manager::get_elka_dev(
    uint8_t dev_num, uint8_t port_num,
    uint8_t port_type, uint8_t buf_type, uint8_t size, char *dev_name) {
  if (!_elka_devs[dev_num]) {
    _elka_devs[dev_num] = new PX4Port(port_num, port_type,
        buf_type, size, dev_name);
  } else {
    PX4_ERR("Failed to allocate DeviceNode");
    errno = ENOMEM;
  }
  
  return _elka_devs[dev_num];
}

bool elka::Manager::check_elka_dev(int dev_num) {
  if (_elka_devs[dev_num]) return true;
  else return false;
}

int elka::Manager::stop_elka_dev(int dev_num) {
  delete _elka_devs[dev_num];
  return PX4_ERROR;
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

