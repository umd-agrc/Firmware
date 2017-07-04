#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>

#include "snapdragon_uart_manager.h"

//============== Static initialization ================
uart::Manager *uart::Manager::_instance = nullptr;

//-----------------Public Methods----------------------
//-----------------------------------------------------
bool uart::Manager::initialize() {
  if (_instance == nullptr) {
    _instance = new uart::Manager();
  }

  return _instance != nullptr;
}

//-----------------------------------------------------
uart::UARTPort *uart::Manager::get_uart_dev(
    uint8_t port_num, uint8_t buf_t, uint8_t size,
    char *dev_name) {
  if (!_uart_devs[port_num]) {
    _uart_devs[port_num] = new UARTPort(port_num, buf_t,
                                          size, dev_name);

    if (_uart_devs[port_num] ) {
      int ret = _uart_devs[port_num]->init();

      if (ret != PX4_OK) {
        PX4_ERR("Initialization of DeviceNode failed (%i)", ret);
        errno = -ret;
        delete _uart_devs[port_num];
        _uart_devs[port_num] = nullptr;
      }
    } else {
      PX4_ERR("Failed to allocate DeviceNode");
      errno = ENOMEM;
    }
  }


  return _uart_devs[port_num];
}

bool uart::Manager::check_uart_dev(uint8_t port_num) {
  if (_uart_devs[port_num]) return true;
  else return false;
}

void uart::Manager::stop_uart_dev(uint8_t port_num) {
  delete _uart_devs[port_num];
  _uart_devs[port_num] = nullptr;
}

bool uart::Manager::print_statistics(bool reset) {
  return false;
}

//-----------------Private Methods---------------------
//-----------------------------------------------------
uart::Manager::Manager() {
  for (uint8_t i=0; i < MAX_UART_PORTS; i++) {
    _uart_devs[i] = nullptr;
  }
}

//-----------------------------------------------------
uart::Manager::~Manager() {
  for (uint8_t i=0; i < MAX_UART_PORTS; i++) {
    if (_uart_devs[i]) {
      delete _uart_devs[i];
    }
  }
}

