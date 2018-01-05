#ifndef SNAPDRAGON_UART_MANAGER_H
#define SNAPDRAGON_UART_MANAGER_H

#include "snapdragon_uart.h"
#include "snapdragon_uart_devices.h"

namespace uart {
class Manager;
}

class uart::Manager {
public:
  /**
   * Initialize the singleton. Call this before everything else.
   * @return true on success
   */
  static bool initialize(); 

  /**
   * Method to get the singleton instance for the uart::Manager
   * Make sure initialize() is called first.
   * @return uorb::Manager*
   */
  static uart::Manager *get_instance() {
    return _instance;
  }

  /**
   * Print statistics
   * @param reset, if true reset statistics afterwards
   * @return true if something printed, false otherwise
   */
  bool print_statistics(bool reset);

  /**
   * Get a uart device to be used to communicate with an external
   * device.
   * @param port_num, tty port 1-6 available
   * @param dev_name, name to call device
   * @return DeviceNode with corresponding parameters on success,
   *         nullptr on failure
   */
  uart::UARTPort *get_uart_dev(uint8_t port_num, uint8_t buf_t,
      uint8_t size, char *dev_name);

  /**
   * Check whether a uart dev exists at the desired port
   * @param port_num, tty port 1-6 to check
   * @return true if uart dev exists, else false
   */
  bool check_uart_dev(uint8_t port_num);

  /**
   * Stop a uart device
   * @param port_num, port to stop
   * @return true if stop is successful, else false
   */
  void stop_uart_dev(uint8_t port_num);

private:
  // Data members
  static Manager *_instance;
  
  UARTPort *_uart_devs[MAX_UART_PORTS]; // Allow at most MAX_UART_PORTS devices

  // Class methods
  Manager();
  ~Manager();
};

#endif
