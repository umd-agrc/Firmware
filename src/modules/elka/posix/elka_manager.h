#ifndef ELKA_MANAGER_H
#define ELKA_MANAGER_H

#include <elka/common/elka.h>

//#include "elka.h"
#include "elka_devices.h"

namespace elka {
class Manager;
}

class elka::Manager {
public:
  /**
   * Initialize the singleton. Call this before everything else.
   * @return true on success
   */
  static bool initialize(); 

  /**
   * Method to get the singleton instance for the elka::Manager
   * Make sure initialize() is called first.
   * @return uorb::Manager*
   */
  static elka::Manager *get_instance() {
    return _instance;
  }

  /**
   * Print statistics
   * @param reset, if true reset statistics afterwards
   * @return true if something printed, false otherwise
   */
  bool print_statistics(bool reset);

  /**
   * Get an elka device to be used to communicate with an external
   * device.
   * @param port_num, tty port 1-6 available
   * @param dev_name, name to call device
   * @return DeviceNode with corresponding parameters on success,
   *         nullptr on failure
   */
  elka::PX4Port *get_elka_dev(uint8_t dev_num, uint8_t port_num,
      uint8_t port_type, uint8_t buf_type, uint8_t size, char *dev_name);

  /**
   * Check if elka device exists
   * @param dev_num, device number 1-2 available
   * @return true if exists, else false
   */
  bool check_elka_dev(int dev_num);

  /**
   * Stop all running ports on elka device
   * @param dev_num, device number 1-2 available
   * @return 0 if exit successfully, else false
   */
  int stop_elka_dev(int dev_num);

private:
  // Data members
  static Manager *_instance;
  
  PX4Port *_elka_devs[MAX_ELKA_DEVS]; // Allow at most MAX_ELKA_DEVS devices

  // Class methods
  Manager();
  ~Manager();
};

#endif
