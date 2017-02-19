#ifndef ELKA_MANAGER_H
#define ELKA_MANAGER_H

#include "elka.h"
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
  elka::DeviceNode *get_elka_dev(int port_num, char *dev_name);
  
private:
  // Data members
  static Manager *_instance;
  
  DeviceNode *_elka_devs[MAX_ELKA_DEVS]; // Allow at most MAX_ELKA_DEVS devices

  // Class methods
  Manager();
  ~Manager();
};

#endif
