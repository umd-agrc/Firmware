#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <string>
#include <cstring>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>

#include "snapdragon_uart.h"
#include "snapdragon_uart_manager.h"

extern "C" { __EXPORT int snapdragon_uart_main(int argc, char *argv[]); }

static int daemon_task[MAX_UART_PORTS];
static volatile bool thread_should_exit[MAX_UART_PORTS];
static volatile bool thread_running[MAX_UART_PORTS];

int snapdragon_uart_dev_loop(int argc, char **argv);

static uart::Manager *mgr = nullptr;

int snapdragon_uart_main(int argc, char *argv[]) {

	if (argc < 2) {
		PX4_WARN("Missing action <start <port_num>|stop <port_num>\
status>");
		return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {

    if (mgr == nullptr) {
      if (!uart::Manager::initialize()) {
        PX4_ERR("snapdragon_uart manager alloc failed");
        return -ENOMEM;
      }

      // Create the manager
      mgr = uart::Manager::get_instance();

      if (mgr == nullptr) {
        return -errno;
      }
    }

    int port_num;
    char dev_name[42];
    if (argc < 3) {
      PX4_WARN("Missing port number specification");
      return PX4_OK;
    }
    
    int i=2;
    while (argv[i]) {
      if (!(port_num = std::atoi(argv[i])) ||
          port_num < 1 || port_num > MAX_UART_PORTS) {
        PX4_WARN("Invalid port number specification %s",
            argv[i]);
        continue;
      } else if (i+2 >= MAX_UART_PORTS) {
        PX4_WARN("Too many ports specified");

      } else if (mgr !=nullptr
        && !mgr->check_uart_dev(port_num)) {
        thread_should_exit[port_num] = false;
        sprintf(dev_name,"/dev/tty-%d",port_num);
        daemon_task[port_num] = px4_task_spawn_cmd(
          dev_name,
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT,
          1000,
          snapdragon_uart_dev_loop,
          (char * const *)&argv[i]);
 
        unsigned constexpr max_wait_us = 1000000;
        unsigned constexpr max_wait_steps = 2000;
        unsigned j;

        for (j=0; j < max_wait_steps; j++) {
          usleep(max_wait_us / max_wait_steps);
          if (thread_running[port_num]) {
            break;
          }
        }
        return !(j < max_wait_steps);
      }

      i++;
    } 

	} else if (!strcmp(argv[1],"stop")) {
    if (argc < 3) {
      // stop all ports
      for (int i=0; i < MAX_UART_PORTS; i++) {
        if (!thread_running[i]) {
          PX4_WARN("uart dev %d already stopped",i);
          return PX4_OK;
        }

        thread_should_exit[i] = true;

        while(thread_running[i]) {
          usleep(200000);
          PX4_WARN(".");
        }

        PX4_WARN("terminated.");

        return PX4_OK;
      }
    } else {
      int i=2, port_num;
      while (argv[i]) {
        if (!(port_num = std::atoi(argv[i]))) {
          PX4_WARN("Invalid stop command, port %s not available",
            argv[i]);
          i++;
          continue;
        }

        if (!thread_running[i]) {
          PX4_WARN("uart dev %d already stopped",i);
          return PX4_OK;
        }

        thread_should_exit[i] = true;

        while(thread_running[i]) {
          usleep(200000);
          PX4_WARN(".");
        }

        PX4_WARN("terminated.");

        i++;
      }

      return PX4_OK;
    }

  } else if (!strcmp(argv[1],"status")) {
    if (mgr != nullptr) {
      mgr->print_statistics(true);
    } else {
      PX4_INFO("snapdragon_uart is not running");
    }

    return PX4_OK;

  } else {
		PX4_WARN("Action not supported");
	}

	return PX4_OK;
}

int snapdragon_uart_dev_loop(int argc, char **argv) {
  char dev_nm[42];
  int port_num = std::atoi(argv[0]);

  sprintf(dev_nm,"/dev/tty-%d",port_num);

  PX4_INFO("Starting port %s for uart", dev_nm);
  uart::DeviceNode *snapdragon = mgr->get_uart_dev(
      port_num, dev_nm);

  snapdragon->assign_read_callback();

  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to sensor_combined topic
  int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
  // Limit update rate to 5Hz
  orb_set_interval(sensor_sub_fd, 200);
  // Define sensor_combined_s struct to update
  struct sensor_combined_s raw;

  // Advertise attitude topic
  struct vehicle_attitude_s att;
  memset(&att, 0, sizeof(att));
  orb_advert_t att_pub = orb_advertise(
      ORB_ID(vehicle_attitude), &att);

  // Define topic subscribers to wait for with polling
  px4_pollfd_struct_t fds[] = {
    {.fd = sensor_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;

  //FIXME debugging uart
  char debug_tx_buf[1024] = "john henry";
  //char debug_rx_buf[1024];

  while (!thread_should_exit[port_num]) {

    //FIXME debugging uart
    snapdragon->write(debug_tx_buf);
    usleep(1000000);
    //snapdragon->read(debug_rx_buf);

    // Wait for sensor update of 1 file descriptor for 1000ms
    poll_ret = px4_poll(fds, 1, 1000);

    // Handle the poll result
    if (poll_ret == 0) {
      // None of our providers is giving us data
      PX4_ERR("Got no data within a second");
    } else if (poll_ret < 0) {
      // Should be an emergency
      if (error_counter < 10 || error_counter % 50 == 0) {
        // Use a counter to prevent flooding and slowing us down
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
      }

      error_counter++;
    } else {

      if (fds[0].revents & POLLIN) {
        // Obtained data for the first file descriptor
        orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
        // Copy sensors raw data into local buffer
        // TODO Here will copy into DeviceNode ringbuf
        PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
            (double)raw.accelerometer_m_s2[0],
            (double)raw.accelerometer_m_s2[1],
            (double)raw.accelerometer_m_s2[2]);
      }

      // Set att and publish this information for other apps
      att.rollspeed = raw.accelerometer_m_s2[0];
      att.pitchspeed = raw.accelerometer_m_s2[1];
      att.yawspeed = raw.accelerometer_m_s2[2];
      orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
    }
  }

  mgr->stop_uart_dev(port_num);

  PX4_INFO("Exiting uart device thread %d",port_num);

  return PX4_OK;
}
