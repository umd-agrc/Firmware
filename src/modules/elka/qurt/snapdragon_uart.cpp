#include <cstring>
#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <string>
#include <unistd.h>
#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include <uORB/topics/input_rc.h>

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

    uint8_t port_num;
    char dev_name[MAX_NAME_LEN];
    if (argc < 3) {
      PX4_WARN("Missing port number specification");
      return PX4_OK;
    }
    
    uint8_t i=2;// Start at argv index
    while (argv[i]) {
      if (!(port_num = (uint8_t) std::atoi(argv[i])) ||
          port_num < 1 || port_num > MAX_UART_PORTS) {
        PX4_WARN("Invalid port number specification %s",
            argv[i]);
        continue;
      } else if (i-2 >= MAX_UART_PORTS) {
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
      for (uint8_t i=0; i < MAX_UART_PORTS; i++) {
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
      uint8_t i=2, port_num;
      while (argv[i]) {
        if (!(port_num = (uint8_t) std::atoi(argv[i]))) {
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
  char dev_nm[MAX_NAME_LEN];
  uint8_t port_num = (uint8_t)std::atoi(argv[0]),
          buf_type = PRIORITY_QUEUE;

  sprintf(dev_nm,"/dev/tty-%d",port_num);

  PX4_INFO("Starting port %s for uart", dev_nm); fflush(stdout);

  uint8_t queue_sz = 42;
  uart::UARTPort *snapdragon = mgr->get_uart_dev(
      port_num, buf_type, queue_sz, dev_nm);

  snapdragon->assign_read_callback();

  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to elka msg, elka msg ack, and input_rc (TODO only if necessary)
  int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
  int elka_ack_sub_fd = orb_subscribe(ORB_ID(elka_msg_ack));
  int elka_sub_fd = orb_subscribe(ORB_ID(elka_msg));

  // Set update rate to 100Hz
  orb_set_interval(input_rc_sub_fd, 10);
  orb_set_interval(elka_ack_sub_fd, 10);
  orb_set_interval(elka_sub_fd, 10);

  /*
  struct input_rc_s input_rc;
  struct elka_msg_ack_s elka_ack_snd, elka_ack_rcv;
  struct elka_msg_s elka_snd, elka_snd;
  memset(&input_rc, 0, sizeof(input_rc));
  memset(&elka_ack_snd, 0, sizeof(elka_ack_snd));
  memset(&elka_ack_rcv, 0, sizeof(elka_ack_rcv));
  memset(&elka_snd, 0, sizeof(elka_snd));
  memset(&elka_rcv, 0, sizeof(elka_rcv));
  
  // Advertise elka msg and elka msg ack
  orb_advert_t elka_ack_pub = orb_advertise(
      ORB_ID(elka_msg_ack), &elka_ack_snd);
  orb_advert_t elka_msg_pub = orb_advertise(
      ORB_ID(elka_msg), &elka_snd);
  //orb_advert_t elka_ack_pub = orb_advertise(
  //    ORB_ID(elka_msg_ack), NULL);
  //orb_advert_t elka_msg_pub = orb_advertise(
  //    ORB_ID(elka_msg), NULL);
  */

  // Define topic subscribers to wait for with polling
  px4_pollfd_struct_t fds[] = {
    {.fd = input_rc_sub_fd, .events = POLLIN},
    {.fd = elka_ack_sub_fd, .events = POLLIN},
    {.fd = elka_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;

  thread_running[port_num] = true;

  // nxt_msg_type contains next message for
  // send & receive
  uint8_t parse_res, ack_res, nxt_msg_type[2]; 
  // snd_id is sender id for received elka_msgs and elka_msg_acks
  dev_id_t snd_id;

  //FIXME debugging
  /*
  // Set PX4 elka module to paused
  snapdragon->set_dev_state_msg(snapdragon->_elka_snd,
                                STATE_START,
                                true);
                                */
  //snapdragon->push_msg(snapdragon->_elka_snd, true);

  while (!thread_should_exit[port_num]) {
    snapdragon->update_time();

    usleep(500000);

    // Wait for up to 500ms for data
    // FIXME this should be tied to msg threshold
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 500);

    if (poll_ret == 0) {
      // None of our providers is giving us data
      PX4_ERR("Got no data");
    } else if (poll_ret < 0) {
      // Should be an emergency
      if (error_counter < 10 || error_counter % 50 == 0) {
        // Use a counter to prevent flooding and slowing us down
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
      }

      error_counter++;
    } else {

      if (fds[0].revents & POLLIN) { // input_rc
        orb_copy(ORB_ID(input_rc), input_rc_sub_fd,
            &snapdragon->_input_rc);

        /*
        // TODO check paused before setting and sending for efficiency
        // Set PX4 elka module to paused
        snapdragon->set_dev_state_msg(elka_snd_px4, STATE_PAUSE,
            true, true);
        snapdragon->push_msg(elka_snd_px4, true, true);
        */
      }
      
      if (fds[1].revents & POLLIN) { // elka_msg_ack
        orb_copy(ORB_ID(elka_msg_ack), elka_ack_sub_fd,
            &snapdragon->_elka_ack_rcv);
        
        // TODO push to _rc_buf and check in a parse_elka_msg()
        // TODO Check message timestamp
        if ( (ack_res = snapdragon->check_ack(
                snapdragon->_elka_ack_rcv)
              == elka_msg_ack_s::ACK_FAILED) ) {
          PX4_ERR("Failed acknowledgement for msg id: " PRMIT "",
              snapdragon->_elka_ack_rcv.msg_id);
        } else if (ack_res != elka_msg_ack_s::ACK_NULL) {
        }
      }

      if (fds[2].revents & POLLIN) { // elka_msg
        // TODO Check message timestamp
        //if (elka_rcv.timestamp - snapdragon->_now < msg_threshold) {
          orb_copy(ORB_ID(elka_msg), elka_sub_fd,
                    &snapdragon->_elka_rcv);

          // Push msg to rx_buf 
          // Do not push if you are the sender to prevent cycles from
          // forming
          get_elka_msg_id_attr(&snd_id, NULL, NULL, NULL, NULL,
              snapdragon->_elka_rcv.msg_id);
          if (cmp_dev_id_t(snd_id, snapdragon->_id))
            snapdragon->push_msg(snapdragon->_elka_rcv, false);

          usleep(5000);
        //}
      }

      // remove and parse message from rx_buf
      // send ack if the message requires one 
      if ( (nxt_msg_type[1] = snapdragon->remove_msg(
              snapdragon->_elka_rcv,
              snapdragon->_elka_ack_rcv,
              false)) != MSG_NULL &&
          (parse_res = snapdragon->parse_elka_msg(
              snapdragon->_elka_rcv,
              snapdragon->_elka_ack_snd))
          == MSG_FAILED ) {
        PX4_ERR("Failed message for msg id: %" PRMIT "",
          snapdragon->_elka_rcv.msg_id);
      }

      if (parse_res & TYPE_EXPECTING_ACK) {
        snapdragon->push_msg(snapdragon->_elka_ack_snd, true);
      }
    
      if ( (nxt_msg_type[0] = snapdragon->get_msg(
                                snapdragon->_elka_snd,
                                snapdragon->_elka_ack_snd,
                                true))
           != MSG_NULL &&
           nxt_msg_type[0] != MSG_FAILED ) {

        //PX4_INFO("sending msg on port %d w msg_id %d",
        //            i, elka_snd.msg_id);
        //orb_publish(ORB_ID(elka_msg), elka_msg_pub, &elka_snd);
        snapdragon->send_msg(snapdragon->_elka_snd);

        if (!(snapdragon->_elka_snd.msg_id & ID_EXPECTING_ACK))
          // TODO This scheme of popping only works if messages are
          // pushed here and not in a separate thread
          snapdragon->pop_msg(true);
        else // Sleep for a short time to let ack process
          usleep(20000);
      }

      // Check if input_rc msgs are gone, and we can start using our messages
      // again
      /*
      if (input_rc.timestamp - snapdragon->_now > msg_threshold) {
        // TODO publish useful message
        // TODO send useful message thru UART
        if (false) orb_publish(ORB_ID(elka_msg), elka_msg_pub, &elka_snd_px4);
      } else {
        PX4_INFO("paused");
      }
      */
    }
  }

  mgr->stop_uart_dev(port_num);

  PX4_INFO("Exiting uart device thread %d",port_num);

  thread_running[port_num] = false;

  return PX4_OK;
}
