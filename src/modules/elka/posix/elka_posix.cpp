//TODO add in ability for single ELKA to use multiple ports
#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#include "elka_posix.h"
#include "elka_manager.h"

extern "C" { __EXPORT int elka_main(int argc, char *argv[]); }

//TODO may want to add in ability for single ELKA dev to keep track
// of running ports with static array var
// (can be checked in thread loop)
static int daemon_task[MAX_ELKA_DEVS];
static volatile bool thread_should_exit[MAX_ELKA_DEVS];
static volatile bool thread_running[MAX_ELKA_DEVS];
static int num_elka_devs = 0;

void usage();
int elka_dev_loop(int argc, char **argv);

static elka::Manager *mgr = nullptr;

void usage() {
  PX4_WARN("usage:\n\t\
elka <start -d <dev_num> -p <port_num> -t <port_type>|\n\
stop -d <dev_num> -p <port_num> -t <port_type>|\n\
status>");
}

int elka_main(int argc, char *argv[]) {

	if (argc < 2) {
		PX4_WARN("Missing action.");
    usage();
    return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {

    if (mgr == nullptr) {
      if (!elka::Manager::initialize()) {
        PX4_ERR("elka manager alloc failed");
        return -ENOMEM;
      }

      // Create the manager
      mgr = elka::Manager::get_instance();

      if (mgr == nullptr) {
        return -errno;
      }
    }

    // Order of thread args is:
    //    [0] dev num
    //    [1] port num
    //    [2] port type 
    char *thread_args[] = {NULL, NULL, NULL, NULL};
    char dev_name[MAX_NAME_LEN];
    uint8_t dev_num=0, port_num=0, port_type=0;
    if (argc < 3) {
      PX4_WARN("Missing port number specification");
      return PX4_OK;
    }

    int i=2; // Start after first keyword
    while (argv[i]) {
      if (num_elka_devs >= MAX_ELKA_DEVS) {
        PX4_WARN("Too many elka devs specified");
        return PX4_ERROR;
      }

      int dev_args = i;
      bool collecting_args = true;
      while (collecting_args) {
        // Check if next arg is a new device spec
        // and all specs filled for this device
        if ((!argv[dev_args] || std::atoi(argv[dev_args])) &&
            dev_num && port_num && port_type){
          collecting_args = false;

        // Assign device if not assigned
        } else if (!strcmp(argv[dev_args],"-d") && !dev_num) {
          if (!(dev_num = (uint8_t)std::atoi(argv[++dev_args])) ||
              dev_num < 1 || dev_num > MAX_ELKA_DEVS) {
            PX4_WARN("Invalid device number specification %s",
                argv[dev_args-1]);
            return PX4_ERROR;
          }

          thread_args[0] = (char *)&dev_num;
          dev_args++;
          
        // Assign port if not assigned
        } else if (!strcmp(argv[dev_args],"-p") && !port_num) {
          if (!(port_num = (uint8_t)std::atoi(argv[++dev_args])) ||
              port_num < 1 || port_num > MAX_SERIAL_PORTS) {
            PX4_WARN("Invalid port number specification %s",
                argv[dev_args-1]);
            return PX4_ERROR;
          }

          thread_args[1] = (char *)&port_num;
          dev_args++;
          
        } else if (!strcmp(argv[dev_args],"-t") && !port_type) {
          if (!strcmp(argv[++dev_args],"UART"))
            //FIXME add other port types
            port_type = PORT_UART;
          else {
            PX4_WARN("Invalid port type specification %s",
                argv[dev_args-1]);
            return PX4_ERROR;
          }

          thread_args[2] = (char *)&port_type;
          dev_args++;

        // Invalid arg
        } else {
          PX4_ERR("Invalid argument");
          usage();
          return PX4_OK;
        }
      }

      i=dev_args; // Set device index var to the current arg index var
      
      if (mgr !=nullptr && !mgr->check_elka_dev(dev_num)) {
        thread_should_exit[dev_num] = false;
        sprintf(dev_name,"elka_dev_%d",dev_num);
        daemon_task[dev_num] = px4_task_spawn_cmd(
          dev_name,
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT,
          1000,
          elka_dev_loop,
          (char * const *)thread_args);
 
        unsigned constexpr max_wait_us = 1000000;
        unsigned constexpr max_wait_steps = 2000;
        unsigned j;

        for (j=0; j < max_wait_steps; j++) {
          usleep(max_wait_us / max_wait_steps);
          if (thread_running[dev_num]) {
            break;
          }
        }
        return !(j < max_wait_steps);
      }

      i++;
    } 

	} else if (!strcmp(argv[1],"status")) {
    if (mgr != nullptr) {
      mgr->print_statistics(true);
    } else {
      PX4_INFO("elka is not running");
    }

    return PX4_OK;

  //FIXME does this work?
  } else if (!strcmp(argv[0],"stop")) {
    if (argc < 3) {
      // stop all ports
      for (int i=0; i < MAX_ELKA_DEVS; i++) {
        if (!thread_running[i]) {
          PX4_WARN("elka dev %d already stopped",i);
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
      int i=2; // Start at `-` args

      while (argv[i]) {
        uint8_t dev_num=0, port_num=0;

        int dev_args = i;
        bool collecting_args = true;
        while (collecting_args) {
          // Check if next arg is a new device spec
          // and all specs filled for this device
          if ((!argv[dev_args] || std::atoi(argv[dev_args])) &&
              dev_num && port_num){
            collecting_args = false;

          // Assign device if not assigned
          } else if (!strcmp(argv[dev_args],"-d") && !dev_num) {
            if (!(dev_num = (uint8_t)std::atoi(argv[++dev_args])) ||
                dev_num < 1 || dev_num > MAX_ELKA_DEVS) {
              PX4_WARN("Invalid device number specification %s",
                  argv[dev_args-1]);
              return PX4_ERROR;
            }
            
          // Assign port if not assigned
          } else if (!strcmp(argv[dev_args],"-p") && !port_num) {
            if (!(port_num = (uint8_t)std::atoi(argv[++dev_args])) ||
                port_num < 1 || port_num > MAX_SERIAL_PORTS) {
              PX4_WARN("Invalid port number specification %s",
                  argv[dev_args-1]);
              return PX4_ERROR;
            }
            
          // Invalid arg
          } else {
            PX4_ERR("Invalid argument");
            usage();
            return PX4_OK;
          }
        }

        i=dev_args; // Set device index var to the current arg index var
        if (!thread_running[dev_num]) {
          PX4_WARN("elka dev %d already stopped",dev_num);
          return PX4_OK;
        }

        thread_should_exit[dev_num] = true;

        while(thread_running[dev_num]) {
          usleep(200000);
          PX4_WARN(".");
        }

        PX4_WARN("terminated.");

        i++;
      }

      return PX4_OK;
    }

  } else {
		PX4_WARN("Action not supported");
	}

	return PX4_OK;
}


// Assumes that argv of the format:
//    [0] dev_num
//    [1] port_num
//    [2] port_type
int elka_dev_loop(int argc, char **argv) {
  char dev_name[MAX_NAME_LEN];
  uint8_t dev_num = (uint8_t)*argv[0],
          port_num = (uint8_t)*argv[1],
          port_type = (uint8_t)*argv[2],
          buf_type = PRIORITY_QUEUE; 

  sprintf(dev_name,"elka_%d",dev_num);

  // Increment number of elka devs on start
  num_elka_devs++;
  PX4_INFO("Adding elka device %s", dev_name);

  uint8_t queue_sz = 42;
  elka::PX4Port *elka = mgr->get_elka_dev(
      dev_num, port_num, port_type, buf_type, queue_sz, dev_name);

  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to topics
  int elka_ack_sub_fd = orb_subscribe(ORB_ID(elka_msg_ack));
  int elka_ret_sub_fd = orb_subscribe(ORB_ID(elka_msg));
  // Set update rate to 100Hz
  orb_set_interval(elka_ack_sub_fd, 10);
  // Set update rate to 100Hz
  orb_set_interval(elka_ret_sub_fd, 10);

  /*
  //TODO add these as class variables
  // elka_ack_snd is ack to be sent after parsing next cmd from rx_buf
  // elka_ack_rcv is ack received after sending msg from tx_buf
  struct elka_msg_ack_s elka_ack_rcv, elka_ack_snd;
  // elka_snd is msg to send from tx buf
  // elka_ret is msg to push to rx buf
  // elka_rcv_cmd is msg to be parsed from rx buf
  struct elka_msg_s elka_snd, elka_ret, elka_ret_cmd;
  memset(&elka_ack_snd, 0, sizeof(elka_ack_snd));
  memset(&elka_ack_rcv, 0, sizeof(elka_ack_rcv));
  memset(&elka_snd, 0, sizeof(elka_snd));
  memset(&elka_ret, 0, sizeof(elka_ret));
  memset(&elka_ret_cmd, 0, sizeof(elka_ret_cmd));
  
  // Advertise attitude topic
  orb_advert_t elka_msg_pub = orb_advertise(
      ORB_ID(elka_msg), &elka_snd);
  orb_advert_t elka_ack_pub = orb_advertise(
      ORB_ID(elka_msg_ack), &elka_ack_snd);
  
  //orb_advert_t elka_msg_pub = orb_advertise(
  //    ORB_ID(elka_msg), NULL);
  //orb_advert_t elka_ack_pub = orb_advertise(
  //    ORB_ID(elka_msg_ack), NULL);
  */

  // Define topic subscribers to wait for with polling
  px4_pollfd_struct_t fds[] = {
    {.fd = elka_ack_sub_fd, .events = POLLIN},
    {.fd = elka_ret_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;

  thread_running[dev_num] = true;

  //FIXME debugging uart
  int debug_length = 12;
  uint8_t debug_data[MAX_MSG_LEN+1] = {0,1,3,1,0,1,3,1,0,1,3,1};
  
  // Determines result of parsing previous message.
  // If MSG_NULL, then don't send ack.
  // Else send ack.
  // nxt_msg_type[0] is tx, nxt_msg_type[1] is rx
  // nxt_msg_len[0] is tx, nxt_msg_len[1] is rx
  uint8_t parse_res, nxt_msg_type[2], 
          nxt_msg_len[2], dbg = 0; 
  // snd_id is sender id for received elka_msgs and elka_ack_msgs
  dev_id_t snd_id;

  // DEBUGGING: Perform routing -------------
  // FIXME
  elka->set_dev_props_msg(
      elka->_id,
      (dev_id_t)0,
      true,
      elka->_elka_snd);

  get_elka_msg_id_attr(NULL,NULL,NULL,
      &nxt_msg_type[0],&nxt_msg_len[0],
      elka->_elka_snd.msg_id);

  elka->add_msg(
      nxt_msg_type[0], 
      nxt_msg_len[0],
      elka->_elka_snd.num_retries,
      elka->_elka_snd.msg_num,
      elka->_elka_snd.data,
      NULL);
  // END DEBUGGING: Perform routing ----------

  // Poll each loop cycle. Parse thru each elka_msg to send
  // to correct port buffer.
  // TODO figure out how to receive multiple messages at once thru uORB
  while (!thread_should_exit[dev_num]) {
    elka->update_time();

    // Set and publish elka_snd if message exists
    // First thing done is write to msg queue
    // Next thing done is set message from tx_buf
    // to send thru orb_publish

    // Remove next message from _rx_buf to parse
    // Skip parsing if there is no message in buffer
    // Push ack if necessary
    if ((nxt_msg_type[1] = elka->remove_msg(elka->_elka_ret_cmd, 
                                           elka->_elka_ack_rcv,
                                           false)) == MSG_ACK) {
        // TODO push to _rc_buf and check in a parse_elka_msg()
        // TODO Check message timestamp
        if (elka->check_ack(elka->_elka_ack_rcv)
            == elka_msg_ack_s::ACK_FAILED) {
          PX4_ERR("Failed acknowledgement for msg id: " PRMIT "",
              elka->_elka_ack_rcv.msg_id);
        }
    } else if (nxt_msg_type[1] != MSG_NULL) {
      // Parse message
      // Send ack if the message requires one 
      if ((parse_res = elka->parse_elka_msg(
                          elka->_elka_ret_cmd))
          == MSG_FAILED) {
        PX4_ERR("Failed message for msg id: " PRMIT "",
            elka->_elka_ret_cmd.msg_id);
      }
      if (parse_res & TYPE_EXPECTING_ACK) {
        elka->push_msg(elka->_elka_ack_snd, true);
      }
    }

    //FIXME debugging
    debug_data[6] = dbg++;

    if (elka->get_state() == STATE_RESUME) {
      // For debugging, alternate between sending port msg
      // and motor_cmd
      // TODO routing
      if (false) {
        if ((dbg/2) % 2) {
          elka->add_msg(MSG_PORT_CTL,
                  debug_length,
                  0,
                  0,
                  debug_data,
                  NULL);
        } else {
          elka->add_msg(MSG_MOTOR_CMD,
                  debug_length,
                  0,
                  0,
                  debug_data,
                  NULL);
        }
      }

      if ( (nxt_msg_type[0] = elka->get_msg(
                                elka->_elka_snd,
                                elka->_elka_ack_snd,
                                true))
           == MSG_ACK ) {
        elka->send_msg(elka->_elka_ack_snd);
      } else if (nxt_msg_type[0] != MSG_NULL &&
                 nxt_msg_type[0] != MSG_FAILED ) {

        elka->send_msg(elka->_elka_snd);

        if (!(elka->_elka_snd.msg_id & ID_EXPECTING_ACK)) {
          // TODO This scheme of popping only works if messages are
          // pushed here and not in a separate thread
          elka->pop_msg(true);
        } else {// Sleep for a short time to let ack process
          usleep(20000);
        }
      }
    }

    // Wait for up to 200ms for data
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 200);

    // Handle the poll result
    if (poll_ret == 0) {
      // None of our providers is giving us data
      PX4_WARN("Got no data");
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
        orb_copy(ORB_ID(elka_msg_ack), elka_ack_sub_fd,
            &elka->_elka_ack_rcv);

        // TODO push to _rx_buf and check in a parse_elka_msg()
        // TODO Check message timestamp
        if (elka->check_ack(elka->_elka_ack_rcv)
            == elka_msg_ack_s::ACK_FAILED) {
          PX4_ERR("Failed acknowledgement for msg id: " PRMIT "",
              elka->_elka_ack_rcv.msg_id);
        }
      }
      
      if (fds[1].revents & POLLIN) {
        // Obtained data for the first file descriptor
        orb_copy(ORB_ID(elka_msg),
            elka_ret_sub_fd,
            &elka->_elka_ret);

        // Push msg to rx_buf 
        // Do not push if you are the sender to prevent cycles from
        // forming
        get_elka_msg_id_attr(&snd_id, NULL, NULL, NULL, NULL,
            elka->_elka_ret.msg_id);
        if (cmp_dev_id_t(snd_id, elka->_id))
          elka->push_msg(elka->_elka_ret, false);
      }
    }

    usleep(1000000);
  }

  mgr->stop_elka_dev(dev_num);

  PX4_INFO("Exiting elka device thread %d",dev_num);

  // Decrement number of elka devs on end
  num_elka_devs--;

  thread_running[dev_num] = false;

  return PX4_OK;
}
