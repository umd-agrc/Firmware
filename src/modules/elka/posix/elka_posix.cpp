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

// Thread/task 0 is rx loop
// Thread/task 1 is parse loop
static int daemon_task[2];
static volatile bool thread_should_exit[2];
static volatile bool thread_running[2];

void usage();
int elka_rx_loop(int argc, char **argv);
int elka_parse_loop(int argc, char **argv);

static elka::PX4Port *elka_dev = nullptr;

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


    // Order of thread args is:
    //    [0] dev num
    //    [1] port num
    //    [2] port type 
    uint8_t dev_num=0, port_num=0, port_type=0;
    if (argc < 3) {
      PX4_WARN("Missing port number specification");
      return PX4_OK;
    }

    int i=2; // Start after first keyword
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

        dev_args++;
        
      // Assign port if not assigned
      } else if (!strcmp(argv[dev_args],"-p") && !port_num) {
        if (!(port_num = (uint8_t)std::atoi(argv[++dev_args])) ||
            port_num < 1 || port_num > MAX_SERIAL_PORTS) {
          PX4_WARN("Invalid port number specification %s",
              argv[dev_args-1]);
          return PX4_ERROR;
        }

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

        dev_args++;

      // Invalid arg
      } else {
        PX4_ERR("Invalid argument");
        usage();
        return PX4_OK;
      }
    }

    uint8_t queue_sz = 42;
    uint8_t buf_type = PRIORITY_QUEUE; 
    char parse_name[MAX_NAME_LEN], rx_name[MAX_NAME_LEN],
         dev_name[MAX_NAME_LEN];

    sprintf(dev_name, "elka");
    sprintf(parse_name,"elka_parse");
    sprintf(rx_name,"elka_rx");

    if (elka_dev == nullptr) {
      if (!elka::PX4Port::initialize(
            port_num, port_type,
            buf_type, queue_sz, dev_name)) {
        PX4_ERR("elka device alloc failed");
        return -ENOMEM;
      }

      // Create the device
      elka_dev = elka::PX4Port::get_instance();

      if (elka_dev == nullptr) {
        return -errno;
      }
    }

    // Start parse task
    thread_should_exit[1] = false;
    daemon_task[1] = px4_task_spawn_cmd(
      parse_name,
      SCHED_DEFAULT,
      SCHED_PRIORITY_DEFAULT,
      1400,
      elka_parse_loop,
      NULL);

    unsigned constexpr max_wait_us = 1000000;
    unsigned constexpr max_wait_steps = 2000;
    unsigned j;

    for (j=0; j < max_wait_steps; j++) {
      usleep(max_wait_us / max_wait_steps);
      if (thread_running[1]) {
        // Start rx task
        thread_should_exit[0] = false;
        daemon_task[0] = px4_task_spawn_cmd(
          rx_name,
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT,
          1000,
          elka_rx_loop,
          NULL);

        for (j=0; j < max_wait_steps; j++) {
          usleep(max_wait_us / max_wait_steps);
          if (thread_running[0]) {
            break;
          }
        }
        break;
      }
    }

    return !(j < max_wait_steps);

	} else if (!strcmp(argv[1],"status")) {
    if (elka_dev != nullptr) {
      elka_dev->print_statistics(true);
    } else {
      PX4_INFO("elka is not running");
    }

    return PX4_OK;

  //FIXME does this work?
  } else if (!strcmp(argv[0],"stop")) {
    if (!thread_running[0] &&
        !thread_running[1]) {
      PX4_WARN("elka dev already stopped");
      return PX4_OK;
    }

    thread_should_exit[0] = true;
    thread_should_exit[1] = true;

    while(thread_running[0] ||
          thread_running[1]) {
      usleep(200000);
      PX4_WARN(".");
    }

    PX4_WARN("terminated.");

    return PX4_OK;
  } else {
		PX4_WARN("Action not supported");
	}

	return PX4_OK;
}

// Assumes that argv of the format:
//    [0] dev_num
//    [1] port_num
//    [2] port_type
int elka_rx_loop(int argc, char **argv) {
  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to topics
  int elka_ack_sub_fd = orb_subscribe(ORB_ID(elka_msg_ack));
  int elka_ret_sub_fd = orb_subscribe(ORB_ID(elka_msg));
  // Set update rate to 100Hz
  orb_set_interval(elka_ack_sub_fd, 10);
  // Set update rate to 100Hz
  orb_set_interval(elka_ret_sub_fd, 10);

  // snd_id is sender id for received elka_msgs and elka_ack_msgs
  dev_id_t snd_id;

  // Define topic subscribers to wait for with polling
  px4_pollfd_struct_t fds[] = {
    {.fd = elka_ack_sub_fd, .events = POLLIN},
    {.fd = elka_ret_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;

  thread_running[0] = true;

  // Poll each loop cycle. Parse thru each elka_msg to send
  // to correct port buffer.
  // TODO figure out how to receive multiple messages at once thru uORB
  while (!thread_should_exit[0]) {
    elka_dev->update_time();

    // Wait for up to 500ms for data
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
            &elka_dev->_elka_ack_rcv);

        // TODO Check message timestamp
        get_elka_msg_id_attr(&snd_id, NULL, NULL, NULL, NULL,
            elka_dev->_elka_ack_rcv.msg_id);
        if (cmp_dev_id_t(snd_id, elka_dev->_id))
          elka_dev->push_msg(elka_dev->_elka_ack_rcv, false);
      }
      
      if (fds[1].revents & POLLIN) {
        // Obtained data for the first file descriptor
        orb_copy(ORB_ID(elka_msg),
            elka_ret_sub_fd,
            &elka_dev->_elka_rcv);

        // Push msg to rx_buf 
        // Do not push if you are the sender to prevent
        // cycles from forming
        get_elka_msg_id_attr(&snd_id, NULL, NULL, NULL, NULL,
            elka_dev->_elka_rcv.msg_id);
        if (cmp_dev_id_t(snd_id, elka_dev->_id))
          elka_dev->push_msg(elka_dev->_elka_rcv, false);
      }
    }
  }

  thread_running[0] = false;

  return PX4_OK;
}

int elka_parse_loop(int argc, char **argv) {
  thread_running[1] = true;

  int debug_length = 12;
  uint8_t debug_data[MAX_MSG_LEN+1] = {0,1,3,1,0,1,3,1,0,1,3,1};

  // Determines result of parsing previous message.
  // If MSG_NULL, then don't send ack.
  // Else send ack.
  // nxt_msg_type[0] is tx, nxt_msg_type[1] is rx
  // nxt_msg_len[0] is tx, nxt_msg_len[1] is rx
  uint8_t nxt_msg_type[2], nxt_msg_len[2],
          parse_res, dbg = 0; 

  // DEBUGGING: Perform routing -------------
  // FIXME
  elka_dev->set_dev_props_msg(
      elka_dev->_id,
      (dev_id_t)0,
      true,
      elka_dev->_elka_snd);

  get_elka_msg_id_attr(NULL,NULL,NULL,
      &nxt_msg_type[0],&nxt_msg_len[0],
      elka_dev->_elka_snd.msg_id);

  elka_dev->add_msg(
      nxt_msg_type[0], 
      nxt_msg_len[0],
      elka_dev->_elka_snd.num_retries,
      elka_dev->_elka_snd.msg_num,
      elka_dev->_elka_snd.data,
      NULL);
  // END DEBUGGING: Perform routing ----------

  while (!thread_should_exit[1]) {
    // Remove next message from _rx_buf to parse
    // Skip parsing if there is no message in buffer
    // Push ack if necessary
    if ((nxt_msg_type[1] = elka_dev->remove_msg(elka_dev->_elka_rcv_cmd, 
                                           elka_dev->_elka_ack_rcv_cmd,
                                           false)) == MSG_ACK) {
      if ((parse_res = elka_dev->parse_elka_msg(
                          elka_dev->_elka_ack_rcv_cmd))
          == MSG_FAILED) {
        PX4_ERR("Failed message for msg id: " PRMIT "",
            elka_dev->_elka_ack_rcv_cmd.msg_id);
      }
    } else if (nxt_msg_type[1] != MSG_NULL) {
      // Parse message
      // Send ack if the message requires one 
      if ((parse_res = elka_dev->parse_elka_msg(
                          elka_dev->_elka_rcv_cmd))
          == MSG_FAILED) {
        PX4_ERR("Failed message for msg id: " PRMIT "",
            elka_dev->_elka_rcv_cmd.msg_id);
      }
    }

    //FIXME debugging
    debug_data[6] = dbg++;

    if (elka_dev->get_state() == STATE_RESUME) {
      // For debugging, alternate between sending port msg
      // and motor_cmd
      // TODO routing
      if ((dbg/2) % 2) {
        elka_dev->add_msg(MSG_PORT_CTL,
                debug_length,
                0,
                0,
                debug_data,
                NULL);
      } else {
        elka_dev->add_msg(MSG_ELKA_CTL,
                debug_length,
                0,
                0,
                debug_data,
                NULL);
      }

      // Transmit next message from _tx_sb
      if ( (nxt_msg_type[0] = elka_dev->get_msg(
                                elka_dev->_elka_snd,
                                elka_dev->_elka_ack_snd,
                                true))
           == MSG_ACK ) {
        elka_dev->send_msg(elka_dev->_elka_ack_snd);
        elka_dev->erase_msg(elka_dev->_elka_ack_snd.msg_id,
                            elka_dev->_elka_ack_snd.msg_num,
                            true);
      } else if (nxt_msg_type[0] != MSG_NULL &&
                 nxt_msg_type[0] != MSG_FAILED ) {

        elka_dev->send_msg(elka_dev->_elka_snd);

        if (!(elka_dev->_elka_snd.msg_id & ID_EXPECTING_ACK)) {
          elka_dev->erase_msg(elka_dev->_elka_snd.msg_id,
                              elka_dev->_elka_snd.msg_num,
                              true);
        } else {// Sleep for a short time to let ack process
          usleep(20000);
        }
      }
    }

    usleep(1000000);
  }

  thread_running[1] = false;
  return PX4_OK;
}
