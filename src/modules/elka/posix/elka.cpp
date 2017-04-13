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

#include "elka.h"
#include "elka_manager.h"

extern "C" { __EXPORT int elka_main(int argc, char *argv[]); }

//TODO may want to add in ability for single ELKA dev to keep track
// of running ports with static array var
// (can be checked in thread loop)
static int daemon_task[MAX_ELKA_DEVS];
static volatile bool thread_should_exit[MAX_ELKA_DEVS];
static volatile bool thread_running[MAX_ELKA_DEVS];
static int num_elka_devs = 0;
const hrt_abstime msg_threshold = 500000;

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
      } else if (!mgr->check_elka_port(dev_num,port_num)) {
        //TODO add port to existing device
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
          buf_type = UINT8_ARRAY; // FIXME why is this not creating UINT8 array?

  sprintf(dev_name,"elka_%d",dev_num);

  // Increment number of elka devs on start
  num_elka_devs++;
  PX4_INFO("Adding elka device %s", dev_name);
  elka::DeviceNode *elka = mgr->get_elka_dev(
      dev_num, port_num, port_type, buf_type, dev_name);

  // Define poll_return for defined file descriptors
  int poll_ret;

  // Subscribe to topics
  int elka_ack_sub_fd = orb_subscribe(ORB_ID(elka_msg_ack));
  int elka_ret_sub_fd = orb_subscribe(ORB_ID(elka_msg));
  // Set update rate to 100Hz
  orb_set_interval(elka_ack_sub_fd, 10);
  // Set update rate to 100Hz
  orb_set_interval(elka_ret_sub_fd, 10);

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
  uint8_t parse_res; 

  // Poll each loop cycle. Parse thru each elka_msg to send
  // to correct port buffer.
  while (!thread_should_exit[dev_num]) {
    elka->update_time();
    // Publish elka msgs on ports
    for (uint8_t i=0; i < MAX_SERIAL_PORTS; i++) {
      // Set and publish elka_snd if message exists
      // First thing done is write to msg queue
      // Next thing done is set message from tx_buf to send thru orb_publish
      // Remove next message from rx_buf to parse
      elka->remove_msg(i, elka_ret_cmd, false);

      if (elka->get_port_state(i) == STATE_RESUME &&
          elka->push_msg(i, MSG_PORT_CTL, debug_length, debug_data, true) &&
          elka->remove_msg(i, elka_snd, true)) {
        PX4_INFO("state: %d", elka->get_port_state(i));
        //PX4_INFO("sending msg on port %d w msg_id %d", i, elka_snd.msg_id);
        orb_publish(ORB_ID(elka_msg), elka_msg_pub, &elka_snd);

      }
    }

    // Wait for up to 200ms for data
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 200);

    // Handle the poll result
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

      if (fds[0].revents & POLLIN) {
        // Obtained data for the first file descriptor
        orb_copy(ORB_ID(elka_msg_ack), elka_ack_sub_fd, &elka_ack_rcv);
        // TODO Here will copy into DeviceNode ringbuf
        // FIXME must parse thru msg to ensure that it is of the correct type
        // TODO Check message timestamp
        PX4_INFO("ELKA message ack:\n\tmsg_id: %d\n\tmsg_num: %d\n\tresult: %d",
            elka_ack_rcv.msg_id,
            elka_ack_rcv.msg_num,
            elka_ack_rcv.result);

        if (elka->check_ack(elka_ack_rcv) == elka_msg_ack_s::ACK_FAILED) {
          PX4_ERR("Failed acknowledgement for msg id: %d",
              elka_ack_rcv.msg_id);
        }

      }
      
      if (fds[1].revents & POLLIN) {
        //FIXME send to parsing function
        // Obtained data for the first file descriptor
        orb_copy(ORB_ID(elka_msg), elka_ret_sub_fd, &elka_ret);
        // push msg to rx_buf 
        elka->push_msg(elka_ret, false);
      }
    }

    // remove and parse message from rx_buf
    // send ack if the message requires one 
    if ((parse_res = elka->parse_elka_msg(elka_ret_cmd,elka_ack_snd)) ==
                  MSG_FAILED) {
      PX4_ERR("Failed message for msg id: %d",
          elka_ret_cmd.msg_id);
    } else if (parse_res != MSG_NULL) {
      orb_publish(ORB_ID(elka_msg_ack), elka_ack_pub, &elka_ack_snd);
    }

    usleep(1000000);
  }

  mgr->stop_elka_dev(dev_num);

  PX4_INFO("Exiting elka device thread %d",dev_num);

  // Decrement number of elka devs on end
  num_elka_devs--;

  return PX4_OK;
}

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t port_num, uint8_t port_type, 
    uint8_t snd_side, uint8_t rcv_side) {
  if (snd_id)
    *snd_id = (port_num << 5) | (port_type << 2) | (snd_side);
  if (rcv_id)
    *rcv_id = (port_num << 5) | (port_type << 2) | (rcv_side);
}

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    struct snd_rcv_id_s &snd_id_struct,
    struct snd_rcv_id_s &rcv_id_struct) {
  if (snd_id)
    *snd_id = (snd_id_struct.port_num << 5) |
              (snd_id_struct.port_type << 2) |
              (snd_id_struct.proc_side);

  if (rcv_id)
    *rcv_id = (rcv_id_struct.port_num << 5) |
              (rcv_id_struct.port_type << 2) |
              (rcv_id_struct.proc_side);
}

void get_snd_rcv_id(uint8_t *snd_id, uint8_t *rcv_id,
    uint32_t msg_id) {
  if (snd_id && rcv_id)
    get_elka_msg_id_attr(snd_id, rcv_id, NULL, NULL, msg_id);
  else if (snd_id)
    get_elka_msg_id_attr(snd_id, NULL, NULL, NULL, msg_id);
  else if (rcv_id)
    get_elka_msg_id_attr(NULL, rcv_id, NULL, NULL, msg_id);
}

void get_snd_rcv_id_attr( uint8_t *port_num, uint8_t *port_type,
    uint8_t *proc_side,
    uint8_t id) {
  if (port_num)
    *port_num = (id & PORT_NUM) >> 5;
  if (port_type)
    *port_type = (id & PORT_TYPE) >> 2;
  if (proc_side)
    *proc_side = (id & PROC_SIDE);
}

void get_snd_rcv_id_attr(struct snd_rcv_id_s *id_struct, uint8_t id) {
  if (id_struct) {
    id_struct->port_num = (id & PORT_NUM) >> 5;
    id_struct->port_type = (id & PORT_TYPE) >> 2;
    id_struct->proc_side = (id & PROC_SIDE);
  }    
}

// Call this before sending elka_msg or elka_msg_ack to set msg_id field
void get_elka_msg_id(uint32_t *msg_id,
    uint8_t snd_id, uint8_t rcv_id,
    uint8_t msg_type, uint8_t length) {
  if (msg_id) {
    *msg_id = (snd_id << 24) | (rcv_id << 16) | (msg_type << 8) | (length);
  }
}

void get_elka_msg_id(uint32_t *msg_id, struct elka_msg_id_s &msg_id_struct) {
  if (msg_id) {
    *msg_id = (msg_id_struct.snd_id << 24) |
              (msg_id_struct.rcv_id << 16) |
              (msg_id_struct.type << 8) |
              (msg_id_struct.length);
  }
}

void get_elka_msg_id_attr(struct elka_msg_id_s *msg_id_struct, uint32_t msg_id) {
  if (msg_id_struct && msg_id) {
    msg_id_struct->snd_id = (msg_id & SENDER_ID) >> 24;
    msg_id_struct->rcv_id = (msg_id & RECEIVER_ID) >> 16;
    msg_id_struct->type = (msg_id & MESSAGE_TYPE) >> 8;
    msg_id_struct->length = (msg_id & MESSAGE_LENGTH);
  }
}

void get_elka_msg_id_attr(uint8_t *snd_id, uint8_t *rcv_id,
    uint8_t *msg_type, uint8_t *length,
    uint32_t msg_id) {
  if (snd_id)
    *snd_id = (msg_id & SENDER_ID) >> 24;

  if (rcv_id)
    *rcv_id = (msg_id & RECEIVER_ID) >> 16;

  if (msg_type)
    *msg_type = (msg_id & MESSAGE_TYPE) >> 8;

  if (length)
    *length = (msg_id & MESSAGE_LENGTH);
}

// Check ELKA ack against known msg_id and msg_num
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    uint32_t &msg_id, uint16_t &msg_num) {
  uint8_t ack_snd_id, ack_rcv_id, ack_msg_type, ack_msg_len,
          self_snd_id, self_rcv_id, self_msg_type, self_msg_len;
  get_elka_msg_id_attr(&ack_snd_id,
                       &ack_rcv_id,
                       &ack_msg_type,
                       &ack_msg_len,
                       elka_msg_ack.msg_id);

  get_elka_msg_id_attr(&self_snd_id,
                       &self_rcv_id,
                       &self_msg_type,
                       &self_msg_len,
                       msg_id);

  // Check that message is for u
  if ( (ack_snd_id == self_rcv_id) &&
          (ack_rcv_id == self_snd_id) ) { // message for u
    // Check that message parameters match 
    if ( !( (ack_msg_type == self_msg_type) &&
            (ack_msg_len == self_msg_len) &&
            (elka_msg_ack.msg_num == msg_num) ) ) {
      PX4_ERR("Ack message specified incorrectly");
      return elka_msg_ack_s::ACK_FAILED;
    } else {
      return elka_msg_ack.result;
    }
  } else { // msg not for u
    return elka_msg_ack_s::ACK_NULL;
  }
}

// Check ELKA ack against known msg_id and msg_num
uint8_t check_elka_ack(struct elka_msg_ack_s &elka_msg_ack,
    struct elka_msg_id_s &msg_id, uint16_t &msg_num) {
  uint8_t ack_snd_id, ack_rcv_id, ack_msg_type, ack_msg_len;

  get_elka_msg_id_attr(&ack_snd_id,
                       &ack_rcv_id,
                       &ack_msg_type,
                       &ack_msg_len,
                       elka_msg_ack.msg_id);
  
  // Check that message is for u
  if ( (ack_snd_id == msg_id.rcv_id) &&
          (ack_rcv_id == msg_id.snd_id) ) { // message for u
    // Check that message parameters match 
    if ( !( (ack_msg_type == msg_id.type) &&
            (ack_msg_len == msg_id.length) &&
            (elka_msg_ack.msg_num == msg_num) ) ) {
      PX4_ERR("Ack message specified incorrectly");
      return elka_msg_ack_s::ACK_FAILED;
    } else {
      return elka_msg_ack.result;
    }
  } else { // msg not for u
    return elka_msg_ack_s::ACK_NULL;
  }
}


void print_char_array(char *buf) {
  char to_print[MAX_MSG_LEN+1];
  memcpy(to_print,buf,*(buf+1)+2);
  to_print[*(buf+1)+3] = 0;
  PX4_INFO("char array: %s",buf);
}

void print_uint8_array(uint8_t *buf) {
  char to_print[4*MAX_MSG_LEN+1],
       a_char[5]; // 3 chars max for a uint8 number and empty space
  uint8_t len = *(buf+1);
  uint8_t i=0;

  memset(to_print,0,4*MAX_MSG_LEN+1);

  while (i++ < len+2) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  PX4_INFO("uint8 array: %s",to_print);
}

//TODO
/*
void print_ringbuf(pringbuf_t buf) {
}
*/
