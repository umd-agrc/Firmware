#ifndef ELKA_DEVICES_H
#define ELKA_DEVICES_H

#include <drivers/drv_hrt.h>
#include <map>
#include <stdlib.h>
#include <elka/common/elka.h>
#include <elka/common/elka_comm.h>
#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include <utility>

#include "elka_posix.h"
#include "inet_comm.h"

namespace elka {
class PX4Port;
}

// Manages connection with an ELKA device
// Determines mode of input (autonomous or Spektrum)
// Sends elka_msg_s messages to snapdragon_uart
class elka::PX4Port : public elka::CommPort {
public:

  // elka_ack_snd is ack to be sent after parsing next cmd from rx_buf
  // elka_ack_rcv is ack received after sending msg from tx_buf
  struct elka_msg_ack_s _elka_ack_rcv, _elka_ack_snd;
  // elka_snd is msg to send from tx buf
  // elka_ret is msg to push to rx buf
  // elka_rcv_cmd is msg to be parsed from rx buf
  struct elka_msg_s _elka_snd, _elka_ret, _elka_ret_cmd;

  orb_advert_t _elka_msg_pub;
  orb_advert_t _elka_ack_pub;

  PX4Port(uint8_t port_num, uint8_t port_type, uint8_t buf_type,
      uint8_t size, char *dev_name);

  ~PX4Port();

  // Start serial thread
  int init();

  // Add message to buffer
  // Adds message to buffer for all applicable
  // devices unless target_dev is specified
  // @return msg_type
  uint8_t add_msg(uint8_t msg_type,
                  uint8_t len,
                  uint8_t num_retries,
                  uint16_t msg_num,
                  uint8_t *data,
                  dev_id_t *target_dev);

  // Remove and send front message from buffer
  uint8_t send_msg(elka_msg_s &elka_msg);
  uint8_t send_msg(elka_msg_ack_s &elka_msg);
  
  // Parse elka msg and set ack if applicable 
  // If control message, perform appropriate actions on serial port.
  // If from ELKA device, set message in RX buffer.
  // @return msg_type if msg is meant for u
  //         MSG_NULL if msg not meant for u
  //         MSG_FAILED If msg meant for u and incorrect
  uint8_t parse_elka_msg(elka_msg_s &elka_ret);

  // Check ack for sent message
  // Check ack with respect to port number from elka_ack.msg_id
  uint8_t check_ack(struct elka_msg_ack_s &elka_ack);

  // Get state of internal state machine
  // @return elka state defined by ELKA_CTL_<state>
  uint8_t get_state();
  
  // Set elka state in elka_msg. May push this to a buffer after
  uint8_t set_dev_state_msg(
      elka_msg_s &elka_snd,
      dev_id_t rcv_id,
      uint8_t state,
      bool elka_ctl);

  // Update _now variable with current time
  void update_time();

private:
  
  bool start_port() override;
  bool stop_port() override;
  bool pause_port() override;
  bool resume_port() override;

  /*
  // Map from port id to port num
  // IDs correspond to _ports[i]->_id
  std::map<dev_id_t, uint8_t> _port_num_map;
  */

  // This must be updated frequently thru callback or otherwise!
  hrt_abstime _now;
  char _dev_name[MAX_NAME_LEN];
  Child _inet_proc;

  void wait_for_child(Child *child);

  // Class methods
  int deinit();

  // Helper functions for parsing returned elka message based on current state
  // @return msg type:
  //         parse_motor_cmd() always returns MSG_NULL
  //         parses_port_ctl and parse_elka_ctl return:
  //         MSG_FAILED if msg parsing failed
  //         MSG_NULL if msg not for u
  //         Can return all other types of msgs except for MSG_ACK
  // Resumes elka if paused or started. Starts elka if stopped
  uint8_t parse_motor_cmd(elka_msg_s &elka_ret,
                          elka_msg_ack_s &elka_ack,
                          struct elka_msg_id_s &msg_id);
  uint8_t parse_port_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);
  uint8_t parse_elka_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);

};
#endif
