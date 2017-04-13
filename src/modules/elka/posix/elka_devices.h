#ifndef ELKA_DEVICES_H
#define ELKA_DEVICES_H

#include <drivers/drv_hrt.h>
#include <map>
#include <stdlib.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include <utility>

#include "elka.h"

namespace elka {
class DeviceNode;
}

// Manages connection with an ELKA device
// Determines mode of input (autonomous or Spektrum)
// Sends elka_msg_s messages to snapdragon_uart
class elka::DeviceNode {
public:
  DeviceNode(uint8_t port_num, uint8_t port_type, uint8_t buf_type, char *dev_name);
  ~DeviceNode();

  // Start serial thread
  int init();

  // Check if port is running correctly
  // @return port_type if exists, else 0
  uint8_t check_port(uint8_t port_num);
  
  // Set parse elka msg and set ack if applicable 
  // If control message, perform appropriate actions on serial port.
  // If from ELKA device, set message in RX buffer.
  // @return msg_type if msg is meant for u
  //         MSG_NULL if msg not meant for u
  //         MSG_FAILED If msg meant for u and incorrect
  uint8_t parse_elka_msg(elka_msg_s &elka_ret, elka_msg_ack_s &elka_ack);
  
  // Set message on buffer
  // Can be used to set messages returned or to send
  // @param elka_msg = message to push to buffer
  // @param bool tx = true if push to tx buffer
  //                  false if push to rx buffer
  // @return msg_type if msg is pushed successfully 
  //         MSG_NULL if msg not meant for u in the case of tx=false
  //         MSG_FAILED if msg meant for u and incorrect
  //                    if msg is not pushed correctly
  uint8_t push_msg(elka_msg_s &elka_msg, bool tx);

  // Write message to TX buffer if there is a message to write
  // First byte is msg type. Second byte is msg length. Next bytes
  // are data.
  // @return true if successful, else false
  uint8_t push_msg(uint8_t port_num, uint8_t msg_type, uint8_t msg_len,
      void *data, bool tx);

  // Retrieve message from buffer and set to elka_msg
  // @param elka_msg = message to set from buffer
  // @param bool tx = true if remove from tx buffer
  //                  false if remove from rx buffer
  bool remove_msg(uint8_t port_num, elka_msg_s &elka_msg, bool tx); 

  //FIXME add in cababilities for bad messages to add up
  //      before being handled
  // Check ack for sent message
  // Check ack with respect to port number from elka_ack.msg_id
  uint8_t check_ack(struct elka_msg_ack_s &elka_ack);

  // Add port access
  // @return true if success, else false
  bool add_port(uint8_t port_num, uint8_t port_type, uint8_t buf_type);

  // Delete port access
  // @return true if success, else false
  bool delete_port(uint8_t port_num);

  // Get state of internal state machine
  // @return elka state defined by ELKA_CTL_<state>
  uint8_t get_port_state(uint8_t port_num);

  // Update _now variable with current time
  void update_time();

private:
  // Data members
  
  struct SerialBuffer {
    void *_buffer;

    uint16_t _msg_num;
    // Ack msg nums are for expectant ack msgs
    uint8_t _type;

    SerialBuffer(uint8_t buf_type);
    ~SerialBuffer();
  };

  struct SerialPort {
    struct SerialBuffer *_tx_buf;
    struct SerialBuffer *_rx_buf;
    // map of <msg_num,<num_retries,data>> key-value pairs
    // for msgs waiting to receive ack
    std::map<uint16_t, uint8_t> _expecting_ack;
    uint8_t _state;
    uint8_t _snd_id;
    uint8_t _rcv_id;
    SerialPort(uint8_t port_n, uint8_t port_t,
        uint8_t buf_t);
    ~SerialPort();
    bool start_port();
    bool stop_port();
    bool pause_port();
    bool resume_port();
  };
  
  // If _ports[i] exists then it is healthy
  SerialPort *_ports[MAX_SERIAL_PORTS];
  // This must be updated frequently thru callback or otherwise!
  hrt_abstime _now;
  char _dev_name[MAX_NAME_LEN];

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
                          struct elka_msg_id_s &msg_id,
                          struct snd_rcv_id_s &ret_id);
  uint8_t parse_port_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id,
                         struct snd_rcv_id_s &ret_id);
  uint8_t parse_elka_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id,
                         struct snd_rcv_id_s &ret_id);


};
#endif
