#ifndef SNAPDRAGON_UART_DEVICES_H
#define SNAPDRAGON_UART_DEVICES_H

#include <drivers/drv_hrt.h>
#include <map>
#include <elka/common/elka.h>
#include <elka/common/elka_comm.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#include "snapdragon_uart.h"
#include "basic_uart.h"

namespace uart {
class DeviceNode;
}

class uart::DeviceNode {
public:
  // This must be updated frequently thru callback or otherwise!
  hrt_abstime _now; 

  DeviceNode(uint8_t port_num, char *dev_name, uint8_t buf_t);
  ~DeviceNode();

  // Open port for UART
  int init();

  // Serial methods ------------------------------
  // @return serial_fd for desired port_num
  int open();

  int close();

  int assign_read_callback();

  // Read from Elka side
  int remove_elka(elka_msg_s &elka_ret);

  // Write to ELKA side
  int write(elka_msg_s &elka_snd);

  // Read and write to ELKA side
  int read_write(elka_msg_s &elka_ret, elka_msg_s &elka_snd);

  // Check if port is running correctly
  // @return port_type if exists, else 0
  uint8_t check_port(uint8_t port_num, bool px4);

  // Set elka state
  void set_state_msg(elka_msg_s &elka_snd, uint8_t state, bool px4);

  // Set parse elka msg and set ack if applicable 
  // If control message, perform appropriate actions on serial port.
  // If from ELKA device, set message in RX buffer.
  // @param elka_ret message to parse
  // @param elka_ack ack to set
  // @param px4 if from PX4, then true
  //            else false
  // @return msg_type if msg is meant for u
  //         MSG_NULL if msg not meant for u
  //         MSG_FAILED If msg meant for u and incorrect
  uint8_t parse_elka_msg(elka_msg_s &elka_ret, elka_msg_ack_s &elka_ack,
      bool px4);

  // Set message on buffer
  // Can be used to set messages returned or to send
  // @param elka_msg = message to push to buffer
  // @param bool tx = true if push to tx buffer
  //                  false if push to rx buffer
  // @return msg_type if msg is pushed successfully 
  //         MSG_NULL if msg not meant for u in the case of tx=false
  //         MSG_FAILED if msg meant for u and incorrect
  //                    if msg is not pushed correctly
  uint8_t push_msg(elka_msg_s &elka_msg, bool px4, bool tx);

  // Retrieve message from buffer and set to elka_msg
  // @param elka_msg = message to set from buffer
  // @param bool tx = true if remove from tx buffer
  //                  false if remove from rx buffer
  bool remove_msg(elka_msg_s &elka_msg, bool px4, bool tx);

  //FIXME add in cababilities for bad messages to add up
  //      before being handled
  // Check ack for sent message
  // Check ack with respect to port number from elka_ack.msg_id
  uint8_t check_ack(struct elka_msg_ack_s &elka_ack,
      bool px4);

  // Update _now variable with current time
  void update_time();

private:

  // Encapsulates necessary aspects of communication to either PX4 ELKA or
  // hardware ELKA
  struct MultiPort : elka::CommPort {
    MultiPort(uint8_t port_num, uint8_t buf_type, uint8_t proc_side);
    ~MultiPort();
    bool start_port() override;
    bool stop_port() override;
    bool pause_port() override;
    bool resume_port() override;
  };

  // Data members
  uint8_t _state; // state of Snapdragon UART
  int _serial_fd;
  char _dev_name[MAX_NAME_LEN];
  struct MultiPort *_elka_comm;
  struct MultiPort *_px4_comm;
 
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
                          struct snd_rcv_id_s &ret_id,
                          bool px4);
  // Control message to serial port drivers.
  // Can start, stop, pause, resume ELKA port behavior.
  // Still allow Spektrum to run.
  uint8_t parse_port_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id,
                         struct snd_rcv_id_s &ret_id,
                         bool px4);
  // Pass control message between PX4 ELKA and ELKA hardware
  uint8_t parse_elka_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id,
                         struct snd_rcv_id_s &ret_id,
                         bool px4);

  int deinit();
};
#endif
