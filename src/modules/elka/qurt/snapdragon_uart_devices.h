#ifndef SNAPDRAGON_UART_DEVICES_H
#define SNAPDRAGON_UART_DEVICES_H

#include <drivers/drv_hrt.h>
#include <map>
#include <px4_posix.h>
#include <elka/common/elka.h>
#include <elka/common/elka_comm.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include <uORB/topics/input_rc.h>

#include "snapdragon_uart.h"
#include "basic_uart.h"

namespace uart {
class UARTPort;
}

class uart::UARTPort : public elka::CommPort {
public:
  // This must be updated frequently thru callback or otherwise!
  hrt_abstime _now; 

  input_rc_s _input_rc;
  elka_msg_ack_s _elka_ack_snd, _elka_ack_rcv;
  elka_msg_s _elka_snd, _elka_rcv;

  // Advertise elka msg and elka msg ack
  orb_advert_t _elka_ack_pub;
  orb_advert_t _elka_msg_pub;

  UARTPort(uint8_t port_num, uint8_t buf_t, uint8_t size,
      char *dev_name);
  ~UARTPort();

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
  int write(elka_msg_s &elka_msg);

  // Read and write to ELKA side
 // int read_write(elka_msg_s &elka_ret, elka_msg_s &elka_snd);
  
  // End serial methods ------------------------------

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

  // Set elka state in elka_msg. May push this to a buffer after
  uint8_t set_dev_state_msg(
      elka_msg_s &elka_snd,
      dev_id_t rcv_id,
      uint8_t state,
      bool elka_ctl);

  // Send message at front of _tx_buf in appropriate manner
  uint8_t send_msg(elka_msg_s &elka_msg);
  uint8_t send_msg(elka_msg_ack_s &elka_msg);

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
  uint8_t parse_elka_msg(
      elka_msg_s &elka_ret,
      elka_msg_ack_s &elka_ack);

  // Check ack with most recent message sent
  // Check ack with respect to port number from elka_ack.msg_id
  // @param bool px4 = true if check with px4 buffer
  //                   false if check with elka buffer
  uint8_t check_ack(struct elka_msg_ack_s &elka_ack);

  // Update _now variable with current time
  void update_time();

private:

  bool start_port() override;
  bool stop_port() override;
  bool pause_port() override;
  bool resume_port() override;

  // Data members
  uint8_t _state; // state of Snapdragon UART
  int _serial_fd;
  char _dev_name[MAX_NAME_LEN];
 
  /*
  // Map from port id to port num
  std::map<dev_id_t, uint8_t> _port_num_map;
  */

  // Helper functions for parsing returned elka message based on current state
  // @return msg type:
  //         parse_motor_cmd always returns MSG_NULL on success
  //                                          MSG_FAILED on failure
  //         parse_port_ctl and parse_elka_ctl return:
  //           MSG_FAILED if msg parsing failed
  //          MSG_NULL if msg not for u
  //         Can return all other types of msgs except for MSG_ACK
  // Resumes elka if paused or started. Starts elka if stopped
  uint8_t parse_motor_cmd(elka_msg_s &elka_ret,
                          elka_msg_ack_s &elka_ack,
                          struct elka_msg_id_s &msg_id);

  // Control message to serial port drivers.
  // Can start, stop, pause, resume ELKA port behavior.
  // Still allow Spektrum to run.
  uint8_t parse_port_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);

  // Pass control message between PX4 ELKA and ELKA hardware
  uint8_t parse_elka_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);

  int deinit();
};
#endif
