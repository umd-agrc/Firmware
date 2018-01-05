#include <cstring>
#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>

#include "snapdragon_uart_devices.h"

//-----------------Public Methods----------------------
uart::UARTPort::UARTPort(uint8_t port_num, uint8_t buf_t,
    uint8_t size, char *dev_name)
  : elka::CommPort(port_num, PORT_UART, buf_t, size) {

  get_snd_params(&_snd_params,
      port_num, PORT_UART, DEV_PROP_QURT_SIDE);

  if (!(init() == PX4_OK)) {
    PX4_ERR("Unable to initialize elka device %s",dev_name);
    errno = ECANCELED;
  } else {
    strcpy(_dev_name, dev_name);
  }
}

uart::UARTPort::~UARTPort() {
  delete _tx_buf;
  delete _rx_buf;

  if (deinit() != PX4_OK) {
    PX4_ERR("Unable to deinitialize elka device with %s",
        _dev_name);
  }
}

// Open UART port
// @return PX4_OK on success, else PX4_ERROR
int uart::UARTPort::init() {
  memset(&_input_rc, 0, sizeof(_input_rc));
  memset(&_elka_ack_snd, 0, sizeof(_elka_ack_snd));
  memset(&_elka_ack_rcv, 0, sizeof(_elka_ack_rcv));
  memset(&_elka_snd, 0, sizeof(_elka_snd));
  memset(&_elka_rcv, 0, sizeof(_elka_rcv));

  _elka_ack_pub = orb_advertise(
      ORB_ID(elka_msg_ack), &_elka_ack_snd);
  _elka_msg_pub = orb_advertise(
      ORB_ID(elka_msg), &_elka_snd);
  //orb_advert_t elka_ack_pub = orb_advertise(
  //    ORB_ID(elka_msg_ack), NULL);
  //orb_advert_t elka_msg_pub = orb_advertise(
  //    ORB_ID(elka_msg), NULL);

  // Add device properties
  _routing_table[_id].add_prop(DEV_PROP_QURT_SIDE);
  _routing_table[_id].add_prop(DEV_PROP_SENSE_LOCATION);
  _routing_table[_id].add_prop(DEV_PROP_SPIN_MOTORS);

  start_port();
  if (open() == PX4_OK) {
    PX4_INFO("Initialized Snapdragon UART device to communicate on port %s with fd %d",
        _dev_name, _serial_fd);
    return PX4_OK;
  } else {
    return PX4_ERROR;
  }
}

// Close port access
// @return PX4_OK on success, else PX4_ERROR
int uart::UARTPort::deinit() {
  stop_port();
  if (close() == PX4_OK)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::UARTPort::open() {
  if ((_serial_fd = serial_open(_port_num,
          _tx_buf,_rx_buf)) > 0 )
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::UARTPort::close() {
  if (!(_serial_fd = serial_close(_serial_fd, _port_num)))
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::UARTPort::assign_read_callback() {
  if (assign_serial_read_callback(_serial_fd, _port_num)
         == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

// Write to ELKA side
int uart::UARTPort::write(elka_msg_s &elka_msg) {
  return write_elka_msg(_serial_fd, elka_msg);
}

uint8_t uart::UARTPort::add_msg(
    uint8_t msg_type,
    uint8_t len,
    uint8_t num_retries,
    uint16_t msg_num,
    uint8_t *data,
    dev_id_t *target_dev) {
  std::vector<dev_id_t> target_devs;
  
  // If msg_type is MSG_ROUTE_DEV_PROPS
  // and there is no target device,
  // then just push message in all possible ways,
  // because this is supposed to be a broadcast message.
  // If msg_type is not MSG_ROUTE_DEV_PROPS
  // or there is a target device,
  // then form vector of target devices.
  // TODO
  if (msg_type == MSG_ROUTE_DEV_PROPS && !target_dev) {
    elka_msg_s elka_msg;
    get_elka_msg_id(&elka_msg.msg_id,
      _id, 0, _snd_params,
      msg_type, len);

    memcpy(elka_msg.data, data, len);

    push_msg(elka_msg, true);

    return msg_type;
  } else if (target_dev) {
    target_devs.push_back(*target_dev); 
  } else {
    std::map<dev_id_t, elka::DeviceRoute, dev_id_tCmp>::iterator
      dev_routes = _routing_table.begin();
    for (; dev_routes != _routing_table.end(); dev_routes++) {
      if ( check_dev_compatible(msg_type,
                               dev_routes->first) )
        target_devs.push_back(dev_routes->first);
    }
  }

  std::vector<dev_id_t>::iterator curr_dev =
    target_devs.begin();;
  if (msg_type == MSG_ACK) {
    elka_msg_ack_s elka_msg;

    for (; curr_dev != target_devs.end(); curr_dev++) {
      get_elka_msg_id(&elka_msg.msg_id,
        _id, *curr_dev, _snd_params,
        msg_type, len);

      elka_msg.num_retries = num_retries;
      elka_msg.msg_num = msg_num;
      elka_msg.result = *data;

      push_msg(elka_msg, true);
    }
  } else {
    elka_msg_s elka_msg;
    
    for (; curr_dev != target_devs.end(); curr_dev++) {
      get_elka_msg_id(&elka_msg.msg_id,
        _id, *curr_dev, _snd_params,
        msg_type, len);

      // num_retries and msg_num not used if tx msg
      //elka_msg.num_retries = num_retries;
      //elka_msg.msg_num = msg_num;
      memcpy(elka_msg.data, data, len);

      push_msg(elka_msg, true);
    }
  }

  return msg_type;
}

uint8_t uart::UARTPort::set_dev_state_msg(
    elka_msg_s &elka_snd,
    dev_id_t dst_id,
    uint8_t state,
    bool elka_ctl) {
  uint8_t msg_t;

  if (elka_ctl) {
    msg_t = MSG_ELKA_CTL;
  } else {
    msg_t = MSG_PORT_CTL; 
  }

  set_state_msg(elka_snd,state,
      _id, dst_id,
      _snd_params, msg_t, MSG_STATE_LENGTH);

  return state;
}

// TODO eventually put this in common elka library
// TODO get rid of parameter, as _elka_snd should be set
// TODO publish or write serial depending upon context
// FIXME should check for serial connection, not device side
// TODO wireless link
uint8_t uart::UARTPort::send_msg(elka_msg_s &elka_msg) {
  dev_id_t rcv_id;
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, &rcv_id, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  // If this is an initial network building message, then
  // send in all possible ways
  if (broadcast_msg(rcv_id)) {
    write_elka_msg(_serial_fd, elka_msg);
    orb_publish(ORB_ID(elka_msg),
                _elka_msg_pub,
                &elka_msg);
  } else if (check_route(rcv_id) &&
          _routing_table[rcv_id].check_prop(DEV_PROP_ELKA_SIDE)) {
    write_elka_msg(_serial_fd, elka_msg);
  } else if (check_route(rcv_id)) {
    orb_publish(ORB_ID(elka_msg),
                _elka_msg_pub,
                &elka_msg);
  } else {
    return MSG_FAILED;
  }

  return msg_type;
}

uint8_t uart::UARTPort::send_msg(elka_msg_ack_s &elka_msg) {
  dev_id_t rcv_id;
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, &rcv_id, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  if (broadcast_msg(rcv_id)) {
    write_elka_msg(_serial_fd, elka_msg);
    orb_publish(ORB_ID(elka_msg),
                _elka_msg_pub,
                &elka_msg);
  } else if (check_route(rcv_id) &&
         _routing_table[rcv_id].check_prop(DEV_PROP_ELKA_SIDE)) {
    write_elka_msg(_serial_fd, elka_msg);
  } else if (check_route(rcv_id)) {
    orb_publish(ORB_ID(elka_msg_ack),
                _elka_ack_pub,
                &elka_msg);
  } else {
    return MSG_FAILED;
  }

  return msg_type;
}

uint8_t uart::UARTPort::parse_elka_msg(elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_ack) {
  struct elka_msg_id_s msg_id;
  uint8_t parse_res;
  bool in_route;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);

  // Check that device can be reached.
  // If message is not meant for you, then push it thru.
  // If message is meant for you, then check message type
  // to determine correct method of parsing message.
  if (( !(in_route = check_route(msg_id.rcv_id)) &&
        !broadcast_msg(msg_id.rcv_id) ) ||
      initial_msg(elka_msg.msg_id)) {
    return MSG_NULL;
  } else if (in_route &&
             cmp_dev_id_t(msg_id.rcv_id, _id) &&
             cmp_dev_id_t(msg_id.snd_id, _id)) {
    // Push message along if:
    //    It can be reached 
    //    It is not for you
    //    It is not from you (avoid creating cycle in graph)
    return push_msg(elka_msg, true);
  }

  switch(msg_id.type) {
    case MSG_MOTOR_CMD:
      parse_res = parse_motor_cmd(elka_msg, elka_ack, msg_id);
      break;
    case MSG_PORT_CTL:
      parse_res = parse_port_ctl(elka_msg, elka_ack, msg_id);
      break;
    case MSG_ELKA_CTL:
      parse_res = parse_elka_ctl(elka_msg, elka_ack, msg_id);
      break;
    case MSG_ROUTE_DEV_PROPS:
    case MSG_ROUTE_REQUEST_HB:
    case MSG_ROUTE_HB:
    case MSG_ROUTE_CHANGED:
    case MSG_ROUTE_TABLE:
      elka_msg_s ret_routing_msg;
      parse_res = parse_routing_msg(
          elka_msg, msg_id, elka_ack, ret_routing_msg);
      break;
    default:
      return MSG_FAILED;
      break;
  }

  // Send ack if necessary 
  if (parse_res & TYPE_EXPECTING_ACK) {
    push_msg(elka_ack, true);
  }

  return parse_res;
}

// Helper functions for parsing returned elka message based on current state
// @return msg type:
//         parse_motor_cmd always returns MSG_NULL on success
//                                          MSG_FAILED on failure
//         parse_port_ctl and parse_elka_ctl return:
//         MSG_FAILED if msg parsing failed
//         MSG_NULL if msg not for u
//         Can return all other types of msgs except for MSG_ACK
// Resumes elka if paused or started. Starts elka if stopped
// TODO add state as variable?
// FIXME not correct for this port
uint8_t uart::UARTPort::parse_motor_cmd(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  return MSG_FAILED;
}

uint8_t uart::UARTPort::parse_port_ctl(elka_msg_s &elka_msg,
                       elka_msg_ack_s &elka_ack,
                       struct elka_msg_id_s &msg_id) {
  uint8_t ret;
  get_elka_msg_id_attr(NULL, NULL, NULL, &ret, NULL,
      elka_msg.msg_id);
     
  elka_ack.msg_num = elka_msg.msg_num;

  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;
  elka_ack.num_retries = elka_msg.num_retries;
  return ret;
}

uint8_t uart::UARTPort::parse_elka_ctl(elka_msg_s &elka_msg,
                       elka_msg_ack_s &elka_ack,
                       struct elka_msg_id_s &msg_id) {
  uint8_t ret;
  get_elka_msg_id_attr(NULL, NULL, NULL, &ret, NULL,
      elka_msg.msg_id);
     
  elka_ack.msg_num = elka_msg.msg_num;

  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;
  elka_ack.num_retries = elka_msg.num_retries;

  return ret;
}

// Check ack with most recent sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t uart::UARTPort::check_ack(struct elka_msg_ack_s &elka_ack) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;
  uint8_t ret;

  get_elka_msg_id_attr(&msg_id, elka_ack.msg_id);

  // Check that ack is meant for this device
  if (!cmp_dev_id_t(msg_id.rcv_id, _id)) {
    return elka_msg_ack_s::ACK_NULL;
  } else {
    sb = _tx_buf;
  }

  // First check if message number has recently been acked
  // If message has recently been acked then
  // return elka_msg_ack_s::ACK_NULL
  // Else then check if message exists in buffer
  //    If message exists, then return check_elka_ack(...)
  //        Then add message number to list of recently acked messages
  //        Then remove message from buffer
  //    Else return elka_msg_ack_s::ACK_NULL
  if ((ret = sb->check_recent_acks(elka_ack.msg_num)) !=
              elka_msg_ack_s::ACK_NULL) {
    elka::ElkaBufferMsg *ebm;
    if ((ebm = sb->get_buffer_msg(
            elka_ack.msg_id, elka_ack.msg_num, false))) {
      if ( (ret = check_elka_ack(elka_ack,
                  ebm->_msg_id,
                  ebm->_rmv_msg_num,
                  ebm->_num_retries)) ==
           elka_msg_ack_s::ACK_FAILED) {
        PX4_WARN("Ack failed msg_id %d msg_num %d. %d retries",
            ebm->_msg_id, ebm->_rmv_msg_num,
            ebm->_num_retries);
      } else if (ret != elka_msg_ack_s::ACK_NULL) {
        // Message received and processed fine, so erase it
        PX4_INFO("erasing message");
        sb->erase_msg(elka_ack.msg_id, elka_ack.msg_num, false);
        sb->push_recent_acks(elka_ack.msg_num);
      }
    } else {
      ret = elka_msg_ack_s::ACK_NULL;
    }
  }

  return ret;
}

// Update _now variable with current time
void uart::UARTPort::update_time() {
  _now = hrt_absolute_time();
}

//-----------------Private Methods---------------------
bool uart::UARTPort::start_port() {
  _state = STATE_START;
  _state = STATE_RESUME;
  return true;
}

bool uart::UARTPort::stop_port() {
  _state = STATE_STOP;
  return true;
}

bool uart::UARTPort::pause_port() {
  _state = STATE_PAUSE;
  return true;
}

bool uart::UARTPort::resume_port() {
  _state = STATE_RESUME;
  return true;
}

