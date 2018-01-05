#include <cstring>
#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>

#include "elka_devices.h"

//-----------------Public Methods----------------------
elka::PX4Port::PX4Port(uint8_t port_num, uint8_t port_type,
    uint8_t buf_type, uint8_t size, char *dev_name) 
    : elka::CommPort(port_num, port_type, buf_type, size) {
  if (!(init() == PX4_OK)) {
    PX4_ERR("Unable to initialize elka device %s",dev_name);
    errno = ECANCELED;
  } else {
    strcpy(_dev_name, dev_name);

    get_snd_params(&_snd_params, port_num, port_type,
      DEV_PROP_POSIX_SIDE);
  }
}

elka::PX4Port::~PX4Port() {
  if (!(deinit() == PX4_OK)) {
    PX4_ERR("Unable to deinitialize elka device %s",
    _dev_name);
    errno = ECANCELED;
  };

  delete _tx_buf;
  delete _rx_buf;
}

int elka::PX4Port::init() {
  memset(&_elka_ack_snd, 0, sizeof(_elka_ack_snd));
  memset(&_elka_ack_rcv, 0, sizeof(_elka_ack_rcv));
  memset(&_elka_snd, 0, sizeof(_elka_snd));
  memset(&_elka_ret, 0, sizeof(_elka_ret));
  memset(&_elka_ret_cmd, 0, sizeof(_elka_ret_cmd));
  
  // Advertise attitude topic
  _elka_msg_pub = orb_advertise(
      ORB_ID(elka_msg), &_elka_snd);
  _elka_ack_pub = orb_advertise(
      ORB_ID(elka_msg_ack), &_elka_ack_snd);
  
  //orb_advert_t elka_msg_pub = orb_advertise(
  //    ORB_ID(elka_msg), NULL);
  //orb_advert_t elka_ack_pub = orb_advertise(
  //    ORB_ID(elka_msg_ack), NULL);

  // Add device properties
  _routing_table[_id].add_prop(DEV_PROP_POSIX_SIDE);
  _routing_table[_id].add_prop(DEV_PROP_PERFORM_LOCALIZATION);
  _routing_table[_id].add_prop(DEV_PROP_SENSE_LOCATION);
  _routing_table[_id].add_prop(DEV_PROP_SPIN_MOTORS);
  _routing_table[_id].add_prop(DEV_PROP_USE_CAMERA);
  _routing_table[_id].add_prop(DEV_PROP_HAS_CAMERA);

  start_port();

  return PX4_OK;
}

// msg_num and num_retries are useful parameters only
// with MSG_ACK.
// In all other cases these parameters are handled upon
// message pushing/getting
uint8_t elka::PX4Port::add_msg(
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
    std::map<dev_id_t, DeviceRoute, dev_id_tCmp>::iterator
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

// TODO eventually put this in common elka library
// TODO get rid of parameter, as _elka_snd should be set
uint8_t elka::PX4Port::send_msg(elka_msg_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  orb_publish(ORB_ID(elka_msg),
              _elka_msg_pub,
              &elka_msg);

  // write to socket 
  /*  
  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      CLIENT);

  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      SERVER);
  */

  return msg_type;
}

uint8_t elka::PX4Port::send_msg(elka_msg_ack_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  orb_publish(ORB_ID(elka_msg_ack),
              _elka_ack_pub,
              &elka_msg);

  // write to socket 
  /*  
  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      CLIENT);

  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      SERVER);
  */

  return msg_type;
}

// For now: Do nothing with elka_msg.msg_num field
uint8_t elka::PX4Port::parse_elka_msg(elka_msg_s &elka_msg) {
  elka_msg_ack_s elka_ack;
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

uint8_t elka::PX4Port::parse_motor_cmd(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t state = _state;

  // Get ELKA to STATE_RESUME state if possible
  switch (state) {
    case STATE_START:
      resume_port();
      break;
    case STATE_STOP:
      start_port();
      break;
    case STATE_PAUSE:
      resume_port();
      break;
    case STATE_RESUME:
      break;
    default:
      break;
  }

  // Check state again to see that it is STATE_RESUME
  state = _state;
  if (state == STATE_RESUME) {
    if (push_msg(elka_msg, false)) {
      return MSG_NULL;
    }
  } else {
    return MSG_FAILED;
  }

  return msg_id.type;
}

uint8_t elka::PX4Port::parse_port_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {

  elka_ack.msg_num = elka_msg.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  elka_ack.result = elka_msg_ack_s::ACK_FAILED;
  return MSG_FAILED; //TODO
}

uint8_t elka::PX4Port::parse_elka_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  elka_ack.msg_num = elka_msg.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  uint8_t nxt_state = elka_msg.data[0];
  switch (nxt_state) {
    case STATE_START:
      if (!start_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_STOP:
      if (!stop_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_PAUSE:
      if (!pause_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_RESUME:
      if (!resume_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    default:
      elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;
      return MSG_FAILED;
      break;
  }

  elka_ack.result = elka_msg_ack_s::ACK_ACCEPTED;

  return msg_id.type;
}

// Check ack for sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t elka::PX4Port::check_ack(struct elka_msg_ack_s &elka_ack) {
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
    ElkaBufferMsg *ebm;
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

uint8_t elka::PX4Port::get_state() {
  return _state;
}

uint8_t elka::PX4Port::set_dev_state_msg(
    elka_msg_s &elka_snd,
    dev_id_t rcv_id,
    uint8_t state,
    bool elka_ctl) {
  uint8_t msg_t;
  if (elka_ctl) {
    msg_t = MSG_ELKA_CTL;
  } else {
    msg_t = MSG_PORT_CTL;
  }

  set_state_msg(elka_snd, state,
                _id, rcv_id,
                _snd_params, msg_t, 1);
  return state;
}

void elka::PX4Port::update_time() {
  _now = hrt_absolute_time();
}

//-----------------Private Methods---------------------
void elka::PX4Port::wait_for_child(Child *child) {
	int pid, status;

	while ((pid = waitpid(-1, &status, 0)) != -1) {
		if (pid == child->pid) {
			child->pid = -1;
		}
	}

	// Check loop not necessary
	if (child->pid != -1) {
		PX4_ERR("Child %d died without being tracked", (int)child->pid);
	}
}

int elka::PX4Port::deinit() {
  _routing_table.clear();
  stop_port();
  return PX4_OK;
}

bool elka::PX4Port::start_port() {
  _state = STATE_START;

  //FIXME determine client or server programattically
  // For client
  /*
  socket_proc_start(
      &_inet_proc,
      "192.168.1.1",
      CLIENT,
      _tx_buf,
      _rx_buf);
  
  // For server
  socket_proc_start(
      &_inet_proc,
      NULL,
      SERVER,
      _tx_buf,
      _rx_buf);
  */

  resume_port();

  return true;
}

bool elka::PX4Port::stop_port() {
  _state = STATE_STOP;
  
  //wait_for_child(&_inet_proc);

  return true;
}

bool elka::PX4Port::pause_port() {
  _state = STATE_PAUSE;
  return true;
}

bool elka::PX4Port::resume_port() {
  _state = STATE_RESUME;
  return true;
}
