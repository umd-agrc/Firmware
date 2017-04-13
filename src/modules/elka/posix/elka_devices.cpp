#include <cstring>
#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>

#include "elka_devices.h"

//-----------------Public Methods----------------------
elka::DeviceNode::DeviceNode(uint8_t port_num, uint8_t port_type,
    uint8_t buf_type, char *dev_name) {
  if (!(init() == PX4_OK)) {
    PX4_ERR("Unable to initialize elka device %s",dev_name);
    errno = ECANCELED;
  } else {
    add_port(port_num,port_type,buf_type);
    strcpy(_dev_name, dev_name);
  }
}

elka::DeviceNode::~DeviceNode() {
  if (!(deinit() == PX4_OK)) {
    PX4_ERR("Unable to deinitialize elka device %s",
    _dev_name);
    errno = ECANCELED;
  };
}

int elka::DeviceNode::init() {
  for (int i=0; i < MAX_SERIAL_PORTS; i++) {
    _ports[i] = nullptr;
  }
  return PX4_OK;
}

uint8_t elka::DeviceNode::check_port(uint8_t port_num) {
  if (_ports[port_num]) {
    return (_ports[port_num]->_snd_id & PORT_TYPE) >> 1;
  }
  else {
    return PORT_NONE; 
  }
}

// For now: Do nothing with elka_ret.msg_num field
uint8_t elka::DeviceNode::parse_elka_msg(elka_msg_s &elka_ret,
    elka_msg_ack_s &elka_ack) {
  uint8_t type;
  struct elka_msg_id_s msg_id;
  struct snd_rcv_id_s ret_id;

  get_elka_msg_id_attr(&msg_id, elka_ret.msg_id);
  get_snd_rcv_id_attr(&ret_id, msg_id.snd_id);

  // Check that port exists.
  // Then check that message is meant for u.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(type = check_port(ret_id.port_num))) {
    PX4_WARN("ELKA msg being sent to non-existent port %d",
        ret_id.port_num);
    return MSG_NULL;
  } if (!(msg_id.snd_id == _ports[ret_id.port_num]->_rcv_id) ||
      !(msg_id.rcv_id == _ports[ret_id.port_num]->_snd_id)) {
    return MSG_NULL;
  }
  
  switch(msg_id.type) {
    case MSG_MOTOR_CMD:
      return parse_motor_cmd(elka_ret, elka_ack, msg_id, ret_id);
      break;
    case MSG_PORT_CTL:
      return parse_port_ctl(elka_ret, elka_ack, msg_id, ret_id);
      break;
    case MSG_ELKA_CTL:
      PX4_INFO("received elka ctl msg");
      return parse_elka_ctl(elka_ret, elka_ack, msg_id, ret_id);
      break;
    default:
      return MSG_FAILED;
      break;
  }
}

uint8_t elka::DeviceNode::parse_motor_cmd(elka_msg_s &elka_ret,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id,
                        struct snd_rcv_id_s &ret_id) {
  uint8_t state = _ports[ret_id.port_num]->_state;

  // Get ELKA to STATE_RESUME state if possible
  switch (state) {
    case STATE_START:
      _ports[ret_id.port_num]->resume_port();
      break;
    case STATE_STOP:
      _ports[ret_id.port_num]->start_port();
      break;
    case STATE_PAUSE:
      _ports[ret_id.port_num]->resume_port();
      break;
    case STATE_RESUME:
      break;
    default:
      break;
  }

  // Check state again to see that it is STATE_RESUME
  state = _ports[ret_id.port_num]->_state;
  if (state == STATE_RESUME) {
    if (_ports[ret_id.port_num]->_rx_buf->_type == UINT8_ARRAY) {  
      *(uint8_t *)_ports[ret_id.port_num]->_rx_buf->_buffer =
            msg_id.type;
      *((uint8_t *)(_ports[ret_id.port_num]->_rx_buf->_buffer)+1) =
            msg_id.length;
      memcpy((uint8_t *)(_ports[ret_id.port_num]->_rx_buf->_buffer) + 2,
        elka_ret.data, msg_id.length);

    } else if (_ports[ret_id.port_num]->_rx_buf->_type == CHAR_ARRAY) {
      *(uint8_t *)_ports[ret_id.port_num]->_rx_buf->_buffer =
            msg_id.type;
      *((uint8_t *)(_ports[ret_id.port_num]->_rx_buf->_buffer)+1) =
            msg_id.length;
      memcpy((uint8_t *)(_ports[ret_id.port_num]->_rx_buf->_buffer) + 2,
        elka_ret.data, msg_id.length);

    //TODO RINGBUF
    } else {
    }
  } else {
  }

  return MSG_NULL;
}

uint8_t elka::DeviceNode::parse_port_ctl(elka_msg_s &elka_ret,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id,
                        struct snd_rcv_id_s &ret_id) {

  elka_ack.msg_num = elka_ret.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      MSG_ACK, ACK_LEN);

  elka_ack.result = elka_msg_ack_s::ACK_FAILED;
  return MSG_FAILED; //TODO
}

uint8_t elka::DeviceNode::parse_elka_ctl(elka_msg_s &elka_ret,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id,
                        struct snd_rcv_id_s &ret_id) {
  elka_ack.msg_num = elka_ret.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      MSG_ACK, ACK_LEN);


  uint8_t nxt_state = elka_ret.data[0];
  switch (nxt_state) {
    case STATE_START:
      if (!_ports[ret_id.port_num]->start_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_STOP:
      if (!_ports[ret_id.port_num]->stop_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_PAUSE:
      if (!_ports[ret_id.port_num]->pause_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_RESUME:
      if (!_ports[ret_id.port_num]->resume_port()) {
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

uint8_t elka::DeviceNode::push_msg(elka_msg_s &elka_msg, bool tx) {
  void *buffer;
  uint8_t port_type, buf_type;
  struct elka_msg_id_s msg_id;
  struct snd_rcv_id_s ret_id;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);
  get_snd_rcv_id_attr(&ret_id, msg_id.snd_id);

  // Check that port exists.
  // Then check that message is meant for u.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(port_type = check_port(ret_id.port_num))) {
    return MSG_FAILED;
  }
  if (!(msg_id.snd_id == _ports[ret_id.port_num]->_rcv_id) ||
      !(msg_id.rcv_id == _ports[ret_id.port_num]->_snd_id)) {
    return MSG_NULL;
  }

  if (tx) { // use tx buf
    buffer = (void *) _ports[ret_id.port_num]->_tx_buf->_buffer;
    buf_type = _ports[ret_id.port_num]->_tx_buf->_type;
  } else { // use rx buf
    buffer = (void *) _ports[ret_id.port_num]->_rx_buf->_buffer;
    buf_type = _ports[ret_id.port_num]->_rx_buf->_type;
  }
 
  if (buf_type == CHAR_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)(buffer)+1) = msg_id.length;
    memcpy((uint8_t *)(buffer) + 2, elka_msg.data, msg_id.length);

  } else if (buf_type == UINT8_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)(buffer)+1) = msg_id.length;
    memcpy((uint8_t *)(buffer) + 2, elka_msg.data, msg_id.length);
    
  //TODO RINGBUF
  } else {
    return MSG_FAILED;
  }

  return msg_id.type;
}

// First byte is msg type. Second byte is msg length. Next bytes
// are data.
uint8_t elka::DeviceNode::push_msg(uint8_t port_num, uint8_t msg_type,
    uint8_t msg_len, void *data, bool tx) {
  void *buffer;
  uint8_t port_type, buf_type;
  
  // Check that port exists.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(port_type = check_port(port_num))) {
    return MSG_FAILED;
  }

  if (tx) { // use tx buf
    buffer = (void *) _ports[port_num]->_tx_buf->_buffer;
    buf_type = _ports[port_num]->_tx_buf->_type;
  } else { // use rx buf
    buffer = (void *) _ports[port_num]->_rx_buf->_buffer;
    buf_type = _ports[port_num]->_rx_buf->_type;
  }

  if (buf_type == UINT8_ARRAY) {
    *(uint8_t *)buffer = msg_type;
    *((uint8_t *)(buffer)+1) = msg_len;
    memcpy((uint8_t *)(_ports[port_num]->_tx_buf->_buffer) + 2,
      data, msg_len);

  } else if (buf_type == CHAR_ARRAY) {
    *(uint8_t *)buffer = msg_type;
    *((uint8_t *)(buffer)+1) = msg_len;
    memcpy((uint8_t *)(_ports[port_num]->_tx_buf->_buffer) + 2,
      data, msg_len);
  //TODO RINGBUF
  } else {
    return MSG_FAILED;
  }

  return msg_type;
}

// Message numbers must be incremented upon message removal b/c
// messages are sorted upon adding into buffers
bool elka::DeviceNode::remove_msg(uint8_t port_num,
    elka_msg_s &elka_msg, bool tx) {
  void *buffer;
  uint16_t *msg_num;
  uint8_t port_type, buf_type;

  // Check that port exists.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(port_type = check_port(port_num))) {
    return false;
  }

  // Set parameters to associate with the correct buffer
  if (tx) { // use tx buf
    buffer = (void *) _ports[port_num]->_tx_buf->_buffer;
    msg_num = &(_ports[port_num]->_tx_buf->_msg_num);
    buf_type = _ports[port_num]->_tx_buf->_type;
    elka_msg.msg_num = _ports[port_num]->_tx_buf->_msg_num++;
  } else { // use rx buf
    buffer = (void *) _ports[port_num]->_rx_buf->_buffer;
    msg_num = &(_ports[port_num]->_rx_buf->_msg_num);
    buf_type = _ports[port_num]->_rx_buf->_type;
    elka_msg.msg_num = _ports[port_num]->_rx_buf->_msg_num++;
  }

  if (buf_type == CHAR_ARRAY) {
    // First byte is msg type. Second byte is msg length. Next
    // bytes are data
    get_elka_msg_id(&elka_msg.msg_id,
        _ports[port_num]->_snd_id,
        _ports[port_num]->_rcv_id,
        *(uint8_t *)buffer, 
        *((uint8_t *)buffer + 1));

    memcpy(elka_msg.data,
      ((uint8_t *)buffer),
      *((uint8_t *)buffer + 1) + 2);

  } else if (buf_type == UINT8_ARRAY) {
    // First byte is msg type. Second byte is msg length. Next
    // bytes are data
    get_elka_msg_id(&elka_msg.msg_id,
        _ports[port_num]->_snd_id,
        _ports[port_num]->_rcv_id,
        *(uint8_t *)buffer, 
        *((uint8_t *)buffer + 1));

    memcpy(elka_msg.data,
      ((uint8_t *)buffer),
      *((uint8_t *)buffer + 1) + 2);

  } else { // ringbuf
    // Decrement msg_num bc msg will not be sent
    (*msg_num)--;
    return false;
  }   

  if (tx && (elka_msg.msg_id & ID_EXPECTING_ACK)) {
    //FIXME copy data
    _ports[port_num]->_expecting_ack[elka_msg.msg_num] = 
      std::pair(0,elka_msg.data);
  }

  return true;
}

// Check ack for sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t elka::DeviceNode::check_ack(struct elka_msg_ack_s &elka_ack) {
  uint32_t msg_id;
  uint8_t rcv_id, port_num, msg_type, msg_len, ret;

  get_elka_msg_id_attr(&rcv_id, NULL, &msg_type, &msg_len,
      elka_ack.msg_id);
  get_snd_rcv_id_attr(&port_num, NULL, NULL, rcv_id);

  // Check if port wants ack and ack message number is expected on
  // port
  // If not, then skip checking process
  if (check_port(port_num) == PORT_NONE ||
      _ports[port_num]->_expecting_ack.find(elka_ack.msg_num)
        == _ports[port_num]->_expecting_ack.end()) {
    return elka_msg_ack_s::ACK_NULL;
  }

  get_elka_msg_id(&msg_id,
    _ports[port_num]->_snd_id, _ports[port_num]->_rcv_id,
    msg_type, msg_len);

  // TODO handle ACK_DENIED, ACK_UNSUPPORTED, ACK_ACCEPTED separately
  // Because this method already ensures that a message exists with the
  // same message number, it is not necessary to send this device's
  // expected msg num to the check_elka_ack() method
  if ((ret = check_elka_ack(elka_ack, msg_id,elka_ack.msg_num)) !=
              elka_msg_ack_s::ACK_NULL) {
    if (ret == elka_msg_ack_s::ACK_FAILED){ // Message failed
      // Increment num retries
      _ports[port_num]->_expecting_ack[elka_ack.msg_num]++; 
    } else { // Message received and processed fine
      _ports[port_num]->_expecting_ack.erase(elka_ack.msg_num);
    }
  }
  
  return ret;
}

//TODO when struct buf *b->buffer is instantiated, point to new statically
// allocated buffer
// Can that data be stored anonymously and pointed to with a void pointer?
bool elka::DeviceNode::add_port(uint8_t port_num, uint8_t port_type, uint8_t buf_type) {
  _ports[port_num] = new SerialPort(port_num,port_type,buf_type);
  // Start port
  _ports[port_num]->start_port();
  return true;
}

bool elka::DeviceNode::delete_port(uint8_t port_num) {
  _ports[port_num]->stop_port();
  delete _ports[port_num];
  _ports[port_num] = nullptr;
  return true;
}

uint8_t elka::DeviceNode::get_port_state(uint8_t port_num) {
  if (check_port(port_num) != PORT_NONE)
    return _ports[port_num]->_state;
  else return STATE_NULL;
}

void elka::DeviceNode::update_time() {
  _now = hrt_absolute_time();
}

//-----------------Private Methods---------------------
int elka::DeviceNode::deinit() {
  for (int i=0; i < MAX_SERIAL_PORTS; i++) {
    delete_port(i);
  }

  return PX4_OK;
}

//-----------------SerialPort Methods---------------------

//TODO add in ringbuf support
elka::DeviceNode::SerialPort::SerialPort(uint8_t port_n, uint8_t port_t,
    uint8_t buf_t) {
  _tx_buf = new SerialBuffer(buf_t);
  _rx_buf = new SerialBuffer(buf_t);

  _state = STATE_STOP;

  // Set sender and receiver id for this port (1-1 relationship)
  get_snd_rcv_id(&_snd_id, &_rcv_id,
      port_n, port_t,
      POSIX_SIDE, QURT_SIDE);
}

elka::DeviceNode::SerialPort::~SerialPort() {
  delete _tx_buf;
  delete _rx_buf;
}

bool elka::DeviceNode::SerialPort::start_port() {
  _state = STATE_START;
  _state = STATE_RESUME;
  return true;
}

bool elka::DeviceNode::SerialPort::stop_port() {
  _state = STATE_STOP;
  return true;
}

bool elka::DeviceNode::SerialPort::pause_port() {
  _state = STATE_PAUSE;
  return true;
}

bool elka::DeviceNode::SerialPort::resume_port() {
  _state = STATE_RESUME;
  return true;
}

//-----------------SerialBuffer Methods---------------------

//TODO add in array cast
elka::DeviceNode::SerialBuffer::SerialBuffer(uint8_t buf_type) {
  _type = buf_type;

  _msg_num=0;

  if (buf_type == CHAR_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else if (buf_type == UINT8_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else {
    PX4_ERR("Unsupported buffer type for ELKA serial port");
    errno = EINVAL;
  }
}

elka::DeviceNode::SerialBuffer::~SerialBuffer() {
  if (_type == CHAR_ARRAY) {
    free(_buffer);
  } else if (_type == UINT8_ARRAY) {
    free(_buffer);
  } else {
    PX4_ERR("Unsupported buffer type for ELKA serial port");
    errno = EINVAL;
  }
}

