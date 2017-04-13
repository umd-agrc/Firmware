#include <cstring>
#include <errno.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>

#include "snapdragon_uart_devices.h"

//-----------------Public Methods----------------------
uart::DeviceNode::DeviceNode(uint8_t port_num, char *dev_name, uint8_t buf_t) {
  _elka_comm = new CommPort(port_num, buf_t, ELKA_SIDE);
  _px4_comm = new CommPort(port_num, buf_t, POSIX_SIDE);
  strcpy(_dev_name, dev_name);

  if (!(init() == PX4_OK)) {
    PX4_ERR("Unable to initialize elka device %s",dev_name);
    errno = ECANCELED;
  } else {
  }
}

uart::DeviceNode::~DeviceNode() {
  delete _elka_comm;
  delete _px4_comm;

  if (deinit() != PX4_OK) {
    PX4_ERR("Unable to deinitialize elka device with %s",
        _dev_name);
  };
}

// Open UART port
// @return PX4_OK on success, else PX4_ERROR
int uart::DeviceNode::init() {
  if (open() == PX4_OK) {
    PX4_INFO("Initialized Snapdragon UART device to communicate on port %s",
        _dev_name);
    return PX4_OK;
  } else {
    return PX4_ERROR;
  }
}

// Close port access
// @return PX4_OK on success, else PX4_ERROR
int uart::DeviceNode::deinit() {
  if (close() == PX4_OK)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::open() {
  if ((_serial_fd = serial_open(_elka_comm->_port_num,
          _elka_comm->_tx_buf->_buffer, _elka_comm->_tx_buf->_type,
          _elka_comm->_rx_buf->_buffer, _elka_comm->_rx_buf->_type))
      > 0 )
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::close() {
  if (!(_serial_fd = serial_close(_serial_fd, _elka_comm->_port_num)))
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::assign_read_callback() {
  if (assign_serial_read_callback(_serial_fd, _elka_comm->_port_num)
      == _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

/*
int uart::DeviceNode::read(elka_msg_s &elka_ret) {
  if (_rx_buf->type == CHAR_ARRAY) {
    // First byte is type, second byte is length
    get_elka_msg_id(&elka_ret.msg_id,
        _snd_id, _rcv_id,
        *((char *)_rx_buf->_buffer),
        *((char *)(_rx_buf->_buffer+1));
    // Copy length + 2*element_size bytes to _rx_buffer
    memcpy(_rx_buf->_buffer, elka_ret.data,
        *((char *)(_rx_buf->_buffer+1) +
          2*sizeof(*(char *)_rx_buf->_buffer));
  } else if (_rx_buf->type == UINT8_ARRAY) {
    // First byte is type, second byte is length
    get_elka_msg_id(&elka_ret.msg_id,
        _snd_id, _rcv_id,
        *((uint8_t *)_rx_buf->_buffer),
        *((uint8_t *)(_rx_buf->_buffer+1));
    // Copy length + 2*element_size bytes to _rx_buffer
    memcpy(_rx_buf->_buffer, elka_ret.data,
        *((uint8_t *)(_rx_buf->_buffer+1) +
          2*sizeof(*(uint8_t *)_rx_buf->_buffer));
  } else {
    return PX4_ERROR;
  }
  return PX4_OK;
}

int uart::DeviceNode::write(elka_msg_s &elka_snd) {
  uint8_t msg_type, msg_length;
  get_elka_msg_id_attr(NULL, NULL,
                       &msg_type, &msg_length,
                       elka_snd.msg_id);

  //Write to _tx_buffer
  memset(_tx_buf->_buffer, msg_type, sizeof(msg_type));
  memset(_tx_buf->_buffer + sizeof(msg_type),
      msg_length, sizeof(msg_length));
  memcpy(_tx_buf->_buffer + sizeof(msg_type) + sizeof(msg_length),
      elka_snd.data,msg_length);

  if (serial_write(_serial_fd, _port_num,
        _tx_buf->_buffer,
        msg_length + sizeof(msg_type) +
        sizeof(msg_length)) ==
      _serial_fd)
    return PX4_OK;
  else return PX4_ERROR;
}

int uart::DeviceNode::read_write(elka_msg_s &elka_ret, elka_msg_s &elka_snd) {
  write(elka_snd);
  read(elka_ret);

  return PX4_OK;
}

*/

uint8_t uart::DeviceNode::check_port(uint8_t port_num, bool px4) {
  if (px4 && _px4_comm->_port_num == port_num) {
    return (_px4_comm->_snd_id & PORT_TYPE) >> 1;
  } else if (!px4 && _elka_comm->_port_num == port_num) {
    return (_elka_comm->_snd_id & PORT_TYPE) >> 1;
  } else {
    return PORT_NONE; 
  }
}

uint8_t uart::DeviceNode::push_msg(elka_msg_s &elka_msg, bool px4, bool tx) {
  void *buffer;
  uint8_t buf_type, snd_id, rcv_id;
  struct elka_msg_id_s msg_id;
  struct snd_rcv_id_s ret_id;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);
  get_snd_rcv_id_attr(&ret_id, msg_id.snd_id);

  if (px4 && tx) { // use tx buf
    buffer = (void *) _px4_comm->_tx_buf->_buffer;
    buf_type = _px4_comm->_tx_buf->_type;
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else if (px4 && !tx) { // use rx buf
    buffer = (void *) _px4_comm->_rx_buf->_buffer;
    buf_type = _px4_comm->_rx_buf->_type;
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else if (!px4 && tx) {
    buffer = (void *) _elka_comm->_tx_buf->_buffer;
    buf_type = _elka_comm->_tx_buf->_type;
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  } else {
    buffer = (void *) _elka_comm->_rx_buf->_buffer;
    buf_type = _elka_comm->_rx_buf->_type;
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  }

  // Then check that message is meant for u.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(msg_id.snd_id == rcv_id) ||
      !(msg_id.rcv_id == snd_id)) {
    return MSG_NULL;
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

//TODO parse different message types
//     MSG_PORT_CTL requires ack from this device
//     MSG_ELKA_CTL requires ack from Elka FCU
bool uart::DeviceNode::remove_msg(elka_msg_s &elka_msg,
    bool px4, bool tx) {
  void *buffer;
  uint16_t *msg_num;
  uint8_t buf_type, snd_id, rcv_id;
  std::map<uint16_t,uint8_t> *expecting_ack;

  // Set parameters to associate with the correct buffer
  if (px4 && tx) { // use px4 tx buf
    buffer = (void *) _px4_comm->_tx_buf->_buffer;
    msg_num = &(_px4_comm->_tx_buf->_msg_num);
    buf_type = _px4_comm->_tx_buf->_type;
    expecting_ack = &(_px4_comm->_expecting_ack);
    elka_msg.msg_num = _px4_comm->_tx_buf->_msg_num++;
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else if (px4 && !tx) { // use px4 rx buf
    buffer = (void *)(_px4_comm->_rx_buf->_buffer);
    msg_num = &(_px4_comm->_rx_buf->_msg_num);
    buf_type = _px4_comm->_rx_buf->_type;
    expecting_ack = &(_px4_comm->_expecting_ack);
    elka_msg.msg_num = _px4_comm->_rx_buf->_msg_num++;
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else if (!px4 && tx) { // use elka tx buf
    buffer = (void *)(_elka_comm->_tx_buf->_buffer);
    msg_num = &(_elka_comm->_tx_buf->_msg_num);
    buf_type = _elka_comm->_tx_buf->_type;
    expecting_ack = &(_elka_comm->_expecting_ack);
    elka_msg.msg_num = _elka_comm->_tx_buf->_msg_num++;
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  } else { // use elka rx buf
    buffer = (void *)(_elka_comm->_rx_buf->_buffer);
    msg_num = &(_elka_comm->_rx_buf->_msg_num);
    buf_type = _elka_comm->_rx_buf->_type;
    expecting_ack = &(_elka_comm->_expecting_ack);
    elka_msg.msg_num = _elka_comm->_rx_buf->_msg_num++;
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  }

  if (buf_type == CHAR_ARRAY) {
    // First byte is msg type. Second byte is msg length. Next
    // bytes are data
    get_elka_msg_id(&elka_msg.msg_id,
        snd_id, rcv_id,
        *(uint8_t *)buffer, 
        *((uint8_t *)buffer + 1));

    memcpy(elka_msg.data,
      ((uint8_t *)buffer),
      *((uint8_t *)buffer + 1) + 2);

  } else if (buf_type == UINT8_ARRAY) {
    // First byte is msg type. Second byte is msg length. Next
    // bytes are data
    get_elka_msg_id(&elka_msg.msg_id,
        snd_id, rcv_id,
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
    (*expecting_ack)[elka_msg.msg_num] = 0;
  }

  return true;
}

void uart::DeviceNode::set_state_msg(elka_msg_s &elka_snd, uint8_t state, bool px4) {
  uint16_t *msg_num;
  uint8_t snd_id, rcv_id;
  std::map<uint16_t,uint8_t> *expecting_ack;

  if (px4) { // use px4 tx buf
    msg_num = &(_px4_comm->_tx_buf->_msg_num);
    expecting_ack = &(_px4_comm->_expecting_ack);
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else { // use px4 rx buf
    msg_num = &(_elka_comm->_rx_buf->_msg_num);
    expecting_ack = &(_elka_comm->_expecting_ack);
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  } 

  // Set msg_id
  get_elka_msg_id(&elka_snd.msg_id,
      snd_id, rcv_id,
      MSG_ELKA_CTL,
      1);

  elka_snd.msg_num = *msg_num++;
  elka_snd.data[0] = state;
  elka_snd.timestamp = _now;
}

uint8_t uart::DeviceNode::parse_elka_msg(elka_msg_s &elka_ret,
    elka_msg_ack_s &elka_ack, bool px4) {
  uint8_t type, snd_id, rcv_id;
  struct elka_msg_id_s msg_id;
  struct snd_rcv_id_s ret_id;

  get_elka_msg_id_attr(&msg_id, elka_ret.msg_id);
  get_snd_rcv_id_attr(&ret_id, msg_id.snd_id);

  if (px4) {
    snd_id = _px4_comm->_snd_id;
    rcv_id = _px4_comm->_rcv_id;
  } else {
    snd_id = _elka_comm->_snd_id;
    rcv_id = _elka_comm->_rcv_id;
  }

  // Then check that message is meant for u.
  // Then check message type to determine correct method of parsing
  // message.
  if (!(type = check_port(ret_id.port_num, px4))) {
    PX4_WARN("ELKA msg being sent to non-existent port %d",
        ret_id.port_num);
    return MSG_NULL;
  } else if (!(msg_id.snd_id == rcv_id) ||
      !(msg_id.rcv_id == snd_id)) {
    return MSG_NULL;
  }
  
  switch(msg_id.type) {
    case MSG_MOTOR_CMD:
      return parse_motor_cmd(elka_ret, elka_ack, msg_id, ret_id, px4);
      break;
    case MSG_PORT_CTL:
      return parse_port_ctl(elka_ret, elka_ack, msg_id, ret_id, px4);
      break;
    case MSG_ELKA_CTL:
      return parse_elka_ctl(elka_ret, elka_ack, msg_id, ret_id, px4);
      break;
    default:
      return MSG_FAILED;
      break;
  }

}

// Helper functions for parsing returned elka message based on current state
// @return msg type:
//         parse_motor_cmd() always returns MSG_NULL
//         parses_port_ctl and parse_elka_ctl return:
//         MSG_FAILED if msg parsing failed
//         MSG_NULL if msg not for u
//         Can return all other types of msgs except for MSG_ACK
// Resumes elka if paused or started. Starts elka if stopped
// TODO add state as variable?
uint8_t uart::DeviceNode::parse_motor_cmd(elka_msg_s &elka_ret,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id,
                        struct snd_rcv_id_s &ret_id,
                        bool px4) {
  uint8_t type;
  void *buffer;

  if (px4) {
    type = _elka_comm->_rx_buf->_type;
    // Send motor command to elka
    buffer = (void *) _elka_comm->_rx_buf->_buffer;
  } else {
    type = _px4_comm->_rx_buf->_type;
    // Send motor command to px4
    buffer = (void *) _px4_comm->_rx_buf->_buffer;
  }

  if (type == UINT8_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)buffer + 1) = msg_id.length;
    memcpy(((uint8_t *)buffer + 2),
      elka_ret.data,
      msg_id.length);
  } else if (type == CHAR_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)buffer + 1) = msg_id.length;
    memcpy(((uint8_t *)buffer + 2),
      elka_ret.data,
      msg_id.length);
      
  //TODO RINGBUF
  } else {
    return MSG_FAILED;
  }

  return MSG_NULL;
}

uint8_t uart::DeviceNode::parse_port_ctl(elka_msg_s &elka_ret,
                       elka_msg_ack_s &elka_ack,
                       struct elka_msg_id_s &msg_id,
                       struct snd_rcv_id_s &ret_id,
                       bool px4) {
  uint8_t ret;
  get_elka_msg_id_attr(NULL, NULL, &ret, NULL, elka_ret.msg_id);
     
  elka_ack.msg_num = elka_ret.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      MSG_ACK, ACK_LEN);

  elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;
  return ret;
}

uint8_t uart::DeviceNode::parse_elka_ctl(elka_msg_s &elka_ret,
                       elka_msg_ack_s &elka_ack,
                       struct elka_msg_id_s &msg_id,
                       struct snd_rcv_id_s &ret_id,
                       bool px4) {
  uint8_t type;
  void *buffer;

  if (px4) {
    type = _elka_comm->_rx_buf->_type;
    // Send motor command to elka
    buffer = (void *) _elka_comm->_rx_buf->_buffer;
  } else {
    type = _px4_comm->_rx_buf->_type;
    // Send motor command to px4
    buffer = (void *) _px4_comm->_rx_buf->_buffer;
  }

  if (type == UINT8_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)buffer + 1) = msg_id.length;
    memcpy(((uint8_t *)buffer + 2),
      elka_ret.data,
      msg_id.length);
  } else if (type == CHAR_ARRAY) {
    *(uint8_t *)buffer = msg_id.type;
    *((uint8_t *)buffer + 1) = msg_id.length;
    memcpy(((uint8_t *)buffer + 2),
      elka_ret.data,
      msg_id.length);
      
  //TODO RINGBUF
  } else {
  }

  return MSG_NULL;
}

// Check ack for sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t uart::DeviceNode::check_ack(struct elka_msg_ack_s &elka_ack,
    bool px4) {
  uint32_t msg_id;
  uint8_t snd_id, rcv_id, port_num, msg_type, msg_len, ret;
  std::map<uint16_t,uint8_t> *expecting_ack;

  if (px4) { // use px4 tx buf
    expecting_ack = &(_px4_comm->_expecting_ack);
    snd_id = _px4_comm->_snd_id;
  } else { // use px4 rx buf
    expecting_ack = &(_elka_comm->_expecting_ack);
    snd_id = _elka_comm->_snd_id;
  } 

  get_elka_msg_id_attr(&rcv_id, NULL, &msg_type, &msg_len,
      elka_ack.msg_id);
  get_snd_rcv_id_attr(&port_num, NULL, NULL, rcv_id);

  // Check if port wants ack and ack message number is expected on
  // port
  // If not, then skip checking process
  if (expecting_ack->find(elka_ack.msg_num)
        == expecting_ack->end()) {
    return elka_msg_ack_s::ACK_NULL;
  }

  get_elka_msg_id(&msg_id,
    snd_id, rcv_id,
    msg_type, msg_len);

  // TODO handle ACK_DENIED, ACK_UNSUPPORTED, ACK_ACCEPTED separately
  // Because this method already ensures that a message exists with the
  // same message number, it is not necessary to send this device's
  // expected msg num to the check_elka_ack() method
  if ((ret = check_elka_ack(elka_ack, msg_id,elka_ack.msg_num)) !=
              elka_msg_ack_s::ACK_NULL) {
    if (ret == elka_msg_ack_s::ACK_FAILED){ // Message failed
      // Increment num retries
      (*expecting_ack)[elka_ack.msg_num]++; 
    } else { // Message received and processed fine
      expecting_ack->erase(elka_ack.msg_num);
    }
  }
  
  return ret;
}


// Update _now variable with current time
void uart::DeviceNode::update_time() {
  _now = hrt_absolute_time();
}

//-----------------SerialBuffer Methods---------------------

//TODO add in array cast
uart::DeviceNode::SerialBuffer::SerialBuffer(uint8_t buf_type) {
  _type = buf_type;

  _msg_num = 0;

  if (buf_type == CHAR_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else if (buf_type == UINT8_ARRAY) {
    _buffer = malloc(MAX_MSG_LEN);
  } else {
    PX4_ERR("Unsupported buffer type for Snapdragon UART port");
    errno = EINVAL;
  }
}

uart::DeviceNode::SerialBuffer::~SerialBuffer() {
  if (_type == CHAR_ARRAY) {
    free(_buffer);
  } else if (_type == UINT8_ARRAY) {
    free(_buffer);
  } else {
    PX4_ERR("Unsupported buffer type for Snapdragon UART port");
    errno = EINVAL;
  }
}

//-----------------CommPort Methods---------------------

uart::DeviceNode::CommPort::CommPort(uint8_t port_num,
    uint8_t buf_type, uint8_t proc_side) {
  _port_num = port_num;
  _proc_side = proc_side;
  _tx_buf = new SerialBuffer(buf_type);
  _rx_buf = new SerialBuffer(buf_type);

  // Set sender and receiver id for this port (1-1 relationship)
  get_snd_rcv_id(&_snd_id, &_rcv_id,
      port_num, PORT_UART,
      QURT_SIDE, proc_side);

}

uart::DeviceNode::CommPort::~CommPort() {
  delete _tx_buf;
  delete _rx_buf;
}
