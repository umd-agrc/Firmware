#ifndef ELKA_COMM_H
#define ELKA_COMM_H

#include "elka.h"

#include <vector>
#include <map>

namespace elka {
  struct SerialBuffer;
  struct CommPort;
  struct DeviceRoute;
  struct ElkaBufferMsg;
}

struct elka::ElkaBufferMsg {
  msg_id_t _msg_id;
  // Num msg pushed to buffer
  // Useful for ordering messages when re-sorting buffer
  // Not the same as msg when removed from buffer
  uint16_t _push_msg_num; 
  // Num msg with which to check for ack
  uint16_t _rmv_msg_num; 
  uint8_t _data[MAX_MSG_LEN];
  uint8_t _num_retries;
  bool _expecting_ack;
  ElkaBufferMsg();
  /*
  ElkaBufferMsg(elka_msg_s &msg);
  ElkaBufferMsg(elka_msg_ack_s &msg);
  */
  ElkaBufferMsg(msg_id_t msg_id,
      uint16_t push_msg_num,
      uint16_t rmv_msg_num,
      uint8_t *data);
  ~ElkaBufferMsg();
  void clear_contents();
  uint8_t get_result();
};

struct elka::SerialBuffer {
  //TODO add ability to search _buffer for messages in the event that
  //_buffer is re-sorted when a higher priority message arrives
  //after the previous top message is sent and is expecting ack
  std::vector<ElkaBufferMsg> _buffer;
  // List of recently (successfully) acked messages to allow multiple
  // acks to be sent in the case that another instance of the message
  // is sent before the ack arrives
  //
  // Should be smaller than max message number
  uint16_t _recent_acks[RECENT_ACKS_LEN];
  // Write index for _recent_acks. Write like circular buffer
  uint16_t _recent_acks_end;
  // Buffer length for _recent_acks
  uint16_t _recent_acks_len;

  uint16_t _push_msg_num;
  uint16_t _rmv_msg_num;
  uint16_t _max_size;
private:
  uint8_t _type;
  dev_id_t _port_id;

public:
  SerialBuffer(dev_id_t port_id, uint8_t buf_type, uint16_t size);
  ~SerialBuffer();

  // Methods for recent_acks
  // Check if msg num has been (successfully) recently acked
  // Recent acks only need to be checked by msg_num bc msgs
  // from a buffer originating a message will not produce
  // duplicate message numbers. Duplicate message numbers will
  // only be present in buffers in the middle of a route.
  uint8_t check_recent_acks(uint16_t msg_num);
  void push_recent_acks(uint16_t msg_num);

  // Set message on buffer
  // Can be used to set messages returned or to send
  // @param elka_msg = message to push to buffer
  // @return msg_type if msg is pushed successfully 
  //         MSG_NULL if msg not meant for u
  //         MSG_FAILED if msg meant for u and incorrect
  //                    if msg is not pushed correctly
  uint8_t push_msg(elka_msg_ack_s &msg);
  uint8_t push_msg(elka_msg_s &msg);
  // This one assumes the message has not been retried
  uint8_t push_msg(msg_id_t msg_id, uint8_t *data);
  uint8_t push_msg(
      msg_id_t msg_id,
      uint8_t *data,
      uint16_t rmv_msg_num,
      uint8_t num_retries);

  // Retrieve message from buffer
  // Get front message
  // If tx, then use _rmv_msg_num index as outgoing _rmv_msg_num
  //    rx, then use _push_msg_num as outgoing _rmv_msg_num
  // @return msg type if msg is gotten
  //         MSG_NULL if msg is not gotten
  uint8_t get_msg(elka_msg_s &elka_msg,
                  elka_msg_ack_s &elka_msg_ack,
                  bool tx);

  // Get pointer to vector element referenced by snd_id and msg_num
  // @param msg_id = msg_id to use to get snd_id
  // @param msg_num = if passed, get message with _rmv_msg_num
  //                  equal to msg_num
  // @param tx = true if sending message
  //             false if receiving message
  // @return pointer to matching ElkaBufferMsg if element exists //         nullptr if element doesn't exist
  //         Note: Return value not valid if buffer is subsequently
  //         re-ordered
  ElkaBufferMsg *get_buffer_msg(msg_id_t msg_id, uint16_t msg_num,
      bool ack);

  // Retrieve message from buffer and set to elka_msg
  // Then remove message from buffer
  // @param elka_msg = message to set from buffer
  // @param bool tx = true if remove from tx buffer
  //                  false if remove from rx buffer
  // @return msg type if msg is removed
  //         MSG_NULL if msg is not removed
  uint8_t remove_msg(elka_msg_s &elka_msg,
                     elka_msg_ack_s &elka_msg_ack,
                     bool tx); 

  // Erase message at front of buffer
  // @return  MSG_NULL if msg is removed or buffer is empty
  //          MSG_FAILED if msg is not removed
  uint8_t pop_msg();

  // Erase specified message from buffer
  // @param msg_id = msg id of msg to remove
  //                 Used to identify correct sender
  // @param msg_num = message number to remove
  //                      if not set then pop front message
  // @param tx = true if sending message
  //             false if receiving message
  void erase_msg(msg_id_t msg_id, uint16_t msg_num, bool tx);

  // Check buffer front message type
  // Useful to determine whether message is elka_msg
  // or elka_msg_ack
  // @return msg_type
  uint8_t buffer_front_type();
  
private:
  class Compare {
  public:
    uint8_t msg_priority(uint8_t msg_type);
    bool operator() (ElkaBufferMsg p, ElkaBufferMsg q);
  };

  Compare _comp;
};

// Contains device id, route to the device, and the properties of the
// device
struct elka::DeviceRoute {
  DeviceRoute();
  // Constructor
  DeviceRoute(std::vector<dev_id_t> *route,
              std::vector<dev_prop_t> *props);
  // Copy constructor
  DeviceRoute(DeviceRoute *dr);

  std::vector<dev_id_t> _route;
  // Sorted so that it is easier to check equality
  std::vector<dev_prop_t> _props;
  bool _heartbeat; // true if alive, false if not
  bool _new_dev; // true if haven't received routing table
                // else false
  // Define route priority by comparing route sizes
  bool operator <(const DeviceRoute &rhs) const {
    return _route.size() < rhs._route.size();
  }

  void add_prop(dev_prop_t prop);
  void remove_prop(dev_prop_t prop);
  bool check_prop(dev_prop_t prop);
  bool check_alive();

  // Compare route to see which is preferable
  // Shorter route is preferable
  // @return -1 if current route is not preferable
  //         0 if routes are equal
  //         1 if current route is preferable
  static int8_t route_cmp(std::vector<dev_id_t> *r1,
                          std::vector<dev_id_t> *r2);

  // Compare properties to see if they are the same
  // @return true if same
  //         else false
  static bool dev_props_cmp(std::vector<dev_prop_t> *p1,
                            std::vector<dev_prop_t> *p2);

  static void print_dev_props(
      std::vector<dev_prop_t> &props);

  static void print_dev_route(
      std::vector<dev_id_t> &route);
};

// Encapsulates necessary aspects of communication to either PX4 ELKA or
// hardware ELKA
struct elka::CommPort {
  struct elka::SerialBuffer *_tx_buf;
  struct elka::SerialBuffer *_rx_buf;
  uint8_t _port_num;
  dev_id_t _id;
  // _id is included in _routing_table
  std::map<dev_id_t, DeviceRoute, dev_id_tCmp> _routing_table;
  uint8_t _snd_params;
  uint8_t _state;
  CommPort(uint8_t port_n, uint8_t port_t, uint8_t buf_type,
      uint8_t size);
  // virtual destructor so that derived class destructor is called.
  virtual ~CommPort(); 
  virtual bool start_port() = 0;
  virtual bool stop_port() = 0;
  virtual bool pause_port() = 0;
  virtual bool resume_port() = 0;
  
  //TODO add push_msg(msg_id_t msg_id, uint8_t *data, bool tx)  overload
  //     so that elka_msg_s doesn't need to be formed each time from
  //     send_msg()
  // Set message on buffer
  // Can be used to set messages returned or to send
  // @param elka_msg = message to push to buffer
  // @param bool tx = true if push to tx buffer
  //                  false if push to rx buffer
  // @return msg_type if msg is pushed successfully 
  //         MSG_NULL if msg not meant for u in the case of tx=false
  //         MSG_FAILED if msg meant for u and incorrect
  //                    if msg is not pushed correctly
  uint8_t push_msg(dev_id_t &dst, uint8_t msg_type,
                   uint8_t len, uint8_t *data);
  uint8_t push_msg(elka_msg_s &elka_msg, bool tx);
  uint8_t push_msg(elka_msg_ack_s &elka_msg, bool tx);

  // Retrieve next message from buffer and set to elka_msg
  // @param port_num = port number to use
  // @param elka_msg = message to set from buffer
  // @param rmv_msg_num = retrieve message with specific msg num
  //                      negative values mean remove from front
  // @param bool tx = true if remove from tx buffer
  //                  false if remove from rx buffer
  uint8_t get_msg(elka_msg_s &elka_msg,
                  elka_msg_ack_s &elka_msg_ack,
                  bool tx);

  // Retrieve message from buffer and set to elka_msg
  // Then, remove message from buffer
  // @param elka_msg = message to set from buffer
  // @param bool tx = true if remove from tx buffer
  //                  false if remove from rx buffer
  uint8_t remove_msg(elka_msg_s &elka_msg,
                     elka_msg_ack_s &elka_msg_ack,
                     bool tx); 

  // Removes message from front of buffer
  uint8_t pop_msg(bool tx); 



  //FIXME currently depends on dev_id_t being 2B
  //FIXME currently depends on dev_prop_t being 1B
  // Pushes ret_routing_msg if response is requested
  uint8_t parse_routing_msg(
    elka_msg_s &elka_msg,
    struct elka_msg_id_s &msg_id,
    elka_msg_ack_s &elka_ack,
    elka_msg_s &ret_routing_msg);

  // Routing table functions:

  // Compare route to see which is preferable
  // Shorter route is preferable
  // @return -1 if current route is not preferable
  //         0 if routes are equal
  //         1 if current route is preferable
  int8_t route_cmp(dev_id_t &d, std::vector<dev_id_t> &r1);

  // @return true if current properties of d are same as p1
  bool dev_props_cmp(dev_id_t &d, std::vector<dev_prop_t> &p1);

  // @return true if route to dst contains el
  //         else false
  bool check_route_contains(dev_id_t &dst, dev_id_t &el);

  // @return true if device exists in _routing_table
  //          and is alive
  //         false if not
  bool check_route(dev_id_t &d);

  // Changes route in _routing_table
  // @param dev = device to change
  //              if not in _routing_table then add route
  // @param dr = new DeviceRoute
  //             if NULL then remove route
  void change_route(dev_id_t &dev, DeviceRoute *dr);

  // Change route in _routing_table. This can be used
  // to target changing only _route or _props while leaving
  // the other alone
  // @param dev = device to change
  //              if not in _routing_table then add route
  // @param route = _route of dev
  // @param props = _props of dev
  void change_route(dev_id_t &dev,
                    std::vector<dev_id_t> *route,
                    std::vector<dev_prop_t> *props);

  // Get next device to send to in route
  // @param end = Endpoint device
  // @param nxt = Set in this function
  //              Next device id to send to
  //              Set to NULL if device can't be reached
  //              or if device is this device
  // @return true if this device is the endpoint
  //         else false
  bool get_next_dev(dev_id_t &end, dev_id_t *nxt);

  //FIXME currently depends on dev_prop_t being 1B
  //      and dev_id_t being 2B
  // Set routing table from data array
  static int16_t parse_route_table(
      uint8_t len,
      uint8_t *data,
      std::map<dev_id_t, DeviceRoute, dev_id_tCmp> &routing_table);

  //FIXME currently depends on dev_prop_t being 1B
  // Set device properties from data array
  static int16_t parse_dev_props(
      uint8_t len,
      uint8_t *data,
      std::vector<dev_prop_t> &props);

  //FIXME currently depends on dev_id_t being 2B
  // Set device route from data array
  static int16_t parse_dev_route(
      uint8_t len,
      uint8_t *data,
      std::vector<dev_id_t> &route);

  //FIXME currently depends on dev_prop_t being 1B
  void set_dev_props_msg(
      dev_id_t snd_id,
      dev_id_t rcv_id,
      bool req_resp,
      elka_msg_s &ret_routing_msg);

  //FIXME currently depends on dev_id_t being 2B
  void set_route_changed_msg(
      dev_id_t &snd_id,
      dev_id_t &rcv_id,
      bool req_resp,
      elka_msg_s &ret_routing_msg);

  //FIXME currently depends on dev_id_t being 2B
  void set_route_table_msg(
      dev_id_t &snd_id,
      dev_id_t &rcv_id,
      bool req_resp,
      elka_msg_s &ret_routing_msg);

  uint16_t set_dev_msg_part(
      dev_id_t dst,
      uint8_t *data);

  // Check if device should receive message of msg_type
  bool check_dev_compatible(uint8_t msg_type,
                       dev_id_t dst);
  

  static void print_elka_route_msg(elka_msg_s &elka_msg);
  static void print_routing_table(
      std::map<dev_id_t,DeviceRoute, dev_id_tCmp> &routing_table);
};



#endif
