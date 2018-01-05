#pragma once

#include <map>
#include <set>
#include <cstring>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_packet.h>

#include "serial_defines.h"
#include "basic_messenger.h"
#include "basic_uart.h"

// Define messenger types
#define MESSENGER_SERIAL 0

// Define message destinations
#define DST_ELKA 0
#define DST_SNAP 1
#define DST_XBEE 2

namespace elka {
  // Singleton class for managing messages
  class BasicMessageMgr;
}
  
class elka::BasicMessageMgr {
private:
  BasicMessageMgr(){_cnt=0;};
  BasicMessageMgr(BasicMessageMgr const &){};
  //BasicMessageMgr &operator=(BasicMessageMgr const &){};
  static BasicMessageMgr *_inst;

  std::map<int,BasicMessenger*> _messengers;
  uint8_t _cnt;
  // Mapping between integer descriptors (ides) and BasicMessengers
  // Key is descriptor of destination. Value is BasicMessenger to
  // use
  //std::map<int,BasicMessenger> _routes;

  // Queue of messages received and message sorting protocols
  // Theory: Messages from unimportant sources should expire
  //         May result in lost connection. This is ok given the
  //         computational limits
  uint8_t _max_num_msgs=5; //TODO constructor to set max num msgs
  static uint8_t elka_msg_priority(const elka_msg_s &msg) {
    uint8_t ret;
    switch(msg.type) {
      case MSG_TYPE_NONE:
        ret=254;
      case MSG_TYPE_KILL:
        ret=0;
      case MSG_TYPE_SPEKTRUM:
        ret=1;
      case MSG_TYPE_VISION_POS:
        ret=3;
      case MSG_TYPE_LOCAL_POS:
        ret=4;
      case MSG_TYPE_SETPOINT:
        ret=2;
      case MSG_TYPE_THRUST:
        ret=5;
      case MSG_TYPE_GAINS:
        ret=6;
      default:
        ret=255;
    }
    return ret;
  }
  struct elka_msg_cmp {
    bool operator()(const elka_msg_s &a,const elka_msg_s &b){
      return elka_msg_priority(a)<elka_msg_priority(b) &&
             a.timestamp<b.timestamp;
    }
  };
  std::set<elka_msg_s,elka_msg_cmp> _msgs;

public:
  static BasicMessageMgr *instance();

  /* Add messenger
   * @param messenger_type = type of messenger to add
   * @param context = additional info about messenger.
   *                  e.g. file name for serial messenger
   * @return messenger integer descriptor or error code
   */
  int add_messenger(uint8_t messenger_type, void *context);
  int8_t remove_messenger(int msgr);

  /* Send message to destination
   * @param msgr = messenger to use
   * @param pkt = packet to send
   * @return success or error code
   */
  int8_t send(int msgr,elka_packet_s *pkt);

  /* Get message from queue
   * @return message
   */
  elka_msg_s get_msg();
  int8_t add_msg(elka_msg_s);
  int8_t parse_packets();
};

