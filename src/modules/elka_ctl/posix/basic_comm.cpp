#include <px4_log.h>
#include <px4_defines.h>

#include "basic_comm.h"

// Global static pointer to ensure single instance of class
elka::BasicMessageMgr *elka::BasicMessageMgr::_inst=NULL;
elka::BasicMessageMgr *elka::BasicMessageMgr::instance() {
  if (!_inst) // Enforce singleton isn't created new each time
    _inst=new BasicMessageMgr();
  return _inst;
}

int elka::BasicMessageMgr::add_messenger(
    uint8_t messenger_type, void *context) {
  int8_t ret;
  if (messenger_type==MESSENGER_SERIAL) {
    char *des=(char *)context;
    _messengers[_cnt]=new SnapdragonSerialMessenger(des);
    ret=_cnt;
    _cnt++;       
  } else {
    ret=MSG_MGR_ERROR;
  }
  return ret;
}

// Parse raw data into elka messages
int8_t elka::BasicMessageMgr::parse_packets() {
  static uint8_t buf[MAX_ELKA_PACKET_LEN];
  elka_msg_s msg;
  uint8_t idx=1,packet_len;
  //Iterate thru messengers
  for (auto it=_messengers.begin(); it!=_messengers.end(); it++) {
    if (it->second->_data_rdy) {
      memcpy(
          buf,
          it->second->_rx_buf,
          it->second->_rx_buf[ELKA_MSG_PACKET_LEN]+1);
      it->second->_data_rdy=false;
      packet_len=buf[ELKA_MSG_PACKET_LEN];
#if defined(ELKA_DEBUG) && defined(DEBUG_MGR_PARSE)
      PX4_INFO("parsing array from msgr %s:",it->second->_des);
      PX4_INFO("packet len: %d",packet_len);
      print_array(buf,packet_len+1);
#endif
      while (idx<=packet_len) {
        if (check_msg_header(&(buf[idx+ELKA_MSG_LOCAL_MSG_OFFSET]))
            !=ELKA_SUCCESS)
          return MSG_ERROR;
        msg.len=buf[idx+ELKA_MSG_LEN]-ELKA_MSG_HEADER_LEN;
        msg.type=buf[idx+ELKA_MSG_TYPE];
        memcpy(
          msg.data,
          &(buf[idx+ELKA_MSG_DATA_OFFSET]),
          msg.len);
        idx+=buf[idx+ELKA_MSG_LEN]+1;
        add_msg(msg);

#if defined(ELKA_DEBUG) && defined(DEBUG_MGR_PARSE)
        print_elka_msg(msg);
#endif
      }
    }
  }
  return ELKA_SUCCESS;
}

int8_t elka::BasicMessageMgr::remove_messenger(int msgr) {
  int8_t ret;
  auto it=_messengers.find(msgr);
  if (it!=_messengers.end()) {
    _messengers.erase(it);
    ret=ELKA_SUCCESS;
  } else {
    ret=MSG_MGR_ERROR;
  }
  return ret;
}

int8_t elka::BasicMessageMgr::send(
    int msgr,elka_packet_s *pkt){
  std::map<int,BasicMessenger*>::iterator it=
    _messengers.find(msgr);
  if (it!=_messengers.end()) {
    return it->second->send(pkt);
  }
  else {
    return MSG_MGR_ERROR;
  }
}

elka_msg_s elka::BasicMessageMgr::get_msg() {
  elka_msg_s ret;
  if (!_msgs.empty()) {
    ret=*_msgs.begin(); 
    _msgs.erase(_msgs.begin());
  } else {
    // Return nonetype message
    ret.type=MSG_TYPE_NONE;
  }
  return ret;
}

int8_t elka::BasicMessageMgr::add_msg(elka_msg_s m) {
  int8_t ret;
  _msgs.insert(m);
  if (_msgs.size()>_max_num_msgs) {
    auto it=_msgs.end();
    --it;
    _msgs.erase(it);
  }
  ret=ELKA_SUCCESS;
  return ret;
}
