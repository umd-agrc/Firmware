#pragma once

#include <vector>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include "serial_defines.h"
#include "basic_navigator.h"
#include "basic_comm.h"
#include "genann.h"

#define GAINS_LEN 12

struct gains_s {
  // array containing gains, k:
  //  0: k_p_x
  //  1: k_p_y
  //  2: k_p_z
  //  3: k_d_x
  //  4: k_d_y
  //  5: k_d_z
  //  6: k_p_r
  //  7: k_p_p,
  //  8: k_p_yaw,
  //  9: k_d_r
  //  10: k_d_p,
  //  11: k_d_yaw}
  float k[GAINS_LEN];
};

namespace elka {
  class BasicController; 
}

int run_controller(int argc, char **argv);

class elka::BasicController {
private:
  static BasicController *_inst;
  BasicController();
  BasicNavigator _nav;
  BasicMessageMgr *_msg_mgr;

  struct plan_element_cmp {
    bool operator()(const PlanElement *a,const PlanElement *b){
			if (PlanElement::priority(a)<PlanElement::priority(b))
				return true;
			else
				return a->_init_time<b->_init_time;
    }
  };

  std::set<PlanElement *,plan_element_cmp> _plan;

  elka_msg_s _msg;
public:
  static BasicController *instance();
  // Neural network inputs (27):
  // [xe,ye,ze,vxe,vye,vze,re,pe,yawe,dre,dpe,dyawe,
  //  kpx,kpy,kpz,kdx,kdy,kdz,kpr,kpp,kpyaw,kdr,kdp,kdyaw,
  //  Ixx,Iyy,Izz,prop_specs?]
  // 1 hidden layer (15 nodes)
  // Nn outputs(12,gain updates):
  // [dkpx,dkpy,dkpz,dkdx,dkdy,dkdz,dkpr,dkpp,dkpyaw,dkdr,dkdp,dkdyaw]
  char _nn_ctl_filename[128];
  genann *_nn_ctl;
  /* Initialize nn either from file or random
   */
  void nn_ctl_init(const char *filename);
  void nn_ctl_delete();
  /* Load nn_ctl from file
   * @return true if loaded, else false
   */
  bool nn_ctl_read(const char *filename);
  /* Write nn_ctl to file*/
  void nn_ctl_write(const char *filename);

  BasicNavigator *get_navigator() {return &_nav;};
  int8_t parse_plan_file(const char *plan_file);
  //TODO Clear setpoints from plan_element
  //     For now just reset setpoints
  void erase_plan_element(
      std::set<PlanElement *,plan_element_cmp>::iterator it) {
    PlanElement *e;
    e=*it;
    _plan.erase(it);
    delete e;
    e=nullptr;
  }
  int8_t execute_plan();
	void print_plan() {
		char s[1024];
		sprintf(s,"Current plan\nPlan size: %d\n",_plan.size());
		uint8_t i=1;
		for(auto it=_plan.begin();it!=_plan.end();it++) {
			char e[40];
			sprintf(e,"%d) Element type: %d,time: %" PRIu64 "+%" PRIu64 "\n",
				i,(*it)->_type,(*it)->_start,(*it)->_dt);
			i++;
			strcat(s,e);
		}
		PX4_INFO("%s",s);
	}
  int8_t parse_msg();
  void set_msg() {
    _msg=_msg_mgr->get_msg();
  }
  //Temporary method to get messenger until message
  //routing is implemented
  //Temporary messengers array as well
  int _msgrs[10]; uint8_t msgr_idx;
  int8_t add_messenger(int msgr);
  int8_t start();
  int8_t exit();
  void print_state();
};
