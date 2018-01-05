#include <stdio.h>
//#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <istream>
#include <uORB/topics/elka_msg.h>

#include "basic_control.h"

static bool thread_running_,thread_should_exit_;
static int daemon_task_;

elka::BasicController *elka::BasicController::_inst=NULL;

elka::BasicController *elka::BasicController::instance() {
  if (!_inst)
    _inst=new BasicController;

  return _inst;
}

elka::BasicController::BasicController() {
  thread_running_=false;
  thread_should_exit_=true;
  _nav=BasicNavigator();
  _msg_mgr=BasicMessageMgr::instance();
}

int8_t elka::BasicController::start() {
  if (!thread_running_) {
    char thread_name[256];
    sprintf(thread_name,"elka_controller");
    thread_should_exit_ = false;
    daemon_task_ = px4_task_spawn_cmd(
      thread_name,
      SCHED_DEFAULT,
      SCHED_PRIORITY_DEFAULT,
      800,
      run_controller,
      NULL);

    unsigned constexpr max_wait_us = 1000000;
    unsigned constexpr max_wait_steps = 2000;
    unsigned j;

    for (j=0; j < max_wait_steps; j++) {
      usleep(max_wait_us / max_wait_steps);
      if (thread_running_) {
        PX4_INFO("Started basic controller");
        break;
      }
    }
    return !(j < max_wait_steps);

  } else {
    return ELKA_SUCCESS; 
  }
}

int8_t elka::BasicController::exit() {
  if (!thread_running_) {
    PX4_WARN("basic controller already stopped");
    return ELKA_SUCCESS;
  }

  thread_should_exit_ = true;

  while(thread_running_) {
    usleep(200000);
    PX4_WARN(".");
  }

  PX4_WARN("basic controller terminated.");

  return ELKA_SUCCESS;
}

void elka::BasicController::nn_ctl_init(const char *filename) {
  if (!nn_ctl_read(filename)) {
    //_nn_ctl=genann_init(27,1,15,12);
  }
}

void elka::BasicController::nn_ctl_delete() {
  //genann_free(_nn_ctl);
}

bool elka::BasicController::nn_ctl_read(const char *filename) {
  if (file_exists(filename)) {
    FILE *f=fopen(filename,"r");
    //_nn_ctl=genann_read(f);
    fclose(f);
    return true;
  } else return false;
}

void elka::BasicController::nn_ctl_write(const char *filename) {
  // Create/overwrite file w/`filename`
  FILE *f=fopen(filename,"w");
  //genann_write(_nn_ctl,f);
  fclose(f);
}

int8_t elka::BasicController::parse_plan_file(const char *plan_file) {
  FILE *f=nullptr;
	char plan_file_path[144]="\0", *plan_fp;
  strcat(plan_file_path,ELKA_DIR);
  strcat(plan_file_path,FLIGHT_PLAN_DIR);
  strcat(plan_file_path,plan_file);
	plan_fp=trim_path(plan_file_path);
  PX4_INFO("Reading plan file path: %s",plan_fp);
  if (file_exists(plan_fp)) {
    f=fopen(plan_fp,"r");
  } else {
    PX4_WARN("plan file doesn't exist!");
    return PARSE_ERROR;
  }

  //TODO parse file
  char line[128],*line_ptr,phrase[10][20];
  uint8_t j=0,k=0;
  while (fgets(line,sizeof(line),f)!=NULL) {
    j=0;k=0;
    line_ptr=line;
    while (*line_ptr) {
      SKIP(line_ptr);
			if (WANT(line_ptr)) {
				while (WANT(line_ptr)) {
					phrase[j][k]=*line_ptr;
					line_ptr++;
					k++;
				}
				phrase[j][k]=0;
				j++;
			}
    }
    
    // Parse phrase read into _plan
    for (k=0;k<j;k++) {
      if (!strcmp(phrase[k],"calibrate")) {
        _plan.insert(new PlanElement(PLAN_ELEMENT_CALIBRATE,
					PLAN_ELEMENT_DEFAULT_LEN));
      } else if (!strcmp(phrase[k],"check")) {
        _plan.insert(new PlanElement(PLAN_ELEMENT_CHECK,
					PLAN_ELEMENT_DEFAULT_LEN));
      } else if (!strcmp(phrase[k],"takeoff")) {
        std::vector<math::Vector<POSITION_LEN>> v;
				float p[POSITION_LEN]={0,0,HOVER_DEFAULT_HEIGHT,0,0};
        v.push_back(p);
        _plan.insert(new PlanElement(PLAN_ELEMENT_TAKEOFF,
					PLAN_ELEMENT_DEFAULT_LEN,v));
      } else if (!strcmp(phrase[k],"land")) {
        std::vector<math::Vector<POSITION_LEN>> v;
				float p[POSITION_LEN]={0,0,0,0,0};
        v.push_back(p);
        _plan.insert(new PlanElement(PLAN_ELEMENT_LAND,
					PLAN_ELEMENT_DEFAULT_LEN,v));
      } else if (!strcmp(phrase[k],"hover")) {
        std::vector<math::Vector<POSITION_LEN>> v;
				float p[POSITION_LEN]={0,0,HOVER_DEFAULT_HEIGHT,0,0};
        v.push_back(p);
        _plan.insert(new PlanElement(PLAN_ELEMENT_HOVER,
					PLAN_ELEMENT_DEFAULT_LEN,v));
      } else {
        PX4_WARN("Unrecognized plan word %s",phrase[k]);
      }
    }
  }
	print_plan();
  return ELKA_SUCCESS;
}

int8_t elka::BasicController::execute_plan() {
  static PlanElement *e;
  static std::set<PlanElement *,plan_element_cmp>::iterator it;
  uint8_t ret=ELKA_SUCCESS;
  // Get next plan element
  it=_plan.begin();
  while (it!=_plan.end() && (*it)->_completed) {
    e=*it;
    _plan.erase(it);
    delete e;
    e=nullptr;
    it++;
  }
	if (it==_plan.end()) return ELKA_SUCCESS;
#if defined(ELKA_DEBUG) && defined(DEBUG_CONTROLLER)
	e->print_element();
#endif
  switch(e->_type) {
  case PLAN_ELEMENT_CALIBRATE:
    // Skip for now
    if (!e->_begun) {
      e->_begun=true;
      e->_completed=true;
      ret=ELKA_SUCCESS;
    }
    break;
  case PLAN_ELEMENT_CHECK:
    // Skip for now
    if (!e->_begun) {
      e->_begun=true;
      e->_completed=true;
      ret=ELKA_SUCCESS;
    }
    break;
  case PLAN_ELEMENT_TAKEOFF:
    if (!e->_begun) {
      e->_begun=true;
      ret=_nav.generate_setpoints(e->_positions);
    }
    break;
  case PLAN_ELEMENT_LAND:
    if (!e->_begun) {
      e->_begun=true;
      ret=_nav.generate_setpoints(e->_positions);
    }
    break;
  case PLAN_ELEMENT_HOVER:
    if (!e->_begun) {
      e->_begun=true;
      ret=_nav.generate_setpoints(e->_positions);
    }
    break;
  }
  return ret;
}

int8_t elka::BasicController::parse_msg() {
  if (_msg.type==MSG_TYPE_GAINS) {
    // Do some stuff with gains for nn
  } else if (_msg.type==MSG_TYPE_ERROR) {

  } else if (_msg.type==MSG_TYPE_TEST) {
    PX4_INFO("Test command:");
    print_elka_msg(_msg);
  } else if (_msg.type==MSG_TYPE_NONE) {
    // Do nothing
  }
  return ELKA_SUCCESS;
}

int8_t elka::BasicController::add_messenger(int msgr) {
    _msgrs[msgr_idx]=msgr;
    return ELKA_SUCCESS;
}

int run_controller(int argc, char **argv) {
  elka::BasicController *ctl=elka::BasicController::instance();
  thread_running_=true;
  thread_should_exit_=false;

  // Set up neural network
  strcpy(ctl->_nn_ctl_filename,"/dev/fs/.elka/nn/ctl.nn");
  ctl->nn_ctl_init(ctl->_nn_ctl_filename);

	while(!thread_should_exit_) {
		//ctl->execute_plan();
		ctl->set_msg();
		//ctl->parse_msg();
		//usleep(10000);
	}
	return 0;
  while(!thread_should_exit_) {
    ctl->execute_plan();
    ctl->set_msg();
    ctl->parse_msg();
    usleep(10000);
  }
  thread_running_=false;
  return ELKA_SUCCESS;
}
