#include <stdio.h>
//#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <istream>

#include "basic_control.h"

static bool thread_running_,thread_should_exit_;
static int daemon_task_;
struct hrt_call	print_state_call_;
#if defined(ELKA_DEBUG) && defined(DEBUG_NAVIGATOR)
static int print_state_call_interval_;
#endif

void print_state(void *arg);

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
      2500,
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

// Assumes well-formatted message
int8_t elka::BasicController::parse_plan_element(uint8_t element_type,
	hrt_abstime t) {
	_plan.insert(new PlanElement(element_type,t));
  return ELKA_SUCCESS;
}

int8_t elka::BasicController::parse_plan_element(plan_element_params_s el) {
  _plan.insert(new PlanElement(&el));
  return ELKA_SUCCESS;
}

void elka::BasicController::wait_for_nav() {
  if (_nav._pose_init) return;

  PX4_INFO("Waiting for SLAM pose");
  uint32_t i=0;
  while (!_nav._pose_init) {
    PX4_INFO(".");
    usleep(200000);
    i++;
  }
} 

int8_t elka::BasicController::execute_plan() {
  static std::set<PlanElement *,plan_element_cmp>::iterator it;
  uint8_t ret=ELKA_SUCCESS;
  // Get next plan element
  for (it=_plan.begin();it!=_plan.end() && (*it)->_completed;it++) {
    PX4_INFO("Plan element completed type %d",(*it)->_type);
    erase_plan_element(it);
  }
	if (it==_plan.end()) return ELKA_SUCCESS;

  // Handle new plan element
  if (!(*it)->_begun) {
    // Load/perform plan element operations if not already begun
    // Either loading is here and plan actions are done concurrently
    //    e.g. trajectory
    // or performance is done here one time
    //    e.g. calibrate|check
    switch((*it)->_type) {
    case plan_element_params_s::TYPE_NONE:
      break;
    case plan_element_params_s::TYPE_CALIBRATE:
      PX4_INFO("Performing calibration");
      break;
    case plan_element_params_s::TYPE_CHECK:
      PX4_INFO("Performing check");
      break;
    case plan_element_params_s::TYPE_TRAJECTORY:
      PX4_INFO("Loading trajectory");
      wait_for_nav();
      _nav.reset_setpoints();
      //TODO place hover hold setpoint in while new trajectory is being
      //generated
      ret=_nav.trajectory(&(*it)->_params);
      break;
    default:
      PX4_WARN("Invalid plan element type %d",(*it)->_type);
      break;
    }
  }

  (*it)->update();

  if ((*it)->_timeout) {
    PX4_INFO("Plan element timeout type %d",(*it)->_type);
    erase_plan_element(it);
    return ELKA_SUCCESS;
  }
  return ret;
}

int8_t elka::BasicController::parse_msg() {
  if (_msg.type)
    PX4_INFO("type: %d",_msg.type);

  if (_msg.type==MSG_TYPE_GAINS) {
    // If nn_ctl outputs gains,
    // send gains as error + current position as input
    // nn struct:
    //  in: last position
    //  out: controller gains
  } else if (_msg.type==MSG_TYPE_ERROR) {

  } else if (_msg.type==MSG_TYPE_TEST) {
    PX4_INFO("Test command:");
    print_elka_msg(_msg);
  } else if (_msg.type==MSG_TYPE_MOTOR_INPUTS) {
    // If nn_ctl outputs motor inputs,
    // send motor inputs as error + current position as input
    // nn struct:
    //  in: last position
    //  out: motor inputs 

  } else if (_msg.type==MSG_TYPE_NONE) {
    // Do nothing
  }
  return ELKA_SUCCESS;
}

int8_t elka::BasicController::add_messenger(int msgr) {
    _msgrs[msgr_idx]=msgr;
    return ELKA_SUCCESS;
}

void elka::BasicController::print_state() {
  auto it=_plan.begin();
  if (it!=_plan.end())
    PX4_INFO("Current plan element type: %d",(*it)->_type);
#if defined(ELKA_DEBUG) && defined(DEBUG_NAVIGATOR)
  _nav.print_setpoints();
#endif
}

int run_controller(int argc, char **argv) {
  elka::BasicController *ctl=elka::BasicController::instance();
  thread_running_=true;
  thread_should_exit_=false;

  ctl->msgr_idx=0;

#if defined(ELKA_DEBUG) && defined(DEBUG_NAVIGATOR)
  print_state_call_interval_=450000; // us
  hrt_call_every(&print_state_call_, 0,
           (print_state_call_interval_),
           (hrt_callout)&print_state,
           (void *)ctl);
#endif

	while(!thread_should_exit_) {
		ctl->execute_plan();
		ctl->set_msg();
		ctl->parse_msg();
#if defined(ELKA_DEBUG) && defined(DEBUG_CONTROLLER)
    ctl->print_plan();
#endif
		usleep(25000);
	}
  thread_running_=false;
  return ELKA_SUCCESS;
}

void print_state(void *arg) {
  elka::BasicController *ctl=(elka::BasicController *)arg;
  ctl->print_state();
}
