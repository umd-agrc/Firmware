#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <vector>
#include <utility>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/nn_in.h>
#include <uORB/topics/nn_out.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/plan_element_params.h>
#include <lib/mathlib/math/Vector.hpp>

#include "nn_defines.h"
#include "nn_utils.h"
#include "genann.h"

extern "C" { __EXPORT int nn_main(int argc, char *argv[]); }

static int daemon_task_;
static volatile bool thread_should_exit_;
static volatile bool thread_running_;
static genann *nn_ctl_;

int nn_loop(int argc, char **argv);
int8_t parse_plan_file(
	std::vector<plan_element_params_s>*plan,const char *plan_file);

static void usage();
void usage() {
  PX4_WARN("usage: nn <start | stop | status>");
}

int nn_main(int argc, char *argv[]) {
	if (argc < 2) {
		PX4_WARN("Missing action.");
    usage();
    return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
    if (!thread_running_) {
      char thread_name[256];
      sprintf(thread_name,"elka_nn");

      thread_should_exit_ = false;
      daemon_task_ = px4_task_spawn_cmd(
        thread_name,
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        2000,
        nn_loop,
        &argv[2]);

      unsigned constexpr max_wait_us = 1000000;
      unsigned constexpr max_wait_steps = 2000;
      unsigned j;

      for (j=0; j < max_wait_steps; j++) {
        usleep(max_wait_us / max_wait_steps);
        if (thread_running_) {
          break;
        }
      }
      return !(j < max_wait_steps);
    } else {
      PX4_INFO("elka nn is already running.");
    }


  } else if (!strcmp(argv[1], "stop")) {
    if (!thread_running_) {
      PX4_WARN("elka nn already stopped");
      return PX4_OK;
    }

    thread_should_exit_ = true;

    while(thread_running_) {
      usleep(200000);
      PX4_WARN(".");
    }

    PX4_WARN("terminated.");

    return PX4_OK;
  } else if (!strcmp(argv[1], "status")) {
    if (thread_running_) {
      PX4_INFO("elka nn is running");
    } else {
      PX4_INFO("elka nn is not running");
    }

    return PX4_OK;

  } else {
		PX4_WARN("Action not supported");
  }

  return PX4_OK;
}

int nn_loop(int argc, char **argv) {
	thread_running_=true;
  nn_in_s nn_ctl_in;
  nn_out_s nn_ctl_out;
  elka_msg_s elka_posix;
  plan_element_params_s plan_element_params;
  std::vector<plan_element_params_s> l_plan_element_params;
	
	memset(&elka_posix,0,sizeof(elka_posix));
  memset(&nn_ctl_in,0,sizeof(nn_ctl_in));
  memset(&nn_ctl_out,0,sizeof(nn_ctl_out));
  memset(&plan_element_params,0,sizeof(plan_element_params));

  //Read in and publish plan file
	char plan_file[66]="test_takeoff.plan";

	//orb_advert_t elka_posix_pub=orb_advertise(ORB_ID(elka_msg), &elka_posix);
	orb_advert_t plan_element_params_pub=
    orb_advertise(ORB_ID(plan_element_params), &plan_element_params);
	// Sleep for one second before publishing plan file
	usleep(1000000);

	write_elka_msg_header(&elka_posix,9,MSG_TYPE_PLAN_ELEMENT);
  parse_plan_file(&l_plan_element_params,plan_file);
	for (auto it=l_plan_element_params.begin();it!=l_plan_element_params.end();it++) {
		//orb_publish(ORB_ID(elka_msg), elka_posix_pub, &elka_posix);
    memcpy(&plan_element_params, &(*it), sizeof(plan_element_params));
    orb_publish(ORB_ID(plan_element_params), plan_element_params_pub, &(*it));
		// Sleep so that DSP side can receive messages w/o missing any
		// DSP receives elka_msg at 30Hz
		usleep(500000);
	}

  /*
  if (!file_exists(*argv)) {
    PX4_WARN("Unable to open nn ctl file");
    if (!(nn_ctl_=genann_init(27,1,15,12))) {
      PX4_ERR("Failed to initialize ctl nn");
      return PX4_ERROR;
    }
  } else {
    FILE *f=fopen(*argv,"r");
    nn_ctl_=genann_read(f);
    fclose(f);
  }
  */

  // Define poll_return for defined file descriptors
  int poll_ret;

  int nn_ctl_in_sub_fd = orb_subscribe(ORB_ID(nn_in));

  // Set update rate to 100Hz
  orb_set_interval(nn_ctl_in_sub_fd, 10);

  px4_pollfd_struct_t fds[] = {
    {.fd = nn_ctl_in_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;

  while (!thread_should_exit_) {
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 500);

    // Handle the poll result
    if (poll_ret == 0) {
      // None of our providers is giving us data
      //PX4_ERR("Got no data");
    } else if (poll_ret < 0) {
      // Should be an emergency
      if (error_counter < 10 || error_counter % 50 == 0) {
        // Use a counter to prevent flooding and slowing us down
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
        thread_should_exit_=true;
      }

      error_counter++;
    } else {
      if (fds[0].revents & POLLIN) { // input_rc
        orb_copy(ORB_ID(nn_in), nn_ctl_in_sub_fd, &nn_ctl_in);
        if (nn_ctl_in.type == nn_in_s::NN_CTL_TRAIN) {
          //genann_train(
          //  nn_ctl_,
          //  (const double *)nn_ctl_in.data,
          //  (const double *)nn_ctl_in.data,
          //  .01);
        } else if (nn_ctl_in.type == nn_in_s::NN_CTL_PREDICT) {
        }
      }
    }
    usleep(50000);
  }

  genann_free(nn_ctl_);

  return PX4_OK;
}

int8_t parse_plan_file(
	std::vector<plan_element_params_s>*plan,const char *plan_file) {
  FILE *f=nullptr;
  plan_element_params_s plan_element;

	char plan_file_path[143]="\0", *plan_fp;

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
    
    // Parse phrase read into plan
    for (k=0;k<j;k++) {
      if (!strcmp(phrase[k],"calibrate")) {
        plan_element.type=plan_element_params_s::TYPE_CALIBRATE;
        plan_element.dt=0.7*PLAN_ELEMENT_DEFAULT_LEN;
      } else if (!strcmp(phrase[k],"check")) {
        plan_element.type=plan_element_params_s::TYPE_CHECK;
        plan_element.dt=0.7*PLAN_ELEMENT_DEFAULT_LEN;
      } else if (!strcmp(phrase[k],"takeoff")) {
        plan_element.type=plan_element_params_s::TYPE_TRAJECTORY;
        plan_element.trajectory_type=plan_element_params_s::TRAJ_TYPE_TAKEOFF;
        plan_element.dt=5*PLAN_ELEMENT_DEFAULT_LEN;
      } else if (!strcmp(phrase[k],"land")) {
        plan_element.type=plan_element_params_s::TYPE_TRAJECTORY;
        plan_element.trajectory_type=plan_element_params_s::TRAJ_TYPE_LAND;
        plan_element.dt=0.7*PLAN_ELEMENT_DEFAULT_LEN;
      } else if (!strcmp(phrase[k],"hover")) {
        plan_element.type=plan_element_params_s::TYPE_TRAJECTORY;
        plan_element.trajectory_type=plan_element_params_s::TRAJ_TYPE_HOVER;
#if defined(ELKA_DEBUG) && defined(DEBUG_HOVER_HOLD)
        plan_element.dt=100*PLAN_ELEMENT_DEFAULT_LEN;
#else
        plan_element.dt=PLAN_ELEMENT_DEFAULT_LEN;
#endif
      } else {
        PX4_WARN("Unrecognized plan word %s",phrase[k]);
      }

			plan->push_back(plan_element);
    }
  }
  return ELKA_SUCCESS;
}
