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

#include <uORB/uORB.h>
#include <uORB/topics/nn_in.h>
#include <uORB/topics/nn_out.h>
#include <lib/mathlib/math/Vector.hpp>

#include "genann.h"

extern "C" { __EXPORT int nn_main(int argc, char *argv[]); }

static int daemon_task_;
static volatile bool thread_should_exit_;
static volatile bool thread_running_;
static genann *nn_ctl_;

void usage();
int nn_loop(int argc, char **argv);

void usage() {
  PX4_WARN("usage: elka_nn <start | stop | status>");
}

inline bool file_exists(const char *s) {
  FILE *f;
  if ((f=fopen(s,"r"))!=NULL) {
    fclose(f);
    return true;
  } else return false;
}

int nn_main(int argc, char *argv[]) {
	if (argc < 3) {
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
        1000,
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
  nn_in_s nn_ctl_in;
  nn_out_s nn_ctl_out;

  memset(&nn_ctl_in,0,sizeof(nn_ctl_in));
  memset(&nn_ctl_out,0,sizeof(nn_ctl_out));

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
      PX4_ERR("Got no data");
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
  }

  genann_free(nn_ctl_);

  return PX4_OK;
}
