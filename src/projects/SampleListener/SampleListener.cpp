#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <px4_app.h>
#include <px4_config.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <modules/muorb/krait/uORBKraitFastRpcChannel.hpp>
#include <uORB/uORB.h>
#include <uORB/uORBManager.hpp>
#include "uORB/topics/vehicle_command.h"
#include "uORB/topics/vehicle_command_ack.h"
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

/* process-specific header files */
#include <projects/sysdefs.h>

extern "C" {
  __EXPORT int SampleListener_main(int argc, char *argv[]);
}

int SampleListener_thread_main(int argc, char *argv[]);

/* Variables */
static bool thread_should_exit = false; /* Daemon exit flag */
static bool thread_running = false; /* Daemon status flag */
static int daemon_task; /* Handle of daemon task/thread */
static px4::AppState appState;

int sub_vcmd;

struct vehicle_command_s vcmd;
struct vehicle_command_ack_s vcmda;

orb_advert_t pub_vcmda;

/* Display usage directions */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: SampleListener 'start'|'stop'|'status'\n\n");
  exit(-1);
} 

int SampleListener_thread_main(int argc, char *argv[]) {
	/* read arguments */
  appState.setRunning(true);
	bool verbose = false;

	for (int i=1; i<argc; i++) {
		if (strcmp(argv[i],"-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		} else {
		}
	}
	
	// Get rid of compiler error for now
	if (verbose){}
	
	/* Welcome user (PX4_INFO prints a line, including an appended\n, with variable arguments */
	PX4_INFO("Started");

  // Set up subscribers
  sub_vcmd = orb_subscribe(ORB_ID(vehicle_command));
  if (sub_vcmd == PX4_ERROR) {
    PX4_ERR("Error subscribing to vehicle_command_ack topic");
    return PX4_ERROR;
  }

  // Set up publishers
  pub_vcmda =
    orb_advertise(ORB_ID(vehicle_command_ack), &vcmda);

  if (pub_vcmda == 0) {
    PX4_ERR("Error publishing vehicle_command_ack");
    return PX4_ERROR;
  }

  // Allow publishers to register with krait and hexagon networks
  // sleep for .1s
  usleep(100000); 

  // Initialize messages to 0 arrays
  memset(&vcmd, 0, sizeof(vcmd)); 
  memset(&vcmda, 0, sizeof(vcmda)); 

  //////FIXME DEBUGGING////////
  int error_cnt = 0;
  bool updated = false;
  while (!appState.exitRequested()) {
    if (orb_check(sub_vcmd, &updated) == 0) {
      if (updated) {
        if (orb_copy(ORB_ID(vehicle_command), sub_vcmd, &vcmd) != 0) {
          if (++error_cnt < 10 ||
              error_cnt % 100 == 0) {;
            PX4_ERR("[%d] Error calling orb copy for vehicle_command_ack",
                error_cnt);
            break;
          }
        } else {
          PX4_INFO("Vehicle command:\t%u\nVehicle param1:\t%f",
            (uint16_t)vcmd.command,
            (double)vcmd.param1);
          if (vcmd.target_system == SYS_SL){
            vcmda.command = vcmd.command;
            vcmda.result = 0;
            orb_publish(ORB_ID(vehicle_command_ack), pub_vcmda, &vcmda);
            PX4_INFO("Vehicle command ack:\t%u\t%u",
              (uint16_t)vcmda.command,
              (uint8_t)vcmda.result);
          }
        }
      }
    }
    usleep(100000);
  }

  PX4_INFO("Exiting");
  appState.setRunning(false);

	fflush(stdout);

	return 0;
  //////FIXME END DEBUGGING////////
}

int SampleListener_main(int argc, char *argv[]) {

  if (argc < 2) {
    usage("missing command");
    return -EINVAL;
  }

  /*
   * Return sensor values
   */
  if(!strcmp(argv[1],"start")) {
    if (appState.isRunning()) {
			PX4_INFO("Already running");
   	return 0; 
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("SampleListener",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 20,
			1200,
			SampleListener_thread_main,
			(argv) ? (char * const *)&argv[2] : (char * const *) NULL);
		thread_running = true;
   	return 0; 
  }

  /*
   * Send sample UART values to ESC
   */
  if(!strcmp(argv[1],"stop")) {
    appState.requestExit();
    PX4_INFO("Exit requested");
   	return 0; 
  }

	if (!strcmp(argv[1],"status")) {
    if (appState.isRunning()) {
			PX4_INFO("Running");
		} else {
			PX4_INFO("Not running");
		}

   	return 0; 
	}

  usage("unrecognized command");
  exit(-1);
}
