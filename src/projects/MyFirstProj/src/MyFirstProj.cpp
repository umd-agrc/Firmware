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
#include "../inc/status.h"
#include <projects/sysdefs.h>

extern "C" {
  __EXPORT int MyFirstProj_main(int argc, char *argv[]);
}

int MyFirstProj_thread_main(int argc, char *argv[]);

/* Variables */
static bool thread_should_exit = false; /* Daemon exit flag */
static bool thread_running = false; /* Daemon status flag */
static int daemon_task; /* Handle of daemon task/thread */
uint32_t uart_ret;
static px4::AppState appState;

int sub_vca;

struct vehicle_command_s vc;
struct vehicle_command_ack_s vca;

orb_advert_t pub_vc;

/* Display usage directions */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: MyFirstProj 'start'|'stop'|'status'\n\n");
  exit(ERROR);
} 

int MyFirstProj_thread_main(int argc, char *argv[]) {
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

  // Set up publishers
  pub_vc =
    orb_advertise(ORB_ID(vehicle_command), &vc);

  if (pub_vc == 0) {
    PX4_ERR("Error publishing vehicle_command_ack");
    return PX4_ERROR;
  }

  // Allow publishers to register with krait and hexagon networks
  // sleep for .1s
  usleep(100000); 

  // Set up subscribers
  sub_vca = orb_subscribe(ORB_ID(vehicle_command_ack));
  if (sub_vca == PX4_ERROR) {
    PX4_ERR("Error subscribing to vehicle_command_ack topic");
    return PX4_ERROR;
  }

  // Initialize messages to 0 arrays
  memset(&vc, 0, sizeof(vc)); 
  memset(&vca, 0, sizeof(vca)); 

  //////FIXME DEBUGGING////////
  int error_cnt = 0;
  int tmp = 0; //Debugging, for vehicle_command param1
  bool updated = false;
  // Set confirmation for first transmission
  vc.confirmation = 0;
  while (!appState.exitRequested()) {
    orb_publish(ORB_ID(vehicle_command), pub_vc, &vc);
    if (orb_check(sub_vca, &updated) == 0) {
      if (updated) {
        if (orb_copy(ORB_ID(vehicle_command_ack), sub_vca, &vca) != 0) {
          if (++error_cnt < 10 ||
              error_cnt % 100 == 0) {;
            PX4_ERR("[%d] Error calling orb copy for vehicle_command_ack",
                error_cnt);
            break;
          }
        } else {
          PX4_INFO("Vehicle command ack:\t%u\t%u",
            (uint16_t)vca.command,
            (uint8_t)vca.result);
        }
        // Update vehicle_command confirmation to vehicle_command_ack result
        vc.confirmation = vca.result;
      }
    }
    // Set vehicle_command to send
    vc.source_system = SYS_MFP;
    vc.target_system = SYS_SFUART;
    vc.command = vehicle_command_s::VEHICLE_CMD_CUSTOM_0;
    vc.param1 = tmp++;
    PX4_INFO("Vehicle command:\t%u\nVehicle param1:\t%f",
        (uint16_t)vc.command,
        (double)vc.param1);

    usleep(100000);
  }

  PX4_INFO("Exiting");
  appState.setRunning(false);

	fflush(stdout);

	return 0;
  //////FIXME END DEBUGGING////////
}

int MyFirstProj_main(int argc, char *argv[]) {

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
   	return SUCCESS; 
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("MyFirstProj",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 20,
			2048,
			MyFirstProj_thread_main,
			(argv) ? (char * const *)&argv[2] : (char * const *) NULL);
		thread_running = true;
   	return SUCCESS; 
  }

  /*
   * Send sample UART values to ESC
   */
  if(!strcmp(argv[1],"stop")) {
    appState.requestExit();
    PX4_INFO("Exit requested");
   	return SUCCESS; 
  }

	if (!strcmp(argv[1],"status")) {
    if (appState.isRunning()) {
			PX4_INFO("Running");
		} else {
			PX4_INFO("Not running");
		}

   	return SUCCESS; 
	}

  usage("unrecognized command");
  exit(ERROR);
}
