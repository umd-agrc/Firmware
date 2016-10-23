#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <px4_posix.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <projects/SnapdragonSerialFramework/framework/snapdragon_linaro/inc/SFMessenger.h>

/* process-specific header files */


  __EXPORT int MyFirstProj_main(int argc, char *argv[]);

int MyFirstProj_thread_main(int argc, char *argv[]);

/* Variables */
static bool thread_should_exit = false; /* Daemon exit flag */
static bool thread_running = false; /* Daemon status flag */
static int daemon_task; /* Handle of daemon task/thread */
uint32_t uart_ret;

/* Display usage directions */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: MyFirstProj 'start'|'stop'|'status'\n\n");
	exit(1);
}

int MyFirstProj_thread_main(int argc, char *argv[]) {
	/* read arguments */
	bool verbose = false;
  char str[256];

	for (int i=1; i<argc; i++) {
		if (strcmp(argv[i],"-v") == 0 || strcmp (argv[i], "--verbose") == 0) {
			verbose = true;
		} else {
		}
	}
	
	// Get rid of compiler error for now
	if (verbose){}
	
	/* Welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("\n[MyFirstProj] started");

  uart_ret = esc_write_read();
  if (uart_ret == SUCCESS) {
    warnx("\nUART write/read returned success");
  } else if (uart_ret == ERROR) {
    warnx("\nUART write/read returned error");
    thread_should_exit = true;
  } else { // shouldn't happen! Should initiate shutdown
    sprintf(str,"\nUART write/read returned %d\n",uart_ret);
    warnx(str);
    thread_should_exit = true;
  }

	while(!thread_should_exit) {
	}

	printf("[MyFirstProj] exiting, killing muorb and stopping all motors.\n");

	fflush(stdout);

	return 0;
	
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
		if (thread_running) {
			printf("MyFirstProj already running\n");
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("MyFirstProj",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 20,
			2048,
			MyFirstProj_thread_main,
			(argv) ? (char * const *)&argv[2] : (char * const *) NULL);
		thread_running = true;
   	exit(0); 
  }

  /*
   * Send sample UART values to ESC
   */
  if(!strcmp(argv[1],"stop")) {
		thread_should_exit = true;
   	exit(0); 
  }

	if (!strcmp(argv[1],"status")) {
		if (thread_running) {
			printf("\tMyFirstproj is running\n");
		} else {
			printf("\tMyFirstProj not started\n");
		}

   	exit(0); 
	}

  usage("unrecognized command");
  exit(1);
}
