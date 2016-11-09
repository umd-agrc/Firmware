#include <px4_app.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_middleware.h>
#include <px4_tasks.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

#include "../inc/SFUartDriver.h"
#include "../../common/status.h"

// Handler for daemon_task
static int daemon_task;

extern "C" {
  __EXPORT int SFUart_main(int argc, char **argv);
}

static void usage() {
  PX4_INFO("usage: MyFirstProj [start|stop|status]");
}

int SFUart_entry(int argc, char *argv[]) {
  const char *device = nullptr;
  int ch;
  int myoptind = 1;
  const char *myoptarg = nullptr;

  px4::init(argc, argv, "SFUart");

  PX4_DEBUG("SFUart");
  
  while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'd':
      device = myoptarg;
      break;
    }
  }

  SFUartDriver sf(device);
  sf.main();

  PX4_DEBUG("outtie");
  return SUCCESS;
}

int SFUart_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage();
    return PX4_ERROR;
  }

  if (!strcmp(argv[1],"start")) {
    if (SFUartDriver::appState.isRunning()) {
      PX4_INFO("SFUart already running");
      return SUCCESS;
    }

    daemon_task = px4_task_spawn_cmd("SFUart",
        SCHED_DEFAULT,
        SCHED_PRIORITY_MAX - 5,
        2000,
        SFUart_entry,
			  (argv) ? (char * const *)&argv: (char * const *) NULL);

    return SUCCESS;
  } else if (!strcmp(argv[1],"stop")) {
    SFUartDriver::appState.requestExit();
    return SUCCESS;
  } else if (!strcmp(argv[1],"status")) {
    if (SFUartDriver::appState.isRunning()) {
      PX4_INFO("SFUart is running");
    } else {
      PX4_INFO("SFUart is not started");
    }
    return SUCCESS;
  }
  
  usage();
  return PX4_ERROR;
}

