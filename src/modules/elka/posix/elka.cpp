#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <string.h>

#include "elka.h"
#include "elka_manager.h"

extern "C" { __EXPORT int elka_main(int argc, char *argv[]); }

static elka::Manager *mgr = nullptr;

int elka_main(int argc, char *argv[]) {

	if (argc < 2) {
		PX4_WARN("Missing action <start|status>");
		return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
    if (mgr != nullptr) {
      PX4_WARN("Already loaded");
      return 0;
    }

    if (!elka::Manager::initialize()) {
      PX4_ERR("elka manager alloc failed");
      return -ENOMEM;
    }

    // Create the manager
    mgr = elka::Manager::get_instance();

    if (mgr == nullptr) {
      return -errno;
    }

	} else if (!strcmp(argv[1],"status")) {
    if (mgr != nullptr) {
      mgr->print_statistics(true);
    } else {
      PX4_INFO("elka is not running");
    }

    return PX4_OK;

  } else {
		PX4_WARN("Action not supported");
	}

	return PX4_OK;
}
