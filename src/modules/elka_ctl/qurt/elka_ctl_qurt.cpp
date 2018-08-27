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

#include "elka_ctl_qurt.h"

extern "C" { __EXPORT int elka_ctl_qurt_main(int argc, char *argv[]); }

static int daemon_task;
static volatile bool _thread_should_exit;
static volatile bool _thread_running;

static char _elka_uart[18];
static char _elka_uart_buf[45];
static bool _elka_uart_open;
static int _elka_uart_fd;

struct termios _elka_uart_oldtio;

void usage() {
  PX4_WARN("usage: elka_ctl_qurt <start | stop | status>");
}

int elka_ctl_qurt_main(int argc, char *argv[]) {
	if (argc < 2) {
		PX4_WARN("Missing action.");
    usage();
    return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
    if (!_thread_running) {
      char thread_name[256];
      sprintf(thread_name,"elka_ctl_posix");

      _thread_should_exit = false;
      daemon_task = px4_task_spawn_cmd(
        thread_name,
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        400,
        elka_ctl_loop,
        &argv[2]);

      unsigned constexpr max_wait_us = 1000000;
      unsigned constexpr max_wait_steps = 2000;
      unsigned j;

      for (j=0; j < max_wait_steps; j++) {
        usleep(max_wait_us / max_wait_steps);
        if (_thread_running) {
          break;
        }
      }
      return !(j < max_wait_steps);
    } else {
      PX4_INFO("elka control qurt-side is already running.");
    }


  } else if (!strcmp(argv[1], "stop")) {
    if (!_thread_running) {
      PX4_WARN("elka control qurt-side already stopped");
      return PX4_OK;
    }

    _thread_should_exit = true;

    while(_thread_running) {
      usleep(200000);
      PX4_WARN(".");
    }

    PX4_WARN("terminated.");

    return PX4_OK;
  } else if (!strcmp(argv[1], "status")) {
    if (_thread_running) {
      PX4_INFO("elka control posix-side is running");
    } else {
      PX4_INFO("elka control posix-side is not running");
    }

    return PX4_OK;

  } else {
		PX4_WARN("Action not supported");
  }

  return PX4_OK;
}

int elka_ctl_loop(int argc, char *argv[]) {
  _thread_running = true;
  
  elka_uart_set_params();
  uint8_t open_file_error_count = 0;
  while (!_elka_uart_open) {
    elka_uart_open();
    usleep(5000);
    open_file_error_count++;
    if (open_file_error_count == 100) {
      PX4_ERR("Failed to open elka uart file");
      _thread_running = false;
      return PX4_ERROR;
    }
  }

  // Define poll_return for defined file descriptors
  int poll_ret;

  elka_packet_s elka_pkt;
  memset(&elka_pkt,0,sizeof(elka_pkt));

  int elka_packet_sub_fd = orb_subscribe(ORB_ID(elka_packet));

  orb_set_interval(elka_packet_sub_fd, 30);

  px4_pollfd_struct_t fds[] = {
    {.fd = elka_packet_sub_fd, .events = POLLIN},
  };

  int error_counter = 0;
  uint32_t no_data_counter = 0;

  while (!_thread_should_exit) {
    poll_ret = px4_poll(&fds[0], sizeof(fds)/sizeof(fds[0]), 500);

    // Handle the poll result
    if (poll_ret == 0) {
      // None of our providers is giving us data
      if (!(no_data_counter%3))
        PX4_ERR("Got no data");
      no_data_counter++;
    } else if (poll_ret < 0) {
      // Should be an emergency
      if (error_counter < 10 || error_counter % 50 == 0) {
        // Use a counter to prevent flooding and slowing us down
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
        _thread_should_exit=true;
      }

      error_counter++;
    } else {
      if (fds[0].revents & POLLIN) { // input_rc
        orb_copy(ORB_ID(elka_packet),
            elka_packet_sub_fd,
            &elka_pkt);
        elka_uart_send(&elka_pkt);
      }
    }
  }
  
  _thread_running = false;

  return PX4_OK;
}

int elka_uart_set_interface_attribs(int fd, int baud, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		PX4_ERR("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, baud);
	cfsetispeed (&tty, baud);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing

	tty.c_lflag = 0;                // no canonical processing,
                                  // no echoing, no signaling characters

	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  // shut off xon/xoff ctrl
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // map CR to NL (st CR will terminate input)
  // ignore modem controls
  // enable reading
  tty.c_iflag |= (ICRNL | CLOCAL | CREAD);

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
																	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

  // Clean the modem line
  tcflush(fd, TCIFLUSH);

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		PX4_ERR("error %d from tcsetattr", errno);
    return PX4_ERROR;
	}
	return PX4_OK;
}

void elka_uart_set_blocking(int fd, int should_block) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		PX4_ERR("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		PX4_ERR("error %d setting term attributes", errno);
}

void elka_uart_set_params() {
  _elka_uart_fd = -1;
  _elka_uart_open = false;
  strcpy(_elka_uart,"/dev/tty-1");
}

int8_t elka_uart_open() {
	PX4_INFO("Opening serial port %s", _elka_uart);
  _elka_uart_fd = ::open(_elka_uart, O_RDWR | O_NOCTTY);
  if (_elka_uart_fd > 0) {
    PX4_INFO("Opened serial port number %d",_elka_uart_fd);

    // Save current serial port settings
    tcgetattr(_elka_uart_fd,&_elka_uart_oldtio);
    usleep(10000);
		// Set baud to 38400 bps, 8n1 (no parity)
		elka_uart_set_interface_attribs(_elka_uart_fd,B38400,0);
    usleep(5000);
		// Set no blocking
		elka_uart_set_blocking(_elka_uart_fd, 0);
    usleep(5000);
  } else {
    //FIXME log error!
    PX4_INFO("Error opening serial port");
    _elka_uart_fd = PX4_ERROR;
    _elka_uart_open = false;
    return PX4_ERROR;
  }
  _elka_uart_open = true;
	return PX4_OK;
}

int8_t elka_uart_close() {
	PX4_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(_elka_uart_fd,TCSANOW,&_elka_uart_oldtio);

  usleep(5000);
  _elka_uart_open = false;
  if (!::close(_elka_uart_fd)) {
    _elka_uart_fd = -1;
    PX4_INFO("Successfully closed serial port %s", _elka_uart);
  } else {
    PX4_INFO("Error closing serial port");
    _elka_uart_fd = PX4_ERROR;
    return PX4_ERROR;
  }

	return PX4_OK;
}

int8_t elka_uart_send(elka_packet_s *pkt) {

  uint8_t open_file_error_count = 0;
  while (!_elka_uart_open) {
    elka_uart_open();
    usleep(5000);
    open_file_error_count++;
    if (open_file_error_count == 100) {
      PX4_ERR("Failed to open elka uart file");
      _thread_running = false;
      return PX4_ERROR;
    }
  }

  int num_bytes_written = 0;


  memcpy(_elka_uart_buf,pkt->data,pkt->len);

  num_bytes_written = write(_elka_uart_fd,
      (const char *)_elka_uart_buf,
      _elka_uart_buf[0]+1);

  if (num_bytes_written == _elka_uart_buf[0]+1) {
    //Reset packet len byte
    _elka_uart_buf[0]=0;
#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL_WRITE)
    PX4_INFO("Wrote %d bytes to %s", num_bytes_written,_elka_uart);
    print_array((const uint8_t *)_elka_uart_buf,
        _elka_uart_buf[1]);
#endif
  } else {
    PX4_ERR("Failed to write to %s",_elka_uart);
    PX4_INFO("Closing file %s",_elka_uart);
    elka_uart_close();
    return PX4_ERROR;
  }

	return PX4_OK;
}
