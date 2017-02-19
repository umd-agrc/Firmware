#include <dev_fs_lib_serial.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <px4_log.h>

#include "basic_uart.h"
#include "platform.h"
#include "status.h"

const char *serial_path[MAX_UART_DEV_NUM] = {
  "/dev/tty-1", "/dev/tty-2",
  "/dev/tty-3", "/dev/tty-4",
  "/dev/tty-5", "/dev/tty-6"
};

struct termios oldtio;

void port_read_callback(void *context, char *buffer, size_t num_bytes) {
  int rx_dev_id = (int)context;
  char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

  if (num_bytes > 0) {
    memcpy(rx_buffer, buffer, num_bytes);
    rx_buffer[num_bytes] = 0;
    PX4_INFO("/dev/tty-%d read callback received bytes [%d]: %s",
      rx_dev_id, num_bytes, rx_buffer);
  } else {
    PX4_ERR("error: read callback with no data in the buffer");
  }
}

int set_interface_attribs(int fd, int baud, int parity) {
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
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block) {
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

int serial_open(int port_num) {
  int serial_fd;
  PX4_INFO("Opening serial port");
  serial_fd = open(serial_path[port_num-1], O_RDWR | O_NOCTTY);
  if (serial_fd >= SUCCESS) {
    PX4_INFO("Opened serial port %s", serial_path[port_num-1]);

    // Save current serial port settings
    tcgetattr(serial_fd,&oldtio);
		// Set baud to 115200 bps, 8n1 (no parity)
		set_interface_attribs(serial_fd,B115200,0);
		// Set no blocking
		set_blocking(serial_fd, 0);
  } else {
  //FIXME log error!
	PX4_INFO("Error opening serial port");
    serial_fd = ERROR;
  }

  return serial_fd;
}

int serial_close(int fd) {
	PX4_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(fd,TCSANOW,&oldtio);

  if (!close(fd)) {
    PX4_INFO("Successfully closed serial port number %d", fd);
  } else {
    PX4_INFO("Error closing serial port");
    fd = ERROR;
  }

  return fd;
}

int assign_serial_read_callback(int fd, int port_num) {
  int res;

  PX4_INFO("Beginning serial read callback setup");

  struct dspal_serial_ioctl_receive_data_callback recv_cb;
  recv_cb.rx_data_callback_func_ptr = port_read_callback;

  recv_cb.context = (void *)(1);

  res = ioctl(fd,
      SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
      (void *)&recv_cb);

  PX4_INFO("Using callback on fd %d",fd);
  PX4_INFO("Set serial read callback on %s %s",
    serial_path[port_num-1], res < SUCCESS ? "failed" : "succeeded");

  if (res < SUCCESS) {
    //TODO associate with any file if necessary
    PX4_INFO("Closing file %s",
      serial_path[port_num-1]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_read(int fd, int port_num, char *rx_buffer) {
  int num_bytes_read = 0;

  PX4_INFO("Beginning serial read");
  
  num_bytes_read = read(fd, rx_buffer,
      SERIAL_SIZE_OF_DATA_BUFFER);
  PX4_INFO("%s read bytes [%d]: %s",
      serial_path[port_num-1], num_bytes_read, rx_buffer);

  if (num_bytes_read < 0) {
    //TODO associate with any file if necessary
    PX4_INFO("Closing file %s",
      serial_path[port_num]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_write(int fd, int port_num, char *tx_buffer){
  int num_bytes_written = 0;

  PX4_INFO("Beginning serial write");

  num_bytes_written = write(fd,
      (const char *)tx_buffer,
      strlen(tx_buffer));

  if (num_bytes_written == (ssize_t)strlen(tx_buffer)) {
    //TODO associate with any file if necessary
    PX4_INFO("Wrote %d bytes to %s", num_bytes_written,
        serial_path[port_num-1]);
  } else {
    //TODO associate with any file if necessary
    PX4_ERR("failed to write to %s", serial_path[port_num]);
    PX4_INFO("Closing file %s", serial_path[port_num]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_read_write(int fd, int port_num,
    char *rx_buffer, char *tx_buffer) {
  int res = fd;
  if (serial_write(fd, port_num, tx_buffer) == fd) {
		usleep(SERIAL_SIZE_OF_DATA_BUFFER*100);
		if (serial_read(fd, port_num, rx_buffer) != fd) {
      res = ERROR;
		} 
	} else {
		res = ERROR;
	}
  return res;
}
