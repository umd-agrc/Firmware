#include <dev_fs_lib_serial.h>
#include <errno.h>
#include <fcntl.h>
#include <px4_log.h>
#include <px4_defines.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "basic_uart.h"

const char *spektrum_serial_path[MAX_NUM_SERIAL_DEVS] = {
  "/dev/tty-2", "/dev/tty-1",
  "/dev/tty-3", "/dev/tty-4",
  "/dev/tty-5", "/dev/tty-6"
};

int serial_fd = -1;
struct termios spektrum_oldtio;

void spektrum_port_read_callback(void *context, char *buffer, size_t num_bytes) {
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

int spektrum_set_interface_attribs(int fd, int baud, int parity) {
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
    PX4_ERROR;
	}
	return PX4_OK;
}

void spektrum_set_blocking(int fd, int should_block) {
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

void spektrum_print_array(const uint8_t *buf, uint8_t len) {
  char to_print[7*MAX_MSG_LEN+1],
       a_char[7]; // 3 chars max for a uint8 number and empty space
  uint8_t i=0;

  memset(to_print,0,7*MAX_MSG_LEN+1);

  while (i++ < len) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  PX4_INFO("array: %s",to_print);
}

void spektrum_print_char_array(const char *buf, uint8_t len) {
  PX4_INFO("char array");
  spektrum_print_array((const uint8_t *)buf,len);
}

int spektrum_serial_open() {
	PX4_INFO("Opening serial port");
  serial_fd = open(spektrum_serial_path[0], O_RDWR | O_NOCTTY);
  if (serial_fd >= PX4_OK) {
    PX4_INFO("Opened serial port number %d", serial_fd);

    // Save current serial port settings
    tcgetattr(serial_fd,&spektrum_oldtio);
		// Set baud to 115200 bps, 8n1 (no parity)
		spektrum_set_interface_attribs(serial_fd,B38400,0);
		// Set no blocking
		spektrum_set_blocking(serial_fd, 0);
  } else {
  //FIXME log error!
	PX4_INFO("Error opening serial port");
    serial_fd = PX4_ERROR;
  }
  return serial_fd;
}

int spektrum_serial_close(int fd) {
	PX4_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(fd,TCSANOW,&spektrum_oldtio);

  if (!close(fd)) {
    PX4_INFO("Successfully closed serial port number %d", fd);
  } else {
    PX4_INFO("Error closing serial port");
    fd = PX4_ERROR;
  }

  return fd;
}

int spektrum_serial_read_callback(int fd) {
  int res;

  PX4_INFO("Beginning serial read callback setup");

  struct dspal_serial_ioctl_receive_data_callback recv_cb;
  recv_cb.rx_data_callback_func_ptr = spektrum_port_read_callback;

  recv_cb.context = (void *)(1);

  res = ioctl(fd,
      SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
      (void *)&recv_cb);

  PX4_INFO("Set serial read callback on %s %s",
    spektrum_serial_path[0], res < PX4_OK ? "failed" : "succeeded");

  if (res < PX4_OK) {
    //TODO associate with any file if necessary
    PX4_INFO("Closing file %s",
      spektrum_serial_path[0]);
    close(fd);
    fd = PX4_ERROR;
  }

	return fd;
}

int spektrum_serial_read(int fd, char* rx_buffer) {
  int num_bytes_read = 0;

  PX4_INFO("Beginning serial read");
  
  num_bytes_read = read(fd, rx_buffer,
      SERIAL_SIZE_OF_DATA_BUFFER);
  PX4_INFO("%s read bytes [%d]:",
      spektrum_serial_path[0], num_bytes_read);
  //spektrum_print_char_array(rx_buffer, num_bytes_read);

  if (num_bytes_read < 0) {
    //TODO associate with any file if necessary
    PX4_INFO("Closing file %s",
      spektrum_serial_path[0]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int spektrum_serial_write(int fd, const char* tx_buffer,
    uint8_t tx_buf_len) {
  int num_bytes_written = 0;

  PX4_INFO("Beginning serial write");

  PX4_INFO("txbuflen: %d",tx_buf_len);
  spektrum_print_array((const uint8_t *)tx_buffer, tx_buf_len);

  num_bytes_written = write(fd,
      (const char *)tx_buffer,
      tx_buf_len);

  if (num_bytes_written == tx_buf_len) {
    //TODO associate with any file if necessary
    PX4_INFO("Wrote %d bytes to %s", num_bytes_written,
        spektrum_serial_path[0]);
  } else {
    //TODO associate with any file if necessary
    PX4_ERR("failed to write to %s", spektrum_serial_path[0]);
    PX4_INFO("Closing file %s", spektrum_serial_path[0]);
    close(fd);
    fd = PX4_ERROR;
  }

	return fd;
}

int spektrum_serial_read_write(int fd, char* rx_buffer,
    const char* tx_buffer, uint8_t tx_buf_len){
  int res = fd;
  if (spektrum_serial_write(fd, tx_buffer, tx_buf_len) == fd) {
		usleep(SERIAL_SIZE_OF_DATA_BUFFER*100);
		if (spektrum_serial_read(fd, rx_buffer) != fd) {
    res = PX4_ERROR;
		} 
	} else {
		res = PX4_ERROR;
	}
  return res;
}

