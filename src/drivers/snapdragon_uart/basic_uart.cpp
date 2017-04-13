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
#include <px4_defines.h>

#include "basic_uart.h"
#include "platform.h"
#include "status.h"

//FIXME
//#include "c-ringbuf/ringbuf.h"

struct buf {
  uint8_t type;
  void *buffer;
};

const char *serial_path[MAX_UART_DEV_NUM] = {
  "/dev/tty-1", "/dev/tty-2",
  "/dev/tty-3", "/dev/tty-4",
  "/dev/tty-5", "/dev/tty-6"
};

struct termios oldtio;

/*
void *_tx_buffer[MAX_UART_PORTS];
void *_rx_buffer[MAX_UART_PORTS];
*/
buffer _tx[MAX_UART_PORTS];
buffer _rx[MAX_UART_PORTS];

//FIXME should use read methods of ringbuf
void port_read_callback(void *context, char *buffer, size_t num_bytes) {
  int rx_dev_id = (int)context;

  if (num_bytes > 0) {
    memcpy_into(&_rx[rx_dev_id], buffer, num_bytes);
    PX4_INFO("/dev/tty-%d read callback received bytes [%d]:",
      rx_dev_id, num_bytes);
    buffer_print_next_msg(&_rx[rx_dev_id]);
  } else {
    PX4_ERR("error: read callback with no data in the buffer");
  }
}

void buffer_print_next_msg(buffer *buf) {
  if (!buf || !buf->buffer) {
    PX4_ERR("Can't print UART message. Buffer does not exist.");
  } if (buf->type == CHAR_ARRAY) {
    print_char_array((char *)buf->buffer);
  } else if (buf->type == UINT8_ARRAY) {
    print_uint8_array((uint8_t *)buf->buffer);
  //TODO
  } else if (buf->type == RINGBUF) {
  } else if (buf->type == NO_BUF) {
  } else {
    PX4_ERR("Can't print UART message. Buffer type undefined.");
  }
}

int memcpy_into(buffer *dst, char *src, size_t num_bytes) {
  if (dst->type == CHAR_ARRAY ||
      dst->type == UINT8_ARRAY) {
    memcpy(dst->buffer, src, num_bytes);
  } else if (dst->type == RINGBUF) {
    //TODO
    return PX4_ERROR;
  } else if (dst->type == NO_BUF) {
    PX4_INFO("Can't copy UART message. No buffer available!");
    return PX4_ERROR;
  } else {
    PX4_ERR("Can't copy UART message. Buffer type undefined");
    return PX4_ERROR;
  }
  return PX4_OK;
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

int serial_open(int port_num,
    void *tx_buf, uint8_t tx_buf_type,
    void *rx_buf, uint8_t rx_buf_type) {
  int serial_fd;
  serial_fd = open(serial_path[port_num-1], O_RDWR | O_NOCTTY);
  if (serial_fd >= SUCCESS) {
    PX4_INFO("Opened serial port %s", serial_path[port_num-1]);

    // Save current serial port settings
    tcgetattr(serial_fd,&oldtio);
		// Set baud to 115200 bps, 8n1 (no parity)
		set_interface_attribs(serial_fd,B115200,0);
		// Set no blocking
		set_blocking(serial_fd, 0);

    //FIXME have to assign with &buf[0]?
    _tx[port_num].type = tx_buf_type;
    _tx[port_num].buffer = tx_buf;
    _rx[port_num].type = rx_buf_type;
    _rx[port_num].buffer = rx_buf;

  } else {
    //FIXME log error!
    PX4_ERR("Error opening serial port");
    serial_fd = ERROR;
  }

  return serial_fd;
}

int serial_close(int fd, int port_num) {
	PX4_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(fd,TCSANOW,&oldtio);

  if (!close(fd)) {
    PX4_INFO("Successfully closed serial port number %d", fd);

    _tx[port_num].type = 0;
    _tx[port_num].buffer = NULL;
    _rx[port_num].type = 0;
    _rx[port_num].buffer = NULL;
  } else {
    PX4_INFO("Error closing serial port");
    fd = ERROR;
  }

  return fd;
}

//FIXME may have to set data rate with 
// SERIAL_IOCTL_SET_DATA_RATE and DSPAL_SIO_BITRATE_115200
int assign_serial_read_callback(int fd, int port_num) {
  int res;

  PX4_INFO("Beginning serial read callback setup");

  struct dspal_serial_ioctl_receive_data_callback recv_cb;
  recv_cb.rx_data_callback_func_ptr = port_read_callback;

  recv_cb.context = (void *)(port_num);

  res = ioctl(fd,
      SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
      (void *)&recv_cb);

  PX4_INFO("Using callback on fd %d",fd);
  PX4_INFO("Set serial read callback on %s %s",
    serial_path[port_num-1], res < SUCCESS ? "failed" : "succeeded");

  if (res < SUCCESS) {
    PX4_INFO("Closing file %s",
      serial_path[port_num-1]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_read(int fd, int port_num, uint8_t *rx_buffer) {
  int num_bytes_read = 0;

  PX4_INFO("Beginning serial read");
  
  num_bytes_read = read(fd, rx_buffer,
      SERIAL_SIZE_OF_DATA_BUFFER);
  PX4_INFO("%s read bytes [%d]: %s",
      serial_path[port_num-1], num_bytes_read, rx_buffer);

  if (num_bytes_read < 0) {
    PX4_INFO("Closing file %s",
      serial_path[port_num]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_write(int fd, int port_num,
    uint8_t *tx_buffer, uint8_t tx_buffer_len){
  int num_bytes_written = 0;

  PX4_INFO("Beginning serial write");

  print_uint8_array(tx_buffer);
  num_bytes_written = write(fd,
      (const uint8_t *)tx_buffer,
      tx_buffer_len);

  if (num_bytes_written == (ssize_t)tx_buffer_len) {
    PX4_INFO("Wrote %d bytes to %s", num_bytes_written,
        serial_path[port_num-1]);
  } else {
    PX4_ERR("failed to write to %s", serial_path[port_num]);
    PX4_INFO("Closing file %s", serial_path[port_num]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_read_write(int fd, int port_num,
    uint8_t *rx_buffer, uint8_t *tx_buffer, uint8_t tx_buffer_len) {
  int res = fd;
  if (serial_write(fd, port_num, tx_buffer, tx_buffer_len) == fd) {
		usleep(SERIAL_SIZE_OF_DATA_BUFFER*100);
		if (serial_read(fd, port_num, rx_buffer) != fd) {
      res = ERROR;
		} 
	} else {
		res = ERROR;
	}
  return res;
}
