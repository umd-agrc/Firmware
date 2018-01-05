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
  "/dev/tty-4", "/dev/tty-1",
  "/dev/tty-3", "/dev/tty-2",
  "/dev/tty-5", "/dev/tty-6"
};

int serial_fd = -1;
struct termios spektrum_oldtio;


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


//TODO take des argument
elka::SnapdragonSerialMessenger::SnapdragonSerialMessenger(char *des){
  PX4_INFO("Beginning serial read callback setup");
  strcpy(_des,des);

  if (open() == SERIAL_ERROR)
    PX4_ERR("Unable to create serial messenger");

  struct dspal_serial_ioctl_receive_data_callback recv_cb;
  recv_cb.rx_data_callback_func_ptr =
    elka::SnapdragonSerialMessenger::read_cb_helper;

  recv_cb.context = (void *)(this);

  int res = ioctl(_fd,
      SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
      (void *)&recv_cb);

  PX4_INFO("Set serial read callback on %s %s",
    _des, res < 0 ? "failed" : "succeeded");

  if (res < ELKA_SUCCESS) {
    PX4_INFO("Closing file %s",
      _des);
    close();
    _fd = SERIAL_ERROR;
  }
}

void elka::SnapdragonSerialMessenger::read_cb_helper(
    void *context, char *buffer, size_t num_bytes){
  ((SnapdragonSerialMessenger *)context)->read_callback(
    buffer,num_bytes);
}

void elka::SnapdragonSerialMessenger::read_callback(
    char *buffer, size_t num_bytes) {
  memcpy(_rx_buf,buffer,num_bytes);
#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL_READ)
  PX4_INFO("Callback reading incoming message:");
  print_array(_rx_buf,num_bytes);
#endif
  _data_rdy=true;
}

int8_t elka::SnapdragonSerialMessenger::probe(void *available) {
    return SERIAL_ERROR;
}

int8_t elka::SnapdragonSerialMessenger::open() {
	PX4_INFO("Opening serial port");
  _fd = ::open((char *)_des, O_RDWR | O_NOCTTY);
  if (_fd > 0) {
    PX4_INFO("Opened serial port number %d",_fd);

    // Save current serial port settings
    tcgetattr(_fd,&spektrum_oldtio);
		// Set baud to 38400 bps, 8n1 (no parity)
		spektrum_set_interface_attribs(_fd,B38400,0);
		// Set no blocking
		spektrum_set_blocking(_fd, 0);
  } else {
    //FIXME log error!
    PX4_INFO("Error opening serial port");
    _fd = SERIAL_ERROR;
    return SERIAL_ERROR;
  }
	return ELKA_SUCCESS;
}

int8_t elka::SnapdragonSerialMessenger::close() {
	PX4_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(_fd,TCSANOW,&spektrum_oldtio);

  usleep(2000);
  if (!::close(_fd)) {
    PX4_INFO("Successfully closed serial port number %d", _fd);
  } else {
    PX4_INFO("Error closing serial port");
    _fd = SERIAL_ERROR;
    return SERIAL_ERROR;
  }

	return ELKA_SUCCESS;
}

int8_t elka::SnapdragonSerialMessenger::send(
    elka_packet_s *pkt) {
  int num_bytes_written = 0;

  //Reset packet len byte
  _tx_buf[ELKA_MSG_PACKET_LEN]=0;

  memcpy(_tx_buf,pkt->data,pkt->len);

  num_bytes_written = write(_fd,
      (const char *)_tx_buf,
      _tx_buf[ELKA_MSG_PACKET_LEN]+1);

  if (num_bytes_written == _tx_buf[ELKA_MSG_PACKET_LEN]+1) {
    //TODO associate with any file if necessary
#if defined(ELKA_DEBUG) && defined(DEBUG_SERIAL_WRITE)
    PX4_INFO("Wrote %d bytes to %s", num_bytes_written,_des);
    print_array((const uint8_t *)_tx_buf,
        _tx_buf[ELKA_MSG_PACKET_LEN]+1);
#endif
  } else {
    PX4_ERR("failed to write to %s",_des);
    PX4_INFO("Closing file %s",_des);
    close();
    return SERIAL_ERROR;
  }

	return ELKA_SUCCESS;
}

//TODO add to BasicMessageMgr _msgs queue
int8_t elka::SnapdragonSerialMessenger::recv() {
  int num_bytes_read = 0;

  PX4_INFO("Beginning serial read");
  
  num_bytes_read = read(_fd, _rx_buf,
      MAX_ARR_LEN);

  _data_rdy=true;

  PX4_INFO("%s read %d bytes:",
      _des, num_bytes_read);
  //spektrum_print_char_array(rx_buffer, num_bytes_read);

  if (num_bytes_read < 0) {
    //TODO associate with any file if necessary
    PX4_INFO("Closing file %s",
      _des);
    close();
    return SERIAL_ERROR;
  }

	return ELKA_SUCCESS;
}
