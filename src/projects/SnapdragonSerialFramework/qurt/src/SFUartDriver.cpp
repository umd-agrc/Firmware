#include "../inc/SFUartDriver.h"
#include "../../common/status.h"
#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "modules/uORB/uORB.h"
#include "px4_middleware.h"
#include "px4_defines.h"
#include <stdlib.h>
#include <drivers/drv_hrt.h>

px4::AppState SFUartDriver::appState;

SFUartDriver::SFUartDriver(char* port){
  memset(dev_port,0,sizeof(dev_port));
  strcpy(dev_port,port);
}

SFUartDriver::SFUartDriver(void){
  memset(dev_port,0,sizeof(dev_port));
  strcpy(dev_port,DEFAULT_ELKA_SERIAL_PATH);
}

SFUartDriver::~SFUartDriver(){}

uint32_t SFUartDriver::start(void){
  return ENOSYS;
}

int SFUartDriver::main(){
  appState.setRunning(true);
  return SUCCESS;
}

uint32_t SFUartDriver::test_port(void){
	char tx_buffer[SERIAL_DATA_BUFFER_LENGTH];
	char rx_buffer[SERIAL_DATA_BUFFER_LENGTH];
	unsigned int num_bytes_written = 0;
  int result = SUCCESS;
  int num_bytes_read = 0;
	int active_devices = 0;
  int serial_fd = -1;

	PX4_INFO("Beginning esc write read callback");

	// Try to open uart port
	serial_fd = open(DEFAULT_ELKA_SERIAL_PATH, O_RDWR);
	PX4_DEBUG("Opening %s O_RDWR mode %s",
		DEFAULT_ELKA_SERIAL_PATH,
		(serial_fd < SUCCESS) ? "fail" : "succeed");

	// Write and read from open serial port
	active_devices = 0;
	
	if (!(serial_fd < SUCCESS)) {

		memset(rx_buffer, 0, SERIAL_DATA_BUFFER_LENGTH);
		sprintf(tx_buffer, "Message from %s\n", DEFAULT_ELKA_SERIAL_PATH);

		num_bytes_written = write(serial_fd, (const char *) tx_buffer,
			strlen(tx_buffer));

		if (num_bytes_written == strlen(tx_buffer)) {
			PX4_DEBUG("Written %d bytes to %s", num_bytes_written,
				DEFAULT_ELKA_SERIAL_PATH);
			active_devices++;

			memset(rx_buffer, 0, SERIAL_DATA_BUFFER_LENGTH);
			num_bytes_read = read(serial_fd, rx_buffer, SERIAL_DATA_BUFFER_LENGTH);
			PX4_DEBUG("%s read bytes [%d]: %s",
				DEFAULT_ELKA_SERIAL_PATH, num_bytes_read, rx_buffer);

		} else {
			PX4_ERR("Failed to write to %s", DEFAULT_ELKA_SERIAL_PATH);
			close(serial_fd);
			result = ERROR;
		}

	} else {

		result = ERROR;

	}

	// Close all devices
	if (serial_fd >= SUCCESS) {
		close(serial_fd);
	}

	PX4_INFO("ESC write/read %s",
		result == SUCCESS ? "Success" : "Failed");

	return result;
}

uint32_t SFUartDriver::uart_read(char* buf, int len){
  return ENOSYS;
}

uint32_t SFUartDriver::uart_write(char* msg, int len){
  return ENOSYS;
}

uint32_t SFUartDriver::uart_write_read(
      char* msg, int msg_len, char* buf, int buf_len){
  return ENOSYS;
}
