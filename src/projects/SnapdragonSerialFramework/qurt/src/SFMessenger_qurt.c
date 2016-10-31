#include <dev_fs_lib_serial.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* process-specific header files */
#include "../inc/dsp_log.h"
#include "../../common/status.h"

/* interface header files */
#include "uart_interface.h"

// TODO shared memory stuff

#define ELKA_SERIAL_PATH "/dev/tty-2" // J13 module

#define SERIAL_DATA_BUFFER_LENGTH  128
#define SERIAL_WRITE_DELAY_IN_USECS (8000*10)

#define MAX_UART_DEVICE_NUM 6
#define NUM_UART_DEVICE_ENABLED 4


int serial_fd = -1;

//using namespace SerialFramework;

//TODO
uint32 uart_interface_esc_read(){return ERROR;}
uint32 uart_interface_esc_write(){return ERROR;}

/* 
 * Call this callback upon request to send a message to ELKA
 * Write to serial port each time it is requested
 * Return SUCCESS on successful write/read, FAILURE on failed write/read
 */
uint32 uart_interface_esc_write_read() {
	char tx_buffer[SERIAL_DATA_BUFFER_LENGTH];
	char rx_buffer[SERIAL_DATA_BUFFER_LENGTH];
	unsigned int num_bytes_written = 0;
  int result = SUCCESS;
  int num_bytes_read = 0;
	int active_devices = 0;

	LOG_INFO("Beginning esc write read callback");

	// Try to open uart port
	serial_fd = open(ELKA_SERIAL_PATH, O_RDWR);
	LOG_INFO("Opening %s O_RDWR mode %s",
		ELKA_SERIAL_PATH,
		(serial_fd < SUCCESS) ? "fail" : "succeed");

	// Write and read from open serial port
	active_devices = 0;
	
	if (!(serial_fd < SUCCESS)) {

		memset(rx_buffer, 0, SERIAL_DATA_BUFFER_LENGTH);
		sprintf(tx_buffer, "Message from %s\n", ELKA_SERIAL_PATH);

		num_bytes_written = write(serial_fd, (const char *) tx_buffer,
			strlen(tx_buffer));

		if (num_bytes_written == strlen(tx_buffer)) {
			LOG_DEBUG("Written %d bytes to %s", num_bytes_written,
				ELKA_SERIAL_PATH);
			active_devices++;

			memset(rx_buffer, 0, SERIAL_DATA_BUFFER_LENGTH);
			num_bytes_read = read(serial_fd, rx_buffer, SERIAL_DATA_BUFFER_LENGTH);
			LOG_DEBUG("%s read bytes [%d]: %s",
				ELKA_SERIAL_PATH, num_bytes_read, rx_buffer);

		} else {
			LOG_ERR("Failed to write to %s", ELKA_SERIAL_PATH);
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

	LOG_INFO("ESC write/read %s",
		result == SUCCESS ? "Success" : "Failed");

	return result;

}
