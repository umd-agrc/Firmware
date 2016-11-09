#include "modules/uORB/uORB.h"
#include "px4_middleware.h"
#include "px4_defines.h"

#include <dev_fs_lib_serial.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <px4_log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* process-specific header files */
#include "../inc/SFUartDriver.h"
#include "../../common/status.h"
#include <projects/sysdefs.h>

px4::AppState SFUartDriver::appState;

/////// Public methods ////////////
SFUartDriver::SFUartDriver(const char* port){
  memset(dev_port,0,sizeof(dev_port));
  m_dev_port = port;
}

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

  if (uart_initialize(m_dev_port) < 0) {
    PX4_ERR("Failed to initialize UART.");
    return PX4_ERROR;
  }

  // Set up subscribers
  // Set these up first to avoid deadlock
  while ((m_sub_vc = orb_subscribe(ORB_ID(vehicle_command)))
            == PX4_ERROR && !appState.exitRequested()) {
    PX4_ERR("Error subscribing to vehicle_command topic, retrying");
  }

  // Set up publishers
  m_pub_vca =
    orb_advertise(ORB_ID(vehicle_command_ack), &m_vca);

  if (m_pub_vca == 0) {
    PX4_ERR("Error publishing vehicle_command_ack");
    return PX4_ERROR;
  }

  int i=0;
  while (!appState.exitRequested()) {
    bool updated = false;

    if(orb_check(m_sub_vc, &updated) == 0) {
      if (updated) {
        PX4_DEBUG("[%d] vehicle_command is updated... reading new value",i);
        if (orb_copy(ORB_ID(vehicle_command), m_sub_vc, &m_vc) != 0) {
          m_vca.command = vehicle_command_s::VEHICLE_CMD_RESULT_FAILED;
          m_vca.result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
          PX4_ERR("[%d] Error calling orb copy for vehicle_command... ",i);
          break;
        } else {
          if (m_vc.target_system == SYS_SFUART) {
            m_vca.command = m_vc.command;
            m_vca.result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;
          }
        }
      } else {
        PX4_DEBUG("[%d] vehicle_command is not updated", i);
        // No ack necessary
      }

      if (m_pub_vca != nullptr) {
        orb_publish(ORB_ID(vehicle_command_ack), m_pub_vca, &m_vca);
      } else { // should not happen, should exit before this!
        m_pub_vca =
          orb_advertise(ORB_ID(vehicle_command_ack), &m_vca);

        if (m_pub_vca == 0) {
          PX4_ERR("Error publishing vehicle_command_ack");
          return PX4_ERROR;
        }
      }
    } else {
      PX4_ERR(
        "[%d] Error checking the updated status for vehicle command...",i);
      break;
    }
    ++i;
    usleep(100000);
  }

  appState.setRunning(false);
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
			result = PX4_ERROR;
		}

	} else {

		result = PX4_ERROR;

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

/////// Private methods ////////////
//TODO
void SFUartDriver::cb_serial(void *context, char *buffer, size_t num_bytes) {
  
}

uint32_t SFUartDriver::health(){return PX4_ERROR;}

int SFUartDriver::uart_initialize(const char *device){
  // FIXME uncomment once other side of UART is implemented
  //m_fd = open(device, O_RDWR | O_NONBLOCK);
  m_fd = open(device, O_RDWR);

  if (m_fd == -1) {
    PX4_ERR("Failed to open UART.");
    return ERR_OPEN_UART;
  }

  struct dspal_serial_ioctl_data_rate rate;

  rate.bit_rate = DSPAL_SIO_BITRATE_115200;

  int ret = ioctl(m_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);

  if (ret) {
    PX4_ERR("Failed to set UART bitrate.");
    return ERR_SET_DATA_RATE;
  }

  struct dspal_serial_ioctl_receive_data_callback cb;

  cb.rx_data_callback_func_ptr = cb_serial;
  ret = ioctl(m_fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&cb);

  if (ret) {
    PX4_ERR("Failed to setup UART flow control options.");
    return ERR_UART_FLOW_CTL;
  }

  return SUCCESS;
}

int SFUartDriver::uart_deinitialize() {
  return close(m_fd);
}
