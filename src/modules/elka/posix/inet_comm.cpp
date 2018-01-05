#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_log.h>
#include <px4_defines.h>

#include "inet_comm.h"

// _sockfd = file descriptor returned by socket system call
//           [0] is client-side
//           [1] is server-side
// _newsockfd = file descriptor returned by accept system call
// _portno = port number on which the server (this dev) accepts cxns
//           [0] is client-side
//           [1] is server-side
int _sockfd[2], _newsockfd, _portno[2];

// _clilen = size of address of client. Needed for accept system call
socklen_t _clilen;

// Buffers for read/write calls
// buffer [0] is client-side
// buffer [1] is server-side
uint8_t _tx_buffer[2][MAX_MSG_LEN], _rx_buffer[2][MAX_MSG_LEN];

// A sockaddr_in is a structure containing an internet address
//  {
//    short sin_family; Must be AF_INET
//    u_short sin_port;
//    struct  in_addr sin_addr;
//    char    sin_zero[8]; Not used, must be 0
//  }
//  _serv_addr[0] is client-side repr of server addr
//  _serv_addr[1] is server-side repr of server addr
struct sockaddr_in _serv_addr[2], _cli_addr;

struct hostent *_server;

struct in_addr _ipaddr;

// Num bytes read/written on read/write call
int _num_bytes;

// Send to ELKA HW
elka::SerialBuffer *_tx_sb;
// Receive from ELKA HW
elka::SerialBuffer *_rx_sb;

void set_sock_opts(int fd, uint8_t sock_side) {
  if (sock_side == SERVER) {
    int reuseaddr_opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR,
               (const char *) &reuseaddr_opt, sizeof(int));

/*
    struct timeval timeout_struct;
    timeout_struct.tv_sec = 5;
    timeout_struct.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
               &timeout_struct, sizeof(struct timeval));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
               &timeout_struct, sizeof(struct timeval));
*/
  } else if (sock_side == CLIENT) {
/*
    struct timeval timeout_struct;
    timeout_struct.tv_sec = 5;
    timeout_struct.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
               &timeout_struct, sizeof(struct timeval));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
               &timeout_struct, sizeof(struct timeval));
*/
  } else {
    PX4_ERR("socket side must be CLIENT or SERVER");
  }
}

int socket_open(
    int portno,
    const char *hostaddr,
    uint8_t sock_side) {
  if (sock_side == SERVER) {

    // User must pass in port number on which the server will accept
    // connections
    _portno[1] = portno;
    
    // Create new socket:
    //    AF_INET -> Internet domain 
    //               Other choice is UNIX if two processes share a common
    //               file system
    //    SOCK_STREAM -> Data read in a bytewise stream
    //                   Other choice is SOCK_DGRAM, which chunks
    //                   messages
    //    0 -> Protocol
    //         TCP for SOCK_STREAM
    //         UDP for SOCK_DGRAM
    _sockfd[1] = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_sockfd[1] < 0) {
      PX4_ERR("Error opening socket");
      return PX4_ERROR;
    } 

    memset(&_serv_addr[1], 0, sizeof(_serv_addr[1]));

    // Socket address family
    _serv_addr[1].sin_family = AF_INET;
    // IP address. INADDR_ANY gets IP address of the server machine
    _serv_addr[1].sin_addr.s_addr = INADDR_ANY;
    // Port number converted to network byte order
    _serv_addr[1].sin_port = htons(_portno[1]);
    
    set_sock_opts(_sockfd[1], sock_side);

    // Bind socket to an address
    if (bind(_sockfd[1], (struct sockaddr *) &_serv_addr[1],
          sizeof(_serv_addr[1])) < 0) {
      PX4_ERR("Error binding");
      return PX4_ERROR;
    }

    // Listen on the socket for connections.
    // 5 is the size of the backlog queue, which determines the number
    // of connections that can be waiting while the process is handling
    // a particular connection
    listen(_sockfd[1], 5);

    _clilen = sizeof(_cli_addr);

    // FIXME should not block
    // Causes the process to block until a client connects to a server
    // Returns a new file descriptor, on which all communication on this
    // connection should be done
    _newsockfd = accept(_sockfd[1],
        (struct sockaddr *) &_cli_addr,
        &_clilen);
    
    if (_newsockfd < 0) {
      PX4_ERR("Error on accept");
      return PX4_ERROR;
    }

    memset(_tx_buffer[1], 0, MAX_MSG_LEN);
    memset(_rx_buffer[1], 0, MAX_MSG_LEN);

    return _newsockfd;
  } else if (sock_side == CLIENT) {
    _portno[0] = portno;

    _sockfd[0] = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_sockfd[0] < 0) {
      PX4_ERR("Error opening socket");
      return PX4_ERROR;
    }

    memset(&_serv_addr[0], 0, sizeof(_serv_addr[0]));

		/*FIXME get client to attach to host dynamically
    inet_pton(AF_INET, hostaddr, &_ipaddr);
    _server = gethostbyaddr(&_ipaddr, sizeof(struct in_addr), AF_INET);
    if (!_server) {
      PX4_ERR("Error, no such host");
      return PX4_ERROR;
    }

    memcpy(_server->h_addr,
           &_serv_addr[0].sin_addr.s_addr,
           _server->h_length);
		*/

		//FIXME debugging: hardcoded ip addr of host
		//_serv_addr[0].sin_addr.s_addr = inet_addr("192.168.1.1");
		_serv_addr[0].sin_addr.s_addr = inet_addr(hostaddr);
		
    _serv_addr[0].sin_family = AF_INET;
    _serv_addr[0].sin_port = htons(_portno[0]);

    if (connect(_sockfd[0],
                (struct sockaddr *) &_serv_addr[0],
                sizeof(_serv_addr[0])) < 0) {
      PX4_ERR("Error connecting");
      return PX4_ERROR;
    }

    memset(_tx_buffer[0], 0, MAX_MSG_LEN);
    memset(_rx_buffer[0], 0, MAX_MSG_LEN);

    return _sockfd[0];
  } else {
    PX4_ERR("socket side must be CLIENT or SERVER");
    return PX4_ERROR;
  }

}

int socket_close(int fd, uint8_t sock_side) {

  if (sock_side == SERVER) {
    close(fd);
    close(_sockfd[1]);

    _sockfd[1] = -1;
    _newsockfd = -1;
  } else if (sock_side == CLIENT) {
    close(fd);
    _sockfd[0] = -1;
  } else {
    PX4_ERR("socket side must be CLIENT or SERVER");
    return PX4_ERROR;
  }

  return PX4_OK;
}

int socket_read(
    int fd,
    uint8_t* rx_buffer,
    uint8_t len,
    uint8_t sock_side) {
  if (sock_side == SERVER) {
    memset(_rx_buffer[1],0,len);
    memset(rx_buffer,0,len);
    _num_bytes = read(fd, _rx_buffer[1],
        len < MAX_MSG_LEN ? len : MAX_MSG_LEN);

    if (_num_bytes < 0)
      PX4_ERR("Error reading from socket");
    else 
      memcpy(rx_buffer, _rx_buffer[1], _num_bytes);
  } else if (sock_side == CLIENT) {
    _num_bytes = read(fd, _rx_buffer[0],
        len < MAX_MSG_LEN ? len : MAX_MSG_LEN);

    if (_num_bytes < 0)
      PX4_ERR("Error reading from socket");
    else 
      memcpy(rx_buffer, _rx_buffer[0], _num_bytes);
  } else {
    PX4_ERR("socket side must be CLIENT or SERVER");
    return PX4_ERROR;
  }

  /*
  char buf[1024];
  array_to_cstring(buf, rx_buffer, _num_bytes);
  PX4_INFO("Reading %d bytes: %s", _num_bytes, buf);
  */

  return _num_bytes;
}

int socket_write(
    int fd,
    uint8_t* tx_buffer,
    uint8_t len,
    uint8_t sock_side) {

  _num_bytes = len < MAX_MSG_LEN - 1 ? len : MAX_MSG_LEN;

  if (sock_side == SERVER) {
    memcpy(_tx_buffer[1], tx_buffer, _num_bytes);

    _num_bytes = write(fd, _tx_buffer[1], _num_bytes);

    if (_num_bytes < 0)
      PX4_ERR("Error writing to socket");
  } else if (sock_side == CLIENT) {
    memcpy(_tx_buffer[0], tx_buffer, _num_bytes);

    _num_bytes = write(fd, _tx_buffer[0], _num_bytes);

    if (_num_bytes < 0)
      PX4_ERR("Error writing to socket");
  } else {
    PX4_ERR("socket side must be CLIENT or SERVER");
    return PX4_ERROR;
  }

  /*
  char buf[1024];
  array_to_cstring(buf, tx_buffer, _num_bytes);
  PX4_INFO("Writing %d bytes: %s", _num_bytes, buf);
  */

  return _num_bytes;
}

int socket_proc_start(
		Child *child,
		const char *hostaddr,
		uint8_t sock_side,
    elka::SerialBuffer *tx_sb,
    elka::SerialBuffer *rx_sb) {
	
	if ((child->pid = fork()) < 0) {
		return PX4_ERROR;	
	} else if (child->pid == 0) {
		socket_loop(hostaddr, sock_side,
                tx_sb, rx_sb);
		exit(0);
	} else {
		return PX4_OK;
	}

	return PX4_OK;
}

int socket_loop(
		const char *hostaddr,
		uint8_t sock_side,
    elka::SerialBuffer *tx_sb,
    elka::SerialBuffer *rx_sb) {
	int sock_fd;

  _tx_sb = tx_sb;
  _rx_sb = rx_sb;

  if (sock_side == SERVER) {
    PX4_INFO("Opening socket as server");
  } else if (sock_side == CLIENT) {
    PX4_INFO("Opening socket as client");
  } else {
    PX4_ERR("Socket side must be either `client` or `server`");
    return PX4_ERROR;
  }

  sock_fd = socket_open(7, hostaddr, sock_side);

  PX4_INFO("socket opened");

  while(socket_read_elka_msg(sock_fd, sock_side) >= 0) {
    PX4_INFO("derf");
    sleep(1);
  }

  PX4_INFO("Closing server socket");
  if (socket_close(sock_fd, sock_side) == PX4_OK)
    sock_fd = -1;
  else {
    PX4_ERR("Error closing socket");
    return PX4_ERROR;
  }

  _tx_sb = nullptr;
  _rx_sb = nullptr;

	return 0;
}

int socket_read_elka_msg(
    int fd,
    uint8_t sock_side) {
  msg_id_t msg_id = 0;
  uint16_t msg_num = 0;
  int i = 0, num_bytes;
  size_t msg_id_sz = sizeof(msg_id), msg_num_sz = sizeof(msg_num);
  uint8_t buffer[MAX_MSG_LEN];
  uint8_t num_retries;

  if ( (num_bytes = socket_read(
                      fd,
                      buffer,
                      MAX_MSG_LEN,
                      sock_side))
        < 0 ) {
    PX4_ERR("Error reading from socket");
    return PX4_ERROR;

  } else if (num_bytes > 0) {

    PX4_INFO("performing socket read cb"); fflush(stdout);

    // Parse in msg id and msg num
    // Bitshifting in LSB first format
    while (i < msg_id_sz && i < num_bytes) {
      msg_id |= buffer[i] << (i*8);
      i++;
    }
    while (i < msg_num_sz && i < num_bytes) {
      msg_num |= buffer[i] << ((i-msg_id_sz)*8);
      msg_num |= ((uint8_t)(*buffer) << ((i-msg_id_sz)*8));
      i++;
    }

    num_retries = buffer[i++];

    _rx_sb->push_msg(
        msg_id,
        buffer,
        msg_num,
        num_retries);

    if (i >= num_bytes) {
      PX4_ERR("error: read callback with not enough data in the buffer");
    }
  }

  return num_bytes;
}

int socket_write_elka_msg(
    int fd,
    elka_msg_s &elka_msg,
    uint8_t sock_side) {
  static uint16_t serial_msg_len = (uint16_t)MAX_MSG_LEN
                                  +(uint16_t)elka_msg_s::MSG_OFFSET;
  uint8_t serial_msg[serial_msg_len];
  uint8_t len;
  
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,&len,
      elka_msg.msg_id);

  // Write serial message with output from serialization
  serialize_elka_msg(serial_msg, elka_msg);

  return socket_write(fd,
                      serial_msg,
                      len + elka_msg_s::MSG_OFFSET,
                      sock_side);
}

int socket_write_elka_msg(
    int fd,
    elka_msg_ack_s &elka_msg,
    uint8_t sock_side) {
  uint8_t serial_msg[elka_msg_ack_s::ACK_LEN];

  // Write serial message with output from serialization and
  // length of elka_msg_ack
  serialize_elka_msg_ack(serial_msg, elka_msg);

  return socket_write(fd,
                      serial_msg,
                      elka_msg_ack_s::ACK_LEN,
                      sock_side);
}
