#ifndef INET_COMM
#define INET_COMM

#include <elka/common/elka.h>
#include <elka/common/elka_comm.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#include "elka_posix.h"

#define CLIENT 0
#define SERVER 1

#define CLIENT_SIDE "client"
#define SERVER_SIDE "server"

typedef struct Child {
	FILE *fp_to;
	FILE *fp_from;
	pid_t pid;
} Child;

// Socket methods
// Open/close/read/write sockets for inet access

// sock_side = CLIENT or SERVER

void set_sock_opts(int fd, uint8_t sock_side);

int socket_open(
    int portno,
    const char *hostname,
    uint8_t sock_side);

int socket_close(int fd, uint8_t sock_side);

int socket_read(
    int fd,
    uint8_t* rx_buffer,
    uint8_t len,
    uint8_t sock_side);

int socket_write(
    int fd,
    uint8_t* tx_buffer,
    uint8_t len,
    uint8_t sock_side); 

int socket_proc_start(
		Child *child,
		const char *hostaddr,
		uint8_t sock_side,
    elka::SerialBuffer *tx_sb,
    elka::SerialBuffer *rx_sb);

int socket_loop(
		const char *hostaddr,
		uint8_t sock_side,
    elka::SerialBuffer *tx_sb,
    elka::SerialBuffer *rx_sb);

int socket_read_elka_msg(
    int fd,
    uint8_t sock_side);

int socket_write_elka_msg(
    int fd,
    elka_msg_s &elka_msg,
    uint8_t sock_side);

int socket_write_elka_msg(
    int fd,
    elka_msg_ack_s &elka_msg,
    uint8_t sock_side);

#endif
