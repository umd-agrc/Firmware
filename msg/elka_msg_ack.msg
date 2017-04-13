uint16 MAX_MSG_NUM = 65535

uint8 ACK_NULL = 0 # not for u
uint8 ACK_ACCEPTED = 1 # success
uint8 ACK_DENIED = 2 # message correct, not accepted
uint8 ACK_UNSUPPORTED = 3 # not an option
uint8 ACK_FAILED = 4 # message not correct 

uint32 msg_id
uint16 msg_num # To be checked against sender
uint8 result

# Bitwise values for elka_msg defined in elka.h
## msg_id
## 8b sender id , 8b receiver id , 8b msg_type , 8b length
### sender_id = (msg_id & SENDER_ID) >> 24
### receiver_id = (msg_id & RECEIVER_ID) >> 16
### type = (msg_id & MESSAGE_TYPE) >> 8
#### expecting_ack = (msg_id & ID_EXPECTING ACK) >> 8
#### expecting_ack = (msg_type & TYPE_EXPECTING_ACK)
### length = (msg_id & MESSAGE_LENGTH)
#### length of MSG_ACK is 1

## Sender/receiver id format (uint8_t)
## 4b port_num , 3b port_type , 1b proc_side
### port_num = (id & PORT_NUM) >> 5
### port_type = (id & PORT_TYPE) >> 2
### proc_side = (id & PROC_SIDE)
