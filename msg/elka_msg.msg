uint8 NUM_OUTPUTS = 4
uint8 MSG_OFFSET = 11 # Data starts after offset (header) bytes

#FIXME add separate id parameter for checking acks
uint64 msg_id
uint16 msg_num
uint8 num_retries
uint8[256] data

# Bitwise values for elka_msg defined in elka.h
## msg_id (64b
## 16b sender id , 16b receiver id ,
## 8b sender params , 8b receiver params ,
## 8b msg_type , 8b length
### snd_id = (msg_id & SENDER_ID) >> 48
### rcv_id = (msg_id & RECEIVER_ID) >> 32
### snd_params = (msg_id & SENDER_PARAMS) >> 24
### rcv_params = (msg_id & RECEIVER_PARAMS) >> 16
### type = (msg_id & MESSAGE_TYPE) >> 8
#### expecting_ack = (msg_id & ID_EXPECTING ACK) >> 8
#### expecting_ack = (msg_type & TYPE_EXPECTING_ACK)
### length = (msg_id & MESSAGE_LENGTH)
#### length of MSG_ACK is 1

## Sender/receiver params format (uint8_t)
## 4b port_num , 3b port_type , 1b proc_side
### port_num = (params & PORT_NUM) >> 5
### port_type = (params & PORT_TYPE) >> 2
