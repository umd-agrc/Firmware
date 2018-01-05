# An elka message composes anelka packet
uint8 MAX_MSG_LEN=40

uint8 len # length of message excluding this byte
uint8 type
uint8 data[MAX_PACKET_LEN]
