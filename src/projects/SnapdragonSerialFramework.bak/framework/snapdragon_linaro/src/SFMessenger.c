/* process-specific headers */
#include "../inc/SFMessenger.h"

/* interface headers */
#include "uart_interface.h"

// TODO shared memory stuff

uint32_t esc_read(){
  return (uint32_t) uart_interface_esc_read();
}

uint32_t esc_write(){
  return (uint32_t) uart_interface_esc_write();
}

uint32_t esc_write_read(){
  return (uint32_t) uart_interface_esc_write_read();
}
