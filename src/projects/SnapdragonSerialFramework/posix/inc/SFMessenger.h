#ifndef SF_MESSENGER_H
#define SF_MESSENGER_H

#include <stdint.h>
#include "../../common/status.h"

uint32_t esc_read(void);

uint32_t esc_write(void);

uint32_t esc_write_read(void);

#endif
