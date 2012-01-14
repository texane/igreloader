#ifndef SCAB_H_INCLUDED
# define SCAB_H_INCLUDED



/* common */

#define SCAB_CMD_FRAME 0
#define SCAB_CMD_SYNC 1
#define SCAB_CMD_ENABLE 2
#define SCAB_CMD_SET_CAN_TIMES 3
#define SCAB_CMD_SET_CAN_FILTER 4
#define SCAB_CMD_CLEAR_CAN_FILTER 5
#define SCAB_CMD_STATUS 6
/* must be the last one */
#define SCAB_CMD_INVALID 7

/* SCAB_STATUS_XXX */
#define SCAB_STATUS_SUCCESS 0
#define SCAB_STATUS_FAILURE 1
#define SCAB_STATUS_INVALID 2

/* command buffer fixed size, in bytes */
/* largest command: cmd + can_sid + can_payload */
#define SCAB_CMD_SIZE (1 + 2 + 8)


#if !defined(__MPLAB_BUILD)

/* exported API */

#include <stdint.h>
#include "serial.h"

typedef serial_handle_t scab_handle_t;

#define SCAB_STATIC_INITIALIZER { -1, }

int scab_open(scab_handle_t*, const char*);
int scab_close(scab_handle_t*);
int scab_sync_serial(scab_handle_t*);
int scab_read_frame(scab_handle_t*, uint16_t*, uint8_t*);
int scab_write_frame(scab_handle_t*, uint16_t, const uint8_t*);
int scab_enable_bridge(scab_handle_t*);
int scab_disable_bridge(scab_handle_t*);
int scab_set_can_filter(scab_handle_t*, uint16_t, uint16_t);
int scab_clear_can_filter(scab_handle_t*);

static inline int scab_get_handle_fd(scab_handle_t* h)
{
  return h->fd;
}

#endif /* BUILD_MPLAB */


#endif /* ! SCAB_H_INCLUDED */
