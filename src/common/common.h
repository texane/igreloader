#ifndef COMMON_H_INCLUDED
# define COMMON_H_INCLUDED


/* command buffer size
 */

#define CMD_BUF_SIZE 8


/* command ids
 */

#define CMD_ID_WRITE_PMEM 0
#define CMD_ID_READ_PMEM 1
#define CMD_ID_WRITE_CMEM 2
#define CMD_ID_STATUS 3
#define CMD_ID_GOTO 4


/* CAN related addressing
 */

/* boot group special nodes */
#define HOST_NODE_ID 0x0000

/* CAN groups */
#define BOOT_GROUP_ID 0x0000

/* CAN priorities */
#define LOW_PRIO_ID 0x0000
#define NORMAL_PRIO_ID 0x0001
#define HIGH_PRIO_ID 0x0002

/* CAN addressing macros */
#define MAKE_CAN_SID(__p, __g, __n) (((__p) << 9) | ((__g) << 3) | __n)


#endif /* ! COMMON_H_INCLUDED */
