#ifndef DEV_H_INCLUDED
# define DEV_H_INCLUDED


#include <stdint.h>


#define FLASH_PAGE_SIZE (512 * 3)


static inline unsigned int get_mem_flags(uint32_t addr, uint16_t size)
{
#define MEM_FLAG_USER (1 << 0) /* config otherwise */
#define MEM_FLAG_VECTOR (1 << 1)
#define MEM_FLAG_FLASH (1 << 2)
#define MEM_FLAG_DCR (1 << 3) /* device config register */
#define MEM_FLAG_DEVID (1 << 4)
#define MEM_FLAG_RESERVED (1 << 5)

  const uint32_t hi = addr + size;

  unsigned int flags = 0;

  if (hi <= 0x800000)
  {
    /* user memory space */
    flags |= MEM_FLAG_USER;
    if (hi <= 0x200) flags |= MEM_FLAG_VECTOR;
    else if (hi <= 0x15800) flags |= MEM_FLAG_FLASH;
    else flags |= MEM_FLAG_RESERVED;
  }
  else
  {
    /* config memory space */
    if ((addr >= 0xf80000) && (hi <= 0xf80018)) flags |= MEM_FLAG_DCR;
    else if ((addr >= 0xff0000) && (hi <= 0xff0002)) flags |= MEM_FLAG_DEVID;
    else flags |= MEM_FLAG_RESERVED;
  }

  return flags;
}


#endif /* ! DEV_H_INCLUDED */
