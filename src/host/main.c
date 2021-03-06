#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include "dev.h"
#include "hex.h"
#include "lendian.h"
#include "scab_api.h"
#include "scab_common.h"
#include "../common/common.h"


/* communication routines */

#define HOST_FILTER_MASK 0x1ff /* exclude priority */
#define HOST_FILTER_VALUE MAKE_CAN_SID(0, BOOT_GROUP_ID, HOST_NODE_ID)

static int com_init(scab_handle_t** handle, const char* devname)
{
  if (scab_open(handle, devname)) return -1;

  scab_sync_serial(*handle);

  if (scab_set_can_filter(*handle, HOST_FILTER_MASK, HOST_FILTER_VALUE))
  {
    scab_close(*handle);
    *handle = NULL;
    return -1;
  }

  return 0;
}

static inline int com_close(scab_handle_t* handle)
{
  scab_clear_can_filter(handle);
  return scab_close(handle);
}

static inline int com_write
(scab_handle_t* handle, uint16_t sid, const uint8_t* buf)
{
  return scab_write_frame(handle, sid, buf);
}

static int com_read_timeout
(scab_handle_t* handle, uint8_t* buf, unsigned int ms)
{
  /* ms the timeout in milliseconds, or 0 */
  /* return -2 on timeout */

  if (ms != 0)
  {
    int err;
    struct timeval tm;
    fd_set fds;

    tm.tv_sec = ms / 1000;
    tm.tv_usec = 1000 * (ms % 1000);

    FD_ZERO(&fds);
    FD_SET(scab_get_handle_fd(handle), &fds);
  
    err = select(scab_get_handle_fd(handle) + 1, &fds, NULL, NULL, &tm);
    /* timeout or error */
    if (err != 1)  return err == 0 ? -2 : -1;
  }

  return scab_read_frame(handle, NULL, buf);
}

static inline int com_read(scab_handle_t* handle, uint8_t* buf)
{
  return com_read_timeout(handle, buf, 0);
}


/* communication helpers */

static uint8_t dummy_buf[CMD_BUF_SIZE];

static inline int com_read_ack(scab_handle_t* handle)
{
  return com_read(handle, dummy_buf);
}

static inline int com_write_ack(scab_handle_t* handle, uint16_t sid)
{
  return com_write(handle, sid, dummy_buf);
}


/* dump program memory buffer */

__attribute__((unused))
static void dump_pmem(uint32_t x, const uint8_t* s, size_t n)
{
  for (; n; --n, s += sizeof(uint32_t), x += 2)
    printf("%08x: %08x\n", x, *(uint32_t*)s);
}


/* write hex file to device memory */

static int do_write
(
 scab_handle_t* handle,
 unsigned int sid,
 int ac, char** av
)
{
  const char* const filename = av[0];
  const unsigned int noconf = (ac == 2 ? !strcmp(av[1], "noconf") : 0);

  hex_range_t* ranges;
  hex_range_t* pos;
  hex_range_t* new_range;
  hex_range_t* next_pos;
  hex_range_t* first_dcr_range = NULL;
  unsigned int flags;
  size_t off;
  size_t i;
  size_t new_size;
  size_t page_size;
  uint32_t last_addr;
  uint8_t buf[CMD_BUF_SIZE];

  if (hex_read_ranges(filename, &ranges) == -1)
  {
    printf("invalid hex file\n");
    goto on_error;
  }

  if (ranges == NULL)
  {
    printf("no range to write\n");
    goto on_error;
  }

  /* merge contiguous lines as one block */
  hex_merge_ranges(&ranges);

  /* remove bootloader reserved areas */
  for (pos = ranges; pos != NULL; pos = next_pos)
  {
    next_pos = pos->next;

    /* [ 0x0000 - 0x0008 [: dont overwrite 2 reserved instructions */
    if (pos->addr < 0x0008)
    {
      last_addr = pos->addr + pos->size;
      if (last_addr > 0x0008) last_addr = 0x0008;

      off = (size_t)(last_addr - pos->addr);
      pos->off += off;
      pos->size -= off;
      pos->addr = last_addr;
    }

    /* reserved bootloader area */
    /* warning: must match device.X/igreboot_p33FJ128GP802.gld */
#define FIRST_BOOT_ADDR (0x200 * 2)
#define LAST_BOOT_ADDR (0x600 * 2)

    last_addr = pos->addr + pos->size;

    /* check if the range overlaps boot area. 4 cases to handle. */
    if ((pos->addr < FIRST_BOOT_ADDR) && (last_addr > LAST_BOOT_ADDR))
    {
      /* range left and right sides span boot area */
      /* resize left, create a new range for right */

      /* create new range */
      new_size = last_addr - LAST_BOOT_ADDR;
      new_range = malloc(offsetof(hex_range_t, buf) + new_size);
      new_range->off = 0;
      new_range->next = pos->next;
      new_range->prev = pos;
      new_range->addr = LAST_BOOT_ADDR;
      new_range->size = new_size;
      off = LAST_BOOT_ADDR - pos->addr;
      memcpy(new_range->buf, pos->buf + pos->off + off, new_size);

      /* link new range */
      if (pos->next != NULL) pos->next->prev = new_range;
      pos->next = new_range;

      /* update the current range */
      pos->size -= new_size + (LAST_BOOT_ADDR - FIRST_BOOT_ADDR);
    }
    else if ((pos->addr < FIRST_BOOT_ADDR) && (last_addr > FIRST_BOOT_ADDR))
    {
      /* left side spans boot area */

      off = (size_t)(last_addr - FIRST_BOOT_ADDR);
      pos->size -= off;
    }
    else if ((pos->addr < LAST_BOOT_ADDR) && (last_addr > LAST_BOOT_ADDR))
    {
      /* right side spans boot area */

      off = (size_t)(LAST_BOOT_ADDR - pos->addr);
      pos->addr = LAST_BOOT_ADDR;
      pos->off += off;
      pos->size -= off;
    }
    else if ((pos->addr >= FIRST_BOOT_ADDR) && (last_addr <= LAST_BOOT_ADDR))
    {
      /* range is fully included in boot area, remove */

      if (pos->next != NULL) pos->next->prev = pos->prev;

      if (pos->prev == NULL)
      {
	ranges = pos->next;
	if (ranges != NULL) ranges->prev = NULL;
      }
      else pos->prev->next = pos->next;

      free(pos);
    }
    else
    {
      /* range does not span boot area */
      if (pos->addr >= LAST_BOOT_ADDR)
      {
	/* range area sorted by addr, can break */
	break ;
      }
    }
  }

  /* for each range in program memory, write pages */
  for (pos = ranges; pos != NULL; pos = pos->next)
  {
    if (pos->size == 0)
    {
      printf("warning: pos->size == 0\n");
      continue ;
    }

    flags = get_mem_flags(pos->addr, pos->size);

    if (flags & MEM_FLAG_RESERVED)
    {
      printf("invalid memory\n");
      goto on_error;
    }

    if ((flags & MEM_FLAG_USER) == 0)
    {
      /* written later */
      if (first_dcr_range == NULL) first_dcr_range = pos;
      continue ;
    }

    /* handle unaligned first page */
    off = 0;
    page_size = PAGE_BYTE_COUNT - (pos->addr % PAGE_BYTE_COUNT);

    while (1)
    {
      /* check if bigger than */
      if ((off + page_size) > pos->size) page_size = pos->size - off;

      printf("CMD_ID_WRITE_PMEM [ %x, %x [\n",
	     (pos->addr + off) / 2,
	     (pos->addr + off + page_size) / 2);

      /* initiate write sequence */
      buf[0] = CMD_ID_WRITE_PMEM;
      write_uint32(buf + 1, (pos->addr + off) / 2);
      write_uint16(buf + 5, page_size / 4);
      if (com_write(handle, sid, buf)) goto on_error;

      /* command ack */
      if (com_read_ack(handle)) goto on_error;

      /* send the page 8 bytes (2 program words) at a time */
      for (i = 0; i < page_size; i += 8)
      {
	if (com_write(handle, sid, pos->buf + pos->off + off + i))
	  goto on_error;

	/* frame ack */
	if (com_read_ack(handle)) goto on_error;
      }

      /* page programming ack */
      if (com_read_ack(handle)) goto on_error;

      /* next page or done */
      off += page_size;
      if (off == pos->size) break ;

      /* adjust page size */
      page_size = PAGE_BYTE_COUNT;
    }
  }

  printf("[x] program code memory done\n");

#if 1 /* write all device configuration registers in once */
  if ((noconf == 0) && (first_dcr_range != NULL))
  {
    /* in bytes, CMD_BUF_SIZE to avoid last read overflow */
    uint8_t dcr_buf[DCR_BYTE_COUNT + CMD_BUF_SIZE];

    printf("read DCR area\n");
    printf("CMD_ID_READ_PMEM(%x, %u)\n",
	   DCR_BYTE_ADDR / 2, DCR_BYTE_COUNT / 4);

    /* read the device DCR area */
    buf[0] = CMD_ID_READ_PMEM;
    write_uint32(buf + 1, DCR_BYTE_ADDR / 2);
    write_uint16(buf + 5, DCR_BYTE_COUNT / 4);
    if (com_write(handle, sid, buf)) goto on_error;
    if (com_read_ack(handle)) goto on_error;
    for (i = 0; i < DCR_BYTE_COUNT; i += 8)
    {
      if (com_read(handle, dcr_buf + i)) goto on_error;
      if (com_write_ack(handle, sid)) goto on_error;
    }

    printf("[x] read DCR area\n");

    /* update according to hex ranges  */
    for (pos = first_dcr_range; pos != NULL; pos = pos->next)
    {
      /* break if non DCR, since contiguous */
      flags = get_mem_flags(pos->addr, pos->size);
      if (!(flags & MEM_FLAG_DCR)) break ;

      off = pos->addr - DCR_BYTE_ADDR;
      memcpy(dcr_buf + off, pos->buf + pos->off, pos->size);
    }

    printf("write DCR area\n");

    /* write dcr_buf */
    buf[0] = CMD_ID_WRITE_CMEM;
    if (com_write(handle, sid, buf)) goto on_error;
    if (com_read_ack(handle)) goto on_error;
    for (i = 0; i < DCR_BYTE_COUNT; i += 8)
    {
      if (com_write(handle, sid, dcr_buf + i)) goto on_error;
      if (com_read_ack(handle)) goto on_error;
    }
    if (com_read_ack(handle)) goto on_error;

    printf("[x] write DCR area\n");
  }
#endif /* write dcr_buf */

 on_error:
  if (ranges != NULL) hex_free_ranges(ranges);

  return 0;
}


/* read program memory */

static int do_read
(
 scab_handle_t* handle,
 unsigned int sid,
 int ac, char** av
)
{
  /* addr the first word address to read */
  /* size the word count to read */

  const uint32_t addr = strtoul(av[0], NULL, 16);
  const uint16_t size = strtoul(av[1], NULL, 10);
  uint8_t* read_buf = NULL;
  uint8_t cmd_buf[CMD_BUF_SIZE];
  int err = -1;
  size_t i;

  /* todo: validate addr (alignment, not reserved) */
  /* todo: check size */

  /* allocate a bit larger to avoid overflow on last read */
  read_buf = malloc((size * 4) + CMD_BUF_SIZE);
  if (read_buf == NULL) goto on_error;

  cmd_buf[0] = CMD_ID_READ_PMEM;
  write_uint32(cmd_buf + 1, addr);
  write_uint16(cmd_buf + 5, size);
  if (com_write(handle, sid, cmd_buf)) goto on_error;

  /* command ack */
  if (com_read_ack(handle)) goto on_error;

  for (i = 0; i < (size * 4); i += 8)
  {
    if (com_read(handle, read_buf + i)) goto on_error;

    /* frame ack */
    if (com_write_ack(handle, sid)) goto on_error;
  }

  /* print the buffer */
  for (i = 0; i < (size * 4); i += 4)
    printf("%06x: %08x\n", addr + i / 2, read_uint32(read_buf + i) & 0xffffff);
  printf("\n");

  /* success */
  err = 0;

 on_error:
  if (read_buf != NULL) free(read_buf);

  return err;
}


/* goto specified address */

static int do_goto
(
 scab_handle_t* handle,
 unsigned int sid,
 int ac, char** av
)
{
  const uint32_t addr = strtoul(av[0], NULL, 16);
  uint8_t cmd_buf[CMD_BUF_SIZE];

  cmd_buf[0] = CMD_ID_GOTO;
  write_uint32(cmd_buf + 1, addr);
  if (com_write(handle, sid, cmd_buf)) return -1;
  if (com_read_ack(handle)) return -1;
  return 0;
}


/* retrive the status of one or all devices */

static int do_status
(
 scab_handle_t* handle,
 unsigned int sid
)
{
  /* bootid the device id or (unsigned int)-1 for all */

  uint16_t sids[8];
  uint8_t buf[CMD_BUF_SIZE];
  unsigned int i;
  unsigned int n;
  int err = -1;
  int ret;

  if (sid == (unsigned int)-1)
  {
    n = sizeof(sids) / sizeof(sids[0]);
    for (i = 0; i < n; ++i)
      sids[i] = MAKE_CAN_SID(HIGH_PRIO_ID, BOOT_GROUP_ID, i);
  }
  else
  {
    n = 1;
    sids[0] = (uint8_t)sid;
  }

  for (i = 0; i < n; ++i)
  {
    const uint8_t nodeid = GET_CAN_NODE_ID(sids[i]);

    buf[0] = CMD_ID_STATUS;
    if (com_write(handle, sids[i], buf)) goto on_error;
    ret = com_read_timeout(handle, buf, 1000);
    if (ret == -1) goto on_error;

    printf("device %02x: [%c]\n", nodeid, ret == -2 ? '!' : 'x');
  }

  /* success */
  err = 0;

 on_error:
  return err;
}


/* main */

int main(int ac, char** av)
{
  /* command lines:
     ./a.out write <serial_device> <devid> <file.hex> <noconf>
     ./a.out read <serial_device> <devid> <addr> <size>
     ./a.out goto <serial_device> <devid> <addr>
     ./a.out status <serial_device> <devid>
   */

  const char* const what = av[1];
  const char* const devname = av[2];
  const unsigned int devid = (ac == 3) ? (unsigned int)-1 : atoi(av[3]);
  unsigned int sid;
  scab_handle_t* handle = NULL;

  sid = MAKE_CAN_SID(HIGH_PRIO_ID, BOOT_GROUP_ID, devid);

  if (strcmp(devname, "null"))
  {
    if (com_init(&handle, devname) == -1)
    {
      printf("com_init(%s) == -1\n", devname);
      goto on_error;
    }
  }

  /* program device flash */
  if (strcmp(what, "write") == 0)
  {
    if (do_write(handle, sid, ac - 4, av + 4) == -1)
    {
      printf("do_program() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "read") == 0)
  {
    if (do_read(handle, sid, ac - 4, av + 4) == -1)
    {
      printf("do_read() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "goto") == 0)
  {
    if (do_goto(handle, sid, ac - 4, av + 4) == -1)
    {
      printf("do_goto() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "status") == 0)
  {
    if (do_status(handle, sid) == -1)
    {
      printf("do_status() == -1\n");
      goto on_error;
    }
  }
  else
  {
    printf("%s: invalid command\n", what);
    goto on_error;
  }

 on_error:
  if (handle != NULL) com_close(handle);

  return 0;
}
