#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include "dev.h"
#include "hex.h"
#include "lendian.h"
#include "serial.h"
#include "../common/common.h"


/* todo: once testing done, messages should
   CAN frame info should directly be wrapped
   into a serial frame so that the endpoint
   acts as a serial to CAN bridge.
 */


/* communication routines */

static int com_init(serial_handle_t* handle, const char* devname)
{
  static const serial_conf_t conf = { 38400, 8, SERIAL_PARITY_DISABLED, 1 };

  if (serial_open(handle, devname) == -1)
  {
    printf("serial_open(%s) == -1\n", devname);
    return -1;
  }

  if (serial_set_conf(handle, &conf) == -1)
  {
    printf("serial_set_conf() == -1\n");
    serial_close(handle);
    return -1;
  }

  return 0;
}

static inline void com_close(serial_handle_t* handle)
{
  serial_close(handle);
}

static int com_write(serial_handle_t* handle, const uint8_t* buf)
{
  size_t size = CMD_BUF_SIZE;
  size_t tmp = 0;

  for (; size; size -= tmp)
  {
    if (serial_write(handle, buf, size, &tmp))
      break ;
  }

  return (size == 0) ? 0 : -1;
}

static inline int com_read(serial_handle_t* handle, uint8_t* buf)
{
  return serial_readn(handle, buf, CMD_BUF_SIZE);
}

static inline int com_read_timeout
(serial_handle_t* handle, uint8_t* buf, unsigned int ms)
{
  /* ms the timeout, in milliseconds */
  /* return -2 on timeout */

  struct timeval tm;
  int err;
  fd_set fds;

  tm.tv_sec = ms / 1000;
  tm.tv_usec = 1000 * (ms % 1000);

  FD_ZERO(&fds);
  FD_SET(handle->fd, &fds);
  
  err = select(handle->fd + 1, &fds, NULL, NULL, &tm);
  if (err == 1)
  {
    /* no timeout */
    return com_read(handle, buf);
  }

  /* timeout or error */
  return err == 0 ? -2 : -1;
}


/* communication helpers */

static uint8_t dummy_buf[CMD_BUF_SIZE];

static inline int com_read_ack(serial_handle_t* handle)
{
  return com_read(handle, dummy_buf);
}

static inline int com_write_ack(serial_handle_t* handle)
{
  return com_write(handle, dummy_buf);
}


/* write hex file to device memory */

static int do_write
(
 serial_handle_t* handle,
 unsigned int bootid,
 int ac, char** av
)
{
  const char* const filename = av[0];

  hex_range_t* ranges;
  hex_range_t* pos;
  hex_range_t* first_dcr_range = NULL;
  unsigned int flags;
  size_t off;
  size_t i;
  size_t page_size;
  uint8_t buf[CMD_BUF_SIZE];

  if (hex_read_ranges(filename, &ranges) == -1)
  {
    printf("invalid hex file\n");
    goto on_error;
  }

  hex_merge_ranges(&ranges);

  hex_print_ranges(ranges);

  /* for each range in program memory, write pages */
  for (pos = ranges; pos != NULL; pos = pos->next)
  {
    flags = get_mem_flags(pos->addr, pos->size);

    if (pos->size == 0)
    {
      printf("warning: pos->size == 0\n");
      continue ;
    }

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

    /* user code relocated */
    pos->addr += 0x8000;

    /* handle unaligned first page */
    off = 0;
    page_size = PAGE_BYTE_COUNT - (pos->addr % PAGE_BYTE_COUNT);

    while (1)
    {
      /* check if bigger than */
      if ((off + page_size) > pos->size) page_size = pos->size - off;

      printf("CMD_ID_WRITE_PMEM(%x, %u)\n",
	     (pos->addr + off) / 2,
	     page_size / 4);

      /* initiate write sequence */
      buf[0] = CMD_ID_WRITE_PMEM;
      write_uint32(buf + 1, (pos->addr + off) / 2);
      write_uint16(buf + 5, page_size / 4);
      if (com_write(handle, buf)) goto on_error;

      /* command ack */
      if (com_read_ack(handle)) goto on_error;

      /* send the page 8 bytes (2 program words) at a time */
      for (i = 0; i < page_size; i += 8)
      {
	if (com_write(handle, pos->buf + off + i)) goto on_error;

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
  if (first_dcr_range != NULL)
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
    if (com_write(handle, buf)) goto on_error;
    if (com_read_ack(handle)) goto on_error;
    for (i = 0; i < DCR_BYTE_COUNT; i += 8)
    {
      if (com_read(handle, dcr_buf + i)) goto on_error;
      if (com_write_ack(handle)) goto on_error;
    }

    printf("[x] read DCR area\n");

    /* update according to hex ranges  */
    for (pos = first_dcr_range; pos != NULL; pos = pos->next)
    {
      /* break if non DCR, since contiguous */
      flags = get_mem_flags(pos->addr, pos->size);
      if (!(flags & MEM_FLAG_DCR)) break ;

      off = pos->addr - DCR_BYTE_ADDR;
      memcpy(dcr_buf + off, pos->buf, pos->size);
    }

    printf("write DCR area\n");

    /* write dcr_buf */
    buf[0] = CMD_ID_WRITE_CMEM;
    if (com_write(handle, buf)) goto on_error;
    if (com_read_ack(handle)) goto on_error;
    for (i = 0; i < DCR_BYTE_COUNT; i += 8)
    {
      printf("write_dcr %u: 0x%08x 0x%08x\n",
	     i,
	     *(uint32_t*)(dcr_buf + i),
	     *(uint32_t*)(dcr_buf + i + 4));

      if (com_write(handle, dcr_buf + i)) goto on_error;
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
 serial_handle_t* handle,
 unsigned int bootid,
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
  if (com_write(handle, cmd_buf)) goto on_error;

  /* command ack */
  if (com_read_ack(handle)) goto on_error;

  for (i = 0; i < (size * 4); i += 8)
  {
    if (com_read(handle, read_buf + i)) goto on_error;

    /* frame ack */
    if (com_write_ack(handle)) goto on_error;
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
 serial_handle_t* handle,
 unsigned int bootid,
 int ac, char** av
)
{
  const uint32_t addr = strtoul(av[0], NULL, 16);
  uint8_t cmd_buf[CMD_BUF_SIZE];

  cmd_buf[0] = CMD_ID_GOTO;
  write_uint32(cmd_buf + 1, addr);
  if (com_write(handle, cmd_buf)) return -1;
  return 0;
}


/* retrive the status of one or all devices */

static int do_status
(
 serial_handle_t* handle,
 unsigned int bootid
)
{
  /* bootid the device id or (unsigned int)-1 for all */

  uint8_t bootids[16];
  uint8_t buf[CMD_BUF_SIZE];
  unsigned int i;
  unsigned int n;
  int err = -1;
  int ret;

  if (bootid == (unsigned int)-1)
  {
    n = sizeof(bootids) / sizeof(bootids[0]);
    for (i = 0; i < n; ++i) bootids[i] = (uint8_t)i;
  }
  else
  {
    n = 1;
    bootids[0] = (uint8_t)bootid;
  }

  for (i = 0; i < n; ++i)
  {
    buf[0] = CMD_ID_STATUS;
    buf[1] = (uint8_t)bootids[i];
    if (com_write(handle, buf)) goto on_error;
    ret = com_read_timeout(handle, buf, 1000);
    if (ret == -1) goto on_error;
    printf("device %02x: %c\n", bootids[i], ret == -2 ? 'x' : '!');
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
     ./a.out write <serial_device> <bootid> <file.hex>
     ./a.out read <serial_device> <bootid> <addr> <size>
     ./a.out goto <serial_device> <bootid> <addr>
     ./a.out status <serial_device> <bootid>
   */

  const char* const what = av[1];
  const char* const devname = av[2];
  const unsigned int bootid = (ac == 3) ? (unsigned int)-1 : atoi(av[3]);
  serial_handle_t handle = { -1, };

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
    if (do_write(&handle, bootid, ac - 4, av + 4) == -1)
    {
      printf("do_program() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "read") == 0)
  {
    if (do_read(&handle, bootid, ac - 4, av + 4) == -1)
    {
      printf("do_read() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "goto") == 0)
  {
    if (do_goto(&handle, bootid, ac - 4, av + 4) == -1)
    {
      printf("do_goto() == -1\n");
      goto on_error;
    }
  }
  else if (strcmp(what, "status") == 0)
  {
    if (do_status(&handle, bootid) == -1)
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
  if (handle.fd != -1) com_close(&handle);

  return 0;
}
