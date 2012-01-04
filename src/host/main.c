#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include "dev.h"
#include "hex.h"
#include "lendian.h"
#include "../common/common.h"


/* write hex file to device memory */

static int do_program
(
 void* dev, unsigned int bootid,
 int ac, char** av
)
{
  const char* const filename = av[0];

  hex_range_t* ranges;
  hex_range_t* pos;
  unsigned int flags;
  size_t off;
  size_t i;
  size_t page_size;
  uint8_t buf[8];

  if (hex_read_ranges(filename, &ranges) == -1)
  {
    printf("invalid hex file\n");
    goto on_error;
  }

  hex_merge_ranges(&ranges);

  /* for each range, write pages */
  for (pos = ranges; pos != NULL; pos = pos->next)
  {
    flags = get_mem_flags(pos->addr, pos->size);

    if (flags & MEM_FLAG_RESERVED)
    {
      printf("invalid memory\n");
      goto on_error;
    }

    if ((flags & MEM_FLAG_USER) == 0)
    {
      printf("TODO: configuration space\n");
      continue ;
    }

    /* handle unaligned first page */
    off = 0;
    page_size = FLASH_PAGE_SIZE - (pos->addr % FLASH_PAGE_SIZE);

    while (1)
    {
      /* adjust page size */
      page_size = FLASH_PAGE_SIZE;
      if ((off + page_size) > pos->size) page_size = pos->size - off;

      /* todo: check device addr range */

      /* initiate write sequence */
      buf[0] = CMD_ID_WRITE_PROGRAM;
      write_uint32(buf + 1, pos->addr + off);
      write_uint16(buf + 5, page_size);

      /* todo: com_send(dev, buf); */

      /* todo: wait for command ack */

      /* send the page 8 bytes at a time */
      for (i = 0; i < pos->size; i += 8)
      {
	/* todo: com_send(dev, pos->buf + off + i); */

	/* todo: wait for data ack */
      }

      /* todo: wait for page programming ack */

      /* next page or done */
      off += page_size;
      if (off == pos->size) break ;
    }
  }

 on_error:
  if (ranges != NULL) hex_free_ranges(ranges);

  return 0;
}


/* read program memory */

static int do_read
(
 void* dev, unsigned int bootid,
 int ac, char** av
)
{
  const uint32_t addr = strtoul(av[0], NULL, 16);
  const uint16_t size = strtoul(av[1], NULL, 10);
  uint8_t* read_buf = NULL;
  uint8_t cmd_buf[8];
  int err = -1;
  size_t i;

  /* todo: validate addr (alignment, not reserved) */
  /* todo: check size */

  /* allocate a bit larger to avoid overflow on last read */
  read_buf = malloc(size + sizeof(cmd_buf));
  if (read_buf == NULL) goto on_error;

  cmd_buf[0] = CMD_ID_READ_PROGRAM;
  write_uint32(cmd_buf + 1, addr);
  write_uint16(cmd_buf + 5, addr);
  /* todo: com_send(cmd_buf) */

  /* read 2 24 bits words per frame */
  for (i = 0; i < size; i += 6)
  {
    /* todo: com_read(read_buf + i); */
    /* todo: ack */
  }

  /* print the buffer */
  for (i = 0; i < size; ++i) printf(" %02x", read_buf[i]);
  printf("\n");

  /* success */
  err = 0;

 on_error:
  if (read_buf != NULL) free(read_buf);

  return err;
}


/* main */

int main(int ac, char** av)
{
  /* command lines:
     ./a.out write <serial_device> <bootid> <file.hex>
     ./a.out read <serial_device> <bootid> <addr> <size>
   */

  const char* const what = av[1];
  const char* const devname = av[2];
  const unsigned int bootid = atoi(av[3]);

  /* todo: initialize serial */

  /* program device flash */
  if (strcmp(what, "write") == 0)
  {
    if (do_program(NULL, bootid, ac - 4, av + 4) == -1)
    {
      printf("do_program() == -1\n");
      return -1;
    }
  }
  else if (strcmp(what, "read") == 0)
  {
    if (do_read(NULL, bootid, ac - 4, av + 4) == -1)
    {
      printf("do_read() == -1\n");
      return -1;
    }
  }
  else
  {
    printf("%s: invalid command\n", what);
  }

  return 0;
}
