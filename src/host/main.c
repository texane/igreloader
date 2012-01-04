#include <stdio.h>
#include "dev.h"
#include "hex.h"
#include "lendian.h"
#include "../common/common.h"


/* write hex file to device memory */

static int do_program(void* dev, const char* filename)
{
  /* filename a hex file */

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


/* main */

int main(int ac, char** av)
{
  /* command line: ./a.out <device> <file.hex> */

  const char* const devname = av[1];
  const char* const filename = av[2];

  /* todo: initialize serial */

  /* program device flash */
  if (do_program(NULL, filename) == -1)
  {
    printf("programming failed\n");
    return -1;
  }

  return 0;
}
