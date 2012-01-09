#ifndef HEX_H_INCLUDED
# define HEX_H_INCLUDED


#include <stdint.h>
#include <sys/types.h>


typedef struct hex_range
{
  struct hex_range* next;
  struct hex_range* prev;

  uint32_t addr;

  /* off is an offset in buf */
  size_t off;
  /* size does not include off */
  size_t size;
  /* buf total size is off + size */
  uint8_t buf[1];

} hex_range_t;


int hex_read_ranges(const char*, hex_range_t**);
void hex_free_ranges(hex_range_t*);
void hex_merge_ranges(hex_range_t**);
void hex_print_ranges(const hex_range_t*);


#endif /* ! HEX_H_INCLUDED */
