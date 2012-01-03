#ifndef LENDIAN_H_INCLUDED
# define LENDIAN_H_INCLUDED


#include <stdint.h>


/* little endianness related routines */

static inline uint16_t read_uint16(uint8_t* s)
{
  return *(uint16_t*)s;
}

static inline uint32_t read_uint32(uint8_t* s)
{
  return *(uint32_t*)s;
}

static inline void write_uint16(uint8_t* s, uint16_t x)
{
  *(uint16_t*)s = x;
}

static inline void write_uint32(uint8_t* s, uint32_t x)
{
  *(uint32_t*)s = x;
}


#endif /* ! LENDIAN_H_INCLUDED */
