#include <p33Fxxxx.h>
#include "../common/common.h"


/* configuration bits */

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


/* int types */

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;


/* static configuration */

#define CONFIG_USE_ECAN 1

#if CONFIG_USE_ECAN
# define CONFIG_USE_UART 0
#else
# define CONFIG_USE_UART 1
#endif

#define CONFIG_USE_LENDIAN 1


#if 0 /* todo */
/* software reset value */
/* todo: put at a special location */
static uint16_t is_soft_reset = 0;
#endif /* todo */


/* oscillator */

#define CONFIG_OSC_FOSC 79227500
#define OSC_FCY (CONFIG_OSC_FOSC / 2)

static void osc_setup(void)
{
  /* fast rc oscillator */

  PLLFBD = 41;
  CLKDIVbits.PLLPOST = 0;
  CLKDIVbits.PLLPRE = 0;

  OSCTUN = 0;
  RCONbits.SWDTEN = 0;

  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(0x01);
  while (OSCCONbits.COSC != 1) ;
  while (OSCCONbits.LOCK != 1) ;

  return 0;
}


/* self boot id */

static inline uint8_t get_boot_id(void)
{
#define CONFIG_BOOT_ID 2

#if defined(CONFIG_BOOT_ID)
  return CONFIG_BOOT_ID;
#else
  /* todo: read from dip switch */
  return 0;
#endif
}


/* goto user code entrypoint */

static inline void go_to(uint16_t addr)
{
  __asm__ __volatile__
  (
   "goto %0 \n\t"
   :
   : "r"(addr)
  );

  /* never reached */
}


/* delay */

static inline void delay(void)
{
  /* todo */
}


/* endianness */

#if CONFIG_USE_LENDIAN
/* data are sent in little endian, local arch is little too */
#define read_uint16(__s) (*(uint16_t*)(__s))
#define read_uint32(__s) (*(uint32_t*)(__s))
#define write_uint16(__s, __x) *(uint16_t*)(__s) = __x
#define write_uint32(__s, __x) *(uint32_t*)(__s) = __x
#else /* local is big endian */
/* TODO */
#endif /* CONFIG_USE_LENDIAN */


/* get 16 bits parts from 32 bits value */

#define LO(__x) (((__x) >> 0) & 0xffff)
#define HI(__x) (((__x) >> 16) & 0xffff)


/* communication layer */

#define COM_FRAME_SIZE 8

#if CONFIG_USE_ECAN

static void ecan_setup(uint8_t id)
{
  /* todo: use id as a boot command filter */
}

static void ecan_write(uint8_t* s)
{
}

static void ecan_read(uint8_t* s)
{
}

#define com_setup ecan_setup
#define com_write ecan_write
#define com_read ecan_read

#else /* CONFIG_USE_UART */

static void uart_setup(uint8_t id)
{
#define CONFIG_UART_BAUDRATE 38400
#define BRGVAL ((OSC_FCY / (16 * CONFIG_UART_BAUDRATE)) - 1)

#define CONFIG_UART_RXPIN 15
#define CONFIG_UART_RXTRIS TRISBbits.TRISB15
#define CONFIG_UART_TXTRIS TRISBbits.TRISB14
#define CONFIG_UART_TXPIN RPOR7bits.RP14R

  AD1PCFGL = 0xFFFF;

  RPINR18bits.U1RXR = CONFIG_UART_RXPIN;
  CONFIG_UART_RXTRIS = 1;
  CONFIG_UART_TXTRIS = 0;
  CONFIG_UART_TXPIN = 3;

  U1MODEbits.STSEL = 0;
  U1MODEbits.PDSEL = 0;
  U1MODEbits.ABAUD = 0;
  U1MODEbits.BRGH = 0;
  U1BRG = BRGVAL;

  U1MODEbits.UARTEN = 1;
  U1STAbits.UTXEN = 1;

#if 0 /* interrupts disabled */
  IFS0bits.U1RXIF = 0;
  IPC2bits.U1RXIP = 3;
  IEC0bits.U1RXIE = 1;
#endif /* interrupts disabled */
}

static void uart_write(uint8_t* s)
{
  unsigned int i;

  for (i = 0; i < COM_FRAME_SIZE; ++i, ++s)
  {
    while (U1STAbits.UTXBF) ;
    U1TXREG = *s;
  }
}

static void uart_read(uint8_t* s)
{
  /* todo: check for errors */

  unsigned int i;

  for (i = 0; i < COM_FRAME_SIZE; ++i, ++s)
  {
    while (U1STAbits.URXDA == 0) ;
    *s = U1RXREG;
  }
}

#define com_setup uart_setup
#define com_write uart_write
#define com_read uart_read

#endif /* CONFIG_USE_UART */


/* program memory routines */

static inline void read_program_word
(uint16_t haddr, uint16_t laddr, uint16_t hdata, uint16_t ldata)
{
  /* read a 24 bits program word at addr to [data] */

  /* scratch variable. use asm("w4") to force a register. */
  register uint16_t tmp;

  __asm__ __volatile__
  (
   "mov %1, tblpag \n\t"
   "tblrdl [ %2 ], %0 \n\t"
   "mov %5, [ %4 ] \n\t"
   "tblrdh [ %2 ], %0 \n\t"
   "mov %5, [ %3 ] \n\t"
   : "=&r"(tmp)
   : "r"(haddr), "r"(laddr), "r"(hdata), "r"(ldata), "0"(tmp)
  );
}

static inline void write_program_word
(uint16_t haddr, uint16_t laddr, uint16_t hdata, uint16_t ldata)
{
  /* write a 24 bits program word at addr with data */
  /* note that the latch must be flushed to memory on a row basis */

  __asm__ __volatile__
  (
   "mov %0, tblpag \n\t"
   "tblwth %2, [ %1 ] \n\t"
   "tblwtl %3, [ %1 ] \n\t"
   :
   : "r"(haddr), "r"(laddr), "r"(hdata), "r"(ldata)
  );
}

static inline void erase_program_page
(uint16_t addrhi, uint16_t addrlo)
{
  register uint16_t tmp;

  __asm__ __volatile__
  (
   "mov #0x4042, %0 \n\t"
   "mov %3, NVMCON \n\t"

   /* load page address */
   "mov %1, TBLPAG \n\t"
   "tblwtl %0, [ %2 ] \n\t"

   /* disable interrupt for 5 next insns */
   "disi #5 \n\t"

   /* protection sequence */
   "mov #0x55, %0 \n\t"
   "mov %3, NVMKEY \n\t"
   "mov #0xaa, %0 \n\t"
   "mov %3, NVMKEY \n\t"

   /* start erase sequence */
   "bset NVMCOM, #15 \n\t"

   /* wait for end of sequence */
#if 1
   "nop \n\t"
   "nop \n\t"
#else
   "1: btsc NVMCON, #15 \n\t"
   "bra 1b \n\t"
#endif

   : "=&r"(tmp)
   : "r"(addrhi), "r"(addrlo), "0"(tmp)
  );
}

static inline void flush_program_latches(void)
{
  /* write one program memory row */
  /* latches previously filled with write_program_word */

  register uint16_t tmp;

  /* see erase_page for comments */
  __asm__ __volatile__
  (
   "mov #0x4001, %0 \n\t"
   "mov %1, NVMCON \n\t"
   "disi #5 \n\t"
   "mov #0x55, %0 \n\t"
   "mov %1, NVMKEY \n\t"
   "mov #0xaa, %0 \n\t"
   "mov %1, NVMKEY \n\t"
   "bset NVMCOM, #15 \n\t"
#if 1
   "nop \n\t"
   "nop \n\t"
#else
   "1: btsc NVMCON, #15 \n\t"
   "bra 1b \n\t"
#endif
   : "=&r"(tmp)
   : "0"(tmp)
  );
}


/* commands */

static void read_process_cmd(void)
{
#define ROW_INSN_COUNT 64
#define ROW_BYTE_COUNT (ROW_INSN_COUNT * 3)
#define PAGE_INSN_COUNT 512
#define PAGE_BYTE_COUNT (PAGE_INSN_COUNT * 3)

  /* 8 is added to avoid overflow in com_read */
  static uint8_t page_buf[PAGE_BYTE_COUNT + 8];

  uint8_t cmd_buf[8];
  uint32_t addr;
  uint16_t size;
  uint16_t off;
  uint16_t i;
  uint16_t j;

  com_read(cmd_buf);

  switch (read_uint8(cmd_buf + 0))
  {
  case CMD_ID_WRITE_PROGRAM:
    {
      /* write a program page */

      addr = read_uint32(cmd_buf + 1);
      size = read_uint16(cmd_buf + 5);

      /* command ack */
      com_write(cmd_buf);

      /* read the page before erasing, if not a full page. */
      off = 0;
      if (size != PAGE_BYTE_COUNT)
      {
	off = addr % PAGE_BYTE_COUNT;

	/* read 24 bits at a time */
	for (i = 0; i < PAGE_BYTE_COUNT; i += 3)
	{
	  const uint32_t tmp = (uint32_t)(page_buf + i);
	  read_program_word(HI(addr), LO(addr), HI(tmp), LO(tmp));
	}
      }

      /* erase page */
      erase_program_page(HI(addr), LO(addr));

      /* receive the page */
      for (i = 0; i < size; i += COM_FRAME_SIZE)
      {
	com_read(page_buf + off + i);

	/* frame ack */
	com_write(cmd_buf);
      }

      /* write a whole page. i incremented by inner loop */
      for (i = 0; i < PAGE_BYTE_COUNT; )
      {
	/* fill the one row program memory buffer 3 bytes at a time */
	for (j = 0; j < (ROW_BYTE_COUNT / 3); i += 3, j += 3, addr += 3)
	{
	  const uint32_t tmp = *(uint32_t*)(buf + i);
	  write_program_word(HI(addr), LO(addr), HI(tmp), LO(tmp));
	}

	/* finalize row write operation */
	flush_program_latches();
      }

      /* page programming ack */
      com_write(cmd_buf);

      break ;
    }

  case CMD_ID_WRITE_CONFIG:
    {
      /* todo */

      break ;
    }

  case CMD_ID_READ_PROGRAM:
    {
      addr = read_uint32(cmd_buf + 1);
      size = read_uint16(cmd_buf + 5);

      /* assume addr is aligned on program word boundary */
      /* assume size is a multiple of word size */

      /* command ack */
      com_write(cmd_buf);

      while (size)
      {
	/* fill cmd_buffer[0:5] */
	for (i = 0; size && (i < 6); i += 3, addr += 3, size -= 3)
	{
	  const uint32_t tmp = (uint32_t)(cmd_buf + i);
	  read_program_word(HI(addr), LO(addr), HI(tmp), LO(tmp));
	}

	com_write(cmd_buf);

	/* frame ack */
	com_read(cmd_buf);
      }

      break ;
    }

  case CMD_ID_READ_CONFIG:
    {
      /* todo */

      break ;
    }

  case CMD_ID_STATUS:
    {
      /* todo: reply status */

      break ;
    }

  case CMD_ID_GOTO:
    {
      /* jump to the entrypoint */
      addr = read_uint32(cmd_buf + 1);
      go_to(LO(addr));
      break ;
    }

  default: break ;
  }
}


/* main */

int main(void)
{
#if 0 /* todo */
  /* look for soft reset */
  if (is_soft_reset)
  {
    is_soft_reset = 0;
    go_to();
  }
#endif /* todo */

  osc_setup();

  com_setup(get_boot_id());

  while (1) read_process_cmd();

  return 0;
}
