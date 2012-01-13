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
}


/* self boot id */

static inline uint16_t get_boot_id(void)
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

#define CONFIG_USE_LENDIAN 1

#if CONFIG_USE_LENDIAN
/* data are sent in little endian, local arch is little too */
#define read_uint8(__s) (*(uint8_t*)(__s))
#define read_uint16(__s) (*(uint16_t*)(__s))
#define read_uint32(__s) (*(uint32_t*)(__s))
#define write_uint16(__s, __x) *(uint16_t*)(__s) = __x
#define write_uint32(__s, __x) *(uint32_t*)(__s) = __x
#else /* local is big endian */
/* TODO */
#endif /* CONFIG_USE_LENDIAN */


/* get 16 bits parts from 32 bits value */

#define LO(__x) ((__x) & 0xffff)
#define HI(__x) (((__x) >> 16) & 0xffff)


/* communication layer */

#define CONFIG_USE_ECAN 1

#if CONFIG_USE_ECAN

uint16_t ecan_bufs[4][8] __attribute__((space(dma), aligned(4 * 16)));

static void ecan_setup(uint16_t id)
{
  /* baud rate */
#define FCAN 40000000 
#define BITRATE 125000  
#define NTQ 10
#define BRP_VAL ((FCAN / (2 * NTQ * BITRATE)) - 1)

  /* setup dma channel 0 for tx buffer */
  DMACS0 = 0;
  DMA0CON = 0x2020; 
  DMA0PAD = 0x0442;
  DMA0CNT = 7;
  DMA0REQ = 0x0046;	
  DMA0STA = __builtin_dmaoffset(&ecan_bufs);
  DMA0CONbits.CHEN = 1;

  /* setup dma channel 2 for rx buffer */
  DMACS0 = 0;
  DMA2CON = 0x0020;
  DMA2PAD = 0x0440;	
  DMA2CNT = 7;
  DMA2REQ = 0x0022;	
  DMA2STA = __builtin_dmaoffset(&ecan_bufs);
  DMA2CONbits.CHEN = 1;

  /* setup pins */
  /* c1rx mapped to rb6 */
  /* c1tx mapped to rb7. refer to table 11-2. */
  RPINR26bits.C1RXR = 6;
  RPOR3bits.RP7R = 0x10;
  TRISBbits.TRISB6 = 1;
  TRISBbits.TRISB7 = 0;

  /* configuration mode */
  C1CTRL1bits.REQOP = 4;
  while (C1CTRL1bits.OPMODE != 4);

  /* FCAN is selected to be FCY */
  C1CTRL1bits.CANCKS = 0x1;

  C1CFG1bits.BRP = BRP_VAL;
  C1CFG1bits.SJW = 0x1;
  C1CFG2bits.SEG1PH = 0x2;
  C1CFG2bits.SEG2PHTS = 0x1;
  C1CFG2bits.SEG2PH = 0x2;
  C1CFG2bits.PRSEG = 0x1;
  C1CFG2bits.SAM = 0x1;

  /* 4 messages buffered in DMA RAM */
  C1FCTRLbits.DMABS = 0;

  /* dummy filter */
  C1CTRL1bits.WIN = 1;
  /* select acceptance mask 0 filter 0 buffer 1 */
  C1FMSKSEL1bits.F0MSK = 0;

  /* accept boot group messages targeted to id, whatever prio */
  C1RXM0SID = (0x3f7 << 5) | (1 << 3); /* mide bit */
  C1RXF0SID = MAKE_CAN_SID(0, BOOT_GROUP_ID, id) << 5;

  /* use buffer 1 for incoming messages */
  C1BUFPNT1bits.F0BP = 1;

  /* enable filter 0 */
  C1FEN1bits.FLTEN0 = 1;

  /* clear window bit to access ECAN control registers */
  C1CTRL1bits.WIN = 0;

  /* put the module in normal mode */
  C1CTRL1bits.REQOP = 0;
  while (C1CTRL1bits.OPMODE != 0) ;

  C1RXFUL1 = 0;
  C1RXFUL2 = 0;
  C1RXOVF1 = 0;
  C1RXOVF2 = 0;

  /* buffer N a transmit buffer */
  C1TR01CONbits.TXEN0 = 1;
  C1TR01CONbits.TXEN1 = 0;
  C1TR23CONbits.TXEN2 = 0;
  C1TR23CONbits.TXEN3 = 0;
}

static void ecan_write(uint8_t* s)
{
#define ecan_tx_buf (ecan_bufs[0])

  /* wait for previous transmission to end */
  while (C1TR01CONbits.TXREQ0) ;

  ecan_tx_buf[0] = MAKE_CAN_SID( HIGH_PRIO_ID, BOOT_GROUP_ID, HOST_NODE_ID ) << 2;

  ecan_tx_buf[1] = 0;
#define CAN_DATA_SIZE 8
  ecan_tx_buf[2] = CAN_DATA_SIZE;

  ecan_tx_buf[3] = ((uint16_t)s[1] << 8) | s[0];
  ecan_tx_buf[4] = ((uint16_t)s[3] << 8) | s[2];
  ecan_tx_buf[5] = ((uint16_t)s[5] << 8) | s[4];
  ecan_tx_buf[6] = ((uint16_t)s[7] << 8) | s[6];

  C1TR01CONbits.TXREQ0 = 1;
}

static inline unsigned int ecan_is_rx(void)
{
  /* return 0 if no rx buffer full */

  if (C1RXFUL1bits.RXFUL1) return 1;
  else if (C1RXFUL1bits.RXFUL2) return 2;
  else if (C1RXFUL1bits.RXFUL3) return 3;
  return 0;
}

static void ecan_read(uint8_t* s)
{
  unsigned int buf_index;

  while ((buf_index = ecan_is_rx()) == 0) ;

#define ecan_rx_buf (ecan_bufs[buf_index])

#if 0
  *id = (ecan_rx_buf[0] & 0x1ffc) >> 2;
#endif

  s[0] = (uint8_t)ecan_rx_buf[3];
  s[1] = (uint8_t)(ecan_rx_buf[3] >> 8);
  s[2] = (uint8_t)ecan_rx_buf[4];
  s[3] = (uint8_t)(ecan_rx_buf[4] >> 8);
  s[4] = (uint8_t)ecan_rx_buf[5];
  s[5] = (uint8_t)(ecan_rx_buf[5] >> 8);
  s[6] = (uint8_t)ecan_rx_buf[6];
  s[7] = (uint8_t)(ecan_rx_buf[6] >> 8);

  if (buf_index == 1) C1RXFUL1bits.RXFUL1 = 0;
  else if (buf_index == 2) C1RXFUL1bits.RXFUL2 = 0;
  else if (buf_index == 3) C1RXFUL1bits.RXFUL3 = 0;
}

#define com_setup ecan_setup
#define com_write ecan_write
#define com_read ecan_read

#else /* CONFIG_USE_UART */

static void uart_setup(uint16_t id)
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

  for (i = 0; i < CMD_BUF_SIZE; ++i, ++s)
  {
    while (U1STAbits.UTXBF) ;
    U1TXREG = *s;
  }
}

static void uart_read(uint8_t* s)
{
  /* todo: check for errors */

  unsigned int i;

  for (i = 0; i < CMD_BUF_SIZE; ++i, ++s)
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
(uint16_t haddr, uint16_t laddr, uint16_t data)
{
  /* read a 24 bits program word at addr to [data] */

  register uint16_t hdata asm("w7") = data + 2;

  __asm__ __volatile__
  (
   "mov %0, TBLPAG \n\t"
   "tblrdl [ %1 ], [ %3 ] \n\t"
   "tblrdh [ %1 ], [ %2 ] \n\t"

   :
   : "r"(haddr), "r"(laddr), "r"(hdata), "r"(data)
  );
}

static inline void write_program_word
(uint16_t haddr, uint16_t laddr, uint16_t hdata, uint16_t ldata)
{
  /* write a 24 bits program word at addr with data */
  /* note that the latch must be flushed to memory on a row basis */

  __asm__ __volatile__
  (
   "mov %0, TBLPAG \n\t"
   "tblwtl %3, [ %1 ] \n\t"
   "tblwth %2, [ %1 ] \n\t"
   :
   : "r"(haddr), "r"(laddr), "r"(hdata), "r"(ldata)
  );
}

static inline void erase_program_page
(uint16_t addrhi, uint16_t addrlo)
{
  register uint16_t tmp asm("w7");

  __asm__ __volatile__
  (
   "mov #0x4042, %0 \n\t"
   "mov %3, NVMCON \n\t"

   /* load page address */
   "mov %1, TBLPAG \n\t"
   "tblwtl %2, [ %2 ] \n\t"

   /* disable interrupt for 5 next insns */
   "disi #5 \n\t"

   /* protection sequence */
   "mov #0x55, %0 \n\t"
   "mov %3, NVMKEY \n\t"
   "mov #0xaa, %0 \n\t"
   "mov %3, NVMKEY \n\t"

   /* start erase sequence */
   "bset NVMCON, #15 \n\t"

   /* wait for end of sequence */
   "nop \n\t"
   "nop \n\t"
   "1: btsc NVMCON, #15 \n\t"
   "bra 1b \n\t"

   : "=&r"(tmp)
   : "r"(addrhi), "r"(addrlo), "0"(tmp)
  );
}

static inline void flush_program_latches(void)
{
  /* write one program memory row */
  /* latches previously filled with write_program_word */
  /* see erase_page for comments */

  register uint16_t tmp asm("w7");

  __asm__ __volatile__
  (
   "mov #0x4001, %0 \n\t"
   "mov %1, NVMCON \n\t"
   "disi #5 \n\t"
   "mov #0x55, %0 \n\t"
   "mov %1, NVMKEY \n\t"
   "mov #0xaa, %0 \n\t"
   "mov %1, NVMKEY \n\t"
   "bset NVMCON, #15 \n\t"
   "nop \n\t"
   "nop \n\t"
   "1: btsc NVMCON, #15 \n\t"
   "bra 1b \n\t"

   : "=&r"(tmp)
   : "0"(tmp)
  );
}

static inline void write_conf_byte(uint16_t hi, uint16_t lo, uint16_t x)
{
  register uint16_t tmp asm("w7");

  __asm__ __volatile__
  (
   "mov %1, TBLPAG \n\t"
   "tblwtl %3, [ %2 ] \n\t"
   "mov #0x4000, %0 \n\t"
   "mov %4, NVMCON \n\t"
   "disi #5 \n\t"
   "mov #0x55, %0 \n\t"
   "mov %4, NVMKEY \n\t"
   "mov #0xaa, %0 \n\t"
   "mov %4, NVMKEY \n\t"
   "bset NVMCON, #15 \n\t"
   "nop \n\t"
   "nop \n\t"
   "1: btsc NVMCON, #15 \n\t"
   "bra 1b \n\t"
   : "=&r"(tmp)
   : "r"(hi), "r"(lo), "r"(x), "0"(tmp)
  );
}


/* commands */

static inline uint16_t read_uint16_unaligned(uint8_t* p)
{
  return ((uint16_t)p[1] << 8) | p[0];
}

static inline uint32_t read_uint32_unaligned(uint8_t* p)
{
  return ((*(uint32_t*)(p + 1)) << 8) | p[0];
}

static void read_process_cmd(void)
{
#define ROW_WORD_COUNT 64
#define ROW_BYTE_COUNT (ROW_WORD_COUNT * 4) /* 256 */
#define PAGE_WORD_COUNT 512
#define PAGE_BYTE_COUNT (PAGE_WORD_COUNT * 4) /* 2048 */

  /* CMD_BUF_SIZE added to avoid overflow in com_read */
  uint8_t page_buf[PAGE_BYTE_COUNT + CMD_BUF_SIZE];
  uint8_t cmd_buf[CMD_BUF_SIZE];
  uint32_t addr;
  uint16_t size;
  uint16_t off;
  uint16_t i;
  uint16_t j;

  com_read(cmd_buf);

  switch (cmd_buf[0])
  {
  case CMD_ID_WRITE_PMEM:
    {
      /* write a program page */
      /* note that addr and size are in words, not bytes */

      addr = read_uint32_unaligned(cmd_buf + 1);
      size = read_uint16_unaligned(cmd_buf + 5);

      /* command ack */
      com_write(cmd_buf);

      /* read the page before erasing, if not a full page */
      off = addr % PAGE_WORD_COUNT;
      if (off || (size != PAGE_WORD_COUNT))
      {
	/* adjust addr to start of page */
	addr -= off;

	/* offset in bytes */
	off *= 2;

	/* read one word at a time */
	for (i = 0, j = 0; i < PAGE_BYTE_COUNT; i += 4, j += 2)
	{
	  const uint32_t tmp = addr + j;
	  read_program_word(HI(tmp), LO(tmp), (uint16_t)(page_buf + i));
	}
      }

      /* receive the page */
      for (i = 0; i < (size * 4); i += CMD_BUF_SIZE)
      {
	com_read(page_buf + off + i);

	/* frame ack */
	com_write(cmd_buf);
      }

      /* erase page */
      erase_program_page(HI(addr), LO(addr));

      /* write a whole page. i incremented by inner loop */
      for (i = 0; i < PAGE_BYTE_COUNT; )
      {
	/* fill the one row program memory latch one word at a time */
	for (j = 0; j < ROW_WORD_COUNT; i += 4, ++j, addr += 2)
	{
	  const uint32_t tmp = *(uint32_t*)(page_buf + i);
	  write_program_word(HI(addr), LO(addr), HI(tmp), LO(tmp));
	}

	/* finalize row write operation */
	flush_program_latches();
      }

      /* page programming ack */
      com_write(cmd_buf);

      break ;
    }

  case CMD_ID_READ_PMEM:
    {
      /* addr the first program word */
      /* size the program word count to read */

      addr = read_uint32_unaligned(cmd_buf + 1);
      size = read_uint16_unaligned(cmd_buf + 5);

      /* assume addr is aligned on program word boundary */
      /* assume size is a multiple of program word size */

      /* command ack */
      com_write(cmd_buf);

      while (size)
      {
	/* read 2 program insn at a time */
	for (i = 0; size && (i < CMD_BUF_SIZE); i += 4, addr += 2, size -= 1)
	  read_program_word(HI(addr), LO(addr), (uint16_t)(cmd_buf + i));

	com_write(cmd_buf);

	/* frame ack */
	com_read(cmd_buf);
      }

      break ;
    }

  case CMD_ID_WRITE_CMEM:
    {
      /* write device configuration registers */

      /* size in bytes */
      size = (0xf80018 - 0xf80000) * 2;
      addr = 0xf80000;

      /* command ack */
      com_write(cmd_buf);

      /* receive the data */
      for (i = 0; i < size; i += CMD_BUF_SIZE)
      {
	com_read(cmd_buf);

	/* program one byte at a time */
	for (j = 0; j < CMD_BUF_SIZE; j += 4, addr += 2)
	  write_conf_byte(HI(addr), LO(addr), (uint16_t)cmd_buf[j]);

	/* frame ack */
	com_write(cmd_buf);
      }

      /* page programming ack */
      com_write(cmd_buf);

      break ;
    }

  case CMD_ID_STATUS:
    {
      /* todo: reply status */

      /* debugging */
      for (i = 0; i < CMD_BUF_SIZE; ++i)
	cmd_buf[i] = 'a' + (uint8_t)i;

      com_write(cmd_buf);

      break ;
    }

  case CMD_ID_GOTO:
    {
      addr = read_uint32_unaligned(cmd_buf + 1);

      /* command ack */
      com_write(cmd_buf);

      /* jump to the entrypoint */
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
