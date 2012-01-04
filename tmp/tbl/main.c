#include <p33Fxxxx.h>


_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;


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


static void uart_setup(void)
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

static void uart_write(uint8_t* s, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; ++i, ++s)
  {
    while (U1STAbits.UTXBF) ;
    U1TXREG = *s;
  }
}

static uint8_t uart_read(void)
{
  /* todo: check for errors */

  uint8_t x;

  while (U1STAbits.URXDA == 0) ;
  x = U1RXREG;
}


/* program memory routines */

static inline void read_program_word
(uint16_t haddr, uint16_t laddr, uint16_t data)
{
  /* read a 24 bits program word at addr to [data] */

  __asm__ __volatile__
  (
   "mov %0, TBLPAG \n\t"
   "tblrdl [ %1 ], [ %2++ ] \n\t"
   "tblrdh.b [ %1 ], [ %2 ] \n\t"
   :
   : "r"(haddr), "r"(laddr), "r"(data)
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
   "tblwtl %0, [ %2 ] \n\t"

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


static uint8_t* to_hex(uint8_t x)
{
  static uint8_t buf[2];
  uint8_t nibble;

  nibble = x >> 4;
  if (nibble >= 0xa) buf[0] = 'a' + nibble - 0xa;
  else buf[0] = '0' + nibble;

  nibble = x & 0xf;
  if (nibble >= 0xa) buf[1] = 'a' + nibble - 0xa;
  else buf[1] = '0' + nibble;

  return buf;
}


static inline uint8_t from_hex(uint8_t c)
{
  if (c >= 'a') return 0xa + c - 'a';
  return c - '0';
}


static uint32_t read_uint32(void)
{
  uint32_t addr = 0;
  unsigned int i;

  for (i = 0; i < 32; i += 4)
    addr |= (uint32_t)from_hex(uart_read()) << (28 - i);

  return addr;
}


static uint8_t read_cmd(void)
{
  return uart_read();
}


int main(void)
{
  uint32_t addr;
  unsigned int i;
  uint8_t buf[4];
  uint8_t cmd;

  osc_setup();

  uart_setup();

  while (1)
  {
    cmd = read_cmd();

    buf[0] = '\r';
    buf[1] = '\n';
    uart_write(buf, 2);

    addr = read_uint32();

    buf[0] = '\r';
    buf[1] = '\n';
    uart_write(buf, 2);

    if (cmd == 'r')
    {
      buf[0] = 0x2a;
      buf[1] = 0x2a;
      buf[2] = 0x2a;
      buf[3] = 0;
      read_program_word(addr >> 16, addr & 0xffff, (uint16_t)buf);
      for (i = 0; i < 4; ++i) uart_write(to_hex(buf[3 - i]), 2);

      buf[0] = '\r';
      buf[1] = '\n';
      uart_write(buf, 2);
    }
    else if (cmd == 'w')
    {
      const uint16_t hi = addr >> 16;
      const uint16_t lo = addr & 0xffff;

      erase_program_page(hi, lo);
      write_program_word(hi, lo, 0x0102, 0x0304);
      flush_program_latches();

      buf[0] = '\r';
      buf[1] = '\n';
      uart_write(buf, 2);
    }
  }

  return 0;
}
