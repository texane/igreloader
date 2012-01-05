#include <p33Fxxxx.h>


/* notes
   UART wraps CAN frames using the following format:
   <CAN_ID:2>,<CAN_PAYLOAD:8>
 */


/* configuration bits */

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


/* int types */

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;


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


/* ecan */

#define CAN_DATA_SIZE 8

static void ecan_setup(void)
{
  /* todo: configure in listenall mode */
}

static void ecan_write(uint16_t id, uint8_t* s)
{
  /* todo */
}

static void ecan_read(uint16_t* id, uint8_t* s)
{
  /* todo */
}

static inline unsigned int ecan_is_rx(void)
{
  /* todo */

  return 0;
}


/* uart */

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

static inline void uart_write_uint8(uint8_t x)
{
  while (U1STAbits.UTXBF) ;
  U1TXREG = x;
}

static void uart_write(uint16_t id, uint8_t* s)
{
  unsigned int i;

  /* write id, little first */
  uart_write_uint8(id & 0xff);
  uart_write_uint8(id >> 8);

  for (i = 0; i < CAN_DATA_SIZE; ++i, ++s)
    uart_write_uint8(*s);
}

static inline unsigned int uart_is_rx(void)
{
  return U1STAbits.URXDA != 0;
}

static inline uint8_t uart_read_uint8(void)
{
  while (uart_is_rx() == 0) ;
  return U1RXREG;
}

static void uart_read(uint16_t* id, uint8_t* s)
{
  unsigned int i;

  /* id sent little endian */
  i = uart_read_uint8();
  *id = (uart_read_uint8() << 8) | (uint8_t)i;

  for (i = 0; i < CAN_DATA_SIZE; ++i, ++s)
    *s = uart_read_uint8();
}


/* main */

int main(void)
{
  uint8_t buf[CAN_DATA_SIZE];
  uint16_t id;
  unsigned int do_sleep;

  osc_setup();
  uart_setup();
  ecan_setup();

  while (1)
  {
    do_sleep = 1;

    if (uart_is_rx())
    {
      do_sleep = 0;
      uart_read(&id, buf);
      ecan_write(id, buf);
    }

    if (ecan_is_rx())
    {
      do_sleep = 0;
      ecan_read(&id, buf);
      uart_write(id, buf);
    }

    if (do_sleep)
    {
      /* todo: enter sleep mode */
    }
  }

  return 0;
}
