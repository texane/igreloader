#include <p33Fxxxx.h>


/* TODO
   idle mode requires data to be processed
   in the interrupt handler. in polling mode
   race may occur between checking for data
   availability and pwrsav instruction, and
   the cpu would idle forever.
 */
#define CONFIG_USE_IDLE 0


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
  /* baud rate */
#define FCAN 40000000 
#define BITRATE 125000  
#define NTQ 10
#define BRP_VAL ((FCAN / (2 * NTQ * BITRATE)) - 1)

  /* setup pins */
  /* c1rx mapped to rb6 */
  /* c1tx mapped to rb7. refer to table 11-2. */
  RPINR26bits.C1RXR = 6;
  RPOR3bits.RP7R = 0x10;
  TRISBbits.TRISB6 = 1;
  TRISBbits.TRISB7 = 0;

  /* configuration mode */
  C1CTRL1bits.REQOP=4;
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
	
  /* 4 CAN Messages to be buffered in DMA RAM */	
  C1FCTRLbits.DMABS = 0b000;

  /* todo: message filters */
		
  /* put the module in normal mode */
  /* todo: put in listen all mode */
  C1CTRL1bits.REQOP = 0;
  while (C1CTRL1bits.OPMODE != 0) ;

#if 0 /* todo: enable module */
  /* clear the buffer and overflow flags */
  C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
  /* ECAN1, Buffer 0 is a Transmit Buffer */
  C1TR01CONbits.TXEN0=1;			
  /* ECAN1, Buffer 1 is a Receive Buffer */
  C1TR01CONbits.TXEN1=0;	
  /* ECAN1, Buffer 2 is a Receive Buffer */
  C1TR23CONbits.TXEN2=0;	
  /* ECAN1, Buffer 3 is a Receive Buffer */
  C1TR23CONbits.TXEN3=0;	
  /* Message Buffer 0 Priority Level */
  C1TR01CONbits.TX0PRI=0b11; 		
#endif
		
  /* configure the device to interrupt on the receive buffer full flag */
  /* clear the buffer full flags */
  C1RXFUL1=0;
  C1INTFbits.RBIF=0;
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

#if CONFIG_USE_IDLE

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
  /* data not processed here. only used to leave idle mode. */
  IFS0bits.U1RXIF = 0;
  return ;
}

#endif

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

#if CONFIG_USE_IDLE
  IFS0bits.U1RXIF = 0;
  IPC2bits.U1RXIP = 3;
  IEC0bits.U1RXIE = 1;
#endif
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


#if CONFIG_USE_IDLE

/* enter idle mode */

static inline void idle(void)
{
  __asm__ __volatile__
  (
   "disi #3 \n\t"
   "bts U1STA, #URXDA \n\t"
   "bs skip_idle \n\t"
   "pwrsav #1 \n\t"
   "skip_idle:\n\t"
  );
}

#endif


/* main */

int main(void)
{
  uint8_t buf[CAN_DATA_SIZE];
  uint16_t id;

  osc_setup();
  uart_setup();
  ecan_setup();

  while (1)
  {
#if CONFIG_USE_IDLE
    idle();
#else
    if (uart_is_rx())
    {
      uart_read(&id, buf);
      ecan_write(id, buf);
    }

    if (ecan_is_rx())
    {
      ecan_read(&id, buf);
      uart_write(id, buf);
    }
#endif
  }

  return 0;
}
