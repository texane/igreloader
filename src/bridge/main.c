#include <p33Fxxxx.h>


/* configuration bits */

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


/* int types */

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;


/* polling or interrupt */

#define CONFIG_USE_INTERRUPT 0


/* notes
   UART wraps CAN frames using the following format:
   <CAN_ID:2>,<CAN_PAYLOAD:8>
 */

#define CAN_DATA_SIZE 8


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

uint16_t ecan_bufs[4][8] __attribute__((space(dma), aligned(16)));

#if CONFIG_USE_INTERRUPT

static inline unsigned int ecan_is_rx(void);
static void ecan_read(unsigned int, uint16_t*, uint8_t*);
static void uart_write(uint16_t, uint8_t*);

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{
  if (C1INTFbits.TBIF)
  {
    C1INTFbits.TBIF = 0;	    
  }

  if (C1INTFbits.RBIF)
  {
    const unsigned int buf_index = ecan_is_rx();
    if (buf_index)
    {
      static uint8_t buf[CAN_DATA_SIZE];
      uint16_t id;
      ecan_read(buf_index, &id, buf);
      uart_write(id, buf);
    }

    C1INTFbits.RBIF = 0;
  }

  IFS2bits.C1IF = 0;
}

#endif /* CONFIG_USE_INTERRUPT */

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
	
  /* 4 messages buffered in DMA RAM */
  C1FCTRLbits.DMABS = 0;

  /* dummy filter */
  C1CTRL1bits.WIN = 1;
  /* select acceptance mask 0 filter 0 buffer 1 */
  C1FMSKSEL1bits.F0MSK = 0;

  /* accept all standard sids */
  C1RXM0SID = 0;
  C1RXF0SID = 0;
  C1RXM0SID = 0x0008; /* mide bit */
  C1RXF0SID = 0;

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

  /* buffer N a transmit or receive buffer */
  C1TR01CONbits.TXEN0 = 1;
  C1TR01CONbits.TXEN1 = 0;
  C1TR23CONbits.TXEN2 = 0;
  C1TR23CONbits.TXEN3 = 0;

  /* configure the device to interrupt on the receive buffer full flag */
  /* clear the buffer full flags */
  C1RXFUL1 = 0;
  C1INTFbits.RBIF = 0;

  /* setup dma channel 0 for tx buffer */
  DMACS0 = 0;
  DMA0CON = 0x2020; 
  DMA0PAD = 0x0442;
  DMA0CNT = 7;
  DMA0REQ = 0x0046;	
  DMA0STA = __builtin_dmaoffset(&ecan_bufs);
  DMA0CONbits.CHEN = 1;
	
  /* setup the DMA channel 2 for rx buffer */
  DMACS0 = 0;
  DMA2CON = 0x0020;
  DMA2PAD = 0x0440;	
  DMA2CNT = 7;
  DMA2REQ = 0x0022;	
  DMA2STA = __builtin_dmaoffset(&ecan_bufs);
  DMA2CONbits.CHEN = 1;

#if CONFIG_USE_INTERRUPT
  IEC2bits.C1IE = 1;
  C1INTEbits.TBIE = 1;
  C1INTEbits.RBIE = 1;
#endif
}

static inline unsigned int ecan_is_rx(void)
{
  /* return 0 if no rx buffer full */

  if (C1RXFUL1bits.RXFUL1) return 1;
  else if (C1RXFUL1bits.RXFUL2) return 2;
  else if (C1RXFUL1bits.RXFUL3) return 3;
  return 0;
}

static void ecan_write(uint16_t id, uint8_t* s)
{
#define ecan_tx_buf (ecan_bufs[0])

  /* wait for previous transmission to end */
  while (C1TR01CONbits.TXREQ0) ;

  ecan_tx_buf[0] = id << 2;
  ecan_tx_buf[1] = 0;
  ecan_tx_buf[2] = CAN_DATA_SIZE;

  ecan_tx_buf[3] = ((uint16_t)s[1] << 8) | s[0];
  ecan_tx_buf[4] = ((uint16_t)s[3] << 8) | s[2];
  ecan_tx_buf[5] = ((uint16_t)s[5] << 8) | s[4];
  ecan_tx_buf[6] = ((uint16_t)s[7] << 8) | s[6];

  C1TR01CONbits.TXREQ0 = 1;
}

static void ecan_read(unsigned int buf_index, uint16_t* id, uint8_t* s)
{
#define ecan_rx_buf (ecan_bufs[buf_index])

  *id = (ecan_rx_buf[0] & 0x1ffc) >> 2;

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


/* uart */

#if CONFIG_USE_INTERRUPT

static void uart_read(uint16_t*, uint8_t*);
static inline unsigned int uart_is_rx(void);

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
  static uint8_t buf[CAN_DATA_SIZE];
  uint16_t id;

  if (uart_is_rx())
  {
    uart_read(&id, buf);
    ecan_write(id, buf);
  }

  IFS0bits.U1RXIF = 0;
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

#if CONFIG_USE_INTERRUPT
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
  return (U1STAbits.URXDA != 0);
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


#if CONFIG_USE_INTERRUPT

/* enter idle mode */

static inline void idle(void)
{
  __asm__ __volatile__
  (
   "pwrsav #1 \n\t"
  );
}

#endif /* CONFIG_USE_INTERRUPT */


/* main */

int main(void)
{
#if (CONFIG_USE_INTERRUPT == 0)
  uint8_t buf[CAN_DATA_SIZE];
  uint16_t id;
  unsigned int buf_index;
#endif

  osc_setup();
  uart_setup();
  ecan_setup();

  while (1)
  {
#if CONFIG_USE_INTERRUPT
    idle();
#else /* polling */
    if (uart_is_rx())
    {
      uart_read(&id, buf);
      ecan_write(id, buf);
    }

    buf_index = ecan_is_rx();
    if (buf_index)
    {
      ecan_read(buf_index, &id, buf);
      uart_write(id, buf);
    }
#endif
  }

  return 0;
}
