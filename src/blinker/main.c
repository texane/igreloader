#include <p33Fxxxx.h>


#define CONFIG_USE_INTERRUPT 1


/* configuration bits */

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


/* timer interrupt. test if it is written correcly. */

static unsigned char x = 0;
static unsigned char a = 0; 

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
  if (IFS0bits.T1IF == 0) return ;

  if (((++a) & 3) == 0)
  {
    PORTAbits.RA0 = x & 1;
    x ^= 1;
  }

  IFS0bits.T1IF = 0;
}

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

int main(void)
{
  AD1PCFGL = 0xFFFF;

  osc_setup();

  TRISAbits.TRISA0 = 0;

  PORTAbits.RA0 = 1;

  /* timer0 configured in timer mode */
  IEC0bits.T1IE = 0;
  T1CONbits.TON = 0;
  IFS0bits.T1IF = 0;
  IEC0bits.T1IE = 1;

  /* clocked from FCY / prescaler (256) */
  T1CONbits.TCKPS = 3;
  TMR1 = 0x0000;
  PR1 = 0xffff;

  /* start the timer */
  T1CONbits.TON = 1;

  while (1) ;

  return 0;
}
