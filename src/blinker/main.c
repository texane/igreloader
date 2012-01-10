#include <p33Fxxxx.h>
/* #include "bootloader.h" */


/* configuration bits */

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FBS(BWRP_WRPROTECT_OFF);


int main(void)
{
#if 0
  register unsigned int i;
  register unsigned int j;

  TRISAbits.TRISA0 = 0;

 redo:

  for (i = 0; i < 10000; ++i)
    for (j = 0; j < 1000; ++j)
      asm volatile ("nop");
  PORTAbits.RA0 = 0;

  for (i = 0; i < 10000; ++i)
    for (j = 0; j < 1000; ++j)
      asm volatile ("nop");
  PORTAbits.RA0 = 1;

  goto redo;
#else
  TRISAbits.TRISA0 = 0;
  PORTAbits.RA0 = 1;
  while (1) ;
#endif

  return 0;
}
