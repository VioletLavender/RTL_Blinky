//----------------------------------------------------------------------------//
// This file implement MCU Timer peripherals for Link Layer
//
//----------------------------------------------------------------------------//
#include "porting_LLtimer.h"
//#include "rf_phy.h"


/******************************************************************************
 * Public Functions
 ******************************************************************************/

//void Tiny_Delay(uint32_t u32Usec)
//{
//    CLK_SysTickDelay(u32Usec);
//}

#if defined(ewarm) || defined(DOXYGEN)
void SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif

#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif

extern uint32_t SystemCoreClock ;
#if defined(rvmdk) || defined(__ARMCC_VERSION)
__asm void
SysCtlDelay(uint32_t ui32Count)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif

void delay_us(u16 us)
{
    u32 xus = us;
    SysCtlDelay(xus * (72000000 / 4000000));
}

void delay_ms(u16 ms)
{
    u32 xms = ms;
    SysCtlDelay(xms * (1000 * (SystemCoreClock / 4000000)));
}




