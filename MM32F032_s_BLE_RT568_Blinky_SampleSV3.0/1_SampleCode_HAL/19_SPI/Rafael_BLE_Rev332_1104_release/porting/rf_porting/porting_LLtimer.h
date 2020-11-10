#ifndef __PORTING_LLTIMER_H__
#define __PORTING_LLTIMER_H__

#include "ble_basicType.h"   //Uint8 define
#include "mcu_definition.h"  //include "stdint.h" for uint16_t        


//extern void Tiny_Delay(uint32_t u32Usec);
extern void delay_us(u16 us);
extern void delay_ms(u16 ms);


#endif  //__PORTING_LLTIMER_H__


