#ifndef __MCU_DEFINITION_H__
#define __MCU_DEFINITION_H__

#include <stdint.h>
#include "config.h"


//#define _MCU_ARM_CORTEX_   //no actual use, just for InterruptDisable

/* InterruptEnable */
#ifdef _MCU_ARM_CORTEX_
//#define InterruptDisable      __disable_irq
//#define InterruptEnable       __enable_irq
#endif  //(#ifdef _MCU_ARM_CORTEX_)


//#define __NOP  

#endif  //__MCU_DEFINITION_H__


