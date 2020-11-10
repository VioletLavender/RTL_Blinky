/*----------------------------------------------------------------------------*/
/* This file implement MCU peripherals like: UART, GPIO, etc for BLE          */
/*----------------------------------------------------------------------------*/

#include "porting_flash.h"
//#include "NuMicro.h"
#include "host.h"

/*******************************************************************
 *      Const Code Decelaration
 *******************************************************************/
//define flash array for BLE bonding information / data information
#ifdef _SMP_ON_
#ifdef _BOND_ON_

#if defined (__CC_ARM)
const uint8_t INFO_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK][SIZE_OF_FLASH_PAGE] __attribute__((at(BONDING_INFORMATION_ADDRESS)));
const uint8_t DATA_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK][SIZE_OF_FLASH_PAGE] __attribute__((at(BONDING_DATA_ADDRESS)));

#elif defined (__ICCARM__)
const uint8_t INFO_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK][SIZE_OF_FLASH_PAGE] @(BONDING_INFORMATION_ADDRESS);
const uint8_t DATA_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK][SIZE_OF_FLASH_PAGE] @(BONDING_DATA_ADDRESS);

#endif

/* flash program function for BLE */
void flash_program_internal_use(uint32_t u32Addr, uint32_t u32Data)
{
//    __disable_irq();
//    /* Unlock protected registers */
//    SYS_UnlockReg();
//    /* Enable FMC ISP function */
//    FMC_Open();

//    FMC_ENABLE_AP_UPDATE();
//    FMC_Write(u32Addr, u32Data);
//    FMC_DISABLE_AP_UPDATE();

//    /* Disable FMC ISP function */
//    FMC_Close();
//    /* Lock protected registers */
//    SYS_LockReg();
//    __enable_irq();
}

/* flash erase function for BLE */
int32_t flash_erase_internal_use(uint32_t u32Addr)
{
//    int32_t EraseStatus = 0;

//    __disable_irq();

//    /* Unlock protected registers */
//    SYS_UnlockReg();
//    /* Enable FMC ISP function */
//    FMC_Open();

//    FMC_ENABLE_AP_UPDATE();
//    EraseStatus = FMC_Erase(u32Addr);
//    FMC_DISABLE_AP_UPDATE();

//    /* Disable FMC ISP function */
//    FMC_Close();
//    /* Lock protected registers */
//    SYS_LockReg();

//    __enable_irq();

//    return EraseStatus;
}
#endif
#endif


