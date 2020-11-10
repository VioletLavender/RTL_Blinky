/**************************************************************************//**
* @file       rf_phy.h
* @brief      This file provides the declaration that RF IC control needed.
*
* @defgroup rf_phy RF PHY
* @{
* @details  Provides the declaration that RF IC control needed. (rf_phy.h).
* @}
*****************************************************************************/

#ifndef __RF_PHY_H__
#define __RF_PHY_H__

#include "BleAppSetting.h"
#include "ble_cmd.h"


/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup rf_phy_def RF PHY Definition.
 * @{
 * @details Here shows the definitions of RF PHY.
 * @note Do not modify the value of definitions.
 * @ingroup rf_phy
 * @}
 */

/** @defgroup rf_phy_def_chipId Chip ID Definition
 * @{
 * @ingroup rf_phy_def
 */
#define MP_A1      102u                  /**< chip_id = 102 (0x66). */
#define MP_A2      103u                  /**< chip_id = 103 (0x67). */
/** @} */


/** @defgroup rf_phy_def_TRxWritePort TX/RX Buffer Write Port Definition
 * @{
 * @ingroup rf_phy_def
 */
#define RFIP_REG_254            254U          /**< Register 254. */
#define RFIP_REG_255            255U          /**< Register 255. */


#define TX_BUFFER_WRITE_PORT    RFIP_REG_254  /**< TX Buffer Write Port. */
#define RX_BUFFER_READ_PORT     RFIP_REG_255  /**< RX Buffer Write Port. */
/** @} */


/** @defgroup mcuWakeupTime MCU Wake-Up Required Time from Sleep Mode
 * @{
 * @ingroup rf_phy_def
 *
 * This parameter adjusts the retention time required for the MCU to wake up from sleep mode, plus some necessary parameters set via SPI operation.
 * The unit of the MCU wake-up retention time is 125us.
 *
 * @note The range of MCU wake-up retention time is 125us to 2ms.
*/
#define MCU_WAKEUP_RETENSIONTIME_MAX  16 /**< 2ms = 16*125us */
#define MCU_WAKEUP_RETENSIONTIME_MIN  1  /**< 125us = 1*125us */
/** @} */


/**
 * @ingroup rf_phy_def
 * @defgroup ble_cmd_rfMode BLE RF Mode Definition
 * @{
 * @details BLE RF mode definition.
 * @name BleRFMode
 * @brief  The definition of BLE RF Mode.
 * @{
 */
typedef Uint8 BleRF_Mode;
#define BLERFMODE_ACTIVE           0x00 /**< RF is in active mode. */
#define BLERFMODE_SLEEP            0x01 /**< RF entered sleep mode. */
/** @} */
/** @} */


//========== SW Feature options =========//
// will be moved to _rafael_phy.h in SDK
//==========================================

/** use RF timer as LL timerU
 * @ingroup rf_phy_def
 */
#define _TMR_USE_INTERNAL_      //Use RF internal RTC as timer
#ifndef _HCI_HW

/** Enable Auto-Sleep Function by LL.
 * @ingroup rf_phy_def
 */
//#define _PWR_DOWN_BY_LL_        //Un-define it to stop the auto-sleep function by LL.

#ifndef _APP_CURRENT_TEST_

#endif

#endif


#define _SW_PATCH_MP_
#ifndef _TMR_USE_INTERNAL_
#undef _SW_PATCH_MP_
#endif

#ifdef _TMR_USE_INTERNAL_
//#define DUR_LL_WAKEUP_MIN       32+6    //4us, 1.6
//#define DUR_LL_WAKEUP_MIN       800     //OK
//#define DUR_LL_WAKEUP_MIN       240     //OK
//#define DUR_LL_WAKEUP_MIN       1600    //OK
//#define DUR_LL_WAKEUP_MIN       80    //6us
//#define DUR_LL_WAKEUP_MIN       60    //16us, 4
//#define DUR_LL_WAKEUP_MIN       70    //10us
//#define DUR_LL_WAKEUP_MIN       75    //11, 3us
//#define DUR_LL_WAKEUP_MIN       78    //6us
//#define DUR_LL_WAKEUP_MIN       76    //10us
#define DUR_LL_WAKEUP_MIN       1600    //1600*125u=200ms, sleep mode 33us deviation
#endif


/** Enable WDT Reset.
 * @ingroup rf_phy_def
 */
//#define _HW_PRG_RESET_


/** LL Power-Saving Duration. (1600*125u=200ms)
 * @ingroup rf_phy_def
 */
#define DUR_LL_WAKEUP_MIN                 1600    //1600*125u=200ms, LL power-saving duration


/** Free Time Period to the Next Event over 10ms will make the IC Go Sleep. (example: 80*125us=10ms)
 * @ingroup rf_phy_def
 */
#define DUR_PWR_DOWN_PERIOD_FCTR_125MULT  40   //125*n (us), example: 40*125us=5ms, free time period to the next event over 10ms will make the IC go sleep.
//================================================



/**************************************************************************
 * Functions
 **************************************************************************/

/** @defgroup rf_phy_function RF PHY Function
 * @{
 * @details Here shows the functions of RF PHY.
 * @ingroup rf_phy
 * @}
 */

/** Initializes the BLE stack.
 * @ingroup rf_phy_function 
 * @attention Must initial BLE stack after @ref RF_SpiIoMapping.
 */
void RF_Init(void);



/** Get BLE Device Chip ID
 *
 *  This function is used to get BLE Device Chip ID.
 *
 * @ingroup rf_phy_function
 *
 * @return  @ref rf_phy_def_chipId "Chip ID"
 */
uint8_t ChipId_Get(void);


/** Get BLE LL Timeline.
 *  Timeline unit is 125us
 *
 * @ingroup rf_phy_function
 */
uint32_t LLTimeline_Get(void); // will be moved to _rafael_phy.h in SDK


/** Get TX FIFI Address
 *
 *  This function is used to get TX FIFO RAM start address.
 *
 * @ingroup rf_phy_function
 *
 * @return  TX FIFO address.
 */
uint32_t BleTxFIFOAddr_Get(void);



/** Get Current BLE Device RF Mode
 *
 *  This function is used to get current RF mode.
 *
 * @ingroup rf_phy_function
 *
 * @retval  BLERFMODE_ACTIVE : The BLE device RF is in active mode.
 * @retval  BLERFMODE_SLEEP  : The BLE device RF is in sleep mode.
 */
BleRF_Mode BleRFMode_Get(void);


/** Enable BLE Kernel.
 *  Must do it after @ref RF_SpiIoMapping and @ref RF_Init.
 *
 * @ingroup rf_phy_function
 * @note If no any BLE Stack (LL&Host) task is running then run user application task. \n
 * The task priority is LL>Host>User_app.
 *
 * @retval  BLESTACK_STATUS_FREE     : Stack (LL&Host task) is not running, can run user application task.
 * @retval  BLESTACK_STATUS_ERR_BUSY : Stack (LL&Host task) is running, can NOT run user application task.
 */
BleStackStatus Ble_Kernel_Root(void);



/** Set BLE RSSI Offset Value to adjust RSSI Value
 *
 * @ingroup rf_phy_function
 *
 * @note The default BLE RSSI Offset Value is +26dBm.
 * @return none
 */
void setRF_RssiValueOffset(uint8_t offset);


/** Set RF 16MHz Clock output
 *
 * This function is used to enable RF 16MHz Clock output.
 *
 * @ingroup rf_phy_function
 *
 * @attention This command MUST be called before @ref RF_Init. \n
 * @note RF sleep mode will be disabled automatically when user enable RF 16MHz Clock output.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : RF 16MHz Clock output CANNOT be enabled while running BLE stack.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setRF_Enable16MHzOutput(void);


/** Set RF Enter Deep Sleep Mode
 *
 * @ingroup rf_phy_function
 * @attention   RF can NOT enter Deep Sleep Mode if enabled 16M CLK output. \n
 * @note  It will not wake up until call @ref setRF_WakeUpFromDeepSleep. \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : This command is not supported if RF 16MHz Clock output enabled.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setRF_EnterDeepSleep(void);


/** Set RF External Wakeup From Deep Sleep Mode
 *
 * @ingroup rf_phy_function
 * @note   This command is not supported if enabled 16M CLK output. \n
 * @note   After external wakeup from deep sleep mode, RF will do @ref RF_Init again. \n
 *         Then RF is in standby mode.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : This command is not supported if RF 16MHz Clock output enabled.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setRF_WakeUpFromDeepSleep(void);


/** Set MCU Wake-up Retention Time
 *
 * This parameter adjusts the retention time required for the MCU to wake up from sleep mode, plus some necessary parameters set via SPI operation. \n
 * The unit of the MCU wake-up retention time is 125us.
 *
 * @ingroup rf_phy_function
 *
 * @attention This command MUST be called before @ref RF_Init. \n
 * @note The default MCU Wake-up Retention Time is set to 5(625us = 5*125us). \n
 * @note MCU wake-up retention time range is from @ref MCU_WAKEUP_RETENSIONTIME_MIN to @ref MCU_WAKEUP_RETENSIONTIME_MAX. \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : MCU Wake-up Retention Time CANNOT be modified while running BLE stack.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setMCU_WakeupRetentionTime(uint8_t retentionTime);

#endif
