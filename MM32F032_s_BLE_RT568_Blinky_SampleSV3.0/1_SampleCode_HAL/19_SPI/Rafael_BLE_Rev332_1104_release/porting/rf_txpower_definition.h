#ifndef __RF_TXPOWER_DEFINITION_H__
#define __RF_TXPOWER_DEFINITION_H__
/*----------------------------------------------------------------------------*/
/* This file implement RF TX Power Register Settings                          */
/*----------------------------------------------------------------------------*/
#include "mcu_definition.h"


/** BLE TX power level definition */
static uint8_t TXPOWER_0DBM[] = {0xEF, 0x42, 0x81, 0x40, 0x2A};   /**< 0 dBm.   */
static uint8_t TXPOWER_4DBM[] = {0xEF, 0x82, 0xA4, 0x5A, 0x05};   /**< 4 dBm.   */
static uint8_t TXPOWER_8DBM[] = {0xEF, 0xD2, 0x8E, 0x7A, 0x05};   /**< 8 dBm.   */
static uint8_t TXPOWER_10DBM[] = {0xEF, 0xC2, 0x8F, 0x7A, 0x05};  /**< 10 dBm.   */


#endif //__RF_TXPOWER_DEFINITION_H__
