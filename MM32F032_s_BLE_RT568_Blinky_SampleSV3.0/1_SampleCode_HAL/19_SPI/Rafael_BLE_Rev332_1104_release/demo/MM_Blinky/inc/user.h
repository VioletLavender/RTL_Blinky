#ifndef __USER_H__
#define __USER_H__

#include "config.h"
#include "ble_cmd.h"
void BleApp_Init(void);
void Ble_Slave_AdvInit(void);
void Ble_Slave_StartADV(void);
void Ble_Master_Init(void);
void Ble_Initial(uint8_t role);
void BleApp_Main(void);
void trspx_receive_data_callback(uint8_t length, uint8_t *data);
static void BleEvent_Callback(BleCmdEvent event, void* param);
void trspx_send(uint8_t *data, uint16_t len);
void BleApp_Main(void);
#endif


