/*******************************************************************
 *
 * File Name  : L2CAP.H
 * Description:
 *
 *
 *
 *******************************************************************
 *
 *      Copyright (c) 2020, All Right Reserved
 *      Rafael Microelectronics Co. Ltd.
 *      Taiwan, R.O.C.
 *
 *******************************************************************/

#ifndef _BLE_L2CAP_H_
#define _BLE_L2CAP_H_

#define L2CAP_E0                                0x80
#define ML2CAP_EVENT_NULL                       (L2CAP_E0+0x00)

typedef struct L2CAPBuf
{
    Uint8 DataBuf[251];         //251: maximun LL MTU after Bluetooth Spec. ver.4.2
    Uint8 connID_LL;
    Uint8 len_L2CAPBuf;
} L2CAPBuf;
extern L2CAPBuf L2CAP_Buf;
#endif
