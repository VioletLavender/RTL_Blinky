/******************************************************************************
 * @file    uart.h
 * @author  King
 * @version V1.00
 * @date    20-May-2020
 * @brief   ......
 ******************************************************************************
 * @attention
 * 
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 * <H2><CENTER>&COPY; COPYRIGHT 2020 MINDMOTION </CENTER></H2>
******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif


#undef  EXTERN


#ifdef  __UART_C__
#define EXTERN
#else
#define EXTERN extern
#endif


#define UART_REC_LEN  			200  	//定义最大接收字节数 200
#define RX_buffer_size 100



/* Includes -----------------------------------------------------------------*/
#include "config.h"

/* Exported constants -------------------------------------------------------*/
/* Exported types -----------------------------------------------------------*/
/* Exported macro -----------------------------------------------------------*/



/* Exported functions -------------------------------------------------------*/
EXTERN void UARTx_Configure(UART_TypeDef           *UARTx,
                            uint32_t                BaudRate,
                            UART_WordLength_TypeDef WordLength,
                            UART_Stop_Bits_TypeDef  StopBits,
                            UART_Parity_TypeDef     Parity);

void DMA_Config(DMA_Channel_TypeDef* dam_chx, u32 cpar, u32 cmar, u16 cndtr);
#ifdef __cplusplus
}
#endif
void UART_nvic_config(void);
void UartSendByte(uint8_t dat);
void UartSendGroup(uint8_t* buf, uint8_t len);
void UART_TX_Send(uint32_t len, uint8_t *ptr);        //show data on UART
#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

