/******************************************************************************
 * @file    uart.c
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
#define __UART_C__


/* Includes -----------------------------------------------------------------*/
#include "uart.h"
#include "user.h"

u8 gSendBuff[RX_buffer_size] = {0};
u8 rf_Buff[RX_buffer_size] = {0};
bool transport_flag = false;
uint8_t UART_RX_BUF[UART_REC_LEN];
uint16_t UART_RX_STA = 0;



/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/


/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention
******************************************************************************/
void UARTx_Configure(UART_TypeDef           *UARTx,
                     uint32_t                BaudRate,
                     UART_WordLength_TypeDef WordLength,
                     UART_Stop_Bits_TypeDef  StopBits,
                     UART_Parity_TypeDef     Parity)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    if (UARTx == UART1)
    {
        /* Enable UART1 clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART1, &UART_InitStructure);


        UART_ITConfig(UART1, UART_IER_RXIDLE, ENABLE);
        UART_DMACmd(UART1, UART_DMAReq_EN, ENABLE);


        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA9 & PA10 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        DMA_Config(DMA1_Channel3, (u32)&UART1->RDR, (u32)gSendBuff, 100);
        UART_nvic_config();
//              UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//开启串口接受中断

        /* Enable UART */
        UART_Cmd(UART1, ENABLE);
    }
    else if (UARTx == UART2)
    {
        /* Enable UART2 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART2, &UART_InitStructure);


        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA2 & PA3 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Enable UART */
        UART_Cmd(UART2, ENABLE);
    }
    else
    {
    }
}

void DMA_Config(DMA_Channel_TypeDef *dam_chx, u32 cpar, u32 cmar, u16 cndtr)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(dam_chx);
    DMA_StructInit(&DMA_InitStructure);
    //DMA transfer peripheral address
    DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;
    //DMA transfer memory address
    DMA_InitStructure.DMA_MemoryBaseAddr = cmar;
    //DMA transfer direction from peripheral to memory
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //DMA cache size
    DMA_InitStructure.DMA_BufferSize = cndtr;
    //After receiving the data, the peripheral address is forbidden to move
    //backward
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //After receiving the data, the memory address is shifted backward
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //Define the peripheral data width to 8 bits
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    //M2M mode is disabled
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(dam_chx, &DMA_InitStructure);

    UART_DMACmd(UART1, UART_DMAReq_EN, ENABLE);
    // UARTy_DMA1_Channel enable
    DMA_Cmd(dam_chx, ENABLE);

}


void DMA_Enable(DMA_Channel_TypeDef *dma_chx)
{
    DMA_Cmd(dma_chx, ENABLE);
}

uint16_t DMA_data = 0;
u16 DMA_data_first = 0;
void UART1_IRQHandler(void)
{
    if (UART_GetITStatus(UART1, UART_IER_RXIDLE) != RESET)
    {
        UART_ClearITPendingBit(UART1, UART_IER_RXIDLE);
        DMA_Cmd(DMA1_Channel3, DISABLE);
        DMA_ClearFlag(DMA1_FLAG_TC3);
        transport_flag = true;
        DMA_data = RX_buffer_size - DMA_GetCurrDataCounter(DMA1_Channel3);
        trspx_send((uint8_t *)gSendBuff, DMA_data);
        UART_TX_Send(DMA_data, gSendBuff);
        memset(gSendBuff, 0, DMA_data);
        exDMA_SetMemoryAddress(DMA1_Channel3, (u32)gSendBuff);
        exDMA_SetTransmitLen(DMA1_Channel3, 100);
        DMA_Enable(DMA1_Channel3);
    }
}
void UART_nvic_config(void)
{

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;     //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

}

void UartSendByte(u8 dat)
{
    UART_SendData(UART1, dat);
    while (!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
}

void UART_TX_Send(uint32_t len, uint8_t *ptr)
{
    while (len--)
        UartSendByte(*ptr++);
}

//void UART1_IRQHandler(void)                   //串口1中断服务程序
//{
//    u8 Res;
//    if(UART_GetITStatus(UART1, UART_IT_RXIEN)  != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//    {
//              transport_flag=true;
//        UART_ClearITPendingBit(UART1,UART_IT_RXIEN);
//        Res =UART_ReceiveData(UART1); //读取接收到的数据
//        if((Res==0x0d)&&((UART_RX_STA&0X3FFF)>0))
//        {
//            UART_RX_STA|=0x4000;
//            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
//            UART_RX_STA++;
//        }
//        else if((UART_RX_STA&0x4000)&&((UART_RX_STA&0X3FFF)>0))//接收到了0x0d
//        {
//            if(Res==0x0a)
//            {
//                UART_RX_STA|=0x8000;
//
//            }
//            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
//            UART_RX_STA++;
//
//        }
//        else{
//            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
//            UART_RX_STA++;
//            UART_RX_STA=UART_RX_STA&0X3FFF;
//            if((UART_RX_STA)>(UART_REC_LEN-1))
//                UART_RX_STA=0;//接收数据错误,重新开始接收
//        }
//    }
//}
/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention
******************************************************************************/
int fputc(int ch, FILE *f)
{
    /* Send a character to the UART */
    UART_SendData(DEBUG_UART, (uint8_t)ch);

    /* Loop until the end of transmission */
    while (!UART_GetFlagStatus(DEBUG_UART, UART_FLAG_TXEPT));

    return ch;
}


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

