/******************************************************************************
 * @file    bsp_irq_gpio.c
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
#define __PORTING_MISC_C__


/* Includes -----------------------------------------------------------------*/
#include "porting_misc.h"
#include "porting_spi.h"
#include "porting_LLtimer.h"
#include "systick.h"


/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
#ifndef MAX_MBLK_SIZE
#ifdef _HCI_HW_
#define MHC_HCI_HW_PKT_TYPE     Uint8 HCI_Pckt_Typ; Uint8 HCI_Event_Code;
#else   //(#ifdef _HCI_HW_)
#define MHC_HCI_HW_PKT_TYPE
#endif  //(#ifdef _HCI_HW_)

//!!! Don't modify the following definition.//
typedef struct Len_Def_MHC_Le_Adv_Report_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Num_Reports;
    Uint8 HCI_Event_Type0;
    Uint8 HCI_Addr_Type0;
    Uint8 HCI_Addr0[6];         //6: LEN_BD_ADDR
    Uint8 HCI_Length0;
    Uint8 HCI_Data0[31+1];      //1: Uint8 HCI_RSSI0, 31: LEN_ADV_SCAN_DATA_MAX;
    //Uint8 HCI_RSSI0;
} Len_Def_MHC_Le_Adv_Report_Para;
#define MAX_MBLK_SIZE           (sizeof(Len_Def_MHC_Le_Adv_Report_Para))

//!!! Don't modify the following definition.//
typedef struct MBLK
{
    void  *Next;        /* point to next mblk */
    Uint8 Primitive;    /* primitive type */
    union
    {
        Uint8 Data[MAX_MBLK_SIZE];
    } Para;
} MBLK;

#ifndef MAX_MBLK_NO
//For user defined manually.
#define MAX_MBLK_NO_EXTRA_BY_USER_DEF   0

//Please check with RafaelMicro to see if this value matches the maximum number of connections in the LIB that LinkLayer can handle (less than or equal to either).
#define NUM_LL_ROLE                     2

//!!! Don't modify the following definition.//
//#define MAX_MBLK_RSV_LL               (NUM_LL_ROLE*5+5)         //!!! Don't modify this.//
#define MAX_MBLK_NO_                     ((NUM_LL_ROLE*5+5)+6*5+MAX_MBLK_NO_EXTRA_BY_USER_DEF)     /* maximum message block number */
#endif  //(#ifndef MAX_MBLK_NO)

MBLK MsgBlk[MAX_MBLK_NO_];
const Uint8 MAX_MBLK_NO = MAX_MBLK_NO_;
#endif  //(#ifndef MAX_MBLK_SIZE)

/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/
void MCU_GpioResetInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RF_RESET_CLOCK, ENABLE);
    /* Configure RF_RESET_PIN in output pushpull mode */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_RESET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RF_RESET_PORT, &GPIO_InitStructure);

   
    //test pin PB7
    RCC_AHBPeriphClockCmd(RF_RESET_CLOCK, ENABLE);
    /* Configure RF_RESET_PIN in output pushpull mode */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_TEST_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RF_TEST_PORT, &GPIO_InitStructure);
}

void MCU_SetGpioResetIdle(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RF_RESET_CLOCK, ENABLE);
    /* Configure RF_RESET_PIN in output pushpull mode */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_RESET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  //set input to avoid current leakage during FT/SLT test
    GPIO_Init(RF_RESET_PORT, &GPIO_InitStructure);
}

void MCU_GpioReset(void)
{
    /* Reset RF by using RF_RESET_PIN */
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_ResetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);
}
/******************************************************************************
 * @brief    Assign GPIO INT pin
 * @param       
 * @retval      
 * @attention  
******************************************************************************/
void MCU_GpioPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* RF_IRQ_CLOCK enable */
    RCC_AHBPeriphClockCmd(RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_FLOATING ;//GPIO_Mode_IPD ; //GPIO_Mode_FLOATING
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);  
}

void MCU_GpioIntEnable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
    // Configure PB8 as Input mode and enable interrupt by rising edge trigger
    MCU_GpioPinInit();      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//IRQ - pb8
	SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void MCU_GpioIntDisable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//IRQ - pb8
	SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/* GPIO Interrupt Handler */

//void EXTI4_15_IRQHandler(void)
//{
//    extern void LL_GPIO_Isr(void);
//    
//    if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
//    {
//        EXTI_ClearITPendingBit(EXTI_Line8);  
//        LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
//    }   
////    LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
//}

void EXTI4_15_IRQHandler(void)
{
    extern void LL_GPIO_Isr(void);
    extern void delay_us(u16 us) ;
    if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
//        LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
        while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 1)  
        {
            LL_GPIO_Isr(); 
            //Tiny_Delay(1);
//            LL_GPIO_Isr(); 
        }
        //EXTI_ClearITPendingBit(EXTI_Line8);
    }     
}

void spiGpioWriteReg(const unsigned char regAddr, const unsigned char regData)
{

    unsigned char SPICount;                               // Counter used to clock out the data
    unsigned char SPIData;                                // Define a data structure for the SPI data.

    SPI_CS = 1;                                           // Make sure we start with /CS high
    SPI_CK = 0;                                           // and CK low

    SPI_CS = 0;                                           // Set /CS low to start the SPI cycle 25nS
    // Although SPIData could be implemented as an "int", resulting in one
    // loop, the routines run faster when two loops are implemented with
    // SPIData implemented as two "char"s.

    delay_us(1);

    //Address 1th byte
    SPIData = regAddr & 0x7F;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                    // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }                                                     // and loop back to send the next bit
    // Repeat for the Data byte
    //Address 2nd byte
    SPIData = (regAddr & 0x80) >> 7;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                     // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }

    //Data
    SPIData = regData;                                    // Preload the data to be sent with Data
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
    {
        if (SPIData & 0x80)
            SPI_MOSI = 1;
        else
            SPI_MOSI = 0;

        delay_us(1);
        SPI_CK = 1;
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;
    }

    delay_us(1);

    SPI_CS = 1;
    SPI_MOSI = 0;
}

/******************************************************************************
 * @brief    SPI IO mapping. Must do this after Power ON
 * @param
 * @retval
 * @attention
******************************************************************************/
void SPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOB Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN | GPIO_Pin_12 | GPIO_Pin_13 |  \
                                    GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention  SPI IO mapping. Must do this after Power ON
******************************************************************************/
void RF_SpiIoMapping(void)
{
    /*
     *------------------------------------------------
     *          NSS     SCK     MISO    MOSI   IRQ  AF
     *------------------------------------------------
     *   SPI2   PB12    PB13    PB14    PB15   PB8  AF0
     *------------------------------------------------
     */
    /*(1) Here set all CLK,CS,MOSI,NISO,INT as output */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 0;
    SPI_MOSI = 0;
    SPI_MISO = 0;
    DEFAULT_INT = 0;
//    GPIO_SetBits(GPIOB, GPIO_Pin_12);  /* SPI_CS = 1 ;*/
//    GPIO_ResetBits(GPIOB, GPIO_Pin_13); /* SPI_CK=0; */
//    GPIO_ResetBits(GPIOB, GPIO_Pin_15); /* SPI_MOSI=0; */
//    GPIO_ResetBits(GPIOB, GPIO_Pin_14); /* SPI_MISO=0; */
//    GPIO_ResetBits(RF_IRQ_PORT, RF_IRQ_PIN); /* DEFAULT_INT=0; */

    //(2) Write R248, R249 GPIO select
    //Write R248=8'b10,001,000, R249=8'b0,100,011,0
    //GPIO0[2:0]=0 - CS
    //GPIO1[2:0]=1 - CLK
    //GPIO2[2:0]=2 - MOSI
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=4 - INT
    delay_us(25);
    spiGpioWriteReg(248, 0x88);
    delay_us(25);
    spiGpioWriteReg(249, 0x46);
    delay_us(25);

    /* (3) Output all pin as HIGH, last 10ms(>1ms). trigger HW take effect */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 1;
    SPI_MOSI = 1;
    SPI_MISO = 1;
    DEFAULT_INT = 1;
    SysTick_DelayMs(10);  //Delay 10ms

    //(4) Init GPIO & SPI
    MCU_GpioPinInit();        //Set GPIO interrupt pin as input
    /* Init SPI_BLE : SPI master, clk=12M, mode 0, 8-bit, MSB first  */
    BSP_SPI_BLE_Init();

    SPI_1BYT_SetTx(249, (0x46 | 0x80));  //set RF MISO, INT as output

    //manual control
    SPI_1BYT_SetTx(53, 0xC0); //To gurantee DC/DC power on when set R40=0x90

    //enable LDO
//    SPI_1BYT_SetTx(40, 0xC0);
    SPI_1BYT_SetTx(40, 0xC0);// 截取正常波形中是 0x90 而不是 0xc0
    SysTick_DelayMs(25);        //Put delay after LDO_enable, or set register may have strange behavior!

    //enable chip
    SPI_1BYT_SetTx(53, 0x80); //Enable chip
    SysTick_DelayMs(10);   //Put delay after chip_enable, or set register may have strange behavior!
}

void System_PowerDown(void)
{
//    if(BleRFMode_Get() == BLERFMODE_SLEEP)  //RF_sleep
//    {
//        /* Check if all the debug messages are finished */
//        UART_WAIT_TX_EMPTY(UART0);

//        /* Unlock protected registers before entering Power-down mode */
//        SYS_UnlockReg();

//        /* LVR must be enabled and the POR will be enabled automatically */
//        SYS_ENABLE_LVR();

//        /* Turn off internal analog POR circuit */
//        SYS_Disable_AnalogPORCircuit();

//        /* Disable Power-on Reset */
//        SYS_DISABLE_POR();

//        /* Enter to Power-down mode */
//        CLK_PowerDown();

//        /* Lock protected registers */
//        SYS_LockReg();
//    }
//    else
//    {
//        CLK_Idle();
//    }
}

#ifdef _PROFILE_OTA_
void System_Reboot(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_SET_LDROM_BOOT();
    NVIC_SystemReset();
}

uint32_t  Crc32_Checksum(uint32_t flash_addr, uint32_t size)
{
    uint32_t ChkSum;

    /* Enable CRC clock */
    CLK_EnableModuleClock(CRC_MODULE);

    SYS_UnlockReg();

    FMC_Open();

    ChkSum = FMC_GetChkSum(flash_addr, size);
    FMC_Close();
    SYS_LockReg();

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    return ChkSum;
}
#endif

void UART1_SendData(Uint8* data, Uint8 len)
{
//    UART_Write(UART1,data,len);
//    UART_WAIT_TX_EMPTY(UART1);
}
/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

