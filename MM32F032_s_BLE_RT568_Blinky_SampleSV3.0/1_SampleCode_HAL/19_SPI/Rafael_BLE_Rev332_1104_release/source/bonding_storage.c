/*******************************************************************
 *
 * File Name  : bonding_storage.C
 * Description:
 *
 *******************************************************************
 *
 *      Copyright (c) 2020, All Right Reserved
 *      Rafael Microelectronics Co. Ltd.
 *      Taiwan, R.O.C.
 *
 *******************************************************************/
#pragma push
//#pragma Otime
#pragma Ospace
//#pragma OPTIMIZE (9, SPEED)

/*******************************************************************
 *      Include List
 *******************************************************************/
#include "knl_pblc.h"
#include "_ble_host.h"
#include "porting_flash.h"

/*******************************************************************
 *      Global Variable Defines
 *******************************************************************/
#ifdef _SMP_ON_
    #ifdef _BOND_ON_
        extern const Uint8 INFO_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK][SIZE_OF_FLASH_PAGE];
        extern const Uint8 DATA_FLASHBOND[NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK][SIZE_OF_FLASH_PAGE];
    #endif
#endif
/*******************************************************************
 *      Function Prototype Decelaration
 *******************************************************************/
#ifdef _SMP_ON_
    #ifdef _BOND_ON_
        void ersDATA_FLASHBOND(Uint8 offset);
        void ersINFO_FLASHBOND(Uint8 offset);
        void mark_PID_INFO_FLASHBOND(Uint8 offset);
        void mark_PID_DATA_FLASHBOND(Uint8 offset);

        extern void flash_program_internal_use(uint32_t u32Addr, uint32_t u32Data);
        extern int32_t flash_erase_internal_use(uint32_t u32Addr);

        uint8_t *cmd_FlashBond(uint8_t opcode, uint8_t *para_data);
        void fil_INFO_FLASHBONDwEXST_PBID(void);
        void fil_DATA_FLASHBONDwEXST_PBID(void);
    #endif
#endif
/*******************************************************************
 *      Const Code Decelaration
 *******************************************************************/

/*******************************************************************
 *      Program Code
 *******************************************************************/
#ifdef _SMP_ON_
#ifdef _BOND_ON_
/* Function : Erase BLE Bound data space */
void ersDATA_FLASHBOND(Uint8 offset)
{
    flash_erase_internal_use((uint32_t)&DATA_FLASHBOND + ((uint16_t)offset << 9));
}

/* Function : Erase BLE Bound information space */
void ersINFO_FLASHBOND(Uint8 offset)
{
    flash_erase_internal_use((uint32_t)&INFO_FLASHBOND + ((uint16_t)offset << 9));
}

/* Function : Mark PID to BLE Bound information space */
void mark_PID_INFO_FLASHBOND(Uint8 offset)
{
    uint16_t i, j;
    uint32_t u32Pattern;

    j = offset * NUM_OF_INFO_BLK_ONE_PAGE;

    for (i = 0; i < SIZE_OF_FLASH_PAGE; i = i + SIZE_OF_INFO_BLK)
    {
        u32Pattern = 0xFFFF0000;
        u32Pattern |= (j);
        u32Pattern |= (j << 8);
        flash_program_internal_use(((uint32_t)&INFO_FLASHBOND + ((uint16_t)offset << 9) + i), u32Pattern);
        j++;
    }
}

/* Function : Mark PID to BLE Bound data space */
void mark_PID_DATA_FLASHBOND(Uint8 offset)
{
    uint16_t i, j;
    uint32_t u32Pattern;

    j = offset * NUM_OF_DAT_GRP_BLK_ONE_PAGE;

    for (i = 0; i < SIZE_OF_FLASH_PAGE; i = i + SIZE_OF_DAT_GRP_BLK)
    {
        u32Pattern = 0xFFFF0000;
        u32Pattern |= (j);
        u32Pattern |= (j << 8);
        flash_program_internal_use(((uint32_t)&DATA_FLASHBOND + ((uint16_t)offset << 9) + i), u32Pattern);
        j++;
    }
}

uint8_t *cmd_FlashBond(uint8_t opcode, uint8_t *para_data)
{
    Uint8 j;
    Uint16 x, i, k;
    Uint32 u32Pattern;
    Uint16 offsetaddr;
    Uint16 addr;

    switch (opcode)
    {
    case CMD_FB_INIT_INFO_FLASHBOND:
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; j++) // j = 0,1
        {
            ersINFO_FLASHBOND(j);
            mark_PID_INFO_FLASHBOND(j);
        }
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_INIT_DATA_FLASHBOND:
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK; j++) // j = 0,1
        {
            ersDATA_FLASHBOND(j);
            mark_PID_DATA_FLASHBOND(j);
        }
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_CHK_IF_FLASH_INITED:
        *para_data = FLH_BND_ERR_CODE_FLASH_NOT_INI;
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; j++) // j = 0,1
        {
            for (x = 0; x < SIZE_OF_FLASH_PAGE; x = (x + SIZE_OF_INFO_BLK)) // x = 0,64,128,192,256,320,384,448
            {
                k = j;
                if (INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PID + x] == (0x00 + (k << 3) + (x >> 6)))
                {
                    if (INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PID + x + 1] == INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PID + x])
                    {
                        if (INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PID + x] != 0xFF)
                        {
                            for (i = (TAB_INFO_FLASHBOND_PBID + x); i < (SIZE_OF_INFO_BLK + x); i++)
                            {
                                if (INFO_FLASHBOND[j][i] != 0xFF)
                                {
                                    break;
                                }
                            }
                        }
                        else
                        {
                            return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                        }
                    }
                    else
                    {
                        return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                    }
                }
                else
                {
                    return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                }
                k = i;
                while (k < SIZE_OF_INFO_BLK + x)
                {
                    for (i = (2 + 2 + SMP_PARA_BOND_SIZE + x); i < (SIZE_OF_INFO_BLK + x); i++)
                    {
                        if (INFO_FLASHBOND[j][i] != 0xFF)
                        {
                            break;
                        }
                    }
                    if (i != SIZE_OF_INFO_BLK + x)
                    {
                        return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                    }
                    k = (k + SIZE_OF_INFO_BLK + x);
                }
                if (i != SIZE_OF_INFO_BLK + x)
                {
                    return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                }
            }
        }
        if (j == NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK)
        {
            *para_data = FLH_BND_ERR_CODE_NO_ERR;
        }
        break;

    case CMD_FB_GET_EXST_PBID:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        k = 0;
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; j++) // j = 0,1
        {
            for (x = 0; x < NUM_OF_INFO_BLK_ONE_PAGE; x++) // x = 0,1,2,3,4,5,6,7
            {
                if ((INFO_FLASHBOND[(NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (NUM_OF_INFO_BLK_ONE_PAGE - 1 - x)))] != 0xFF))
                {
                    k = INFO_FLASHBOND[((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j)][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (NUM_OF_INFO_BLK_ONE_PAGE - 1 - x)))];
                    i = INFO_FLASHBOND[((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j)][(TAB_INFO_FLASHBOND_PID + (TAB_FLASH_BONDING_INFO_BLK * (NUM_OF_INFO_BLK_ONE_PAGE - 1 - x)))];
                    break;  //FLH_BND_ERR_CODE_NO_ERR
                }
            }
            if (x != NUM_OF_INFO_BLK_ONE_PAGE)
            {
                if ((k & 0x0F) == 0) //slove the issue: cant not erase at connect mode
                {
                    if ((INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PBID]) != 0xFF)
                    {
                        if ((INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PBID]) == (k + 1))
                        {
                            k = INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PBID];
                            i = INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PID];
                        }
                    }
                }
                break;//FLH_BND_ERR_CODE_NO_ERR
            }
        }
        if (k == 0)
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXST_PBID;
        }
        else
        {
            *(para_data + TAB_PARA_DATA_PID) = i;
            *(para_data + TAB_PARA_DATA_PBID) = k;
        }
        break;

    case CMD_FB_GET_NXT_PBID:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        k = 0;
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; j++) //j = 0,1
        {
            for (x = 0; x < NUM_OF_INFO_BLK_ONE_PAGE; x++) // x = 0,1,2,3,4,5,6,7
            {
                if (INFO_FLASHBOND[(NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (7 - x)))] != 0xFF)
                {
                    *(para_data + TAB_PARA_DATA_PBID) = INFO_FLASHBOND[((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j)][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (7 - x)))] + 1;
                    *(para_data + TAB_PARA_DATA_PID) = INFO_FLASHBOND[((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j)][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (7 - x)))] + 1;
                    if (x == 0)
                    {
                        k = TAB_INFO_FLASHBOND_PBID;
                    }
                    else
                    {
                        k = TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * (NUM_OF_INFO_BLK_ONE_PAGE - x));
                        j = (NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j;
                    }
                    break;
                }
            }
            if (k != 0)
            {
                //erase page
                break;
            }
        }
        if (k == 0)
        {
            *(para_data + TAB_PARA_DATA_PBID) = 0x01;
            k = TAB_INFO_FLASHBOND_PBID;
            j = 0;
        }
        if (*(para_data + TAB_PARA_DATA_PBID) == 0)
        {
            *(para_data + TAB_PARA_DATA_PBID) = 0x01;
        }
        if (INFO_FLASHBOND[j][k] != 0xFF)
        {
            *para_data = FLH_BND_ERR_CODE_NO_FREE_PBID;
        }
        else
        {
            u32Pattern = 0;
            u32Pattern |= *(para_data + TAB_PARA_DATA_PBID);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PBID);
            u32Pattern = (u32Pattern << 8) | INFO_FLASHBOND[j][k - TAB_INFO_FLASHBOND_PBID];
            u32Pattern = (u32Pattern << 8) | INFO_FLASHBOND[j][k - TAB_INFO_FLASHBOND_PBID];
            flash_program_internal_use(((uint32_t)&INFO_FLASHBOND + ((uint16_t)j << 9) + (k - TAB_INFO_FLASHBOND_PBID)), u32Pattern);
        }
        break;

    case CMD_FB_GET_INFO_FLASHBOND_PARA_BOND:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (i = 0; i < SIZE_OF_FLASH_PAGE; i = i + SIZE_OF_INFO_BLK) // i = 0,64,128,192,256,320,384,448
        {
            if (INFO_FLASHBOND[(*(para_data + TAB_PARA_DATA_PID) >> 3)][(TAB_INFO_FLASHBOND_PBID + i)] == *(para_data + TAB_PARA_DATA_PBID))
            {
                for (j = 0; j < SMP_PARA_BOND_SIZE; j++)
                {
                    *(para_data + TAB_PARA_DATA_INI_ADDR + j) = INFO_FLASHBOND[(*(para_data + TAB_PARA_DATA_PID) >> 3)][(TAB_INFO_FLASHBOND_INI_ADDR + i + j)];
                }
                break;
            }
        }
        if (i == SIZE_OF_FLASH_PAGE)
        {
            *para_data = FLH_BND_ERR_CODE_PID_DNT_MATCH;
        }
        break;

    case CMD_FB_GET_DATA_FLASHBOND_EXST_PBID_DBLK_START:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (i = 0; i < NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK; i++) // i = 0,1
        {
            for (x = 0; x < NUM_OF_DAT_GRP_BLK_ONE_PAGE; x++) // x = 0,1,2,3
            {
                if (DATA_FLASHBOND[(NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - i][TAB_DATA_FLASHBOND_PBID + (TAB_FLASH_BONDING_DATA_GRP_BLK * ((NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1) - x))] == *(para_data + TAB_PARA_DATA_PBID))
                {
                    break;
                }
            }
            if (x != NUM_OF_DAT_GRP_BLK_ONE_PAGE)
            {
                break;
            }
        }
        if (i == NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK)
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXST_PBID;
        }
        else
        {
            i = (NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - i;
            *(para_data + TAB_PARA_DATA_PID) = (i * NUM_OF_DAT_GRP_BLK_ONE_PAGE) + (NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1 - x);
            j = TAB_DATA_FLASHBOND_DAT_START + 1;
            while (j < SIZE_OF_DAT_GRP_BLK)
            {
                if (DATA_FLASHBOND[i][j + (TAB_FLASH_BONDING_DATA_GRP_BLK * ((NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1) - x))] != 0xFF)
                {
                    k = DATA_FLASHBOND[i][j + (TAB_FLASH_BONDING_DATA_GRP_BLK * ((NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1) - x))];
                    j = j + k + 2;
                    if (((k + 2) & 0x03) != 0)
                    {
                        j = j + (4 - ((k + 2) & 0x03));
                    }
                }
                else
                {
                    j--;
                    break;
                }
            }
            if (j >= (SIZE_OF_DAT_GRP_BLK - 4))
            {
                *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
            }
            *(para_data + TAB_PARA_DATA_DAT_START) = j;
        }
        break;

    case CMD_FB_GET_DATA_FLASHBOND_NXT_PBID_DBLK_START:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (i = 0; i < NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK; i++) // i = 0,1
        {
            for (x = 0; x < NUM_OF_DAT_GRP_BLK_ONE_PAGE; x++) // x = 0,1,2,3
            {
                if (DATA_FLASHBOND[(NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - i][TAB_DATA_FLASHBOND_PBID + (TAB_FLASH_BONDING_DATA_GRP_BLK * ((NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1) - x))] != 0xFF)
                {
                    break;
                }
            }
            if (x != NUM_OF_DAT_GRP_BLK_ONE_PAGE)
            {
                break;
            }
        }
        if (i == NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK)
        {
            i = 0;
            x = 0;
        }
        else
        {
            if (x == 0)
            {
                i++;
                i = i >> 1;
            }
            else
            {
                i = (NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - i;
                x = (NUM_OF_DAT_GRP_BLK_ONE_PAGE - x);
            }
        }
        *(para_data + TAB_PARA_DATA_PID) = (i * NUM_OF_DAT_GRP_BLK_ONE_PAGE) + x;

        u32Pattern = 0;
        u32Pattern |= *(para_data + TAB_PARA_DATA_PBID);
        u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PBID);
        u32Pattern = (u32Pattern << 8) | DATA_FLASHBOND[i][TAB_FLASH_BONDING_DATA_GRP_BLK * x];
        u32Pattern = (u32Pattern << 8) | DATA_FLASHBOND[i][TAB_FLASH_BONDING_DATA_GRP_BLK * x];
        flash_program_internal_use(((uint32_t)&DATA_FLASHBOND + ((uint16_t)i << 9) + TAB_FLASH_BONDING_DATA_GRP_BLK * x), u32Pattern);
        *(para_data + TAB_PARA_DATA_DAT_START) = TAB_DATA_FLASHBOND_DAT_START;
        break;

    case CMD_FB_PSH_DATA_FLASHBOND_EXST_PBID_DBLK:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        if (*(para_data + TAB_PARA_DATA_DAT_START) >= (SIZE_OF_DAT_GRP_BLK - 4))
        {
            *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
        }
        else
        {
            for (x = 0; x < NUM_OF_DAT_GRP_BLK_ONE_PAGE; x++) // x = 0,1,2,3
            {
                if (DATA_FLASHBOND[(*(para_data + TAB_PARA_DATA_PID) >> 2)][TAB_DATA_FLASHBOND_PBID + (TAB_FLASH_BONDING_DATA_GRP_BLK * ((NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1) - x))] != 0xFF)
                {
                    break;
                }
            }
            x = NUM_OF_DAT_GRP_BLK_ONE_PAGE - 1 - x;
            addr = (*(para_data + TAB_PARA_DATA_PID) >> 2);
            offsetaddr = *(para_data + TAB_PARA_DATA_DAT_START) + (TAB_FLASH_BONDING_DATA_GRP_BLK * x);

            if (((*(para_data + TAB_PARA_DATA_DAT_SIZE) + 2) & 0x03) != 0)
            {
                x = (((*(para_data + TAB_PARA_DATA_DAT_SIZE)) + 2) >> 2) + 1;
            }
            else
            {
                x = (*(para_data + TAB_PARA_DATA_DAT_SIZE) + 2) >> 2;
            }
            i = 0;
            for (j = 0; j < x; j++)
            {
                Knl_MemCpy((Uint8 *)(&u32Pattern), (para_data + (TAB_PARA_DATA_DAT_PTR + i)), 4);
                flash_program_internal_use(((uint32_t)&DATA_FLASHBOND + ((uint16_t)addr << 9) + offsetaddr), u32Pattern);
                i = i + 4;
                offsetaddr = offsetaddr + 4;
            }
        }
        break;

    case CMD_FB_CHK_IF_FLASHBOND_NEED_TO_ERASE_PAGE:
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; j++)
        {
            for (x = 0; x < NUM_OF_INFO_BLK_ONE_PAGE; x++)
            {
                if (INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * x)] == 0xFF)
                {
                    break;
                }
            }
            if (x != NUM_OF_INFO_BLK_ONE_PAGE)
            {
                for (x = 0; x < NUM_OF_INFO_BLK_ONE_PAGE; x++)
                {
                    if (INFO_FLASHBOND[j][TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * x)] != 0xFF)
                    {
                        break;
                    }
                }
                if (x != NUM_OF_INFO_BLK_ONE_PAGE)
                {
                    break;
                }
            }
        }
        if (j != NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK)
        {
            ersINFO_FLASHBOND((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j);
            mark_PID_INFO_FLASHBOND((NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK - 1) - j);
        }
        for (j = 0; j < NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK; j++) // j = 0,1
        {
            for (x = 0; x < NUM_OF_DAT_GRP_BLK_ONE_PAGE; x++) // x = 0,1,2,3
            {
                if ((DATA_FLASHBOND[j][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_DATA_GRP_BLK * x))] == 0xFF))
                {
                    break;
                }
            }
            if (x != NUM_OF_DAT_GRP_BLK_ONE_PAGE)
            {
                for (x = 0; x < NUM_OF_DAT_GRP_BLK_ONE_PAGE; x++)
                {
                    if ((DATA_FLASHBOND[j][(TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_DATA_GRP_BLK * x))] != 0xFF))
                    {
                        break;
                    }
                }
                if (x != NUM_OF_DAT_GRP_BLK_ONE_PAGE)
                {
                    break;
                }
            }
        }
        if (j != NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK)
        {
            ersDATA_FLASHBOND((NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - j);
            mark_PID_DATA_FLASHBOND((NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK - 1) - j);
        }
        break;

    default:
        break;
    }
    return para_data;
}

/*********************************************************************
** fil_INFO_FLASHBONDwEXST_PBID
*********************************************************************/
void fil_INFO_FLASHBONDwEXST_PBID(void)
{
    Uint8 i, program_cnt;
    Uint8 *tmp;
    Uint16 j, offsetaddr, addr;
    Uint32 u32Pattern, u32FF;

    tmp = cmd_FlashBond(CMD_FB_GET_EXST_PBID, (uint8_t *)smp_Para_Bond_tmp);
    if (*tmp == FLH_BND_ERR_CODE_NO_ERR)
    {
        tmp = cmd_FlashBond(CMD_FB_GET_INFO_FLASHBOND_PARA_BOND, (uint8_t *)smp_Para_Bond_tmp);
        if (*tmp == FLH_BND_ERR_CODE_NO_ERR)
        {
            if (Knl_MemComp((Uint8 *)&smp_Para_Bond, (tmp + TAB_PARA_DATA_INI_ADDR), SMP_PARA_BOND_SIZE))
            {
                for (i = 0; i < NUM_OF_INFO_BLK_ONE_PAGE; i++)
                {
                    if (INFO_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 3][TAB_INFO_FLASHBOND_PBID + (TAB_FLASH_BONDING_INFO_BLK * i)] == *(tmp + TAB_PARA_DATA_PBID))
                    {
                        j = TAB_INFO_FLASHBOND_INI_ADDR + (TAB_FLASH_BONDING_INFO_BLK * i);
                        break;
                    }
                }

                if ((SMP_PARA_BOND_SIZE & 0x03) != 0)   // divider 4; flash program = 4bytes
                {
                    program_cnt = (SMP_PARA_BOND_SIZE >> 2) + 1;
                }
                else
                {
                    program_cnt = (SMP_PARA_BOND_SIZE >> 2);
                }

                addr = (*(tmp + TAB_PARA_DATA_PID)) >> 3;
                offsetaddr = j;

                i = 0;
                for (j = 0; j < program_cnt; j++)
                {
                    Knl_MemCpy((Uint8 *)(&u32Pattern), (Uint8 *)&smp_Para_Bond[i], 4);
                    if (j == (program_cnt - 1))
                    {
                        if ((SMP_PARA_BOND_SIZE & 0x03) != 0)
                        {
                            u32FF = 0xFFFFFFFF;  //flash erase value
                            Knl_MemCpyInv((Uint8 *)(&u32Pattern) + (SMP_PARA_BOND_SIZE & 0x03), (Uint8 *)&u32FF, (4 - (SMP_PARA_BOND_SIZE & 0x03)));
                        }
                    }
                    flash_program_internal_use(((uint32_t)&INFO_FLASHBOND + ((uint16_t)addr << 9) + offsetaddr), u32Pattern);
                    i = i + 4;
                    offsetaddr = offsetaddr + 4;
                }
            }
        }
    }
}

/*********************************************************************
** fil_DATA_FLASHBONDwEXST_PBID
*********************************************************************/
void fil_DATA_FLASHBONDwEXST_PBID(void)
{
    uint8_t i, j, k, *tmp;
    Uint16 x;

    tmp = cmd_FlashBond(CMD_FB_GET_EXST_PBID, (uint8_t *)smp_Para_Bond_tmp);
    if (*tmp == FLH_BND_ERR_CODE_NO_ERR)
    {
        tmp = cmd_FlashBond(CMD_FB_GET_DATA_FLASHBOND_EXST_PBID_DBLK_START, (uint8_t *)smp_Para_Bond_tmp);
        if (*tmp != FLH_BND_ERR_CODE_NO_EXST_PBID)
        {
            j = TAB_DATA_FLASHBOND_DAT_START + 1;
            x = (((*(tmp + TAB_PARA_DATA_PID)) & 0x03) << 7);
            while (j < * (tmp + TAB_PARA_DATA_DAT_START))
            {
                if (DATA_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 2][j + x] != 0xFF)
                {
                    for (i = 0; i < (DATA_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 2][j + x]); i++)
                    {
                        *((uint8_t *) * (ATTRIBUTE_SERVER_BOND_PARAM + DATA_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 2][j + x - 1]) + i) = DATA_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 2][j + x + i + 1];
                    }
                    k = DATA_FLASHBOND[(*(tmp + TAB_PARA_DATA_PID)) >> 2][j + x];
                    j = j + k + 2;
                    if (((k + 2) & 0x03) != 0)
                    {
                        j = j + (4 - ((k + 2) & 0x03));
                    }
                }
            }
        }
    }
}
#endif
#endif
#pragma pop





