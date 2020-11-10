#ifndef __PORTING_FLASH_H__
#define __PORTING_FLASH_H__

//Flash address of bonding information and data
#define BONDING_INFORMATION_ADDRESS                    0x0000F800       /**< Address of bonding information block.*/
#define BONDING_DATA_ADDRESS                           0x0000FC00       /**< Address of bonding data block.*/

#ifdef _PROFILE_OTA_
#define FOTA_UPDATE_BUFFER_FW_ADDRESS                  0x00010E00      /**<Bank size 0xF000 .*/
#define FOTA_UPDATE_BANK_INFO_ADDRESS                  0x0001FE00      /**< Address of FOTA update information*/
#endif  //_PROFILE_OTA_

//Do not modify below definition
#define SIZE_OF_FLASH_PAGE                             512              /**< Size of Flash Page Type.*/
#define SIZE_OF_BONDING_INFORMATION                    1024             /**< Total size of BLE bonding information array.*/
#define SIZE_OF_BONDING_DATA                           1024             /**< Total size of BLE bonding Data array.*/

#ifdef _PROFILE_OTA_
#define SIZE_OF_FOTA_INFO                              512             /**< 1 page for saving FOTA upgrade information.*/
#define SIZE_OF_FOTA_BANK                              0xF000          /**< Total size of FW flash bank.*/
#endif  //_PROFILE_OTA_

#define SIZE_OF_INFO_BLK                               64               /**< Size of BLE bonding information block.*/
#define SIZE_OF_DAT_GRP_BLK                            128              /**< Size of BLE bonding Data block.*/
#define NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK         (SIZE_OF_BONDING_INFORMATION/SIZE_OF_FLASH_PAGE) /**< Total number of flash size of bound information block.*/
#define NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK      (SIZE_OF_BONDING_DATA/SIZE_OF_FLASH_PAGE)        /**< Total number of flash size of bound data block.*/
#define NUM_OF_INFO_BLK_ONE_PAGE                       (SIZE_OF_FLASH_PAGE/SIZE_OF_INFO_BLK)            /**< Amount of information bound using flash page.*/
#define NUM_OF_DAT_GRP_BLK_ONE_PAGE                    (SIZE_OF_FLASH_PAGE/SIZE_OF_DAT_GRP_BLK)         /**< Amount of data bound using flash page.*/
#define TAB_FLASH_BONDING_INFO_BLK                     SIZE_OF_INFO_BLK                                 /**< Tab Size of BLE bonding information block.*/
#define TAB_FLASH_BONDING_DATA_GRP_BLK                 SIZE_OF_DAT_GRP_BLK                              /**< Tab size of BLE bonding Data block.*/

#endif  //__PORTING_FLASH_H__


