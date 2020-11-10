/**************************************************************************//**
 * @file       ble_basicType.h
 * @brief      Provide the Definition of Basic Type and Basic Constant.
 *
 * @defgroup ble_basicType BLE Basic Type Definition
 * @{
 * @details    Basic type and constant definition. (ble_basicType.h).
 * @}
 *
 *****************************************************************************/
#ifndef _BLE_BASICTYPE_H_
#define _BLE_BASICTYPE_H_


/**
 * @defgroup ble_asicType BLE Basic Type Definition
 * @{
 * @ingroup ble_basicType
 * @details Basic type definition.
 */
typedef unsigned char           Uint8;
typedef unsigned short int      Uint16;
typedef unsigned int            Uint32;
typedef signed char             Sint8;
typedef signed short int        Sint16;
typedef signed int              Sint32;

/** @} */

/*--------------------------------------*/
/* Basic constant definition            */
/*--------------------------------------*/
/**
 * @defgroup ble_basicConstant BLE Basic Constant Definition
 * @{
 * @ingroup ble_basicType
 * @details Basic constant definition.
 */
#define SUCCESS_       0                  /**< Success.*/
#define FAIL_          (!SUCCESS_)        /**< Fail.*/

#define NO_            0                  /**< No.*/
#define YES_           (!NO_)             /**< Yes.*/

#define ENABLE_        (1UL)              /**< ENABLE.*/
#define DISABLE_       (0UL)              /**< DISABLE.*/
/** @} */


#endif // _BLE_BASICTYPE_H_


