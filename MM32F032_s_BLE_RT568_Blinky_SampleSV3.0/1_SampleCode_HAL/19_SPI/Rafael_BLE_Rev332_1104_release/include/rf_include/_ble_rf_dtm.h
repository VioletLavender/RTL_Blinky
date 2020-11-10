#ifndef _BLE_RF_DTM_H__
#define _BLE_RF_DTM_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define LE_TEST_SETUP                   0                               /**< DTM command: Set PHY or modulation, configure upper two bits of length,
                                                                             request matrix of supported features or request max values of parameters. */
#define LE_RECEIVER_TEST                1                               /**< DTM command: Start receive test. */
#define LE_TRANSMITTER_TEST             2                               /**< DTM command: Start transmission test. */
#define LE_TEST_END                     3                               /**< DTM command: End test and send packet report. */

#define LE_TEST_SETUP_RESET             0                               /**< DTM command parameter: Stop TX/RX, reset the packet length upper bits and set the PHY to 1Mbit. */
#define LE_TEST_SETUP_SET_UPPER         1                               /**< DTM command parameter: Set the upper two bits of the length field. */
#define LE_TEST_SETUP_SET_PHY           2                               /**< DTM command parameter: Select the PHY to be used for packets. */
#define LE_TEST_SETUP_SELECT_MODULATION 3                               /**< DTM command parameter: Select standard or stable modulation index. Stable modulation index is not supported. */
#define LE_TEST_SETUP_READ_SUPPORTED    4                               /**< DTM command parameter: Read the supported test case features. */
#define LE_TEST_SETUP_READ_MAX          5                               /**< DTM command parameter: Read the max supported time and length for packets. */

#define LE_PHY_1M                       1                               /**< DTM command parameter: Set PHY for future packets to use 1MBit PHY. */
#define LE_PHY_2M                       2                               /**< DTM command parameter: Set PHY for future packets to use 2MBit PHY. */
#define LE_PHY_LE_CODED_S8              3                               /**< DTM command parameter: Set PHY for future packets to use coded PHY with S=8. */
#define LE_PHY_LE_CODED_S2              4                               /**< DTM command parameter: Set PHY for future packets to use coded PHY with S=2 */


#define LE_PACKET_REPORTING_EVENT       0x8000                          /**< DTM Packet reporting event, returned by the device to the tester. */
#define LE_TEST_STATUS_EVENT_SUCCESS    0x0000                          /**< DTM Status event, indicating success. */
#define LE_TEST_STATUS_EVENT_ERROR      0x0001                          /**< DTM Status event, indicating an error. */

#define DTM_PKT_PRBS9                   0x00                            /**< Bit pattern PRBS9. */
#define DTM_PKT_0X0F                    0x01                            /**< Bit pattern 11110000 (LSB is the leftmost bit). */
#define DTM_PKT_0X55                    0x02                            /**< Bit pattern 10101010 (LSB is the leftmost bit). */
#define DTM_PKT_0XFF                    0x03                            /**< Bit pattern 11111111 (Used only for coded PHY). */

#define DTM_VENDORSPECIFIC_TX_RANDOM_CH_DISABLE               1         /*TX Random Channel disable*/
#define DTM_VENDORSPECIFIC_TX_RANDOM_CH_ENABLE                2         /*TX Random CH enable*/
#define DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_UNLIMITED        3         /*TX package number unlimited*/
#define DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_BY_USER          4         /*TX package number by user*/
#define DTM_VENDORSPECIFIC_TX_POWER_SELECT                    5         /*TX power select 0,4,8 and 10dBm*/
#define DTM_VENDORSPECIFIC_RX_GET_RSSI                        6         /*RX need print RSSI Value*/
#define DTM_VENDORSPECIFIC_GO_SLEEP_MODE                     20
#define DTM_VENDORSPECIFIC_GO_DEEP_SLEEP_MODE                21



// Note: DTM_PKT_VENDORSPECIFIC, is not a packet type
#define PACKET_TYPE_MAX                 DTM_PKT_0XFF                    /**< Highest value allowed as DTM Packet type. */


//------------------- Direct Test mode related function ------------------------//
/**@brief Function for calling when a complete command has been prepared by the Tester.
 *
 * @param[in]   cmd       One of the DTM_CMD values (bits 14:15 in the 16-bit UART format).
 * @param[in]   freq      Phys. channel no - actual frequency = (2402 + freq * 2) MHz (bits 8:13 in
 *                        the 16-bit UART format).
 * @param[in]   length    Payload length, 0..37 (bits 2:7 in the 16-bit UART format).
 * @param[in]   payload   One of the DTM_PKT values (bits 0:1 in the 16-bit UART format).
 *
 * @return      DTM_SUCCESS or one of the DTM_ERROR_ values
 */
uint32_t dtm_cmd(dtm_cmd_t cmd, dtm_freq_t freq, uint32_t length, dtm_pkt_type_t payload);

#ifdef __cplusplus
}
#endif

#endif // BLE_DTM_H__

/** @} */
