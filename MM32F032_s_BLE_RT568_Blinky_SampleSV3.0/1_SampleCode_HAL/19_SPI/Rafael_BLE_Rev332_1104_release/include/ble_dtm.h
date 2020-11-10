#ifndef BLE_DTM_H__
#define BLE_DTM_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



/**
 *@brief Configuration parameters.
 */
#define DTM_BITRATE                     UART_BAUDRATE_BAUDRATE_Baud19200/**< Serial bitrate on the UART */

/**
 *@details The UART poll cycle in micro seconds.
 *          A baud rate of e.g. 19200 bits / second, and 8 data bits, 1 start/stop bit, no flow control,
 *          give the time to transmit a byte: 10 bits * 1/19200 = approx: 520 us.
 *          To ensure no loss of bytes, the UART should be polled every 260 us.
 */
#if DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud9600
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/9600/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud14400
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/14400/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud19200
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/19200/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud28800
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/28800/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud38400
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/38400/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud57600
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/57600/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud76800
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/768000/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud115200
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/115200/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud230400
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/230400/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud250000
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/250000/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud460800
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/460800/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud921600
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/921600/2))
#elif DTM_BITRATE == UART_BAUDRATE_BAUDRATE_Baud1M
#define UART_POLL_CYCLE ((uint32_t)(10*1e6/1e6/2))
#else
// It is possible to find values that work for other baud rates, but the formula above is not
// guaranteed to work for all values. Suitable values may have to be found by trial and error.
#error "Unsupported baud rate set."
#endif


/**
 *@brief Return codes from dtm_cmd().
 */
#define DTM_SUCCESS                     0x00                            /**< Indicate that the DTM function completed with success. */
#define DTM_ERROR_ILLEGAL_CHANNEL       0x01                            /**< Physical channel number must be in the range 0..39. */
#define DTM_ERROR_INVALID_STATE         0x02                            /**< Sequencing error: Command is not valid now. */
#define DTM_ERROR_ILLEGAL_LENGTH        0x03                            /**< Payload size must be in the range 0..37. */
#define DTM_ERROR_ILLEGAL_CONFIGURATION 0x04                            /**< Parameter out of range (legal range is function dependent). */
#define DTM_ERROR_UNINITIALIZED         0x05                            /**< DTM module has not been initialized by the application. */


/**
 * @brief BLE DTM command codes.
 */
typedef uint32_t dtm_cmd_t;

/**
 * @brief BLE DTM event type.
 */
typedef uint32_t dtm_event_t;

/**
 * @brief BLE DTM frequency type.
 */
typedef uint32_t dtm_freq_t;

/**
 * @brief BLE DTM packet types.
 */
typedef uint32_t dtm_pkt_type_t;


/**@brief Function for initializing DTM module
 *
 * @return DTM_SUCCESS on successful initialization of the DTM module.
*/
uint32_t dtm_init(void);


/**@brief Function for giving control to dtmlib for handling timer and radio events.
 *
 * @return Time counter, incremented every 260 us.
 */
uint32_t dtm_wait(void);


/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
 *
 * @param[in]   command : The packed UART command.
 * @return      result status.
 */
uint32_t dtm_cmd_put(uint16_t command);


/**@brief Function for reading the result of a DTM command
 *
 * @param[out] p_dtm_event : Pointer to buffer for 16 bit event code according to DTM standard.
 *
 * @return true: new event, false: no event since last call.
 */
uint8_t dtm_event_get(dtm_event_t * p_dtm_event);



/**@brief Function for Get the RSSI flag,
 *
 * @return 1: The last RSSI value will be provided after dtm_event_get, 0: no No RSSI value provided.
*/
uint8_t dtm_get_gui_get_rssi_flag(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_DTM_H__

/** @} */
