#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"

// This was written by following:
// https://github.com/bjornspockeli/custom_ble_service_example

// Generate UUIDs here:
// https://www.uuidgenerator.net/version4

// Shio Mic Stream Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define CUSTOM_SERVICE_UUID_BASE         {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define CUSTOM_SERVICE_UUID               0x1400
#define CUSTOM_VALUE_CHAR_UUID            0x1401
#define MIC_CHAR_UUID                     0x1402
#define TIME_SYNC_MASTER_CHAR_UUID        0x1403

#define BLE_CUS_DEF(_name) \
  static ble_cus_t _name; \
  NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_cus_on_ble_evt, &_name)

typedef struct ble_cus_s ble_cus_t; // Forward declaration

typedef enum
{
  BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
  BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
  BLE_CUS_EVT_TRANSFER_1KB,
  BLE_CUS_EVT_DISCONNECTED,
  BLE_CUS_EVT_CONNECTED,
} ble_cus_evt_type_t;

typedef struct
{
  ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
  uint32_t           bytes_transfered_cnt;   //!< Number of bytes sent during the transfer. */
} ble_cus_evt_t;

typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint8_t                       initial_custom_value;          /**< Initial custom value */
  ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s
{
  uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
  uint8_t                       uuid_type;
  ble_gatts_char_handles_t      mic_value_handles;              /**< Handles related to the Custom Value characteristic. */
  ble_gatts_char_handles_t      time_sync_master_handles;
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint16_t                      max_payload_len;                //!< Maximum number of bytes that can be sent in one notification. */
  uint32_t                      kbytes_sent;                    //!< Number of kilobytes sent. */
  uint32_t                      bytes_sent;                     //!< Number of bytes sent. */
  uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
};

uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);
uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t custom_value);
void ble_cus_on_gatt_evt(ble_cus_t * p_cus, nrf_ble_gatt_evt_t const * p_gatt_evt);
bool ble_cus_transmit(ble_cus_t * p_cus, uint8_t * data, uint16_t length);