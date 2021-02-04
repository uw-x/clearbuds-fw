NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

void bleAdvertisingStart();
void bleInit(void);
void bleSendData(uint8_t * data, int length);
bool bleCanTransmit(void);
bool bleBufferHasSpace(uint16_t length);
uint32_t bleGetRingBufferBytesAvailable(void);
void blePushSequenceNumber(void);