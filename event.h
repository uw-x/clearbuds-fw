typedef enum {
  EVENT_NONE = 0,
  EVENT_ACCEL_MOTION,
  EVENT_ACCEL_STATIC,
  EVENT_AUDIO_MIC_DATA_READY,
  EVENT_BLE_DATA_STREAM_START,
  EVENT_BLE_DATA_STREAM_STOP,
  EVENT_BLE_RADIO_START,
  EVENT_BLE_SEND_DATA_DONE,
  EVENT_TIME_SYNC_MASTER_ENABLE,
  EVENT_TIME_SYNC_SLAVE_ENABLE,
} event_t;

void    eventQueueInit(void);
bool    eventQueueEmpty(void);
event_t eventQueueFront(void);
event_t eventQueuePop(void);
void    eventQueuePush(event_t);