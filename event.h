typedef enum {
  EVENT_NONE = 0,
  EVENT_ACCEL_MOTION,
  EVENT_ACCEL_STATIC,
  EVENT_AUDIO_MIC_DATA_READY,
  EVENT_BLE_DATA_STREAM_START,
  EVENT_BLE_DATA_STREAM_STOP,
} event_t;

bool    eventQueueIsEmpty(void);
event_t eventQueueFront(void);
void    eventQueuePush(event_t);
event_t eventQueuePop(void);
void    eventQueueInit(void);