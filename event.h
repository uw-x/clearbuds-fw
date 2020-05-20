typedef enum {
  EVENT_NONE = 0,
  EVENT_ACCEL_MOTION,
  EVENT_ACCEL_STATIC,
} event_t;

bool    eventQueueIsEmpty(void);
event_t eventQueueFront(void);
void    eventQueuePush(event_t);
event_t eventQueuePop(void);
void    eventQueueInit(void);