#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>
#include "sdk_common.h"
#include "event.h"

#define EVENT_QUEUE_DEPTH 20

static uint8_t head = 0;
static uint8_t tail = 0;
static event_t eventQueue[EVENT_QUEUE_DEPTH] = {0};

bool eventQueueIsEmpty(void)
{
  return (head == tail);
}

event_t eventQueueFront(void)
{
  return eventQueue[head];
}

event_t eventQueuePop(void)
{
  event_t event = eventQueue[head];
  head = (head+1) % EVENT_QUEUE_DEPTH;
  return event;
}

void eventQueuePush(event_t event)
{
  eventQueue[eventQueueIsEmpty() ? head : tail]  = event;
  tail = (tail+1) % EVENT_QUEUE_DEPTH;
}

void eventQueueInit(void)
{
  head = tail = 0;
}