#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "sdk_common.h"
#include "nrf_assert.h"

#include "event.h"

typedef struct nd {
  event_t event;
  struct nd* next;
} node_t;

typedef struct {
  node_t* head;
  node_t* tail;
} queue_t;

static queue_t *eventQueue;

static node_t* createNode(event_t event)
{
  node_t* node;
  node = (node_t*) malloc(sizeof(node_t));
  ASSERT(node != NULL);

  node->event = event;
  node->next = NULL;

  return node;
}

bool eventQueueIsEmpty(void)
{
  return (eventQueue->head == NULL);
}

event_t eventQueueFront(void)
{
  return eventQueue->head->event;
}

event_t eventQueuePop(void)
{
  node_t* head = eventQueue->head;
  event_t event = EVENT_NONE;

  if (eventQueue->head != NULL) {
    event = eventQueue->head->event;
    eventQueue->head = eventQueue->head->next;
    free(head);
  }

  return event;
}

void eventQueuePush(event_t event)
{
  node_t* node;
  node = createNode(event);

  if (eventQueueIsEmpty()) {
    eventQueue->head = node;
    eventQueue->tail = node;
  } else {
    eventQueue->tail->next = node;
    eventQueue->tail = eventQueue->tail->next;
  }
}

void eventQueueInit(void)
{
  eventQueue = (queue_t*) malloc(sizeof(queue_t));
  eventQueue->head = NULL;
  eventQueue->tail = NULL;
}