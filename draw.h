/*
 * draw.h
 * maruchi kim
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <nrfx.h>
#include "gpio.h"

#include "arm_const_structs.h"

#define GRAPH_WINDOW_HEIGHT              20                                     //!< Graph window height used in draw function.

void draw_fft_data(float32_t *, uint16_t, uint16_t);