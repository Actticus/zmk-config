/*
 * Mood RGB engine: internal API shared with the &mood behavior.
 */

#pragma once

#include <stdint.h>

// Apply a user command to the current state and return the resulting absolute
// state packed into 32 bits (see mood_rgb.c). Does not change anything.
int mood_rgb_calc(uint32_t cmd, uint32_t arg, uint32_t *packed);

// Apply an absolute packed state (from a local key press or from the central half).
int mood_rgb_apply(uint32_t packed);

// Align the animation clock with the central half.
int mood_rgb_set_time(uint32_t t);
