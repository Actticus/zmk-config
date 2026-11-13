/*
 * Mood RGB: commands for the &mood behavior.
 *
 * Usage in a keymap:
 *   &mood MOOD_TOG               toggle lighting on/off
 *   &mood MOOD_NEXT / MOOD_PREV  cycle moods
 *   &mood MOOD_SET MOOD_SUNSET   select a specific mood
 *   &mood MOOD_BRI_UP / _DN      brightness
 *   &mood MOOD_SPD_UP / _DN      animation speed
 *   &mood MOOD_HUE_UP / _DN      color of the "custom" mood
 */

#pragma once

#define MOOD_TOG_CMD 0
#define MOOD_ON_CMD 1
#define MOOD_OFF_CMD 2
#define MOOD_NEXT_CMD 3
#define MOOD_PREV_CMD 4
#define MOOD_SET_CMD 5
#define MOOD_BRI_UP_CMD 6
#define MOOD_BRI_DN_CMD 7
#define MOOD_SPD_UP_CMD 8
#define MOOD_SPD_DN_CMD 9
#define MOOD_HUE_UP_CMD 10
#define MOOD_HUE_DN_CMD 11

// Internal: absolute state/time pushed from the central half to the peripheral.
#define MOOD_SYNC_STATE_CMD 100
#define MOOD_SYNC_TIME_CMD 101

#define MOOD_TOG MOOD_TOG_CMD 0
#define MOOD_ON MOOD_ON_CMD 0
#define MOOD_OFF MOOD_OFF_CMD 0
#define MOOD_NEXT MOOD_NEXT_CMD 0
#define MOOD_PREV MOOD_PREV_CMD 0
#define MOOD_SET MOOD_SET_CMD
#define MOOD_BRI_UP MOOD_BRI_UP_CMD 0
#define MOOD_BRI_DN MOOD_BRI_DN_CMD 0
#define MOOD_SPD_UP MOOD_SPD_UP_CMD 0
#define MOOD_SPD_DN MOOD_SPD_DN_CMD 0
#define MOOD_HUE_UP MOOD_HUE_UP_CMD 0
#define MOOD_HUE_DN MOOD_HUE_DN_CMD 0

// Moods
#define MOOD_OCEAN 0  // calm: slow waves of deep blue and teal
#define MOOD_SUNSET 1 // warm: amber -> coral -> magenta -> purple
#define MOOD_AURORA 2 // northern lights: green/aqua/violet curtains
#define MOOD_FIRE 3   // cozy fireplace flicker
#define MOOD_STARS 4  // night sky with twinkling stars
#define MOOD_FOCUS 5  // static warm white
#define MOOD_NEON 6   // rainbow party
#define MOOD_CUSTOM 7 // single adjustable color, gently breathing

#define MOOD_COUNT 8
