/*
 * Mood RGB: LED strip lighting engine with "mood" presets.
 *
 * Replaces ZMK's built-in rgb_underglow. Split sync model: the central (left)
 * half is the single source of truth.
 *  - Every user command is converted on the central into an absolute packed
 *    state and sent to the peripheral, so a missed message can never invert a
 *    toggle.
 *  - The central also pushes its full state + animation clock on idle/wake,
 *    after (re)connecting to the peripheral and periodically.
 *  - While connected, the peripheral follows the central's idle state instead
 *    of its own, so both halves always switch on/off together. This is what
 *    the stock implementation gets wrong: each half auto-offs on its own
 *    activity and restores a stale "state before sleep" on wake.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/workqueue.h>

#include <dt-bindings/zmk/mood_rgb.h>
#include "mood_rgb.h"

#define MOOD_CENTRAL_SENDS                                                                         \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
#define MOOD_IS_PERIPHERAL                                                                         \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

#if MOOD_CENTRAL_SENDS
#include <zmk/split/central.h>
#include <zmk/events/position_state_changed.h>
#if IS_ENABLED(CONFIG_BT)
#include <zephyr/bluetooth/conn.h>
#endif
#endif

#if MOOD_IS_PERIPHERAL
#include <zmk/events/split_peripheral_status_changed.h>
#endif

LOG_MODULE_REGISTER(mood_rgb, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)
#error "A zmk,underglow chosen node must be declared"
#endif

#if !DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_mood_rgb)
#error "Mood RGB needs a zmk,behavior-mood-rgb behavior node"
#endif

#define STRIP_NODE DT_CHOSEN(zmk_underglow)
#define NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

// The behavior's device name travels over the split link (max 8 chars).
#define BEHAVIOR_NAME DEVICE_DT_NAME(DT_INST(0, zmk_behavior_mood_rgb))
BUILD_ASSERT(sizeof(BEHAVIOR_NAME) <= 9, "Mood behavior node name must be at most 8 characters");

BUILD_ASSERT(CONFIG_MOOD_RGB_BRT_MIN <= CONFIG_MOOD_RGB_BRT_MAX, "BRT_MIN must be <= BRT_MAX");

#define FADE_MS 450
#define BLEND_MS 900
#define Q16_ONE 65535U

/* ---------------------------------------------------------------- state -- */

// Packed absolute state, the unit of sync between halves.
#define PK_ON BIT(0)
#define PK_IDLE BIT(1)
#define PK_MOOD_SHIFT 2  // 4 bits
#define PK_BRT_SHIFT 6   // 7 bits
#define PK_SPD_SHIFT 13  // 3 bits
#define PK_HUE_SHIFT 16  // 8 bits

struct mood_cfg {
    bool on;
    uint8_t mood;
    uint8_t brightness; // percent
    uint8_t speed;      // index into speed_mult
    uint8_t hue;        // 0..255, used by MOOD_CUSTOM
};

// Animation clock units per millisecond for each speed level; 4 = normal.
static const uint8_t speed_mult[] = {1, 2, 4, 6, 9};
#define SPEED_LEVELS ARRAY_SIZE(speed_mult)
BUILD_ASSERT(CONFIG_MOOD_RGB_SPD_START < ARRAY_SIZE(speed_mult), "SPD_START out of range");
BUILD_ASSERT(CONFIG_MOOD_RGB_MOOD_START < MOOD_COUNT, "MOOD_START out of range");

static struct mood_cfg cfg;

static const struct device *strip;
static struct led_rgb pixels[NUM_PIXELS];
static uint32_t led_seed[NUM_PIXELS];

// Activity / link state, written from event context, read by the renderer.
static volatile bool local_active = true;
#if MOOD_IS_PERIPHERAL
static volatile bool central_connected;
static volatile bool remote_idle;
#endif

// Animation clock: t = base_t + (now - base_ms) * speed_mult. A pure function
// of uptime, so it keeps running while the LEDs are off and is trivial to
// align with the other half.
static struct k_spinlock clock_lock;
static uint32_t clock_base_t;
static int64_t clock_base_ms;

static uint32_t clock_now_locked(int64_t now) {
    return clock_base_t + (uint32_t)((now - clock_base_ms) * speed_mult[cfg.speed]);
}

#if MOOD_CENTRAL_SENDS
static uint32_t clock_now(void) {
    k_spinlock_key_t key = k_spin_lock(&clock_lock);
    uint32_t t = clock_now_locked(k_uptime_get());
    k_spin_unlock(&clock_lock, key);
    return t;
}
#endif

// Change the config, re-basing the clock so a speed change doesn't make the
// animation jump. Returns true if anything changed.
static bool set_cfg(const struct mood_cfg *c) {
    k_spinlock_key_t key = k_spin_lock(&clock_lock);
    bool changed = memcmp(c, &cfg, sizeof(*c)) != 0;
    if (c->speed != cfg.speed) {
        int64_t now = k_uptime_get();
        clock_base_t = clock_now_locked(now);
        clock_base_ms = now;
    }
    cfg = *c;
    k_spin_unlock(&clock_lock, key);
    return changed;
}

static uint32_t pack(const struct mood_cfg *c, bool idle) {
    return (c->on ? PK_ON : 0) | (idle ? PK_IDLE : 0) | ((uint32_t)c->mood << PK_MOOD_SHIFT) |
           ((uint32_t)c->brightness << PK_BRT_SHIFT) | ((uint32_t)c->speed << PK_SPD_SHIFT) |
           ((uint32_t)c->hue << PK_HUE_SHIFT);
}

static int unpack(uint32_t p, struct mood_cfg *c) {
    struct mood_cfg r = {
        .on = p & PK_ON,
        .mood = (p >> PK_MOOD_SHIFT) & 0x0F,
        .brightness = (p >> PK_BRT_SHIFT) & 0x7F,
        .speed = (p >> PK_SPD_SHIFT) & 0x07,
        .hue = (p >> PK_HUE_SHIFT) & 0xFF,
    };
    if (r.mood >= MOOD_COUNT || r.speed >= SPEED_LEVELS || r.brightness > 100) {
        return -EINVAL;
    }
    r.brightness = CLAMP(r.brightness, CONFIG_MOOD_RGB_BRT_MIN, CONFIG_MOOD_RGB_BRT_MAX);
    *c = r;
    return 0;
}

static bool display_wanted(void) {
    if (!cfg.on) {
        return false;
    }
#if IS_ENABLED(CONFIG_MOOD_RGB_AUTO_OFF_IDLE)
#if MOOD_IS_PERIPHERAL
    if (central_connected) {
        return !remote_idle;
    }
#endif
    return local_active;
#else
    return true;
#endif
}

/* -------------------------------------------------------------- color math -- */

struct rgb8 {
    uint8_t r, g, b;
};

// Perceptual 0..255 -> linear 0..65535 (gamma 2.2).
static const uint16_t gamma16[256] = {
    0,     0,     2,     4,     7,     11,    17,    24,    32,    42,    53,    65,    79,
    94,    111,   129,   148,   169,   192,   216,   242,   270,   299,   330,   362,   396,
    432,   469,   508,   549,   591,   635,   681,   729,   779,   830,   883,   938,   995,
    1053,  1113,  1175,  1239,  1305,  1373,  1443,  1514,  1587,  1663,  1740,  1819,  1900,
    1983,  2068,  2155,  2243,  2334,  2427,  2521,  2618,  2717,  2817,  2920,  3024,  3131,
    3240,  3350,  3463,  3578,  3694,  3813,  3934,  4057,  4182,  4309,  4438,  4570,  4703,
    4838,  4976,  5115,  5257,  5401,  5547,  5695,  5845,  5998,  6152,  6309,  6468,  6629,
    6792,  6957,  7124,  7294,  7466,  7640,  7816,  7994,  8175,  8358,  8543,  8730,  8919,
    9111,  9305,  9501,  9699,  9900,  10102, 10307, 10515, 10724, 10936, 11150, 11366, 11585,
    11806, 12029, 12254, 12482, 12712, 12944, 13179, 13416, 13655, 13896, 14140, 14386, 14635,
    14885, 15138, 15394, 15652, 15912, 16174, 16439, 16706, 16975, 17247, 17521, 17798, 18077,
    18358, 18642, 18928, 19216, 19507, 19800, 20095, 20393, 20694, 20996, 21301, 21609, 21919,
    22231, 22546, 22863, 23182, 23504, 23829, 24156, 24485, 24817, 25151, 25487, 25826, 26168,
    26512, 26858, 27207, 27558, 27912, 28268, 28627, 28988, 29351, 29717, 30086, 30457, 30830,
    31206, 31585, 31966, 32349, 32735, 33124, 33514, 33908, 34304, 34702, 35103, 35507, 35913,
    36321, 36732, 37146, 37562, 37981, 38402, 38825, 39252, 39680, 40112, 40546, 40982, 41421,
    41862, 42306, 42753, 43202, 43654, 44108, 44565, 45025, 45487, 45951, 46418, 46888, 47360,
    47835, 48313, 48793, 49275, 49761, 50249, 50739, 51232, 51728, 52226, 52727, 53230, 53736,
    54245, 54756, 55270, 55787, 56306, 56828, 57352, 57879, 58409, 58941, 59476, 60014, 60554,
    61097, 61642, 62190, 62741, 63295, 63851, 64410, 64971, 65535,
};

static const uint8_t sin_lut[256] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182,
    185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228,
    230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253,
    253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250,
    250, 249, 248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222,
    220, 218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179, 176, 173,
    170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115,
    112, 109, 106, 103, 100, 97,  93,  90,  88,  85,  82,  79,  76,  73,  70,  67,  65,  62,  59,
    57,  54,  52,  49,  47,  44,  42,  40,  37,  35,  33,  31,  29,  27,  25,  23,  21,  20,  18,
    17,  15,  14,  12,  11,  10,  9,   7,   6,   5,   5,   4,   3,   2,   2,   1,   1,   1,   0,
    0,   0,   0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,   10,
    11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,  37,  40,  42,  44,
    47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,  79,  82,  85,  88,  90,  93,  97,
    100, 103, 106, 109, 112, 115, 118, 121, 124,
};

static inline uint8_t sin8(uint8_t x) { return sin_lut[x]; }

static inline uint8_t scale8(uint8_t v, uint8_t s) { return ((uint16_t)v * (s + 1)) >> 8; }

static uint32_t hash32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

static struct rgb8 rgb_scale(struct rgb8 c, uint8_t s) {
    return (struct rgb8){scale8(c.r, s), scale8(c.g, s), scale8(c.b, s)};
}

static inline uint8_t lerp8(uint8_t a, uint8_t b, uint8_t f) {
    return a + (((int)b - (int)a) * f) / 255;
}

static struct rgb8 rgb_lerp(struct rgb8 a, struct rgb8 b, uint8_t f) {
    return (struct rgb8){lerp8(a.r, b.r, f), lerp8(a.g, b.g, f), lerp8(a.b, b.b, f)};
}

// Gradient through evenly spaced stops; cyclic palettes wrap back to stop 0.
static struct rgb8 palette(const struct rgb8 *stops, uint8_t n, bool cyclic, uint8_t pos) {
    uint8_t segments = cyclic ? n : n - 1;
    uint32_t scaled = (uint32_t)pos * segments;
    uint8_t idx = scaled >> 8;
    uint8_t frac = scaled & 0xFF;
    uint8_t next = cyclic ? (idx + 1) % n : MIN(idx + 1, n - 1);
    return rgb_lerp(stops[idx], stops[next], frac);
}

static struct rgb8 hsv8(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t region = h / 43;
    uint8_t rem = (h - region * 43) * 6;
    uint8_t p = (v * (255 - s)) >> 8;
    uint8_t q = (v * (255 - ((s * rem) >> 8))) >> 8;
    uint8_t t = (v * (255 - ((s * (255 - rem)) >> 8))) >> 8;

    switch (region) {
    case 0:
        return (struct rgb8){v, t, p};
    case 1:
        return (struct rgb8){q, v, p};
    case 2:
        return (struct rgb8){p, v, t};
    case 3:
        return (struct rgb8){p, q, v};
    case 4:
        return (struct rgb8){t, p, v};
    default:
        return (struct rgb8){v, p, q};
    }
}

// Smooth 1D value noise; x is 8.8 fixed point (one random value per 256 units).
static uint8_t noise8(uint32_t x, uint32_t seed) {
    uint32_t cell = x >> 8;
    uint32_t f = x & 0xFF;
    int a = hash32(cell * 0x9E3779B1U + seed) >> 24;
    int b = hash32((cell + 1) * 0x9E3779B1U + seed) >> 24;
    int s = (f * f * (768 - 2 * f)) >> 16; // smoothstep, 0..255
    return a + (((b - a) * s) >> 8);
}

/* ---------------------------------------------------------------- moods -- */

static const struct rgb8 pal_ocean[] = {
    {0, 14, 60}, {0, 60, 170}, {0, 140, 190}, {10, 200, 170}, {0, 90, 210},
};

static const struct rgb8 pal_sunset[] = {
    {255, 110, 0}, {255, 50, 20}, {230, 20, 90}, {130, 0, 150}, {255, 40, 70},
};

static const struct rgb8 pal_aurora[] = {
    {0, 255, 90}, {0, 220, 180}, {30, 90, 255}, {140, 0, 255}, {0, 200, 120},
};

static const struct rgb8 pal_fire[] = {
    {90, 6, 0}, {200, 25, 0}, {255, 70, 0}, {255, 140, 10}, {255, 200, 80},
};

static const struct rgb8 star_base = {8, 14, 70};
static const struct rgb8 star_cold = {200, 220, 255};
static const struct rgb8 star_warm = {255, 210, 150};
static const struct rgb8 focus_white = {255, 196, 130};

// Time scale (normal speed, 4 units/ms): an 8-bit phase of (t >> s) loops in
// ~1 s for s=4, ~4 s for s=6, ~8 s for s=7, ~16 s for s=8, ~33 s for s=9.
static struct rgb8 render_mood(uint8_t mood, int i, uint32_t t) {
    uint32_t seed = led_seed[i];
    uint8_t off = seed >> 24;

    switch (mood) {
    case MOOD_OCEAN: {
        struct rgb8 c = palette(pal_ocean, ARRAY_SIZE(pal_ocean), true, (t >> 9) + (off >> 1));
        return rgb_scale(c, 140 + scale8(sin8((t >> 6) + off), 115));
    }
    case MOOD_SUNSET: {
        struct rgb8 c = palette(pal_sunset, ARRAY_SIZE(pal_sunset), true, (t >> 9) + off / 3);
        return rgb_scale(c, 170 + scale8(sin8((t >> 7) + off), 85));
    }
    case MOOD_AURORA: {
        struct rgb8 c = palette(pal_aurora, ARRAY_SIZE(pal_aurora), true, (t >> 8) + (off >> 2));
        uint8_t n = noise8(t >> 4, seed);
        return rgb_scale(c, 35 + scale8(scale8(n, n), 220));
    }
    case MOOD_FIRE: {
        uint8_t heat = (noise8(t >> 2, seed) * 3 + noise8(t, seed ^ 0x5bd1e995U)) >> 2;
        struct rgb8 c = palette(pal_fire, ARRAY_SIZE(pal_fire), false, heat);
        return rgb_scale(c, 90 + scale8(heat, 165));
    }
    case MOOD_STARS: {
        uint32_t u = (t >> 5) + ((uint32_t)off << 2);
        uint32_t phase = u & 1023;
        uint32_t h = hash32((u >> 10) * 0x2545F491U + seed);
        if (phase >= 256 || (h & 1)) {
            return star_base;
        }
        uint8_t e = phase < 48 ? phase * 255 / 48 : 255 - (phase - 48) * 255 / 208;
        return rgb_lerp(star_base, (h & 6) ? star_cold : star_warm, scale8(e, e));
    }
    case MOOD_FOCUS:
        return focus_white;
    case MOOD_NEON:
        return hsv8((t >> 7) + off / 3, 255, 255);
    case MOOD_CUSTOM:
    default: {
        struct rgb8 c = hsv8(cfg.hue + (off >> 4) - 8, 240, 255);
        return rgb_scale(c, 200 + scale8(sin8((t >> 7) + off), 55));
    }
    }
}

/* -------------------------------------------------------------- renderer -- */
// Everything below runs on the ZMK low priority work queue.

static bool running;
static uint32_t fade;      // Q16, global on/off fade
static uint32_t shown_bri; // Q16 linear brightness, smoothed
static uint16_t blend = 256;
static uint8_t shown_mood, prev_mood;
static int64_t last_frame_ms;

static uint32_t bri_to_linear(uint8_t percent) {
    return (uint32_t)percent * percent * Q16_ONE / 10000;
}

static inline uint8_t to_led(uint8_t perceptual, uint32_t scale) {
    uint64_t v = ((uint64_t)gamma16[perceptual] * scale + (1U << 23)) >> 24;
    return MIN(v, 255);
}

static void blank_strip(void) {
    memset(pixels, 0, sizeof(pixels));
    led_strip_update_rgb(strip, pixels, NUM_PIXELS);
}

static void frame_tick(struct k_timer *timer);
K_TIMER_DEFINE(frame_timer, frame_tick, NULL);

static void frame_work_handler(struct k_work *work) {
    if (!running) {
        return;
    }

    int64_t now = k_uptime_get();
    uint32_t dt = CLAMP(now - last_frame_ms, 0, 250);
    last_frame_ms = now;

    k_spinlock_key_t key = k_spin_lock(&clock_lock);
    uint32_t t = clock_now_locked(now);
    k_spin_unlock(&clock_lock, key);

    uint32_t fade_step = dt * Q16_ONE / FADE_MS;
    if (display_wanted()) {
        fade = MIN(fade + fade_step, Q16_ONE);
    } else {
        fade = fade > fade_step ? fade - fade_step : 0;
        if (fade == 0) {
            blank_strip();
            k_timer_stop(&frame_timer);
            running = false;
            return;
        }
    }

    uint8_t mood = cfg.mood;
    if (mood != shown_mood) {
        prev_mood = shown_mood;
        shown_mood = mood;
        blend = 0;
    }
    if (blend < 256) {
        blend = MIN(256, blend + MAX(1, dt * 256 / BLEND_MS));
    }

    uint32_t target = bri_to_linear(cfg.brightness);
    shown_bri = (int32_t)shown_bri + ((int32_t)target - (int32_t)shown_bri) * (int32_t)dt /
                                         ((int32_t)dt + 120);
    if (abs((int32_t)target - (int32_t)shown_bri) < 16) {
        shown_bri = target;
    }

    uint32_t scale = (uint64_t)shown_bri * fade / Q16_ONE;

    for (int i = 0; i < NUM_PIXELS; i++) {
        struct rgb8 c = render_mood(shown_mood, i, t);
        if (blend < 256) {
            c = rgb_lerp(render_mood(prev_mood, i, t), c, MIN(blend, 255));
        }
        pixels[i].r = to_led(c.r, scale);
        pixels[i].g = to_led(c.g, scale);
        pixels[i].b = to_led(c.b, scale);
    }

    int err = led_strip_update_rgb(strip, pixels, NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the LED strip (%d)", err);
    }
}

K_WORK_DEFINE(frame_work, frame_work_handler);

static void frame_tick(struct k_timer *timer) {
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &frame_work);
}

// Start rendering if the lighting should be visible. Stopping is handled by
// the renderer itself after the fade-out.
static void kick_work_handler(struct k_work *work) {
    if (running || !display_wanted()) {
        return;
    }
    running = true;
    fade = 0;
    shown_mood = cfg.mood;
    blend = 256;
    shown_bri = bri_to_linear(cfg.brightness);
    last_frame_ms = k_uptime_get();
    k_timer_start(&frame_timer, K_NO_WAIT, K_MSEC(CONFIG_MOOD_RGB_FRAME_MS));
}

K_WORK_DEFINE(kick_work, kick_work_handler);

static void kick(void) { k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &kick_work); }

// Going to deep sleep: turn the LEDs off right now, they latch their last color.
static void sleep_work_handler(struct k_work *work) {
    k_timer_stop(&frame_timer);
    running = false;
    fade = 0;
    blank_strip();
}

K_WORK_DEFINE(sleep_work, sleep_work_handler);

/* -------------------------------------------------------------- settings -- */

#if IS_ENABLED(CONFIG_SETTINGS)
static void save_work_handler(struct k_work *work) {
    uint32_t packed = pack(&cfg, false);
    settings_save_one("mood_rgb/state", &packed, sizeof(packed));
}

K_WORK_DELAYABLE_DEFINE(save_work, save_work_handler);

static int mood_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                             void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "state", &next) && !next) {
        uint32_t packed;
        if (len != sizeof(packed)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &packed, sizeof(packed));
        if (rc < 0) {
            return rc;
        }
        struct mood_cfg c;
        if (unpack(packed, &c) == 0) {
            set_cfg(&c);
        }
        return 0;
    }
    return -ENOENT;
}

static int mood_settings_commit(void) {
    kick();
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mood_rgb, "mood_rgb", NULL, mood_settings_set, mood_settings_commit,
                               NULL);
#endif

static void schedule_save(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#endif
}

/* ------------------------------------------------------------ split sync -- */

#if MOOD_CENTRAL_SENDS
static void send_to_peripherals(uint32_t cmd, uint32_t arg) {
    struct zmk_behavior_binding binding = {
        .behavior_dev = BEHAVIOR_NAME,
        .param1 = cmd,
        .param2 = arg,
    };
    struct zmk_behavior_binding_event event = {
        .position = 0,
        .timestamp = k_uptime_get(),
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
    };
    for (int i = 0; i < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; i++) {
        zmk_split_central_invoke_behavior(i, &binding, event, true);
    }
}

static atomic_t fast_syncs;

static void sync_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(sync_work, sync_work_handler);

static void sync_work_handler(struct k_work *work) {
    send_to_peripherals(MOOD_SYNC_STATE_CMD, pack(&cfg, !local_active));
    send_to_peripherals(MOOD_SYNC_TIME_CMD, clock_now());

    if (atomic_get(&fast_syncs) > 0) {
        atomic_dec(&fast_syncs);
        k_work_reschedule(&sync_work, K_SECONDS(2));
    } else if (local_active) {
        k_work_reschedule(&sync_work, K_MSEC(CONFIG_MOOD_RGB_SYNC_INTERVAL_MS));
    }
}

#if IS_ENABLED(CONFIG_BT)
// The peripheral half just (re)connected: it may have missed anything, so push
// the state a few times while its GATT discovery completes.
static void mood_bt_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;
    if (err || bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    atomic_set(&fast_syncs, 4);
    k_work_reschedule(&sync_work, K_SECONDS(2));
}

BT_CONN_CB_DEFINE(mood_rgb_conn_cb) = {
    .connected = mood_bt_connected,
};
#endif
#endif // MOOD_CENTRAL_SENDS

/* ------------------------------------------------------------ public API -- */

int mood_rgb_calc(uint32_t cmd, uint32_t arg, uint32_t *packed) {
    struct mood_cfg c = cfg;

    switch (cmd) {
    case MOOD_SYNC_STATE_CMD:
        *packed = arg;
        return 0;
    case MOOD_TOG_CMD:
        c.on = !c.on;
        break;
    case MOOD_ON_CMD:
        c.on = true;
        break;
    case MOOD_OFF_CMD:
        c.on = false;
        break;
    case MOOD_NEXT_CMD:
        c.mood = (c.mood + 1) % MOOD_COUNT;
        c.on = true;
        break;
    case MOOD_PREV_CMD:
        c.mood = (c.mood + MOOD_COUNT - 1) % MOOD_COUNT;
        c.on = true;
        break;
    case MOOD_SET_CMD:
        if (arg >= MOOD_COUNT) {
            return -EINVAL;
        }
        c.mood = arg;
        c.on = true;
        break;
    case MOOD_BRI_UP_CMD:
        c.brightness = MIN(c.brightness + CONFIG_MOOD_RGB_BRT_STEP, CONFIG_MOOD_RGB_BRT_MAX);
        c.on = true;
        break;
    case MOOD_BRI_DN_CMD:
        c.brightness = MAX((int)c.brightness - CONFIG_MOOD_RGB_BRT_STEP, CONFIG_MOOD_RGB_BRT_MIN);
        c.on = true;
        break;
    case MOOD_SPD_UP_CMD:
        c.speed = MIN(c.speed + 1, SPEED_LEVELS - 1);
        c.on = true;
        break;
    case MOOD_SPD_DN_CMD:
        c.speed = c.speed > 0 ? c.speed - 1 : 0;
        c.on = true;
        break;
    case MOOD_HUE_UP_CMD:
        c.hue += CONFIG_MOOD_RGB_HUE_STEP;
        c.on = true;
        break;
    case MOOD_HUE_DN_CMD:
        c.hue -= CONFIG_MOOD_RGB_HUE_STEP;
        c.on = true;
        break;
    default:
        return -ENOTSUP;
    }

    *packed = pack(&c, !local_active);
    return 0;
}

int mood_rgb_apply(uint32_t packed) {
    if (!strip) {
        return -ENODEV;
    }

    struct mood_cfg c;
    int err = unpack(packed, &c);
    if (err) {
        return err;
    }

#if MOOD_IS_PERIPHERAL
    remote_idle = packed & PK_IDLE;
#endif

    if (set_cfg(&c)) {
        schedule_save();
    }

#if MOOD_CENTRAL_SENDS
    // Keep the periodic sync alive (it stops itself while idle).
    if (!k_work_delayable_is_pending(&sync_work)) {
        k_work_reschedule(&sync_work, K_MSEC(CONFIG_MOOD_RGB_SYNC_INTERVAL_MS));
    }
#endif

    kick();
    return 0;
}

int mood_rgb_set_time(uint32_t t) {
#if MOOD_IS_PERIPHERAL
    k_spinlock_key_t key = k_spin_lock(&clock_lock);
    clock_base_t = t;
    clock_base_ms = k_uptime_get();
    k_spin_unlock(&clock_lock, key);
#endif
    return 0;
}

/* ---------------------------------------------------------------- events -- */

static int mood_rgb_event_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        local_active = act->state == ZMK_ACTIVITY_ACTIVE;
        if (act->state == ZMK_ACTIVITY_SLEEP) {
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &sleep_work);
            struct k_work_sync sync;
            k_work_flush(&sleep_work, &sync);
            return ZMK_EV_EVENT_BUBBLE;
        }
#if MOOD_CENTRAL_SENDS
        // Tell the peripheral right away so both halves dim/wake together.
        k_work_reschedule(&sync_work, K_NO_WAIT);
#endif
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }

#if MOOD_IS_PERIPHERAL
    const struct zmk_split_peripheral_status_changed *split =
        as_zmk_split_peripheral_status_changed(eh);
    if (split) {
        central_connected = split->connected;
        remote_idle = false; // the central will send its real state shortly
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(mood_rgb, mood_rgb_event_listener);
ZMK_SUBSCRIPTION(mood_rgb, zmk_activity_state_changed);
#if MOOD_IS_PERIPHERAL
ZMK_SUBSCRIPTION(mood_rgb, zmk_split_peripheral_status_changed);
#endif

/* ------------------------------------------------------------------ init -- */

static int mood_rgb_init(void) {
    strip = DEVICE_DT_GET(STRIP_NODE);
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device is not ready");
        strip = NULL;
        return -ENODEV;
    }

    cfg = (struct mood_cfg){
        .on = IS_ENABLED(CONFIG_MOOD_RGB_ON_START),
        .mood = CONFIG_MOOD_RGB_MOOD_START,
        .brightness = CONFIG_MOOD_RGB_BRT_START,
        .speed = CONFIG_MOOD_RGB_SPD_START,
        .hue = CONFIG_MOOD_RGB_HUE_START,
    };
    clock_base_t = 0;
    clock_base_ms = k_uptime_get();

    // Per-LED randomness, different for each half so they don't mirror.
    uint32_t half = MOOD_IS_PERIPHERAL ? 0xA5A5 : 0;
    for (int i = 0; i < NUM_PIXELS; i++) {
        led_seed[i] = hash32((half << 8) + i + 1);
    }

    kick();
    return 0;
}

SYS_INIT(mood_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
