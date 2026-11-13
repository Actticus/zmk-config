/*
 * &mood behavior: controls the Mood RGB engine on both halves.
 */

#define DT_DRV_COMPAT zmk_behavior_mood_rgb

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <dt-bindings/zmk/mood_rgb.h>

#include "mood_rgb.h"

LOG_MODULE_DECLARE(mood_rgb, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_parameter_value_metadata no_arg_values[] = {
    {.display_name = "Toggle On/Off", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_TOG_CMD},
    {.display_name = "Turn On", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_ON_CMD},
    {.display_name = "Turn Off", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_OFF_CMD},
    {.display_name = "Next Mood", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_NEXT_CMD},
    {.display_name = "Previous Mood", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_PREV_CMD},
    {.display_name = "Brightness Up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_BRI_UP_CMD},
    {.display_name = "Brightness Down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_BRI_DN_CMD},
    {.display_name = "Speed Up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_SPD_UP_CMD},
    {.display_name = "Speed Down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_SPD_DN_CMD},
    {.display_name = "Custom Hue Up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_HUE_UP_CMD},
    {.display_name = "Custom Hue Down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_HUE_DN_CMD},
};

static const struct behavior_parameter_value_metadata set_values[] = {
    {.display_name = "Set Mood", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_SET_CMD},
};

static const struct behavior_parameter_value_metadata mood_values[] = {
    {.display_name = "Ocean", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_OCEAN},
    {.display_name = "Sunset", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_SUNSET},
    {.display_name = "Aurora", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_AURORA},
    {.display_name = "Fireplace", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_FIRE},
    {.display_name = "Starry Night", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_STARS},
    {.display_name = "Focus", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_FOCUS},
    {.display_name = "Neon", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_NEON},
    {.display_name = "Custom Color", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = MOOD_CUSTOM},
};

static const struct behavior_parameter_metadata_set sets[] = {
    {
        .param1_values = no_arg_values,
        .param1_values_len = ARRAY_SIZE(no_arg_values),
    },
    {
        .param1_values = set_values,
        .param1_values_len = ARRAY_SIZE(set_values),
        .param2_values = mood_values,
        .param2_values_len = ARRAY_SIZE(mood_values),
    },
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(sets),
    .sets = sets,
};

#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

// Runs on the central only: turn a relative command (toggle, next, +/-) into the
// resulting absolute state, which is then applied locally and sent as-is to the
// peripheral. Both halves therefore always end up in the same state.
static int convert_to_absolute(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    uint32_t packed;
    if (binding->param1 == MOOD_SYNC_STATE_CMD || binding->param1 == MOOD_SYNC_TIME_CMD) {
        return 0;
    }
    int err = mood_rgb_calc(binding->param1, binding->param2, &packed);
    if (err) {
        LOG_ERR("Unsupported mood command %d/%d", binding->param1, binding->param2);
        return err;
    }
    binding->param1 = MOOD_SYNC_STATE_CMD;
    binding->param2 = packed;
    return 0;
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case MOOD_SYNC_STATE_CMD:
        return mood_rgb_apply(binding->param2);
    case MOOD_SYNC_TIME_CMD:
        return mood_rgb_set_time(binding->param2);
    default: {
        uint32_t packed;
        int err = mood_rgb_calc(binding->param1, binding->param2, &packed);
        return err ? err : mood_rgb_apply(packed);
    }
    }
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_mood_rgb_driver_api = {
    .binding_convert_central_state_dependent_params = convert_to_absolute,
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_mood_rgb_driver_api);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
