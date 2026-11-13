if(CONFIG_MOOD_RGB)
  zephyr_include_directories(include)
  target_sources(app PRIVATE
    src/mood_rgb.c
    src/behavior_mood_rgb.c
  )
endif()
