idf_component_register(
    SRCS "core.c" "console.c"
    INCLUDE_DIRS "include"
    REQUIRES drivers driver esp_timer
)
