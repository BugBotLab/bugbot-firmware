idf_component_register(
    SRCS "bugbot_module.c" "runner.c"
    INCLUDE_DIRS "."
    REQUIRES pocketpy bugbot_core
)
