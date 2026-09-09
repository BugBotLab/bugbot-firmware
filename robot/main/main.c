/* BugBot robot firmware entry point (ESP32-P4). */
#include "bugbot_core.h"
#include "bugbot_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

extern void bugbot_console_task(void *arg);

void app_main(void) {
    printf("BUGBOT %s CONTRACT 1\n", BUGBOT_FW_VERSION);
    bugbot_core_set_runner(bugbot_py_exec);
    bugbot_core_start();
    xTaskCreatePinnedToCore(bugbot_console_task, "console", 8192, NULL, 4, NULL, 0);
    /* TODO: if /spiffs/main.py exists and the boot button is not held, run it */
}
