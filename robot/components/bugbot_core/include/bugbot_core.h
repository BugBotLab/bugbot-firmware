/* bugbot_core: the owner of everything with a deadline. Runs as FreeRTOS tasks;
 * the Python module only ever calls the shims in bugbot_shims.h, implemented in core.c. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUGBOT_FW_VERSION      "0.1.0-skeleton"
#define BUGBOT_DEADMAN_MS      500
#define BUGBOT_VBAT_CUTOFF_V   3.2f
#define BUGBOT_VBAT_WARN_V     3.4f
#define BUGBOT_I2C_HZ          100000

/* boot: drivers, then the 100 Hz control task, the 50 Hz fusion task, the camera task */
void bugbot_core_start(void);

/* the interpreter is plugged in at boot (bugbot_api provides it), so core does not depend on it */
typedef int (*bugbot_runner_t)(const char *src);   /* returns 0 on success; prints BUGBOT RUNNING/DONE/ERROR */
void bugbot_core_set_runner(bugbot_runner_t fn);
int  bugbot_core_exec(const char *src);
void bugbot_core_mark_script_start(void);   /* clock() counts from the last call */

/* the program runner: the console hands it a program, it runs it on the interpreter task */
void bugbot_core_run_program(const char *src, size_t len);
void bugbot_core_stop_program(void);
bool bugbot_core_program_running(void);

#ifdef __cplusplus
}
#endif
