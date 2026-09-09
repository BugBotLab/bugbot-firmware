/* console.c: the USB CDC console the IDE talks to (docs/PROTOCOL_USB.md).
 *
 * Lines:  BUGBOT HELLO            -> BUGBOT <fw> CONTRACT <n>
 *         BUGBOT RUN <bytes>\n<program>   -> BUGBOT RUNNING ... BUGBOT DONE | BUGBOT ERROR <msg>
 *         BUGBOT STOP             -> BUGBOT STOPPED
 *         BUGBOT SAVE <bytes>\n<program>  -> stored as main.py, run at power-up
 * Anything the program prints goes back as plain lines.
 *
 * The interpreter is not a dependency of this component: bugbot_api registers its runner
 * with bugbot_core_set_runner() at boot (main.c), so core never links against pocketpy.
 *
 * SKELETON: the CDC transport is TinyUSB CDC on the P4; the read loop here is the framing only.
 */
#include "bugbot_core.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static bugbot_runner_t runner;

void bugbot_core_set_runner(bugbot_runner_t fn) { runner = fn; }

int bugbot_core_exec(const char *src) {
    if (!runner) { printf("BUGBOT ERROR no interpreter\n"); return 1; }
    bugbot_core_mark_script_start();     /* clock() counts from here */
    return runner(src);
}

static bool read_line(char *buf, size_t n) {
    size_t i = 0; int c;
    while ((c = getchar()) != EOF) {
        if (c == '\n') { buf[i] = 0; return true; }
        if (i + 1 < n) buf[i++] = (char)c;
    }
    return false;
}

static char *read_exact(size_t len) {
    char *p = malloc(len + 1); size_t got = 0; int c;
    while (got < len && (c = getchar()) != EOF) p[got++] = (char)c;
    p[got] = 0; return p;
}

void bugbot_console_task(void *arg) {
    (void)arg;
    char line[64];
    for (;;) {
        if (!read_line(line, sizeof line)) continue;
        if (strncmp(line, "BUGBOT ", 7) != 0) continue;
        const char *cmd = line + 7;
        if (strcmp(cmd, "HELLO") == 0) {
            printf("BUGBOT %s CONTRACT %d\n", BUGBOT_FW_VERSION, 1);
        } else if (strncmp(cmd, "RUN ", 4) == 0) {
            size_t len = (size_t)strtoul(cmd + 4, NULL, 10);
            char *src = read_exact(len);
            bugbot_core_run_program(src, len);
            free(src);
        } else if (strcmp(cmd, "STOP") == 0) {
            bugbot_core_stop_program();
            printf("BUGBOT STOPPED\n");
        } else if (strncmp(cmd, "SAVE ", 5) == 0) {
            size_t len = (size_t)strtoul(cmd + 5, NULL, 10);
            char *src = read_exact(len);
            /* TODO: write to /spiffs/main.py */
            free(src);
            printf("BUGBOT SAVED\n");
        }
    }
}
