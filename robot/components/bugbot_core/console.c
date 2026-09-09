/* console.c: the USB CDC console the IDE talks to (docs/PROTOCOL_USB.md), and the pocketpy runner.
 *
 * Lines:  BUGBOT HELLO            -> BUGBOT <fw> CONTRACT <n>
 *         BUGBOT RUN <bytes>\n<program>   -> BUGBOT RUNNING ... BUGBOT DONE | BUGBOT ERROR <msg>
 *         BUGBOT STOP             -> BUGBOT STOPPED
 *         BUGBOT SAVE <bytes>\n<program>  -> stored as main.py, run at power-up
 * Anything the program prints goes back as plain lines.
 *
 * SKELETON: the CDC transport is ESP-IDF's usb_serial_jtag / tinyusb CDC on the P4; the read loop
 * here is the framing only.
 */
#include "bugbot_core.h"
#include "pocketpy.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern void bugbot_module_init(void);

static void py_print(const char *s) { fputs(s, stdout); fflush(stdout); }

int bugbot_py_exec(const char *src) {
    static bool inited;
    if (!inited) { py_initialize(); py_callbacks()->print = py_print; bugbot_module_init(); inited = true; }
    printf("BUGBOT RUNNING\n");
    bool ok = py_exec(src, "<program>", EXEC_MODE, NULL);
    if (!ok) {
        char *msg = py_formatexc();
        printf("BUGBOT ERROR %s\n", msg ? msg : "unknown");
        free(msg);
        py_clearexc(NULL);
        return 1;
    }
    printf("BUGBOT DONE\n");
    return 0;
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
