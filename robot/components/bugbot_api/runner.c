/* runner.c: runs a student's program on pocketpy. Registered with bugbot_core at boot. */
#include "bugbot_api.h"
#include "pocketpy.h"
#include <stdio.h>
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
