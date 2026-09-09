/* bugbot_api: the Python `bugbot` module (contract v1) and the pocketpy runner. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int bugbot_py_exec(const char *src);   /* run a program; 0 on success. Register with bugbot_core_set_runner(). */

#ifdef __cplusplus
}
#endif
