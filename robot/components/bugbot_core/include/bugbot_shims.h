/* bugbot_shims.h: the C surface the Python `bugbot` module calls (contract v1).
 * Implemented by bugbot_core (control loop, fusion, safety) and the drivers.
 * Units follow the contract: cm, cm/s, degrees clockwise-positive, percent. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUGBOT_CONTRACT_VERSION 1

/* program control */
bool     bugbot_shim_should_stop(void);          /* true once the IDE/console asked to stop the script */
void     bugbot_shim_delay_ms(uint32_t ms);       /* yields the interpreter task */
float    bugbot_shim_clock_s(void);               /* seconds since the current script started (clock()) */

/* motion: components -100..100 in the robot frame (forward, right, clockwise). Feeds the deadman. */
void     bugbot_shim_drive(float fwd, float lat, float rot);
void     bugbot_shim_stop(void);
void     bugbot_shim_keepalive(void);            /* wait() calls this to keep the deadman fed */

/* sensors */
float    bugbot_shim_distance_cm(void);           /* nearest object ahead from the ToF centre columns */
bool     bugbot_shim_tof_grid_cm(uint16_t out[64]); /* 8x8 row-major, row 0 = far */
float    bugbot_shim_heading_deg(void);           /* 0..360, clockwise positive, absolute, minus the reset offset */
void     bugbot_shim_position_cm(float *x, float *y);
void     bugbot_shim_velocity_cms(float *vx, float *vy);
void     bugbot_shim_imu_deg(float *heading, float *pitch, float *roll);
int      bugbot_shim_battery_pct(void);
void     bugbot_shim_reset_heading(void);
void     bugbot_shim_reset_position(void);

/* outputs */
void     bugbot_shim_led(uint8_t r, uint8_t g, uint8_t b);
void     bugbot_shim_servo(uint8_t index, float deg); /* index 0 or 1, 0..180 */

/* vision */
typedef struct { int id; float cx, cy, dist_cm; } bugbot_tag_t;
typedef struct { int cx, cy, area, x0, y0, x1, y1; float aspect; } bugbot_blob_t;
typedef struct { int edge_count; float dominant_angle_deg; } bugbot_edges_t;
typedef struct { int x1, y1, x2, y2; float score; int kp[10]; } bugbot_face_t;
bool     bugbot_shim_set_cv(const char *mode);   /* "apriltag" | "blob" | "contour" | "face" | "none" */
int      bugbot_shim_tag_count(void);
bool     bugbot_shim_tag_get(int i, bugbot_tag_t *out);
int      bugbot_shim_blob_count(void);
bool     bugbot_shim_blob_get(int i, bugbot_blob_t *out);
bool     bugbot_shim_edges_get(bugbot_edges_t *out);
int      bugbot_shim_face_count(void);
bool     bugbot_shim_face_get(int i, bugbot_face_t *out);
void     bugbot_shim_camera_suspend(void);
void     bugbot_shim_camera_resume(void);

/* diagnostics (not part of the contract) */
int      bugbot_shim_motor_ok(void);
int      bugbot_shim_motor_raw_test(uint32_t ms);

#ifdef __cplusplus
}
#endif
