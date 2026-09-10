/* core.c: control loop, fusion, safety, and the shim implementations.
 *
 * SKELETON. Structure and rules are real; every driver call below goes to a stub in
 * components/drivers until the boards arrive. Nothing here has run on hardware.
 *
 * Tasks:
 *   control (100 Hz): deadman, battery cut-off, command -> four DRV8830 duties
 *   fusion  (50 Hz):  PMW3360 flow -> velocity/position, BNO055 -> heading/pitch/roll
 *   tof     (15 Hz):  VL53L5CX 8x8 grid
 *   camera  (5-15 Hz): OV5647 frame -> active CV consumer
 *   python:           runs the student's program on pocketpy
 */
#include "bugbot_core.h"
#include "bugbot_shims.h"
#include "drivers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ---- shared state ---------------------------------------------------------- */
static struct {
    float cmd_fwd, cmd_lat, cmd_rot;    /* -100..100 */
    int64_t last_cmd_us;
    float x_cm, y_cm, vx_cms, vy_cms;   /* fused, robot start frame */
    float heading_deg, pitch_deg, roll_deg, heading_offset_deg;
    float vbat_v;
    uint16_t tof[64];
    bool tof_ok;
    volatile bool stop_program;
    volatile bool program_running;
    char cv_mode[12];
} S;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

static int64_t now_us(void) { return esp_timer_get_time(); }

/* ---- control task (100 Hz) --------------------------------------------------- */
static void control_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        float f, l, r; int64_t age;
        taskENTER_CRITICAL(&lock);
        f = S.cmd_fwd; l = S.cmd_lat; r = S.cmd_rot; age = now_us() - S.last_cmd_us;
        taskEXIT_CRITICAL(&lock);
        /* 500 ms deadman: a program that stops talking stops the robot */
        if (age > (int64_t)BUGBOT_DEADMAN_MS * 1000 && (f != 0 || l != 0 || r != 0)) {
            taskENTER_CRITICAL(&lock); S.cmd_fwd = S.cmd_lat = S.cmd_rot = 0; taskEXIT_CRITICAL(&lock);
            f = l = r = 0;
        }
        /* 3.2 V cut-off: keep thinking, stop driving */
        float vbat = drv_battery_read_v();
        taskENTER_CRITICAL(&lock); S.vbat_v = vbat; taskEXIT_CRITICAL(&lock);
        if (vbat < BUGBOT_VBAT_CUTOFF_V && !drv_usb_present()) f = l = r = 0;
        /* the mix: vibration drive, four motors in an X. Placeholder mix until measured. */
        float m[4] = { f + l + r, f - l - r, f - l + r, f + l - r };
        for (int i = 0; i < 4; i++) drv_motor_set(i, fmaxf(-100, fminf(100, m[i])) / 100.0f);
        /* motor fault line: latched faults are cleared with IN1=IN2=0 on that driver */
        if (drv_motor_fault_pending()) drv_motor_clear_faults();
    }
}

/* ---- fusion task (50 Hz) ------------------------------------------------------ */
static void fusion_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
        float dx, dy; drv_flow_read_cm(&dx, &dy);          /* robot frame since last read */
        float h, p, r; drv_imu_read_deg(&h, &p, &r);
        taskENTER_CRITICAL(&lock);
        S.heading_deg = h; S.pitch_deg = p; S.roll_deg = r;
        S.vx_cms = dx / 0.02f; S.vy_cms = dy / 0.02f;
        float hr = h * (float)M_PI / 180.0f;
        S.x_cm += dx * cosf(hr) + dy * sinf(hr);
        S.y_cm += -dx * sinf(hr) + dy * cosf(hr);
        taskEXIT_CRITICAL(&lock);
    }
}

static void tof_task(void *arg) {
    (void)arg;
    for (;;) {
        uint16_t g[64]; bool ok = drv_tof_read_cm(g);
        taskENTER_CRITICAL(&lock); memcpy(S.tof, g, sizeof g); S.tof_ok = ok; taskEXIT_CRITICAL(&lock);
        vTaskDelay(pdMS_TO_TICKS(66));
    }
}

void bugbot_core_start(void) {
    drivers_init();                 /* I2C 100 kHz, LPn high before ToF, CAM_EN high before camera */
    S.last_cmd_us = now_us();
    strcpy(S.cv_mode, "none");
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(fusion_task,  "fusion",  4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(tof_task,     "tof",     4096, NULL, 3, NULL, 0);
}

/* ---- shims: the Python module's view -------------------------------------------- */
bool bugbot_shim_should_stop(void) { return S.stop_program; }
void bugbot_shim_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }

static int64_t script_t0_us;
void  bugbot_core_mark_script_start(void) { script_t0_us = now_us(); }
float bugbot_shim_clock_s(void) { return (float)(now_us() - script_t0_us) / 1e6f; }

void bugbot_shim_drive(float fwd, float lat, float rot) {
    taskENTER_CRITICAL(&lock);
    S.cmd_fwd = fwd; S.cmd_lat = lat; S.cmd_rot = rot; S.last_cmd_us = now_us();
    taskEXIT_CRITICAL(&lock);
}
void bugbot_shim_stop(void) { bugbot_shim_drive(0, 0, 0); }
void bugbot_shim_keepalive(void) { taskENTER_CRITICAL(&lock); S.last_cmd_us = now_us(); taskEXIT_CRITICAL(&lock); }

bool bugbot_shim_tof_grid_cm(uint16_t out[64]) {
    taskENTER_CRITICAL(&lock); memcpy(out, S.tof, 64 * sizeof(uint16_t)); bool ok = S.tof_ok; taskEXIT_CRITICAL(&lock);
    return ok;
}
float bugbot_shim_distance_cm(void) {
    uint16_t g[64]; if (!bugbot_shim_tof_grid_cm(g)) return 400.0f;
    uint16_t best = 400;
    for (int row = 0; row < 6; row++) for (int c = 3; c <= 4; c++) if (g[row * 8 + c] < best) best = g[row * 8 + c];
    return (float)best;
}
float bugbot_shim_heading_deg(void) {
    taskENTER_CRITICAL(&lock); float h = S.heading_deg - S.heading_offset_deg; taskEXIT_CRITICAL(&lock);
    h = fmodf(h, 360.0f); if (h < 0) h += 360.0f; return h;
}
void bugbot_shim_position_cm(float *x, float *y) { taskENTER_CRITICAL(&lock); *x = S.x_cm; *y = S.y_cm; taskEXIT_CRITICAL(&lock); }
void bugbot_shim_velocity_cms(float *vx, float *vy) { taskENTER_CRITICAL(&lock); *vx = S.vx_cms; *vy = S.vy_cms; taskEXIT_CRITICAL(&lock); }
void bugbot_shim_imu_deg(float *h, float *p, float *r) {
    *h = bugbot_shim_heading_deg();
    taskENTER_CRITICAL(&lock); *p = S.pitch_deg; *r = S.roll_deg; taskEXIT_CRITICAL(&lock);
}
int bugbot_shim_battery_pct(void) {
    taskENTER_CRITICAL(&lock); float v = S.vbat_v; taskEXIT_CRITICAL(&lock);
    float pct = (v - 3.3f) / (4.15f - 3.3f) * 100.0f;      /* rough 1S curve; replaced by a table */
    return (int)fmaxf(0, fminf(100, pct));
}
void bugbot_shim_reset_heading(void) { taskENTER_CRITICAL(&lock); S.heading_offset_deg = S.heading_deg; taskEXIT_CRITICAL(&lock); }
void bugbot_shim_reset_position(void) { taskENTER_CRITICAL(&lock); S.x_cm = S.y_cm = 0; taskEXIT_CRITICAL(&lock); }

void bugbot_shim_led(uint8_t r, uint8_t g, uint8_t b) { drv_led_set(r, g, b); }
void bugbot_shim_servo(uint8_t index, float deg) { drv_servo_set(index, deg); }

bool bugbot_shim_set_cv(const char *mode, const char *colour) {
    static const char *modes[] = {"apriltag", "blob", "line", "contour", "face", "none"};
    static const char *colours[] = {"red", "green", "blue", "yellow"};
    if (strcmp(mode, "blob") == 0) {
        bool ok = false;
        if (colour) for (unsigned i = 0; i < 4; i++) if (strcmp(colour, colours[i]) == 0) ok = true;
        if (!ok) return false;                      /* the blob detector tracks one colour, and it must be named */
    }
    for (unsigned i = 0; i < 6; i++) if (strcmp(mode, modes[i]) == 0) { strncpy(S.cv_mode, mode, sizeof S.cv_mode - 1); drv_camera_set_mode(mode); return true; }
    return false;
}
/* the line detector and the bump sense arrive with the camera pipeline and the IMU fusion; until then: nothing seen */
bool bugbot_shim_line(float *cx_px, float *angle_deg) { (void)cx_px; (void)angle_deg; return false; }
bool bugbot_shim_bumped(void) { return false; }
/* the radio rides on the dongle link, which does not exist yet: messages are dropped */
void bugbot_shim_send(const char *text) { (void)text; }
int  bugbot_shim_tag_count(void) { return 0; }
bool bugbot_shim_tag_get(int i, bugbot_tag_t *o) { (void)i; (void)o; return false; }
int  bugbot_shim_blob_count(void) { return 0; }
bool bugbot_shim_blob_get(int i, bugbot_blob_t *o) { (void)i; (void)o; return false; }
bool bugbot_shim_edges_get(bugbot_edges_t *o) { (void)o; return false; }
int  bugbot_shim_face_count(void) { return 0; }
bool bugbot_shim_face_get(int i, bugbot_face_t *o) { (void)i; (void)o; return false; }
void bugbot_shim_camera_suspend(void) { drv_camera_power(false); }
void bugbot_shim_camera_resume(void) { drv_camera_power(true); }

int bugbot_shim_motor_ok(void) { return drv_motor_probe(); }
int bugbot_shim_motor_raw_test(uint32_t ms) { drv_motor_set(0, 0.5f); vTaskDelay(pdMS_TO_TICKS(ms)); drv_motor_set(0, 0); return 1; }

/* ---- program runner ------------------------------------------------------------- */
bool bugbot_core_program_running(void) { return S.program_running; }
void bugbot_core_stop_program(void) { S.stop_program = true; bugbot_shim_stop(); }

static char *pending_src;
static void python_task(void *arg) {
    (void)arg;
    S.program_running = true; S.stop_program = false;
    bugbot_core_exec(pending_src);
    free(pending_src); pending_src = NULL;
    bugbot_shim_stop();
    S.program_running = false;
    vTaskDelete(NULL);
}
void bugbot_core_run_program(const char *src, size_t len) {
    if (S.program_running) { bugbot_core_stop_program(); while (S.program_running) vTaskDelay(1); }
    pending_src = malloc(len + 1); memcpy(pending_src, src, len); pending_src[len] = 0;
    xTaskCreatePinnedToCore(python_task, "python", 16384, NULL, 2, NULL, 0);
}
