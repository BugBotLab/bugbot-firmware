/* drivers_stub.c: compile-time stand-ins so the firmware structure builds before the boards exist.
 * Each function is replaced by a real driver file (drv8830.c, pmw3360.c, bno055.c, vl53l5cx.c,
 * ov5647.c, ws2812.c, servo.c, battery.c) during bring-up. Keep this file until every one is real. */
#include "drivers.h"
#include <stdio.h>

void  drivers_init(void) {
    /* order matters on the real robot:
       1. I2C master at 100 kHz on SDA 26 / SCL 27
       2. ToF LPn (GPIO2) high, wait 2 ms, then talk to the VL53L5CX
       3. CAM_EN (GPIO23) high, wait for the camera regulators, then SCCB
       4. DRV8830s: read FAULT registers, clear
       5. PMW3360: power-up sequence, upload SROM
       6. BNO055: NDOF mode, wait 650 ms */
}

void  drv_motor_set(int index, float duty) { (void)index; (void)duty; }
bool  drv_motor_fault_pending(void) { return false; }
void  drv_motor_clear_faults(void) {}
int   drv_motor_probe(void) { return 0; }

void  drv_flow_read_cm(float *dx, float *dy) { *dx = 0; *dy = 0; }
void  drv_imu_read_deg(float *h, float *p, float *r) { *h = 0; *p = 0; *r = 0; }
bool  drv_tof_read_cm(uint16_t out[64]) { for (int i = 0; i < 64; i++) out[i] = 400; return false; }

float drv_battery_read_v(void) { return 3.9f; }
bool  drv_usb_present(void) { return true; }

void  drv_led_set(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
void  drv_servo_set(int index, float deg) { (void)index; (void)deg; }

void  drv_camera_power(bool on) { (void)on; }
void  drv_camera_set_mode(const char *m) { (void)m; }
