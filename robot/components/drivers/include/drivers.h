/* drivers.h: one function set per device on the BugBot stack. Pins and addresses from the
 * hardware notes (docs/HARDWARE_NOTES.md). All stubs until bring-up. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* pins (ESP32-P4 module GPIOs) */
#define PIN_I2C_SDA        26
#define PIN_I2C_SCL        27
#define PIN_TOF_LPN        2
#define PIN_TOF_INT        3
#define PIN_SERVO0         4
#define PIN_SERVO1         5
#define PIN_LED_DATA       7
#define PIN_FLOW_MOTION    8
#define PIN_FLOW_NCS       9
#define PIN_FLOW_SCLK      10
#define PIN_FLOW_MOSI      11
#define PIN_FLOW_MISO      12
#define PIN_VBAT_SENSE     20   /* ADC1_CH4, VSYS_SW / 2 */
#define PIN_CAM_EN         23

/* I2C addresses */
#define ADDR_DRV8830_0     0x60
#define ADDR_DRV8830_1     0x61
#define ADDR_DRV8830_2     0x62
#define ADDR_DRV8830_3     0x64
#define ADDR_BNO055        0x28
#define ADDR_VL53L5CX      0x29
#define ADDR_OV5647        0x36   /* on the camera's own SCCB lines */

void  drivers_init(void);

/* motors: DRV8830 x4, duty -1..1, current limit 400 mA (0R5 sense) */
void  drv_motor_set(int index, float duty);
bool  drv_motor_fault_pending(void);            /* MOT_INT low */
void  drv_motor_clear_faults(void);             /* IN1=IN2=0 on the faulted driver(s), read FAULT registers */
int   drv_motor_probe(void);                    /* 1 = all four ACK */

/* optical flow: PMW3360 over SPI, returns displacement in cm since the last call, robot frame */
void  drv_flow_read_cm(float *dx, float *dy);

/* IMU: BNO055 in NDOF mode, degrees */
void  drv_imu_read_deg(float *heading, float *pitch, float *roll);

/* ToF: VL53L5CX 8x8, cm, row 0 = top of the view (looking up), row 7 = bottom (the mat) */
bool  drv_tof_read_cm(uint16_t out[64]);

/* battery */
float drv_battery_read_v(void);
bool  drv_usb_present(void);

/* LED (one WS2812, 3.3 V data) and servos (3.3 V PWM, 50 Hz) */
void  drv_led_set(uint8_t r, uint8_t g, uint8_t b);
void  drv_servo_set(int index, float deg);

/* camera: OV5647 over MIPI CSI on the P4 */
void  drv_camera_power(bool on);
void  drv_camera_set_mode(const char *cv_mode);

#ifdef __cplusplus
}
#endif
