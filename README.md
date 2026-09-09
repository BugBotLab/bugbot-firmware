# bugbot-firmware

Firmware for the BugBot robot (ESP32-P4 on the Vision board) and for the USB dongle. C under ESP-IDF, with the pocketpy Python interpreter on top: students' programs are Python, the firmware owns everything with a deadline.

Part of the [BugBot open-source plan](https://github.com/BugBotLab). The Python surface is defined by [api/CONTRACT.md](api/CONTRACT.md) (contract v1) and is the same on the robot, in the simulator and from the laptop.

## Status

Phase A skeleton, written before the boards arrive. It compiles the structure, not the robot: drivers are stubs against the datasheets, the `bugbot` module is ported from the previous robot to contract v1, and nothing has run on hardware. Bring-up follows [docs/HARDWARE_NOTES.md](docs/HARDWARE_NOTES.md).

## Layout

| path | what |
|---|---|
| `api/CONTRACT.md` | the `bugbot` API contract, the authority for every implementation |
| `api/bugbot_module_v0_s3.c` | the previous robot's module, kept for reference while porting |
| `robot/` | ESP-IDF project for the ESP32-P4 |
| `robot/main/` | `main.c`: boot, tasks, the USB console that runs programs |
| `robot/components/pocketpy/` | the interpreter (vendored) |
| `robot/components/bugbot_api/` | `bugbot_module.c`, the Python `bugbot` module (contract v1) |
| `robot/components/bugbot_core/` | control loop, sensor fusion, safety, the USB console; `bugbot_shims.h` is the C surface the Python module calls |
| `robot/components/drivers/` | one driver per device: DRV8830 x4, PMW3360, BNO055, VL53L5CX, OV5647, WS2812, servos, battery sense |
| `dongle/` | ESP-IDF project for the USB dongle: USB CDC to Wi-Fi bridge |
| `docs/` | hardware notes the firmware must honour, the USB protocol, the dongle protocol |

## The robot's rules

From the hardware design, all implemented in `bugbot_core`, none negotiable from Python:

- 500 ms motion deadman: no command, motors stop.
- 3.2 V battery cut-off (VSYS_SW/2 on GPIO20, ADC1_CH4); warn at 3.4 V.
- No hard reversal when running from USB with no cell.
- MOT_INT is the wired-OR fault line of the four DRV8830s; a latched fault is cleared with IN1 = IN2 = 0.
- ToF LPn (GPIO2) high before any I2C to the module; CAM_EN (GPIO23) high before talking to the camera.
- I2C at 100 kHz.

The full list with the reasoning is in `docs/HARDWARE_NOTES.md`.

## Build

ESP-IDF 5.4 or later with the ESP32-P4 target.

```
cd robot
idf.py set-target esp32p4
idf.py build
```

`build.ps1` wraps this on Windows. Flashing: the P4's ROM supports DFU and serial download on the USB-C port; the IDE does it from the browser.

## Licence

MIT, BugBotLab Ltd. pocketpy is MIT (Blueloveth).
