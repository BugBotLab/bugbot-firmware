# IDE to robot link over USB (draft)

The IDE talks to a BugBot over WebSerial (USB CDC on the robot's USB-C, 115200 baud, text). This is a draft; the firmware repo owns the final version and both sides carry the same contract version.

## Commands from the IDE

| Line | Meaning |
|---|---|
| `BUGBOT HELLO` | robot replies `BUGBOT <firmware version> CONTRACT <n>` |
| `BUGBOT RUN <bytes>` then exactly `<bytes>` of program text | store the program and run it; stdout streams back as plain lines |
| `BUGBOT STOP` | stop the running program and all motors |
| `BUGBOT SAVE <bytes>` then the program | store as `main.py` so it runs at power-up |

## Lines from the robot

Plain text is program output. Lines starting with `BUGBOT ` are status: `BUGBOT RUNNING`, `BUGBOT DONE`, `BUGBOT ERROR <message>`, `BUGBOT STOPPED`.

## Flashing

Firmware update uses the ESP32-P4 ROM bootloader over the same port (DFU or serial download), driven by esptool-js in the IDE. Not part of this protocol.
