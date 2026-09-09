# BugBot dongle

An ESP32 on USB that bridges the laptop to the robot over Wi-Fi, so the laptop never joins the robot's network. `bugbot-python` talks to it over USB serial; it talks to the robot over Wi-Fi.

## Status

Skeleton. The protocol it carries is the successor of the ESP-NOW protocol v1 (`bugbot-python/docs/protocol-v1-espnow.md`): same header, auth block and COMMAND/RESPONSE sub-types, with the transport changed to Wi-Fi and the additions listed in `bugbot-python/CONTRACT_GAP.md`. It is written up as `docs/PROTOCOL_DONGLE.md` when the first robot is on the bench.

## Layout

| path | what |
|---|---|
| `main/` | USB CDC to the laptop, Wi-Fi to the robot, pairing store |

## Open

- Which ESP32: a C3 or C6 module on a small USB-A or USB-C stick. The C6 matches the robot's radio.
- Pairing: keep the robot-side `device_id` and rotatable `pairing_key` from v1; the dongle stores the roster.
