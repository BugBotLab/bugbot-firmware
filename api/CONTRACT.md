# BugBot `bugbot` API contract, version 1

Draft 1, 9 September 2026. This is the student-facing surface that must behave identically on every target. It supersedes the surface described in the old docs (`docs/python-api/`) and implemented in `bugbot_module.c` (robot, pocketpy) and `Code/Software/bugbot` (laptop). Decisions from the 9 September discussion are baked in: pocketpy stays as the robot interpreter; there is no buzzer on the new robot.

## Targets

| Code | Target | How the script runs |
|---|---|---|
| R | robot | uploaded to the ESP32-P4 and run by pocketpy against the C firmware |
| P | proxy | runs on the laptop, each call goes to the robot through the dongle |
| S | sim | runs in the browser IDE (or the Python package) against the simulated robot |

A function marked R P S behaves the same on all three. Nothing in version 1 is single-target. A script starts with `import bugbot; bugbot.go()` on every target; `go()` connects where a connection is needed and injects every public name into the script's globals. `from bugbot import *` is the explicit equivalent.

## Conventions

- Speed: 0 to 100, percent of full power. Out-of-range values are clamped silently.
- Distance: centimetres. Velocity: centimetres per second.
- Angles: degrees. Heading is 0 to 360, clockwise positive, absolute from the IMU's magnetometer; `reset_heading()` makes the current facing 0.
- Position: (x, y) in centimetres from where the robot started or was last reset; x is right, y is forward.
- Robot frame for `drive`: forward positive, right positive, clockwise rotation positive.
- Blocking calls are marked. Everything else returns immediately.
- Motion deadman: if no motion command arrives for 500 ms the robot stops. Non-blocking motion must be re-sent; blocking calls handle this internally.
- Errors: a stopped script raises `RuntimeError("BugBot: script stopped")` from any call; a sensor that cannot answer within 2 s raises `TimeoutError`.

## Motion

| Function | Targets | Behaviour |
|---|---|---|
| `forward(speed=50, distance=None)` | R P S | Drive forward. With `distance` (cm), blocks until the optical-flow odometry says it has been covered, then stops. |
| `backward(speed=50, distance=None)` | R P S | Same, backward. |
| `left(speed=50, distance=None)` | R P S | Strafe left (holonomic). Same distance option. |
| `right(speed=50, distance=None)` | R P S | Strafe right. |
| `spin_left(speed=50)` | R P S | Spin anticlockwise on the spot, non-blocking. |
| `spin_right(speed=50)` | R P S | Spin clockwise, non-blocking. |
| `turn(degrees, speed=30)` | R P S | Spin by an angle, closed loop on the IMU, blocking. Positive is clockwise. Safety timeout at 4x the expected time. |
| `drive(fwd, lat, rot=0)` | R P S | The primitive: three components, each -100 to 100, robot frame. Non-blocking. |
| `stop()` | R P S | All motors off. |
| `wait(seconds)` | R P S | Sleep, but keeps the deadman fed for the current motion and honours a stop request. |
| `clock()` | R P S | Seconds since the program started, float. Simulated time in the sim. |
| `bumped()` | R P S | True for about 0.3 s after the robot bumps into something (accelerometer jolt on the robot; a contact in the sim). |

Removed: the laptop's tank-style `drive(left, right)`; `left`/`right` as spins.

## Outputs

| Function | Targets | Behaviour |
|---|---|---|
| `led(colour)` or `led(r, g, b)` | R P S | The one RGB LED. Colour names: red, green, blue, yellow, cyan, magenta, white, orange, purple, pink, off. |
| `servo(index, angle)` | R P S | index 0 or 1, angle 0 to 180. |
| `grip()` | R P S | Close the gripper (servo 0). Returns True if it closed on a ball within about 2 cm of the front. Blocks ~0.3 s. |
| `release()` | R P S | Open the gripper, letting go of the ball. Blocks ~0.3 s. |
| `holding()` | R P S | The colour of the ball in the gripper, or None. |
| `kick(power=100)` | R P S | Fire the kicker (servo 1): the ball in front, held or in reach, shoots off along the heading. Power 0 to 100; 100 sends a ball about 80 cm on the mat. Returns True if it hit a ball. Blocks ~0.3 s. |

Removed: `beep()`. The new robot has no buzzer. Lessons that used a beep use `led()` instead.

## Radio

| Function | Targets | Behaviour |
|---|---|---|
| `send(text)` | R P S | Broadcast a short text message (up to 200 characters) to every other robot on the mat. Non-blocking. On the robot it goes over the dongle link, which relays it to the others. |
| `messages()` | R P S | Every message received since the last call, oldest first, as `[from, text]` pairs (`from` is the sender's name). Your own messages are not included. |

There is no addressing: everyone hears everything, so a message that is meant for one robot carries that robot's name in its text. In a game `me.send(text)` and `me.messages` do the same.

## Sensors

| Function | Targets | Returns |
|---|---|---|
| `distance()` | R P S | Nearest object ahead, cm, from the ToF grid's centre columns. |
| `tof_grid()` | R P S | 64 ints, cm, 8x8 row-major, 45 x 45 degrees. Rows are elevation: row 0 looks up, row 7 down at the mat a few cm ahead; rows 2 and 3 look level. Column 0 is the left. |
| `heading()` | R P S | Absolute heading, degrees 0 to 360. |
| `position()` | R P S | `(x, y)` cm. |
| `velocity()` | R P S | `(vx, vy)` cm/s from the optical-flow sensor, robot frame. New. |
| `imu()` | R P S | `(heading, pitch, roll)` degrees. New. |
| `battery()` | R P S | 0 to 100 percent. |
| `reset_heading()` | R P S | Current facing becomes 0. |
| `reset_position()` | R P S | Current spot becomes (0, 0). New. |

`tof_grid()` in proxy mode needs a new READ type on the dongle link; it is no longer robot-only.

## Vision

| Function | Targets | Returns |
|---|---|---|
| `set_cv(mode, colour=None)` | R P S | `"apriltag"`, `"blob"`, `"line"`, `"contour"`, `"face"`, `"none"`. Runs on the P4. `"blob"` needs the colour to track: `"red"`, `"green"`, `"blue"` or `"yellow"` (one detector, one colour at a time). |
| `apriltags()` | R P S | list of `[id, cx_px, cy_px, dist_cm]`. |
| `blobs()` | R P S | list of `[cx, cy, area, x0, y0, x1, y1, aspect]`. |
| `line()` | R P S | `[cx_px, angle_deg]` for the dark line on the mat ahead, `[]` if none in view. `cx` is where the line crosses the picture about 10 cm ahead (160 = under the nose); `angle` is its direction, positive to the right. Needs `set_cv("line")`. |
| `edges()` | R P S | `[edge_count, dominant_angle_deg]`. |
| `faces()` | R P S | list of `[x1, y1, x2, y2, score, [10 keypoints]]`. |
| `camera_suspend()`, `camera_resume()` | R P S | Power the camera down and up. |

In the sim, vision functions run on the rendered camera view, so the same lesson code works. `tinyml_result()` is dropped from version 1 and returns when the P4 model pipeline is defined.

## Diagnostics (not for lessons)

`motor_ok()`, `motor_raw_test(ms)` stay on the robot for bring-up. They are not part of the contract and may change.

## Return types

pocketpy on the robot returns lists, not tuples, for multi-value results. The laptop and sim implementations return tuples where this document says tuple. Student code that unpacks (`x, y = position()`) works on all three; code that compares types does not, and lessons avoid it.

## Versioning

This document is version 1. Firmware, dongle, laptop library and IDE each report which contract version they implement, and the IDE refuses to run a lesson written for a newer version than the connected robot.

## Conformance

Each implementation carries the same test script, `api_conformance.py`, which calls every function in this table and checks types, ranges and units. It runs in the sim in CI, on the laptop against a robot in proxy mode, and on the robot in upload mode before a release.


Tags on other robots: every BugBot wears an AprilTag on its body (ids from 100 up), so `apriltags()` reports other robots with their distance and bearing, the same as a marker. A lit LED on another robot shows up in `blobs()` as a small patch of the nearest named colour.
