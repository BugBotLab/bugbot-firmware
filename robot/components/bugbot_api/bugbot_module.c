/*
 * bugbot_module.c: the Python `bugbot` module for pocketpy, contract v1.
 *
 * Ported from the previous robot's module (api/bugbot_module_v0_s3.c) with the
 * contract changes applied:
 *   left/right strafe (with optional distance); spin_left/spin_right spin
 *   drive(fwd, lat, rot=0) is the primitive; no tank drive
 *   everything in cm; tof_grid() replaces lidar_grid(); beep() removed
 *   new: velocity(), imu(), reset_position()
 *   forward/backward/left/right with distance close on odometry, not on time
 *
 * go() injects every public name into __main__ so the student's script can
 * call forward(), stop() etc. without a prefix.
 */
#include "pocketpy.h"
#include "bugbot_shims.h"
#include <math.h>
#include <string.h>

#define TURN_SPEED    30.0f
#define DEFAULT_SPEED 50.0f

/* ---- helpers ------------------------------------------------------------- */

static bool raise_stopped(void) { return RuntimeError("BugBot: script stopped"); }

static bool get_float(py_StackRef argv, int i, float *out) {
    py_Ref a = py_arg(i);
    if (py_isint(a))   { *out = (float)py_toint(a);   return true; }
    if (py_isfloat(a)) { *out = (float)py_tofloat(a); return true; }
    return TypeError("expected a number, got %t", a->type);
}

static float clamp_speed(float v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }
static float clamp_signed(float v) { return v < -100 ? -100 : (v > 100 ? 100 : v); }

/* Drive until the odometry says `distance_cm` has been covered, then stop and settle.
   Timeout at 4x the expected time at 20 cm/s full scale (the placeholder scale; replaced by calibration). */
static bool travel(float fwd, float lat, float speed, float distance_cm) {
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_drive(fwd, lat, 0);
    if (distance_cm < 0) return true;             /* non-blocking form */
    float x0, y0; bugbot_shim_position_cm(&x0, &y0);
    float expected_ms = distance_cm / (20.0f * clamp_speed(speed) / 100.0f + 1e-3f) * 1000.0f;
    uint32_t max_ms = (uint32_t)(expected_ms * 4); if (max_ms < 1000) max_ms = 1000;
    uint32_t elapsed = 0;
    while (elapsed < max_ms) {
        bugbot_shim_delay_ms(10); elapsed += 10;
        bugbot_shim_keepalive();
        if (bugbot_shim_should_stop()) { bugbot_shim_stop(); return raise_stopped(); }
        float x, y; bugbot_shim_position_cm(&x, &y);
        if (hypotf(x - x0, y - y0) >= distance_cm) break;
    }
    bugbot_shim_stop();
    bugbot_shim_delay_ms(200);                    /* settle, as the sim does */
    return true;
}

/* forward/backward/left/right(speed=50, distance=None) */
#define MOVE_FN(NAME, FWD_SIGN, LAT_SIGN)                                                       \
    static bool NAME(int argc, py_StackRef argv) {                                              \
        if (argc > 2) return TypeError(#NAME "([speed[, distance]]) takes 0 to 2 arguments");   \
        float sp = DEFAULT_SPEED, dist = -1.0f;                                                 \
        if (argc >= 1 && !get_float(argv, 0, &sp)) return false;                                \
        if (argc == 2 && !py_isnone(py_arg(1)) && !get_float(argv, 1, &dist)) return false;     \
        sp = clamp_speed(sp);                                                                   \
        if (!travel((FWD_SIGN) * sp, (LAT_SIGN) * sp, sp, dist < 0 ? -1.0f : fabsf(dist))) return false; \
        py_newnone(py_retval()); return true;                                                   \
    }
MOVE_FN(bb_forward,  1, 0)
MOVE_FN(bb_backward, -1, 0)
MOVE_FN(bb_left,     0, -1)
MOVE_FN(bb_right,    0, 1)

static bool bb_spin(int argc, py_StackRef argv, float sign) {
    if (argc > 1) return TypeError("spin([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_drive(0, 0, sign * clamp_speed(sp));
    py_newnone(py_retval()); return true;
}
static bool bb_spin_left(int argc, py_StackRef argv)  { return bb_spin(argc, argv, -1.0f); }
static bool bb_spin_right(int argc, py_StackRef argv) { return bb_spin(argc, argv,  1.0f); }

/* turn(degrees, speed=30): closed loop on the IMU heading, positive = clockwise. */
static bool bb_turn(int argc, py_StackRef argv) {
    if (argc < 1 || argc > 2) return TypeError("turn(degrees[, speed]) takes 1 or 2 arguments");
    float deg; if (!get_float(argv, 0, &deg)) return false;
    float spd = TURN_SPEED;
    if (argc == 2 && !get_float(argv, 1, &spd)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    float target = fabsf(deg);
    if (target < 0.5f) { py_newnone(py_retval()); return true; }
    uint32_t max_ms = (uint32_t)(target / 15.0f * 1000.0f) * 4; if (max_ms < 500) max_ms = 500;
    bugbot_shim_drive(0, 0, deg > 0 ? clamp_speed(spd) : -clamp_speed(spd));
    float prev = bugbot_shim_heading_deg(), total = 0; uint32_t elapsed = 0;
    while (total < target - 2.0f && elapsed < max_ms) {
        bugbot_shim_delay_ms(10); elapsed += 10;
        bugbot_shim_keepalive();
        if (bugbot_shim_should_stop()) { bugbot_shim_stop(); return raise_stopped(); }
        float cur = bugbot_shim_heading_deg(), d = cur - prev;
        if (d > 180) d -= 360; else if (d < -180) d += 360;
        total += fabsf(d); prev = cur;
    }
    bugbot_shim_stop();
    bugbot_shim_delay_ms(200);
    py_newnone(py_retval()); return true;
}

/* drive(fwd, lat, rot=0) */
static bool bb_drive(int argc, py_StackRef argv) {
    if (argc < 2 || argc > 3) return TypeError("drive(fwd, lat[, rot]) takes 2 or 3 arguments");
    float f, l, r = 0;
    if (!get_float(argv, 0, &f) || !get_float(argv, 1, &l)) return false;
    if (argc == 3 && !get_float(argv, 2, &r)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_drive(clamp_signed(f), clamp_signed(l), clamp_signed(r));
    py_newnone(py_retval()); return true;
}

static bool bb_stop(int argc, py_StackRef argv) { (void)argc; (void)argv; bugbot_shim_stop(); py_newnone(py_retval()); return true; }

/* wait(seconds): keeps the deadman fed, honours stop. */
static bool bb_wait(int argc, py_StackRef argv) {
    if (argc != 1) return TypeError("wait(seconds) takes 1 argument");
    float s; if (!get_float(argv, 0, &s)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    uint32_t ms = (uint32_t)(s * 1000.0f), elapsed = 0;
    while (elapsed < ms) {
        uint32_t chunk = (ms - elapsed > 10) ? 10 : (ms - elapsed);
        bugbot_shim_delay_ms(chunk); elapsed += chunk;
        bugbot_shim_keepalive();
        if (bugbot_shim_should_stop()) return raise_stopped();
    }
    py_newnone(py_retval()); return true;
}

/* clock(): seconds since this script started. */
static bool bb_clock(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newfloat(py_retval(), bugbot_shim_clock_s());
    return true;
}

/* ---- outputs ------------------------------------------------------------- */

static const struct { const char *name; uint8_t r, g, b; } COLOURS[] = {
    {"red",255,0,0},{"green",0,255,0},{"blue",0,0,255},{"yellow",255,255,0},{"cyan",0,255,255},
    {"magenta",255,0,255},{"white",255,255,255},{"orange",255,128,0},{"purple",128,0,128},
    {"pink",255,105,180},{"off",0,0,0},{"black",0,0,0},
};

static bool bb_led(int argc, py_StackRef argv) {
    if (argc == 1 && py_isstr(py_arg(0))) {
        const char *n = py_tostr(py_arg(0));
        for (unsigned i = 0; i < sizeof COLOURS / sizeof COLOURS[0]; i++)
            if (strcmp(n, COLOURS[i].name) == 0) { bugbot_shim_led(COLOURS[i].r, COLOURS[i].g, COLOURS[i].b); py_newnone(py_retval()); return true; }
        return ValueError("unknown colour '%s'", n);
    }
    if (argc != 3) return TypeError("led(colour) or led(r, g, b)");
    float r, g, b;
    if (!get_float(argv, 0, &r) || !get_float(argv, 1, &g) || !get_float(argv, 2, &b)) return false;
    #define C8(v) ((uint8_t)((v) < 0 ? 0 : ((v) > 255 ? 255 : (v))))
    bugbot_shim_led(C8(r), C8(g), C8(b));
    py_newnone(py_retval()); return true;
}

static bool bb_servo(int argc, py_StackRef argv) {
    if (argc != 2) return TypeError("servo(index, angle) takes 2 arguments");
    float idx, deg;
    if (!get_float(argv, 0, &idx) || !get_float(argv, 1, &deg)) return false;
    if (idx != 0 && idx != 1) return ValueError("servo index must be 0 or 1");
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    bugbot_shim_servo((uint8_t)idx, deg);
    py_newnone(py_retval()); return true;
}

/* ---- sensors ------------------------------------------------------------- */

static bool bb_distance(int argc, py_StackRef argv) { (void)argc; (void)argv; py_newfloat(py_retval(), bugbot_shim_distance_cm()); return true; }

static bool bb_tof_grid(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    uint16_t g[64]; bool ok = bugbot_shim_tof_grid_cm(g);
    py_newlistn(py_retval(), 64); py_Ref lst = py_retval();
    for (int i = 0; i < 64; i++) py_newint(py_list_getitem(lst, i), ok ? (py_i64)g[i] : 0);
    return true;
}

static bool bb_heading(int argc, py_StackRef argv) { (void)argc; (void)argv; py_newfloat(py_retval(), bugbot_shim_heading_deg()); return true; }

static bool bb_position(int argc, py_StackRef argv) {
    (void)argc; (void)argv; float x, y; bugbot_shim_position_cm(&x, &y);
    py_newlistn(py_retval(), 2); py_Ref l = py_retval();
    py_newfloat(py_list_getitem(l, 0), x); py_newfloat(py_list_getitem(l, 1), y); return true;
}

static bool bb_velocity(int argc, py_StackRef argv) {
    (void)argc; (void)argv; float vx, vy; bugbot_shim_velocity_cms(&vx, &vy);
    py_newlistn(py_retval(), 2); py_Ref l = py_retval();
    py_newfloat(py_list_getitem(l, 0), vx); py_newfloat(py_list_getitem(l, 1), vy); return true;
}

static bool bb_imu(int argc, py_StackRef argv) {
    (void)argc; (void)argv; float h, p, r; bugbot_shim_imu_deg(&h, &p, &r);
    py_newlistn(py_retval(), 3); py_Ref l = py_retval();
    py_newfloat(py_list_getitem(l, 0), h); py_newfloat(py_list_getitem(l, 1), p); py_newfloat(py_list_getitem(l, 2), r); return true;
}

static bool bb_battery(int argc, py_StackRef argv) { (void)argc; (void)argv; py_newint(py_retval(), bugbot_shim_battery_pct()); return true; }
static bool bb_reset_heading(int argc, py_StackRef argv) { (void)argc; (void)argv; bugbot_shim_reset_heading(); py_newnone(py_retval()); return true; }
static bool bb_reset_position(int argc, py_StackRef argv) { (void)argc; (void)argv; bugbot_shim_reset_position(); py_newnone(py_retval()); return true; }

/* ---- vision -------------------------------------------------------------- */

/* set_cv(mode[, colour]): one detector at a time; "blob" needs the colour to track. */
static bool bb_set_cv(int argc, py_StackRef argv) {
    if (argc < 1 || argc > 2 || !py_isstr(py_arg(0))) return TypeError("set_cv(mode[, colour]) takes a string and an optional colour");
    const char *m = py_tostr(py_arg(0));
    const char *colour = NULL;
    if (argc == 2) { if (!py_isstr(py_arg(1))) return TypeError("set_cv: the colour must be a string"); colour = py_tostr(py_arg(1)); }
    if (!bugbot_shim_set_cv(m, colour)) return ValueError("set_cv: unknown mode '%s' (or a blob without a colour)", m);
    py_newnone(py_retval()); return true;
}

/* line(): [cx_px, angle_deg] of the line on the mat ahead, or [] */
static bool bb_line(int argc, py_StackRef argv) {
    (void)argc; (void)argv; float cx, angle;
    if (!bugbot_shim_line(&cx, &angle)) { py_newlistn(py_retval(), 0); return true; }
    py_newlistn(py_retval(), 2); py_Ref l = py_retval();
    py_newint(py_list_getitem(l, 0), (py_i64)(cx + 0.5f)); py_newfloat(py_list_getitem(l, 1), angle); return true;
}

/* bumped(): the accelerometer felt a jolt in the last moment */
static bool bb_bumped(int argc, py_StackRef argv) { (void)argc; (void)argv; py_newbool(py_retval(), bugbot_shim_bumped()); return true; }

static bool bb_apriltags(int argc, py_StackRef argv) {
    (void)argc; (void)argv; int n = bugbot_shim_tag_count();
    py_newlistn(py_retval(), n); py_Ref outer = py_retval();
    for (int i = 0; i < n; i++) {
        bugbot_tag_t t; if (!bugbot_shim_tag_get(i, &t)) continue;
        py_newlistn(py_list_getitem(outer, i), 4); py_Ref in = py_list_getitem(outer, i);
        py_newint(py_list_getitem(in, 0), t.id); py_newfloat(py_list_getitem(in, 1), t.cx);
        py_newfloat(py_list_getitem(in, 2), t.cy); py_newfloat(py_list_getitem(in, 3), t.dist_cm);
    }
    return true;
}

static bool bb_blobs(int argc, py_StackRef argv) {
    (void)argc; (void)argv; int n = bugbot_shim_blob_count();
    py_newlistn(py_retval(), n); py_Ref outer = py_retval();
    for (int i = 0; i < n; i++) {
        bugbot_blob_t b; if (!bugbot_shim_blob_get(i, &b)) continue;
        py_newlistn(py_list_getitem(outer, i), 8); py_Ref in = py_list_getitem(outer, i);
        int v[7] = {b.cx, b.cy, b.area, b.x0, b.y0, b.x1, b.y1};
        for (int k = 0; k < 7; k++) py_newint(py_list_getitem(in, k), v[k]);
        py_newfloat(py_list_getitem(in, 7), b.aspect);
    }
    return true;
}

static bool bb_edges(int argc, py_StackRef argv) {
    (void)argc; (void)argv; bugbot_edges_t e;
    if (!bugbot_shim_edges_get(&e)) { e.edge_count = 0; e.dominant_angle_deg = 0; }
    py_newlistn(py_retval(), 2); py_Ref l = py_retval();
    py_newint(py_list_getitem(l, 0), e.edge_count); py_newfloat(py_list_getitem(l, 1), e.dominant_angle_deg); return true;
}

static bool bb_faces(int argc, py_StackRef argv) {
    (void)argc; (void)argv; int n = bugbot_shim_face_count();
    py_newlistn(py_retval(), n); py_Ref outer = py_retval();
    for (int i = 0; i < n; i++) {
        bugbot_face_t f; if (!bugbot_shim_face_get(i, &f)) continue;
        py_newlistn(py_list_getitem(outer, i), 6); py_Ref face = py_list_getitem(outer, i);
        py_newint(py_list_getitem(face, 0), f.x1); py_newint(py_list_getitem(face, 1), f.y1);
        py_newint(py_list_getitem(face, 2), f.x2); py_newint(py_list_getitem(face, 3), f.y2);
        py_newfloat(py_list_getitem(face, 4), f.score);
        py_newlistn(py_list_getitem(face, 5), 10); py_Ref kps = py_list_getitem(face, 5);
        for (int k = 0; k < 10; k++) py_newint(py_list_getitem(kps, k), f.kp[k]);
    }
    return true;
}

static bool bb_camera_suspend(int argc, py_StackRef argv) { (void)argc; (void)argv; bugbot_shim_camera_suspend(); py_newnone(py_retval()); return true; }
static bool bb_camera_resume(int argc, py_StackRef argv)  { (void)argc; (void)argv; bugbot_shim_camera_resume();  py_newnone(py_retval()); return true; }

/* ---- diagnostics (not in the contract) ------------------------------------ */
static bool bb_motor_ok(int argc, py_StackRef argv) { (void)argc; (void)argv; py_newint(py_retval(), bugbot_shim_motor_ok()); return true; }
static bool bb_motor_raw_test(int argc, py_StackRef argv) {
    float ms = 2000; if (argc >= 1 && !get_float(argv, 0, &ms)) return false;
    py_newint(py_retval(), bugbot_shim_motor_raw_test((uint32_t)ms)); return true;
}

/* ---- registration ---------------------------------------------------------- */

typedef struct { const char *name; py_CFunction fn; } entry_t;
static const entry_t API[] = {
    {"forward", bb_forward}, {"backward", bb_backward}, {"left", bb_left}, {"right", bb_right},
    {"spin_left", bb_spin_left}, {"spin_right", bb_spin_right}, {"turn", bb_turn}, {"drive", bb_drive},
    {"stop", bb_stop}, {"wait", bb_wait}, {"clock", bb_clock},
    {"led", bb_led}, {"servo", bb_servo},
    {"distance", bb_distance}, {"tof_grid", bb_tof_grid}, {"heading", bb_heading}, {"position", bb_position},
    {"velocity", bb_velocity}, {"imu", bb_imu}, {"battery", bb_battery},
    {"reset_heading", bb_reset_heading}, {"reset_position", bb_reset_position},
    {"set_cv", bb_set_cv}, {"apriltags", bb_apriltags}, {"blobs", bb_blobs}, {"line", bb_line}, {"edges", bb_edges}, {"faces", bb_faces},
    {"bumped", bb_bumped},
    {"camera_suspend", bb_camera_suspend}, {"camera_resume", bb_camera_resume},
    {"motor_ok", bb_motor_ok}, {"motor_raw_test", bb_motor_raw_test},
};

static bool bb_go(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_GlobalRef m = py_getmodule("__main__");
    if (m) for (unsigned i = 0; i < sizeof API / sizeof API[0]; i++) py_bindfunc(m, API[i].name, API[i].fn);
    py_newnone(py_retval()); return true;
}

void bugbot_module_init(void) {
    py_GlobalRef mod = py_newmodule("bugbot");
    for (unsigned i = 0; i < sizeof API / sizeof API[0]; i++) py_bindfunc(mod, API[i].name, API[i].fn);
    py_bindfunc(mod, "go", bb_go);
    py_newint(py_getreg(0), BUGBOT_CONTRACT_VERSION);
    py_setdict(mod, py_name("CONTRACT_VERSION"), py_getreg(0));
}
