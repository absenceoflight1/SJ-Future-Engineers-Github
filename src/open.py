from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Color, Port
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub

hub   = PrimeHub()
steer = Motor(Port.A)
drive = Motor(Port.C)
color = ColorSensor(Port.E)
ultra = UltrasonicSensor(Port.D)

drive.control.limits(acceleration=20000)
steer.control.limits(acceleration=20000)

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                        TUNABLE CONSTANTS                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

# ── SPEED ──────────────────────────────────────────────────────────────────────

SPEED = 100   # Driving speed (dc%)

# ── GYRO / STEERING CORRECTION ────────────────────────────────────────────────

KP           = 2.0   # Heading error multiplier
KR           = 0.3   # Angular rate dampener
STEER_CENTER = 0     # Straight-ahead steer angle (degrees)
STEER_LIMIT  = 60    # Maximum steer angle allowed (degrees)

# ── TURNS: forward arc ────────────────────────────────────────────────────────

TURN_STEER   = 55    # Steer angle during arc turn (degrees)
TURN_DEGREES = 155   # Drive motor travel during arc turn (degrees)

# ── LAPS ───────────────────────────────────────────────────────────────────────

TOTAL_LAPS    = 3
LINES_TO_STOP = TOTAL_LAPS * 4   # 4 lines per lap
COOLDOWN_MS   = 1500              # ms cooldown after each line crossing

# ── COLOR THRESHOLDS: BLUE LINE ───────────────────────────────────────────────

BLUE_H_MIN = 200;  BLUE_H_MAX = 235
BLUE_S_MIN = 60;   BLUE_S_MAX = 100
BLUE_V_MIN = 30;   BLUE_V_MAX = 70
BLUE_REF_MIN = 10; BLUE_REF_MAX = 40

# ── COLOR THRESHOLDS: ORANGE LINE ─────────────────────────────────────────────

ORANGE_H_MIN = 330; ORANGE_H_MAX = 20
ORANGE_S_MIN = 30;  ORANGE_S_MAX = 85
ORANGE_V_MIN = 40;  ORANGE_V_MAX = 85
ORANGE_REF_MIN = 20; ORANGE_REF_MAX = 55

# ── WALL FOLLOWING ────────────────────────────────────────────────────────────

WALL_MIN_MM   = 120   # Hard safety wall distance (mm)
WALL_KP       = 0.1  # Wall centering proportional gain
WALL_MAX_OFF  = 12    # Max heading offset from wall follow (degrees)
WALL_VALID_MM = 500   # Ignore side readings beyond this (mm)

# ── INNER SWEEP ───────────────────────────────────────────────────────────────

SWEEP_OFFSET  = 12   # Heading angle toward inner lane during sweep (degrees)
SWEEP_DIST_MM = 80   # Forward distance of sweep (mm)
SWEEP_SPEED   = 100   # Drive speed during sweep (dc%)

# ── STATE VARS ─────────────────────────────────────────────────────────────────

target    = 0
direction = 0
lap_lines = 0
on_line   = False
gyro_on   = False
wall_off  = 0
turn_pend = None

wf_dl      = [9999, 9999, 9999]
wf_dr      = [9999, 9999, 9999]
wf_dl_last = 9999
wf_dr_last = 9999

cooldown_tmr = StopWatch()
loop_tmr     = StopWatch()

# ── BOOT ───────────────────────────────────────────────────────────────────────

steer.run_target(1000, 0)
hub.light.on(Color.YELLOW)
print("booting...")
wait(2000)
hub.imu.reset_heading(0)
wait(300)
if abs(hub.imu.heading()) > 2:
    hub.imu.reset_heading(0)
    wait(300)

print("starting ESP32...")
lms = PUPRemoteHub(Port.B)
lms.add_channel('hl', 'hhhhhh')
wait(1500)

print("ready  laps:", TOTAL_LAPS, "lines:", LINES_TO_STOP)
hub.light.on(Color.GREEN)
hub.speaker.beep(1000, 10)

# ── HELPERS ────────────────────────────────────────────────────────────────────

def read_cam():
    try:
        cx, cy, cw, cid, dl, dr = lms.call('hl')
        return (cx, cy, cw, 0, cid,
                dl if dl > 0 else 9999,
                dr if dr > 0 else 9999)
    except:
        return 0, 0, 0, 0, 0, 9999, 9999

def do_gyro():
    if not gyro_on:
        return
    err  = hub.imu.heading() - (target + wall_off)
    rate = hub.imu.angular_velocity()[2]
    cor  = max(-STEER_LIMIT, min(STEER_LIMIT, KP * err - KR * rate))
    steer.run_target(1000, int(STEER_CENTER + cor), wait=False)

def is_blue(h, s, v, ref):
    return BLUE_H_MIN <= h <= BLUE_H_MAX \
       and BLUE_S_MIN <= s <= BLUE_S_MAX \
       and BLUE_V_MIN <= v <= BLUE_V_MAX \
       and BLUE_REF_MIN <= ref <= BLUE_REF_MAX

def is_orange(h, s, v, ref):
    return (h >= ORANGE_H_MIN or h <= ORANGE_H_MAX) \
       and ORANGE_S_MIN <= s <= ORANGE_S_MAX \
       and ORANGE_V_MIN <= v <= ORANGE_V_MAX \
       and ORANGE_REF_MIN <= ref <= ORANGE_REF_MAX

# ── WALL FOLLOW: full centering with 3-sample median ──────────────────────────

def wall_follow(dl, dr):
    global wall_off, wf_dl, wf_dr, wf_dl_last, wf_dr_last
    wall_off = 0

    if not gyro_on or turn_pend or direction == 0:
        return

    wf_dl = [wf_dl[1], wf_dl[2], dl]
    wf_dr = [wf_dr[1], wf_dr[2], dr]

    def med(a, b, c):
        if a > b: a, b = b, a
        if b > c: b, c = c, b
        if a > b: a, b = b, a
        return b

    dlf = med(*wf_dl)
    drf = med(*wf_dr)

    if dlf < 9999: wf_dl_last = dlf
    else:          dlf = wf_dl_last
    if drf < 9999: wf_dr_last = drf
    else:          drf = wf_dr_last

    dl_ok = dlf < WALL_VALID_MM
    dr_ok = drf < WALL_VALID_MM

    if dl_ok and dlf < WALL_MIN_MM:
        wall_off = WALL_MAX_OFF
        return
    if dr_ok and drf < WALL_MIN_MM:
        wall_off = -WALL_MAX_OFF
        return

    if dl_ok and dr_ok:
        wall_off = max(-WALL_MAX_OFF, min(WALL_MAX_OFF, (drf - dlf) * WALL_KP))
    elif dl_ok and dlf < WALL_MIN_MM * 1.5:
        wall_off =  WALL_MAX_OFF // 2
    elif dr_ok and drf < WALL_MIN_MM * 1.5:
        wall_off = -(WALL_MAX_OFF // 2)

# ── INNER SWEEP ───────────────────────────────────────────────────────────────

def inner_sweep(tdir):
    sdir = -1 if tdir == 'blue' else 1
    deg  = int(SWEEP_DIST_MM * 3.4)

    drive.reset_angle(0)
    while abs(drive.angle()) < deg:
        err = hub.imu.heading() - (target + SWEEP_OFFSET * sdir)
        cor = max(-STEER_LIMIT,
                  min(STEER_LIMIT, KP * err - KR * hub.imu.angular_velocity()[2]))
        steer.run_target(1000, int(STEER_CENTER + cor), wait=False)
        drive.dc(SWEEP_SPEED)
        wait(10)

    drive.reset_angle(0)
    while abs(drive.angle()) < deg:
        err = hub.imu.heading() - target
        cor = max(-STEER_LIMIT,
                  min(STEER_LIMIT, KP * err - KR * hub.imu.angular_velocity()[2]))
        steer.run_target(1000, int(STEER_CENTER + cor), wait=False)
        drive.dc(SWEEP_SPEED)
        wait(10)

# ── TURN: forward arc ─────────────────────────────────────────────────────────

def do_turn(tdir):
    global target, lap_lines, direction, on_line, turn_pend, wall_off

    hub.speaker.beep(500, 100)
    lap_lines += 1
    turn_pend  = None
    wall_off   = 0

    if tdir == 'blue':
        hub.light.on(Color.BLUE)
        steer.run_target(1000, TURN_STEER, wait=False)
        target    -= 90
        direction  = 1
    else:
        hub.light.on(Color.ORANGE)
        steer.run_target(1000, -TURN_STEER, wait=False)
        target    += 90
        direction  = 2

    print("turn", tdir, "| line", lap_lines, "| lap", lap_lines // 4)

    drive.reset_angle(0)
    while abs(drive.angle()) < TURN_DEGREES:
        drive.dc(SPEED)
        do_gyro()

    hub.light.on(Color.GREEN)
    on_line = False
    inner_sweep(tdir)

# ── STARTUP ────────────────────────────────────────────────────────────────────

gyro_on = True
target  = 0
loop_tmr.reset()

# ── MAIN LOOP ──────────────────────────────────────────────────────────────────

while True:

    loop_tmr.reset()

    hsv = color.hsv()
    ref = color.reflection()
    h, s, v = hsv.h, hsv.s, hsv.v

    cx, cy, cw, ch, cid, d_left, d_right = read_cam()

    # ── wall centering ────────────────────────────────────────────────────────

    wall_follow(d_left, d_right)

    # ── drive ─────────────────────────────────────────────────────────────────

    do_gyro()
    drive.dc(SPEED)

    # ── lap line detection ─────────────────────────────────────────────────────

    if on_line and cooldown_tmr.time() > COOLDOWN_MS:
        on_line = False
    if not on_line:
        if is_blue(h, s, v, ref) and direction in (0, 1):
            on_line = True; cooldown_tmr.reset(); do_turn('blue')
        elif is_orange(h, s, v, ref) and direction in (0, 2):
            on_line = True; cooldown_tmr.reset(); do_turn('orange')

    if lap_lines >= LINES_TO_STOP:
        break

    wait(10)

# ── FINISH ─────────────────────────────────────────────────────────────────────

drive.brake()
steer.run_target(1000, 0)
hub.light.on(Color.GREEN)
for _ in range(3):
    hub.speaker.beep(1000, 300)
    wait(400)

print("done!", TOTAL_LAPS, "laps | open challenge")