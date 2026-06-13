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

SPEED = 40   # Driving speed (dc%)

# ── GYRO / STEERING CORRECTION ────────────────────────────────────────────────

KP           = 2.0   # Heading error multiplier
KR           = 0.3   # Angular rate dampener
STEER_CENTER = 0     # Straight-ahead steer angle (degrees)
STEER_LIMIT  = 60    # Maximum steer angle allowed (degrees)

# ── TURNS: wait-for-wall + reverse ────────────────────────────────────────────

TURN_STEER          = 60    # Full lock steer angle for reverse turn (degrees)
WALL_TURN_MM        = 100   # Front wall distance that fires the turn (mm)
REVERSE_MS          = 2500  # Duration of reverse during turn (ms)
WALL_TAP_REVERSE_MS = 500   # Reverse duration each time wall is hit while waiting (ms)

# ── LAPS ───────────────────────────────────────────────────────────────────────

TOTAL_LAPS    = 3
LINES_TO_STOP = TOTAL_LAPS * 4   # 4 lines per lap
COOLDOWN_MS   = 1500              # ms cooldown after each line crossing

# ── COLOR THRESHOLDS: BLUE LINE ───────────────────────────────────────────────

BLUE_H_MIN = 205;  BLUE_H_MAX = 230
BLUE_S_MIN = 65;   BLUE_S_MAX = 95
BLUE_V_MIN = 35;   BLUE_V_MAX = 65
BLUE_REF_MIN = 15; BLUE_REF_MAX = 35

# ── COLOR THRESHOLDS: ORANGE LINE ─────────────────────────────────────────────

ORANGE_H_MIN = 335; ORANGE_H_MAX = 15
ORANGE_S_MIN = 35;  ORANGE_S_MAX = 80
ORANGE_V_MIN = 45;  ORANGE_V_MAX = 80
ORANGE_REF_MIN = 25; ORANGE_REF_MAX = 50

# ── HUSKYLENS PILLAR COLOR ID RANGES ──────────────────────────────────────────
# IDs 1–5  → GREEN pillar → robot dodges LEFT
# IDs 6–10 → RED   pillar → robot dodges RIGHT

GREEN_ID_MIN = 1;  GREEN_ID_MAX = 5
RED_ID_MIN   = 6;  RED_ID_MAX   = 10

# ── OBSTACLE DODGE ────────────────────────────────────────────────────────────

DODGE_CY_START  = 30    # cy where ramp begins (pillar far away)
DODGE_CY_FULL   = 160   # cy where full offset is reached (pillar close)
DODGE_SNAP_CY   = 100   # cy threshold for instant full dodge on first sight
DODGE_MAX_OFF   = 35    # Maximum dodge heading offset (degrees)
DODGE_HOLD_MS   = 700   # Hold time after pillar clears frame (ms)
DODGE_RETURN_MS = 600   # Blend-back time after hold (ms)

# ── COUNTER-STEER ─────────────────────────────────────────────────────────────

COUNTER_OFF_RED   = -25   # Left snap after red dodge (degrees)
COUNTER_OFF_GREEN =  30   # Right snap after green dodge (degrees)
COUNTER_HOLD_MS   = 400   # Hold duration (ms)
COUNTER_BLEND_MS  = 300   # Fade duration (ms)

# ── EMERGENCY BRAKE ───────────────────────────────────────────────────────────

ULTRA_FRONT_MM   = 120   # Front distance that triggers emergency stop (mm)
ULTRA_REVERSE_MS = 1500  # Reverse duration after emergency stop (ms)
BLIND_MS         = 1500  # Camera blackout window after emergency stop (ms)

# ── WALL SAFETY NUDGE ─────────────────────────────────────────────────────────

OBS_WALL_MIN_MM  = 150   # Side distance that triggers wall nudge (mm)
OBS_WALL_MAX_OFF = 8     # Heading offset applied by wall nudge (degrees)

# ── STARTUP STRAIGHT ──────────────────────────────────────────────────────────

PARK_EXIT_STEER_ANGLE = -80   # Hard steer angle during exit (degrees; negative = left)
PARK_EXIT_STEER_MS    = 1000  # Time to hold steer before driving (ms)
PARK_EXIT_DRIVE_SPEED =   50  # Drive speed during exit (dc%)
PARK_EXIT_DRIVE_MS    = 1500  # How long to drive forward out of bay (ms)
PARK_EXIT_STRAIGHT_MS = 1000  # How long to gyro-straight after exit (ms)

# ── AVOID STATES ───────────────────────────────────────────────────────────────

AVOID_NONE      = 0
AVOID_ACTIVE    = 1
AVOID_HOLDING   = 2
AVOID_RETURNING = 3
AVOID_COUNTER   = 4

# ── STATE VARS ─────────────────────────────────────────────────────────────────

target      = 0
direction   = 0
lap_lines   = 0
on_line     = False
gyro_on     = False
brake_on    = False

avoid_state = AVOID_NONE
avoid_dir   = 0
avoid_off   = 0
wall_off    = 0

cam_blind  = False
blind_tmr  = StopWatch()

hold_tmr     = StopWatch()
return_tmr   = StopWatch()
counter_tmr  = StopWatch()
cooldown_tmr = StopWatch()
loop_tmr     = StopWatch()

# ── BOOT ───────────────────────────────────────────────────────────────────────

steer.run_target(1000, 0)
hub.light.on(Color.YELLOW)
print("booting...")
wait(2000)

print("starting ESP32...")
lms = PUPRemoteHub(Port.B)
lms.add_channel('hl', 'hhhhhh')
wait(1500)

print("ready  laps:", TOTAL_LAPS, "lines:", LINES_TO_STOP)
hub.light.on(Color.GREEN)
hub.speaker.beep(1000, 100)
hub.speaker.beep(1200, 100)

# ── HELPERS ────────────────────────────────────────────────────────────────────

def front_dist():
    try:
        d = ultra.distance()
        return d if d else 9999
    except:
        return 9999

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
    total_off = avoid_off + wall_off
    err  = hub.imu.heading() - (target + total_off)
    rate = hub.imu.angular_velocity()[2]
    cor  = max(-STEER_LIMIT, min(STEER_LIMIT, KP * err - KR * rate))
    steer.run_target(1000, int(STEER_CENTER + cor), wait=False)

def is_green_block(cid):
    return GREEN_ID_MIN <= cid <= GREEN_ID_MAX

def is_red_block(cid):
    return RED_ID_MIN <= cid <= RED_ID_MAX

def is_any_block(cid):
    return is_green_block(cid) or is_red_block(cid)

def dodge_dir_from_cid(cid):
    return -1 if is_green_block(cid) else 1

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

def compute_dodge_off(cy):
    if cy <= DODGE_CY_START:
        return 0
    if cy >= DODGE_CY_FULL:
        return DODGE_MAX_OFF
    return DODGE_MAX_OFF * (cy - DODGE_CY_START) // (DODGE_CY_FULL - DODGE_CY_START)

def reset_dodge():
    global avoid_state, avoid_dir, avoid_off
    avoid_state = AVOID_NONE
    avoid_dir   = 0
    avoid_off   = 0

def wall_follow(dl, dr):
    global wall_off
    wall_off = 0
    dl_live = dl if dl > 0 else 9999
    dr_live = dr if dr > 0 else 9999
    if dl_live < OBS_WALL_MIN_MM:
        wall_off = OBS_WALL_MAX_OFF
    elif dr_live < OBS_WALL_MIN_MM:
        wall_off = -OBS_WALL_MAX_OFF

# ── EMERGENCY STOP ─────────────────────────────────────────────────────────────

def emergency_stop():
    global cam_blind
    print("EMERGENCY BRAKE")
    hub.light.on(Color.RED)
    drive.brake()
    steer.run_target(500, 0)
    wait(300)

    t = StopWatch()
    while t.time() < ULTRA_REVERSE_MS:
        steer.run_target(500, 0, wait=False)
        drive.dc(-SPEED)
        wait(10)
    drive.brake()
    wait(100)
    steer.run_target(500, 0)
    wait(200)

    reset_dodge()
    cam_blind = True
    blind_tmr.reset()
    hub.light.on(Color.YELLOW)
    print("blind window open for", BLIND_MS, "ms")

# ── TURN: wait-for-wall + reverse ─────────────────────────────────────────────

def wait_for_wall():
    hub.speaker.beep(800, 100)
    print("waiting for wall...")
    while True:
        d = front_dist()
        if d <= WALL_TURN_MM:
            print("wall at", d, "mm — reversing briefly then re-checking")
            t = StopWatch()
            while t.time() < WALL_TAP_REVERSE_MS:
                drive.dc(-SPEED)
                wait(10)
            continue
        drive.dc(SPEED)
        do_gyro()
        wait(10)

def do_turn(tdir):
    global target, lap_lines, direction, on_line, wall_off, gyro_on

    wait_for_wall()

    lap_lines += 1
    reset_dodge()
    wall_off = 0

    drive.brake()
    wait(100)

    if tdir == 'blue':
        hub.light.on(Color.BLUE)
        steer.run_target(1000, -TURN_STEER)
        target    -= 90
        direction  = 1
    else:
        hub.light.on(Color.ORANGE)
        steer.run_target(1000, TURN_STEER)
        target    += 90
        direction  = 2

    wait(100)
    print("turn", tdir, "| line", lap_lines, "| lap", lap_lines // 4)

    gyro_on = False
    t = StopWatch()
    while t.time() < REVERSE_MS:
        drive.dc(-SPEED)
        wait(10)

    drive.brake()
    steer.run_target(500, 0)
    wait(200)

    gyro_on = True
    hub.light.on(Color.GREEN)
    on_line = False

# ── STARTUP ────────────────────────────────────────────────────────────────────

# 1. Reset IMU before anything moves
hub.imu.reset_heading(0)
wait(300)
target = 0
print("IMU reset — starting exit")

# 2. Drive out
hub.light.on(Color.MAGENTA)
steer.run_target(1000, PARK_EXIT_STEER_ANGLE, wait=False)
wait(PARK_EXIT_STEER_MS)
drive.dc(PARK_EXIT_DRIVE_SPEED)
wait(PARK_EXIT_DRIVE_MS)
drive.brake()
hub.light.on(Color.GREEN)
print("exit done")

# 3. Gyro straight, holding heading from after the exit
brake_on = True
gyro_on  = True
print("gyro straight for", PARK_EXIT_STRAIGHT_MS, "ms")
straight_tmr = StopWatch()
while straight_tmr.time() < PARK_EXIT_STRAIGHT_MS:
    do_gyro()
    drive.dc(SPEED)
    wait(10)
drive.brake()
wait(100)
print("starting race")

loop_tmr.reset()

# ── MAIN LOOP ──────────────────────────────────────────────────────────────────

while True:

    loop_tmr.reset()

    hsv = color.hsv()
    ref = color.reflection()
    h, s, v = hsv.h, hsv.s, hsv.v

    cx, cy, cw, ch, cid, d_left, d_right = read_cam()
    df = front_dist()

    # ── emergency brake ───────────────────────────────────────────────────────

    if brake_on and df < ULTRA_FRONT_MM:
        print("front:", df, "mm — braking")
        emergency_stop()
        loop_tmr.reset()
        continue

    # ── camera blind window ───────────────────────────────────────────────────

    if cam_blind:
        if blind_tmr.time() >= BLIND_MS:
            cam_blind = False
            hub.light.on(Color.GREEN)
            print("blind window closed")
        else:
            avoid_off = 0
            wall_follow(d_left, d_right)
            do_gyro()
            drive.dc(SPEED)
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
            continue

    # ── wall safety nudge ─────────────────────────────────────────────────────

    wall_follow(d_left, d_right)

    # ── dodge state machine ───────────────────────────────────────────────────

    line_incoming = (is_blue(h, s, v, ref) and direction in (0, 1)) or \
                    (is_orange(h, s, v, ref) and direction in (0, 2))

    if line_incoming and avoid_state == AVOID_COUNTER and not on_line:
        reset_dodge()
        print("line detected mid counter-steer — cancelling offset")

    block_visible = is_any_block(cid)

    if avoid_state == AVOID_NONE:
        if block_visible:
            avoid_dir   = dodge_dir_from_cid(cid)
            avoid_state = AVOID_ACTIVE
            if cy >= DODGE_SNAP_CY:
                avoid_off = DODGE_MAX_OFF * avoid_dir
                print("SNAP dodge  id:", cid, "cy:", cy, "off:", avoid_off)
            else:
                avoid_off = compute_dodge_off(cy) * avoid_dir
                print("ramp dodge  id:", cid, "cy:", cy, "off:", avoid_off)
            hub.light.on(Color.RED if is_red_block(cid) else Color.GREEN)

    if avoid_state == AVOID_ACTIVE:
        if block_visible:
            raw_off = compute_dodge_off(cy) * avoid_dir
            if avoid_dir == 1 and raw_off > avoid_off:
                avoid_off = raw_off
            elif avoid_dir == -1 and raw_off < avoid_off:
                avoid_off = raw_off
            hub.light.on(Color.RED if avoid_dir == 1 else Color.GREEN)
        else:
            avoid_state = AVOID_HOLDING
            hold_tmr.reset()
            hub.light.on(Color.YELLOW)
            print("cleared — holding:", avoid_off)

    elif avoid_state == AVOID_HOLDING:
        same_color = (avoid_dir == 1 and is_red_block(cid)) or \
                     (avoid_dir == -1 and is_green_block(cid))
        if block_visible and same_color:
            avoid_state = AVOID_ACTIVE
            hub.light.on(Color.RED if avoid_dir == 1 else Color.GREEN)
        elif hold_tmr.time() >= DODGE_HOLD_MS:
            avoid_state = AVOID_RETURNING
            return_tmr.reset()
            hub.light.on(Color.WHITE)
            print("returning...")

    elif avoid_state == AVOID_RETURNING:
        elapsed_ret = return_tmr.time()
        if elapsed_ret >= DODGE_RETURN_MS:
            avoid_state = AVOID_COUNTER
            counter_tmr.reset()
            avoid_off = COUNTER_OFF_RED if avoid_dir == 1 else COUNTER_OFF_GREEN
            print("counter-steer off:", avoid_off)
            hub.light.on(Color.CYAN)
        else:
            avoid_off = DODGE_MAX_OFF * avoid_dir * (DODGE_RETURN_MS - elapsed_ret) // DODGE_RETURN_MS

    elif avoid_state == AVOID_COUNTER:
        elapsed_ctr = counter_tmr.time()
        if elapsed_ctr < COUNTER_HOLD_MS:
            pass
        elif elapsed_ctr < COUNTER_HOLD_MS + COUNTER_BLEND_MS:
            blend_elapsed = elapsed_ctr - COUNTER_HOLD_MS
            counter_start = COUNTER_OFF_RED if avoid_dir == 1 else COUNTER_OFF_GREEN
            avoid_off     = counter_start * (COUNTER_BLEND_MS - blend_elapsed) // COUNTER_BLEND_MS
        else:
            avoid_off   = 0
            avoid_dir   = 0
            avoid_state = AVOID_NONE
            hub.light.on(Color.GREEN)
            print("counter-steer done")

    else:
        avoid_off = 0

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

print("done!", TOTAL_LAPS, "laps | obstacle mode")