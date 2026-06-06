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

# speeds
SPEED       = 40
SPEED_FAST  = 100

# gyro
KP = 2.0
KR = 0.3
STEER_CENTER = 0
STEER_LIMIT  = 60

# turns
TURN_STEER        = 60
TURN_DEGREES      = 160
TURN_STEER_FAST   = 55
TURN_DEGREES_FAST = 155

# laps
TOTAL_LAPS    = 3
LINES_TO_STOP = TOTAL_LAPS * 4
COOLDOWN_MS   = 1500

# color thresholds (HSV)
BLUE_H_MIN = 205;  BLUE_H_MAX = 230
BLUE_S_MIN = 65;   BLUE_S_MAX = 95
BLUE_V_MIN = 35;   BLUE_V_MAX = 65
BLUE_REF_MIN = 15; BLUE_REF_MAX = 35

ORANGE_H_MIN = 335; ORANGE_H_MAX = 15
ORANGE_S_MIN = 35;  ORANGE_S_MAX = 80
ORANGE_V_MIN = 45;  ORANGE_V_MAX = 80
ORANGE_REF_MIN = 25; ORANGE_REF_MAX = 50

# camera / obstacle
CAM_MIN_WIDTH      = 30
CAM_HEADING_OFFSET = 30
HOLD_TIME          = 800
RETURN_TIME        = 800

# front ultrasonic (obstacle mode only)
ULTRA_FRONT_MM   = 120
ULTRA_REVERSE_MS = 1500
REV_STEER_RED    =  50
REV_STEER_GREEN  = -50
COUNTER_MS       = 400

# wall following (fast mode only)
WALL_MIN_MM   = 120
WALL_KP       = 0.06
WALL_MAX_OFF  = 12
WALL_VALID_MM = 500

# soft wall nudge (obstacle mode only)
OBS_WALL_MIN_MM  = 150
OBS_WALL_MAX_OFF = 8

# inner sweep (obstacle mode only)
SWEEP_OFFSET  = 12
SWEEP_DIST_MM = 80
SWEEP_SPEED   = 40

# parking exit
PARK_FWD_STEER   = -40
PARK_FWD_SPEED   = 80
PARK_FWD_MS      = 1100
PARK_STRAIGHTEN  = 30
PARK_REV_SPEED   = 85
PARK_REV_MS      = 700
PARK_STEER_OUT   = -35
PARK_OUT_SPEED   = 100
PARK_OUT_MS      = 1300
PARK_ALIGN_STEER = 40
PARK_ALIGN_SPEED = 90
PARK_ALIGN_MS_RIGHT = 2500
PARK_ALIGN_MS_LEFT  = 2000

# parallel parking
PARALLEL_SPEED        = 50
PARALLEL_STEER_SHARP  = 60
PARALLEL_STEER_CENTER = 0

# spike detection
PARK_SPIKE_THRESHOLD  = 150
PARK_BASELINE_SAMPLES = 5
PARK_DETECT_SPEED     = 30

# mode detection threshold
WALL_DETECT_MM = 300

# avoid states
AVOID_NONE      = 0
AVOID_ACTIVE    = 1
AVOID_RETURNING = 2
AVOID_HOLDING   = 3

# state vars
target      = 0
direction   = 0
lap_lines   = 0
on_line     = False
avoid_off   = 0
avoid_state = AVOID_NONE
avoid_dir   = 0
wall_off    = 0
gyro_on     = False
brake_on    = False
fast_mode   = False
turn_pend   = None

wf_dl      = [9999, 9999, 9999]
wf_dr      = [9999, 9999, 9999]
wf_dl_last = 9999
wf_dr_last = 9999

hold_tmr     = StopWatch()
return_tmr   = StopWatch()
cooldown_tmr = StopWatch()

# boot
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

dl_samples = []
dr_samples = []
for _ in range(3):
    try:
        _, _, _, _, dl, dr = lms.call('hl')
        if dl > 0: dl_samples.append(dl)
        if dr > 0: dr_samples.append(dr)
    except:
        pass
    wait(200)

try:
    df = ultra.distance()
    if df is None: df = 9999
except:
    df = 9999

dl = min(dl_samples) if dl_samples else 9999
dr = min(dr_samples) if dr_samples else 9999

print("boot dist  F:", df, "L:", dl, "R:", dr)

close = 0
if df <= WALL_DETECT_MM: close += 1
if dl <= WALL_DETECT_MM: close += 1
if dr <= WALL_DETECT_MM: close += 1

both_sides = dl <= WALL_DETECT_MM and dr <= WALL_DETECT_MM

if close >= 2 or both_sides:
    fast_mode = False
    hub.light.on(Color.RED)
    hub.speaker.beep(frequency=800, duration=200)
    print("obstacle mode  close sensors:", close)
else:
    fast_mode = True
    hub.light.on(Color.WHITE)
    hub.speaker.beep(frequency=1500, duration=150)
    hub.speaker.beep(frequency=1800, duration=150)
    print("fast mode  close sensors:", close)

print("ready  laps:", TOTAL_LAPS, " lines:", LINES_TO_STOP)
hub.light.on(Color.GREEN)
hub.speaker.beep(1000, 100)
hub.speaker.beep(1200, 100)

# ── helpers ──────────────────────────────────────────────────────────────────

def spd():
    return SPEED_FAST if fast_mode else SPEED

def t_steer():
    return TURN_STEER_FAST if fast_mode else TURN_STEER

def t_deg():
    return TURN_DEGREES_FAST if fast_mode else TURN_DEGREES

def do_gyro():
    if not gyro_on:
        return
    err  = hub.imu.heading() - (target + avoid_off + wall_off)
    rate = hub.imu.angular_velocity()[2]
    cor  = max(-STEER_LIMIT, min(STEER_LIMIT, KP * err - KR * rate))
    steer.run_target(1000, int(STEER_CENTER + cor), wait=False)

def front_dist():
    try:
        d = ultra.distance()
        return d if d else 9999
    except:
        return 9999

def read_cam():
    try:
        cx, cy, cw, cid, dl, dr = lms.call('hl')
        return cx, cw, cid, (dl if dl > 0 else 9999), (dr if dr > 0 else 9999)
    except:
        return 0, 0, 0, 9999, 9999

def wall_follow(dl, dr):
    global wall_off, wf_dl, wf_dr, wf_dl_last, wf_dr_last
    wall_off = 0

    if not gyro_on or turn_pend or direction == 0:
        return

    if fast_mode:
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

    else:
        dl_live = dl if dl > 0 else 9999
        dr_live = dr if dr > 0 else 9999

        if dl_live < OBS_WALL_MIN_MM:
            wall_off = OBS_WALL_MAX_OFF
        elif dr_live < OBS_WALL_MIN_MM:
            wall_off = -OBS_WALL_MAX_OFF

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

def emergency_stop():
    print("EMERGENCY BRAKE")
    hub.light.on(Color.RED)
    drive.brake()
    steer.run_target(500, 0)
    wait(300)

    _, cw, cid, _, _ = read_cam()
    if cid == 1:   rs = REV_STEER_RED
    elif cid == 2: rs = REV_STEER_GREEN
    else:          rs = 0

    t = StopWatch()
    while t.time() < ULTRA_REVERSE_MS:
        steer.run_target(500, rs, wait=False)
        drive.dc(-spd())
        wait(10)
    drive.brake()
    wait(100)

    if rs != 0:
        t2 = StopWatch()
        while t2.time() < COUNTER_MS:
            steer.run_target(500, -rs, wait=False)
            drive.dc(-spd())
            wait(10)
        drive.brake()
        wait(100)

    steer.run_target(500, 0)
    wait(200)
    hub.light.on(Color.GREEN)

def inner_sweep(tdir):
    if fast_mode:
        return
    sdir = -1 if tdir == 'blue' else 1
    deg  = int(SWEEP_DIST_MM * 3.4)

    drive.reset_angle(0)
    while abs(drive.angle()) < deg:
        _, cw, cid, _, _ = read_cam()
        if front_dist() < ULTRA_FRONT_MM and brake_on:
            emergency_stop()
            return
        if cid in (1, 2) and cw >= CAM_MIN_WIDTH:
            break
        err = hub.imu.heading() - (target + SWEEP_OFFSET * sdir)
        cor = max(-STEER_LIMIT, min(STEER_LIMIT, KP * err - KR * hub.imu.angular_velocity()[2]))
        steer.run_target(1000, int(STEER_CENTER + cor), wait=False)
        drive.dc(SWEEP_SPEED)
        wait(10)

    drive.reset_angle(0)
    while abs(drive.angle()) < deg:
        _, cw, cid, _, _ = read_cam()
        if front_dist() < ULTRA_FRONT_MM and brake_on:
            emergency_stop()
            return
        if cid in (1, 2) and cw >= CAM_MIN_WIDTH:
            break
        err = hub.imu.heading() - target
        cor = max(-STEER_LIMIT, min(STEER_LIMIT, KP * err - KR * hub.imu.angular_velocity()[2]))
        steer.run_target(1000, int(STEER_CENTER + cor), wait=False)
        drive.dc(SWEEP_SPEED)
        wait(10)

def parking_exit():
    global brake_on, gyro_on
    print("parking exit...")
    hub.light.on(Color.MAGENTA)

    df = front_dist()
    _, _, _, dl, dr = read_cam()
    fc = df <= WALL_DETECT_MM
    lc = dl <= WALL_DETECT_MM
    rc = dr <= WALL_DETECT_MM
    print("park dist  F:", df, "L:", dl, "R:", dr)

    if fc and rc and not lc:
        side = "LEFT"
        fs, st, so, al, am = -PARK_FWD_STEER, -PARK_STRAIGHTEN, -PARK_STEER_OUT, -PARK_ALIGN_STEER, PARK_ALIGN_MS_LEFT
        hub.light.on(Color.BLUE)
    elif fc and lc and not rc:
        side = "RIGHT"
        fs, st, so, al, am = PARK_FWD_STEER, PARK_STRAIGHTEN, PARK_STEER_OUT, PARK_ALIGN_STEER, PARK_ALIGN_MS_RIGHT
        hub.light.on(Color.ORANGE)
    else:
        side = "RIGHT"
        fs, st, so, al, am = PARK_FWD_STEER, PARK_STRAIGHTEN, PARK_STEER_OUT, PARK_ALIGN_STEER, PARK_ALIGN_MS_RIGHT
        hub.light.on(Color.WHITE)
        print("ambiguous — defaulting right")

    print("exit side:", side)

    steer.run_target(500, fs, wait=False)
    drive.run(PARK_FWD_SPEED)
    wait(PARK_FWD_MS)

    drive.brake()
    wait(100)
    steer.run_target(500, st)
    drive.run(-PARK_REV_SPEED)
    wait(PARK_REV_MS)
    drive.brake()
    wait(100)

    steer.run_target(500, so)
    drive.run(PARK_OUT_SPEED)
    wait(PARK_OUT_MS)

    steer.run_target(500, al, wait=False)
    drive.run(PARK_ALIGN_SPEED)
    wait(am)
    steer.run_target(500, 0)
    drive.brake()

    brake_on = True
    gyro_on  = True
    hub.light.on(Color.GREEN)
    print("parking done, exited", side)

def do_turn(tdir):
    global target, lap_lines, direction, on_line
    global avoid_off, avoid_state, avoid_dir, wall_off, turn_pend

    hub.speaker.beep(500, 100)
    lap_lines  += 1
    turn_pend   = None
    avoid_off   = 0
    avoid_state = AVOID_NONE
    avoid_dir   = 0
    wall_off    = 0

    if tdir == 'blue':
        hub.light.on(Color.BLUE)
        steer.run_target(1000, t_steer(), wait=False)
        target    -= 90
        direction  = 1
    else:
        hub.light.on(Color.ORANGE)
        steer.run_target(1000, -t_steer(), wait=False)
        target    += 90
        direction  = 2

    print("turn", tdir, "| line", lap_lines, "| lap", lap_lines // 4)

    drive.reset_angle(0)
    while abs(drive.angle()) < t_deg():
        drive.dc(spd())
        do_gyro()

    hub.light.on(Color.GREEN)
    on_line = False
    inner_sweep(tdir)

# ── parallel parking (obstacle mode only) ────────────────────────────────────

def parallel_park_right():
    speed = PARALLEL_SPEED
    sharp = PARALLEL_STEER_SHARP

    print("parallel park RIGHT")
    hub.light.on(Color.ORANGE)
    hub.speaker.beep(900, 150)

    steer.run_target(1000, -sharp, wait=False)
    wait(500)
    drive.run(-speed)
    wait(2000)

    drive.brake()
    wait(500)
    steer.run_target(1000, sharp, wait=False)
    wait(500)

    drive.run(speed)
    wait(3000)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(3000)
    drive.run(-speed)
    wait(4000)
    drive.brake()
    wait(500)

    steer.run_target(1000, sharp, wait=False)
    wait(1000)
    drive.run(-speed)
    wait(2500)
    drive.brake()
    wait(500)

    steer.run_target(1000, -sharp, wait=False)
    wait(500)
    drive.run(speed)
    wait(1500)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(2000)
    drive.run(-speed)
    wait(1000)

    steer.run_target(1000, -sharp, wait=False)
    wait(500)
    drive.run(speed)
    wait(1500)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(2000)

    hub.light.on(Color.GREEN)
    print("parallel park RIGHT done")


def parallel_park_left():
    speed = PARALLEL_SPEED
    sharp = PARALLEL_STEER_SHARP

    print("parallel park LEFT")
    hub.light.on(Color.BLUE)
    hub.speaker.beep(900, 150)

    steer.run_target(1000, sharp, wait=False)
    wait(500)
    drive.run(-speed)
    wait(2000)

    drive.brake()
    wait(500)
    steer.run_target(1000, -sharp, wait=False)
    wait(500)

    drive.run(speed)
    wait(3000)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(3000)
    drive.run(-speed)
    wait(4000)
    drive.brake()
    wait(500)

    steer.run_target(1000, -sharp, wait=False)
    wait(1000)
    drive.run(-speed)
    wait(2500)
    drive.brake()
    wait(500)

    steer.run_target(1000, sharp, wait=False)
    wait(500)
    drive.run(speed)
    wait(1500)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(2000)
    drive.run(-speed)
    wait(1000)

    steer.run_target(1000, sharp, wait=False)
    wait(500)
    drive.run(speed)
    wait(1500)
    drive.brake()
    wait(500)

    steer.run_target(1000, PARALLEL_STEER_CENTER, wait=False)
    wait(2000)

    hub.light.on(Color.GREEN)
    print("parallel park LEFT done")


def scan_for_parking():
    print("scan_for_parking: creeping, watching for bay...")
    hub.light.on(Color.MAGENTA)
    hub.speaker.beep(600, 200)

    right_buf = []
    left_buf  = []

    timeout = StopWatch()
    SCAN_TIMEOUT_MS = 10000

    while timeout.time() < SCAN_TIMEOUT_MS:
        _, _, _, d_left, d_right = read_cam()

        if d_right <= 0 or d_right >= 9000: d_right = 9999
        if d_left  <= 0 or d_left  >= 9000: d_left  = 9999

        if d_right < 9000:
            right_buf.append(d_right)
            if len(right_buf) > PARK_BASELINE_SAMPLES:
                right_buf.pop(0)

        if d_left < 9000:
            left_buf.append(d_left)
            if len(left_buf) > PARK_BASELINE_SAMPLES:
                left_buf.pop(0)

        if len(right_buf) >= PARK_BASELINE_SAMPLES:
            r_base = sum(right_buf[:-1]) / (PARK_BASELINE_SAMPLES - 1)
            r_curr = right_buf[-1]
            if r_curr > r_base + PARK_SPIKE_THRESHOLD:
                print("RIGHT bay detected! base:", r_base, "curr:", r_curr)
                drive.brake()
                wait(200)
                parallel_park_right()
                return

        if len(left_buf) >= PARK_BASELINE_SAMPLES:
            l_base = sum(left_buf[:-1]) / (PARK_BASELINE_SAMPLES - 1)
            l_curr = left_buf[-1]
            if l_curr > l_base + PARK_SPIKE_THRESHOLD:
                print("LEFT bay detected! base:", l_base, "curr:", l_curr)
                drive.brake()
                wait(200)
                parallel_park_left()
                return

        do_gyro()
        drive.dc(PARK_DETECT_SPEED)
        wait(10)

    drive.brake()
    steer.run_target(500, 0)
    print("scan_for_parking: timeout, no bay found")
    hub.light.on(Color.RED)


# ── startup gate ──────────────────────────────────────────────────────────────

if not fast_mode:
    parking_exit()
    hub.imu.reset_heading(0)
    wait(300)
    target = 0
    print("heading reset, starting race")
else:
    brake_on = True
    gyro_on  = True
    hub.imu.reset_heading(0)
    wait(300)
    target = 0
    print("fast mode, skipping parking")

# ── main loop ─────────────────────────────────────────────────────────────────

while True:

    hsv = color.hsv()
    ref = color.reflection()
    h, s, v = hsv.h, hsv.s, hsv.v

    _, cw, cid, d_left, d_right = read_cam()
    df = front_dist()

    # emergency brake — obstacle mode only
    if not fast_mode and brake_on and df < ULTRA_FRONT_MM:
        print("front sensor:", df, "mm — braking")
        emergency_stop()
        continue

    # obstacle avoidance via camera
    if cid in (1, 2) and cw >= CAM_MIN_WIDTH:
        if avoid_state == AVOID_NONE:
            avoid_dir = 1 if cid == 1 else -1
            print("obstacle id:", cid, "w:", cw)
        avoid_state = AVOID_ACTIVE
        avoid_off   = CAM_HEADING_OFFSET * avoid_dir
        hub.light.on(Color.RED if cid == 1 else Color.GREEN)

    elif cid in (1, 2) and cw < CAM_MIN_WIDTH:
        pass

    elif avoid_state == AVOID_ACTIVE:
        avoid_state = AVOID_HOLDING
        hold_tmr.reset()
        hub.light.on(Color.YELLOW)
        print("cleared, holding...")

    elif avoid_state == AVOID_HOLDING:
        if hold_tmr.time() >= HOLD_TIME:
            avoid_state = AVOID_RETURNING
            avoid_off   = -CAM_HEADING_OFFSET * avoid_dir
            return_tmr.reset()
            hub.light.on(Color.WHITE)
            print("returning...")

    elif avoid_state == AVOID_RETURNING:
        if return_tmr.time() >= RETURN_TIME:
            avoid_state = AVOID_NONE
            avoid_off   = 0
            avoid_dir   = 0
            hub.light.on(Color.GREEN)
            print("clear!")

    else:
        avoid_off = 0

    wall_follow(d_left, d_right)
    do_gyro()
    drive.dc(spd())

    if on_line and cooldown_tmr.time() > COOLDOWN_MS:
        on_line = False

    if not on_line:
        if is_blue(h, s, v, ref) and direction in (0, 1):
            on_line = True
            cooldown_tmr.reset()
            do_turn('blue')
        elif is_orange(h, s, v, ref) and direction in (0, 2):
            on_line = True
            cooldown_tmr.reset()
            do_turn('orange')

    if lap_lines >= LINES_TO_STOP:
        break

    wait(10)

# ── post-race ─────────────────────────────────────────────────────────────────

drive.brake()
steer.run_target(1000, 0)

# parallel parking — obstacle mode only, triggered by lateral ultrasonic spike
if not fast_mode:
    print("race done — entering parking scan")
    gyro_on = True
    scan_for_parking()

hub.light.on(Color.GREEN)
for _ in range(3):
    hub.speaker.beep(1000, 300)
    wait(400)

print("done!", TOTAL_LAPS, "laps |", "fast" if fast_mode else "obstacle", "mode")