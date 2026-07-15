#!/opt/anaconda3/bin/python3

"""
buwizz_start.py  -  minimal BuWizz 3.0 Pro control starter (laptop, Python + bleak)

Connects to your BuWizz 3.0 Pro over Bluetooth LE, runs a short, safe motion
self-test on your two motors (XL drive + L steer), and prints live telemetry
(battery, per-port current, drive motor velocity) so you can see the feedback
loop working. This is a foundation to build interactive/autonomous control on.

Protocol values are taken from BuWizz's official 3.0 API document.

Setup:
    python3 -m pip install bleak
    python3 buwizz_start.py

Notes:
    - macOS: first run will prompt for Bluetooth permission for your terminal.
    - Linux: uses BlueZ; connect from here, don't pair in system settings.
    - Do NOT name this file bleak.py.
"""

import argparse
import asyncio
import json
import os
import random
import struct
import time
from bleak import BleakScanner, BleakClient

# ---- Device / protocol constants (from BuWizz 3.0 API) ----------------------
DEVICE_NAME    = "BuWizz3"                                   # advertised name
BUWIZZ_SERVICE = "500592d1-74fb-4481-88b3-9919b1676e93"      # main service UUID
APP_CHAR_HINT  = "2901"   # the application characteristic's distinguishing bytes

CMD_SET_MOTOR      = 0x30  # 6x signed-8bit PWM (-127..127), + brake + lut bytes
CMD_SET_MOTOR_EXT  = 0x31  # 4x int32 refs (ports 1-4, per PU mode) +2x int8 +brake+lut
CMD_XFER_PERIOD    = 0x32  # status report period (ms, 20-255)
CMD_MOTOR_TIMEOUT  = 0x34  # what to do when watchdog trips
CMD_WATCHDOG       = 0x35  # auto-stop if no command within N seconds
CMD_SET_PU_FUNC    = 0x50  # set PU port function (0x10 = simple PWM)
CMD_SET_SERVO_REF  = 0x52  # set servo reference: 4x signed-32bit, PU ports 1-4
STATUS_REPORT_ID   = 0x01

# PU port function bytes (command 0x50)
PU_SIMPLE_PWM = 0x10  # raw PWM, driven via 0x30 motor data
PU_POS_SERVO  = 0x15  # position servo: reference is degrees (cumulative from zero)

# ---- Your build --------------------------------------------------------------
DRIVE_PORT = 1    # PU port the XL drive motor is plugged into (1-4)
STEER_PORT = 2    # PU port the L steering motor is plugged into (1-4)

MAX_DRIVE   = 45  # crawl cap out of 127 (mirrors your power-curve ceiling)
STEER_TIMEOUT = 2.0  # max seconds to wait for the steering servo to converge
STEER_TOL = 3  # consider the target reached within this many degrees

# ---- Steering end-stop calibration ------------------------------------------
# The position servo commands MOTOR-shaft degrees; gearing to the road wheels
# means full lock is many motor degrees away and bounded by hard end stops.
# We find that range by ramping the servo target outward in steps until the
# shaft can no longer follow (it's pressed against a stop), then steer as a
# fraction of the measured lock-to-lock travel. Using the servo (not raw PWM)
# keeps full torque and never leaves servo mode.
CAL_STEP     = 20    # motor-degree increment when probing outward for a stop
CAL_MAX      = 500   # never probe further than this many degrees from center
CAL_LAG      = 10    # if the shaft trails the target by more than this -> at a stop
STEER_MARGIN = 0.85  # only use this fraction of full lock (don't press the stops)

# The lock-to-lock travel is a fixed property of the build, so we cache the
# half-lock span. The encoder's zero is per-session, though, so `center` is NOT
# cached -- each start re-homes to one stop to re-derive it (see quick_home()).
CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "steer_cal.json")

WATCHDOG_S    = 3     # auto-stop if no 0x35 command within this many seconds
WATCHDOG_FEED = 0.5   # re-arm the watchdog at least this often while driving

# ---- Obstacle-avoidance mission ---------------------------------------------
REVERSE_SPEED = 85    # PWM used when backing away from an obstacle
DRIVE_SPEED   = REVERSE_SPEED  # forward PWM (cruise) -- same as reverse
TURN_SPEED    = 100   # PWM while going around with wheels turned (a bit faster)
BACKUP_TIME   = 5.0   # seconds to escape an obstacle (reverse, or forward for a
                      # rear hit) -- 5 s is enough at the higher reverse speed
TURN_DRIVE_TIME = 5.0 # seconds to drive with wheels turned to go around
MAX_RECOVER_DEPTH = 2 # cap on chained forward<->reverse recoveries (boxed in)
MISSION_TIME  = 120.0 # default overall run length (2 min); see --duration

# Cruise acceleration: every ACCEL_INTERVAL seconds of clear straight driving,
# bump the speed by ACCEL_STEP (up to ACCEL_MAX). Resets to DRIVE_SPEED after
# each obstacle, so the car speeds up only while it knows the path is clear.
ACCEL_INTERVAL = 5.0  # seconds of clear driving before each speed bump
ACCEL_STEP     = 10   # PWM added per interval
ACCEL_MAX      = 120  # top cruise PWM (out of 127)

# Obstacle = commanding forward power but the drive motor isn't turning.
STALL_VEL   = 4       # |drive velocity| below this counts as "not moving"
STALL_TIME  = 0.4     # must stay stalled this long (while powered) -> obstacle
DRIVE_GRACE = 0.6     # ignore stalls this long after starting to drive (spin-up)

latest_status = {}
disconnect_info = {"reason": None}
# Filled in by calibrate_steering()/quick_home(): motor-degree center and half.
steer_cal = {"center": 0.0, "half": 0.0}


def load_cache():
    """Return the cached steering half-lock span (float) or None if unavailable
    or unusable."""
    try:
        with open(CACHE_FILE) as f:
            half = float(json.load(f)["half"])
        return half if half >= CAL_STEP else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def save_cache(half):
    """Persist the steering half-lock span next to the script."""
    try:
        with open(CACHE_FILE, "w") as f:
            json.dump({"half": round(half, 1)}, f)
        print(f"  saved steering calibration to {CACHE_FILE}")
    except OSError as e:
        print(f"  [!] could not save calibration: {e}")


def _sb(v):
    """clamp to signed int8 range and return the raw byte value"""
    v = max(-127, min(127, int(v)))
    return v & 0xFF


def motor_packet(values, brake=0, lut=0):
    """values: dict of port(1-6) -> signed PWM (-127..127). Others default 0."""
    m = [0, 0, 0, 0, 0, 0]
    for port, val in values.items():
        m[port - 1] = val
    return bytes([CMD_SET_MOTOR] + [_sb(x) for x in m] + [brake, lut])


def pu_func_packet(functions):
    """functions: dict of PU port(1-4) -> function byte. Others = simple PWM."""
    f = [PU_SIMPLE_PWM] * 4
    for port, fn in functions.items():
        f[port - 1] = fn
    return bytes([CMD_SET_PU_FUNC] + f)


def servo_ref_packet(refs):
    """refs: dict of PU port(1-4) -> signed 32-bit reference. For a position-
    servo port the reference is the target angle in degrees. Others default 0."""
    r = [0, 0, 0, 0]
    for port, val in refs.items():
        r[port - 1] = int(val)
    return bytes([CMD_SET_SERVO_REF]) + b"".join(struct.pack("<i", x) for x in r)


def motor_ext_packet(refs, brake=0, lut=0):
    """Command 0x31: one packet that sets every PU port 1-4 by its own mode --
    PWM (-127..127) for a simple-PWM port, degrees for a position-servo port.
    Lets us drive one motor while holding another as a servo without the two
    fighting over a shared 0x30/0x52 packet. refs: dict of port(1-4) -> ref."""
    r = [0, 0, 0, 0]
    for port, val in refs.items():
        r[port - 1] = int(val)
    body = b"".join(struct.pack("<i", x) for x in r)
    return bytes([CMD_SET_MOTOR_EXT]) + body + bytes([0, 0, brake, lut])


def parse_status(data):
    if not data or data[0] != STATUS_REPORT_ID:
        return None
    flags = data[1]
    out = {
        "error": bool(flags & 0x01),
        "vbat": round(9.0 + data[2] * 0.05, 2),
        "currents": [round(data[3 + i] * 0.015, 2) for i in range(6)],
        "temp": data[9],
        "type": [],     # PU motor type per port (0 = none/unknown), ports 1-4
        "vel": [],      # PU port velocities (signed), ports 1-4
        "abs_pos": [],  # absolute shaft angle 0-359 deg, ports 1-4
        "pos": [],      # cumulative position in degrees (signed), ports 1-4
    }
    # PoweredUp motor data: 8 bytes/port -> type(u8) vel(s8) abs_pos(u16) pos(s32)
    for i in range(4):
        base = 22 + i * 8
        if base + 8 <= len(data):
            mtype, vel, abs_pos, pos = struct.unpack_from("<BbHi", data, base)
            out["type"].append(mtype)
            out["vel"].append(vel)
            out["abs_pos"].append(abs_pos)
            out["pos"].append(pos)
    return out


def on_status(_sender, data):
    s = parse_status(data)
    if s:
        latest_status.update(s)


async def find_app_char(client):
    for service in client.services:
        if service.uuid.lower() == BUWIZZ_SERVICE:
            for c in service.characteristics:
                if APP_CHAR_HINT in c.uuid.lower():
                    return c
    return None


def on_disconnect(_client):
    disconnect_info["reason"] = "peripheral dropped the BLE connection"
    print("  [!] BuWizz disconnected unexpectedly")


async def send(client, char, packet):
    """Write a packet, converting a lost connection into a clear error."""
    if not client.is_connected:
        raise ConnectionError(
            disconnect_info["reason"] or "BuWizz is no longer connected"
        )
    await client.write_gatt_char(char, packet, response=False)


async def feed_watchdog(client, char):
    """Re-arm the connection watchdog. Per the BuWizz 3.0 API, only the 0x35
    command resets the watchdog timer -- motor (0x30) writes do NOT."""
    await send(client, char, bytes([CMD_WATCHDOG, WATCHDOG_S]))


async def hold(client, char, values, seconds, hz=20):
    """Send a motor command repeatedly for `seconds`, re-arming the watchdog."""
    pkt = motor_packet(values)
    end = time.time() + seconds
    last_print = 0.0
    last_feed = 0.0
    while time.time() < end:
        await send(client, char, pkt)
        now = time.time()
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now
        if now - last_print > 0.5 and latest_status:
            c = latest_status.get("currents", [0] * 6)
            v = latest_status.get("vel", [])
            drive_v = v[DRIVE_PORT - 1] if len(v) >= DRIVE_PORT else "?"
            print(f"    vbat={latest_status.get('vbat')}V  "
                  f"I(drive)={c[DRIVE_PORT-1]}A  drive_vel={drive_v}")
            last_print = now
        await asyncio.sleep(1 / hz)


def steer_pos():
    """Current steering motor position in degrees, or None if not reported yet."""
    pos = latest_status.get("pos", [])
    return pos[STEER_PORT - 1] if len(pos) >= STEER_PORT else None


async def servo_to(client, char, target_deg, timeout=STEER_TIMEOUT, tol=STEER_TOL,
                   hz=20):
    """Command the steering position servo to `target_deg` (motor degrees) and
    wait until the shaft reaches it (within `tol`) or `timeout` elapses. The
    command is re-sent every tick because BLE writes here are unacknowledged and
    a lone packet can be dropped. The drive motor is explicitly held at zero so
    the car stays put while the wheels turn. Returns the angle actually reached;
    the onboard PID keeps holding the target after this returns."""
    target = int(target_deg)
    pkt = motor_ext_packet({DRIVE_PORT: 0, STEER_PORT: target})
    end = time.time() + timeout
    last_feed = 0.0
    actual = None
    while time.time() < end:
        await send(client, char, pkt)
        now = time.time()
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now
        actual = steer_pos()
        if actual is not None and abs(actual - target) <= tol:
            break
        await asyncio.sleep(1 / hz)
    return actual


async def probe_stop(client, char, start, direction):
    """Ramp the servo target outward from `start` in CAL_STEP steps until the
    shaft stops advancing (pressed against an end stop). A real stop is detected
    when a step advances the shaft by much less than it was commanded to move --
    not merely when the shaft trails the target (which also happens in motion).
    Returns the furthest angle actually reached."""
    reached = prev = start
    step = 0
    while step < CAL_MAX:
        step += CAL_STEP
        target = start + direction * step
        actual = await servo_to(client, char, target, timeout=1.2, tol=STEER_TOL)
        if actual is None:
            break
        progress = (actual - prev) * direction     # advance this step (deg)
        print(f"    probe {'+' if direction > 0 else '-'}: target={target} deg  "
              f"actual={actual} deg  (advanced {progress:+d})")
        reached = actual
        if progress < CAL_STEP * 0.4:              # barely moved -> at a stop
            break
        prev = actual
    return reached


async def calibrate_steering(client, char):
    """Find both steering end stops by servo probing and record center +
    half-travel (motor deg). Leaves the steer port as a position servo at
    center. Stays in servo mode throughout."""
    print("Calibrating steering travel...")
    await send(client, char, pu_func_packet({STEER_PORT: PU_POS_SERVO}))
    await asyncio.sleep(0.3)
    start = steer_pos()
    if start is None:
        print("  [!] No steering position feedback -- is the steer motor a "
              "PoweredUp motor on the right port?")
        return False
    await servo_to(client, char, start, timeout=0.5, tol=CAL_LAG)  # hold, settle

    right = await probe_stop(client, char, start, +1)
    await servo_to(client, char, start, timeout=3.0, tol=CAL_LAG)  # back to start
    left = await probe_stop(client, char, start, -1)

    steer_cal["center"] = (left + right) / 2.0
    steer_cal["half"] = abs(right - left) / 2.0
    print(f"  left={left} deg  right={right} deg  ->  center={steer_cal['center']:.0f} "
          f"deg, half-lock={steer_cal['half']:.0f} deg (motor)")
    await servo_to(client, char, steer_cal["center"], timeout=3.0)
    if steer_cal["half"] < CAL_STEP:
        print("  [!] Steering travel looks tiny -- calibration may be off.")
        return False
    return True


async def quick_home(client, char, half):
    """Re-home using a cached half-lock span: probe just ONE end stop, then
    derive center = right_stop - half. Faster than a full two-stop calibration
    and still self-correcting for the per-session encoder zero. Leaves the steer
    port as a position servo holding center. Returns True on success."""
    print("Re-homing steering to one stop...")
    await send(client, char, pu_func_packet({STEER_PORT: PU_POS_SERVO}))
    await asyncio.sleep(0.3)
    start = steer_pos()
    if start is None:
        print("  [!] No steering position feedback for re-home.")
        return False
    right = await probe_stop(client, char, start, +1)
    steer_cal["half"] = half
    steer_cal["center"] = right - half
    print(f"  right stop={right} deg, half-lock={half:.0f} deg  ->  "
          f"center={steer_cal['center']:.0f} deg (motor)")
    await servo_to(client, char, steer_cal["center"], timeout=3.0)
    return True


async def steer_to_fraction(client, char, frac):
    """Steer to a fraction of full lock: -1.0 = full left, +1.0 = full right,
    0.0 = center. A safety margin keeps it off the hard stops."""
    frac = max(-1.0, min(1.0, frac))
    target = steer_cal["center"] + frac * steer_cal["half"] * STEER_MARGIN
    actual = await servo_to(client, char, target)
    shown = actual if actual is not None else "?"
    print(f"    steer {frac:+.0%} lock  ->  target={target:.0f} deg  "
          f"actual={shown} deg (motor)")


# ---- Obstacle-avoidance behavior --------------------------------------------
def steer_target(frac):
    """Motor-degree servo target for a steering fraction (-1 left .. +1 right)."""
    frac = max(-1.0, min(1.0, frac))
    return steer_cal["center"] + frac * steer_cal["half"] * STEER_MARGIN


def drive_vel():
    """Current drive-motor velocity (signed), or None if not reported yet."""
    v = latest_status.get("vel", [])
    return v[DRIVE_PORT - 1] if len(v) >= DRIVE_PORT else None


async def drive(client, char, pwm, seconds, steer_deg, detect=True, accelerate=False,
                hz=20):
    """Drive the drive motor at `pwm` for up to `seconds` while holding the
    steering servo at `steer_deg` (motor degrees), via a single 0x31 packet so
    the drive and steer commands never zero each other out.

    If `accelerate` is set, the (forward) speed ramps up by ACCEL_STEP every
    ACCEL_INTERVAL seconds of clear driving, capped at ACCEL_MAX -- used for
    cruising, where a long stretch without a hit means the way is clear.

    If `detect` is set and we are commanding motion (pwm != 0) but the drive
    motor's velocity stays near zero past the spin-up grace period, we treat it
    as an obstacle -- in whichever direction we're going -- stop the drive
    (steering held) and return True. Otherwise return False after the full
    duration."""
    hold_pkt = motor_ext_packet({DRIVE_PORT: 0, STEER_PORT: int(steer_deg)})
    start = time.time()
    end = start + seconds
    last_feed = last_print = 0.0
    stalled_since = None
    cur_pwm = pwm
    while time.time() < end:
        now = time.time()
        if accelerate:
            steps = int((now - start) // ACCEL_INTERVAL)
            cur_pwm = min(ACCEL_MAX, pwm + steps * ACCEL_STEP)
        await send(client, char,
                   motor_ext_packet({DRIVE_PORT: cur_pwm, STEER_PORT: int(steer_deg)}))
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now

        v = drive_vel()
        if detect and pwm != 0 and now - start > DRIVE_GRACE:
            if v is not None and abs(v) < STALL_VEL:
                stalled_since = stalled_since or now
                if now - stalled_since >= STALL_TIME:
                    await send(client, char, hold_pkt)  # stop drive, hold steer
                    where = "ahead" if pwm > 0 else "behind"
                    print(f"    ! stalled (vel~{v}) -> obstacle {where}")
                    return True
            else:
                stalled_since = None

        if now - last_print > 0.5:
            c = latest_status.get("currents", [0] * 6)
            print(f"    pwm={cur_pwm:+d}  drive_vel={v}  I(drive)={c[DRIVE_PORT-1]}A")
            last_print = now
        await asyncio.sleep(1 / hz)
    return False


async def obstacle(client, char, depth=0):
    """Recovery after the car hits something while moving FORWARD: back away,
    then try to steer around it. Mirror of reverse_obstacle()."""
    return await _recover(client, char, going_forward=True, depth=depth)


async def reverse_obstacle(client, char, depth=0):
    """Recovery after the car hits something while REVERSING: pull FORWARD away,
    then try to steer around it in reverse. Mirror of obstacle() -- same moves,
    just the opposite escape direction."""
    return await _recover(client, char, going_forward=False, depth=depth)


async def _recover(client, char, going_forward, depth):
    """Shared recovery body. `going_forward` is the direction we were travelling
    when we hit, so we escape the opposite way and probe (try to go around) in
    the original direction. If we bump a new obstacle while escaping, we recover
    the other way -- bounded by MAX_RECOVER_DEPTH so a boxed-in car can't loop
    forever. Leaves the wheels straight."""
    center = steer_target(0.0)
    # Randomly try around to the right or the left each time.
    turn_frac = random.choice([-1.0, 1.0])
    turn = steer_target(turn_frac)
    turn_name = "right" if turn_frac > 0 else "left"
    # Escape opposite to travel; probe (wheels turned) in the travel direction.
    if going_forward:
        escape_pwm, probe_pwm = -REVERSE_SPEED, TURN_SPEED
        escape_name, probe_name = "reverse", "forward"
    else:
        escape_pwm, probe_pwm = DRIVE_SPEED, -TURN_SPEED
        escape_name, probe_name = "forward", "reverse"
    hit_dir = "front" if going_forward else "rear"
    print(f"  ! {hit_dir} obstacle -- running recovery (depth {depth})")

    # 1. Escape straight away from the obstacle. Watch for a new obstacle in the
    #    escape direction (unless we're at the depth cap): if we bump one while
    #    escaping, recover the other way instead (front<->reverse mirror).
    print(f"  recovery: {escape_name}")
    detect_escape = depth < MAX_RECOVER_DEPTH
    if await drive(client, char, escape_pwm, BACKUP_TIME, center, detect=detect_escape):
        print(f"  recovery: bumped something while moving {escape_name}")
        await _recover(client, char, not going_forward, depth + 1)
        return

    # 2. Stop, turn the wheels (random side), then go around in travel direction.
    print(f"  recovery: wheels {turn_name}, {probe_name}")
    await servo_to(client, char, turn)
    await drive(client, char, probe_pwm, TURN_DRIVE_TIME, turn)

    # 3. Straighten up; the mission loop resumes cruising.
    print("  recovery: straighten")
    await servo_to(client, char, center)


async def run_mission(client, char, duration=MISSION_TIME):
    """Cruise forward and avoid obstacles for about `duration` seconds. Cruising
    accelerates while the path stays clear; each hit triggers the obstacle()
    recovery, then we straighten and cruise again (back at the base speed)."""
    center = steer_target(0.0)
    print(f"\nObstacle-avoidance mission (~{duration:.0f}s)...\n")
    await servo_to(client, char, center)          # wheels straight
    deadline = time.time() + duration
    while time.time() < deadline:
        secs = deadline - time.time()
        print("  cruise forward")
        if await drive(client, char, DRIVE_SPEED, secs, center, accelerate=True):
            await obstacle(client, char)          # recover, then loop and cruise
    print("\nMission time reached.")


async def run_selftest(client, char):
    """Original fixed motion self-test (drive + steer sweep)."""
    print("\nRunning motion self-test...\n")
    print("  drive forward (crawl)")
    await hold(client, char, {DRIVE_PORT: 80}, 2.0)
    print("  stop")
    await hold(client, char, {}, 1.0)
    print("  steer full left")
    await steer_to_fraction(client, char, -1.0)
    print("  steer full right")
    await steer_to_fraction(client, char, 1.0)
    print("  center")
    await steer_to_fraction(client, char, 0.0)
    print("  drive reverse (crawl)")
    await hold(client, char, {DRIVE_PORT: -80}, 2.0)


async def main(recalibrate=False, selftest=False, duration=MISSION_TIME):
    print(f"Scanning for {DEVICE_NAME} ...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print("Not found. Make sure the brick is on (press its button) and in range.")
        return

    print(f"Connecting to {device.address} ...")
    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        app = await find_app_char(client)
        if app is None:
            print("Could not find the application characteristic. Discovered:")
            for service in client.services:
                print("service", service.uuid)
                for c in service.characteristics:
                    print("   char", c.uuid, c.properties)
            return

        await client.start_notify(app, on_status)
        await send(client, app, bytes([CMD_XFER_PERIOD, 50]))          # 20 Hz reports
        await send(client, app, bytes([CMD_MOTOR_TIMEOUT, 0]))          # brake on timeout
        await feed_watchdog(client, app)          # arm watchdog (re-armed in hold())
        await asyncio.sleep(0.3)

        # --recalibrate: do a full two-stop calibration, cache it, and stop.
        if recalibrate:
            if await calibrate_steering(client, app):
                save_cache(steer_cal["half"])
                print("\nRecalibration complete.")
            else:
                print("\nRecalibration failed; cache left unchanged.")
            return

        # Otherwise reuse the cached span with a quick one-stop re-home, falling
        # back to a full calibration if there's no cache (or re-home fails).
        cached = load_cache()
        if cached is not None:
            print(f"Using cached steering span (half-lock={cached:.0f} deg motor).")
            if not await quick_home(client, app, cached):
                print("  Re-home failed; running full calibration...")
                if await calibrate_steering(client, app):
                    save_cache(steer_cal["half"])
        else:
            print("No cached steering calibration; running full calibration...")
            if await calibrate_steering(client, app):
                save_cache(steer_cal["half"])

        try:
            if selftest:
                await run_selftest(client, app)
            else:
                await run_mission(client, app, duration)
        except ConnectionError as e:
            print(f"\nAborting: {e}")
        finally:
            # Only try to stop the motors if we're still connected; otherwise
            # the write would just raise again and mask the real cause.
            if client.is_connected:
                try:
                    await send(client, app, motor_packet({}))  # all stop
                    await asyncio.sleep(0.2)
                    print("\nDone. Motors stopped.")
                except Exception as e:
                    print(f"\nCould not send stop command: {e}")
            else:
                print("\nConnection already lost; brick auto-stops via watchdog.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="BuWizz 3.0 car starter with cached steering calibration.")
    parser.add_argument(
        "--recalibrate", action="store_true",
        help="Only re-run the full steering calibration and save it, then exit "
             "(skip driving).")
    parser.add_argument(
        "--selftest", action="store_true",
        help="Run the fixed motion self-test instead of the obstacle-avoidance "
             "mission.")
    parser.add_argument(
        "--duration", type=float, default=MISSION_TIME, metavar="SECONDS",
        help=f"How long to run the obstacle-avoidance mission, in seconds "
             f"(default {MISSION_TIME:.0f} = 2 minutes).")
    args = parser.parse_args()
    asyncio.run(main(recalibrate=args.recalibrate, selftest=args.selftest,
                     duration=args.duration))