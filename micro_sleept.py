#!/usr/bin/env python3
"""
==============================================================================
 EXPRESSION-BASED MICROSLEEP DETECTOR  --  RASPBERRY PI / OV5647 BUILD
 v5 -- picamera2 camera backend, no logging, no streaming, hardened startup
==============================================================================

 WHAT THIS IS
 ------------
 A headless driver-monitoring loop. It watches the driver's FACE (blendshape
 expressions, not eye geometry), decides how tired they are, and escalates
 through a five-rung ladder that ends in a speed-limit byte on a serial link.

   NORMAL -> NOTICE -> WARNING -> ALARM -> INTERVENTION
             console    1 beep    buzzer   buzzer + speed byte

 Two indicators:
   LED     BCM 24 (header pin 18)  ON while a face is visible. Nothing else.
   BUZZER  BCM 23 (header pin 16)  warn = one short beep;
                                   critical = beep-beep, pause, looping.

 ---------------------------------------------------------------------------
 WHAT CHANGED FROM THE i.MX93 BUILD (v4)
 ---------------------------------------------------------------------------
 CAMERA. The OV5647 is a CSI sensor on the libcamera stack. On Bookworm it
 does NOT appear as a normal V4L2 capture device, so cv2.VideoCapture("0")
 either fails or hands you a raw Bayer node. The default backend is now
 Picamera2. The old V4L2 path is kept for USB webcams (--backend v4l2).

 GPIO. The chip that owns the 40-pin header is not always gpiochip0: it is
 pinctrl-bcm2835 / pinctrl-bcm2711 on Pi 3/4 and pinctrl-rp1 on Pi 5, and
 the numbering has moved between kernel releases. The chip is now found by
 LABEL at startup instead of being hardcoded. Line offsets on the Pi are BCM
 numbers, so line 23 really is header pin 16.

 REMOVED. The CSV feature log, the MJPEG preview server, the JPEG snapshot
 dir, and the whole overlay renderer that existed only to feed those two.
 Console output and the two GPIO indicators are the entire UI now.

 ---------------------------------------------------------------------------
 FIXED IN THIS BUILD (these were real lock-ups, not tidying)
 ---------------------------------------------------------------------------
 [1] GPIO WAS ACQUIRED OUTSIDE THE try/finally THAT RELEASED IT.
     Camera missing, model download failing, serial port busy -- any of the
     ordinary startup errors killed the process with the buzzer line
     requested and never driven low. Now the lines are opened INSIDE the
     protected block, and an atexit hook backs that up.

 [2] CAMERA LOSS MID-ALARM FROZE THE ACTUATORS ON.
     The "no frames for 2s" branch did `continue`, skipping both the policy
     update and the actuator dispatch. A camera brown-out during an ALARM
     left the buzzer looping and the speed byte engaged indefinitely. Camera
     silence is now fed to the policy as a NO-FACE frame, exactly like a
     driver who has left the seat, so the ladder de-escalates normally.
     A watchdog also restarts the camera after --camera-timeout seconds.

 [3] NO PER-FRAME EXCEPTION GUARD.
     One transient error anywhere in the loop killed the monitor. The frame
     body is now wrapped; it takes --max-frame-errors consecutive failures
     to give up.

 [4] THE CLOSURE TIMER COULD PIN INTERVENTION FOREVER.
     If the lid signal never falls below close_off, ev.eyes_open is never
     true, and de-escalation from ALARM and above REQUIRES it. Result: the
     speed byte held at an awake driver until power-cycle. There is now a
     sensor-fault escape: an unbroken closure past latch_fault_s while the
     head is still actively moving is a stuck signal, not an unconscious
     person. It demotes to WARNING, releases the limiter, and says so.

 [5] THE gpioset FALLBACK RESPAWNED A PROCESS TWICE A SECOND.
     The periodic forced re-write bypassed the cache, and on that backend a
     "write" means kill-and-respawn. Force is now a no-op when the child is
     alive and already holding the right value.

 Plus: model download has a timeout; calibration requires enough SAMPLES and
 not merely enough wall-clock; the grabber will not release the device out
 from under a thread still inside a read; a warn beep pre-empted by a
 critical no longer fires late; only one thread ever writes the serial port.

 ---------------------------------------------------------------------------
 THE ESCALATION LADDER  (the part worth reading)
 ---------------------------------------------------------------------------
   LEVEL          TRIGGER                                  DRIVER FEELS
   NORMAL         nothing                                  nothing
   NOTICE         trend risk >= 18, or a first short        nothing (console)
                  microsleep (0.40-0.80 s)
   WARNING        trend risk >= 40, vacant stare, a         ONE short beep
                  microsleep >= 0.80 s, or a 2nd
                  microsleep inside 90 s
   ALARM          eyes shut >= 2.0 s, or 3 microsleeps      repeating buzzer
                  inside 90 s
   INTERVENTION   eyes shut >= 3.5 s, or an ALARM left      buzzer + speed byte
                  unheeded 3.0 s, or 4 microsleeps
                  inside 90 s

 Evidence runs in two channels that are deliberately NOT interchangeable:
   HARD   measured eye closure happening right now.
   TREND  PERCLOS, drowsy face, yawns, vacant stare, distraction. Fused into
          one number and CLAMPED below the alarm bar, so no amount of soft
          evidence can ever touch the throttle on its own. PERCLOS is a
          60-second backward-looking estimate; a driver punished by it
          cannot clear the condition by waking up.

 Four stabilisers apply at every level: debounce before entry, hysteresis on
 the trend threshold, a minimum hold once entered, and a confirmed-recovery
 requirement on the way down.

==============================================================================
 SETUP  (Raspberry Pi OS Bookworm, 64-bit)

   sudo apt update
   sudo apt install -y python3-picamera2 python3-libgpiod python3-serial \
                       libcap-dev libcamera-dev

   # --system-site-packages is REQUIRED: picamera2 and libgpiod come from
   # apt and are not sensibly pip-installable.
   python3 -m venv --system-site-packages ~/msenv
   source ~/msenv/bin/activate
   pip install mediapipe opencv-python-headless

   # sanity check before running this script:
   python3 -c "from picamera2 import Picamera2; import gpiod, mediapipe, cv2"

 If that import line fails on numpy after installing mediapipe, pin it:
   pip install "numpy<2"

 WIRING
   buzzer  BCM23  header pin 16   (most modules are active-low: --buzzer-active-low)
   LED     BCM24  header pin 18   through a 220-330 ohm resistor to GND
   GND     header pin 6, 14 or 20
   camera  OV5647 on the CSI ribbon
   serial  USB-TTL on /dev/ttyUSB0, or the Pi UART on /dev/serial0
           (for the UART: raspi-config -> Interface -> Serial -> login shell NO,
            hardware serial YES; then --serial-port /dev/serial0)

 RUN
   source ~/msenv/bin/activate
   python microsleep_pi.py
   python microsleep_pi.py --sensitivity relaxed     # more rope
   python microsleep_pi.py --hard-only               # trend never alerts
   python microsleep_pi.py --no-serial               # buzzer only
   python microsleep_pi.py --test-buzzer             # play patterns, exit
   python microsleep_pi.py --test-led                # blink the LED, exit
   python microsleep_pi.py --test-camera             # prove the camera, exit
   python microsleep_pi.py --explain                 # print the ladder

 STOP
   Ctrl-C or SIGTERM. Both drive every line LOW before releasing it.
==============================================================================
"""

import argparse
import atexit
import glob
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
except ImportError:
    sys.exit("mediapipe not importable. Activate the venv:\n"
             "  source ~/msenv/bin/activate")

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_FILE = os.path.join(HERE, "face_landmarker.task")
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/face_landmarker/"
    "face_landmarker/float16/1/face_landmarker.task"
)


# =============================================================================
#  1. TUNABLE THRESHOLDS
# =============================================================================
@dataclass
class Config:
    # --- lid closure, the HARD evidence channel ---------------------------
    # These four numbers are the whole safety ladder. Everything else in this
    # file is either supporting evidence or plumbing.
    close_on: float = 0.55            # blendshape value that counts as shut
    close_off: float = 0.35           # ...and the lower bar to STAY shut
    blink_max_s: float = 0.35         # shorter than this is just a blink
    microsleep_s: float = 0.40        # NOTICE   (or WARNING if repeated)
    microsleep_warn_s: float = 0.80   # WARNING  (one beep)
    sleep_s: float = 2.00             # ALARM    (buzzer)
    deep_sleep_s: float = 3.50        # INTERVENTION (speed byte)

    min_event_frames: int = 3         # frames of evidence before we believe it

    # --- stuck-signal escape (fix [4]) -------------------------------------
    # An unbroken closure this long, WHILE the head is still moving like an
    # awake driver's, is a threshold that no longer fits this face -- not a
    # person who has been unconscious for a minute with an alert head.
    latch_fault_s: float = 45.0
    latch_fault_motion_ratio: float = 1.8   # head motion vs calibrated noise

    # --- PERCLOS-E (TREND channel) -----------------------------------------
    perclos_window_s: float = 60.0
    perclos_warn: float = 0.15
    perclos_alarm: float = 0.30

    # --- drowsy expression signature (TREND) -------------------------------
    drowsy_window_s: float = 8.0
    squint_warn: float = 0.22
    browdown_warn: float = 0.20
    flatness_warn: float = 0.55

    # --- yawning (TREND) ---------------------------------------------------
    yawn_on: float = 0.45
    yawn_min_s: float = 1.2
    yawn_window_s: float = 120.0
    yawns_warn: int = 2

    # --- EYES STUCK / vacant stare (TREND) ---------------------------------
    stare_window_s: float = 2.5
    gaze_freeze_ratio: float = 2.0
    gaze_frozen_floor: float = 0.012
    head_freeze_ratio: float = 2.0
    head_frozen_floor: float = 0.60
    no_blink_s: float = 4.0
    stare_min_s: float = 2.5
    stare_eyes_open_max: float = 0.45
    stare_flat_ratio: float = 0.75

    # --- distraction / head pose (TREND) -----------------------------------
    yaw_limit: float = 26.0
    pitch_limit: float = 20.0
    distract_min_s: float = 1.5
    nod_drop_deg: float = 14.0
    nod_window_s: float = 0.7

    # =====================================================================
    #  TREND FUSION  --  and the cap that keeps soft evidence off the bus
    # =====================================================================
    trend_cap: float = 65.0
    risk_decay_per_s: float = 12.0
    recovery_decay_mult: float = 3.0   # drain faster once the eyes are open

    risk_notice: float = 18.0
    risk_warn: float = 40.0
    risk_warn_exit: float = 25.0       # hysteresis

    # =====================================================================
    #  REPETITION  --  the other honest route upward
    # =====================================================================
    # One microsleep is an event. Four in ninety seconds is a driver losing
    # the fight, even if no single closure was long. This channel is what
    # makes it SAFE to stay quiet about the first one.
    micro_window_s: float = 90.0
    # The window is for COUNTING, not HOLDING: a tally that never expires
    # means an alarm raised by repetition is permanently "unheeded" and would
    # auto-escalate every time.
    micro_sustain_s: float = 2.50
    micro_warn_n: int = 2
    micro_alarm_n: int = 3
    micro_intervene_n: int = 4

    # --- confirmation (debounce) -------------------------------------------
    # Hard evidence is a measurement, so it is trusted quickly. Trend
    # evidence is an inference, so it must persist.
    warn_confirm_hard_s: float = 0.10
    warn_confirm_s: float = 0.60
    alarm_confirm_hard_s: float = 0.10
    alarm_confirm_s: float = 1.00
    intervene_confirm_s: float = 0.20

    # --- minimum hold and confirmed recovery -------------------------------
    alarm_min_hold_s: float = 1.50
    intervene_min_hold_s: float = 2.00
    alarm_clear_s: float = 1.00
    intervene_clear_s: float = 1.00

    intervene_after_s: float = 3.0     # ALARM ignored this long

    # A head slumping out of frame looks exactly like a driver leaning out of
    # frame. Assume the dangerous reading and hold the level.
    face_loss_hold_s: float = 3.0

    calib_seconds: float = 5.0
    calib_min_samples: int = 25        # and not merely enough wall-clock
    alarm_cooldown_s: float = 4.0
    status_print_min_s: float = 0.40

    # --- buzzer patterns ---------------------------------------------------
    warn_on_s: float = 0.30            # a nudge, not an alarm
    crit_on_s: float = 0.40
    crit_gap_s: float = 0.25
    crit_pause_s: float = 2.00

    warn_hz: float = 660.0
    crit_hz: float = 880.0
    tone_mode: str = "solid"           # "solid" = active buzzer, "pwm" = passive

    noface_delay_s: float = 2.0
    buzzer_reassert_s: float = 0.5     # forced re-write, heals a dropped write

    # --- face-presence LED -------------------------------------------------
    led_hold_s: float = 0.50
    led_reassert_s: float = 2.0


CFG = Config()


def autotune(cfg: Config, fps: float):
    """
    Raise any threshold that would otherwise rest on fewer than
    min_event_frames samples at the frame rate this board actually achieves.

    Worth internalising: a threshold expressed in SECONDS silently becomes a
    threshold in FRAMES once it meets real hardware. 0.40 s is four frames at
    9 fps and twelve at 30 fps. Any logic comparing frame COUNTS to a
    constant is a frame-rate bug waiting to happen.
    """
    changes = []
    if fps <= 0:
        return changes
    dt = 1.0 / fps
    need = cfg.min_event_frames

    def bump(name, floor, why):
        cur = getattr(cfg, name)
        if floor > cur:
            setattr(cfg, name, floor)
            changes.append(f"{name}: {cur:.2f}s -> {floor:.2f}s  ({why})")

    bump("microsleep_s", round(need * dt, 2), f"needs {need} frames at {fps:.1f} fps")
    bump("blink_max_s", round(max(2, need - 1) * dt, 2), "blink must span >= 2 frames")
    bump("stare_window_s", round(8 * dt, 2), "gaze std needs >= 8 samples")
    bump("nod_window_s", round(5 * dt, 2), "nod needs >= 5 pitch samples")

    if fps < 12.0 and cfg.no_blink_s < 6.0:
        changes.append(f"no_blink_s: {cfg.no_blink_s:.2f}s -> 6.00s "
                       f"(blinks get dropped below 12 fps)")
        cfg.no_blink_s = 6.0

    if cfg.stare_min_s < cfg.stare_window_s:
        old = cfg.stare_min_s
        cfg.stare_min_s = cfg.stare_window_s
        changes.append(f"stare_min_s: {old:.2f}s -> {cfg.stare_min_s:.2f}s "
                       f"(cannot be shorter than its own window)")

    # Keep the closure tiers strictly ordered even after a bump.
    if cfg.microsleep_warn_s <= cfg.microsleep_s:
        cfg.microsleep_warn_s = round(cfg.microsleep_s * 2.0, 2)
        changes.append(f"microsleep_warn_s -> {cfg.microsleep_warn_s:.2f}s "
                       f"(must exceed microsleep_s)")
    if cfg.sleep_s <= cfg.microsleep_warn_s:
        cfg.sleep_s = round(cfg.microsleep_warn_s + 1.0, 2)
        changes.append(f"sleep_s -> {cfg.sleep_s:.2f}s (must exceed the warn tier)")
    if cfg.deep_sleep_s <= cfg.sleep_s:
        cfg.deep_sleep_s = round(cfg.sleep_s + 1.5, 2)
        changes.append(f"deep_sleep_s -> {cfg.deep_sleep_s:.2f}s "
                       f"(must exceed sleep_s)")
    return changes


# =============================================================================
#  2. GPIO  --  CHIP DISCOVERY, THEN ONE LINE HELD FOR THE WHOLE RUN
# =============================================================================
PI_CHIP_LABELS = ("pinctrl-bcm2835", "pinctrl-bcm2711", "pinctrl-bcm2712",
                  "pinctrl-rp1", "bcm2835-gpio", "gpio-brcmstb")


def _chip_info(path):
    """Return (label, num_lines) for a chip, across libgpiod v1 and v2."""
    try:
        import gpiod
    except Exception:
        return None
    try:
        chip = gpiod.Chip(path)
    except Exception:
        try:
            chip = gpiod.Chip(os.path.basename(path))
        except Exception:
            return None
    try:
        try:                                   # v2
            info = chip.get_info()
            return info.label, info.num_lines
        except AttributeError:                 # v1
            n = chip.num_lines() if callable(chip.num_lines) else chip.num_lines
            label = chip.label() if callable(chip.label) else chip.label
            return label, n
    except Exception:
        return None
    finally:
        try:
            chip.close()
        except Exception:
            pass


def _chips_via_gpiodetect():
    """Last resort: parse `gpiodetect`. Output: gpiochip0 [pinctrl-rp1] (54 lines)"""
    out = {}
    try:
        txt = subprocess.run(["gpiodetect"], capture_output=True, timeout=3,
                             text=True).stdout
    except Exception:
        return out
    for line in txt.splitlines():
        m = re.match(r"\s*(\S+)\s+\[([^\]]+)\]\s+\((\d+)\s+lines?\)", line)
        if m:
            name, label, n = m.group(1), m.group(2), int(m.group(3))
            out["/dev/" + name if not name.startswith("/dev/") else name] = (label, n)
    return out


def find_gpiochip(preferred=None, verbose=True):
    """
    Find the chip that owns the 40-pin header.

    Why this is not a constant: it is gpiochip0 on a Pi 3, gpiochip0 on a Pi 4
    (Bookworm) but gpiochip4 on some Pi 4 kernels, and gpiochip4 then later
    gpiochip0 on the Pi 5 as the RP1 driver settled. Hardcoding it means the
    program either fails outright or -- much worse -- drives someone else's
    pins. The LABEL is stable; the number is not.

    Line offsets on this chip are BCM numbers, so line 23 is header pin 16.
    """
    if preferred:
        return preferred

    paths = sorted(glob.glob("/dev/gpiochip*"))
    info = {}
    for p in paths:
        got = _chip_info(p)
        if got:
            info[p] = got
    if not info:
        info = _chips_via_gpiodetect()

    for path, (label, n) in info.items():
        if any(label.startswith(k) for k in PI_CHIP_LABELS) and n >= 30:
            if verbose:
                print(f"gpio: header chip is {os.path.basename(path)} "
                      f"[{label}, {n} lines]")
            return os.path.basename(path)

    # Nothing matched by label. Take the widest chip -- on a Pi that is the
    # header, and on anything else it is at least a defensible guess.
    if info:
        path, (label, n) = max(info.items(), key=lambda kv: kv[1][1])
        if verbose:
            sys.stderr.write(f"gpio: no known Pi chip label found; using "
                             f"{os.path.basename(path)} [{label}, {n} lines]. "
                             f"Override with --gpiochip if that is wrong.\n")
        return os.path.basename(path)

    sys.stderr.write("gpio: could not enumerate any chip; assuming gpiochip0\n")
    return "gpiochip0"


class GpioLine:
    """
    Owns one GPIO line for the lifetime of the process.

    Used TWICE here: the buzzer (BCM23) and the face-presence LED (BCM24).
    Two independent instances, two independent requests, same guarantees.

    The line is requested ONCE and released only at close(). Nothing spawns a
    short-lived process per beep or per blink. On the Pi a released line
    reverts to an input rather than latching, but holding it is still the
    right design: it costs nothing and it is the only way to get glitch-free
    output on hardware that does latch.

    Three backends, in order of preference:
      1. libgpiod v2 python bindings   (gpiod.request_lines)
      2. libgpiod v1 python bindings   (gpiod.Chip)
      3. a PERSISTENT `gpioset` child, replaced only on a value change

    Backend 3 is a last resort:  sudo apt install python3-libgpiod
    """

    def __init__(self, chip="gpiochip0", line=23, active_low=False,
                 consumer="microsleep", debug=False):
        self.chip_name = chip
        self.line_num = int(line)
        self.active_low = active_low
        self.consumer = consumer
        self.debug = debug
        self.backend = None
        self._is_on = None            # None = unknown, forces the first write
        self._proc = None
        self._gpioset_style = None    # "v2" -> -c chip line=v ; "v1" -> chip line=v
        self._lock = threading.Lock()
        self.write_errors = 0

        self._req = None
        self._v1_line = None
        self._v1_chip = None

        self._open()

    # ---- backend selection -------------------------------------------------
    def _open(self):
        if self._try_v2():
            self.backend = "libgpiod-v2"
        elif self._try_v1():
            self.backend = "libgpiod-v1"
        else:
            self.backend = "gpioset-persistent"
        self.set(False, force=True)   # establish a known state immediately

    def _try_v2(self):
        try:
            import gpiod
            from gpiod.line import Direction, Value
        except Exception:
            return False
        try:
            settings = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
            try:
                settings.active_low = self.active_low
            except Exception:
                pass
            path = self.chip_name
            if not path.startswith("/dev/"):
                path = "/dev/" + path
            self._req = gpiod.request_lines(
                path, consumer=self.consumer,
                config={self.line_num: settings},
            )
            self._v2_Value = Value
            return True
        except Exception as e:
            sys.stderr.write(f"libgpiod v2 request failed on {self.chip_name} "
                             f"line {self.line_num} ({e}), trying v1\n")
            self._req = None
            return False

    def _try_v1(self):
        try:
            import gpiod
        except Exception:
            return False
        try:
            name = self.chip_name[5:] if self.chip_name.startswith("/dev/") \
                else self.chip_name
            self._v1_chip = gpiod.Chip(name)
            self._v1_line = self._v1_chip.get_line(self.line_num)
            flags = 0
            try:
                if self.active_low:
                    flags = gpiod.LINE_REQ_FLAG_ACTIVE_LOW
            except Exception:
                flags = 0
            self._v1_line.request(consumer=self.consumer,
                                  type=gpiod.LINE_REQ_DIR_OUT,
                                  flags=flags, default_vals=[0])
            return True
        except Exception as e:
            sys.stderr.write(f"libgpiod v1 request failed ({e}), falling back "
                             f"to persistent gpioset. Consider:\n"
                             f"  sudo apt install python3-libgpiod\n")
            self._v1_line = None
            self._v1_chip = None
            return False

    # ---- the only entry point the rest of the program uses ------------------
    def set(self, on: bool, force: bool = False):
        """
        Drive the line high/low.

        force=True writes the hardware even when the cached state already
        matches. Without it, one silently-failed write leaves the cache and
        the pin disagreeing forever: the cache says "on", so every later
        set(True) is skipped as a no-op and the buzzer stays silent through a
        real event -- or the LED stays dark with a face in frame.

        FIX [5]: on the gpioset backend a "write" is a kill-and-respawn, and
        the respawn briefly releases the line. Honouring `force` there meant
        forking a process twice a second for the whole run, glitching the pin
        each time. Force is therefore ignored when the child is alive and
        already holds the value we want -- which is the only case where the
        forced write would have been redundant anyway.
        """
        with self._lock:
            child_ok = (self._proc is not None and self._proc.poll() is None)
            if self.backend == "gpioset-persistent":
                if self._is_on is on and child_ok:
                    return
            elif self._is_on is on and not force:
                return
            try:
                if self.backend == "libgpiod-v2":
                    v = self._v2_Value.ACTIVE if on else self._v2_Value.INACTIVE
                    self._req.set_value(self.line_num, v)
                elif self.backend == "libgpiod-v1":
                    self._v1_line.set_value(1 if on else 0)
                else:
                    self._spawn_persistent(on)
                self._is_on = on          # trust the cache only after success
                if self.debug:
                    sys.stdout.write(f"[gpio {self.line_num}] "
                                     f"{'HIGH' if on else 'LOW '}"
                                     f"{' (forced)' if force else ''}\n")
                    sys.stdout.flush()
            except Exception as e:
                self._is_on = None        # guarantee the next call retries
                self.write_errors += 1
                if self.write_errors <= 5 or self.write_errors % 100 == 0:
                    sys.stderr.write(f"gpio {self.line_num} set failed "
                                     f"({self.write_errors}): {e}\n")

    def _kill_child(self, timeout=0.5):
        old, self._proc = self._proc, None
        if old is None:
            return
        if old.poll() is None:
            try:
                old.terminate()
                old.wait(timeout=timeout)
            except Exception:
                try:
                    old.kill()
                    old.wait(timeout=timeout)
                except Exception:
                    pass

    def _gpioset_argv(self, raw, style):
        if style == "v2":
            return ["gpioset", "-c", self.chip_name, f"{self.line_num}={raw}"]
        return ["gpioset", self.chip_name, f"{self.line_num}={raw}"]

    def _spawn_persistent(self, on):
        """
        Fallback backend. Kill FIRST, then spawn, then verify the child
        survived. Spawning first cannot work: the old process still owns the
        line, so the new one dies instantly with EBUSY.
        """
        raw = 1 if on else 0
        if self.active_low:
            raw = 1 - raw

        self._kill_child()

        styles = [self._gpioset_style] if self._gpioset_style else ["v2", "v1"]
        last_err = None
        for style in styles:
            try:
                proc = subprocess.Popen(self._gpioset_argv(raw, style),
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.PIPE)
            except FileNotFoundError:
                raise RuntimeError("gpioset not found -- "
                                   "sudo apt install gpiod")
            time.sleep(0.008)             # give it a moment to fail loudly
            if proc.poll() is None:
                self._proc = proc
                self._gpioset_style = style
                return
            try:
                last_err = (proc.stderr.read() or b"").decode(errors="replace").strip()
            except Exception:
                last_err = f"exit {proc.returncode}"
        raise RuntimeError(f"gpioset would not hold the line ({last_err})")

    def close(self):
        """Drive LOW, let it land, then release. Never leave the pin driven."""
        try:
            self.set(False, force=True)
            time.sleep(0.08)
        except Exception:
            pass
        with self._lock:
            try:
                if self.backend == "libgpiod-v2" and self._req is not None:
                    self._req.release()
                    self._req = None
                elif self.backend == "libgpiod-v1" and self._v1_line is not None:
                    self._v1_line.release()
                    self._v1_line = None
                    if self._v1_chip is not None:
                        self._v1_chip.close()
                        self._v1_chip = None
                elif self._proc is not None:
                    self._kill_child()
            except Exception as e:
                sys.stderr.write(f"gpio close: {e}\n")


# =============================================================================
#  2b. BUZZER PATTERN ENGINE
# =============================================================================
@dataclass
class Step:
    """
    One slice of a pattern: hold the buzzer `on` for `dur` seconds.

    freq > 0 requests a real pitch, which only exists on a PASSIVE piezo
    driven by bit-banging the line. An ACTIVE buzzer has its own oscillator
    and only understands on/off, so in "solid" mode freq is discarded.
    """
    on: bool
    dur: float
    freq: float = 0.0


def pattern_warn(c: Config):
    """Warning: a single short beep."""
    return [Step(True, c.warn_on_s, c.warn_hz)]


def pattern_critical(c: Config):
    """Critical: beep-beep, long pause. This list is looped."""
    return [Step(True, c.crit_on_s, c.crit_hz),
            Step(False, c.crit_gap_s),
            Step(True, c.crit_on_s, c.crit_hz),
            Step(False, c.crit_pause_s)]


def sleep_precise(seconds):
    """
    time.sleep() overshoots by ~0.5-1 ms on Linux. At audio frequencies a
    half period is only ~1 ms, so that jitter is audible as a warble. Sleep
    for the bulk, spin for the last 300 us. PWM tone generation only.
    """
    if seconds <= 0:
        return
    end = time.perf_counter() + seconds
    coarse = seconds - 0.0003
    if coarse > 0:
        time.sleep(coarse)
    while time.perf_counter() < end:
        pass


class Buzzer:
    """
    Turns alert LEVELS into buzzer PATTERNS, on its own thread.

    Priority is critical > warn. A critical alert interrupts whatever is
    sounding within one 20 ms slice, and when it clears the pattern is
    abandoned mid-step so the buzzer goes quiet the moment the eyes reopen.

    The detection loop never blocks on any of this.

    The buzzer makes exactly two sounds and both mean "your eyes were shut".
    Face presence lives on the LED. Keeping every sound in one class of
    meaning is what stops a driver learning to ignore it.
    """

    def __init__(self, line: GpioLine, cfg: Config, quiet=False):
        self.line = line
        self.cfg = cfg
        self.quiet = quiet

        # PWM needs thousands of writes a second, which only the in-process
        # backends can do; the gpioset fallback would fork per half-cycle.
        self.pwm = (cfg.tone_mode == "pwm" and line.backend.startswith("libgpiod"))
        if cfg.tone_mode == "pwm" and not self.pwm:
            sys.stderr.write("tone-mode pwm needs a libgpiod backend -- "
                             "falling back to solid beeps\n")

        self._critical = threading.Event()
        self._warn_pending = threading.Event()
        self._stop = threading.Event()
        self._last_force = 0.0
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ---- public API --------------------------------------------------------
    def set_critical(self, on: bool):
        if on:
            # A warn beep queued but not yet played is stale the moment a
            # critical starts: firing it after the critical clears is a beep
            # about a condition that has already been superseded.
            self._warn_pending.clear()
            self._critical.set()
        else:
            self._critical.clear()

    def pulse_warn(self):
        if not self._critical.is_set():
            self._warn_pending.set()

    # ---- low level ---------------------------------------------------------
    def _drive(self, on: bool):
        """
        Every steady-state write goes through here. Normally a cached no-op,
        but every buzzer_reassert_s it forces a real hardware write, so a
        single dropped write self-heals within half a second instead of
        leaving the buzzer silent through a real event -- or screaming after
        one.
        """
        now = time.monotonic()
        force = (now - self._last_force) >= self.cfg.buzzer_reassert_s
        if force:
            self._last_force = now
        self.line.set(on, force=force)

    def _hold(self, seconds, abort=None, level=None):
        """Sleep in 20 ms slices so a higher-priority state can cut in."""
        end = time.monotonic() + seconds
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return True
            if self._stop.is_set():
                return False
            if abort is not None and abort():
                return False
            if level is not None:
                self._drive(level)
            self._stop.wait(min(0.02, max(0.001, remaining)))

    def _tone(self, freq, dur, abort=None):
        """Bit-bang a square wave. Passive piezo only."""
        half = 0.5 / freq
        end = time.monotonic() + dur
        while time.monotonic() < end:
            if self._stop.is_set() or (abort is not None and abort()):
                self.line.set(False, force=True)
                return False
            self.line.set(True)
            sleep_precise(half)
            self.line.set(False)
            sleep_precise(half)
        return True

    def _play(self, steps, abort=None):
        """Run one pass of a pattern. Always leaves the line LOW."""
        try:
            for s in steps:
                if self._stop.is_set() or (abort is not None and abort()):
                    return False
                if not s.on:
                    self._drive(False)
                    if not self._hold(s.dur, abort, level=False):
                        return False
                elif self.pwm and s.freq > 0:
                    if not self._tone(s.freq, s.dur, abort):
                        return False
                else:
                    self._drive(True)
                    if not self._hold(s.dur, abort, level=True):
                        return False
            return True
        finally:
            self._drive(False)

    def _worker(self):
        cfg = self.cfg
        while not self._stop.is_set():
            if self._critical.is_set():
                self._play(pattern_critical(cfg),
                           abort=lambda: not self._critical.is_set())
                continue
            if self._warn_pending.is_set():
                self._warn_pending.clear()
                self._play(pattern_warn(cfg), abort=self._critical.is_set)
                continue
            self._drive(False)        # idle: keep re-asserting OFF
            self._stop.wait(0.05)

    def selftest(self):
        print(f"buzzer backend: {self.line.backend}  "
              f"({self.line.chip_name} line {self.line.line_num}"
              f"{', active-low' if self.line.active_low else ''}"
              f", {'pwm tones' if self.pwm else 'solid beeps'})")
        print(f"  1/2  warn -- one {self.cfg.warn_on_s:.2f}s beep ...")
        self.pulse_warn()
        time.sleep(1.5)
        print("  2/2  critical -- beep-beep / pause, 3 cycles ...")
        self.set_critical(True)
        time.sleep(3 * (self.cfg.crit_on_s * 2 + self.cfg.crit_gap_s
                        + self.cfg.crit_pause_s))
        self.set_critical(False)
        time.sleep(0.6)
        print("  done -- buzzer should now be silent")
        if self.line.write_errors:
            print(f"  WARNING: {self.line.write_errors} gpio write error(s)")

    def shutdown(self):
        self._stop.set()
        self._critical.clear()
        self._warn_pending.clear()
        self._thread.join(timeout=2.0)
        self.line.set(False, force=True)


def led_selftest(led: GpioLine, cycles=5, period=0.5):
    """
    Prove the LED before trusting it. Blink, then hold, then go dark.

    If it stays lit while the pin is LOW, you have it wired between 3V3 and
    the pin: use --led-active-low.
    """
    print(f"led: {led.chip_name} line {led.line_num} via {led.backend}"
          f"{' (active-low)' if led.active_low else ''}")
    for i in range(cycles):
        led.set(True, force=True)
        print(f"  {i+1}/{cycles}  ON")
        time.sleep(period)
        led.set(False, force=True)
        time.sleep(period)
    print("  holding ON for 3s -- check brightness and colour")
    led.set(True, force=True)
    time.sleep(3.0)
    led.set(False, force=True)
    print("  done -- LED should now be dark, and stay dark after exit")
    if led.write_errors:
        print(f"  WARNING: {led.write_errors} gpio write error(s)")


# =============================================================================
#  3. SMALL UTILITIES
# =============================================================================
class TimedBuffer:
    """A deque of (timestamp, value) that self-trims to a time window."""

    def __init__(self, window_s):
        self.window_s = window_s
        self.buf = deque()

    def push(self, t, v):
        self.buf.append((t, v))
        cutoff = t - self.window_s
        while self.buf and self.buf[0][0] < cutoff:
            self.buf.popleft()

    def values(self):
        return np.array([v for _, v in self.buf], dtype=np.float32)

    def span(self):
        if len(self.buf) < 2:
            return 0.0
        return self.buf[-1][0] - self.buf[0][0]

    def mean(self, default=0.0):
        v = self.values()
        return float(v.mean()) if v.size else default

    def std(self, default=0.0):
        v = self.values()
        return float(v.std()) if v.size > 2 else default

    def fraction_above(self, thr):
        v = self.values()
        return float((v > thr).mean()) if v.size else 0.0


class SpeedSerial:
    """
    Transmits a raw speed byte while the policy holds a level that commands
    one. The API is a TARGET VALUE, not a boolean, so a graded response
    (caution byte at ALARM, lower byte at INTERVENTION) is possible.

    Runs on its OWN THREAD: a serial write can block if the far end stops
    reading or the cable is pulled, and a stalled detection loop is a missed
    microsleep.

    Protocol matches the AVR side: one raw byte -- bytes([30]) is the value
    30, not the characters '3' and '0'. Re-sent every resend_s so a single
    dropped byte costs one second, not the whole event.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=9600, normal_speed=None,
                 resend_s=1.0, quiet=False, log_every_s=5.0):
        self.port_name = port
        self.baud = baud
        self.normal_speed = None if normal_speed is None else int(normal_speed)
        self.resend_s = resend_s
        self.quiet = quiet
        self.log_every_s = log_every_s
        self.ser = None
        self.sent = 0
        self.ok = False

        self._target = None
        self._tlock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._shutdown_done = threading.Event()
        self._logged_value = None
        self._logged_at = 0.0
        self._thread = None

        try:
            import serial
        except ImportError:
            sys.stderr.write("pyserial not installed -- no speed output.\n"
                             "  sudo apt install python3-serial\n")
            return

        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1)
            self.ok = True
        except Exception as e:
            sys.stderr.write(f"serial port {port} unavailable ({e}) -- "
                             f"continuing without speed output\n")
            return

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def set_target(self, value):
        """value = the byte to hold, or None to release the limit."""
        with self._tlock:
            if value == self._target:
                return
            self._target = value
        self._wake.set()      # a change goes out now, not at the next tick

    def _send(self, value, tag=""):
        if not self.ok or value is None:
            return
        try:
            self.ser.write(bytes([value & 0xFF]))
            self.sent += 1
            now = time.monotonic()
            if not self.quiet and (value != self._logged_value
                                   or now - self._logged_at >= self.log_every_s):
                self._logged_value = value
                self._logged_at = now
                sys.stdout.write(f"[serial] speed {value} -> {self.port_name}"
                                 f"{tag}\n")
                sys.stdout.flush()
        except Exception as e:
            sys.stderr.write(f"serial write failed: {e}\n")

    def _wait(self, seconds):
        self._wake.wait(seconds)
        self._wake.clear()

    def _worker(self):
        held = None
        try:
            while not self._stop.is_set():
                with self._tlock:
                    target = self._target
                if target is not None:
                    self._send(target, "" if target == held else "  (engaged)")
                    self._wait(self.resend_s)
                else:
                    if held is not None:
                        self._logged_value = None
                        if self.normal_speed is not None:
                            self._send(self.normal_speed, "  (released)")
                        elif not self.quiet:
                            sys.stdout.write("[serial] speed limit released\n")
                            sys.stdout.flush()
                    self._wait(0.2)
                held = target
            # The final byte is written HERE, by the same thread that owns
            # every other write. Doing it from the main thread after a join
            # that might have timed out puts two writers on one port.
            if self.normal_speed is not None:
                self._send(self.normal_speed, "  (shutdown)")
        finally:
            self._shutdown_done.set()

    def shutdown(self):
        self.set_target(None)
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._shutdown_done.wait(timeout=2.0)
            self._thread.join(timeout=0.5)
        if self.ok and self.ser is not None:
            try:
                self.ser.close()
                if not self.quiet:
                    print("serial port closed safely.")
            except Exception:
                pass


# =============================================================================
#  4. EVIDENCE  --  the contract between the detector and the policy
# =============================================================================
@dataclass
class Evidence:
    """
    Everything the policy is allowed to see. Making this an explicit object
    is not ceremony: it is what stops the policy from reaching into the
    detector and reasoning about display strings, which is how "SLEEPING"
    ended up wired straight to the throttle in an earlier build.

    Two channels, deliberately separated:
      closure_s   HARD  -- a measurement of what is true right now
      trend_risk  TREND -- an inference about the last minute, capped
    """
    t: float = 0.0
    face: bool = True
    closure_s: float = 0.0        # continuous eye closure now, 0 if open
    eyes_open: bool = True
    trend_risk: float = 0.0       # already clamped to cfg.trend_cap
    new_microsleep: bool = False  # a microsleep was counted THIS frame
    stare: bool = False
    sensor_fault: bool = False    # the lid signal is stuck, do not trust it
    status: str = "ALERT"         # display label only
    reasons: list = field(default_factory=list)


class SafetyPolicy:
    """
    The escalation state machine between the detector and the actuators.
    The detector reports what it measures; this decides what the vehicle
    does about it.

    Read _evidence() top to bottom -- the entire safety argument is there,
    and every branch is one line of consequence.
    """

    NORMAL, NOTICE, WARNING, ALARM, INTERVENE = 0, 1, 2, 3, 4
    NAMES = {0: "NORMAL", 1: "NOTICE", 2: "WARNING",
             3: "ALARM", 4: "INTERVENTION"}

    def __init__(self, cfg: Config, hard_only=False):
        self.cfg = cfg
        self.hard_only = hard_only       # trend evidence may not alert at all
        self.level = self.NORMAL
        self.reason = "startup"
        self.level_since = time.monotonic()
        self._at_least = {}
        self._below_since = None
        self._micro = deque()
        self._last_micro = None
        self._noface_since = None
        self.interventions = 0
        self.alarms = 0
        self.log = deque(maxlen=200)

    # ---- evidence -----------------------------------------------------------
    def note_microsleep(self, now):
        self._micro.append(now)
        self._last_micro = now

    def micro_count(self):
        return len(self._micro)

    def _evidence(self, now, ev: Evidence):
        """
        Map one frame onto (level, is_hard, why).

        Returns the level the CURRENT EVIDENCE justifies. It is not the level
        the machine adopts -- debounce, min hold and confirmed recovery all
        still stand between this and reality.
        """
        c = self.cfg

        # ---- the lid signal is not to be believed --------------------------
        # A stuck signal must not command an actuator, and equally must not
        # be silent: the driver is now unmonitored and has a right to know.
        if ev.sensor_fault:
            return self.WARNING, False, "eye signal stuck -- SENSOR FAULT"

        n = self.micro_count()
        # Repetition asserts a level only for a short while after the most
        # recent lapse. See the note on micro_sustain_s in Config.
        fresh = (self._last_micro is not None
                 and (now - self._last_micro) <= c.micro_sustain_s)

        # ---- HARD channel: measured eye closure, happening right now -------
        # Three tiers, three very different consequences. 2 s of shut eyes is
        # loud, not expensive; only 3.5 s is expensive.
        if ev.closure_s >= c.deep_sleep_s:
            return self.INTERVENE, True, f"eyes shut {ev.closure_s:.1f}s (unrousable)"
        if ev.closure_s >= c.sleep_s:
            return self.ALARM, True, f"eyes shut {ev.closure_s:.1f}s"
        if ev.closure_s >= c.microsleep_warn_s:
            return self.WARNING, True, f"microsleep {ev.closure_s:.2f}s"
        if ev.closure_s >= c.microsleep_s:
            # A first short microsleep is real but not yet worth a sound. It
            # is safe to stay quiet only BECAUSE the repetition channel below
            # is counting.
            if n >= c.micro_warn_n:
                return self.WARNING, True, f"microsleep {ev.closure_s:.2f}s (#{n})"
            return self.NOTICE, True, f"microsleep {ev.closure_s:.2f}s"

        # ---- REPETITION channel: many short lapses are their own emergency -
        if fresh:
            if n >= c.micro_intervene_n:
                return (self.INTERVENE, True,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")
            if n >= c.micro_alarm_n:
                return (self.ALARM, True,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")
            if n >= c.micro_warn_n:
                return (self.WARNING, False,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")

        # ---- TREND channel: inference, capped, WARNING is its ceiling ------
        if self.hard_only:
            return self.NORMAL, False, ""
        thr = c.risk_warn_exit if self.level >= self.WARNING else c.risk_warn
        if ev.trend_risk >= thr:
            why = ev.reasons[0] if ev.reasons else f"fatigue trend {ev.trend_risk:.0f}"
            return self.WARNING, False, why
        if ev.trend_risk >= c.risk_notice:
            why = ev.reasons[0] if ev.reasons else f"fatigue trend {ev.trend_risk:.0f}"
            return self.NOTICE, False, why
        return self.NORMAL, False, ""

    # ---- transitions --------------------------------------------------------
    def _transition(self, new, now, why):
        old = self.level
        self.level = new
        self.level_since = now
        self.reason = why
        self._below_since = None
        if new > old:
            if new == self.ALARM:
                self.alarms += 1
            if new == self.INTERVENE:
                self.interventions += 1
        if new < self.ALARM and old >= self.ALARM:
            self._last_micro = None
            # Fresh slate. Without this, the microsleeps that justified the
            # alarm immediately justify the next one and the machine never
            # leaves. A working counter is only safe if something clears it.
            self._micro.clear()
        self.log.append((time.strftime("%H:%M:%S"),
                         f"{self.NAMES[old]} -> {self.NAMES[new]}  ({why})"))
        return self.level, why

    def update(self, now, ev: Evidence):
        """Feed one frame. Returns (level, reason)."""
        c = self.cfg

        # ---- 1. no face: freeze an active alarm rather than dropping it -----
        if not ev.face:
            if self._noface_since is None:
                self._noface_since = now
            if self.level >= self.ALARM and \
               (now - self._noface_since) < c.face_loss_hold_s:
                return self.level, "face lost mid-alarm -- level held"
            raw, hard, why = self.NORMAL, False, "driver not visible"
        else:
            self._noface_since = None
            if ev.new_microsleep:
                self.note_microsleep(now)
            while self._micro and now - self._micro[0] > c.micro_window_s:
                self._micro.popleft()
            raw, hard, why = self._evidence(now, ev)

        # ---- 2. how long has evidence been AT LEAST each level? -------------
        for L in (1, 2, 3, 4):
            if raw >= L:
                self._at_least.setdefault(L, now)
            else:
                self._at_least.pop(L, None)

        # ---- 3. escalate once the confirm time for that level has passed ----
        needs = {
            self.NOTICE: 0.0,
            self.WARNING: c.warn_confirm_hard_s if hard else c.warn_confirm_s,
            self.ALARM: c.alarm_confirm_hard_s if hard else c.alarm_confirm_s,
            self.INTERVENE: c.intervene_confirm_s,
        }
        cand, cand_why = self.level, self.reason
        for L in (1, 2, 3, 4):
            t0 = self._at_least.get(L)
            if t0 is not None and (now - t0) >= needs[L] and L > cand:
                cand, cand_why = L, why or f"{self.NAMES[L]} evidence held"

        # ---- 4. the consequence route into INTERVENTION ---------------------
        # Not an evidence level: a level you reach by IGNORING one. The
        # observable sign that a driver has NOT responded to a sounding alarm
        # is that their eyes are still shut. Without that gate this fires on
        # any alarm that merely outlasts the timer, including one the driver
        # already woke up from -- punishing someone for having been asleep
        # rather than for being asleep.
        # Note `raw`, not `cand`: cand is seeded with the CURRENT level, so
        # testing it would read "still at ALARM" even after the evidence had
        # collapsed, and an alarm could escalate itself on nothing.
        if self.level == self.ALARM and raw >= self.ALARM and not ev.eyes_open:
            held = now - self.level_since
            if held >= c.intervene_after_s:
                cand, cand_why = self.INTERVENE, f"alarm unheeded for {held:.1f}s"

        if cand > self.level:
            return self._transition(cand, now, cand_why)

        # ---- 5. de-escalate: min hold + confirmed recovery ------------------
        # From ALARM or above the evidence must fall to WARNING or below, not
        # merely one level, otherwise the machine oscillates 4-3-4-3 and
        # re-cuts the speed every few seconds.
        drop_below = self.WARNING if self.level >= self.ALARM else self.level - 1
        eyes_ok = ev.eyes_open or not ev.face or ev.sensor_fault
        recovered = raw <= drop_below and (eyes_ok or self.level < self.ALARM)

        if recovered and self.level > self.NORMAL:
            if self._below_since is None:
                self._below_since = now
            hold = {self.INTERVENE: c.intervene_min_hold_s,
                    self.ALARM: c.alarm_min_hold_s}.get(self.level, 0.0)
            clear = {self.INTERVENE: c.intervene_clear_s,
                     self.ALARM: c.alarm_clear_s}.get(self.level, 0.4)
            if (now - self.level_since) >= hold and \
               (now - self._below_since) >= clear:
                return self._transition(raw, now, "driver recovered")
        else:
            self._below_since = None

        return self.level, self.reason

    def status_line(self, now):
        held = now - self.level_since
        s = f"{self.NAMES[self.level]} {held:.0f}s"
        if self.level == self.ALARM:
            left = max(0.0, self.cfg.intervene_after_s - held)
            s += f" (intervenes in {left:.1f}s)"
        elif self.level == self.INTERVENE:
            left = max(0.0, self.cfg.intervene_min_hold_s - held)
            if left > 0:
                s += f" (min hold {left:.1f}s)"
        if self.micro_count():
            s += f" [{self.micro_count()} ms/{self.cfg.micro_window_s:.0f}s]"
        return s


def explain_ladder(c: Config, hard_only=False):
    """Print exactly what it takes to reach each level. Use --explain."""
    L = [
        ("NOTICE", "console only", [
            f"trend risk >= {c.risk_notice:.0f}",
            f"one microsleep {c.microsleep_s:.2f}-{c.microsleep_warn_s:.2f}s",
        ]),
        ("WARNING", f"one {c.warn_on_s:.2f}s beep "
                    f"(min {c.alarm_cooldown_s:.0f}s apart)", [
            f"trend risk >= {c.risk_warn:.0f} (leaves below {c.risk_warn_exit:.0f})",
            f"eyes shut >= {c.microsleep_warn_s:.2f}s",
            f"{c.micro_warn_n} microsleeps in {c.micro_window_s:.0f}s",
            "vacant stare / eyes stuck",
            "the lid signal is stuck (sensor fault)",
        ]),
        ("ALARM", "repeating buzzer", [
            f"eyes shut >= {c.sleep_s:.2f}s",
            f"{c.micro_alarm_n} microsleeps in {c.micro_window_s:.0f}s",
            f"held at least {c.alarm_min_hold_s:.1f}s once entered",
        ]),
        ("INTERVENTION", "buzzer + speed byte on the serial link", [
            f"eyes shut >= {c.deep_sleep_s:.2f}s",
            f"an ALARM left unheeded {c.intervene_after_s:.1f}s",
            f"{c.micro_intervene_n} microsleeps in {c.micro_window_s:.0f}s",
            f"held at least {c.intervene_min_hold_s:.1f}s once entered",
        ]),
    ]
    print("\nescalation ladder -- what it takes to reach each level")
    print("=" * 66)
    for name, effect, routes in L:
        print(f"  {name:<13} {effect}")
        for r in routes:
            print(f"        - {r}")
    print(f"\n  trend evidence is capped at {c.trend_cap:.0f} and can never, on its "
          f"own,\n  reach ALARM or INTERVENTION -- only measured eye closure "
          f"and\n  counted repetition can.")
    print(f"\n  an unbroken closure past {c.latch_fault_s:.0f}s with the head "
          f"still moving is\n  treated as a stuck signal, not sleep: it drops "
          f"to WARNING and\n  releases the speed byte.")
    print(f"\n  the LED is NOT part of this ladder. It tracks face presence "
          f"only:\n  on when the camera can see a driver, off "
          f"{c.led_hold_s:.2f}s after it cannot.")
    if hard_only:
        print("  --hard-only is set: trend evidence cannot alert at all.")
    print("=" * 66 + "\n")


class Alerter:
    """
    Console reporting + actuator dispatch.

    Note what is NOT in here: no thresholds, no status-string tests. Every
    judgement about severity already happened in SafetyPolicy. This class
    only knows how to make a noise, light a lamp, and write a byte. Keeping
    it that dumb is what makes the policy auditable.

    Call apply() every loop iteration, including at NORMAL, so held patterns,
    the LED and the speed byte all track the real state.
    """

    def __init__(self, cooldown_s, buzzer, quiet=False, noface_delay_s=2.0,
                 noface_enabled=True, speed=None,
                 intervene_speed=30, caution_speed=None,
                 led=None, led_hold_s=0.5, led_reassert_s=2.0):
        self.cooldown = cooldown_s
        self.buzzer = buzzer
        self.speed = speed
        self.quiet = quiet
        self.noface_delay = noface_delay_s
        self.noface_enabled = noface_enabled
        self.intervene_speed = intervene_speed
        self.caution_speed = caution_speed
        self.last_warn = 0.0
        self.fired = 0
        self.beeps = 0
        self._alarm_active = False
        self._warned_at_level = False
        self._noface_since = None
        self._noface_active = False
        self._face_seen = False

        self.led = led
        self.led_hold_s = led_hold_s
        self.led_reassert_s = led_reassert_s
        self._led_off_at = None
        self._led_forced = 0.0

    def _say(self, text):
        if not self.quiet:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def _note_face_present(self):
        self._noface_since = None
        if self._noface_active:
            self._noface_active = False
            self._say("[ALERT] driver visible again")
        if not self._face_seen:
            self._face_seen = True
            self._say("[ALERT] face acquired  (LED on)")

    def _handle_noface(self):
        now = time.monotonic()
        if self._noface_since is None:
            self._noface_since = now
        if not self._noface_active and now - self._noface_since >= self.noface_delay:
            self._noface_active = True
            self._face_seen = False        # next face re-logs the acquisition
            if self.noface_enabled:
                self._say(f"[ALERT/no-face] driver not visible for "
                          f"{self.noface_delay:.0f}s  (silent)")

    def _update_led(self, face_present):
        """
        The LED follows face presence and nothing else.

        Two deliberate choices.

        1. THE HOLD. MediaPipe drops the occasional frame on a face that has
           not moved at all. Driving the LED straight off ev.face renders
           each of those as a visible flick, and a lamp that flickers for no
           reason is a lamp the driver stops reading. Note the hold is
           SHORTER than noface_delay_s: the console message is about a driver
           who has genuinely left, the LED is about whether the camera has a
           face this instant.

        2. THE PERIODIC FORCE. GpioLine caches its state and skips redundant
           writes. If one write fails silently the cache and the pin disagree
           forever, and the LED would sit dark with a face in frame. Forcing
           a real write every led_reassert_s bounds that to a couple of
           seconds.
        """
        if self.led is None:
            return
        now = time.monotonic()
        if face_present:
            self._led_off_at = None
            want = True
        else:
            if self._led_off_at is None:
                self._led_off_at = now + self.led_hold_s
            want = now < self._led_off_at

        force = (now - self._led_forced) >= self.led_reassert_s
        if force:
            self._led_forced = now
        self.led.set(want, force=force)

    def apply(self, level, ev: Evidence, why=""):
        if not ev.face:
            self._handle_noface()
        else:
            self._note_face_present()
        self._update_led(ev.face)

        # ---- buzzer --------------------------------------------------------
        if level >= SafetyPolicy.ALARM:
            self._warned_at_level = False
            if not self._alarm_active:
                self._alarm_active = True
                self.fired += 1
                self._say(f"\a[ALERT/{SafetyPolicy.NAMES[level].lower()}] "
                          f"{ev.status}  {why}  (buzzer ON)")
            if self.buzzer:
                self.buzzer.set_critical(True)
        else:
            if self._alarm_active:
                self._alarm_active = False
                self._say("[ALERT] alarm cleared  (buzzer OFF)")
            if self.buzzer:
                self.buzzer.set_critical(False)

            if level == SafetyPolicy.WARNING:
                # One beep per ENTRY into WARNING, and never closer together
                # than the cooldown. Re-arming on every frame that happens to
                # land at WARNING is what makes a device sound like a smoke
                # detector through one drowsy stretch.
                now = time.monotonic()
                if not self._warned_at_level and now - self.last_warn >= self.cooldown:
                    self.last_warn = now
                    self._warned_at_level = True
                    self.fired += 1
                    self.beeps += 1
                    self._say(f"[ALERT/warn] {ev.status}  {why}  (beep)")
                    if self.buzzer:
                        self.buzzer.pulse_warn()
            else:
                self._warned_at_level = False

        # ---- serial speed command -----------------------------------------
        if self.speed:
            if level >= SafetyPolicy.INTERVENE:
                self.speed.set_target(self.intervene_speed)
            elif level == SafetyPolicy.ALARM and self.caution_speed is not None:
                self.speed.set_target(self.caution_speed)
            else:
                self.speed.set_target(None)

    def shutdown(self):
        if self.buzzer:
            self.buzzer.set_critical(False)
        if self.led:
            self.led.set(False, force=True)
        if self.speed:
            self.speed.set_target(None)


# =============================================================================
#  5. MODEL + MATH HELPERS
# =============================================================================
def ensure_model():
    if os.path.exists(MODEL_FILE):
        return MODEL_FILE
    print("Downloading face_landmarker.task (~3.7 MB) ...")
    try:
        # A timeout matters here: urlretrieve has none by default, so a
        # captive portal or half-open Wi-Fi hangs startup silently forever.
        with urllib.request.urlopen(MODEL_URL, timeout=20) as r, \
                open(MODEL_FILE + ".part", "wb") as fh:
            fh.write(r.read())
        os.replace(MODEL_FILE + ".part", MODEL_FILE)
        print("Done.")
    except Exception as e:
        try:
            os.remove(MODEL_FILE + ".part")
        except OSError:
            pass
        sys.exit(f"Could not download the model ({e}).\n"
                 f"Fetch it manually from:\n  {MODEL_URL}\n"
                 f"and save it as:\n  {MODEL_FILE}")
    return MODEL_FILE


def matrix_to_euler(mat4):
    """4x4 facial transformation matrix -> (pitch, yaw, roll) in degrees."""
    R = np.array(mat4)[:3, :3]
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        pitch = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[1, 0], R[0, 0])
    else:
        pitch = math.atan2(-R[1, 2], R[1, 1])
        yaw = math.atan2(-R[2, 0], sy)
        roll = 0.0
    return math.degrees(pitch), math.degrees(yaw), math.degrees(roll)


# =============================================================================
#  6. CAMERA  --  THREADED, NEWEST FRAME WINS
# =============================================================================
class BaseGrabber:
    """
    Reads the camera as fast as it will go and keeps ONLY the most recent
    frame. The detector never waits on I/O and never processes a stale frame.

    Dropped frames are a feature, not a fault: a drowsiness decision made on
    a 400 ms old image is worse than no decision at all.

    Subclasses supply _open_device / _read_frame / _close_device. Everything
    else -- the thread, the newest-frame slot, restart, and the shutdown
    ordering -- lives here so both backends behave identically.
    """

    name = "camera"

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._seq = 0
        self._stop = threading.Event()
        self._thread = None
        self.dropped = 0
        self.read_errors = 0
        self.restarts = 0
        self.actual = (0, 0, 0.0)

    # ---- subclass hooks ----------------------------------------------------
    def _open_device(self):
        raise NotImplementedError

    def _read_frame(self):
        """Return a BGR ndarray, or None on failure."""
        raise NotImplementedError

    def _close_device(self):
        raise NotImplementedError

    # ---- lifecycle ---------------------------------------------------------
    def start(self):
        self._open_device()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                frame = self._read_frame()
            except Exception as e:
                frame = None
                self.read_errors += 1
                if self.read_errors <= 3:
                    sys.stderr.write(f"{self.name} read error: {e}\n")
            if frame is None:
                time.sleep(0.02)
                continue
            t = time.monotonic()
            with self._lock:
                if self._frame is not None:
                    self.dropped += 1
                self._frame = frame
                self._stamp = t
                self._seq += 1

    def latest(self, timeout=1.0):
        """Return (frame, capture_time, seq) or None. Consumes the frame."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frame is not None:
                    f, t, s = self._frame, self._stamp, self._seq
                    self._frame = None
                    return f, t, s
            time.sleep(0.003)
        return None

    def restart(self):
        """
        Tear the device down and bring it back.

        A USB brown-out or a CSI link reset leaves the capture object alive
        and useless: it keeps returning failures forever. Reopening is the
        only recovery, and doing it here means the detection loop never has
        to know the camera went away.
        """
        self.restarts += 1
        sys.stderr.write(f"{self.name}: restarting (attempt {self.restarts})\n")
        self._teardown(join_timeout=2.0)
        time.sleep(0.5)
        try:
            self.start()
            sys.stderr.write(f"{self.name}: back up\n")
            return True
        except Exception as e:
            sys.stderr.write(f"{self.name}: restart failed ({e})\n")
            return False

    def _teardown(self, join_timeout=2.0):
        self._stop.set()
        th = self._thread
        self._thread = None
        if th is not None:
            th.join(timeout=join_timeout)
            if th.is_alive():
                # Releasing the device out from under a thread still inside a
                # blocking read is a segfault in the V4L2 backend. Leaking the
                # handle is the better of the two outcomes.
                sys.stderr.write(f"{self.name}: reader thread did not exit; "
                                 f"leaving the device open to avoid a crash\n")
                return
        try:
            self._close_device()
        except Exception as e:
            sys.stderr.write(f"{self.name} close: {e}\n")
        with self._lock:
            self._frame = None

    def release(self):
        self._teardown(join_timeout=2.0)


class Picamera2Grabber(BaseGrabber):
    """
    The OV5647 (and every other CSI sensor) on Bookworm goes through
    libcamera, not V4L2. cv2.VideoCapture either fails outright or hands you
    the raw Bayer node, which is why this backend exists.

    Format note that costs people an afternoon: picamera2's format names
    describe BYTE ORDER IN MEMORY, which is the reverse of the numpy channel
    order. "RGB888" therefore gives you a BGR array -- exactly what OpenCV
    expects. If faces are not being detected at all, the red and blue
    channels are probably swapped for your build: pass --swap-rb.
    """

    name = "picamera2"

    def __init__(self, width, height, fps, camera_num=0, swap_rb=False,
                 vflip=False, hflip=False):
        super().__init__()
        self.width, self.height, self.fps = width, height, fps
        self.camera_num = camera_num
        self.swap_rb = swap_rb
        self.vflip, self.hflip = vflip, hflip
        self._picam = None

    def _open_device(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            raise RuntimeError(
                "picamera2 is not importable.\n"
                "  sudo apt install -y python3-picamera2\n"
                "and create the venv with --system-site-packages, or run with "
                "--backend v4l2 if this is a USB webcam.")
        from libcamera import Transform

        picam = Picamera2(self.camera_num)
        cfg = picam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            controls={"FrameRate": float(self.fps)},
            transform=Transform(vflip=1 if self.vflip else 0,
                                hflip=1 if self.hflip else 0),
            buffer_count=4,
        )
        picam.configure(cfg)
        picam.start()
        time.sleep(0.5)              # let AE/AWB settle before the first frame
        self._picam = picam
        cfgm = picam.camera_configuration()["main"]
        self.actual = (cfgm["size"][0], cfgm["size"][1], float(self.fps))

    def _read_frame(self):
        arr = self._picam.capture_array("main")
        if arr is None:
            return None
        if arr.ndim == 3 and arr.shape[2] == 4:
            arr = arr[:, :, :3]
        if self.swap_rb:
            arr = arr[:, :, ::-1].copy()
        return arr

    def _close_device(self):
        if self._picam is not None:
            try:
                self._picam.stop()
            finally:
                try:
                    self._picam.close()
                except Exception:
                    pass
                self._picam = None


class V4L2Grabber(BaseGrabber):
    """USB UVC webcams, and video files for offline testing."""

    name = "v4l2"

    def __init__(self, source, width, height, fourcc="MJPG"):
        super().__init__()
        self.source = source
        self.width, self.height, self.fourcc = width, height, fourcc
        self.cap = None

    def _open_device(self):
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            raise RuntimeError(f"could not open video source: {self.source}")

        if self.fourcc:
            try:
                fcc = cv2.VideoWriter_fourcc(*self.fourcc)
            except AttributeError:                    # OpenCV >= 4.11
                fcc = cv2.VideoWriter.fourcc(*self.fourcc)
            cap.set(cv2.CAP_PROP_FOURCC, fcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self.cap = cap
        self.actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                       int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                       float(cap.get(cv2.CAP_PROP_FPS)))

    def _read_frame(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def _close_device(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def open_camera(args):
    """Pick a backend and bring it up. Raises on failure."""
    backend = args.backend
    if backend == "auto":
        backend = "v4l2" if args.video else "picamera2"

    if args.video:
        grab = V4L2Grabber(args.video, args.width, args.height, None)
    elif backend == "picamera2":
        grab = Picamera2Grabber(args.width, args.height, args.fps,
                                camera_num=args.camera_num,
                                swap_rb=args.swap_rb,
                                vflip=args.vflip, hflip=args.hflip)
    else:
        src = int(args.camera) if str(args.camera).isdigit() else args.camera
        grab = V4L2Grabber(src, args.width, args.height, args.fourcc or None)

    grab.start()
    return grab


def camera_selftest(args, seconds=4.0):
    """
    Prove the camera before trusting anything downstream: resolution, real
    frame rate, mean brightness, and whether the channel order looks sane.
    """
    grab = open_camera(args)
    try:
        print(f"camera: {grab.name}, configured "
              f"{grab.actual[0]}x{grab.actual[1]}")
        t0 = time.monotonic()
        n, mean = 0, 0.0
        last = None
        while time.monotonic() - t0 < seconds:
            got = grab.latest(timeout=2.0)
            if got is None:
                print("  no frame within 2s")
                continue
            last = got[0]
            n += 1
            mean = float(last.mean())
        el = time.monotonic() - t0
        print(f"  {n} frames in {el:.1f}s  =  {n/max(el,1e-6):.1f} fps")
        if last is not None:
            b, g, r = [float(last[:, :, i].mean()) for i in range(3)]
            print(f"  frame {last.shape[1]}x{last.shape[0]}, "
                  f"mean level {mean:.0f}/255  (B {b:.0f}  G {g:.0f}  R {r:.0f})")
            if mean < 12:
                print("  WARNING: nearly black. Lens cap, ribbon seated the "
                      "wrong way round, or no light.")
            print("  channel order: a face should read R > B under normal "
                  "indoor light.\n"
                  "  If detection never fires, try --swap-rb.")
        if grab.read_errors:
            print(f"  WARNING: {grab.read_errors} read error(s)")
    finally:
        grab.release()


# =============================================================================
#  7. EXPRESSION FEATURE EXTRACTION
# =============================================================================
ENERGY_KEYS = [
    "browDownLeft", "browDownRight", "browInnerUp",
    "browOuterUpLeft", "browOuterUpRight",
    "cheekSquintLeft", "cheekSquintRight", "cheekPuff",
    "eyeSquintLeft", "eyeSquintRight", "eyeWideLeft", "eyeWideRight",
    "jawOpen", "jawForward",
    "mouthSmileLeft", "mouthSmileRight", "mouthFrownLeft", "mouthFrownRight",
    "mouthPucker", "mouthStretchLeft", "mouthStretchRight",
    "mouthPressLeft", "mouthPressRight", "mouthDimpleLeft", "mouthDimpleRight",
    "noseSneerLeft", "noseSneerRight",
]


@dataclass
class Features:
    t: float = 0.0
    eye_closed: float = 0.0
    squint: float = 0.0
    brow_down: float = 0.0
    brow_inner_up: float = 0.0
    eye_wide: float = 0.0
    jaw_open: float = 0.0
    mouth_stretch: float = 0.0
    energy: float = 0.0
    gaze_x: float = 0.0
    gaze_y: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    top_emotion: str = "neutral"
    top_emotion_score: float = 0.0


def extract_features(blendshapes, matrix, t):
    bs = {c.category_name: c.score for c in blendshapes}
    g = bs.get

    f = Features(t=t)
    f.eye_closed = (g("eyeBlinkLeft", 0.0) + g("eyeBlinkRight", 0.0)) / 2.0
    f.squint = (g("eyeSquintLeft", 0.0) + g("eyeSquintRight", 0.0)) / 2.0
    f.brow_down = (g("browDownLeft", 0.0) + g("browDownRight", 0.0)) / 2.0
    f.brow_inner_up = g("browInnerUp", 0.0)
    f.eye_wide = (g("eyeWideLeft", 0.0) + g("eyeWideRight", 0.0)) / 2.0
    f.jaw_open = g("jawOpen", 0.0)
    f.mouth_stretch = (g("mouthStretchLeft", 0.0) + g("mouthStretchRight", 0.0)) / 2.0
    f.energy = float(sum(g(k, 0.0) for k in ENERGY_KEYS))

    look_r = (g("eyeLookOutLeft", 0.0) + g("eyeLookInRight", 0.0)) / 2.0
    look_l = (g("eyeLookInLeft", 0.0) + g("eyeLookOutRight", 0.0)) / 2.0
    look_u = (g("eyeLookUpLeft", 0.0) + g("eyeLookUpRight", 0.0)) / 2.0
    look_d = (g("eyeLookDownLeft", 0.0) + g("eyeLookDownRight", 0.0)) / 2.0
    f.gaze_x = look_r - look_l
    f.gaze_y = look_u - look_d

    if matrix is not None:
        f.pitch, f.yaw, f.roll = matrix_to_euler(matrix)

    ignore = ("_neutral", "eyeLook", "eyeBlink")
    cand = [(v, k) for k, v in bs.items() if not k.startswith(ignore)]
    if cand:
        score, name = max(cand)
        f.top_emotion, f.top_emotion_score = name, score
    return f


# =============================================================================
#  8. CALIBRATION
# =============================================================================
class Calibrator:
    def __init__(self, seconds, min_samples=25):
        self.seconds = seconds
        self.min_samples = min_samples
        self.reset()

    def reset(self):
        self.samples = []
        self.all = []
        self.t0 = None
        self.done = False
        self.base = dict(squint=0.0, brow_down=0.0, brow_inner_up=0.0,
                         energy=1.0, pitch=0.0, yaw=0.0,
                         gaze_x=0.0, gaze_y=0.0,
                         gaze_noise=0.012, head_noise=0.60,
                         eye_open=0.05, eye_open_hi=0.20)

    def feed(self, f):
        """
        Collect EVERY frame, not just ones under a hardcoded 0.4.

        The old filter had two faults. It assumed the answer: a driver whose
        open eyes read 0.45 on this blendshape -- common with small eyes,
        glasses, or a camera below the eyeline -- contributed almost no
        samples, so calibration either stalled or learned from the
        unrepresentative low tail. And it made the open-eye LEVEL unknowable,
        which is exactly the number needed to set close_on for this face.

        Collect everything, then SEPARATE. A driver blinks maybe 5-10% of the
        time, so the bulk of any window is open eyes by definition and the
        median is a robust estimate without assuming where open lives.

        The sample-count floor matters as much as the clock: if the driver
        looks away for a minute and glances back, wall-clock alone would
        "calibrate" the whole session from eleven scattered frames.
        """
        if self.done:
            return
        if self.t0 is None:
            self.t0 = f.t
        self.all.append(f)
        if f.t - self.t0 >= self.seconds and len(self.all) >= self.min_samples:
            self._finish()

    def _finish(self):
        eyes = np.array([s.eye_closed for s in self.all], dtype=np.float32)
        eye_open = float(np.median(eyes))
        # The open cluster is everything not far above the median. Its upper
        # tail is where this driver's OPEN eyes actually reach.
        open_cluster = eyes[eyes <= eye_open + 0.12]
        eye_open_hi = float(np.percentile(open_cluster, 95)) \
            if open_cluster.size >= 5 else eye_open + 0.10

        # Expression baselines still exclude blink frames -- but using the
        # threshold we just LEARNED, not a guessed constant.
        self.samples = [s for s in self.all if s.eye_closed <= eye_open_hi]
        if len(self.samples) < 5:
            self.samples = list(self.all)

        def med(attr):
            return float(np.median([getattr(s, attr) for s in self.samples]))
        self.base = dict(
            squint=med("squint"),
            brow_down=med("brow_down"),
            brow_inner_up=med("brow_inner_up"),
            energy=max(med("energy"), 0.25),
            pitch=med("pitch"),
            yaw=med("yaw"),
            gaze_x=med("gaze_x"),
            gaze_y=med("gaze_y"),
        )
        gx = np.array([s.gaze_x for s in self.samples], dtype=np.float32)
        gy = np.array([s.gaze_y for s in self.samples], dtype=np.float32)
        yw = np.array([s.yaw for s in self.samples], dtype=np.float32)
        self.base["gaze_noise"] = float(min(max(float(math.hypot(gx.std(), gy.std())),
                                                0.004), 0.050))
        self.base["head_noise"] = float(min(max(float(yw.std()), 0.15), 2.50))
        self.base["eye_open"] = eye_open
        self.base["eye_open_hi"] = eye_open_hi
        self.done = True
        self.all = []            # keep only what the rest of the run needs

    def progress(self, t):
        if self.done or self.t0 is None:
            return 1.0
        by_time = (t - self.t0) / self.seconds
        by_count = len(self.all) / max(1, self.min_samples)
        return min(1.0, min(by_time, by_count))


def fit_eye_thresholds(cfg: Config, eye_open_hi: float, allow_lower=False):
    """
    Move close_on/close_off to suit THIS driver, and explain the move.

    The failure this prevents: close_on = 0.55 with close_off = 0.35 is a
    latch. If a driver's open eyes sit at 0.45, one blink pushes the value
    over 0.55 and it never returns below 0.35 -- so the closure timer runs
    forever and the machine reports SLEEPING at a wide-awake person. The only
    defence is to place close_on above where THIS driver's open eyes actually
    live, and to keep close_off far enough below it that an ordinary blink
    always releases the latch.

    Raising is safe: it makes the system less trigger-happy. Lowering makes
    it MORE sensitive than the operator asked for, so it is opt-in.
    """
    want_on = min(0.90, max(0.45, eye_open_hi + 0.18))
    if want_on <= cfg.close_on and not allow_lower:
        return None
    if abs(want_on - cfg.close_on) < 0.03:
        return None
    old_on, old_off = cfg.close_on, cfg.close_off
    cfg.close_on = round(want_on, 2)
    # close_off must sit ABOVE where this driver's open eyes reach, or the
    # latch can never release: the signal would have to fall below its own
    # resting level to count as open again.
    cfg.close_off = round(max(0.15, min(cfg.close_on - 0.10,
                                        eye_open_hi + 0.05)), 2)
    return (f"close_on: {old_on:.2f} -> {cfg.close_on:.2f}, "
            f"close_off: {old_off:.2f} -> {cfg.close_off:.2f}  "
            f"(this driver's open eyes reach {eye_open_hi:.2f})")


# =============================================================================
#  9. THE DETECTOR
# =============================================================================
@dataclass
class State:
    status: str = "STARTING"
    risk: float = 0.0            # display risk = max(trend, hard)
    trend_risk: float = 0.0      # what the policy actually sees
    reasons: list = field(default_factory=list)
    perclos: float = 0.0
    blink_rate: float = 0.0
    yawns: int = 0
    closure_s: float = 0.0
    stare_s: float = 0.0
    flatness: float = 1.0
    microsleeps: int = 0
    sleeps: int = 0
    stares: int = 0
    faults: int = 0


class MicrosleepDetector:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.cal = Calibrator(cfg.calib_seconds, cfg.calib_min_samples)

        self.closure = TimedBuffer(cfg.perclos_window_s)
        self.squint_b = TimedBuffer(cfg.drowsy_window_s)
        self.brow_b = TimedBuffer(cfg.drowsy_window_s)
        self.inner_b = TimedBuffer(cfg.drowsy_window_s)
        self.energy_b = TimedBuffer(cfg.drowsy_window_s)
        self.gx_b = TimedBuffer(cfg.stare_window_s)
        self.gy_b = TimedBuffer(cfg.stare_window_s)
        self.yaw_b = TimedBuffer(cfg.stare_window_s)
        self.pitch_b = TimedBuffer(cfg.nod_window_s)

        self.eyes_shut = False
        self.shut_start = 0.0
        self.shut_frames = 0
        self._micro_latched = False       # counted this closure already?
        self._sleep_latched = False
        self.sensor_fault = False
        self.fault_count = 0
        self.last_blink_t = 0.0
        self.blink_times = deque()
        self.yawning = False
        self.yawn_start = 0.0
        self.yawn_times = deque()
        self.frozen_since = None
        self.frozen_frames = 0
        self._stare_latched = False
        self.distract_since = None
        self._nod_latched_until = 0.0

        self.trend_risk = 0.0
        self.last_t = None
        self.state = State()
        self.microsleep_count = 0
        self.sleep_count = 0
        self.stare_count = 0
        self.event_log = deque(maxlen=200)
        self.tuned = False
        self.dbg = {}

    def retune(self, fps, fit_eyes=True, allow_lower=False):
        """Called once after calibration, when the real fps is known."""
        if self.tuned:
            return []
        self.tuned = True
        changes = []
        if fit_eyes:
            msg = fit_eye_thresholds(self.cfg, self.cal.base["eye_open_hi"],
                                     allow_lower)
            if msg:
                changes.append(msg)
        changes += autotune(self.cfg, fps)
        self.gx_b.window_s = self.cfg.stare_window_s
        self.gy_b.window_s = self.cfg.stare_window_s
        self.yaw_b.window_s = self.cfg.stare_window_s
        self.pitch_b.window_s = self.cfg.nod_window_s
        return changes

    # -- HARD channel ---------------------------------------------------------
    def _lid_closure(self, f):
        """
        Track one continuous eye closure and count it exactly once.

        A LATCH is the right primitive for "do this once per event": set it
        when the event is first recognised, clear it when the event ends. No
        arithmetic, no frame-rate coupling, no way to miss the edge. The
        earlier `shut_frames == min_event_frames` equality could never be
        true at any frame rate where min_event_frames / fps < microsleep_s,
        which silently killed the whole repeated-microsleep route.

        Returns (closure_s, new_microsleep).
        """
        c = self.cfg
        new_micro = False

        if not self.eyes_shut:
            if f.eye_closed > c.close_on:
                self.eyes_shut = True
                self.shut_start = f.t
                self.shut_frames = 1
                self._micro_latched = False
                self._sleep_latched = False
        else:
            if f.eye_closed >= c.close_off:
                self.shut_frames += 1
            else:
                dur = f.t - self.shut_start
                self.eyes_shut = False
                if dur <= c.blink_max_s:
                    self.blink_times.append(f.t)
                    self.last_blink_t = f.t
                self.shut_start = 0.0
                self.shut_frames = 0
                if self.sensor_fault:
                    self.sensor_fault = False
                    self._event("lid signal released -- sensor fault cleared")
                    sys.stderr.write("[DIAGNOSTIC] lid signal released; "
                                     "monitoring resumed\n")

        closure_s = (f.t - self.shut_start) if self.eyes_shut else 0.0

        # ---- STUCK-SIGNAL ESCAPE (fix [4]) ---------------------------------
        # Two things look identical from inside the closure timer: a driver
        # who has been unconscious for a minute, and a close_off that is too
        # high for this face. Suppressing the alarm on a guess would silence
        # the one case that matters, so the discriminator has to be evidence
        # from a DIFFERENT sensor channel: head motion.
        #
        # An unconscious head does not scan the road. If yaw is still moving
        # well above this driver's calibrated resting noise while the lid
        # signal has not released in 45 seconds, the signal is stuck. That
        # frees the throttle without ever going quiet -- the fault itself
        # raises a WARNING, so the driver is told they are unmonitored.
        if self.eyes_shut and not self.sensor_fault and \
           closure_s >= c.latch_fault_s and self.cal.done:
            head_std = self.yaw_b.std()
            floor = max(0.30, self.cal.base.get("head_noise", 0.6)
                        * c.latch_fault_motion_ratio)
            if head_std > floor:
                self.sensor_fault = True
                self.fault_count += 1
                hi = self.cal.base.get("eye_open_hi", 0.2)
                self._event(f"SENSOR FAULT: lid stuck {closure_s:.0f}s")
                sys.stderr.write(
                    f"\n[DIAGNOSTIC] the eye-closure timer has run "
                    f"{closure_s:.0f}s without releasing,\n"
                    f"  while the head is still moving "
                    f"({head_std:.2f} deg vs {floor:.2f} resting). That is a "
                    f"stuck signal,\n"
                    f"  not an unconscious driver. close_off "
                    f"({c.close_off:.2f}) is too high for this face:\n"
                    f"  calibration measured open eyes reaching {hi:.2f}.\n"
                    f"  Dropping to WARNING and RELEASING the speed byte. "
                    f"Restart with\n"
                    f"  --close-on / --close-off set for this driver.\n")
                sys.stderr.flush()

        if self.sensor_fault:
            # Report the signal as unusable rather than as a closure. The
            # policy has its own branch for this and caps the level at
            # WARNING; nothing here reaches an actuator.
            self.state.closure_s = 0.0
            return 0.0, False

        self.state.closure_s = closure_s

        if self.eyes_shut and self.shut_frames >= c.min_event_frames:
            if closure_s >= c.microsleep_s and not self._micro_latched:
                self._micro_latched = True
                self.microsleep_count += 1
                new_micro = True
                self._event(f"microsleep {closure_s:.2f}s ({self.shut_frames}f)")
            if closure_s >= c.sleep_s and not self._sleep_latched:
                self._sleep_latched = True
                self.sleep_count += 1
                self._event(f"SLEEP {closure_s:.1f}s")
        return closure_s, new_micro

    # -- TREND channel --------------------------------------------------------
    def _perclos(self, f, add):
        c = self.cfg
        p = self.closure.fraction_above(c.close_on)
        self.state.perclos = p
        if self.closure.span() < 12.0:
            return
        if p >= c.perclos_alarm:
            add(45.0, f"PERCLOS {p*100:.0f}%")
        elif p >= c.perclos_warn:
            add(22.0, f"PERCLOS {p*100:.0f}%")

    def _drowsy_face(self, f, add):
        c, b = self.cfg, self.cal.base
        if self.squint_b.span() < 3.0:
            return
        squint = self.squint_b.mean() - b["squint"]
        brow = self.brow_b.mean() - b["brow_down"]
        inner = self.inner_b.mean() - b["brow_inner_up"]
        flat = self.energy_b.mean() / b["energy"]
        self.state.flatness = flat

        score = 0.0
        if squint > c.squint_warn:
            score += 14.0
        if brow > c.browdown_warn:
            score += 12.0
        if inner > 0.18:
            score += 10.0
        if flat < c.flatness_warn:
            score += 14.0
        if score >= 20.0:
            add(score, f"drowsy face (squint{squint:+.2f} brow{brow:+.2f} "
                       f"flat{flat:.2f})")

    def _yawn(self, f, add):
        c = self.cfg
        if not self.yawning and f.jaw_open > c.yawn_on and f.mouth_stretch > 0.06:
            self.yawning, self.yawn_start = True, f.t
        elif self.yawning and f.jaw_open < c.yawn_on * 0.6:
            if f.t - self.yawn_start >= c.yawn_min_s:
                self.yawn_times.append(f.t)
                self._event("yawn")
            self.yawning = False

        while self.yawn_times and f.t - self.yawn_times[0] > c.yawn_window_s:
            self.yawn_times.popleft()
        self.state.yawns = len(self.yawn_times)
        if self.state.yawns >= c.yawns_warn:
            add(10.0 * self.state.yawns, f"{self.state.yawns} yawns / 2min")

    def _eyes_stuck(self, f, add):
        """
        Gaze freezing is REQUIRED; head-still / blink-suppression / flat-face
        are supporting evidence, any one of which is enough.
        """
        c, b = self.cfg, self.cal.base
        gaze_thr = max(c.gaze_frozen_floor, b["gaze_noise"] * c.gaze_freeze_ratio)
        head_thr = max(c.head_frozen_floor, b["head_noise"] * c.head_freeze_ratio)

        eyes_open = f.eye_closed < c.stare_eyes_open_max
        if not eyes_open or self.gx_b.span() < c.stare_window_s * 0.7:
            self.frozen_since = None
            self.frozen_frames = 0
            self._stare_latched = False
            self.state.stare_s = 0.0
            self.dbg["support"] = "eyes shut" if not eyes_open else "warming up"
            return False

        gaze_std = math.hypot(self.gx_b.std(), self.gy_b.std())
        head_std = self.yaw_b.std()
        since_blink = f.t - self.last_blink_t
        self.dbg.update(gaze_std=gaze_std, gaze_thr=gaze_thr,
                        head_std=head_std, head_thr=head_thr,
                        since_blink=since_blink)

        gaze_frozen = gaze_std < gaze_thr
        support, why = 0, []
        if head_std < head_thr:
            support += 1
            why.append("head still")
        if since_blink > c.no_blink_s:
            support += 1
            why.append(f"no blink {since_blink:.0f}s")
        if self.energy_b.span() > 2.0 and \
           self.energy_b.mean() / b["energy"] < c.stare_flat_ratio:
            support += 1
            why.append("face flat")
        self.dbg["support"] = ("gaze FROZEN | " if gaze_frozen else "gaze moving | ") + \
                              (", ".join(why) if why else "no support")

        if gaze_frozen and support >= 1:
            if self.frozen_since is None:
                self.frozen_since = f.t
                self.frozen_frames = 0
                self._stare_latched = False
            self.frozen_frames += 1
            dur = f.t - self.frozen_since
            self.state.stare_s = dur
            if dur >= c.stare_min_s and self.frozen_frames >= c.min_event_frames:
                add(55.0 + min(25.0, dur * 3.0),
                    f"EYES STUCK {dur:.1f}s ({', '.join(why)})")
                if not self._stare_latched:      # latch, same lesson as above
                    self._stare_latched = True
                    self.stare_count += 1
                    self._event(f"eyes stuck {dur:.1f}s")
                return True
        else:
            self.frozen_since = None
            self.frozen_frames = 0
            self._stare_latched = False
            self.state.stare_s = 0.0
        return False

    def _distraction(self, f, add):
        c, b = self.cfg, self.cal.base
        yaw = f.yaw - b["yaw"]
        pitch = f.pitch - b["pitch"]

        distracted = False
        if abs(yaw) > c.yaw_limit or abs(pitch) > c.pitch_limit:
            if self.distract_since is None:
                self.distract_since = f.t
            if f.t - self.distract_since >= c.distract_min_s:
                # Deliberately NOT scored: a driver checking a mirror is not
                # fatigued. Kept as a status label only.
                distracted = True
        else:
            self.distract_since = None

        pv = self.pitch_b.values()
        if pv.size > 5 and (pv.max() - pv.min()) > c.nod_drop_deg and \
           pv[-1] < pv.max() - c.nod_drop_deg * 0.7:
            add(50.0, "head nod / bobbing")
            if f.t >= self._nod_latched_until:
                self._nod_latched_until = f.t + 1.5
                self._event("head nod")
        return distracted

    def _event(self, text):
        self.event_log.append((time.strftime("%H:%M:%S"), text))

    # -- main entry point -----------------------------------------------------
    def update(self, f: Features) -> Evidence:
        c = self.cfg
        dt = (f.t - self.last_t) if self.last_t is not None else 0.1
        dt = min(dt, 1.0)
        self.last_t = f.t

        self.cal.feed(f)
        if self.cal.done and self.last_blink_t == 0.0:
            self.last_blink_t = f.t

        self.closure.push(f.t, f.eye_closed)
        self.squint_b.push(f.t, f.squint)
        self.brow_b.push(f.t, f.brow_down)
        self.inner_b.push(f.t, f.brow_inner_up)
        self.energy_b.push(f.t, f.energy)
        self.gx_b.push(f.t, f.gaze_x)
        self.gy_b.push(f.t, f.gaze_y)
        self.yaw_b.push(f.t, f.yaw)
        self.pitch_b.push(f.t, f.pitch)

        while self.blink_times and f.t - self.blink_times[0] > 60.0:
            self.blink_times.popleft()
        self.state.blink_rate = len(self.blink_times)

        # HARD first: it is a measurement and feeds the policy directly.
        closure_s, new_micro = self._lid_closure(f)

        # TREND: fused, then clamped. Nothing below can command an actuator.
        contributions, reasons = [], []

        def add(score, why):
            contributions.append(score)
            reasons.append(why)

        self._perclos(f, add)
        self._drowsy_face(f, add)
        self._yawn(f, add)
        stare = self._eyes_stuck(f, add)
        distracted = self._distraction(f, add)

        if not self.cal.done:
            self.state.status = "CALIBRATING"
            self.state.risk = 0.0
            self.state.trend_risk = 0.0
            self.state.reasons = ["hold a neutral face, look at the road"]
            return Evidence(t=f.t, face=True, closure_s=0.0, eyes_open=True,
                            trend_risk=0.0, status="CALIBRATING",
                            reasons=list(self.state.reasons))

        eyes_open = (f.eye_closed < c.close_off) or self.sensor_fault
        recovering = (closure_s == 0.0 and not stare and eyes_open)
        decay = c.risk_decay_per_s * (c.recovery_decay_mult if recovering else 1.0)

        if contributions:
            target = min(c.trend_cap,
                         max(contributions) +
                         0.35 * (sum(contributions) - max(contributions)))
            if target > self.trend_risk:
                self.trend_risk = target       # fresh evidence: jump up
            else:
                # Evidence has WEAKENED, so fall toward it at the decay rate.
                # Pinning risk to its historical peak while ANY contribution
                # exists is what once kept the buzzer running ~10 s past the
                # eyes reopening.
                self.trend_risk = max(target, self.trend_risk - decay * dt)
        else:
            self.trend_risk = max(0.0, self.trend_risk - decay * dt)
        self.trend_risk = min(self.trend_risk, c.trend_cap)

        # Display-only risk: shows red during a real closure so the operator
        # sees severity, without that number ever reaching the policy.
        if closure_s >= c.sleep_s:
            hard_display = 100.0
        elif closure_s >= c.microsleep_s:
            span = max(1e-6, c.sleep_s - c.microsleep_s)
            hard_display = 55.0 + 45.0 * (closure_s - c.microsleep_s) / span
        else:
            hard_display = 0.0

        if self.sensor_fault:
            status = "SENSOR FAULT"
        elif closure_s >= c.deep_sleep_s:
            status = "DEEP SLEEP"
        elif closure_s >= c.sleep_s:
            status = "SLEEPING"
        elif closure_s >= c.microsleep_s:
            status = "MICROSLEEP"
        elif stare:
            status = "EYES STUCK"
        elif distracted:
            status = "DISTRACTED"
        elif self.trend_risk >= c.risk_warn:
            status = "DROWSY"
        elif self.trend_risk >= c.risk_notice:
            status = "MILD FATIGUE"
        else:
            status = "ALERT"

        if self.sensor_fault:
            reasons = ["lid signal stuck -- eye monitoring unavailable"]
        elif closure_s >= c.microsleep_s:
            reasons = [f"eyes shut {closure_s:.2f}s"] + reasons

        self.state.status = status
        self.state.trend_risk = self.trend_risk
        self.state.risk = max(self.trend_risk, hard_display)
        self.state.reasons = reasons[:3]
        self.state.microsleeps = self.microsleep_count
        self.state.sleeps = self.sleep_count
        self.state.stares = self.stare_count
        self.state.faults = self.fault_count

        return Evidence(t=f.t, face=True, closure_s=closure_s,
                        eyes_open=eyes_open, trend_risk=self.trend_risk,
                        new_microsleep=new_micro, stare=stare,
                        sensor_fault=self.sensor_fault,
                        status=status, reasons=self.state.reasons)

    def no_face(self, t, why="driver not visible") -> Evidence:
        dt = (t - self.last_t) if self.last_t is not None else 0.1
        self.last_t = t
        self.trend_risk = max(0.0, self.trend_risk
                              - self.cfg.risk_decay_per_s * 0.5 * min(dt, 1.0))
        self.state.status = "NO FACE"
        self.state.trend_risk = self.trend_risk
        self.state.risk = self.trend_risk
        self.state.reasons = [why]
        self.state.closure_s = 0.0
        self.state.stare_s = 0.0
        self.eyes_shut = False
        self.shut_frames = 0
        self._micro_latched = False
        self._sleep_latched = False
        self.sensor_fault = False
        self.frozen_since = None
        self.frozen_frames = 0
        self._stare_latched = False
        return Evidence(t=t, face=False, closure_s=0.0, eyes_open=True,
                        trend_risk=self.trend_risk, status="NO FACE",
                        reasons=[why])


# =============================================================================
#  10. MAIN
# =============================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description="Headless microsleep detector for Raspberry Pi + OV5647",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # ---- camera ------------------------------------------------------------
    ap.add_argument("--backend", choices=("auto", "picamera2", "v4l2"),
                    default="auto",
                    help="picamera2 for the CSI camera (OV5647), v4l2 for a "
                         "USB webcam or a file")
    ap.add_argument("--camera-num", type=int, default=0,
                    help="picamera2 camera index (see rpicam-hello --list-cameras)")
    ap.add_argument("--camera", default="0",
                    help="v4l2 backend only: index or /dev/videoN")
    ap.add_argument("--video", default=None, help="run on a file instead")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0,
                    help="requested sensor frame rate; the OV5647 does 62.5 "
                         "at 640x480, but the model will not keep up")
    ap.add_argument("--swap-rb", action="store_true",
                    help="swap red and blue. Try this if no face is ever "
                         "detected in good light")
    ap.add_argument("--vflip", action="store_true", help="camera mounted upside down")
    ap.add_argument("--hflip", action="store_true", help="mirror horizontally")
    ap.add_argument("--fourcc", default="MJPG",
                    help="v4l2 backend only; '' leaves the driver alone")
    ap.add_argument("--infer-width", type=int, default=384,
                    help="downscale width fed to the model; 0 = native")
    ap.add_argument("--camera-timeout", type=float, default=6.0,
                    help="seconds of silence before the camera is restarted")
    ap.add_argument("--test-camera", action="store_true",
                    help="prove the camera and exit")
    ap.add_argument("--explain", action="store_true",
                    help="print the escalation ladder at startup")
    ap.add_argument("--max-frame-errors", type=int, default=10,
                    help="consecutive per-frame failures tolerated before "
                         "giving up")

    # ---- buzzer wiring -----------------------------------------------------
    ap.add_argument("--gpiochip", default=None,
                    help="override the auto-detected header chip "
                         "(e.g. gpiochip0, gpiochip4)")
    ap.add_argument("--gpio-line", type=int, default=23,
                    help="buzzer line = BCM number (23 = header pin 16)")
    ap.add_argument("--buzzer-active-low", action="store_true",
                    help="buzzer sounds when the pin is driven LOW (most "
                         "3-pin modules)")
    ap.add_argument("--no-buzzer", action="store_true",
                    help="console alerts only, do not touch the buzzer GPIO")
    ap.add_argument("--test-buzzer", action="store_true",
                    help="play warn + critical patterns and exit")
    ap.add_argument("--tone-mode", choices=("solid", "pwm"), default=CFG.tone_mode,
                    help="solid = ACTIVE buzzer; pwm = real notes, PASSIVE piezo only")
    ap.add_argument("--buzzer-debug", action="store_true")
    ap.add_argument("--buzzer-reassert", type=float, default=CFG.buzzer_reassert_s)

    # ---- face-presence LED -------------------------------------------------
    ap.add_argument("--led-chip", default=None,
                    help="defaults to the same chip as the buzzer")
    ap.add_argument("--led-line", type=int, default=24,
                    help="LED line = BCM number (24 = header pin 18)")
    ap.add_argument("--led-active-low", action="store_true",
                    help="LED lights when the pin is driven LOW (wired "
                         "between 3V3 and the pin)")
    ap.add_argument("--no-led", action="store_true",
                    help="do not touch the LED GPIO at all")
    ap.add_argument("--test-led", action="store_true",
                    help="blink the LED and exit")
    ap.add_argument("--led-hold", type=float, default=CFG.led_hold_s,
                    help="keep the LED lit this long after the face is lost, "
                         "so one dropped detection frame does not flicker it")

    # ---- buzzer pattern shape ---------------------------------------------
    ap.add_argument("--warn-beep", type=float, default=CFG.warn_on_s,
                    help="length of the single warning beep")
    ap.add_argument("--warn-cooldown", type=float, default=CFG.alarm_cooldown_s,
                    help="minimum seconds between warning beeps")
    ap.add_argument("--crit-on", type=float, default=CFG.crit_on_s)
    ap.add_argument("--crit-gap", type=float, default=CFG.crit_gap_s)
    ap.add_argument("--crit-pause", type=float, default=CFG.crit_pause_s)
    ap.add_argument("--no-noface-alert", action="store_true")
    ap.add_argument("--noface-delay", type=float, default=CFG.noface_delay_s)

    # ---- the safety ladder -------------------------------------------------
    ap.add_argument("--sensitivity", choices=("relaxed", "normal", "strict"),
                    default="normal",
                    help="how readily the system escalates toward the speed byte")
    ap.add_argument("--close-on", type=float, default=None,
                    help="blendshape value that counts as eyes SHUT")
    ap.add_argument("--close-off", type=float, default=None,
                    help="value the signal must fall below to count as OPEN "
                         "again; keep it well under --close-on or the closure "
                         "timer can latch")
    ap.add_argument("--no-auto-eye-threshold", dest="auto_eye",
                    action="store_false",
                    help="do not adapt close_on/close_off to the driver "
                         "measured during calibration")
    ap.add_argument("--auto-eye-lower", action="store_true",
                    help="also allow calibration to LOWER close_on (more "
                         "sensitive than the default; opt-in)")
    ap.add_argument("--microsleep", type=float, default=None,
                    help="closure that counts as a microsleep (NOTICE)")
    ap.add_argument("--microsleep-warn", type=float, default=None,
                    help="closure that earns a single beep (WARNING)")
    ap.add_argument("--sleep", dest="sleep_s", type=float, default=None,
                    help="closure that raises the buzzer (ALARM)")
    ap.add_argument("--deep-sleep", type=float, default=None,
                    help="closure that commands the speed byte (INTERVENTION)")
    ap.add_argument("--micro-warn-n", type=int, default=None)
    ap.add_argument("--micro-alarm-n", type=int, default=None)
    ap.add_argument("--micro-intervene-n", type=int, default=None)
    ap.add_argument("--micro-window", type=float, default=None,
                    help="the repetition window, in seconds")
    ap.add_argument("--intervene-after", type=float, default=None,
                    help="seconds an ALARM may go unheeded before intervening")
    ap.add_argument("--intervene-min-hold", type=float, default=None,
                    help="minimum seconds the speed byte is held once issued")
    ap.add_argument("--trend-cap", type=float, default=None,
                    help="ceiling on fused soft evidence (keep below the "
                         "alarm bar unless you want PERCLOS driving the bus)")
    ap.add_argument("--latch-fault", type=float, default=None,
                    help="unbroken closure after which a moving head means "
                         "the lid signal is stuck, not asleep")
    ap.add_argument("--hard-only", "--buzzer-hard-only", dest="hard_only",
                    action="store_true",
                    help="trend evidence may not raise any alert at all")

    # ---- serial speed output ----------------------------------------------
    ap.add_argument("--serial-port", default="/dev/ttyUSB0",
                    help="USB-TTL adapter, or /dev/serial0 for the Pi UART "
                         "(pins 8/10, console disabled)")
    ap.add_argument("--serial-baud", type=int, default=9600)
    ap.add_argument("--intervene-speed", "--critical-speed", dest="intervene_speed",
                    type=int, default=30,
                    help="raw byte sent repeatedly at INTERVENTION")
    ap.add_argument("--caution-speed", type=int, default=None,
                    help="optional milder byte sent at ALARM")
    ap.add_argument("--normal-speed", type=int, default=None,
                    help="optional byte sent once when the limit is released")
    ap.add_argument("--serial-interval", type=float, default=1.0)
    ap.add_argument("--serial-log-interval", type=float, default=5.0)
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--quiet-serial", action="store_true")

    ap.add_argument("--quiet-alerts", action="store_true")
    ap.add_argument("--calib", type=float, default=CFG.calib_seconds)
    ap.add_argument("--heartbeat", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after N seconds (0 = run forever)")
    return ap


def apply_args_to_config(args):
    CFG.calib_seconds = args.calib
    CFG.warn_on_s = args.warn_beep
    CFG.alarm_cooldown_s = args.warn_cooldown
    CFG.crit_on_s = args.crit_on
    CFG.crit_gap_s = args.crit_gap
    CFG.crit_pause_s = args.crit_pause
    CFG.tone_mode = args.tone_mode
    CFG.noface_delay_s = args.noface_delay
    CFG.buzzer_reassert_s = args.buzzer_reassert
    CFG.led_hold_s = args.led_hold

    # Presets first, explicit flags second -- so a flag always wins.
    if args.sensitivity == "relaxed":
        CFG.microsleep_warn_s = 1.00
        CFG.sleep_s = 2.50
        CFG.deep_sleep_s = 4.50
        CFG.micro_warn_n, CFG.micro_alarm_n, CFG.micro_intervene_n = 3, 4, 5
        CFG.intervene_after_s = 5.0
        CFG.risk_warn = 45.0
    elif args.sensitivity == "strict":
        CFG.microsleep_warn_s = 0.60
        CFG.sleep_s = 1.50
        CFG.deep_sleep_s = 2.50
        CFG.micro_warn_n, CFG.micro_alarm_n, CFG.micro_intervene_n = 2, 2, 3
        CFG.intervene_after_s = 2.0
        CFG.risk_warn = 35.0

    for name, val in (("close_on", args.close_on),
                      ("close_off", args.close_off),
                      ("microsleep_s", args.microsleep),
                      ("microsleep_warn_s", args.microsleep_warn),
                      ("sleep_s", args.sleep_s),
                      ("deep_sleep_s", args.deep_sleep),
                      ("micro_warn_n", args.micro_warn_n),
                      ("micro_alarm_n", args.micro_alarm_n),
                      ("micro_intervene_n", args.micro_intervene_n),
                      ("micro_window_s", args.micro_window),
                      ("intervene_after_s", args.intervene_after),
                      ("intervene_min_hold_s", args.intervene_min_hold),
                      ("trend_cap", args.trend_cap),
                      ("latch_fault_s", args.latch_fault)):
        if val is not None:
            setattr(CFG, name, val)

    # Fail loudly on a ladder that does not make sense, rather than quietly
    # doing something surprising at 2 a.m. on a highway.
    if not (CFG.microsleep_s < CFG.microsleep_warn_s
            < CFG.sleep_s < CFG.deep_sleep_s):
        sys.exit(f"invalid closure ladder: microsleep {CFG.microsleep_s} < "
                 f"warn {CFG.microsleep_warn_s} < sleep {CFG.sleep_s} < "
                 f"deep {CFG.deep_sleep_s} must hold")
    if not (CFG.micro_warn_n <= CFG.micro_alarm_n <= CFG.micro_intervene_n):
        sys.exit("invalid repetition ladder: warn <= alarm <= intervene required")
    if CFG.close_off >= CFG.close_on:
        sys.exit(f"close_off ({CFG.close_off}) must be below close_on "
                 f"({CFG.close_on})")
    if CFG.close_on - CFG.close_off < 0.10:
        sys.stderr.write(f"WARNING: only {CFG.close_on - CFG.close_off:.2f} "
                         f"between close_on and close_off. A narrow band makes "
                         f"the closure timer chatter on ordinary blinks.\n")
    if CFG.latch_fault_s <= CFG.deep_sleep_s * 3:
        sys.stderr.write(f"WARNING: latch_fault_s ({CFG.latch_fault_s:.0f}s) is "
                         f"close to deep_sleep_s; a genuinely unconscious "
                         f"driver could be written off as a sensor fault\n")
    if CFG.trend_cap >= 100.0:
        sys.stderr.write("WARNING: trend_cap is very high; soft evidence can "
                         "now dominate the ladder\n")


def open_gpio(args):
    """
    Bring up both indicator lines. Returns (gpio, buzzer, led).

    Called from INSIDE the protected block in main() -- see fix [1]. Anything
    that opens a GPIO line before the cleanup path exists is one ordinary
    startup error away from leaving a pin driven.
    """
    chip = find_gpiochip(args.gpiochip)
    led_chip = args.led_chip or chip

    if (not args.no_buzzer and not args.no_led
            and chip == led_chip and args.gpio_line == args.led_line):
        sys.exit(f"buzzer and LED are both on {chip} line {args.gpio_line} -- "
                 f"give the LED its own line with --led-line")

    gpio = buzzer = led = None

    if not args.no_buzzer:
        try:
            gpio = GpioLine(chip, args.gpio_line,
                            active_low=args.buzzer_active_low,
                            consumer="microsleep-buzzer",
                            debug=args.buzzer_debug)
            atexit.register(gpio.close)
            buzzer = Buzzer(gpio, CFG, quiet=args.quiet_alerts)
            print(f"buzzer: {gpio.chip_name} line {gpio.line_num} "
                  f"(BCM{gpio.line_num}) via {gpio.backend}"
                  f"{' (active-low)' if gpio.active_low else ''}, "
                  f"{'pwm tones' if buzzer.pwm else 'solid beeps'}")
            if gpio.backend == "gpioset-persistent":
                print("  note: no libgpiod python bindings found. The gpioset "
                      "fallback works,\n        but "
                      "'sudo apt install python3-libgpiod' is more reliable.")
        except Exception as e:
            sys.stderr.write(f"buzzer unavailable ({e}) -- continuing without it\n")
            buzzer, gpio = None, None

    if not args.no_led:
        try:
            led = GpioLine(led_chip, args.led_line,
                           active_low=args.led_active_low,
                           consumer="microsleep-led",
                           debug=args.buzzer_debug)
            atexit.register(led.close)
            print(f"led: {led.chip_name} line {led.line_num} "
                  f"(BCM{led.line_num}) via {led.backend}"
                  f"{' (active-low)' if led.active_low else ''} "
                  f"-- ON while a face is visible, OFF "
                  f"{CFG.led_hold_s:.2f}s after it is lost")
        except Exception as e:
            # Not fatal. A missing indicator lamp must never stop the thing
            # that watches the driver.
            sys.stderr.write(f"led unavailable ({e}) -- continuing without it\n")
            led = None

    return gpio, buzzer, led


def main(argv=None):
    args = build_parser().parse_args(argv)
    apply_args_to_config(args)

    fit_eyes = args.auto_eye and args.close_on is None
    if args.explain:
        explain_ladder(CFG, args.hard_only)

    if args.test_camera:
        camera_selftest(args)
        return

    gpio = buzzer = led = None
    grab = speed_tx = alerter = landmarker = None

    try:
        # ---- FIX [1]: GPIO opens inside the block that closes it ----------
        gpio, buzzer, led = open_gpio(args)

        if args.test_buzzer or args.test_led:
            if args.test_buzzer:
                if buzzer is None:
                    sys.exit("no buzzer to test")
                buzzer.selftest()
            if args.test_led:
                if led is None:
                    sys.exit("no led to test")
                led_selftest(led)
            return

        model = ensure_model()
        landmarker = mp_vision.FaceLandmarker.create_from_options(
            mp_vision.FaceLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=model),
                running_mode=mp_vision.RunningMode.VIDEO,
                num_faces=1,
                min_face_detection_confidence=0.5,
                min_face_presence_confidence=0.5,
                min_tracking_confidence=0.5,
                output_face_blendshapes=True,
                output_facial_transformation_matrixes=True,
            )
        )

        grab = open_camera(args)
        print(f"camera: {grab.name} {grab.actual[0]}x{grab.actual[1]}"
              + (f" @ {grab.actual[2]:.0f} fps requested" if grab.actual[2] else ""))

        if not args.no_serial:
            speed_tx = SpeedSerial(port=args.serial_port, baud=args.serial_baud,
                                   normal_speed=args.normal_speed,
                                   resend_s=args.serial_interval,
                                   quiet=args.quiet_serial,
                                   log_every_s=args.serial_log_interval)
            if speed_tx.ok:
                print(f"serial: {args.serial_port} @ {args.serial_baud} baud, "
                      f"byte {args.intervene_speed} at INTERVENTION"
                      + (f", byte {args.caution_speed} at ALARM"
                         if args.caution_speed is not None else "")
                      + (f", byte {args.normal_speed} on release"
                         if args.normal_speed is not None else ""))
            else:
                speed_tx = None

        det = MicrosleepDetector(CFG)
        policy = SafetyPolicy(CFG, hard_only=args.hard_only)

        print(f"escalation: {args.sensitivity}"
              + ("  [hard evidence only]" if args.hard_only else ""))
        print(f"  WARNING (one beep) : eyes shut >= {CFG.microsleep_warn_s:.2f}s, "
              f"or {CFG.micro_warn_n} microsleeps / {CFG.micro_window_s:.0f}s, "
              f"or trend >= {CFG.risk_warn:.0f}")
        print(f"  ALARM   (buzzer)   : eyes shut >= {CFG.sleep_s:.2f}s, "
              f"or {CFG.micro_alarm_n} microsleeps / {CFG.micro_window_s:.0f}s")
        print(f"  INTERVENE (speed)  : eyes shut >= {CFG.deep_sleep_s:.2f}s, "
              f"or ALARM unheeded {CFG.intervene_after_s:.1f}s, "
              f"or {CFG.micro_intervene_n} microsleeps / {CFG.micro_window_s:.0f}s")
        print(f"  trend evidence capped at {CFG.trend_cap:.0f} -- it can raise a "
              f"warning, never an intervention")
        print(f"  the LED is outside this ladder: face presence only")
        print(f"eyes: shut above {CFG.close_on:.2f}, open below {CFG.close_off:.2f}"
              + ("  (will be fitted to the driver at calibration)" if fit_eyes
                 else "  (fixed)"))

        alerter = Alerter(CFG.alarm_cooldown_s, buzzer, quiet=args.quiet_alerts,
                          noface_delay_s=CFG.noface_delay_s,
                          noface_enabled=not args.no_noface_alert,
                          speed=speed_tx,
                          intervene_speed=args.intervene_speed,
                          caution_speed=args.caution_speed,
                          led=led, led_hold_s=CFG.led_hold_s,
                          led_reassert_s=CFG.led_reassert_s)

        stop = threading.Event()

        def on_signal(signum, _frame):
            print(f"\nsignal {signum} -- shutting down")
            stop.set()

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        t_zero = time.monotonic()
        fps = 0.0
        last_proc = time.monotonic()
        last_status = None
        last_status_at = 0.0
        last_level = SafetyPolicy.NORMAL
        last_beat = time.monotonic()
        last_frame_at = time.monotonic()
        last_ts_ms = -1
        frames = 0
        frame_errors = 0
        st = det.state
        ev = Evidence()
        level = SafetyPolicy.NORMAL

        print("running. Ctrl-C to stop.")
        while not stop.is_set():
            if args.duration and time.monotonic() - t_zero > args.duration:
                break

            try:
                got = grab.latest(timeout=1.0)
                now = time.monotonic()

                # ---- FIX [2]: camera silence is EVIDENCE, not a `continue` -
                # Skipping the policy here is what let a camera brown-out
                # during an ALARM leave the buzzer looping and the speed byte
                # engaged until power-cycle. A dead camera means no face, and
                # the ladder already knows exactly what to do about that.
                if got is None:
                    silent = now - last_frame_at
                    ev = det.no_face(now - t_zero,
                                     f"no camera frame for {silent:.0f}s")
                    st = det.state
                    level, why = policy.update(now, ev)
                    alerter.apply(level, ev, "camera silent")
                    if silent >= args.camera_timeout:
                        if grab.restart():
                            last_frame_at = time.monotonic()
                        else:
                            time.sleep(2.0)
                    continue

                frame, cap_t, _seq = got
                last_frame_at = now
                t = cap_t - t_zero

                if args.infer_width and frame.shape[1] > args.infer_width:
                    scale = args.infer_width / frame.shape[1]
                    small = cv2.resize(frame, None, fx=scale, fy=scale,
                                       interpolation=cv2.INTER_AREA)
                else:
                    small = frame

                rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

                # MediaPipe's VIDEO mode requires strictly increasing stamps.
                ts_ms = int(t * 1000)
                if ts_ms <= last_ts_ms:
                    ts_ms = last_ts_ms + 1
                last_ts_ms = ts_ms

                res = landmarker.detect_for_video(mp_img, ts_ms)

                if res.face_blendshapes:
                    matrix = (res.facial_transformation_matrixes[0]
                              if res.facial_transformation_matrixes else None)
                    f = extract_features(res.face_blendshapes[0], matrix, t)
                    ev = det.update(f)
                else:
                    ev = det.no_face(t)
                st = det.state

                frames += 1
                dt = now - last_proc
                last_proc = now
                if dt > 0:
                    fps = (0.9 * fps + 0.1 * (1.0 / dt)) if fps else 1.0 / dt

                if det.cal.done and not det.tuned:
                    changes = det.retune(fps, fit_eyes=fit_eyes,
                                         allow_lower=args.auto_eye_lower)
                    print(f"\ncalibrated. measured {fps:.1f} fps  |  "
                          f"this driver's open eyes reach "
                          f"{det.cal.base['eye_open_hi']:.2f} "
                          f"(eyes counted shut above {CFG.close_on:.2f}, "
                          f"open again below {CFG.close_off:.2f})")
                    if changes:
                        print("thresholds adjusted for this frame rate:")
                        for c in changes:
                            print("  " + c)
                    else:
                        print("all default thresholds hold at this frame rate")
                    print()

                # ---- policy, then actuators. In that order, always. --------
                # The detector's per-frame view NEVER reaches an actuator
                # directly. It is evidence; the policy decides what it is
                # worth. The LED is the one deliberate exception: it reports a
                # sensor fact (is there a face?), not a safety judgement.
                level, why = policy.update(now, ev)
                if level != last_level:
                    arrow = "^^" if level > last_level else "vv"
                    print(f"[{time.strftime('%H:%M:%S')}] {arrow} "
                          f"{SafetyPolicy.NAMES[last_level]} -> "
                          f"{SafetyPolicy.NAMES[level]}   {why}")
                    last_level = level

                alerter.apply(level, ev, why)

                # Console anti-flap: status is a display string and it flickers.
                if st.status != last_status and \
                   (now - last_status_at) >= CFG.status_print_min_s:
                    last_status_at = now
                    tail = f"  {st.reasons[0]}" if st.reasons else ""
                    print(f"[{time.strftime('%H:%M:%S')}] {st.status:<13} "
                          f"risk {st.risk:5.1f}{tail}")
                    last_status = st.status
                elif args.heartbeat and now - last_beat >= args.heartbeat:
                    last_beat = now
                    print(f"[{time.strftime('%H:%M:%S')}] {st.status:<13} "
                          f"risk {st.risk:5.1f}  [{policy.status_line(now)}]  "
                          f"{fps:.1f}fps  perclos {st.perclos*100:.0f}%  "
                          f"blinks/min {st.blink_rate:.0f}  "
                          f"ms {st.microsleeps} slp {st.sleeps} "
                          f"stuck {st.stares}  dropped {grab.dropped}")

                frame_errors = 0

            except KeyboardInterrupt:
                raise
            except Exception as e:
                # ---- FIX [3]: one bad frame must not kill the monitor -----
                frame_errors += 1
                sys.stderr.write(f"frame error {frame_errors}/"
                                 f"{args.max_frame_errors}: "
                                 f"{type(e).__name__}: {e}\n")
                if frame_errors >= args.max_frame_errors:
                    sys.stderr.write("too many consecutive frame errors -- "
                                     "shutting down\n")
                    break
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Actuators off FIRST, always, whatever else failed.
        if alerter:
            alerter.shutdown()
        if buzzer:
            buzzer.shutdown()
        if gpio:
            gpio.close()
        if led:
            led.close()
        if speed_tx:
            speed_tx.shutdown()
        if grab:
            grab.release()
        if landmarker:
            try:
                landmarker.close()
            except Exception:
                pass

    # ---- summary (outside the finally so a cleanup error cannot hide it) ---
    try:
        elapsed = time.monotonic() - t_zero
        print("\n--- session summary ---")
        print(f"  ran {elapsed/60:.1f} min, {frames} frames, "
              f"{frames/max(elapsed,1e-6):.1f} fps average")
        print(f"  camera frames dropped by design: {grab.dropped}   "
              f"restarts: {grab.restarts}   read errors: {grab.read_errors}")
        print(f"  microsleeps: {det.microsleep_count}   "
              f"sleep events: {det.sleep_count}   "
              f"eyes-stuck episodes: {det.stare_count}   "
              f"sensor faults: {det.fault_count}")
        print(f"  warning beeps: {alerter.beeps}   "
              f"alarms: {policy.alarms}   interventions: {policy.interventions}")
        if speed_tx:
            print(f"  speed bytes sent: {speed_tx.sent}")
        if gpio and gpio.write_errors:
            print(f"  buzzer gpio write errors: {gpio.write_errors}")
        if led and led.write_errors:
            print(f"  led gpio write errors: {led.write_errors}")
        if policy.log:
            print("  escalation log:")
            for ts, evt in policy.log:
                print(f"    {ts}  {evt}")
        if det.event_log:
            print("  events:")
            for ts, evt in det.event_log:
                print(f"    {ts}  {evt}")
    except (NameError, UnboundLocalError):
        pass   # died before the loop ever ran; nothing to summarise


if __name__ == "__main__":
    main()
