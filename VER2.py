#!/usr/bin/env python3
"""
==============================================================================
 EXPRESSION-BASED MICROSLEEP DETECTOR  --  RASPBERRY PI / OV5647 BUILD
 v8 -- vehicle speed always read on /dev/serial0; byte 30 at INTERVENTION
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
 NEW IN v7: SPEED SLABS
 ---------------------------------------------------------------------------
 The vehicle speed arrives on the serial RX line and selects a SLAB. Each
 slab sets every level of the ladder independently:

   SLAB    km/h     NOTICE  WARNING  ALARM  INTERV  unheeded  repeats w/a/i
   MUTED   0-5      off     off      off    off     off       off
   LOW     5-40     0.60    1.20     3.00   4.90    4.50      3/4/5
   MID     40-80    0.40    0.80     2.00   3.50    3.00      2/3/4
   HIGH    80+      0.40    0.60     1.50   2.62    2.01      2/3/4

   (seconds of continuous eye closure; MID = your tuned ladder)

 Everything is editable: values, OFF switches, the edges, the number and
 names of slabs.
   --write-slabs slabs.json      dump the defaults, edit, then
   --slabs slabs.json            load them
   --slab-edges 5,40,80          move the edges
   --slab HIGH:alarm=1.2,intervention=off     change single values
   --explain-slabs               print the final table and exit

 Safety rules (SlabSelector):
   faster slab   applied immediately
   slower slab   needs speed below edge - hysteresis for the dwell time, and
                 never while an event is in progress
   no speed      fallback slab (MID): a dead speed link never mutes the car

 microsleep_s (0.40 s) is NOT per slab: it defines what gets COUNTED for the
 repetition routes, and must mean the same thing at every speed.

 RX FORMATS (--speed-rx):
   byte   one raw byte = km/h, 0-255. Newest byte wins.
   ascii  text lines ending in \\n; the last number on each line is km/h.

 BENCH:  --fixed-speed 110   choose the slab as if at 110 km/h. The real
                             vehicle speed is STILL read and printed; it
                             just does not pick the slab for this run.

 ---------------------------------------------------------------------------
 CARRIED FROM v5
 ---------------------------------------------------------------------------
 picamera2 backend for the OV5647; GPIO chip found by label; no logging or
 streaming; GPIO opened inside try/finally; camera silence fed to the policy
 as no-face; per-frame exception guard; stuck-lid sensor-fault escape;
 gpioset fallback no longer respawns on forced writes.

 ---------------------------------------------------------------------------
 THE ESCALATION LADDER  (baseline = MID slab, or no speed input)
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
          evidence can ever touch the throttle on its own.

==============================================================================
 SETUP  (Raspberry Pi OS Bookworm, 64-bit)

   sudo apt update
   sudo apt install -y python3-picamera2 python3-libgpiod python3-serial \
                       libcap-dev libcamera-dev
   python3 -m venv --system-site-packages ~/msenv
   source ~/msenv/bin/activate
   pip install mediapipe==0.10.18 opencv-python-headless "numpy<2"

 WIRING
   buzzer  BCM23  header pin 16
   LED     BCM24  header pin 18   through a 220-330 ohm resistor to GND
   camera  OV5647 on the CSI ribbon
   serial  USB-TTL on /dev/ttyUSB0:  adapter TX -> AVR RX (speed byte out)
                                     adapter RX <- AVR TX (vehicle speed in)
                                     GND common. 3.3 V logic on the Pi UART:
                                     a 5 V AVR TX needs a divider there.

 RUN
   python microsleep_pi.py                          # AVR on /dev/serial0,
                                                    # raw-byte speed in,
                                                    # byte 30 out at INTERVENTION
   python microsleep_pi.py --speed-rx ascii         # AVR sends "72\\n"
   python microsleep_pi.py --fixed-speed 100 --slabs slabs.json
                          # bench: slab from 100 km/h, real speed still read
   python microsleep_pi.py --test-serial            # prove TX + RX, exit
   python microsleep_pi.py --explain-slabs          # table, exit
   python microsleep_pi.py --write-slabs slabs.json # editable defaults
   python microsleep_pi.py --speed-rx ascii --slabs slabs.json
   python microsleep_pi.py --test-buzzer | --test-led | --test-camera
==============================================================================
"""

import argparse
import atexit
import glob
import json
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
    close_on: float = 0.55
    close_off: float = 0.35
    blink_max_s: float = 0.35
    microsleep_s: float = 0.40        # what COUNTS as a microsleep (fixed)
    microsleep_warn_s: float = 0.80   # WARNING      (slab-controlled)
    sleep_s: float = 2.00             # ALARM        (slab-controlled)
    deep_sleep_s: float = 3.50        # INTERVENTION (slab-controlled)

    min_event_frames: int = 3

    # --- stuck-signal escape -----------------------------------------------
    latch_fault_s: float = 45.0
    latch_fault_motion_ratio: float = 1.8

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

    # --- trend fusion --------------------------------------------------------
    trend_cap: float = 65.0
    risk_decay_per_s: float = 12.0
    recovery_decay_mult: float = 3.0

    risk_notice: float = 18.0
    risk_warn: float = 40.0
    risk_warn_exit: float = 25.0

    # --- repetition ----------------------------------------------------------
    micro_window_s: float = 90.0
    micro_sustain_s: float = 2.50
    micro_warn_n: int = 2
    micro_alarm_n: int = 3
    micro_intervene_n: int = 4

    # --- confirmation (debounce) -------------------------------------------
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

    intervene_after_s: float = 3.0     # slab 'unheeded'

    face_loss_hold_s: float = 3.0

    calib_seconds: float = 5.0
    calib_min_samples: int = 25
    alarm_cooldown_s: float = 4.0
    status_print_min_s: float = 0.40

    # =====================================================================
    #  SPEED SLABS  (v7) -- the SlabSelector writes the fields below
    # =====================================================================
    notice_s: float = 0.40             # closure that raises NOTICE
    notice_on: bool = True             # level enable flags; the policy
    warn_on: bool = True               # never enters a level whose flag
    alarm_on: bool = True              # is False
    intervene_on: bool = True
    slab_hysteresis_kmh: float = 3.0
    slab_down_dwell_s: float = 5.0
    speed_timeout_s: float = 3.0       # older than this = no speed known
    speed_max_kmh: float = 250.0       # anything above is a corrupt reading
    speed_smooth_s: float = 0.5        # EMA time constant on the speed

    # --- buzzer patterns ---------------------------------------------------
    warn_on_s: float = 0.30
    crit_on_s: float = 0.40
    crit_gap_s: float = 0.25
    crit_pause_s: float = 2.00

    warn_hz: float = 660.0
    crit_hz: float = 880.0
    tone_mode: str = "solid"

    noface_delay_s: float = 2.0
    buzzer_reassert_s: float = 0.5

    # --- face-presence LED -------------------------------------------------
    led_hold_s: float = 0.50
    led_reassert_s: float = 2.0


CFG = Config()


def autotune(cfg: Config, fps: float):
    """
    Raise any threshold that would otherwise rest on fewer than
    min_event_frames samples at the frame rate this board actually achieves.
    A threshold in SECONDS silently becomes a threshold in FRAMES on real
    hardware: 0.40 s is four frames at 9 fps and twelve at 30 fps.
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

    if cfg.notice_s < cfg.microsleep_s:
        cfg.notice_s = cfg.microsleep_s
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
    """Find the chip that owns the 40-pin header, by LABEL, not by number."""
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
    Backends: libgpiod v2 -> libgpiod v1 -> persistent `gpioset` child.
    """

    def __init__(self, chip="gpiochip0", line=23, active_low=False,
                 consumer="microsleep", debug=False):
        self.chip_name = chip
        self.line_num = int(line)
        self.active_low = active_low
        self.consumer = consumer
        self.debug = debug
        self.backend = None
        self._is_on = None
        self._proc = None
        self._gpioset_style = None
        self._lock = threading.Lock()
        self.write_errors = 0

        self._req = None
        self._v1_line = None
        self._v1_chip = None

        self._open()

    def _open(self):
        if self._try_v2():
            self.backend = "libgpiod-v2"
        elif self._try_v1():
            self.backend = "libgpiod-v1"
        else:
            self.backend = "gpioset-persistent"
        self.set(False, force=True)

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

    def set(self, on: bool, force: bool = False):
        """Drive the line. force=True rewrites hardware even if cached."""
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
                self._is_on = on
                if self.debug:
                    sys.stdout.write(f"[gpio {self.line_num}] "
                                     f"{'HIGH' if on else 'LOW '}"
                                     f"{' (forced)' if force else ''}\n")
                    sys.stdout.flush()
            except Exception as e:
                self._is_on = None
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
            time.sleep(0.008)
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
        """Drive LOW, let it land, then release."""
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
    on: bool
    dur: float
    freq: float = 0.0


def pattern_warn(c: Config):
    return [Step(True, c.warn_on_s, c.warn_hz)]


def pattern_critical(c: Config):
    return [Step(True, c.crit_on_s, c.crit_hz),
            Step(False, c.crit_gap_s),
            Step(True, c.crit_on_s, c.crit_hz),
            Step(False, c.crit_pause_s)]


def sleep_precise(seconds):
    if seconds <= 0:
        return
    end = time.perf_counter() + seconds
    coarse = seconds - 0.0003
    if coarse > 0:
        time.sleep(coarse)
    while time.perf_counter() < end:
        pass


class Buzzer:
    """Turns alert LEVELS into buzzer PATTERNS on its own thread."""

    def __init__(self, line: GpioLine, cfg: Config, quiet=False):
        self.line = line
        self.cfg = cfg
        self.quiet = quiet
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

    def set_critical(self, on: bool):
        if on:
            self._warn_pending.clear()
            self._critical.set()
        else:
            self._critical.clear()

    def pulse_warn(self):
        if not self._critical.is_set():
            self._warn_pending.set()

    def _drive(self, on: bool):
        now = time.monotonic()
        force = (now - self._last_force) >= self.cfg.buzzer_reassert_s
        if force:
            self._last_force = now
        self.line.set(on, force=force)

    def _hold(self, seconds, abort=None, level=None):
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
            self._drive(False)
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


# =============================================================================
#  3b. VEHICLE SPEED INPUT  (v6)
# =============================================================================
class VehicleSpeed:
    """
    Thread-safe mailbox for the latest vehicle speed.

    The RX thread writes, the detection loop reads. It stores the TIME of the
    reading alongside the value, because a speed with no age is a lie: a
    cable that fell out ten minutes ago still "reads" 110 km/h forever.

    Plausibility is checked here, at the door, so nothing downstream ever
    sees a NaN or a 900 km/h glitch.
    """

    def __init__(self, max_kmh=250.0):
        self._lock = threading.Lock()
        self.max_kmh = max_kmh
        self._kmh = None
        self._stamp = 0.0
        self.received = 0
        self.rejected = 0
        self.peak = 0.0

    def push(self, kmh):
        try:
            v = float(kmh)
        except (TypeError, ValueError):
            self.rejected += 1
            return False
        if math.isnan(v) or v < 0.0 or v > self.max_kmh:
            self.rejected += 1
            return False
        with self._lock:
            self._kmh = v
            self._stamp = time.monotonic()
            self.received += 1
            self.peak = max(self.peak, v)
        return True

    def get(self, now=None):
        """Return (kmh, age_s). kmh is None if nothing ever arrived."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._kmh is None:
                return None, float("inf")
            return self._kmh, now - self._stamp


# =============================================================================
#  3c. SPEED SLABS  (v7)
# =============================================================================
INF = float("inf")
NEVER = 10 ** 9          # a repetition count that can never be reached

SLAB_LEVEL_KEYS = ("notice", "warning", "alarm", "intervention")
SLAB_TIME_KEYS = SLAB_LEVEL_KEYS + ("unheeded",)
SLAB_COUNT_KEYS = ("warn_n", "alarm_n", "intervene_n")
SLAB_KEYS = SLAB_TIME_KEYS + SLAB_COUNT_KEYS

# slab key -> (Config attribute it drives, Config enable flag or None)
SLAB_MAP = {
    "notice":       ("notice_s", "notice_on"),
    "warning":      ("microsleep_warn_s", "warn_on"),
    "alarm":        ("sleep_s", "alarm_on"),
    "intervention": ("deep_sleep_s", "intervene_on"),
    "unheeded":     ("intervene_after_s", None),
    "warn_n":       ("micro_warn_n", None),
    "alarm_n":      ("micro_alarm_n", None),
    "intervene_n":  ("micro_intervene_n", None),
}


def _parse_slab_value(key, raw):
    """'off' / null -> None.  times -> float > 0.  counts -> int >= 1."""
    if raw is None:
        return None
    if isinstance(raw, str):
        s = raw.strip().lower()
        if s in ("off", "none", "null", "-", ""):
            return None
        raw = s
    if key in SLAB_COUNT_KEYS:
        v = int(float(raw))
        if v < 1:
            raise ValueError(f"{key} must be >= 1 or off (got {raw})")
        return v
    v = float(raw)
    if not v > 0 or math.isinf(v):
        raise ValueError(f"{key} must be a positive number of seconds or off "
                         f"(got {raw})")
    return v


@dataclass
class Slab:
    name: str
    from_kmh: float
    values: dict

    def fmt(self, key):
        v = self.values.get(key)
        if v is None:
            return "off"
        return f"{v:d}" if key in SLAB_COUNT_KEYS else f"{v:.2f}s"


class SlabTable:
    """
    The speed bands and what each level needs inside each band.

    Every value is either a number or OFF:
      notice / warning / alarm / intervention
          seconds of continuous eye closure that raise that level.
          OFF = that level cannot happen in this slab, by ANY route
          (closure, repetition or trend).
      unheeded      seconds an ALARM may sound before INTERVENTION.
                    OFF = that route is disabled.
      warn_n / alarm_n / intervene_n
                    microsleeps inside micro_window_s that raise that level.
                    OFF = that repetition route is disabled.

    A slab starts at from_kmh and ends where the next slab starts.
    """

    def __init__(self, slabs, hysteresis_kmh=3.0, down_dwell_s=5.0,
                 fallback="MID"):
        self.slabs = list(slabs)
        self.hysteresis_kmh = float(hysteresis_kmh)
        self.down_dwell_s = float(down_dwell_s)
        self.fallback = str(fallback).upper()

    # ---- construction ------------------------------------------------------
    @classmethod
    def default(cls, cfg: Config):
        """
        MUTED  0-5     everything off
        LOW    5-40    about 1.5x more rope than your tuned ladder
        MID    40-80   exactly your tuned ladder (--sensitivity applies here)
        HIGH   80+     about 25% quicker, with floors so it is never twitchy
        """
        r = lambda x: round(x, 2)
        fl = lambda base, floor: min(base, floor)   # never above the base
        b = cfg
        mid = dict(notice=b.notice_s, warning=b.microsleep_warn_s,
                   alarm=b.sleep_s, intervention=b.deep_sleep_s,
                   unheeded=b.intervene_after_s, warn_n=b.micro_warn_n,
                   alarm_n=b.micro_alarm_n, intervene_n=b.micro_intervene_n)
        low = dict(notice=r(b.notice_s * 1.5),
                   warning=r(b.microsleep_warn_s * 1.5),
                   alarm=r(b.sleep_s * 1.5),
                   intervention=r(b.deep_sleep_s * 1.4),
                   unheeded=r(b.intervene_after_s * 1.5),
                   warn_n=b.micro_warn_n + 1, alarm_n=b.micro_alarm_n + 1,
                   intervene_n=b.micro_intervene_n + 1)
        high = dict(notice=b.notice_s,
                    warning=r(max(b.microsleep_warn_s * 0.75,
                                  fl(b.microsleep_warn_s, b.microsleep_s + 0.10))),
                    alarm=r(max(b.sleep_s * 0.75, fl(b.sleep_s, 1.20))),
                    intervention=r(max(b.deep_sleep_s * 0.75,
                                       fl(b.deep_sleep_s, 2.00))),
                    unheeded=r(max(b.intervene_after_s * 0.67,
                                   fl(b.intervene_after_s, 1.50))),
                    warn_n=b.micro_warn_n, alarm_n=b.micro_alarm_n,
                    intervene_n=b.micro_intervene_n)
        muted = {k: None for k in SLAB_KEYS}
        return cls([Slab("MUTED", 0.0, muted),
                    Slab("LOW", 5.0, low),
                    Slab("MID", 40.0, mid),
                    Slab("HIGH", 80.0, high)],
                   hysteresis_kmh=cfg.slab_hysteresis_kmh,
                   down_dwell_s=cfg.slab_down_dwell_s)

    def names(self):
        return [s.name for s in self.slabs]

    def find(self, name):
        name = str(name).upper()
        for i, s in enumerate(self.slabs):
            if s.name == name:
                return i
        raise KeyError(f"no slab named {name} (have: {', '.join(self.names())})")

    def index_for(self, kmh):
        idx = 0
        for i, s in enumerate(self.slabs):
            if kmh >= s.from_kmh:
                idx = i
        return idx

    # ---- editing -----------------------------------------------------------
    def load_json(self, path):
        """
        Keys you leave out of a slab are inherited from the default slab of
        the same name (or from MID for a new name). Write null / "off" to
        switch something off -- leaving it out does NOT switch it off.
        """
        with open(path) as fh:
            data = json.load(fh)
        if not isinstance(data, dict):
            raise ValueError("slab file must be a JSON object")
        for k in data:
            if k not in ("hysteresis_kmh", "down_dwell_s", "fallback", "slabs"):
                raise ValueError(f"unknown top-level key '{k}'")
        if "hysteresis_kmh" in data:
            self.hysteresis_kmh = float(data["hysteresis_kmh"])
        if "down_dwell_s" in data:
            self.down_dwell_s = float(data["down_dwell_s"])
        if "fallback" in data:
            self.fallback = str(data["fallback"]).upper()
        if "slabs" in data:
            known = {s.name: dict(s.values) for s in self.slabs}
            inherit = known.get("MID") or dict(self.slabs[-1].values)
            new = []
            for i, e in enumerate(data["slabs"]):
                name = str(e.get("name", f"SLAB{i}")).upper()
                if "from_kmh" not in e:
                    raise ValueError(f"slab {name}: 'from_kmh' is required")
                for k in e:
                    if k not in ("name", "from_kmh") and k not in SLAB_KEYS:
                        raise ValueError(f"slab {name}: unknown key '{k}' "
                                         f"(allowed: {', '.join(SLAB_KEYS)})")
                vals = dict(known.get(name, inherit))
                for k in SLAB_KEYS:
                    if k in e:
                        vals[k] = _parse_slab_value(k, e[k])
                new.append(Slab(name, float(e["from_kmh"]), vals))
            self.slabs = new

    def set_edges(self, text):
        """'5,40,80' -> the lower edges of every slab after the first."""
        edges = [float(x) for x in str(text).split(",") if x.strip()]
        if len(edges) != len(self.slabs) - 1:
            raise ValueError(f"--slab-edges needs {len(self.slabs) - 1} values "
                             f"for {len(self.slabs)} slabs, got {len(edges)}")
        for s, e in zip(self.slabs[1:], edges):
            s.from_kmh = e

    def override(self, spec):
        """'HIGH:alarm=1.2,intervention=off,from=85'"""
        if ":" not in spec:
            raise ValueError(f"--slab '{spec}': expected NAME:key=value,...")
        name, body = spec.split(":", 1)
        slab = self.slabs[self.find(name)]
        for item in body.split(","):
            if not item.strip():
                continue
            if "=" not in item:
                raise ValueError(f"--slab '{spec}': '{item}' is not key=value")
            k, v = (x.strip().lower() for x in item.split("=", 1))
            if k in ("from", "from_kmh"):
                slab.from_kmh = float(v)
            elif k in SLAB_KEYS:
                slab.values[k] = _parse_slab_value(k, v)
            else:
                raise ValueError(f"--slab '{spec}': unknown key '{k}' "
                                 f"(allowed: from, {', '.join(SLAB_KEYS)})")

    # ---- checking ----------------------------------------------------------
    def validate(self):
        """Return (errors, warnings). Errors stop the program."""
        err, warn = [], []
        if not self.slabs:
            return ["no slabs defined"], warn
        if self.slabs[0].from_kmh != 0:
            err.append(f"first slab ({self.slabs[0].name}) must start at 0 km/h")
        names = self.names()
        if len(set(names)) != len(names):
            err.append("slab names must be unique")
        for a, b in zip(self.slabs, self.slabs[1:]):
            if b.from_kmh <= a.from_kmh:
                err.append(f"slab edges must increase: {a.name} starts at "
                           f"{a.from_kmh:g}, {b.name} at {b.from_kmh:g}")
        if self.fallback not in names:
            err.append(f"fallback slab '{self.fallback}' does not exist")
        if self.hysteresis_kmh < 0:
            err.append("hysteresis must be >= 0")
        widths = [b.from_kmh - a.from_kmh for a, b in zip(self.slabs, self.slabs[1:])]
        if widths and self.hysteresis_kmh >= min(widths):
            warn.append(f"hysteresis {self.hysteresis_kmh:g} km/h is as wide as "
                        f"the narrowest slab ({min(widths):g} km/h)")
        if self.down_dwell_s < 1.0:
            warn.append(f"down-shift dwell {self.down_dwell_s:g}s is short; "
                        f"slabs may flap at a boundary")

        for s in self.slabs:
            v = s.values
            prev_k, prev_v = None, None
            for k in SLAB_LEVEL_KEYS:
                x = v.get(k)
                if x is None:
                    continue
                if prev_v is not None:
                    bad = x < prev_v if prev_k == "notice" else x <= prev_v
                    if bad:
                        err.append(f"{s.name}: {k} ({x:g}s) must be "
                                   f"{'>=' if prev_k == 'notice' else '>'} "
                                   f"{prev_k} ({prev_v:g}s)")
                prev_k, prev_v = k, x
            counts = [(k, v.get(k)) for k in SLAB_COUNT_KEYS if v.get(k) is not None]
            for (ka, a), (kb, b) in zip(counts, counts[1:]):
                if b < a:
                    err.append(f"{s.name}: {kb} ({b}) must be >= {ka} ({a})")

            levels_on = [k for k in SLAB_LEVEL_KEYS if v.get(k) is not None]
            if not levels_on and s.from_kmh >= 10:
                warn.append(f"{s.name}: every level is OFF from "
                            f"{s.from_kmh:g} km/h -- the driver is unmonitored")
            iv = v.get("intervention")
            if iv is not None and iv < 1.5:
                warn.append(f"{s.name}: intervention at {iv:g}s of shut eyes is "
                            f"trigger-happy for a speed limiter")
            if iv is not None and v.get("alarm") is None:
                warn.append(f"{s.name}: intervention is ON but alarm is OFF -- "
                            f"the limiter will engage with no buzzer first")
            for k in SLAB_COUNT_KEYS:
                lvl = {"warn_n": "warning", "alarm_n": "alarm",
                       "intervene_n": "intervention"}[k]
                if v.get(k) is not None and v.get(lvl) is None:
                    warn.append(f"{s.name}: {k} is set but {lvl} is OFF, so it "
                                f"has no effect")
        return err, warn

    def fit_to_fps(self, microsleep_s):
        """
        After the fps autotune: no closure threshold may be shorter than the
        microsleep definition (it would rest on too few frames), and the
        ladder in each slab must still climb.
        """
        changes = []
        for s in self.slabs:
            prev = None
            for k in SLAB_LEVEL_KEYS:
                x = s.values.get(k)
                if x is None:
                    continue
                new = max(x, microsleep_s)
                if prev is not None and k != "notice":
                    new = max(new, round(prev + 0.10, 2))
                if abs(new - x) > 1e-9:
                    changes.append(f"{s.name}.{k}: {x:.2f}s -> {new:.2f}s "
                                   f"(frame rate / ordering)")
                    s.values[k] = new
                prev = new
        return changes

    # ---- output ------------------------------------------------------------
    def to_dict(self):
        return {
            "hysteresis_kmh": self.hysteresis_kmh,
            "down_dwell_s": self.down_dwell_s,
            "fallback": self.fallback,
            "slabs": [dict(name=s.name, from_kmh=s.from_kmh,
                           **{k: s.values.get(k) for k in SLAB_KEYS})
                      for s in self.slabs],
        }

    def save_json(self, path):
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh, indent=2)
            fh.write("\n")

    def table_text(self, active=None):
        w = 92
        rows = ["", "speed slabs -- seconds of continuous eye closure that raise "
                "each level", "=" * w,
                f"  {'slab':<8} {'km/h':<10} {'NOTICE':>8} {'WARNING':>8} "
                f"{'ALARM':>8} {'INTERV':>8} {'unheeded':>9} | "
                f"{'repeats w/a/i':>14}",
                "-" * w]
        for i, s in enumerate(self.slabs):
            hi = (f"{self.slabs[i + 1].from_kmh:g}" if i + 1 < len(self.slabs)
                  else "")
            rng = f"{s.from_kmh:g}-{hi}" if hi else f"{s.from_kmh:g}+"
            reps = "/".join(s.fmt(k) for k in SLAB_COUNT_KEYS)
            mark = ">" if active == s.name else " "
            rows.append(f"{mark} {s.name:<8} {rng:<10} "
                        + " ".join(f"{s.fmt(k):>8}" for k in SLAB_LEVEL_KEYS)
                        + f" {s.fmt('unheeded'):>9} | {reps:>14}")
        rows += ["-" * w,
                 "  off = that level cannot be reached in this slab by any route.",
                 "  unheeded = seconds an ALARM may sound before INTERVENTION.",
                 "  repeats = microsleeps inside the repetition window for "
                 "WARNING / ALARM / INTERVENTION.",
                 f"  faster slab: applied at once.  slower slab: speed must be "
                 f"{self.hysteresis_kmh:g} km/h below the edge for "
                 f"{self.down_dwell_s:g}s, and never during an event.",
                 f"  no speed reading -> {self.fallback}.",
                 "=" * w, ""]
        return "\n".join(rows)


class SlabSelector:
    """
    Picks the active slab from the vehicle speed and writes its values into
    the shared Config. The detector and the policy read Config live, so they
    follow the slab without knowing slabs exist.

    Safety rules:
      1. Into a FASTER slab: immediately.
      2. Into a SLOWER slab: only after the speed has been below the edge
         minus the hysteresis for down_dwell_s, AND never while an event is
         in progress (eyes shut, or level >= WARNING). Braking must not
         silence an alarm that is sounding.
      3. No speed for speed_timeout_s: the fallback slab (MID by default).
         Note that this UN-mutes a parked car whose speed link has died --
         a dead sensor must never read as "stopped, stay quiet".
    """

    def __init__(self, cfg: Config, table: SlabTable, speed: VehicleSpeed = None,
                 fixed_kmh=None):
        self.cfg = cfg
        self.table = table
        self.speed = speed
        self.fixed_kmh = fixed_kmh
        self.idx = table.find(table.fallback)
        self.kmh = None
        self.stale = True
        self.armed = False
        self.pending = None
        self.switches = 0
        self.time_in = {s.name: 0.0 for s in table.slabs}
        self._last_t = None
        self._down_want = None
        self._down_since = None
        self._baseline = {}

    @property
    def slab(self):
        return self.table.slabs[self.idx]

    def arm(self):
        """Call once, after calibration and fps autotune."""
        self._baseline = {attr: getattr(self.cfg, attr)
                          for attr, _ in SLAB_MAP.values()}
        self.armed = True
        self._apply()

    def baseline(self, attr):
        return self._baseline.get(attr, INF)

    def _apply(self):
        c, vals = self.cfg, self.slab.values
        for key, (attr, flag) in SLAB_MAP.items():
            v = vals.get(key)
            if key in SLAB_COUNT_KEYS:
                setattr(c, attr, NEVER if v is None else int(v))
            elif key == "unheeded":
                setattr(c, attr, INF if v is None else float(v))
            else:
                # An OFF level keeps a finite time (the baseline) so display
                # labels and event counting still work; the FLAG is what
                # stops the policy from ever entering it.
                setattr(c, flag, v is not None)
                setattr(c, attr, float(v) if v is not None
                        else self._baseline[attr])

    def _switch(self, want, now):
        old = self.slab.name
        self.idx = want
        self._down_want = self._down_since = None
        self.pending = None
        self.switches += 1
        self._apply()
        spd = "no speed" if self.kmh is None else f"{self.kmh:.0f} km/h"
        return f"[slab] {old} -> {self.slab.name}  ({spd})"

    def update(self, now, hold=False):
        """Returns a list of console messages (usually empty)."""
        if not self.armed:
            return []
        c, t = self.cfg, self.table
        msgs = []
        dt = 0.0 if self._last_t is None else min(1.0, max(0.0, now - self._last_t))
        self._last_t = now
        self.time_in[self.slab.name] = self.time_in.get(self.slab.name, 0.0) + dt

        if self.fixed_kmh is not None:
            raw, stale = float(self.fixed_kmh), False
        elif self.speed is not None:
            raw, age = self.speed.get(now)
            stale = raw is None or age > c.speed_timeout_s
        else:
            raw, stale = None, True

        if stale != self.stale:
            msgs.append(f"[speed] no reading for >{c.speed_timeout_s:.0f}s -- "
                        f"using slab {t.fallback}" if stale
                        else f"[speed] receiving: {raw:.0f} km/h")
            self.stale = stale

        if stale:
            self.kmh = None
            want = t.find(t.fallback)
        else:
            if self.kmh is None or dt <= 0.0:
                self.kmh = raw
            else:
                a = 1.0 - math.exp(-dt / max(1e-3, c.speed_smooth_s))
                self.kmh += a * (raw - self.kmh)
            want = t.index_for(self.kmh)
            # hysteresis: stay put until clearly below the current slab's edge
            if want < self.idx and \
               self.kmh > self.slab.from_kmh - t.hysteresis_kmh:
                want = self.idx

        if want > self.idx:
            msgs.append(self._switch(want, now))
        elif want < self.idx:
            self.pending = want
            if hold:
                self._down_want = None          # the dwell restarts afterwards
            elif self._down_want != want:
                self._down_want, self._down_since = want, now
            elif now - self._down_since >= t.down_dwell_s:
                msgs.append(self._switch(want, now))
        else:
            self._down_want = self._down_since = None
            self.pending = None
        return msgs

    def vehicle_text(self, now=None):
        """The REAL vehicle speed from RX, whatever picks the slab."""
        if self.speed is None:
            return "vehicle n/a"
        v, age = self.speed.get(now)
        if v is None:
            return "vehicle: nothing received"
        if age > self.cfg.speed_timeout_s:
            return f"vehicle: last {v:.0f} km/h, {age:.0f}s old"
        return f"vehicle {v:.0f} km/h"

    def describe(self):
        if self.fixed_kmh is not None:
            return (f"{self.slab.name} @ bench {self.fixed_kmh:g} km/h, "
                    f"{self.vehicle_text()}")
        spd = "speed n/a" if self.kmh is None else f"{self.kmh:.0f} km/h"
        s = f"{self.slab.name} @ {spd}"
        if self.pending is not None and self.pending != self.idx:
            s += f" (-> {self.table.slabs[self.pending].name} pending)"
        return s


class SpeedSerial:
    """
    One serial port, two directions.

      TX  a raw speed-limit byte while the policy holds a level that commands
          one. Only the TX thread ever writes.
      RX  the vehicle speed, parsed into a VehicleSpeed mailbox. Only the RX
          thread ever reads.

    One reader thread plus one writer thread on the same pyserial object is
    safe on Linux: read() and write() are independent syscalls on the fd.
    Two WRITERS would not be, which is why the shutdown byte is also sent
    from the TX thread.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=9600, normal_speed=None,
                 resend_s=1.0, quiet=False, log_every_s=5.0,
                 rx_format=None, speed_in: VehicleSpeed = None):
        self.port_name = port
        self.baud = baud
        self.normal_speed = None if normal_speed is None else int(normal_speed)
        self.resend_s = resend_s
        self.quiet = quiet
        self.log_every_s = log_every_s
        self.rx_format = rx_format
        self.speed_in = speed_in
        self.ser = None
        self.sent = 0
        self.rx_bytes = 0
        self.rx_errors = 0
        self.ok = False

        self._target = None
        self._tlock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._shutdown_done = threading.Event()
        self._logged_value = None
        self._logged_at = 0.0
        self._thread = None
        self._rx_thread = None

        try:
            import serial
        except ImportError:
            sys.stderr.write("pyserial not installed -- no serial link.\n"
                             "  sudo apt install python3-serial\n")
            return

        try:
            # timeout is the READ timeout: short, so the RX thread notices a
            # shutdown within 0.2 s instead of sitting in read() for a second.
            self.ser = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.2, write_timeout=1)
            try:
                self.ser.reset_input_buffer()   # drop whatever queued before us
            except Exception:
                pass
            self.ok = True
        except Exception as e:
            sys.stderr.write(f"serial port {port} unavailable ({e}) -- "
                             f"continuing without it\n")
            return

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        if self.rx_format and self.speed_in is not None:
            self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self._rx_thread.start()

    # ---- TX -----------------------------------------------------------------
    def set_target(self, value):
        with self._tlock:
            if value == self._target:
                return
            self._target = value
        self._wake.set()

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
            if self.normal_speed is not None:
                self._send(self.normal_speed, "  (shutdown)")
        finally:
            self._shutdown_done.set()

    # ---- RX -----------------------------------------------------------------
    def _rx_worker(self):
        """
        byte  : every byte is km/h; only the NEWEST byte of each read counts.
                No framing, so any debug text from the AVR will be misread
                ('7' is 55 km/h). Keep that line clean, or use ascii.
        ascii : newline-framed; the last number on a line is km/h.
        """
        buf = bytearray()
        num = re.compile(rb"\d+(?:\.\d+)?")
        while not self._stop.is_set():
            try:
                waiting = self.ser.in_waiting
                data = self.ser.read(waiting if waiting else 1)
            except Exception as e:
                self.rx_errors += 1
                if self.rx_errors <= 3:
                    sys.stderr.write(f"serial read failed: {e}\n")
                self._stop.wait(0.5)
                continue
            if not data:
                continue
            self.rx_bytes += len(data)

            if self.rx_format == "byte":
                self.speed_in.push(data[-1])
                continue

            buf += data
            if len(buf) > 512:                 # no newline ever came: resync
                buf = bytearray(buf[-64:])
                self.speed_in.rejected += 1
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                line = bytes(buf[:i])
                del buf[:i + 1]
                found = num.findall(line)
                if found:
                    self.speed_in.push(float(found[-1]))
                elif line.strip():
                    self.speed_in.rejected += 1

    def shutdown(self):
        self.set_target(None)
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._shutdown_done.wait(timeout=2.0)
            self._thread.join(timeout=0.5)
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
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
    Everything the policy is allowed to see.
      closure_s   HARD  -- a measurement of what is true right now
      trend_risk  TREND -- an inference about the last minute, capped
    """
    t: float = 0.0
    face: bool = True
    closure_s: float = 0.0
    eyes_open: bool = True
    trend_risk: float = 0.0
    new_microsleep: bool = False
    stare: bool = False
    sensor_fault: bool = False
    status: str = "ALERT"
    reasons: list = field(default_factory=list)


class SafetyPolicy:
    """
    The escalation state machine between the detector and the actuators.
    It reads every threshold from cfg LIVE, which is why the speed adapter
    can move them without this class knowing speed exists.
    """

    NORMAL, NOTICE, WARNING, ALARM, INTERVENE = 0, 1, 2, 3, 4
    NAMES = {0: "NORMAL", 1: "NOTICE", 2: "WARNING",
             3: "ALARM", 4: "INTERVENTION"}

    def __init__(self, cfg: Config, hard_only=False):
        self.cfg = cfg
        self.hard_only = hard_only
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

    def note_microsleep(self, now):
        self._micro.append(now)
        self._last_micro = now

    def micro_count(self):
        return len(self._micro)

    def allowed(self, level):
        """Does the active slab permit this level at all?"""
        c = self.cfg
        return {self.NORMAL: True, self.NOTICE: c.notice_on,
                self.WARNING: c.warn_on, self.ALARM: c.alarm_on,
                self.INTERVENE: c.intervene_on}[level]

    def _evidence(self, now, ev: Evidence):
        """
        Map one frame onto (level, is_hard, why).

        Every route is gated by the active slab's enable flag, so an OFF
        level cannot be reached by closure, by repetition, or by trend.
        When a level is OFF the check simply falls through to the next one
        down: 4 s of shut eyes in a slab with intervention OFF is an ALARM.
        """
        c = self.cfg

        if ev.sensor_fault:
            if c.warn_on:
                return self.WARNING, False, "eye signal stuck -- SENSOR FAULT"
            if c.notice_on:
                return self.NOTICE, False, "eye signal stuck -- SENSOR FAULT"
            return self.NORMAL, False, ""

        n = self.micro_count()
        fresh = (self._last_micro is not None
                 and (now - self._last_micro) <= c.micro_sustain_s)
        cs = ev.closure_s

        # ---- HARD channel ---------------------------------------------------
        if c.intervene_on and cs >= c.deep_sleep_s:
            return self.INTERVENE, True, f"eyes shut {cs:.1f}s (unrousable)"
        if c.alarm_on and cs >= c.sleep_s:
            return self.ALARM, True, f"eyes shut {cs:.1f}s"
        if c.warn_on and cs >= c.microsleep_warn_s:
            return self.WARNING, True, f"microsleep {cs:.2f}s"
        if c.warn_on and cs >= c.microsleep_s and n >= c.micro_warn_n:
            return self.WARNING, True, f"microsleep {cs:.2f}s (#{n})"
        if c.notice_on and cs >= c.notice_s:
            return self.NOTICE, True, f"microsleep {cs:.2f}s"

        # ---- REPETITION channel --------------------------------------------
        if fresh:
            if c.intervene_on and n >= c.micro_intervene_n:
                return (self.INTERVENE, True,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")
            if c.alarm_on and n >= c.micro_alarm_n:
                return (self.ALARM, True,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")
            if c.warn_on and n >= c.micro_warn_n:
                return (self.WARNING, False,
                        f"{n} microsleeps in {c.micro_window_s:.0f}s")

        # ---- TREND channel: capped, WARNING is its ceiling ------------------
        if self.hard_only:
            return self.NORMAL, False, ""
        thr = c.risk_warn_exit if self.level >= self.WARNING else c.risk_warn
        why = ev.reasons[0] if ev.reasons else f"fatigue trend {ev.trend_risk:.0f}"
        if c.warn_on and ev.trend_risk >= thr:
            return self.WARNING, False, why
        if c.notice_on and ev.trend_risk >= c.risk_notice:
            return self.NOTICE, False, why
        return self.NORMAL, False, ""

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
            self._micro.clear()
        self.log.append((time.strftime("%H:%M:%S"),
                         f"{self.NAMES[old]} -> {self.NAMES[new]}  ({why})"))
        return self.level, why

    def update(self, now, ev: Evidence):
        """Feed one frame. Returns (level, reason)."""
        c = self.cfg

        # ---- 0. the active slab no longer permits the current level --------
        # Happens after a slab change. Drop straight to what the evidence
        # justifies under the NEW slab (which is permitted by construction).
        if self.level > self.NORMAL and not self.allowed(self.level):
            if ev.face:
                raw0 = self._evidence(now, ev)[0]
            else:
                raw0 = self.NORMAL
            return self._transition(raw0, now, "slab change -- level not "
                                               "permitted here")

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

        for L in (1, 2, 3, 4):
            if raw >= L:
                self._at_least.setdefault(L, now)
            else:
                self._at_least.pop(L, None)

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

        if c.intervene_on and self.level == self.ALARM and \
           raw >= self.ALARM and not ev.eyes_open:
            held = now - self.level_since
            if held >= c.intervene_after_s:
                cand, cand_why = self.INTERVENE, f"alarm unheeded for {held:.1f}s"

        if cand > self.level:
            return self._transition(cand, now, cand_why)

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
        if self.level == self.ALARM and self.cfg.intervene_on and \
           math.isfinite(self.cfg.intervene_after_s):
            left = max(0.0, self.cfg.intervene_after_s - held)
            s += f" (intervenes in {left:.1f}s)"
        elif self.level == self.INTERVENE:
            left = max(0.0, self.cfg.intervene_min_hold_s - held)
            if left > 0:
                s += f" (min hold {left:.1f}s)"
        if self.micro_count():
            s += f" [{self.micro_count()} ms/{self.cfg.micro_window_s:.0f}s]"
        return s


def explain_ladder(c: Config, hard_only=False, speed_on=False):
    # With speed input, this is the MID / fallback ladder only.
    L = [
        ("NOTICE", "console only", [
            f"trend risk >= {c.risk_notice:.0f}",
            f"one microsleep {c.notice_s:.2f}-{c.microsleep_warn_s:.2f}s",
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
    print("\nescalation ladder -- what it takes to reach each level"
          + ("  (baseline / MID slab)" if speed_on else ""))
    print("=" * 66)
    for name, effect, routes in L:
        print(f"  {name:<13} {effect}")
        for r in routes:
            print(f"        - {r}")
    print(f"\n  trend evidence is capped at {c.trend_cap:.0f} and can never, on its "
          f"own,\n  reach ALARM or INTERVENTION.")
    print(f"\n  an unbroken closure past {c.latch_fault_s:.0f}s with the head "
          f"still moving is\n  treated as a stuck signal, not sleep.")
    if speed_on:
        print("\n  speed slabs are ON: each slab replaces these values and can "
              "switch\n  any level off. See --explain-slabs.")
    if hard_only:
        print("  --hard-only is set: trend evidence cannot alert at all.")
    print("=" * 66 + "\n")


class Alerter:
    """Console reporting + actuator dispatch. No thresholds live here."""

    def __init__(self, cooldown_s, buzzer, quiet=False, noface_delay_s=2.0,
                 noface_enabled=True, speed=None,
                 intervene_speed=30, caution_speed=None,
                 led=None, led_hold_s=0.5, led_reassert_s=2.0,
                 no_tx_reason=None):
        self.cooldown = cooldown_s
        self.no_tx_reason = no_tx_reason
        self._no_tx_warned = False
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
            self._face_seen = False
            if self.noface_enabled:
                self._say(f"[ALERT/no-face] driver not visible for "
                          f"{self.noface_delay:.0f}s  (silent)")

    def _update_led(self, face_present):
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
        # An intervention that cannot reach the vehicle must SAY so. The
        # buzzer still sounds; only the byte is missing.
        if level >= SafetyPolicy.INTERVENE and not self.speed:
            if not self._no_tx_warned:
                self._no_tx_warned = True
                sys.stderr.write(f"[ALERT/intervention] speed byte NOT sent: "
                                 f"{self.no_tx_reason or 'no serial link'}\n")
                sys.stderr.flush()
        elif level < SafetyPolicy.INTERVENE:
            self._no_tx_warned = False
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

    def _open_device(self):
        raise NotImplementedError

    def _read_frame(self):
        raise NotImplementedError

    def _close_device(self):
        raise NotImplementedError

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
    """OV5647 via libcamera. "RGB888" in picamera2 yields a BGR array."""

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
        time.sleep(0.5)
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
            except AttributeError:
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
        open_cluster = eyes[eyes <= eye_open + 0.12]
        eye_open_hi = float(np.percentile(open_cluster, 95)) \
            if open_cluster.size >= 5 else eye_open + 0.10

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
        self.all = []

    def progress(self, t):
        if self.done or self.t0 is None:
            return 1.0
        by_time = (t - self.t0) / self.seconds
        by_count = len(self.all) / max(1, self.min_samples)
        return min(1.0, min(by_time, by_count))


def fit_eye_thresholds(cfg: Config, eye_open_hi: float, allow_lower=False):
    want_on = min(0.90, max(0.45, eye_open_hi + 0.18))
    if want_on <= cfg.close_on and not allow_lower:
        return None
    if abs(want_on - cfg.close_on) < 0.03:
        return None
    old_on, old_off = cfg.close_on, cfg.close_off
    cfg.close_on = round(want_on, 2)
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
    risk: float = 0.0
    trend_risk: float = 0.0
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
        self._micro_latched = False
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
                if not self._stare_latched:
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

        closure_s, new_micro = self._lid_closure(f)

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
                self.trend_risk = target
            else:
                self.trend_risk = max(target, self.trend_risk - decay * dt)
        else:
            self.trend_risk = max(0.0, self.trend_risk - decay * dt)
        self.trend_risk = min(self.trend_risk, c.trend_cap)

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
                    default="auto")
    ap.add_argument("--camera-num", type=int, default=0)
    ap.add_argument("--camera", default="0",
                    help="v4l2 backend only: index or /dev/videoN")
    ap.add_argument("--video", default=None, help="run on a file instead")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--swap-rb", action="store_true")
    ap.add_argument("--vflip", action="store_true")
    ap.add_argument("--hflip", action="store_true")
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--infer-width", type=int, default=384)
    ap.add_argument("--camera-timeout", type=float, default=6.0)
    ap.add_argument("--test-camera", action="store_true")
    ap.add_argument("--explain", action="store_true",
                    help="print the escalation ladder at startup")
    ap.add_argument("--max-frame-errors", type=int, default=10)

    # ---- buzzer wiring -----------------------------------------------------
    ap.add_argument("--gpiochip", default=None)
    ap.add_argument("--gpio-line", type=int, default=23)
    ap.add_argument("--buzzer-active-low", action="store_true")
    ap.add_argument("--no-buzzer", action="store_true")
    ap.add_argument("--test-buzzer", action="store_true")
    ap.add_argument("--tone-mode", choices=("solid", "pwm"), default=CFG.tone_mode)
    ap.add_argument("--buzzer-debug", action="store_true")
    ap.add_argument("--buzzer-reassert", type=float, default=CFG.buzzer_reassert_s)

    # ---- face-presence LED -------------------------------------------------
    ap.add_argument("--led-chip", default=None)
    ap.add_argument("--led-line", type=int, default=24)
    ap.add_argument("--led-active-low", action="store_true")
    ap.add_argument("--no-led", action="store_true")
    ap.add_argument("--test-led", action="store_true")
    ap.add_argument("--led-hold", type=float, default=CFG.led_hold_s)

    # ---- buzzer pattern shape ---------------------------------------------
    ap.add_argument("--warn-beep", type=float, default=CFG.warn_on_s)
    ap.add_argument("--warn-cooldown", type=float, default=CFG.alarm_cooldown_s)
    ap.add_argument("--crit-on", type=float, default=CFG.crit_on_s)
    ap.add_argument("--crit-gap", type=float, default=CFG.crit_gap_s)
    ap.add_argument("--crit-pause", type=float, default=CFG.crit_pause_s)
    ap.add_argument("--no-noface-alert", action="store_true")
    ap.add_argument("--noface-delay", type=float, default=CFG.noface_delay_s)

    # ---- the safety ladder -------------------------------------------------
    ap.add_argument("--sensitivity", choices=("relaxed", "normal", "strict"),
                    default="normal")
    ap.add_argument("--close-on", type=float, default=None)
    ap.add_argument("--close-off", type=float, default=None)
    ap.add_argument("--no-auto-eye-threshold", dest="auto_eye",
                    action="store_false")
    ap.add_argument("--auto-eye-lower", action="store_true")
    ap.add_argument("--microsleep", type=float, default=None)
    ap.add_argument("--microsleep-warn", type=float, default=None)
    ap.add_argument("--sleep", dest="sleep_s", type=float, default=None)
    ap.add_argument("--deep-sleep", type=float, default=None)
    ap.add_argument("--micro-warn-n", type=int, default=None)
    ap.add_argument("--micro-alarm-n", type=int, default=None)
    ap.add_argument("--micro-intervene-n", type=int, default=None)
    ap.add_argument("--micro-window", type=float, default=None)
    ap.add_argument("--intervene-after", type=float, default=None)
    ap.add_argument("--intervene-min-hold", type=float, default=None)
    ap.add_argument("--trend-cap", type=float, default=None)
    ap.add_argument("--latch-fault", type=float, default=None)
    ap.add_argument("--hard-only", "--buzzer-hard-only", dest="hard_only",
                    action="store_true")

    # ---- speed slabs (v7) ----------------------------------------------------
    sp = ap.add_argument_group("speed slabs")
    sp.add_argument("--speed-rx", choices=("byte", "ascii"), default="byte",
                    help="format of the vehicle speed on the serial RX line "
                         "(always read)")
    sp.add_argument("--fixed-speed", type=float, default=None,
                    help="BENCH: choose the slab as if at this km/h; the real "
                         "vehicle speed is still read and printed")
    sp.add_argument("--speed-print-step", type=float, default=5.0,
                    help="print the vehicle speed when it changes by this "
                         "many km/h (0 = only in the heartbeat)")
    sp.add_argument("--slabs", default=None, metavar="FILE",
                    help="load slab definitions from a JSON file")
    sp.add_argument("--slab-edges", default=None, metavar="E1,E2,...",
                    help="lower km/h edge of every slab after the first, "
                         "e.g. 5,40,80")
    sp.add_argument("--slab", action="append", default=[], metavar="SPEC",
                    help="change one slab, e.g. HIGH:alarm=1.2,intervention=off "
                         "or LOW:from=10. Repeatable. Keys: from, "
                         + ", ".join(SLAB_KEYS))
    sp.add_argument("--slab-hysteresis", type=float, default=None,
                    help="km/h below an edge before a slower slab is considered")
    sp.add_argument("--slab-dwell", type=float, default=None,
                    help="seconds a slower slab must persist before it applies")
    sp.add_argument("--slab-fallback", default=None,
                    help="slab used when no speed is known")
    sp.add_argument("--speed-timeout", type=float, default=CFG.speed_timeout_s,
                    help="seconds without a reading before using the fallback slab")
    sp.add_argument("--explain-slabs", action="store_true",
                    help="print the final slab table and exit")
    sp.add_argument("--write-slabs", default=None, metavar="FILE",
                    help="write the current slab table as JSON and exit")

    # ---- serial link -------------------------------------------------------
    ap.add_argument("--serial-port", default="/dev/serial0",
                    help="USB-TTL adapter, or /dev/serial0 for the Pi UART "
                         "(pins 8/10)")
    ap.add_argument("--test-serial", action="store_true",
                    help="send the INTERVENTION byte for 3 s through the same "
                         "code path the detector uses, show any RX speed, exit")
    ap.add_argument("--serial-baud", type=int, default=9600)
    ap.add_argument("--intervene-speed", "--critical-speed", dest="intervene_speed",
                    type=int, default=30,
                    help="raw byte sent repeatedly at INTERVENTION")
    ap.add_argument("--caution-speed", type=int, default=None)
    ap.add_argument("--normal-speed", type=int, default=None)
    ap.add_argument("--serial-interval", type=float, default=1.0)
    ap.add_argument("--serial-log-interval", type=float, default=5.0)
    ap.add_argument("--no-serial", action="store_true",
                    help="BENCH ONLY: do not send the INTERVENTION byte. "
                         "Vehicle speed is still read.")
    ap.add_argument("--quiet-serial", action="store_true")

    ap.add_argument("--quiet-alerts", action="store_true")
    ap.add_argument("--calib", type=float, default=CFG.calib_seconds)
    ap.add_argument("--heartbeat", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0)
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

    CFG.speed_timeout_s = args.speed_timeout
    if args.slab_hysteresis is not None:
        CFG.slab_hysteresis_kmh = args.slab_hysteresis
    if args.slab_dwell is not None:
        CFG.slab_down_dwell_s = args.slab_dwell

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
    CFG.notice_s = CFG.microsleep_s      # baseline: NOTICE = a microsleep

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
                         f"between close_on and close_off.\n")
    if CFG.latch_fault_s <= CFG.deep_sleep_s * 3:
        sys.stderr.write(f"WARNING: latch_fault_s ({CFG.latch_fault_s:.0f}s) is "
                         f"close to deep_sleep_s\n")
    if CFG.trend_cap >= 100.0:
        sys.stderr.write("WARNING: trend_cap is very high\n")
    if args.fixed_speed is not None:
        print(f"note: --fixed-speed {args.fixed_speed:g} km/h picks the slab "
              f"this run; the vehicle speed ({args.speed_rx}) is still read "
              f"and printed")


def build_slab_table(args):
    """
    Defaults (from the baseline ladder) -> JSON file -> --slab-edges ->
    --slab overrides -> --slab-* settings. Later always wins.
    Any error stops the program: a mistyped slab at 2 a.m. must not
    silently become a different ladder.
    """
    table = SlabTable.default(CFG)
    try:
        if args.slabs:
            table.load_json(args.slabs)
        if args.slab_edges:
            table.set_edges(args.slab_edges)
        for spec in args.slab:
            table.override(spec)
    except (OSError, ValueError, KeyError) as e:
        sys.exit(f"slab configuration: {e}")
    if args.slab_hysteresis is not None:
        table.hysteresis_kmh = args.slab_hysteresis
    if args.slab_dwell is not None:
        table.down_dwell_s = args.slab_dwell
    if args.slab_fallback is not None:
        table.fallback = args.slab_fallback.upper()

    errors, warnings = table.validate()
    for w in warnings:
        sys.stderr.write(f"slab WARNING: {w}\n")
    if errors:
        sys.exit("slab configuration is invalid:\n  " + "\n  ".join(errors))

    worst = max((s.values.get("intervention") or 0.0) for s in table.slabs)
    if worst and CFG.latch_fault_s <= worst * 3:
        sys.stderr.write(f"WARNING: latch_fault_s ({CFG.latch_fault_s:.0f}s) is "
                         f"close to the longest intervention time ({worst:.1f}s)\n")
    return table


def serial_selftest(args):
    """
    Exercise the REAL SpeedSerial class -- the same object the detector
    hands INTERVENTION to -- rather than a one-liner that bypasses it.
    """
    vs = VehicleSpeed(CFG.speed_max_kmh)
    rx = args.speed_rx
    port = args.serial_port
    print(f"serial test: {port} @ {args.serial_baud} baud"
          + (f", listening for speed ({rx})" if rx else ""))
    link = SpeedSerial(port=port, baud=args.serial_baud,
                       normal_speed=args.normal_speed,
                       resend_s=args.serial_interval, quiet=False,
                       log_every_s=0.0, rx_format=rx, speed_in=vs)
    if not link.ok:
        sys.exit(f"serial test FAILED: could not open {port}.\n"
                 f"  - right port? The default is /dev/ttyUSB0; the Pi header "
                 f"UART is --serial-port /dev/serial0\n"
                 f"  - permission: 'groups' must list dialout\n"
                 f"  - busy: 'sudo fuser -v {port}' (serial console, or another "
                 f"copy of this script)")
    last = None

    def watch(seconds):
        nonlocal last
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.1)
            if rx:
                v, _age = vs.get()
                if v is not None and v != last:
                    last = v
                    print(f"  RX speed {v:.0f} km/h")

    try:
        print(f"  1/2  holding byte {args.intervene_speed} for 3 s "
              f"(exactly what INTERVENTION does) ...")
        link.set_target(args.intervene_speed)
        watch(3.2)
        print("  2/2  releasing the limit, listening 3 s more ...")
        link.set_target(None)
        watch(3.0)
    finally:
        link.shutdown()
    print(f"  bytes sent: {link.sent}  (expect about 3-4)")
    if rx:
        print(f"  RX: {link.rx_bytes} bytes, {vs.received} speed readings, "
              f"{vs.rejected} rejected")
        if last is not None and last < 5:
            print("  NOTE: the speed is below 5 km/h, which is the MUTED slab: "
                  "INTERVENTION is OFF there by default.")


def open_gpio(args):
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
                  f"{' (active-low)' if led.active_low else ''}")
        except Exception as e:
            sys.stderr.write(f"led unavailable ({e}) -- continuing without it\n")
            led = None

    return gpio, buzzer, led


def main(argv=None):
    args = build_parser().parse_args(argv)
    apply_args_to_config(args)

    speed_on = True          # v8: vehicle speed is always read
    fit_eyes = args.auto_eye and args.close_on is None

    table = build_slab_table(args) if (speed_on or args.explain_slabs
                                       or args.write_slabs) else None
    if args.write_slabs:
        table.save_json(args.write_slabs)
        print(f"wrote {args.write_slabs} -- edit it, then run with "
              f"--slabs {args.write_slabs}")
        return
    if args.explain_slabs:
        print(table.table_text())
        print("  (before calibration: at a low frame rate the live run may "
              "raise very short\n   values; it prints the final table when "
              "calibration finishes)")
        return
    if args.explain:
        explain_ladder(CFG, args.hard_only, speed_on)

    if args.test_camera:
        camera_selftest(args)
        return
    if args.test_serial:
        serial_selftest(args)
        return

    gpio = buzzer = led = None
    grab = serial_link = alerter = landmarker = None
    vspeed = VehicleSpeed(CFG.speed_max_kmh)
    selector = None
    t_zero = time.monotonic()

    try:
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

        # ---- serial: TX (limiter) and/or RX (vehicle speed) ----------------
        tx_on = not args.no_serial
        rx_on = True             # v8: always read the vehicle speed
        if tx_on or rx_on:
            serial_link = SpeedSerial(
                port=args.serial_port, baud=args.serial_baud,
                normal_speed=args.normal_speed if tx_on else None,
                resend_s=args.serial_interval,
                quiet=args.quiet_serial,
                log_every_s=args.serial_log_interval,
                rx_format=args.speed_rx if rx_on else None,
                speed_in=vspeed)
            if serial_link.ok:
                parts = []
                if tx_on:
                    parts.append(f"TX byte {args.intervene_speed} at INTERVENTION"
                                 + (f", {args.caution_speed} at ALARM"
                                    if args.caution_speed is not None else ""))
                if rx_on:
                    parts.append(f"RX vehicle speed ({args.speed_rx})")
                print(f"serial: {args.serial_port} @ {args.serial_baud} baud, "
                      + "; ".join(parts))
            else:
                sys.stderr.write(f"\n*** {args.serial_port} could not be opened: "
                                 f"NO vehicle speed (slab {table.fallback} is "
                                 f"used) and NO intervention byte ***\n\n")
                serial_link = None

        if speed_on:
            selector = SlabSelector(CFG, table, speed=vspeed,
                                    fixed_kmh=args.fixed_speed)
            src = (f"fixed {args.fixed_speed:.0f} km/h (bench)"
                   if args.fixed_speed is not None
                   else f"serial RX ({args.speed_rx})")
            edges = ", ".join(f"{s.name} {s.from_kmh:g}+" for s in table.slabs)
            print(f"speed slabs: ON, source {src}; {edges}; "
                  f"starts in {table.fallback} (armed after calibration)")

        det = MicrosleepDetector(CFG)
        policy = SafetyPolicy(CFG, hard_only=args.hard_only)

        print(f"escalation: {args.sensitivity}"
              + ("  [hard evidence only]" if args.hard_only else "")
              + ("  [MID / fallback slab; other slabs differ]" if speed_on else ""))
        print(f"  WARNING (one beep) : eyes shut >= {CFG.microsleep_warn_s:.2f}s, "
              f"or {CFG.micro_warn_n} microsleeps / {CFG.micro_window_s:.0f}s, "
              f"or trend >= {CFG.risk_warn:.0f}")
        print(f"  ALARM   (buzzer)   : eyes shut >= {CFG.sleep_s:.2f}s, "
              f"or {CFG.micro_alarm_n} microsleeps / {CFG.micro_window_s:.0f}s")
        print(f"  INTERVENE (speed)  : eyes shut >= {CFG.deep_sleep_s:.2f}s, "
              f"or ALARM unheeded {CFG.intervene_after_s:.1f}s, "
              f"or {CFG.micro_intervene_n} microsleeps / {CFG.micro_window_s:.0f}s")
        print(f"eyes: shut above {CFG.close_on:.2f}, open below {CFG.close_off:.2f}"
              + ("  (will be fitted to the driver at calibration)" if fit_eyes
                 else "  (fixed)"))

        if not tx_on:
            no_tx_reason = "--no-serial is set"
        elif serial_link is None:
            no_tx_reason = (f"serial port {args.serial_port} could not be opened "
                            f"at startup (the Pi UART is --serial-port "
                            f"/dev/serial0)")
        else:
            no_tx_reason = None
        if no_tx_reason:
            sys.stderr.write(f"\n*** INTERVENTION BYTE DISABLED: {no_tx_reason} "
                             f"***\n\n")
        alerter = Alerter(CFG.alarm_cooldown_s, buzzer, quiet=args.quiet_alerts,
                          noface_delay_s=CFG.noface_delay_s,
                          noface_enabled=not args.no_noface_alert,
                          speed=serial_link if tx_on else None,
                          intervene_speed=args.intervene_speed,
                          caution_speed=args.caution_speed,
                          led=led, led_hold_s=CFG.led_hold_s,
                          led_reassert_s=CFG.led_reassert_s,
                          no_tx_reason=no_tx_reason)

        stop = threading.Event()

        def on_signal(signum, _frame):
            print(f"\nsignal {signum} -- shutting down")
            stop.set()

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        def adapt(now):
            """
            Pick the slab BEFORE the policy reads the thresholds. No move to
            a slower slab while an event is live: a closure in progress, or
            any level at WARNING or above.
            """
            if selector is None:
                return
            hold = det.eyes_shut or policy.level >= SafetyPolicy.WARNING
            for msg in selector.update(now, hold=hold):
                print(f"[{time.strftime('%H:%M:%S')}] {msg}")

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
        blocked_noted = False
        speed_shown = None           # last vehicle speed printed
        speed_shown_at = 0.0
        speed_was_live = False

        def show_speed(now):
            """Print the real vehicle speed when it moves, and when it stops
            arriving. Works before calibration too."""
            nonlocal speed_shown, speed_shown_at, speed_was_live
            v, age = vspeed.get(now)
            live = v is not None and age <= CFG.speed_timeout_s
            if live != speed_was_live:
                speed_was_live = live
                if live:
                    print(f"[{time.strftime('%H:%M:%S')}] [vehicle] "
                          f"{v:.0f} km/h")
                    speed_shown, speed_shown_at = v, now
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] [vehicle] no speed "
                          f"on {args.serial_port} for "
                          f">{CFG.speed_timeout_s:.0f}s")
                return
            step = args.speed_print_step
            if live and step > 0 and speed_shown is not None and \
               abs(v - speed_shown) >= step and now - speed_shown_at >= 1.0:
                print(f"[{time.strftime('%H:%M:%S')}] [vehicle] {v:.0f} km/h")
                speed_shown, speed_shown_at = v, now

        print("running. Ctrl-C to stop.")
        while not stop.is_set():
            if args.duration and time.monotonic() - t_zero > args.duration:
                break

            try:
                got = grab.latest(timeout=1.0)
                now = time.monotonic()

                if got is None:
                    silent = now - last_frame_at
                    ev = det.no_face(now - t_zero,
                                     f"no camera frame for {silent:.0f}s")
                    st = det.state
                    adapt(now)
                    show_speed(now)
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
                    if selector is not None:
                        for ch in table.fit_to_fps(CFG.microsleep_s):
                            print("  slab " + ch)
                        selector.arm()
                        print(table.table_text(active=selector.slab.name))
                    print()

                # ---- speed -> thresholds, then policy, then actuators -----
                adapt(now)
                show_speed(now)
                level, why = policy.update(now, ev)

                # A closure long enough for INTERVENTION that the active slab
                # does not allow: say so once per closure, so "nothing
                # happened" is never a mystery.
                if selector is not None and selector.armed:
                    if not CFG.intervene_on and ev.face and \
                       ev.closure_s >= selector.baseline("deep_sleep_s"):
                        if not blocked_noted:
                            blocked_noted = True
                            print(f"[{time.strftime('%H:%M:%S')}] [slab] "
                                  f"INTERVENTION blocked: slab "
                                  f"{selector.slab.name} has intervention OFF "
                                  f"({selector.describe()})")
                    elif ev.closure_s == 0.0:
                        blocked_noted = False

                if level != last_level:
                    arrow = "^^" if level > last_level else "vv"
                    spd = f"   [{selector.describe()}]" if selector else ""
                    print(f"[{time.strftime('%H:%M:%S')}] {arrow} "
                          f"{SafetyPolicy.NAMES[last_level]} -> "
                          f"{SafetyPolicy.NAMES[level]}   {why}{spd}")
                    if level >= SafetyPolicy.INTERVENE and alerter.speed:
                        print(f"           -> sending byte "
                              f"{args.intervene_speed} to {args.serial_port} "
                              f"every {args.serial_interval:g}s while held")
                    last_level = level

                alerter.apply(level, ev, why)

                if st.status != last_status and \
                   (now - last_status_at) >= CFG.status_print_min_s:
                    last_status_at = now
                    tail = f"  {st.reasons[0]}" if st.reasons else ""
                    print(f"[{time.strftime('%H:%M:%S')}] {st.status:<13} "
                          f"risk {st.risk:5.1f}{tail}")
                    last_status = st.status
                elif args.heartbeat and now - last_beat >= args.heartbeat:
                    last_beat = now
                    spd = ""
                    if selector is not None:
                        spd = f"  [{selector.describe()}]"
                        if selector.fixed_kmh is None:
                            spd += f" [{selector.vehicle_text(now)}]"
                    print(f"[{time.strftime('%H:%M:%S')}] {st.status:<13} "
                          f"risk {st.risk:5.1f}  [{policy.status_line(now)}]  "
                          f"{fps:.1f}fps  perclos {st.perclos*100:.0f}%  "
                          f"blinks/min {st.blink_rate:.0f}  "
                          f"ms {st.microsleeps} slp {st.sleeps} "
                          f"stuck {st.stares}  dropped {grab.dropped}{spd}")

                frame_errors = 0

            except KeyboardInterrupt:
                raise
            except Exception as e:
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
        if alerter:
            alerter.shutdown()
        if buzzer:
            buzzer.shutdown()
        if gpio:
            gpio.close()
        if led:
            led.close()
        if serial_link:
            serial_link.shutdown()
        if grab:
            grab.release()
        if landmarker:
            try:
                landmarker.close()
            except Exception:
                pass

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
        if serial_link:
            print(f"  speed bytes sent: {serial_link.sent}   "
                  f"rx bytes: {serial_link.rx_bytes}   "
                  f"rx errors: {serial_link.rx_errors}")
        if selector is not None:
            print(f"  speed readings: {vspeed.received} ok, "
                  f"{vspeed.rejected} rejected, peak {vspeed.peak:.0f} km/h, "
                  f"slab changes {selector.switches}")
            print("  time per slab: " + ", ".join(
                f"{k} {v/60:.1f} min" for k, v in selector.time_in.items()))
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
    except (NameError, UnboundLocalError, AttributeError):
        pass


if __name__ == "__main__":
    main()
