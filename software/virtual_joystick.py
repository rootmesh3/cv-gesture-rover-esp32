#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║        VIRTUAL JOYSTICK - Hand Gesture Robot Car Controller                  ║
║        A passion project by Nomun                                            ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Thread 1 (Main)  : Camera, MediaPipe, OpenCV GUI  @ 30 FPS cap             ║
║  Thread 2 (Audio) : Queue-based pyttsx3 with smart trend debouncing         ║
║  Thread 3 (UDP)   : 20 Hz heartbeat stream to ESP module                    ║
║                                                                              ║
║  Keyboard:  [Q] Quit  |  [M] Mute/Unmute  |  [R] Toggle Cinematic Mode     ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

# ── Standard Library ──────────────────────────────────────────────────────────
import sys
import math
import time
import queue
import socket
import logging
import threading
import collections

# ── Third-Party ───────────────────────────────────────────────────────────────
import cv2
import numpy as np
import mediapipe as mp
import pyttsx3


# ==============================================================================
# SECTION 1 — CONFIGURATION
# ==============================================================================

# ─── Network ──────────────────────────────────────────────────────────────────
ESP_IP   = "192.168.4.1"   # ESP Access-Point IP (or router-assigned IP)
ESP_PORT = 4210             # UDP port your ESP firmware listens on

# ─── Camera ───────────────────────────────────────────────────────────────────
CAMERA_INDEX = 0
TARGET_FPS   = 30

# ─── UDP Heartbeat ────────────────────────────────────────────────────────────
UDP_RATE_HZ  = 20
UDP_INTERVAL = 1.0 / UDP_RATE_HZ

# ─── Gesture Tuning ───────────────────────────────────────────────────────────
# Speed ratio: dist(ThumbTip→IndexTip) / dist(Wrist→MiddleBase)
SPEED_RATIO_MIN = 0.15     # Fully pinched → 0 %
SPEED_RATIO_MAX = 1.30     # Fingers fully spread → 100 %
SPEED_DEAD_ZONE = 5        # Values below this snap to 0 (applied to |speed|)
TILT_DEAD_ZONE  = 10       # Angles within ±10 deg treated as Straight

# ─── Reverse Gear ─────────────────────────────────────────────────────────────
# Fixed reverse speed (negative integer) sent directly to the ESP.
# The ESP firmware interprets negative S values as reverse motor direction.
# Trigger: palm facing down, i.e. MiddleBase[9].y > Wrist[0].y + 0.15
REVERSE_SPEED       = -90   # % power in reverse (negative, fixed magnitude)
PALM_DOWN_THRESHOLD = 0.15  # Normalised Y-distance threshold for palm-down detect

# ─── EMA Smoothing ────────────────────────────────────────────────────────────
# alpha=1.0 = no smoothing; alpha=0.1 = very heavy smoothing (more lag)
EMA_ALPHA = 0.30

# ─── HUD Layout ───────────────────────────────────────────────────────────────
HUD_X, HUD_Y = 10, 10
HUD_W, HUD_H = 348, 220

# ─── Iron Man Reticle ─────────────────────────────────────────────────────────
RETICLE_BASE_R  = 10       # Base circle radius in pixels
PULSE_MAX_DELTA = 6        # Maximum additional radius from speed pulse (0-6 px)

# ─── Speed Trend Detection ────────────────────────────────────────────────────
TREND_WINDOW    = 8        # Frames to accumulate for trend calculation
TREND_THRESHOLD = 12       # Minimum speed-point delta to declare a trend

# ─── MediaPipe ────────────────────────────────────────────────────────────────
MP_DETECT_CONF = 0.72
MP_TRACK_CONF  = 0.60

# ─── Neon Color Palette (all values in OpenCV BGR order) ──────────────────────
# Cyan family  — Throttle line + active reticle
#   Pure cyan in BGR = (255, 255, 0).  We desaturate slightly for a warmer glow.
NEON_CYAN_DARK  = ( 90,  90,   0)   # Outer glow (darkened)
NEON_CYAN_MID   = (200, 200,  10)   # Mid halo
NEON_CYAN_CORE  = (255, 255, 255)   # Bright white core

# Amber / Orange family — Steering axis line
NEON_AMB_DARK   = (  0,  55, 110)   # Outer glow (darkened)
NEON_AMB_MID    = (  0, 140, 230)   # Mid halo
NEON_AMB_CORE   = (  0, 200, 255)   # Bright core

# Red family — Stop reticle
NEON_RED_DARK   = (  0,   0,  80)   # Outer glow (darkened)
NEON_RED_MID    = (  0,   0, 170)   # Mid halo
NEON_RED_CORE   = ( 60,  60, 255)   # Bright red core


# ==============================================================================
# SECTION 2 — LOGGING
# ==============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)-8s] %(asctime)s | %(threadName)-12s | %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


# ==============================================================================
# SECTION 3 — SHARED STATE
# ==============================================================================
state_lock = threading.Lock()

shared_state: dict = {
    "speed"       : 0,       # int 0-100
    "tilt"        : 0,       # int signed degrees (negative=left, positive=right)
    "command"     : "STOP",  # str human-readable label
    "fist"        : False,   # bool emergency-stop flag
    "running"     : True,    # bool — set False to signal all threads to exit
    "muted"       : False,   # bool TTS mute toggle
    "fps"         : 0.0,     # float measured FPS (display only)
    "hand_visible": False,   # bool
}

audio_queue: queue.Queue = queue.Queue(maxsize=8)


# ==============================================================================
# SECTION 4 — EMA FILTER
# ==============================================================================
class EMAFilter:
    """
    Exponential Moving Average filter.
    Smooths raw gesture values to prevent hardware jitter from
    causing command oscillation or choppy speed bars.

    Formula:  value_t = alpha * raw_t + (1 - alpha) * value_(t-1)
    """

    def __init__(self, alpha: float = EMA_ALPHA):
        self.alpha  = alpha
        self._value = None      # None until first observation

    def update(self, raw: float) -> float:
        """Feed a new raw value; returns the smoothed output."""
        if self._value is None:
            self._value = float(raw)
        else:
            self._value = self.alpha * raw + (1.0 - self.alpha) * self._value
        return self._value

    def reset(self) -> None:
        """Forget history (e.g. when the hand disappears)."""
        self._value = None

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0


# ==============================================================================
# SECTION 5 — THREAD 2: AUDIO (pyttsx3, queue-based, never blocks video)
# ==============================================================================
def audio_thread_fn() -> None:
    """
    Consumes text items from `audio_queue` and speaks them.
    A `None` sentinel value signals clean shutdown.
    pyttsx3.runAndWait() is blocking, so we isolate it here.

    Windows SAPI5 COM fix:
        pyttsx3 on Windows uses the SAPI5 COM automation API.  COM objects
        are apartment-threaded and MUST be initialised on each thread that
        uses them.  Without CoInitialize() the engine initialises on the main
        thread's COM apartment and then silently deadlocks on the second
        runAndWait() call (or sometimes the first) when invoked from a
        background thread.  Calling pythoncom.CoInitialize() here, before
        pyttsx3.init(), registers this thread with a new STA apartment and
        keeps every subsequent SAPI5 call within the correct COM context.
    """
    # ── Windows COM initialisation (must be first, before pyttsx3.init) ───────
    try:
        import pythoncom
        pythoncom.CoInitialize()
        log.info("COM STA initialized for audio thread (Windows SAPI5).")
    except ImportError:
        log.warning(
            "pythoncom not found — install pywin32 if audio freezes after the "
            "first announcement:  pip install pywin32"
        )
    except Exception as exc:
        log.warning(f"COM init warning (non-fatal): {exc}")

    # ── pyttsx3 engine ────────────────────────────────────────────────────────
    try:
        engine = pyttsx3.init()
        engine.setProperty("rate",   155)
        engine.setProperty("volume", 1.0)
        log.info("pyttsx3 engine initialized.")
    except Exception as exc:
        log.error(f"pyttsx3 init failed: {exc}  -  Audio thread will not run.")
        return

    while True:
        try:
            text = audio_queue.get(timeout=0.5)
        except queue.Empty:
            with state_lock:
                if not shared_state["running"]:
                    break
            continue

        if text is None:                   # Shutdown sentinel
            audio_queue.task_done()
            break

        with state_lock:
            muted = shared_state["muted"]

        if not muted:
            try:
                engine.say(text)
                engine.runAndWait()
            except RuntimeError as exc:
                log.warning(f"TTS RuntimeError (ignored): {exc}")
            except Exception as exc:
                log.warning(f"TTS error: {exc}")

        audio_queue.task_done()

    log.info("Audio thread exiting cleanly.")


def announce(text: str) -> None:
    """Non-blocking push to the audio queue. Drops silently when full."""
    try:
        audio_queue.put_nowait(text)
    except queue.Full:
        pass


# ==============================================================================
# SECTION 6 — THREAD 3: UDP COMMUNICATION (20 Hz heartbeat)
# ==============================================================================
def udp_thread_fn() -> None:
    """
    Transmits the current command state to the ESP at UDP_RATE_HZ (20 Hz).
    The continuous stream acts as a hardware heartbeat:
    if the laptop dies the ESP can detect packet loss and apply a failsafe.

    Payload format: S[Speed]T[Tilt]
    Examples:
        "S85T-12"  — 85% speed, 12 deg left
        "S0T0"     — stopped / straight
        "S100T30"  — full speed, 30 deg right
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.05)
        log.info(f"UDP socket created.  Target: {ESP_IP}:{ESP_PORT}")
    except OSError as exc:
        log.error(f"Cannot create UDP socket: {exc}  -  UDP thread exiting.")
        return

    last_payload = ""

    while True:
        tick_start = time.perf_counter()

        with state_lock:
            if not shared_state["running"]:
                break
            speed = shared_state["speed"]
            tilt  = shared_state["tilt"]

        # Format: S[Speed]T[Tilt]  (tilt is already signed, e.g. -12 or +30)
        payload = f"S{speed}T{tilt}"

        try:
            sock.sendto(payload.encode("utf-8"), (ESP_IP, ESP_PORT))
            if payload != last_payload:
                log.info(f"UDP TX -> {payload}")   # log.info so packets are visible in terminal
                last_payload = payload
        except OSError as exc:
            log.warning(f"UDP send error (will retry): {exc}")

        elapsed   = time.perf_counter() - tick_start
        sleep_for = UDP_INTERVAL - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

    try:
        sock.close()
    except Exception:
        pass
    log.info("UDP thread exiting cleanly.")


# ==============================================================================
# SECTION 7 — GESTURE PROCESSING (pure functions)
# ==============================================================================
def _dist(a, b) -> float:
    """Euclidean distance between two MediaPipe NormalizedLandmark objects."""
    return math.hypot(a.x - b.x, a.y - b.y)


def is_fist(lm: list) -> bool:
    """
    ROBUST distance-based fist detection (Emergency Stop).

    WHY the old tip.y > pip.y method was flawed:
        Raw Y-coordinate comparison only works when the hand is held upright.
        Tilt the hand downward 90° and every finger tip has a HIGHER y-value
        than its PIP joint even when the fingers are fully extended — triggering
        false emergency stops whenever the user points the hand downward.

    NEW METHOD — purely distance-based, orientation-independent:
        When a finger is curled, its tip folds back toward the palm and
        therefore ends up CLOSER to the wrist than the PIP joint does.

        For each finger:
            dist(Tip → Wrist)  <  dist(PIP → Wrist)  →  finger is curled

        Tested landmark pairs  (Tip index, PIP index):
            Index  → (8,  6)
            Middle → (12, 10)
            Ring   → (16, 14)
            Pinky  → (20, 18)

        All four must be curled for a confirmed fist.
        (Thumb excluded: its curl geometry differs and is already used for speed.)
    """
    pairs = [(8, 6), (12, 10), (16, 14), (20, 18)]
    for tip_idx, pip_idx in pairs:
        tip_to_wrist = _dist(lm[tip_idx], lm[0])
        pip_to_wrist = _dist(lm[pip_idx], lm[0])
        if tip_to_wrist >= pip_to_wrist:
            return False   # At least one finger is NOT fully curled — not a fist
    return True


def is_palm_down(lm: list) -> bool:
    """
    Detects a downward-pointing hand — the Reverse Gear trigger.

    Geometry:
        In OpenCV image space Y increases downward (0 = top of frame).
        When the hand points UP:   MiddleBase[9].y  <  Wrist[0].y
        When the hand points DOWN: MiddleBase[9].y  >  Wrist[0].y

        We require a margin of PALM_DOWN_THRESHOLD (0.15 = 15 % of frame
        height in normalised coords) to prevent accidental triggering during
        nearly-horizontal gestures where the hand is only slightly tilted.

    Ergonomic usage:
        The user holds the hand with fingers pointing downward (like pressing
        a table). The gesture is stable, anatomically distinct from the
        forward-drive posture, and easy to hold while simultaneously tilting
        for left/right steering.
    """
    return lm[9].y > lm[0].y + PALM_DOWN_THRESHOLD


def compute_speed(lm: list) -> float:
    """
    Proportional speed (0.0–100.0) from pinch aperture.
    Normalized against the anatomical reference Wrist[0]→MiddleBase[9]
    to be hand-size and camera-distance independent.
    Returns a float; EMA and dead-zone quantization happen in the main loop.
    """
    thumb_idx = _dist(lm[4], lm[8])
    wrist_mid = _dist(lm[0], lm[9])
    if wrist_mid < 1e-6:
        return 0.0
    ratio = max(SPEED_RATIO_MIN, min(SPEED_RATIO_MAX, thumb_idx / wrist_mid))
    raw   = (ratio - SPEED_RATIO_MIN) / (SPEED_RATIO_MAX - SPEED_RATIO_MIN) * 100.0
    return raw if raw >= SPEED_DEAD_ZONE else 0.0


def compute_tilt(lm: list) -> float:
    """
    Hand tilt angle (degrees) for steering.
    Vector: Wrist[0] → MiddleFingerBase[9], deviation from vertical.
    Negative = left lean, Positive = right lean.
    Returns float; dead zone applied after EMA in main loop.
    """
    dx =  lm[9].x - lm[0].x
    dy = -(lm[9].y - lm[0].y)   # Flip: image y is inverted vs. Cartesian y
    return math.degrees(math.atan2(dx, dy))


def classify_command(speed: int, tilt: int, fist: bool) -> str:
    """
    Maps numeric state to a human-readable command label.

    Speed sign convention:
        speed > 0   → forward motion (0-100 %)
        speed == 0  → stopped
        speed < 0   → reverse motion (REVERSE_SPEED = -40)

    Priority:  EMERGENCY STOP  >  REVERSE  >  FORWARD  >  STOP
    """
    if fist:
        return "EMERGENCY STOP"
    if speed == 0:
        return "STOP"
    if speed < 0:
        # Reverse gear: steer normally while reversing
        if tilt < -TILT_DEAD_ZONE:
            return "REVERSE LEFT"
        if tilt > TILT_DEAD_ZONE:
            return "REVERSE RIGHT"
        return "REVERSE"
    # speed > 0: forward motion
    if tilt < -TILT_DEAD_ZONE:
        return "FORWARD LEFT"
    if tilt > TILT_DEAD_ZONE:
        return "FORWARD RIGHT"
    return "FORWARD"


# ==============================================================================
# SECTION 8 — SMART AUDIO DEBOUNCER
# ==============================================================================
class SmartDebouncer:
    """
    Intelligent TTS state tracker with trend detection.

    Announces:
      - Every DISTINCT command state change (STOP, FORWARD, etc.)
        including EMERGENCY STOP
      - "Accelerating" once when speed consistently rises over several frames
      - "Slowing down" once when speed consistently falls over several frames
    Never re-announces the same state; never spams raw speed numbers.
    """

    _SPEECH_MAP = {
        # ── Forward ────────────────────────────────────────────────────────────
        "STOP"           : "Stopped",
        "FORWARD"        : "Moving forward",
        "FORWARD LEFT"   : "Turning left",
        "FORWARD RIGHT"  : "Turning right",
        "EMERGENCY STOP" : "Emergency stop",
        # ── Reverse ────────────────────────────────────────────────────────────
        # Reverse is a fixed speed (-40), so no "accelerating / slowing down"
        # announcements are needed — only the state-change announcement.
        "REVERSE"        : "Reversing",
        "REVERSE LEFT"   : "Reversing left",
        "REVERSE RIGHT"  : "Reversing right",
    }

    def __init__(self):
        self._last_command  = ""
        self._last_trend    = ""          # "up", "down", or ""
        self._speed_history = collections.deque(maxlen=TREND_WINDOW)

    def update(self, command: str, speed: int) -> None:
        # ── 1. Command change ─────────────────────────────────────────────────
        if command != self._last_command:
            speech = self._SPEECH_MAP.get(command, command.lower())
            announce(speech)
            log.info(f"[Audio] '{speech}'  ({self._last_command!r} -> {command!r})")
            self._last_command = command
            # Reset trend state on every command change to avoid stale history
            self._last_trend = ""
            self._speed_history.clear()
            return   # Don't also check trend in the same frame as a command change

        # ── 2. Speed trend (forward motion only) ──────────────────────────────
        # Reverse uses a fixed speed (REVERSE_SPEED = -40); there is no
        # meaningful speed trend to announce.  Only track trends while the
        # user is actively driving forward.
        if command in ("FORWARD", "FORWARD LEFT", "FORWARD RIGHT"):
            self._speed_history.append(speed)
            if len(self._speed_history) >= TREND_WINDOW:
                delta = int(self._speed_history[-1]) - int(self._speed_history[0])
                if delta > TREND_THRESHOLD:
                    current_trend = "up"
                elif delta < -TREND_THRESHOLD:
                    current_trend = "down"
                else:
                    current_trend = ""      # Plateau — trend has levelled off

                if current_trend and current_trend != self._last_trend:
                    speech = "Accelerating" if current_trend == "up" else "Slowing down"
                    announce(speech)
                    log.info(f"[Audio] '{speech}'  (speed delta = {delta:+d})")
                    self._last_trend = current_trend
                elif not current_trend:
                    # Once a trend levels off, allow it to fire again later
                    self._last_trend = ""
        else:
            # Reverse, Stop, Emergency Stop: no trend tracking needed
            self._speed_history.clear()
            self._last_trend = ""


# ==============================================================================
# SECTION 9 — IRON MAN NEON VISUALS
# ==============================================================================
def _neon_line(
    frame: np.ndarray,
    pt1: tuple, pt2: tuple,
    dark_bgr: tuple, mid_bgr: tuple, core_bgr: tuple,
    outer_thick: int = 9,
    mid_thick: int   = 5,
    core_thick: int  = 2,
) -> None:
    """
    Neon glow line via triple-layer stacking (fast, no extra frame copies).

    Layer 1 — Wide outer glow:   thick line in a dark, saturated color
    Layer 2 — Mid halo:          medium line at full glow brightness
    Layer 3 — Bright core:       thin line in white or near-white

    The visual illusion of soft bloom comes from each layer being drawn
    centered on the previous, progressively brighter and thinner.
    """
    cv2.line(frame, pt1, pt2, dark_bgr, outer_thick, cv2.LINE_AA)
    cv2.line(frame, pt1, pt2, mid_bgr,  mid_thick,   cv2.LINE_AA)
    cv2.line(frame, pt1, pt2, core_bgr, core_thick,  cv2.LINE_AA)


def _neon_circle(
    frame: np.ndarray,
    center: tuple, radius: int,
    dark_bgr: tuple, mid_bgr: tuple, core_bgr: tuple,
    filled: bool      = False,
    outer_thick: int  = 6,
    core_thick: int   = 2,
) -> None:
    """
    Neon glow circle via triple-layer stacking.
    `filled=True` draws a solid disc (for STOP state).
    `filled=False` draws a hollow ring (for GO/active state).
    """
    fill = -1 if filled else core_thick
    cv2.circle(frame, center, radius + 5, dark_bgr, outer_thick,     cv2.LINE_AA)
    cv2.circle(frame, center, radius + 2, mid_bgr,  outer_thick - 2, cv2.LINE_AA)
    cv2.circle(frame, center, radius,     core_bgr, fill,            cv2.LINE_AA)


def draw_iron_man_visuals(
    frame: np.ndarray,
    lm: list,
    speed: int,
    frame_w: int,
    frame_h: int,
) -> None:
    """
    Renders all Iron Man-style neon geometry on top of the MediaPipe skeleton.
    Called in BOTH control mode and cinematic mode.

    Elements:
      1. Throttle line  — Thumb[4] -> Index[8]         (Cyan neon)
      2. Steering line  — Wrist[0] -> MiddleBase[9]    (Amber neon)
      3. State-reactive reticle at the throttle midpoint

    Performance note: uses only standard cv2.line and cv2.circle (no shaders,
    no per-pixel loops). All anti-aliasing is done by OpenCV's LINE_AA flag.
    """
    def px(i: int) -> tuple:
        """Convert a normalized landmark to integer pixel coordinates."""
        return (int(lm[i].x * frame_w), int(lm[i].y * frame_h))

    thumb_tip = px(4)
    index_tip = px(8)
    wrist     = px(0)
    mid_base  = px(9)

    # ── 1. Throttle Line: Thumb[4] -> Index[8]  (Cyan) ───────────────────────
    _neon_line(
        frame, thumb_tip, index_tip,
        dark_bgr=NEON_CYAN_DARK, mid_bgr=NEON_CYAN_MID, core_bgr=NEON_CYAN_CORE,
        outer_thick=9, mid_thick=5, core_thick=2,
    )

    # ── 2. Steering Line: Wrist[0] -> MiddleBase[9]  (Amber, more subtle) ────
    # Thinner than the throttle line so it doesn't overwhelm the skeleton.
    _neon_line(
        frame, wrist, mid_base,
        dark_bgr=NEON_AMB_DARK, mid_bgr=NEON_AMB_MID, core_bgr=NEON_AMB_CORE,
        outer_thick=6, mid_thick=3, core_thick=2,
    )

    # ── 3. State-Reactive Reticle at Throttle Midpoint ───────────────────────
    mid_x  = (thumb_tip[0] + index_tip[0]) // 2
    mid_y  = (thumb_tip[1] + index_tip[1]) // 2
    center = (mid_x, mid_y)

    label_x_offset = RETICLE_BASE_R + 8    # Text starts just to the right

    if speed == 0:
        # ── STOPPED: Solid red disc ───────────────────────────────────────────
        # Filled = motor is off / parked.  Accessibility label: "STOP"
        _neon_circle(
            frame, center, RETICLE_BASE_R,
            dark_bgr=NEON_RED_DARK, mid_bgr=NEON_RED_MID, core_bgr=NEON_RED_CORE,
            filled=True, outer_thick=5, core_thick=-1,
        )
        cv2.putText(
            frame, "STOP",
            (mid_x + label_x_offset, mid_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, NEON_RED_CORE, 1, cv2.LINE_AA,
        )

    elif speed < 0:
        # ── REVERSE: Hollow amber/orange ring, fixed size ─────────────────────
        # Hollow = motor IS running (just in reverse).
        # Fixed radius (no pulse) because REVERSE_SPEED is a constant magnitude.
        # Colour: amber (NEON_AMB family) — visually distinct from both
        #   STOP (red)  and  GO-forward (cyan).
        # Accessibility label: "REV"
        _neon_circle(
            frame, center, RETICLE_BASE_R + 3,
            dark_bgr=NEON_AMB_DARK, mid_bgr=NEON_AMB_MID, core_bgr=NEON_AMB_CORE,
            filled=False, outer_thick=5, core_thick=2,
        )
        cv2.putText(
            frame, "REV",
            (mid_x + RETICLE_BASE_R + 11, mid_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, NEON_AMB_CORE, 1, cv2.LINE_AA,
        )

    else:
        # ── ACTIVE FORWARD: Hollow cyan ring with speed-driven pulse ──────────
        # Oscillation: sin wave at ~2 Hz (4 rad/s), amplitude scaled by speed.
        # Delta strictly bounded 0-6 px so it never occludes adjacent landmarks.
        oscillation = 0.5 + 0.5 * math.sin(time.time() * 4.0)   # 0.0 -> 1.0
        pulse_delta = int(oscillation * PULSE_MAX_DELTA * (speed / 100.0))
        pulse_delta = max(0, min(PULSE_MAX_DELTA, pulse_delta))   # Strict 0-6 px bound

        active_r = RETICLE_BASE_R + pulse_delta
        _neon_circle(
            frame, center, active_r,
            dark_bgr=NEON_CYAN_DARK, mid_bgr=NEON_CYAN_MID, core_bgr=NEON_CYAN_CORE,
            filled=False, outer_thick=5, core_thick=2,
        )
        # Accessibility label "GO"
        cv2.putText(
            frame, "GO",
            (mid_x + active_r + 8, mid_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, NEON_CYAN_CORE, 1, cv2.LINE_AA,
        )


# ==============================================================================
# SECTION 10 — HUD DRAWING (control mode only)
# ==============================================================================
def _draw_speed_bar(
    frame: np.ndarray,
    speed: int,
    x: int, y: int,
    bar_w: int = 205,
    bar_h: int = 17,
) -> None:
    """
    Horizontal speed bar — handles three states:

      speed > 0  : Green -> Yellow -> Red gradient (forward throttle)
      speed == 0 : Empty dark track (stopped)
      speed < 0  : Solid amber fill from the left (reverse gear)

    The reverse bar shows a flat colour (not a gradient) to visually
    distinguish it from the forward throttle at a glance.
    Only ~205 scan-line iterations in the worst case — negligible CPU cost.
    """
    # Track background (always drawn)
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (22, 22, 22), -1)
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (75, 75, 75), 1)

    if speed == 0:
        return

    if speed < 0:
        # ── REVERSE: Solid amber/orange bar ───────────────────────────────────
        # abs(REVERSE_SPEED) = 40, so the bar fills 40 % from the left edge.
        # The amber colour (BGR: 0, 140, 230 ≈ orange) matches the NEON_AMB
        # family used on the reverse reticle and steering line for consistency.
        fill_px = max(1, int(bar_w * abs(speed) / 100))
        cv2.rectangle(
            frame,
            (x + 1, y + 1),
            (x + fill_px - 1, y + bar_h - 1),
            (0, 140, 230),   # BGR orange-amber
            -1,
        )
        # Label: shows the raw negative speed value so the user knows the magnitude
        cv2.putText(
            frame, f"REV {speed}",
            (x + bar_w + 7, y + bar_h - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1, cv2.LINE_AA,
        )
    else:
        # ── FORWARD: Green -> Yellow -> Red gradient ───────────────────────────
        fill_px = max(1, int(bar_w * speed / 100))
        for i in range(fill_px):
            norm = i / max(bar_w - 1, 1)          # 0.0 -> 1.0 across full bar width
            if norm < 0.5:
                r = int(norm * 2 * 255)
                g = 255
            else:
                r = 255
                g = int((1.0 - (norm - 0.5) * 2) * 255)
            cv2.line(frame, (x + i, y + 1), (x + i, y + bar_h - 1), (0, g, r), 1)
        cv2.putText(
            frame, f"{speed}%",
            (x + bar_w + 7, y + bar_h - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (195, 195, 195), 1, cv2.LINE_AA,
        )


def _draw_orientation_dial(
    frame: np.ndarray,
    tilt_deg: int,
    cx: int, cy: int,
    radius: int = 50,
) -> None:
    """
    Aviation-style attitude indicator (artificial horizon).

    IMPORTANT: Degree symbol deliberately avoided here.
    Uses the text 'deg' suffix to prevent OpenCV font rendering artifacts
    that can occur with Unicode/extended ASCII characters on some platforms.
    """
    # Outer reference ring
    cv2.circle(frame, (cx, cy), radius, (60, 60, 60), 1, cv2.LINE_AA)

    # Tick marks at every 30 deg from -90 to +90
    for tick in (-90, -60, -30, 0, 30, 60, 90):
        # We map 0 deg (upward) to OpenCV's -90 deg (pointing up in image space)
        rad      = math.radians(tick - 90)
        ox       = int(cx + radius * math.cos(rad))
        oy       = int(cy + radius * math.sin(rad))
        tick_len = 12 if tick % 90 == 0 else 7
        inner_r  = radius - tick_len
        ix       = int(cx + inner_r * math.cos(rad))
        iy       = int(cy + inner_r * math.sin(rad))
        col      = (120, 120, 120) if tick == 0 else (65, 65, 65)
        cv2.line(frame, (ix, iy), (ox, oy), col, 1, cv2.LINE_AA)

    # L / R axis labels
    for label_deg, label_txt in ((-90, "L"), (90, "R")):
        rad = math.radians(label_deg - 90)
        lx  = int(cx + (radius + 10) * math.cos(rad))
        ly  = int(cy + (radius + 10) * math.sin(rad)) + 4
        cv2.putText(frame, label_txt, (lx - 5, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)

    # Fixed aircraft-crosshair symbol (the "aircraft" the horizon moves under)
    cv2.line(frame, (cx - 14, cy), (cx - 5, cy), (160, 160, 35), 2, cv2.LINE_AA)
    cv2.line(frame, (cx + 5,  cy), (cx + 14, cy), (160, 160, 35), 2, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 3, (160, 160, 35), -1, cv2.LINE_AA)

    # Rotating needle indicating current tilt
    needle_rad = math.radians(tilt_deg - 90)
    needle_len = radius - 5
    tip_x      = int(cx + needle_len * math.cos(needle_rad))
    tip_y      = int(cy + needle_len * math.sin(needle_rad))

    if abs(tilt_deg) <= TILT_DEAD_ZONE:
        needle_col = (0, 220, 220)      # Cyan  -> Straight / dead zone
    elif tilt_deg < 0:
        needle_col = (55, 55, 240)      # Red   -> Left tilt
    else:
        needle_col = (55, 210, 55)      # Green -> Right tilt

    cv2.line(frame, (cx, cy), (tip_x, tip_y), needle_col, 2, cv2.LINE_AA)
    cv2.circle(frame, (tip_x, tip_y), 4, needle_col, -1, cv2.LINE_AA)

    # Tilt readout below dial — uses 'deg' suffix, NO degree symbol
    sign_str = f"+{tilt_deg} deg" if tilt_deg >= 0 else f"{tilt_deg} deg"
    cv2.putText(frame, sign_str,
                (cx - 28, cy + radius + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, (150, 150, 150), 1, cv2.LINE_AA)


def draw_hud(
    frame: np.ndarray,
    speed: int,
    tilt:  int,
    command: str,
    fps: float,
    muted: bool,
    fist: bool,
    hand_visible: bool,
) -> None:
    """
    Full professional dashboard overlay.
    Only called when cinematic_mode == False.

    Transparency:
      cv2.addWeighted(overlay, 0.85, frame, 0.15, ...)
      → 85% dark panel / 15% live frame bleed-through for high contrast
        against any camera background (bright or dark).
    """
    h_frame, w_frame = frame.shape[:2]

    # ── Semi-transparent dark panel ───────────────────────────────────────────
    overlay = frame.copy()
    p1 = (HUD_X, HUD_Y)
    p2 = (HUD_X + HUD_W, HUD_Y + HUD_H)
    cv2.rectangle(overlay, p1, p2, (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)   # High-contrast ratio
    cv2.rectangle(frame, p1, p2, (58, 58, 58), 1)

    # ── Title ─────────────────────────────────────────────────────────────────
    cv2.putText(frame, "VIRTUAL JOYSTICK",
                (HUD_X + 10, HUD_Y + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.54, (0, 195, 255), 1, cv2.LINE_AA)
    cv2.line(frame,
             (HUD_X + 5, HUD_Y + 27),
             (HUD_X + HUD_W - 5, HUD_Y + 27),
             (48, 48, 48), 1)

    # ── Command label ─────────────────────────────────────────────────────────
    # Color coding:
    #   Emergency Stop → bright red       (danger, motor forced off)
    #   Reverse *      → amber/orange     (caution, moving backward)
    #   Forward / Stop → neon green       (normal operating state)
    if fist:
        cmd_color = (40, 40, 240)              # BGR bright red
    elif command.startswith("REVERSE"):
        cmd_color = (0, 140, 230)              # BGR amber/orange (matches REV bar)
    else:
        cmd_color = (0, 255, 130)              # BGR neon green (FORWARD / STOP)
    cv2.putText(frame, command,
                (HUD_X + 10, HUD_Y + 52),
                cv2.FONT_HERSHEY_DUPLEX, 0.63, cmd_color, 1, cv2.LINE_AA)

    # ── Speed section ─────────────────────────────────────────────────────────
    cv2.putText(frame, "SPEED",
                (HUD_X + 10, HUD_Y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, (120, 120, 120), 1, cv2.LINE_AA)
    _draw_speed_bar(frame, speed, HUD_X + 10, HUD_Y + 77)

    # ── Orientation dial ──────────────────────────────────────────────────────
    dial_cx = HUD_X + 70
    dial_cy = HUD_Y + 162
    cv2.putText(frame, "TILT",
                (dial_cx - 13, HUD_Y + 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)
    _draw_orientation_dial(frame, tilt, dial_cx, dial_cy, radius=50)

    # ── Right-side status column ──────────────────────────────────────────────
    sx = HUD_X + 160

    cv2.putText(frame, f"ESP  {ESP_IP}",
                (sx, HUD_Y + 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.putText(frame, f"PORT {ESP_PORT}",
                (sx, HUD_Y + 124),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (95, 95, 95), 1, cv2.LINE_AA)

    hand_col  = (0, 215, 95) if hand_visible else (65, 65, 65)
    hand_txt  = "HAND : DETECTED" if hand_visible else "HAND : NONE"
    cv2.putText(frame, hand_txt,
                (sx, HUD_Y + 144),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, hand_col, 1, cv2.LINE_AA)

    mute_col  = (95, 95, 200) if muted else (0, 195, 255)
    mute_txt  = "[M] MUTED" if muted else "[M] AUDIO ON"
    cv2.putText(frame, mute_txt,
                (sx, HUD_Y + 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, mute_col, 1, cv2.LINE_AA)

    cv2.putText(frame, f"UDP @ {UDP_RATE_HZ} Hz",
                (sx, HUD_Y + 176),
                cv2.FONT_HERSHEY_SIMPLEX, 0.31, (80, 80, 80), 1, cv2.LINE_AA)

    # ── FPS counter — top-right corner, vibrant green ─────────────────────────
    fps_str = f"FPS {fps:4.1f}"
    cv2.putText(frame, fps_str,
                (w_frame - 115, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 80), 1, cv2.LINE_AA)

    # ── Keyboard shortcut bar — bright white for legibility ───────────────────
    cv2.putText(frame,
                "[Q] Quit    [M] Mute    [R] Cinematic",
                (10, h_frame - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, (215, 215, 215), 1, cv2.LINE_AA)


def draw_cinematic_overlay(frame: np.ndarray, fps: float) -> None:
    """
    Minimal cinematic mode indicator.
    Shows a pulsing red REC dot + FPS in the top-right corner.
    Everything else is hidden for a clean recording frame.
    """
    h, w = frame.shape[:2]

    # Pulsing REC dot (brightness oscillates at 2 Hz)
    brightness = int(150 + 105 * (0.5 + 0.5 * math.sin(time.time() * 4.0)))
    dot_color  = (0, 0, brightness)   # BGR red, pulsing

    dot_cx, dot_cy = w - 70, 18
    cv2.circle(frame, (dot_cx, dot_cy), 5, dot_color, -1, cv2.LINE_AA)
    cv2.putText(frame, "REC",
                (dot_cx + 10, dot_cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, dot_color, 1, cv2.LINE_AA)

    # FPS counter stays visible even in cinematic mode (for tuning)
    fps_str = f"FPS {fps:4.1f}"
    cv2.putText(frame, fps_str,
                (w - 115, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 80), 1, cv2.LINE_AA)


def draw_watermark(frame: np.ndarray, cinematic_mode: bool) -> None:
    """
    Author credit watermark, bottom-right corner.
    Rendered in ALL modes. Semi-transparent dark pill background ensures
    it remains readable against any camera feed color.

    NOTE: Slightly more opaque in cinematic mode (intentional — it's a
    passion project marker and should appear in recordings).
    """
    text           = "A passion project by Nomun"
    font, scale, t = cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1
    (tw, th), _    = cv2.getTextSize(text, font, scale, t)
    h, w           = frame.shape[:2]
    x = w - tw - 16
    y = h - 12

    # Semi-transparent dark pill via addWeighted on just that region
    alpha      = 0.72 if not cinematic_mode else 0.55
    roi_y1     = y - th - 5
    roi_y2     = y + 5
    roi_x1     = x - 7
    roi_x2     = x + tw + 7

    # Clamp to frame bounds
    roi_y1 = max(0, roi_y1)
    roi_x1 = max(0, roi_x1)
    roi_y2 = min(h, roi_y2)
    roi_x2 = min(w, roi_x2)

    roi              = frame[roi_y1:roi_y2, roi_x1:roi_x2]
    dark_bg          = np.full_like(roi, (0, 0, 0))
    blended          = cv2.addWeighted(dark_bg, alpha, roi, 1.0 - alpha, 0)
    frame[roi_y1:roi_y2, roi_x1:roi_x2] = blended

    # Thin border on the pill
    cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (45, 45, 45), 1)

    # Text — neutral grey, professional, not distracting
    cv2.putText(frame, text, (x, y), font, scale, (150, 150, 150), t, cv2.LINE_AA)


# ==============================================================================
# SECTION 11 — MAIN THREAD (camera loop, MediaPipe, GUI)
# ==============================================================================
def main() -> None:
    log.info("=" * 64)
    log.info("  Virtual Joystick - Hand Gesture Controller  v3")
    log.info(f"  Target ESP    : {ESP_IP}:{ESP_PORT}")
    log.info(f"  Camera        : index {CAMERA_INDEX}  @  {TARGET_FPS} FPS cap")
    log.info(f"  UDP rate      : {UDP_RATE_HZ} Hz  |  EMA alpha : {EMA_ALPHA}")
    log.info(f"  Reverse speed : S{REVERSE_SPEED}  (palm-down gesture)")
    log.info("  Keyboard      : [Q] Quit | [M] Mute | [R] Cinematic")
    log.info("  Gestures      : Open hand=drive  Fist=E-Stop  PalmDown=Reverse")
    log.info("=" * 64)

    # ──────────────────────────────────────────────────────────────────────────
    # 11-A  Camera Initialization
    # cv2.CAP_DSHOW avoids the 2-5 s MSMF startup delay on Windows 11
    # ──────────────────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        log.critical(
            f"FATAL: Cannot open camera at index {CAMERA_INDEX}.\n"
            "  - Ensure no other application holds the camera\n"
            "  - Try CAMERA_INDEX = 1 or 2 for an external USB camera"
        )
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # Minimise latency / buffer lag
    log.info(
        f"Camera opened: {int(cap.get(3))}x{int(cap.get(4))}"
        f" @ {cap.get(5):.0f} fps (hardware)"
    )

    # ──────────────────────────────────────────────────────────────────────────
    # 11-B  MediaPipe Hands
    # ──────────────────────────────────────────────────────────────────────────
    mp_hands  = mp.solutions.hands
    mp_draw   = mp.solutions.drawing_utils
    mp_styles = mp.solutions.drawing_styles

    hands_model = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=MP_DETECT_CONF,
        min_tracking_confidence=MP_TRACK_CONF,
    )
    log.info(
        f"MediaPipe Hands: detect >= {MP_DETECT_CONF}, track >= {MP_TRACK_CONF}"
    )

    # ──────────────────────────────────────────────────────────────────────────
    # 11-C  EMA Filters + Smart Debouncer
    # ──────────────────────────────────────────────────────────────────────────
    ema_speed  = EMAFilter(alpha=EMA_ALPHA)
    ema_tilt   = EMAFilter(alpha=EMA_ALPHA)
    debouncer  = SmartDebouncer()

    # ──────────────────────────────────────────────────────────────────────────
    # 11-D  Start Background Threads
    # ──────────────────────────────────────────────────────────────────────────
    audio_t = threading.Thread(target=audio_thread_fn, name="AudioThread", daemon=True)
    udp_t   = threading.Thread(target=udp_thread_fn,   name="UDPThread",   daemon=True)
    audio_t.start()
    udp_t.start()
    log.info("Audio and UDP threads launched.")

    # ──────────────────────────────────────────────────────────────────────────
    # 11-E  Main Loop State Variables
    # ──────────────────────────────────────────────────────────────────────────
    frame_interval = 1.0 / TARGET_FPS
    fps_timer      = time.perf_counter()
    fps_count      = 0
    measured_fps   = 0.0
    cinematic_mode = False
    lm             = None       # MediaPipe landmark list; None if no hand detected

    try:
        while True:
            loop_start = time.perf_counter()

            # ── Frame grab ────────────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret or frame is None:
                log.warning("Frame grab failed — retrying in 50 ms ...")
                time.sleep(0.05)
                continue

            # Mirror L/R: makes the hand feel like a natural steering controller.
            # Without this, the user's right-hand tilt moves the car LEFT (confusing).
            frame  = cv2.flip(frame, 1)
            h, w   = frame.shape[:2]

            # ── MediaPipe Inference ───────────────────────────────────────────
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands_model.process(rgb)
            rgb.flags.writeable = True

            # ── Gesture Computation + EMA Smoothing ───────────────────────────
            hand_visible  = False
            fist_detected = False
            palm_down     = False
            lm            = None

            if results.multi_hand_landmarks:
                hand_visible = True
                lm = results.multi_hand_landmarks[0].landmark

                # Draw the default MediaPipe 21-point skeleton first (bottom layer)
                mp_draw.draw_landmarks(
                    frame,
                    results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS,
                    mp_styles.get_default_hand_landmarks_style(),
                    mp_styles.get_default_hand_connections_style(),
                )

                fist_detected = is_fist(lm)
                # Palm-down is only meaningful when the hand is NOT a fist;
                # a closed fist always takes Emergency Stop priority.
                palm_down = is_palm_down(lm) and not fist_detected

                if fist_detected:
                    # Emergency Stop: zero everything immediately
                    raw_speed, raw_tilt = 0.0, 0.0
                elif palm_down:
                    # Reverse Gear: fixed negative speed, but steer normally
                    raw_speed = float(REVERSE_SPEED)   # e.g. -40.0
                    raw_tilt  = compute_tilt(lm)
                else:
                    # Normal forward drive: proportional speed + tilt
                    raw_speed = compute_speed(lm)
                    raw_tilt  = compute_tilt(lm)

                # Apply EMA smoothing to suppress jitter
                smooth_speed = ema_speed.update(raw_speed)
                smooth_tilt  = ema_tilt.update(raw_tilt)
            else:
                # No hand: EMA decays toward 0 gracefully (avoids abrupt stop jump)
                smooth_speed = ema_speed.update(0.0)
                smooth_tilt  = ema_tilt.update(0.0)

            # Quantize smoothed floats to integers for UDP and HUD
            speed = int(smooth_speed)
            tilt  = int(smooth_tilt)

            # Apply dead zones AFTER smoothing.
            # NOTE: Dead zone is symmetric — catches small negatives too,
            # so EMA-transition noise near 0 doesn't trigger spurious REVERSE.
            if -SPEED_DEAD_ZONE < speed < SPEED_DEAD_ZONE:
                speed = 0
            if abs(tilt) < TILT_DEAD_ZONE:
                tilt = 0

            # Hard override: fist always forces full stop regardless of EMA state
            if fist_detected:
                speed, tilt = 0, 0

            command = classify_command(speed, tilt, fist_detected)

            # ── Write shared state (UDP thread reads this) ────────────────────
            with state_lock:
                shared_state["speed"]        = speed
                shared_state["tilt"]         = tilt
                shared_state["command"]      = command
                shared_state["fist"]         = fist_detected
                shared_state["hand_visible"] = hand_visible
                shared_state["fps"]          = measured_fps
                muted                        = shared_state["muted"]

            # ── Smart Audio Debouncing ────────────────────────────────────────
            debouncer.update(command, speed)

            # ── FPS Measurement (updated every second) ────────────────────────
            fps_count += 1
            now = time.perf_counter()
            if now - fps_timer >= 1.0:
                measured_fps = fps_count / (now - fps_timer)
                fps_count    = 0
                fps_timer    = now

            # ── Layer 2: Iron Man Neon Geometry (drawn in BOTH modes) ─────────
            if hand_visible and lm is not None:
                draw_iron_man_visuals(frame, lm, speed, w, h)

            # ── Layer 3: HUD Panel OR Cinematic Minimal Overlay ───────────────
            if not cinematic_mode:
                draw_hud(
                    frame, speed, tilt, command, measured_fps,
                    muted, fist_detected, hand_visible,
                )
            else:
                draw_cinematic_overlay(frame, measured_fps)

            # ── Layer 4: Watermark (always visible) ───────────────────────────
            draw_watermark(frame, cinematic_mode)

            # ── Display ───────────────────────────────────────────────────────
            # Window title uses ONLY standard ASCII characters to avoid
            # OpenCV window-title rendering bugs on Windows
            cv2.imshow("Virtual Joystick - Hand Gesture Controller", frame)

            # ── Keyboard Input ────────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):          # 'q' or Escape
                log.info("Quit key pressed — initiating shutdown ...")
                break

            elif key == ord("m"):
                with state_lock:
                    shared_state["muted"] = not shared_state["muted"]
                    is_muted = shared_state["muted"]
                announce("muted" if is_muted else "audio on")
                log.info(f"Audio {'MUTED' if is_muted else 'UNMUTED'} by user.")

            elif key == ord("r"):
                cinematic_mode = not cinematic_mode
                mode_name = "CINEMATIC" if cinematic_mode else "CONTROL"
                announce(f"{mode_name.lower()} mode")
                log.info(f"Mode toggled -> {mode_name}")

            # ── Hard FPS Cap: sleep for the remaining frame budget ────────────
            elapsed   = time.perf_counter() - loop_start
            sleep_for = frame_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received — shutting down ...")

    # ──────────────────────────────────────────────────────────────────────────
    # 11-F  Graceful Shutdown
    # ──────────────────────────────────────────────────────────────────────────
    finally:
        log.info("Cleaning up threads and resources ...")

        # Signal all threads to exit
        with state_lock:
            shared_state["running"] = False

        # Send None sentinel to unblock the audio thread's queue.get()
        try:
            audio_queue.put(None, timeout=1.0)
        except queue.Full:
            pass

        # Wait with timeouts so we never hang indefinitely
        audio_t.join(timeout=4.0)
        udp_t.join(timeout=2.0)

        if audio_t.is_alive():
            log.warning("Audio thread did not exit within timeout (4 s).")
        if udp_t.is_alive():
            log.warning("UDP thread did not exit within timeout (2 s).")

        cap.release()
        cv2.destroyAllWindows()
        hands_model.close()

        log.info("=" * 64)
        log.info("  Virtual Joystick - stopped cleanly.")
        log.info("=" * 64)


# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    main()
