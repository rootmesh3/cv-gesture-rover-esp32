#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║          VIRTUAL JOYSTICK — Hand Gesture Robot Car Controller                ║
║                      Laptop-Side UDP Controller                              ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Architecture:                                                               ║
║    Thread 1 (Main)  : Camera capture, OpenCV GUI, MediaPipe (30 FPS cap)    ║
║    Thread 2 (Audio) : Queue-based pyttsx3 TTS — never blocks video feed     ║
║    Thread 3 (UDP)   : Continuous 20 Hz heartbeat stream to ESP module       ║
║                                                                              ║
║  UDP Payload Format : S[Speed]T[Tilt]  →  e.g. "S85T-12"                   ║
║  Gesture Controls   : Open hand = move, thumb-index gap = speed,            ║
║                        hand tilt = steering, fist = emergency stop           ║
║  Keyboard Controls  : [Q] Quit   |   [M] Mute / Unmute audio                ║
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

# ── Third-Party ───────────────────────────────────────────────────────────────
import cv2
import numpy as np
import mediapipe as mp
import pyttsx3


# ==============================================================================
# ██████  SECTION 1 — CONFIGURATION
# ==============================================================================

# ─── Network ──────────────────────────────────────────────────────────────────
ESP_IP   = "192.168.4.1"   # ← CHANGE: IP of your ESP module.
#                                Use "192.168.4.1" for ESP in Access Point mode
#                                or check your router DHCP table for station mode
ESP_PORT = 4210             # ← CHANGE: UDP port your ESP firmware listens on

# ─── Camera ───────────────────────────────────────────────────────────────────
CAMERA_INDEX = 0            # 0 = default laptop webcam; increment for USB cameras
TARGET_FPS   = 30           # Hard cap on the main camera loop (saves CPU)

# ─── UDP Heartbeat ────────────────────────────────────────────────────────────
UDP_RATE_HZ  = 20                   # Transmit command 20 times per second
UDP_INTERVAL = 1.0 / UDP_RATE_HZ   # 50 ms between packets

# ─── Gesture Tuning ───────────────────────────────────────────────────────────
# These two ratios define the full speed range.
# Ratio = dist(ThumbTip→IndexTip) / dist(Wrist→MiddleBase)
# Pinch fully → ratio ≈ 0.10  (0% speed)
# Fingers fully apart → ratio ≈ 1.30  (100% speed)
SPEED_RATIO_MIN = 0.15     # Ratio below this → speed = 0
SPEED_RATIO_MAX = 1.30     # Ratio above this → speed = 100 (clamped)
SPEED_DEAD_ZONE = 5        # Speed % values < this are snapped to 0
TILT_DEAD_ZONE  = 10       # Angles within ±10° are treated as "Straight"

# ─── HUD Layout ───────────────────────────────────────────────────────────────
HUD_X, HUD_Y = 10, 10      # Top-left corner of the HUD panel (pixels)
HUD_W, HUD_H = 330, 210    # Width × Height of the HUD panel

# ─── MediaPipe Confidence ─────────────────────────────────────────────────────
MP_DETECT_CONF = 0.72      # Initial hand detection confidence threshold
MP_TRACK_CONF  = 0.60      # Continuous tracking confidence threshold


# ==============================================================================
# ██████  SECTION 2 — LOGGING SETUP
# ==============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)-8s] %(asctime)s | %(threadName)-12s | %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


# ==============================================================================
# ██████  SECTION 3 — SHARED STATE & SYNCHRONISATION PRIMITIVES
# ==============================================================================
# One lock protects the entire shared_state dictionary.
# All threads read/write ONLY while holding this lock.
state_lock = threading.Lock()

shared_state: dict = {
    "speed"      : 0,        # int  0–100 — current speed percentage
    "tilt"       : 0,        # int  degrees, negative=left, positive=right
    "command"    : "STOP",   # str  human-readable label for current action
    "fist"       : False,    # bool emergency-stop flag (closed fist detected)
    "running"    : True,     # bool set to False to signal all threads to exit
    "muted"      : False,    # bool TTS mute toggle
    "fps"        : 0.0,      # float measured main-thread FPS (display only)
    "hand_visible": False,   # bool True when MediaPipe detects a hand
}

# Queue used to pass text to the Audio thread (maxsize prevents memory runaway)
audio_queue: queue.Queue = queue.Queue(maxsize=6)


# ==============================================================================
# ██████  SECTION 4 — THREAD 2: AUDIO (pyttsx3 — non-blocking TTS)
# ==============================================================================
def audio_thread_fn() -> None:
    """
    Dedicated TTS thread.  Consumes text items from `audio_queue` and
    speaks them via pyttsx3.  A `None` sentinel shuts the thread down.

    By isolating pyttsx3 in its own thread we guarantee that
    `engine.runAndWait()` — which is blocking — never stalls the video feed.
    """
    try:
        engine = pyttsx3.init()
        engine.setProperty("rate",   160)   # words per minute (150–180 = natural)
        engine.setProperty("volume", 1.0)   # 0.0–1.0
        log.info("pyttsx3 engine initialised successfully.")
    except Exception as exc:
        log.error(f"pyttsx3 init failed: {exc}  —  Audio thread will not run.")
        # Mark audio thread as gone but don't crash the program
        return

    while True:
        try:
            # Block with a timeout so we can periodically check `running`
            text = audio_queue.get(timeout=0.5)
        except queue.Empty:
            with state_lock:
                if not shared_state["running"]:
                    break       # Main thread has set running=False → clean exit
            continue

        if text is None:        # Sentinel value sent by main thread at shutdown
            audio_queue.task_done()
            break

        with state_lock:
            muted = shared_state["muted"]

        if not muted:
            try:
                engine.say(text)
                engine.runAndWait()
            except RuntimeError as exc:
                # runAndWait can raise if the engine loop is already running
                log.warning(f"TTS RuntimeError (ignored): {exc}")
            except Exception as exc:
                log.warning(f"TTS error: {exc}")

        audio_queue.task_done()

    log.info("Audio thread exiting cleanly.")


def announce(text: str) -> None:
    """
    Non-blocking helper — pushes `text` onto the audio queue.
    Silently drops the message if the queue is already full.
    """
    try:
        audio_queue.put_nowait(text)
    except queue.Full:
        pass   # Better to drop an announcement than to block the main loop


# ==============================================================================
# ██████  SECTION 5 — THREAD 3: UDP COMMUNICATION (20 Hz heartbeat)
# ==============================================================================
def udp_thread_fn() -> None:
    """
    Transmits the current command as a UDP datagram to the ESP module at
    exactly UDP_RATE_HZ (20 Hz).  This continuous stream serves as the
    hardware heartbeat: if the laptop dies the ESP stops receiving packets
    and can implement its own timeout/failsafe.

    Payload format (Option C):  S[Speed]T[Tilt]
      Speed : integer 0–100
      Tilt  : signed integer, negative = left, positive = right
    Examples:
      "S85T-12"   →  85% speed, 12° left tilt
      "S0T0"      →  stopped, straight
      "S100T30"   →  full speed, 30° right tilt
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.05)   # Non-blocking enough; sendto on UDP rarely blocks
        log.info(f"UDP socket created. Target: {ESP_IP}:{ESP_PORT}")
    except OSError as exc:
        log.error(f"Cannot create UDP socket: {exc}  —  UDP thread exiting.")
        return

    last_payload_logged = ""

    while True:
        tick_start = time.perf_counter()

        with state_lock:
            if not shared_state["running"]:
                break
            speed = shared_state["speed"]
            tilt  = shared_state["tilt"]

        # Build payload string
        payload = f"S{speed}T{tilt}"

        try:
            sock.sendto(payload.encode("utf-8"), (ESP_IP, ESP_PORT))
            # Only log when the payload actually changes (avoids log spam)
            if payload != last_payload_logged:
                log.debug(f"UDP TX → {payload}")
                last_payload_logged = payload
        except OSError as exc:
            log.warning(f"UDP send error (will retry): {exc}")

        # ── Precise rate control: sleep for the remainder of the 50 ms window ──
        elapsed    = time.perf_counter() - tick_start
        sleep_for  = UDP_INTERVAL - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

    try:
        sock.close()
    except Exception:
        pass
    log.info("UDP thread exiting cleanly.")


# ==============================================================================
# ██████  SECTION 6 — GESTURE PROCESSING (pure functions, main thread only)
# ==============================================================================

def _dist(a, b) -> float:
    """Euclidean distance between two MediaPipe NormalizedLandmark objects."""
    return math.hypot(a.x - b.x, a.y - b.y)


def is_fist(lm: list) -> bool:
    """
    Emergency-stop detector.  Returns True when ALL four fingers are curled.

    Logic: in image-space y increases downward.  When a finger is curled, its
    tip (e.g. landmark 8) will have a HIGHER y value than its proximal
    inter-phalangeal (PIP) joint (e.g. landmark 6).  We test all four fingers.

    Finger landmark pairs  (Tip index, PIP index):
        Index  → (8,  6)
        Middle → (12, 10)
        Ring   → (16, 14)
        Pinky  → (20, 18)
    """
    finger_pairs = [(8, 6), (12, 10), (16, 14), (20, 18)]
    return all(lm[tip].y > lm[pip].y for tip, pip in finger_pairs)


def compute_speed(lm: list) -> int:
    """
    Proportional speed from pinch aperture.

    1. Measure dist(ThumbTip[4] → IndexTip[8]).
    2. Normalise by the stable anatomical reference dist(Wrist[0] → MiddleBase[9]).
       This makes the measurement hand-size-independent and robust to camera zoom.
    3. Map the resulting ratio to 0–100, clamped at SPEED_RATIO_MIN/MAX.
    4. Apply SPEED_DEAD_ZONE: values < threshold snap to 0.

    Returns: integer in range [0, 100].
    """
    thumb_idx_dist    = _dist(lm[4], lm[8])
    wrist_mid_dist    = _dist(lm[0], lm[9])

    if wrist_mid_dist < 1e-6:          # Guard against division by zero
        return 0

    ratio         = thumb_idx_dist / wrist_mid_dist
    ratio_clamped = max(SPEED_RATIO_MIN, min(SPEED_RATIO_MAX, ratio))

    speed = int(
        (ratio_clamped - SPEED_RATIO_MIN)
        / (SPEED_RATIO_MAX - SPEED_RATIO_MIN)
        * 100
    )
    return speed if speed >= SPEED_DEAD_ZONE else 0


def compute_tilt(lm: list) -> int:
    """
    Hand tilt angle for left/right steering.

    The vector from Wrist[0] → MiddleFingerBase[9] defines the hand's
    longitudinal axis.  We measure its deviation from vertical (pointing up).

    Convention (matching a natural steering motion):
        Tilt left  (anti-clockwise in screen)  → negative degrees
        Tilt right (clockwise in screen)       → positive degrees
        ±TILT_DEAD_ZONE                        → forced to 0° (Straight)

    Returns: signed integer degrees.
    """
    dx =  lm[9].x - lm[0].x     # Positive = wrist moved right of middle base
    dy = -(lm[9].y - lm[0].y)   # Flip y because image-space y is inverted

    angle_deg = int(math.degrees(math.atan2(dx, dy)))

    return 0 if abs(angle_deg) < TILT_DEAD_ZONE else angle_deg


def classify_command(speed: int, tilt: int, fist: bool) -> str:
    """
    Maps numeric state to a human-readable command label used for
    announcements and HUD display.
    """
    if fist:
        return "EMERGENCY STOP"
    if speed == 0:
        return "STOP"
    if tilt < -TILT_DEAD_ZONE:
        return "FORWARD LEFT"
    if tilt > TILT_DEAD_ZONE:
        return "FORWARD RIGHT"
    return "FORWARD"


# ==============================================================================
# ██████  SECTION 7 — HUD DRAWING (OpenCV, main thread only)
# ==============================================================================

def _draw_speed_bar(
    frame: np.ndarray,
    speed: int,
    x: int, y: int,
    bar_w: int = 200,
    bar_h: int = 18,
) -> None:
    """
    Horizontal gradient speed bar: Green (0%) → Yellow (50%) → Red (100%).

    Implementation note: we draw individual vertical scan lines so the
    gradient is rendered mathematically rather than relying on OpenCV
    gradient APIs (which don't exist in the drawing module).
    """
    # ── Background track ──
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (30, 30, 30), -1)
    cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (90, 90, 90), 1)

    if speed <= 0:
        return

    fill_pixels = max(1, int(bar_w * speed / 100))

    for i in range(fill_pixels):
        norm = i / max(bar_w - 1, 1)   # 0.0 → 1.0 across the full bar width
        if norm < 0.5:
            # Green (0,255,0) → Yellow (0,255,255) in BGR
            r = int(norm * 2 * 255)
            g = 255
        else:
            # Yellow → Red (0,0,255) in BGR
            r = 255
            g = int((1.0 - (norm - 0.5) * 2) * 255)
        b = 0
        cv2.line(frame, (x + i, y + 1), (x + i, y + bar_h - 1), (b, g, r), 1)

    # ── Speed percentage label to the right of the bar ──
    cv2.putText(
        frame, f"{speed}%",
        (x + bar_w + 6, y + bar_h - 2),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA,
    )


def _draw_orientation_horizon(
    frame: np.ndarray,
    tilt_deg: int,
    cx: int, cy: int,
    radius: int = 52,
) -> None:
    """
    Aviation-style artificial horizon (attitude indicator).

    Elements:
        • Outer reference circle with degree tick marks at ±30°, ±60°, ±90°.
        • Fixed aircraft-symbol crosshair at centre.
        • Rotating needle indicating the current hand tilt.
          - Cyan   = straight (within dead zone)
          - Red    = tilting left
          - Green  = tilting right

    Angle convention: 0° points straight UP on screen (−90° in OpenCV polar
    coordinates where 0° = right).  We rotate by −90° to achieve this.
    """
    # ── Outer ring ──
    cv2.circle(frame, (cx, cy), radius, (70, 70, 70), 1, cv2.LINE_AA)

    # ── Degree tick marks ──
    for tick in [-90, -60, -30, 0, 30, 60, 90]:
        # Convert conceptual angle (0°=up) to OpenCV's polar angle (0°=right)
        rad        = math.radians(tick - 90)
        outer_x    = int(cx + radius * math.cos(rad))
        outer_y    = int(cy + radius * math.sin(rad))
        inner_len  = radius - (12 if tick % 90 == 0 else (8 if tick % 30 == 0 else 5))
        inner_x    = int(cx + inner_len * math.cos(rad))
        inner_y    = int(cy + inner_len * math.sin(rad))
        tick_color = (130, 130, 130) if tick == 0 else (80, 80, 80)
        cv2.line(frame, (inner_x, inner_y), (outer_x, outer_y), tick_color, 1, cv2.LINE_AA)

    # Label key ticks
    for label_deg, label_txt in [(-90, "L"), (90, "R"), (0, "")]:
        if label_txt:
            rad = math.radians(label_deg - 90)
            lx  = int(cx + (radius + 10) * math.cos(rad))
            ly  = int(cy + (radius + 10) * math.sin(rad)) + 4
            cv2.putText(frame, label_txt, (lx - 5, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (100, 100, 100), 1, cv2.LINE_AA)

    # ── Fixed aircraft crosshair (the "aircraft" that the horizon moves under) ──
    cv2.line(frame, (cx - 14, cy), (cx - 5, cy), (180, 180, 50), 2, cv2.LINE_AA)
    cv2.line(frame, (cx + 5,  cy), (cx + 14, cy), (180, 180, 50), 2, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 3, (180, 180, 50), -1, cv2.LINE_AA)

    # ── Rotating needle ──
    needle_rad  = math.radians(tilt_deg - 90)
    needle_len  = radius - 5
    tip_x       = int(cx + needle_len * math.cos(needle_rad))
    tip_y       = int(cy + needle_len * math.sin(needle_rad))

    if abs(tilt_deg) <= TILT_DEAD_ZONE:
        needle_color = (0, 240, 240)     # Cyan  → Straight
    elif tilt_deg < 0:
        needle_color = (50, 50, 255)     # Red   → Left tilt
    else:
        needle_color = (50, 220, 50)     # Green → Right tilt

    cv2.line(frame, (cx, cy), (tip_x, tip_y), needle_color, 2, cv2.LINE_AA)
    cv2.circle(frame, (tip_x, tip_y), 4, needle_color, -1, cv2.LINE_AA)

    # ── Degree readout below the dial ──
    sign_str = f"{tilt_deg:+d}°"
    cv2.putText(frame, sign_str, (cx - 20, cy + radius + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1, cv2.LINE_AA)


def draw_hud(
    frame:        np.ndarray,
    speed:        int,
    tilt:         int,
    command:      str,
    fps:          float,
    muted:        bool,
    fist:         bool,
    hand_visible: bool,
) -> None:
    """
    Renders the complete professional HUD overlay onto `frame` in-place.

    Layout (inside the semi-transparent panel):
        ┌──────────────────────────────────┐
        │  VIRTUAL JOYSTICK               │  ← title bar
        │  FORWARD LEFT      (command)    │
        │  SPEED ▓▓▓▓▓▓░░░░  72%          │  ← gradient bar
        │                                 │
        │  [horizon dial]  ● HAND: OK     │  ← orientation + status
        │                  🔊 AUDIO ON    │
        └──────────────────────────────────┘

    Corner badges:
        Top-right : FPS counter
        Bottom    : keyboard shortcut reminder
    """
    h_frame, w_frame = frame.shape[:2]

    # ── 1. Semi-transparent dark panel via addWeighted ──────────────────────
    # We draw on an overlay copy and then blend it into the real frame.
    # This avoids darkening landmarks drawn ON TOP of the panel afterward.
    overlay = frame.copy()
    p1 = (HUD_X,           HUD_Y)
    p2 = (HUD_X + HUD_W,   HUD_Y + HUD_H)
    cv2.rectangle(overlay, p1, p2, (15, 15, 15), -1)
    # alpha=0.70 panel / 0.30 original frame for a dark-but-see-through look
    cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)

    # Panel border
    cv2.rectangle(frame, p1, p2, (65, 65, 65), 1)

    # ── 2. Title bar ─────────────────────────────────────────────────────────
    cv2.putText(frame, "VIRTUAL  JOYSTICK",
                (HUD_X + 10, HUD_Y + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 195, 255), 1, cv2.LINE_AA)
    # Thin separator line
    cv2.line(frame,
             (HUD_X + 4,         HUD_Y + 26),
             (HUD_X + HUD_W - 4, HUD_Y + 26),
             (55, 55, 55), 1)

    # ── 3. Command label ─────────────────────────────────────────────────────
    cmd_color = (40, 40, 255) if fist else (0, 255, 130)
    cv2.putText(frame, command,
                (HUD_X + 10, HUD_Y + 50),
                cv2.FONT_HERSHEY_DUPLEX, 0.65, cmd_color, 1, cv2.LINE_AA)

    # ── 4. Speed section ─────────────────────────────────────────────────────
    cv2.putText(frame, "SPEED",
                (HUD_X + 10, HUD_Y + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 140, 140), 1, cv2.LINE_AA)
    _draw_speed_bar(frame, speed, HUD_X + 10, HUD_Y + 76, bar_w=200, bar_h=17)

    # ── 5. Orientation horizon dial ──────────────────────────────────────────
    horizon_cx = HUD_X + 68
    horizon_cy = HUD_Y + 155
    cv2.putText(frame, "TILT",
                (horizon_cx - 14, HUD_Y + 108),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 140), 1, cv2.LINE_AA)
    _draw_orientation_horizon(frame, tilt, horizon_cx, horizon_cy, radius=50)

    # ── 6. Status panel (right side of HUD) ──────────────────────────────────
    sx = HUD_X + 152   # Right-column x origin

    # ESP connection target
    cv2.putText(frame, f"ESP  {ESP_IP}",
                (sx, HUD_Y + 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1, cv2.LINE_AA)
    cv2.putText(frame, f"PORT {ESP_PORT}",
                (sx, HUD_Y + 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1, cv2.LINE_AA)

    # Hand detection status
    hand_color = (0, 230, 110) if hand_visible else (70, 70, 70)
    hand_text  = "HAND : DETECTED" if hand_visible else "HAND : NONE"
    cv2.putText(frame, hand_text,
                (sx, HUD_Y + 145),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, hand_color, 1, cv2.LINE_AA)

    # Mute status
    mute_color = (80, 80, 200) if muted else (0, 195, 255)
    mute_text  = "[M] MUTED" if muted else "[M] AUDIO ON"
    cv2.putText(frame, mute_text,
                (sx, HUD_Y + 162),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, mute_color, 1, cv2.LINE_AA)

    # UDP heartbeat indicator (static label — the thread handles timing)
    cv2.putText(frame, f"UDP @ {UDP_RATE_HZ} Hz",
                (sx, HUD_Y + 179),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1, cv2.LINE_AA)

    # ── 7. FPS counter — top-right corner of frame ────────────────────────────
    fps_txt = f"FPS {fps:4.1f}"
    cv2.putText(frame, fps_txt,
                (w_frame - 115, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (140, 255, 140), 1, cv2.LINE_AA)

    # ── 8. Keyboard shortcut reminder — bottom of frame ───────────────────────
    cv2.putText(frame, "[Q] Quit      [M] Mute / Unmute Audio",
                (10, h_frame - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (90, 90, 90), 1, cv2.LINE_AA)


# ==============================================================================
# ██████  SECTION 8 — MAIN THREAD (camera loop, MediaPipe, GUI)
# ==============================================================================
def main() -> None:
    log.info("=" * 60)
    log.info(" Virtual Joystick — starting up")
    log.info(f"  Target  : {ESP_IP}:{ESP_PORT}")
    log.info(f"  Camera  : index {CAMERA_INDEX}  @  {TARGET_FPS} FPS cap")
    log.info(f"  UDP rate: {UDP_RATE_HZ} Hz heartbeat")
    log.info("=" * 60)

    # ──────────────────────────────────────────────────────────────────────────
    # 8-A  Camera Initialisation
    # ──────────────────────────────────────────────────────────────────────────
    # cv2.CAP_DSHOW is a Windows-specific backend that dramatically reduces
    # the 2–5 second delay seen with the default MSMF backend on Win 11.
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        log.critical(
            f"FATAL: Cannot open camera at index {CAMERA_INDEX}.\n"
            "  • Check that no other app is using the camera.\n"
            "  • Try a different CAMERA_INDEX (0, 1, 2 …).\n"
            "  • On WSL2: use a USB-passthrough or run natively."
        )
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # Keep latency minimal

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    log.info(f"Camera opened: {actual_w}×{actual_h} @ {actual_fps:.0f} FPS (hardware)")

    # ──────────────────────────────────────────────────────────────────────────
    # 8-B  MediaPipe Hands
    # ──────────────────────────────────────────────────────────────────────────
    mp_hands   = mp.solutions.hands
    mp_draw    = mp.solutions.drawing_utils
    mp_styles  = mp.solutions.drawing_styles

    hands_model = mp_hands.Hands(
        static_image_mode=False,      # Video stream mode (uses tracking)
        max_num_hands=1,              # Only the primary hand matters here
        min_detection_confidence=MP_DETECT_CONF,
        min_tracking_confidence=MP_TRACK_CONF,
    )
    log.info(f"MediaPipe Hands initialised (detect≥{MP_DETECT_CONF}, track≥{MP_TRACK_CONF})")

    # ──────────────────────────────────────────────────────────────────────────
    # 8-C  Start Background Threads
    # ──────────────────────────────────────────────────────────────────────────
    audio_t = threading.Thread(
        target=audio_thread_fn, name="AudioThread", daemon=True
    )
    udp_t = threading.Thread(
        target=udp_thread_fn, name="UDPThread", daemon=True
    )

    audio_t.start()
    udp_t.start()
    log.info("Audio and UDP threads launched.")

    # ──────────────────────────────────────────────────────────────────────────
    # 8-D  Main Loop Variables
    # ──────────────────────────────────────────────────────────────────────────
    frame_interval  = 1.0 / TARGET_FPS   # Desired seconds per frame
    last_command    = ""                  # For change-detection → announcements
    fps_timer       = time.perf_counter()
    fps_count       = 0
    measured_fps    = 0.0

    try:
        while True:
            loop_start = time.perf_counter()

            # ── Read frame ──────────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret or frame is None:
                log.warning("Frame grab failed — retrying in 50 ms …")
                time.sleep(0.05)
                continue

            # ── Mirror L/R to make hand feel like a natural "steering wheel" ──
            # Without this flip, the user's right hand moves the car left (confusing).
            frame = cv2.flip(frame, 1)

            # ── MediaPipe inference ─────────────────────────────────────────
            # Convert to RGB (MediaPipe requirement) and mark non-writeable
            # to allow MediaPipe to avoid an internal copy for efficiency.
            rgb                  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable  = False
            results              = hands_model.process(rgb)
            rgb.flags.writeable  = True

            # ── Gesture computation ─────────────────────────────────────────
            hand_visible  = False
            speed         = 0
            tilt          = 0
            fist_detected = False

            if results.multi_hand_landmarks:
                hand_visible = True
                lm           = results.multi_hand_landmarks[0].landmark

                # Draw the hand skeleton over the video frame
                mp_draw.draw_landmarks(
                    frame,
                    results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS,
                    mp_styles.get_default_hand_landmarks_style(),
                    mp_styles.get_default_hand_connections_style(),
                )

                fist_detected = is_fist(lm)

                if fist_detected:
                    # Emergency stop: force everything to zero immediately
                    speed, tilt = 0, 0
                else:
                    speed = compute_speed(lm)
                    tilt  = compute_tilt(lm)

            command = classify_command(speed, tilt, fist_detected)

            # ── Write to shared state (UDP thread reads this) ───────────────
            with state_lock:
                shared_state["speed"]        = speed
                shared_state["tilt"]         = tilt
                shared_state["command"]      = command
                shared_state["fist"]         = fist_detected
                shared_state["hand_visible"] = hand_visible
                shared_state["fps"]          = measured_fps
                muted                        = shared_state["muted"]

            # ── Announce state changes via TTS ──────────────────────────────
            if command != last_command:
                log.info(f"Command: {last_command!r:20s} → {command!r}")
                # Convert to natural speech: "FORWARD LEFT" → "forward left"
                announce(command.replace("_", " ").lower())
                last_command = command

            # ── FPS measurement (updated every second) ──────────────────────
            fps_count += 1
            now = time.perf_counter()
            if now - fps_timer >= 1.0:
                measured_fps = fps_count / (now - fps_timer)
                fps_count    = 0
                fps_timer    = now

            # ── Render HUD overlay ──────────────────────────────────────────
            draw_hud(
                frame, speed, tilt, command, measured_fps,
                muted, fist_detected, hand_visible,
            )

            # ── Show window ─────────────────────────────────────────────────
            cv2.imshow("Virtual Joystick — Hand Gesture Robot Controller", frame)

            # ── Keyboard handling ───────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:   # 'q' or Esc
                log.info("Quit key pressed — initiating shutdown …")
                break

            elif key == ord("m"):
                with state_lock:
                    shared_state["muted"] = not shared_state["muted"]
                    is_muted = shared_state["muted"]
                announce("audio muted" if is_muted else "audio on")
                log.info(f"Audio {'MUTED' if is_muted else 'UNMUTED'} by user.")

            # ── Hard FPS cap: sleep for the remainder of the frame budget ───
            elapsed    = time.perf_counter() - loop_start
            sleep_for  = frame_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — shutting down …")

    # ──────────────────────────────────────────────────────────────────────────
    # 8-E  Graceful Shutdown
    # Signal all background threads, drain queue, wait for joins.
    # ──────────────────────────────────────────────────────────────────────────
    finally:
        log.info("Cleaning up …")

        # 1. Tell threads to exit
        with state_lock:
            shared_state["running"] = False

        # 2. Send sentinel to audio queue so the thread unblocks and exits
        try:
            audio_queue.put(None, timeout=1.0)
        except queue.Full:
            pass

        # 3. Wait for threads (with a timeout so we never hang indefinitely)
        audio_t.join(timeout=4.0)
        udp_t.join(timeout=2.0)

        if audio_t.is_alive():
            log.warning("Audio thread did not exit cleanly within timeout.")
        if udp_t.is_alive():
            log.warning("UDP thread did not exit cleanly within timeout.")

        # 4. Release camera and destroy OpenCV windows
        cap.release()
        cv2.destroyAllWindows()

        # 5. Close MediaPipe resources
        hands_model.close()

        log.info("=" * 60)
        log.info(" Virtual Joystick — stopped cleanly.")
        log.info("=" * 60)


# ==============================================================================
# ██████  ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    main()
