/*
 * ============================================================================
 *  VirtualJoystick_ESP32.ino
 *  ESP32 Firmware — Hand Gesture Robot Car Controller
 * ============================================================================
 *
 *  Board   : ESP-WROOM-32 (ESP32 Dev Board)
 *  Motor   : L298N Dual H-Bridge Driver
 *  IDE     : PlatformIO |  Board: "ESP32 Dev Module"
 *  Core    : ESP32 Arduino Core v2.x  (ledc API used — NOT analogWrite)
 *
 *  PROJECT OVERVIEW
 *  ──────────────────────────────────────────────────────────────────────────
 *  The laptop runs a Python script (virtual_joystick.py) that uses MediaPipe
 *  to read hand gestures from a webcam. It continuously streams a compact UDP
 *  packet to this ESP32 at 20 Hz:
 *
 *      Payload format : "S[Speed]T[Tilt]"
 *      Example        : "S85T-12"  →  85% speed, 12° left turn (forward)
 *      Example        : "S-60T30"  →  60% speed, 30° right turn (REVERSE)
 *      Example        : "S0T0"     →  Emergency stop
 *
 *  This firmware:
 *    1. Boots as a Wi-Fi Access Point (SSID: VirtualJoystick_AP).
 *    2. Listens on UDP port 4210 for those packets.
 *    3. Parses speed (−100 → +100) and tilt (−90 → +90).
 *    4. Computes differential left/right PWM with motor balance calibration.
 *    5. Watches a 500 ms heartbeat timer — if the controller goes silent,
 *       it executes an Emergency Stop to prevent runaway behaviour.
 *
 *  PIN MAPPING (L298N ↔ ESP32)
 *  ──────────────────────────────────────────────────────────────────────────
 *  L298N Pin   │  ESP32 GPIO   │  LEDC Channel  │  Role
 *  ────────────┼───────────────┼────────────────┼──────────────────────────
 *  ENA         │  GPIO 32      │  Channel  0    │  Right Motor PWM enable
 *  IN1         │  GPIO 25      │  —             │  Right Motor FORWARD
 *  IN2         │  GPIO 26      │  —             │  Right Motor REVERSE
 *  IN3         │  GPIO 27      │  —             │  Left  Motor FORWARD
 *  IN4         │  GPIO 14      │  —             │  Left  Motor REVERSE
 *  ENB         │  GPIO 33      │  Channel  1    │  Left  Motor PWM enable
 *  ────────────┼───────────────┼────────────────┼──────────────────────────
 *
 *  STEERING LOGIC SUMMARY
 *  ──────────────────────────────────────────────────────────────────────────
 *  • Positive speed  → Forward direction
 *  • Negative speed  → Reverse direction  (future Python update; fully supported)
 *  • Tilt = 0        → Both motors at equal calibrated speed (straight)
 *  • Tilt > 0 (right)→ RIGHT motor PWM reduced proportionally; LEFT at full
 *  • Tilt < 0 (left) → LEFT  motor PWM reduced proportionally; RIGHT at full
 *  • Reduction factor = abs(tilt) / 90  →  0.0 (straight) .. 1.0 (pivot)
 *
 *  HOW TO CALIBRATE YOUR MOTORS (read the calibration section below)
 *  ──────────────────────────────────────────────────────────────────────────
 *  Three constants control straight-line accuracy. Tune them once:
 *    MAX_SPEED_PWM       — top-speed ceiling (default 240)
 *    LEFT_MOTOR_FACTOR   — scale factor for left  motor (default 1.000)
 *    RIGHT_MOTOR_FACTOR  — scale factor for right motor (default 0.862)
 *
 * ============================================================================
 */

// ── Arduino / ESP32 Core Libraries ───────────────────────────────────────────
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// =============================================================================
// ①  CONFIGURATION — Edit these constants to match your hardware
// =============================================================================

// ── Wi-Fi Access Point ────────────────────────────────────────────────────────
static const char* AP_SSID     = "VirtualJoystick_AP";  // SSID the laptop connects to
static const char* AP_PASSWORD = "  ";             // WPA2 passphrase (≥ 8 chars)
// NOTE: The ESP32 AP always assigns itself 192.168.4.1 by default.

// ── UDP ───────────────────────────────────────────────────────────────────────
static const uint16_t UDP_PORT = 4210;  // Must match ESP_PORT in virtual_joystick.py

// ── L298N Pin Definitions ─────────────────────────────────────────────────────
//   RIGHT Motor (Motor A side of L298N)
static const int PIN_ENA = 32;  // PWM duty cycle  — LEDC Channel 0
static const int PIN_IN1 = 25;  // HIGH = Forward
static const int PIN_IN2 = 26;  // HIGH = Reverse

//   LEFT Motor (Motor B side of L298N)
static const int PIN_IN3 = 27;  // HIGH = Forward
static const int PIN_IN4 = 14;  // HIGH = Reverse
static const int PIN_ENB = 33;  // PWM duty cycle  — LEDC Channel 1

// ── LEDC (PWM) Parameters ─────────────────────────────────────────────────────
static const int LEDC_FREQ_HZ   = 5000;  // 5 kHz — optimal for brushed DC motors
static const int LEDC_RES_BITS  = 8;     // 8-bit → duty range 0 – 255
static const int LEDC_CH_RIGHT  = 0;     // ENA → GPIO 32
static const int LEDC_CH_LEFT   = 1;     // ENB → GPIO 33

// ── Heartbeat Failsafe ────────────────────────────────────────────────────────
static const unsigned long HEARTBEAT_TIMEOUT_MS = 500UL;  // 500 ms of silence → E-Stop

// =============================================================================
// ②  MOTOR CALIBRATION  ← THE THREE KNOBS YOU WILL TUNE
// =============================================================================
//
//  WHY THIS EXISTS
//  ───────────────
//  No two brushed DC motors spin at exactly the same RPM under the same PWM.
//  One motor will always be slightly "stronger." Without correction, the robot
//  drifts to one side even when going straight.
//
//  HOW THE MATH WORKS
//  ──────────────────
//  Given a raw Speed value (0–100 from Python):
//
//    targetPWM = map(Speed, 0, 100, 0, MAX_SPEED_PWM)
//    leftPWM   = targetPWM × LEFT_MOTOR_FACTOR          // e.g. 127 × 1.000 = 127
//    rightPWM  = targetPWM × RIGHT_MOTOR_FACTOR         // e.g. 127 × 0.862 = 109
//
//  After steering is applied on top, the result is clamped to 0–255 and sent
//  to the LEDC channels.
//
//  TUNING PROCEDURE (step-by-step)
//  ────────────────────────────────
//   Step 1  Set MAX_SPEED_PWM = 255 and both FACTORS = 1.000.
//           Flash the firmware and drive straight using the joystick.
//
//   Step 2  Observe which way the robot drifts:
//             Drifts RIGHT → right motor is STRONGER → reduce RIGHT_MOTOR_FACTOR
//             Drifts LEFT  → left motor  is STRONGER → reduce LEFT_MOTOR_FACTOR
//
//   Step 3  Adjust the stronger motor's factor in steps of 0.05 (e.g. 0.95, 0.90…)
//           until the robot drives straight. Typical range: 0.80 – 0.95.
//
//   Step 4  Once balanced, lower MAX_SPEED_PWM if you want a lower top speed
//           (e.g. 200 for 78% max speed). Both motors scale together.
//           Use this as your main "speed governor" without re-tuning the factors.
//
//           Example scenario (default values):
//             MAX_SPEED_PWM = 240  →  top speed is ~94% of motor full-throttle
//             RIGHT motor is stronger, so RIGHT_MOTOR_FACTOR = 0.862
//             At Speed=100: L=240, R=207  →  both wheels turn at the same RPM
//
static const int   MAX_SPEED_PWM      = 230;     // ← TUNE: top PWM ceiling (0–255)
static const float LEFT_MOTOR_FACTOR  = 1.000f;  // ← TUNE: keep at 1.0 for weaker motor
static const float RIGHT_MOTOR_FACTOR = 1.000f;  // ← TUNE: reduce the stronger motor

// =============================================================================
// ③  GLOBAL STATE
// =============================================================================
WiFiUDP           udpSocket;
unsigned long     lastPacketTime = 0;  // millis() of the last valid packet
bool              failsafeActive = false;

// UDP receive buffer — 32 bytes is plenty for "S-100T-90" (10 chars max + margin)
static const int PACKET_BUFFER_SIZE = 32;
char             packetBuffer[PACKET_BUFFER_SIZE];

// =============================================================================
// ④  LOW-LEVEL PWM HELPER (LEDC wrapper)
// =============================================================================

/**
 * ledcWriteChannel()
 * ------------------
 * Thin wrapper around ledcWrite() that clamps the duty to the valid 8-bit range.
 * Using this instead of raw ledcWrite() prevents undefined behavior if the
 * steering math accidentally produces a value outside [0, 255].
 *
 * @param channel  LEDC channel (0 or 1 in this firmware)
 * @param duty     Desired PWM duty cycle (unclamped — will be clamped here)
 */
static inline void ledcWriteChannel(int channel, int duty) {
    ledcWrite(channel, (uint32_t)constrain(duty, 0, 255));
}

// =============================================================================
// ⑤  MOTOR CONTROL PRIMITIVES
// =============================================================================

/**
 * emergencyStop()
 * ---------------
 * Hard-stops both motors immediately.
 * Order matters: zero PWM BEFORE changing direction pins to avoid
 * a brief shoot-through condition in the H-bridge.
 *
 * Called by:
 *   • Heartbeat watchdog (500 ms silence)
 *   • Speed == 0 command from the joystick
 *   • Startup initialisation
 */
void emergencyStop() {
    // ① Kill PWM first so the L298N output stage is de-energised
    ledcWriteChannel(LEDC_CH_RIGHT, 0);
    ledcWriteChannel(LEDC_CH_LEFT,  0);

    // ② De-assert all direction lines (no coasting, no active braking quirks)
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
}

/**
 * driveMotors()
 * -------------
 * Sets direction bits for both motors and then applies PWM duty cycles.
 * Note: direction pins are set BEFORE PWM to guarantee the H-bridge latches
 * the correct polarity before current flows.
 *
 * @param leftPWM   Left  motor duty cycle  (0 – 255, already clamped & calibrated)
 * @param rightPWM  Right motor duty cycle  (0 – 255, already clamped & calibrated)
 * @param forward   true  = IN1-HIGH/IN2-LOW  and  IN3-HIGH/IN4-LOW  (forward)
 *                  false = IN1-LOW /IN2-HIGH and  IN3-LOW /IN4-HIGH  (reverse)
 */
void driveMotors(int leftPWM, int rightPWM, bool forward) {
    // ── RIGHT MOTOR direction pins ─────────────────────────────────────────────
    if (forward) {
        digitalWrite(PIN_IN1, HIGH);   // Forward polarity
        digitalWrite(PIN_IN2, LOW);
    } else {
        digitalWrite(PIN_IN1, LOW);    // Reverse polarity
        digitalWrite(PIN_IN2, HIGH);
    }

    // ── LEFT MOTOR direction pins ──────────────────────────────────────────────
    if (forward) {
        digitalWrite(PIN_IN3, HIGH);   // Forward polarity
        digitalWrite(PIN_IN4, LOW);
    } else {
        digitalWrite(PIN_IN3, LOW);    // Reverse polarity
        digitalWrite(PIN_IN4, HIGH);
    }

    // ── Apply PWM (direction is already latched above) ─────────────────────────
    ledcWriteChannel(LEDC_CH_RIGHT, rightPWM);
    ledcWriteChannel(LEDC_CH_LEFT,  leftPWM);
}

// =============================================================================
// ⑥  DIFFERENTIAL STEERING ENGINE
// =============================================================================

/**
 * applyDifferentialSteering()
 * ---------------------------
 * The core control law. Converts a (speed, tilt) command into distinct left
 * and right motor PWM values and drives the motors.
 *
 * ALGORITHM (5 steps)
 * ────────────────────
 *  Step 1 — Extract direction from the SIGN of speed.
 *  Step 2 — Map abs(speed) [0–100] → targetPWM [0–MAX_SPEED_PWM].
 *  Step 3 — Apply per-motor balance factors to correct hardware mismatch.
 *  Step 4 — Proportionally reduce the INSIDE motor based on tilt magnitude.
 *             steerFactor = abs(tilt) / 90.0  →  0.0 (straight) .. 1.0 (pivot)
 *             Tilt > 0 (right turn) → reduce rightPWM by steerFactor
 *             Tilt < 0 (left  turn) → reduce leftPWM  by steerFactor
 *  Step 5 — Clamp both PWM values to [0, 255] and send to driveMotors().
 *
 * STEERING VISUALISED (Speed=80, Tilt=+45 right):
 *  ┌─────────────────┬──────────────────────────────────────────────┐
 *  │  targetPWM      │  map(80, 0,100, 0,240) = 192                │
 *  │  leftPWM (raw)  │  192 × 1.000 = 192                          │
 *  │  rightPWM (raw) │  192 × 0.862 = 165                          │
 *  │  steerFactor    │  45/90 = 0.50                                │
 *  │  rightPWM (adj) │  165 × (1.0 - 0.50) = 82  ← inside motor   │
 *  │  leftPWM (adj)  │  192                       ← outside motor  │
 *  └─────────────────┴──────────────────────────────────────────────┘
 *
 * @param speed  Signed speed %: -100 (full reverse) .. +100 (full forward)
 * @param tilt   Signed tilt  °: -90 (full left)     .. +90  (full right)
 */
void applyDifferentialSteering(int speed, int tilt) {

    // ── Step 1: Direction ──────────────────────────────────────────────────────
    bool isForward = (speed >= 0);
    int  absSpeed  = abs(speed);  // Magnitude only: 0 – 100

    // ── Early exit on zero speed ───────────────────────────────────────────────
    if (absSpeed == 0) {
        emergencyStop();
        Serial.println("[CTRL] Speed = 0 → Motor STOP");
        return;
    }

    // ── Step 2: Map percentage → PWM ─────────────────────────────────────────
    //   Speed 0   → PWM  0
    //   Speed 100 → PWM  MAX_SPEED_PWM
    int targetPWM = map(absSpeed, 0, 100, 0, MAX_SPEED_PWM);

    // ── Step 3: Motor balance calibration ─────────────────────────────────────
    //   Each motor gets its individual scaling factor.
    //   The faster/stronger motor has a factor < 1.0 to slow it down to match.
    int leftPWM  = (int)((float)targetPWM * LEFT_MOTOR_FACTOR);
    int rightPWM = (int)((float)targetPWM * RIGHT_MOTOR_FACTOR);

    // ── Step 4: Proportional steering reduction ────────────────────────────────
    //
    //   The "inside" motor (the one on the turn side) is slowed down.
    //   steerFactor = 0 means no reduction (straight ahead).
    //   steerFactor = 1 means full reduction (inside motor stops → pivot turn).
    //
    if (tilt != 0) {
        float steerFactor = (float)abs(tilt) / 90.0f;  // 0.0 – 1.0

        if (tilt > 0) {
            // ── RIGHT turn: right motor is the INSIDE motor ────────────────────
            rightPWM = (int)((float)rightPWM * (1.0f - steerFactor));
        } else {
            // ── LEFT turn: left motor is the INSIDE motor ──────────────────────
            leftPWM  = (int)((float)leftPWM  * (1.0f - steerFactor));
        }
    }

    // ── Step 5: Clamp and drive ────────────────────────────────────────────────
    leftPWM  = constrain(leftPWM,  0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    Serial.printf("[CTRL] %s | Spd:%3d%% | Tlt:%4d° | L_PWM:%3d | R_PWM:%3d\n",
                  isForward ? "FWD" : "REV", absSpeed, tilt, leftPWM, rightPWM);

    driveMotors(leftPWM, rightPWM, isForward);
}

// =============================================================================
// ⑦  UDP PAYLOAD PARSER
// =============================================================================

/**
 * parsePayload()
 * --------------
 * Parses a null-terminated C-string in the format "S[Speed]T[Tilt]" into
 * separate integer Speed and Tilt values.
 *
 * DESIGN DECISIONS
 * ─────────────────
 *  • No hardcoded string indices. The split point is found by searching for
 *    the 'T' character, so variable-length payloads are handled correctly:
 *      "S5T0"        →  Speed=5,    Tilt=0
 *      "S100T90"     →  Speed=100,  Tilt=90
 *      "S-100T-45"   →  Speed=-100, Tilt=-45   (future reverse gear)
 *      "S0T0"        →  Speed=0,    Tilt=0     (stop)
 *
 *  • Arduino String::toInt() correctly handles leading '-' signs.
 *
 *  • A malformed packet (missing 'S', missing 'T', empty fields) returns
 *    false without modifying outSpeed / outTilt — the caller discards it
 *    WITHOUT resetting the heartbeat timer, so a corrupt packet can't mask
 *    a genuine connection loss.
 *
 *  • Both outputs are clamped to their valid ranges after parsing so the
 *    motor logic never sees out-of-range values, even if the sender misbehaves.
 *
 * @param payload   Null-terminated received string (e.g. "S85T-12")
 * @param outSpeed  [OUT] Parsed speed: clamped to -100 .. +100
 * @param outTilt   [OUT] Parsed tilt:  clamped to  -90 ..  +90
 * @return          true on success, false on any parse error
 */
bool parsePayload(const char* payload, int& outSpeed, int& outTilt) {
    String msg = String(payload);
    msg.trim();  // Strip any trailing '\n', '\r', or spaces from the UDP frame

    // ── Validate 'S' prefix ───────────────────────────────────────────────────
    if (msg.length() < 4 || msg.charAt(0) != 'S') {
        Serial.printf("[PARSE] Rejected — no 'S' prefix: \"%s\"\n", payload);
        return false;
    }

    // ── Locate the 'T' delimiter ──────────────────────────────────────────────
    int tIdx = msg.indexOf('T');
    if (tIdx <= 1) {
        // tIdx == -1 → 'T' not found at all
        // tIdx ==  1 → nothing between 'S' and 'T' (empty speed field)
        Serial.printf("[PARSE] Rejected — missing/misplaced 'T': \"%s\"\n", payload);
        return false;
    }

    // ── Split at 'T' ──────────────────────────────────────────────────────────
    //   speedStr = everything from index 1 up to (but not including) tIdx
    //   tiltStr  = everything after tIdx
    String speedStr = msg.substring(1, tIdx);        // e.g. "85", "-100", "0"
    String tiltStr  = msg.substring(tIdx + 1);       // e.g. "-12", "90",  "0"

    if (speedStr.length() == 0 || tiltStr.length() == 0) {
        Serial.printf("[PARSE] Rejected — empty field: \"%s\"\n", payload);
        return false;
    }

    // ── Convert and clamp ─────────────────────────────────────────────────────
    outSpeed = constrain(speedStr.toInt(), -100, 100);
    outTilt  = constrain(tiltStr.toInt(),   -90,  90);
    return true;
}

// =============================================================================
// ⑧  SETUP
// =============================================================================
void setup() {
    // ── Serial Monitor ────────────────────────────────────────────────────────
    Serial.begin(115200);
    delay(500);  // Allow the USB-serial bridge to enumerate before printing

    Serial.println(F("\n\n"));
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║    VirtualJoystick ESP32 Firmware — Starting Up      ║"));
    Serial.println(F("║    Hand Gesture Robot Car Controller                 ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println();

    // ── GPIO Direction Setup ──────────────────────────────────────────────────
    Serial.println(F("[INIT] Configuring GPIO pins as OUTPUTs..."));
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    // ENA and ENB are LEDC-driven but must also be declared as outputs
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    Serial.println(F("[INIT] GPIO OK"));

    // ── Safe Power-On State ───────────────────────────────────────────────────
    // Do this BEFORE attaching LEDC so the very first LEDC write can only be 0
    emergencyStop();
    Serial.println(F("[INIT] Motors initialised to STOP (safe power-on state)"));

    // ── LEDC PWM Channel Setup ────────────────────────────────────────────────
    Serial.println(F("[INIT] Configuring LEDC PWM channels..."));

    // ledcSetup(channel, frequency_Hz, resolution_bits)
    ledcSetup(LEDC_CH_RIGHT, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcSetup(LEDC_CH_LEFT,  LEDC_FREQ_HZ, LEDC_RES_BITS);

    // ledcAttachPin(gpio, channel) — binds the GPIO to the LEDC channel
    ledcAttachPin(PIN_ENA, LEDC_CH_RIGHT);
    ledcAttachPin(PIN_ENB, LEDC_CH_LEFT);

    Serial.printf("[INIT] LEDC → Freq: %d Hz | Resolution: %d-bit (0–%d)\n",
                  LEDC_FREQ_HZ, LEDC_RES_BITS, (1 << LEDC_RES_BITS) - 1);
    Serial.printf("[INIT]        CH%d → GPIO%d (ENA/Right)  |  CH%d → GPIO%d (ENB/Left)\n",
                  LEDC_CH_RIGHT, PIN_ENA, LEDC_CH_LEFT, PIN_ENB);

    // ── Wi-Fi Access Point ────────────────────────────────────────────────────
    Serial.println(F("[WIFI] Starting Access Point..."));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    delay(200);  // Necessary: AP stack needs ~100ms to assign the 192.168.4.1 IP

    IPAddress apIP = WiFi.softAPIP();
    Serial.println(F("[WIFI] ─────────────────────────────────────────────"));
    Serial.printf( "[WIFI] SSID     : %s\n",              AP_SSID);
    Serial.printf( "[WIFI] Password : %s\n",              AP_PASSWORD);
    Serial.printf( "[WIFI] IP Addr  : %s\n",              apIP.toString().c_str());
    Serial.printf( "[WIFI] Channel  : %d\n",              WiFi.channel());
    Serial.println(F("[WIFI] ─────────────────────────────────────────────"));

    // ── UDP Socket ────────────────────────────────────────────────────────────
    Serial.printf("[UDP]  Binding socket to port %d...\n", UDP_PORT);
    udpSocket.begin(UDP_PORT);
    Serial.printf("[UDP]  Listening on %s:%d\n", apIP.toString().c_str(), UDP_PORT);

    // ── Calibration Summary ───────────────────────────────────────────────────
    Serial.println(F("[CAL]  ─────────────────────────────────────────────"));
    Serial.printf( "[CAL]  MAX_SPEED_PWM       = %d  (%.1f%% of 255)\n",
                   MAX_SPEED_PWM, (MAX_SPEED_PWM / 255.0f) * 100.0f);
    Serial.printf( "[CAL]  LEFT_MOTOR_FACTOR   = %.3f\n", LEFT_MOTOR_FACTOR);
    Serial.printf( "[CAL]  RIGHT_MOTOR_FACTOR  = %.3f\n", RIGHT_MOTOR_FACTOR);
    Serial.printf( "[CAL]  HEARTBEAT_TIMEOUT   = %lu ms\n", HEARTBEAT_TIMEOUT_MS);
    Serial.println(F("[CAL]  ─────────────────────────────────────────────"));

    // ── Heartbeat Primer ──────────────────────────────────────────────────────
    // Initialise lastPacketTime to now so the watchdog doesn't fire immediately
    // before the laptop has had a chance to connect and send the first packet.
    lastPacketTime = millis();

    Serial.println(F("\n[INIT] Boot complete. Waiting for controller to connect...\n"));
}

// =============================================================================
// ⑨  MAIN LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();

    // =========================================================================
    // BLOCK A — HEARTBEAT FAILSAFE WATCHDOG
    // =========================================================================
    //
    //  The Python script sends packets at 20 Hz (every 50 ms).
    //  If HEARTBEAT_TIMEOUT_MS (500 ms = 10 missed packets) passes with no valid
    //  packet, we force an Emergency Stop.
    //
    //  Reasons this can trigger:
    //    • Laptop disconnected from the AP Wi-Fi
    //    • Python script crashed or was closed by the user
    //    • The user is intentionally stopping (no hand detected, speed=0 sent)
    //    • Physical Wi-Fi interference or range
    //
    if ((now - lastPacketTime) > HEARTBEAT_TIMEOUT_MS) {
        if (!failsafeActive) {
            failsafeActive = true;
            emergencyStop();
            Serial.println(F("\n[FAILSAFE] ⚠  >500 ms without a valid packet — EMERGENCY STOP"));
            Serial.println(F("[FAILSAFE]    Robot halted. Waiting for controller to reconnect..."));
        }
        // While failsafe is active we still poll the socket (below) so we can
        // detect and recover from reconnection without a reboot.
        // But we do NOT call applyDifferentialSteering() until a fresh packet arrives.
    }

    // =========================================================================
    // BLOCK B — UDP RECEIVE & PROCESS
    // =========================================================================
    int packetSize = udpSocket.parsePacket();

    if (packetSize > 0) {
        // ── Read raw bytes ────────────────────────────────────────────────────
        int bytesRead = udpSocket.read(packetBuffer, PACKET_BUFFER_SIZE - 1);
        if (bytesRead <= 0) {
            // parsePacket() said data is available but read() got nothing — skip
            return;
        }
        packetBuffer[bytesRead] = '\0';  // Null-terminate → valid C-string

        // ── Parse ─────────────────────────────────────────────────────────────
        int speed = 0, tilt = 0;
        if (!parsePayload(packetBuffer, speed, tilt)) {
            // Malformed packet: log, discard, and importantly do NOT update
            // lastPacketTime. A corrupt packet must not mask a real timeout.
            return;
        }

        // ── Recovery from failsafe ────────────────────────────────────────────
        if (failsafeActive) {
            failsafeActive = false;
            Serial.println(F("[FAILSAFE] ✓  Valid packet received — resuming normal operation.\n"));
        }

        // ── Update heartbeat clock ────────────────────────────────────────────
        lastPacketTime = now;

        // ── Execute motor command ─────────────────────────────────────────────
        applyDifferentialSteering(speed, tilt);
    }

    // No yield() or delay() needed: parsePacket() is non-blocking and the
    // loop runs at ESP32 native speed (~240 MHz), keeping latency < 1 ms.
}

/*
 * ============================================================================
 *  END OF VirtualJoystick_ESP32.ino
 * ============================================================================
 */
