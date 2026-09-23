# VirtualJoystick ESP32 Firmware

**Hand Gesture Robot Car — ESP32 Controller Firmware**
A passion project by Nomun

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Requirements](#3-hardware-requirements)
4. [Wiring Diagram](#4-wiring-diagram)
5. [Arduino IDE Setup](#5-arduino-ide-setup)
6. [Firmware Configuration](#6-firmware-configuration)
7. [Motor Calibration Guide](#7-motor-calibration-guide)
8. [UDP Payload Protocol](#8-udp-payload-protocol)
9. [Steering Logic Explained](#9-steering-logic-explained)
10. [Heartbeat Failsafe](#10-heartbeat-failsafe)
11. [Serial Monitor Debug Output](#11-serial-monitor-debug-output)
12. [Connecting the Laptop](#12-connecting-the-laptop)
13. [Troubleshooting](#13-troubleshooting)
14. [Future Improvements](#14-future-improvements)

---

## 1. Project Overview

This firmware turns an ESP32 Dev Board into the brain of a hand-gesture-controlled robot car. It works in tandem with the laptop-side Python script (`virtual_joystick.py`) which uses a webcam and Google MediaPipe to track your hand and turn gestures into driving commands.

```
┌─────────────────────────────────┐         Wi-Fi (AP Mode)        ┌───────────────────────────┐
│         LAPTOP                  │ ──── UDP packets at 20 Hz ───► │        ESP32              │
│  virtual_joystick.py            │        "S85T-12"               │  VirtualJoystick_ESP32    │
│  Webcam → MediaPipe → Gestures  │        "S0T0"                  │  Parses → PWM → L298N     │
└─────────────────────────────────┘                                └───────────┬───────────────┘
                                                                               │
                                                                    ┌──────────┴──────────┐
                                                                    │       L298N          │
                                                                    │  Left Motor │ Right  │
                                                                    └─────────────────────┘
```

### Supported Hand Gestures

| Gesture | Robot Action | How It Works |
|---|---|---|
| Pinch open + Hand UP | Move Forward | Wrist below fingers; thumb-index distance controls speed 0–100% |
| Pinch open + Hand DOWN | Move Backward | Wrist above fingers *(future firmware already handles this)* |
| Hand Tilt Left / Right | Steer Left / Right | Wrist-to-MiddleBase angle → tilt −90° to +90° |
| Closed Fist | Emergency Stop | All fingertips curled close to palm → speed forced to 0 |

---

## 2. System Architecture

```
                        ESP32 Firmware Architecture
   ┌─────────────────────────────────────────────────────────────┐
   │  setup()                                                    │
   │    ├── GPIO: Set IN1–IN4, ENA, ENB as OUTPUT               │
   │    ├── emergencyStop() → safe initial state                 │
   │    ├── LEDC: ch0 (ENA/GPIO32) + ch1 (ENB/GPIO33) @ 5kHz   │
   │    ├── WiFi.softAP("VirtualJoystick_AP")  → 192.168.4.1    │
   │    └── udpSocket.begin(4210)                                │
   │                                                             │
   │  loop()  [~240 MHz, non-blocking]                          │
   │    ├── BLOCK A: Heartbeat Watchdog                          │
   │    │     └── now − lastPacketTime > 500 ms → emergencyStop  │
   │    │                                                        │
   │    └── BLOCK B: UDP Receive                                 │
   │          ├── udpSocket.parsePacket()                        │
   │          ├── parsePayload()  "S85T-12" → speed=85, tilt=-12│
   │          ├── lastPacketTime = millis()                      │
   │          └── applyDifferentialSteering(speed, tilt)         │
   │                ├── direction = sign(speed)                  │
   │                ├── targetPWM = map(|speed|, 0,100, 0,MAX)  │
   │                ├── leftPWM  = targetPWM × LEFT_FACTOR       │
   │                ├── rightPWM = targetPWM × RIGHT_FACTOR      │
   │                ├── steerFactor = |tilt| / 90.0              │
   │                ├── (tilt>0) → reduce rightPWM               │
   │                ├── (tilt<0) → reduce leftPWM                │
   │                └── driveMotors(L, R, fwd)                   │
   └─────────────────────────────────────────────────────────────┘
```

---

## 3. Hardware Requirements

| Component | Specification | Notes |
|---|---|---|
| Microcontroller | ESP-WROOM-32 (ESP32 Dev Board) | Any 38-pin dev board works |
| Motor Driver | L298N Dual H-Bridge Module | 5–35V motor supply |
| DC Motors | 3V–12V brushed DC (×2) | Standard TT or N20 gear motors |
| Power Supply | 7.4V 2S LiPo or 6× AA batteries | Powers L298N + motors |
| 5V Regulator | Built-in on most L298N modules | Powers ESP32 from same battery |
| Chassis | Any 2WD robot car chassis | |
| USB Cable | Micro-USB or USB-C | For flashing and Serial Monitor |

> **Power note:** The L298N's onboard 5V regulator (if present) can supply the ESP32's 5V pin. Do **not** power the ESP32 3.3V pin directly from the battery — this bypasses the onboard regulator and will damage the board.

---

## 4. Wiring Diagram

```
                    ┌──────────────────────────────────────────────────────────┐
BATTERY (+) ──────► │ +12V / VS    L298N                                       │
BATTERY (−) ──────► │ GND                                                      │
                    │                                                          │
                    │  ENA ◄──── GPIO 32 (ESP32)  [Right Motor PWM]           │
                    │  IN1 ◄──── GPIO 25 (ESP32)  [Right Motor FWD]           │
                    │  IN2 ◄──── GPIO 26 (ESP32)  [Right Motor REV]           │
                    │                                                          │
                    │  IN3 ◄──── GPIO 27 (ESP32)  [Left Motor FWD]            │
                    │  IN4 ◄──── GPIO 14 (ESP32)  [Left Motor REV]            │
                    │  ENB ◄──── GPIO 33 (ESP32)  [Left Motor PWM]            │
                    │                                                          │
                    │  OUT1 ──► Right Motor (+)                                │
                    │  OUT2 ──► Right Motor (−)                                │
                    │  OUT3 ──► Left  Motor (+)                                │
                    │  OUT4 ──► Left  Motor (−)                                │
                    │                                                          │
                    │  +5V  ──► ESP32 VIN (5V pin)   [Power from L298N reg.]  │
                    │  GND  ──► ESP32 GND             [Common ground]         │
                    └──────────────────────────────────────────────────────────┘

IMPORTANT — Common Ground:
  ESP32 GND and L298N GND and Battery (−) must ALL be connected together.
  Forgetting this is the single most common wiring mistake.
```

### Safe PWM GPIO Notes

The GPIOs used here are deliberately chosen to avoid boot-strapping conflicts:

| GPIO | Role | Why Safe |
|---|---|---|
| 32 | ENA (Right PWM) | Input-only restricted GPIOs avoided; safe for output |
| 25 | IN1 | DAC-capable but safe as digital output |
| 26 | IN2 | DAC-capable but safe as digital output |
| 27 | IN3 | General purpose, no boot conflict |
| 14 | IN4 | No pull-up issues at boot |
| 33 | ENB (Left PWM) | Input-only restricted GPIOs avoided; safe for output |

> **Do not use** GPIO 0, 2, 5, 12, 15 for motor outputs — these affect the boot mode and can prevent the ESP32 from starting correctly.

---

## 5. Arduino IDE Setup

### Step 1 — Install Arduino IDE 2.x

Download from [arduino.cc/en/software](https://www.arduino.cc/en/software).

### Step 2 — Add ESP32 Board Package

1. Open Arduino IDE → **File → Preferences**
2. In "Additional boards manager URLs" paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to **Tools → Board → Boards Manager**
4. Search **esp32** → Install **"esp32 by Espressif Systems"** (v2.x recommended)

### Step 3 — Select Your Board

- **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
- **Tools → Port** → Select the COM port your ESP32 is on (e.g. `COM5` on Windows, `/dev/ttyUSB0` on Linux)

### Step 4 — Recommended Upload Settings

| Setting | Value |
|---|---|
| Upload Speed | 921600 |
| CPU Frequency | 240 MHz |
| Flash Frequency | 80 MHz |
| Flash Mode | QIO |
| Partition Scheme | Default 4MB with spiffs |

### Step 5 — Flash the Firmware

1. Open `VirtualJoystick_ESP32.ino` in Arduino IDE.
2. Click **Upload** (the right-arrow button).
3. If upload fails, hold the **BOOT** button on the ESP32 while clicking Upload, release after "Connecting..." appears.

---

## 6. Firmware Configuration

All user-configurable parameters are at the top of the `.ino` file in clearly marked sections.

### Network (Section ①)

```cpp
static const char*    AP_SSID     = "VirtualJoystick_AP";  // Change SSID here
static const char*    AP_PASSWORD = "12345678";             // Min 8 characters
static const uint16_t UDP_PORT    = 4210;                   // Must match Python script
```

### Failsafe Timeout (Section ①)

```cpp
static const unsigned long HEARTBEAT_TIMEOUT_MS = 500UL;
```

Increase this (e.g. to `1000`) if you experience false E-Stops due to Wi-Fi latency. Decrease it (e.g. to `250`) for faster emergency response.

---

## 7. Motor Calibration Guide

This is the most important tuning step. Three constants live in **Section ② of the firmware**:

```cpp
const int   MAX_SPEED_PWM      = 240;     // Top-speed ceiling
const float LEFT_MOTOR_FACTOR  = 1.000f;  // Left  motor scale
const float RIGHT_MOTOR_FACTOR = 0.862f;  // Right motor scale
```

### What each constant does

**`MAX_SPEED_PWM`** — The absolute PWM ceiling sent to both motors.
- `255` = full 12V to the motors (fastest, hardest to control)
- `200` = ~78% power (slower, easier to control on a desk)
- `240` = default sweet spot

Think of this as your **speed governor**. Set it once and leave it; it scales both motors together without affecting the balance between them.

**`LEFT_MOTOR_FACTOR` / `RIGHT_MOTOR_FACTOR`** — Per-motor correction to compensate for hardware differences. One value is always `1.000` (the weaker motor, running at full target). The other is `< 1.000` (the stronger motor, scaled down to match).

### Calibration Procedure

```
Step 1:  Set MAX_SPEED_PWM = 200, both FACTORS = 1.000.
         Flash and drive the robot on a flat surface.

Step 2:  Command Speed=50, Tilt=0 (straight ahead).
         Observe drift:

         Robot drifts RIGHT → right motor is stronger → reduce RIGHT_MOTOR_FACTOR
         Robot drifts LEFT  → left  motor is stronger → reduce LEFT_MOTOR_FACTOR

Step 3:  Adjust the stronger motor's factor in steps of 0.05:
           1.000 → 0.950 → 0.900 → 0.862 → ...
         Re-flash and test each time.

Step 4:  Once the robot drives straight, lock in that factor.

Step 5:  Now raise MAX_SPEED_PWM to your desired top speed (e.g. 240).
         The balance factors scale with it automatically — no re-tuning needed.
```

### Example (default values)

```
User sets: MAX_SPEED_PWM = 240

Python sends: Speed=100, Tilt=0

Firmware calculates:
  targetPWM = map(100, 0,100, 0,240) = 240
  leftPWM   = 240 × 1.000 = 240
  rightPWM  = 240 × 0.862 = 206

Result: Both wheels turn at the same physical RPM → robot drives straight.
```

---

## 8. UDP Payload Protocol

### Format

```
"S[Speed]T[Tilt]"
```

| Field | Type | Range | Example |
|---|---|---|---|
| Speed | Signed integer | −100 to +100 | `85`, `-60`, `0` |
| Tilt | Signed integer | −90 to +90 | `-12`, `30`, `0` |

### Examples

| Packet | Meaning |
|---|---|
| `S85T-12` | Forward, 85% speed, 12° left turn |
| `S0T0` | Full stop |
| `S100T90` | Max speed, maximum right pivot |
| `S100T-90` | Max speed, maximum left pivot |
| `S-60T30` | **Reverse** 60% speed, 30° right turn *(future)* |
| `S-100T-45` | **Reverse** max speed, 45° left turn *(future)* |

### Parser Design

The parser uses **dynamic splitting at the `T` character** — no hardcoded index positions. This means it handles variable-length payloads correctly:

```
"S5T0"        → speedStr="5",    tiltStr="0"
"S-100T-45"   → speedStr="-100", tiltStr="-45"
"S100T90"     → speedStr="100",  tiltStr="90"
```

A malformed packet (missing `S`, missing `T`, empty fields) is **silently discarded** without updating the heartbeat timer. This prevents corrupt packets from masking a genuine connection loss.

---

## 9. Steering Logic Explained

### Straight Driving

```
Speed = 80, Tilt = 0

targetPWM = map(80, 0,100, 0,240) = 192
leftPWM   = 192 × 1.000 = 192
rightPWM  = 192 × 0.862 = 165  ← already balanced

Both motors at calibrated equal RPM → straight line.
```

### Right Turn (Tilt = +45)

```
Speed = 80, Tilt = +45

steerFactor = 45 / 90 = 0.50

leftPWM  = 192  (outside motor, unchanged)
rightPWM = 165 × (1 - 0.50) = 82   ← inside motor slowed

Left wheel spins faster → robot turns RIGHT.
```

### Pivot Turn (Tilt = +90 or −90)

```
steerFactor = 90 / 90 = 1.00

Inside motor PWM = basePWM × (1 - 1.0) = 0

Outside motor full speed, inside motor stopped → sharp pivot.
```

### Reverse Gear (Speed negative)

```
Speed = -60, Tilt = 0

isForward = false  → IN1=LOW, IN2=HIGH  (right motor reverse polarity)
                    → IN3=LOW, IN4=HIGH  (left  motor reverse polarity)

absSpeed  = 60
targetPWM = map(60, 0,100, 0,240) = 144

Same PWM math as forward, just with reversed direction pins.
```

> **Note:** The Python script currently only sends positive speed values. When you add the reverse gesture (hand pointing DOWN), the Python script will send negative speed values like `S-60T0`. The ESP32 firmware already handles this with no changes needed.

---

## 10. Heartbeat Failsafe

The Python script sends packets at **20 Hz (every 50 ms)**. The ESP32 monitors the time since the last valid packet.

```
Normal operation:
  [0ms]  Packet received → lastPacketTime = now
  [50ms] Packet received → lastPacketTime = now
  [100ms] Packet received → lastPacketTime = now
  ...

Connection lost:
  [0ms]   Last valid packet
  [50ms]  No packet
  [100ms] No packet
  ...
  [500ms] FAILSAFE TRIGGERS → emergencyStop() called
          Motors: both PWM = 0, all IN pins = LOW
          Serial: "[FAILSAFE] ⚠  >500ms without a valid packet — EMERGENCY STOP"

Connection restored:
  [550ms] Valid packet received → failsafeActive = false
          Serial: "[FAILSAFE] ✓  Connection restored — resuming normal operation."
          Motor commands resume immediately.
```

The failsafe only prints once per event (not every loop iteration) to keep the Serial Monitor readable.

---

## 11. Serial Monitor Debug Output

Open the Serial Monitor at **115200 baud** to watch the firmware operate.

### Boot Output

```
╔══════════════════════════════════════════════════════╗
║    VirtualJoystick ESP32 Firmware — Starting Up      ║
║    Hand Gesture Robot Car Controller                 ║
╚══════════════════════════════════════════════════════╝

[INIT] Configuring GPIO pins as OUTPUTs...
[INIT] GPIO OK
[INIT] Motors initialised to STOP (safe power-on state)
[INIT] Configuring LEDC PWM channels...
[INIT] LEDC → Freq: 5000 Hz | Resolution: 8-bit (0–255)
[INIT]        CH0 → GPIO32 (ENA/Right)  |  CH1 → GPIO33 (ENB/Left)
[WIFI] ─────────────────────────────────────────────
[WIFI] SSID     : VirtualJoystick_AP
[WIFI] Password : 12345678
[WIFI] IP Addr  : 192.168.4.1
[WIFI] Channel  : 1
[WIFI] ─────────────────────────────────────────────
[UDP]  Binding socket to port 4210...
[UDP]  Listening on 192.168.4.1:4210
[CAL]  ─────────────────────────────────────────────
[CAL]  MAX_SPEED_PWM       = 240  (94.1% of 255)
[CAL]  LEFT_MOTOR_FACTOR   = 1.000
[CAL]  RIGHT_MOTOR_FACTOR  = 0.862
[CAL]  HEARTBEAT_TIMEOUT   = 500 ms
[CAL]  ─────────────────────────────────────────────

[INIT] Boot complete. Waiting for controller to connect...
```

### Runtime Output (normal driving)

```
[CTRL] FWD | Spd: 80% | Tlt:  +45° | L_PWM:192 | R_PWM: 82
[CTRL] FWD | Spd: 80% | Tlt:   +0° | L_PWM:192 | R_PWM:165
[CTRL] Speed = 0 → Motor STOP
```

### Failsafe Output

```
[FAILSAFE] ⚠  >500 ms without a valid packet — EMERGENCY STOP
[FAILSAFE]    Robot halted. Waiting for controller to reconnect...
[FAILSAFE] ✓  Valid packet received — resuming normal operation.
```

---

## 12. Connecting the Laptop

1. **Flash the firmware** to the ESP32 and power it on.

2. **On the laptop**, open Wi-Fi settings and connect to:
   - Network: `VirtualJoystick_AP`
   - Password: `12345678`

3. **Verify connectivity** (optional):
   ```
   ping 192.168.4.1
   ```

4. **Run the Python script**:
   ```powershell
   cd C:\path\to\your\project
   venv\Scripts\activate
   python virtual_joystick.py
   ```

5. The Serial Monitor will start showing `[CTRL]` lines as soon as the hand is detected and packets arrive.

> **Important:** Your laptop's internet will be disconnected while connected to `VirtualJoystick_AP` since it is an isolated Access Point with no internet gateway. Disconnect from the AP when done if you need internet access.

---

## 13. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Robot doesn't move at all | LEDC not attached correctly | Verify `ledcAttachPin()` calls; check GPIO numbers |
| Robot drifts to one side | Motor mismatch | Reduce the stronger motor's FACTOR (see Section 7) |
| Failsafe triggers immediately | Laptop not connecting to AP | Confirm laptop is on `VirtualJoystick_AP` Wi-Fi |
| Jerky / oscillating movement | Too little EMA smoothing in Python | Increase `EMA_ALPHA` in `virtual_joystick.py` |
| One motor runs in reverse | Wiring polarity on that motor | Swap OUT1/OUT2 or OUT3/OUT4 wires on L298N |
| Motors always at max speed | `map()` receiving wrong range | Check `MAX_SPEED_PWM` is not 0 |
| Upload fails in Arduino IDE | Boot mode issue | Hold BOOT button during upload |
| Serial Monitor shows garbage | Wrong baud rate | Set Serial Monitor to **115200** baud |
| `S-100T-45` causes wrong turn in reverse | Intended: tilt logic is frame-relative | Future: invert tilt when reversing if desired |

---

## 14. Future Improvements

- **Reverse Gear:** The Python script needs a "hand pointing DOWN" gesture to send negative speed values. The ESP32 firmware already fully supports this — no firmware changes needed.
- **Speed Ramping:** Add acceleration/deceleration ramps in `applyDifferentialSteering()` to prevent motor current spikes and improve smoothness.
- **Two ESP32 Modules:** Replace the laptop Wi-Fi link with an ESP32-to-ESP32 radio link using ESP-NOW for lower latency and longer range.
- **Encoder Feedback:** Add wheel encoders for closed-loop speed control, making straight driving more accurate regardless of battery level.
- **OTA Updates:** Add `ArduinoOTA` so firmware can be flashed over Wi-Fi without a USB cable.

---

## File Structure

```
VirtualJoystick_ESP32/
├── VirtualJoystick_ESP32.ino   ← Main firmware (this file)
└── README.md                   ← This document

virtual_joystick.py             ← Laptop-side Python controller script
```

---

*A passion project by Nomun*
