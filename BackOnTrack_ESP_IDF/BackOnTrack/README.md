# Back on Track — ESP32-S3 Posture Corrector

**A wearable posture-awareness prototype using BNO085 IMU + MyoWare 2.0 EMG + ESP32-S3.**

> ⚠️ **DISCLAIMER:** Back on Track is a posture-awareness and habit-building prototype. It is **not a medical device**. It is not intended to diagnose, treat, cure, or prevent any medical condition. Do not use it in clinical or safety-critical applications.

---

## Description

Back on Track uses an Adafruit BNO085 9-DOF IMU over SPI and a MyoWare 2.0 Muscle Sensor over ADC to classify seated posture in real time. A rule-based classifier detects slouching, forward leaning, lateral leaning, and high muscle tension, then outputs telemetry over Serial. A basic gamification system tracks good-posture streaks and awards points.

---

## Hardware

| Component | Part |
|-----------|------|
| MCU | ESP32-S3 development board |
| IMU | Adafruit BNO085 9-DOF IMU Fusion Breakout |
| EMG | SparkFun MyoWare 2.0 Muscle Sensor |
| Power | USB (5V) via development board |

---

## Software Requirements

- **ESP-IDF v5.x** (tested on v5.1+)
- **VS Code** with the **Espressif IDF Extension** installed
- Python 3 (for IDF tools)
- Git

---

## ESP-IDF Setup in VS Code

1. Install VS Code from https://code.visualstudio.com
2. Install the **ESP-IDF** extension (Espressif Systems)
3. Run **"ESP-IDF: Configure ESP-IDF Extension"** from the VS Code command palette
4. Select ESP-IDF v5.x when prompted
5. Clone this repository or unzip the project folder
6. Open the `BackOnTrack/` folder in VS Code (`File > Open Folder`)
7. In the command palette: **"ESP-IDF: Set Espressif Device Target"** → select `esp32s3`
8. Or from the terminal:
   ```bash
   idf.py set-target esp32s3
   ```

---

## Build / Flash / Monitor

```bash
# Set target (only needed once)
idf.py set-target esp32s3

# Build
idf.py build

# Flash (replace /dev/ttyUSB0 with your COM port)
idf.py -p /dev/ttyUSB0 flash

# Flash and monitor simultaneously
idf.py -p /dev/ttyUSB0 flash monitor

# Monitor only (if already flashed)
idf.py -p /dev/ttyUSB0 monitor
```

On Windows, the port will be `COM3`, `COM4`, etc. On macOS it may be `/dev/cu.usbserial-...`.

---

## Folder Structure

```
BackOnTrack/
├── CMakeLists.txt              # Root build config
├── sdkconfig.defaults          # Default Kconfig settings
├── README.md
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp                # App entry point, FreeRTOS tasks, sensor init
├── components/
│   ├── sensor_hal/             # Hardware abstraction layer (SensorContext_t)
│   │   └── include/sensor_hal.h
│   ├── data_types/             # Shared structs (IMU sample, EMG sample, etc.)
│   │   ├── include/data_types.h
│   │   └── data_types.c
│   ├── drivers/                # Low-level sensor drivers
│   │   ├── include/driver_bno085.h
│   │   ├── include/driver_myoware.h
│   │   ├── driver_bno085.c     # BNO085 SPI + SHTP + quaternion conversion
│   │   └── driver_myoware.c    # MyoWare ADC oneshot + EMA filter
│   ├── posture/                # Feature extraction and classifier
│   │   ├── include/posture_features.h
│   │   ├── include/posture_classifier.h
│   │   ├── posture_features.c
│   │   └── posture_classifier.c
│   └── telemetry/              # Serial telemetry (CSV or JSON)
│       ├── include/telemetry_serial.h
│       └── telemetry_serial.c
└── docs/
    ├── wiring.md               # Hardware wiring tables
    ├── calibration.md          # Calibration and threshold tuning guide
    ├── testing.md              # Test checklist
    └── ai_log.md               # AI-assisted development log
```

---

## BNO085 SPI Wiring

See `docs/wiring.md` for full details.

| BNO085 Pin | ESP32-S3 GPIO | Notes |
|------------|---------------|-------|
| VIN | 3.3V | |
| GND | GND | |
| CS | GPIO37 | Active low |
| SCL/SCLK | GPIO38 | SPI clock |
| DI/MOSI | GPIO40 | |
| SDA/MISO | GPIO39 | |
| INT | GPIO5 | **Must have 10K pull-up to 3.3V** |
| RST | GPIO6 | |
| PS0 | 3.3V | **Tie HIGH for SPI mode** |
| PS1 | 3.3V | **Tie HIGH for SPI mode** |

---

## MyoWare 2.0 Wiring

| MyoWare Pin | ESP32-S3 | Notes |
|-------------|----------|-------|
| SIG | GPIO4 (ADC1_CH3) | Signal output; max 3.3V |
| VCC | 3.3V | Power from 3.3V **only** if SIG max is 3.3V |
| GND | GND | Common ground |

> ⚠️ **Safety:** If powering MyoWare from 5V, its SIG output can exceed 3.3V and damage the ESP32-S3 ADC. Either power from 3.3V or use a voltage divider.

---

## Calibration Process

See `docs/calibration.md` for full details.

On every power-up, the firmware automatically runs a 5-second calibration sequence. During calibration:
1. Sit in your **ideal neutral seated posture**
2. Relax your muscles (arms resting naturally)
3. Wait for the calibration complete message on Serial

The firmware captures neutral pitch/roll/yaw from the BNO085 and resting EMG baseline from MyoWare. All classification deviations are relative to this baseline.

---

## Telemetry Output

Connect to the ESP32-S3 UART0 at **115200 baud**. Output format is selected in `telemetry_serial.h`:
- `TELEMETRY_OUTPUT_MODE 0` → CSV (compatible with Arduino Serial Plotter)
- `TELEMETRY_OUTPUT_MODE 1` → JSON (human readable)

---

## Changing Thresholds

All classification thresholds are in `components/data_types/include/data_types.h`:

```c
#define POSTURE_PITCH_SLOUCH_DEG      15.0f
#define POSTURE_PITCH_LEAN_FWD_DEG    30.0f
#define POSTURE_ROLL_LEAN_LEFT_DEG   -10.0f
#define POSTURE_ROLL_LEAN_RIGHT_DEG   10.0f
#define POSTURE_EMG_HIGH_THRESHOLD    80.0f
#define POSTURE_CONFIRM_WINDOW_MS     750
```

---

## Known Limitations

- Rule-based classifier: no machine learning. Classification rules are simple thresholds — not trained on real posture data.
- Yaw drift: Game Rotation Vector has no magnetometer correction, so yaw drifts slowly over time (~1–2°/hour). Pitch and roll are stable.
- No persistent calibration: calibration runs on every boot. Future work could save to NVS.
- No BLE yet: telemetry is serial only in this version. BLE is a TODO.
- No feedback actuator: no buzzer or vibration motor yet — feedback is serial only.

---

## Future Improvements

- [ ] Save calibration to NVS (survives reboot)
- [ ] BLE telemetry (NimBLE peripheral, phone app)
- [ ] Buzzer or vibration feedback on correction events
- [ ] TinyML classifier (replace rule-based with Edge Impulse model)
- [ ] OLED display for on-device status
- [ ] Battery + power management (deep sleep between samples)
- [ ] Data logging to SD card or BLE app for training data collection
