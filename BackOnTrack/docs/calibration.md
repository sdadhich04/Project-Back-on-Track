# Calibration & Tuning Guide — Back on Track

## Calibration Overview

Back on Track uses an automated calibration sequence that runs every time the device powers on. Calibration captures:

1. **Neutral IMU orientation** — BNO085 pitch/roll/yaw while you sit in your ideal posture
2. **Resting EMG baseline** — MyoWare average ADC value with muscles relaxed

All posture deviations are computed relative to this baseline, so the system adapts to your body shape, chair height, and BNO085 mounting position.

---

## Calibration Procedure

1. Power on the device and open the Serial monitor (115200 baud)
2. Wait for the banner message: `Back on Track — Starting`
3. BNO085 and MyoWare init messages will print (check for errors)
4. The console will say: **"Starting calibration in 3 seconds..."**
5. **Sit in your ideal neutral seated posture:**
   - Back straight, not touching the chair back (or lightly touching)
   - Head level, ears over shoulders
   - Feet flat on the floor
   - Arms relaxed at your sides or on armrests
6. **Relax your muscles** — especially the upper trapezius
7. The calibration samples for **5 seconds**
8. On completion, the console prints the captured values:
   ```
   === CALIBRATION COMPLETE ===
     Neutral pitch : -2.13°
     Neutral roll  : 0.45°
     Neutral yaw   : -15.20°
     EMG baseline  : 1843.7 ADC counts
   ```

---

## BNO085 Calibration Notes

- The Game Rotation Vector report does **not** require magnetometer calibration.
- The BNO085 internally runs a sensor fusion algorithm that calibrates its gyro and accelerometer automatically over time.
- Pitch and roll are stable from power-on. Yaw may drift very slowly (~1–2°/hour) since there is no magnetometer correction. For posture, yaw drift is usually acceptable.
- If you see very noisy pitch/roll values during calibration, the BNO085 may still be warming up. Try extending the 3-second pre-delay in `app_main()`.

---

## MyoWare Calibration Notes

- During calibration, keep muscles **completely relaxed**. Even subtle forearm or shoulder tension will inflate the baseline.
- If your resting baseline is above ~3000 ADC counts, the electrode contact may be poor:
  - Clean skin with an alcohol wipe
  - Press electrodes firmly for 5 seconds to improve contact
  - Check that the snap electrode connectors are fully seated
- If the baseline is near 0 or saturated at 4095, check wiring (VCC, GND, SIG connections).

---

## Threshold Tuning

All classification thresholds are defined in `data_types.h`. After calibration, if the classifier is too sensitive or not sensitive enough, adjust these:

```c
// ---- Angle thresholds (in degrees, relative to neutral) ----

// Pitch: how far forward before classified as slouching
#define POSTURE_PITCH_SLOUCH_DEG      15.0f   // Try 10–20°

// Pitch: how far forward before classified as leaning forward
#define POSTURE_PITCH_LEAN_FWD_DEG    30.0f   // Try 25–40°

// Roll: left/right lean thresholds (negative = left, positive = right)
#define POSTURE_ROLL_LEAN_LEFT_DEG   -10.0f   // Try -8° to -15°
#define POSTURE_ROLL_LEAN_RIGHT_DEG   10.0f   // Try 8° to 15°

// ---- EMG threshold (in ADC counts above baseline) ----
#define POSTURE_EMG_HIGH_THRESHOLD    80.0f   // Try 50–150

// ---- Confirmation window ----
// How long a bad posture must persist before triggering a correction event
// Increase to reduce false positives from transient movements (e.g. reaching for a cup)
#define POSTURE_CONFIRM_WINDOW_MS     750     // Try 500–2000 ms
```

### Suggested Tuning Workflow

1. Run the device and generate CSV telemetry
2. Copy the CSV to a spreadsheet or Python/MATLAB script
3. Label segments manually ("this was slouching", "this was good")
4. Find the pitch_deviation and roll_deviation values that separate good from bad
5. Set thresholds 2–3° inside the observed boundary to avoid edge-case misclassifications
6. Re-flash and validate

---

## Multi-Sensor Timing Check

To verify that both sensors are running at their expected rates, check the telemetry timestamps:
- IMU samples should arrive at ~20ms intervals (50 Hz)
- EMG samples at ~10ms intervals (100 Hz)
- Telemetry output at ~100ms intervals (10 Hz)

If timestamps are irregular, check that FreeRTOS tick rate is 1000 Hz (set in `sdkconfig.defaults`).

---

## Debugging Noisy Sensor Readings

**BNO085 noise:**
- Reduce SPI speed from 3 MHz to 1 MHz in `driver_bno085.c` (`clock_speed_hz`)
- Add a small delay between samples
- Check for ground loops (separate power rail grounds if using breadboard)
- Ensure SPI wires are short (<20cm)

**MyoWare noise:**
- High 50/60 Hz hum = poor electrode contact or electrode too close to mains-powered devices
- High broadband noise = lead wires acting as antennas; shorten them
- Offset drifting = skin/electrode interface moisture change; re-prep skin
- Try adjusting `EMG_EMA_ALPHA` in `driver_myoware.c` to `0.05f` for more smoothing

---

## BNO085 Orientation Axis Mapping

The BNO085 mounted flat (X forward, Y right, Z up) gives:
- Pitch positive = lean forward (nose down)
- Pitch negative = lean back
- Roll positive = lean right
- Roll negative = lean left
- Yaw = clockwise rotation from above

If your BNO085 is mounted in a different orientation (rotated on the wearable), the pitch/roll mapping will differ. In that case, adjust the axis swap in `parse_game_rotation_vector()` in `driver_bno085.c`.
