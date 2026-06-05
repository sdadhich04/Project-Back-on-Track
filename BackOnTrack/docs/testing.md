# Testing Checklist — Back on Track

## Pre-Flash Hardware Checks

- [ ] BNO085 PS0 tied to 3.3V (SPI mode)
- [ ] BNO085 PS1 tied to 3.3V (SPI mode)
- [ ] BNO085 INT pin has 10KΩ pull-up to 3.3V
- [ ] All SPI wires shorter than 20 cm (breadboard or jumper wire)
- [ ] MyoWare VCC connected to 3.3V (not 5V, to avoid ADC overvoltage)
- [ ] MyoWare GND connected to ESP32-S3 GND
- [ ] MyoWare SIG connected to GPIO4
- [ ] No short circuits on BNO085 SPI pins (check with multimeter)

## Firmware Build Check

- [ ] `idf.py set-target esp32s3` ran without errors
- [ ] `idf.py build` completed without errors or warnings about missing components
- [ ] All component CMakeLists.txt files are correct (no missing REQUIRES)

## Serial Monitor — Startup Sequence

Power on and check Serial monitor (115200 baud) for this sequence:

- [ ] Banner message printed ("Back on Track — Starting")
- [ ] `BNO085 OK` message appears (not "BNO085 init FAILED")
- [ ] `MyoWare OK` message appears
- [ ] `=== CALIBRATION START ===` message appears
- [ ] Calibration completes with reasonable values:
  - [ ] Neutral pitch within ±30° (typically ±5° for upright sitting)
  - [ ] Neutral roll within ±15°
  - [ ] EMG baseline between 100 and 3500 ADC counts

## Telemetry Output Check

After calibration, CSV or JSON lines should start printing at ~10 Hz.

- [ ] Timestamps are increasing monotonically
- [ ] State field shows `GOOD` (0) when sitting normally
- [ ] pitch_dev and roll_dev are close to 0 in neutral position
- [ ] emg_activity is close to 0 when muscles are relaxed

## Posture Classification Tests

Perform each of these deliberately and verify the correct state appears in telemetry:

- [ ] **SLOUCHING**: Lean forward 15–30° — state should become SLOUCHING (after ~750ms)
- [ ] **LEANING_FORWARD**: Lean aggressively forward >30° — state should become LEANING_FORWARD
- [ ] **LEANING_LEFT**: Lean left — state should become LEANING_LEFT
- [ ] **LEANING_RIGHT**: Lean right — state should become LEANING_RIGHT
- [ ] **HIGH_TENSION**: Tense upper trapezius muscles — state should become HIGH_TENSION
- [ ] **Return to GOOD**: Return to neutral — state should return to GOOD

## Confirmation Window Test

- [ ] Briefly lean forward and immediately return (<750ms) — should NOT trigger SLOUCHING
  (confirms the confirmation window is working)

## Gamification Check

- [ ] `good_ms` increases while in GOOD state
- [ ] `bad_ms` increases while in any bad state
- [ ] `corrections` increments on each confirmed bad posture event
- [ ] `points` increases over time in GOOD state (~1 per second)
- [ ] `streak_ms` resets to 0 after a correction event

## Stress Test

- [ ] Leave running for 5 minutes — no crash, no freezing, telemetry continues
- [ ] No "queue full — sample dropped" warnings appearing frequently
  (occasional drops are OK; frequent drops = tasks misconfigured)

## Known Failure Modes

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| `BNO085 init FAILED` | PS0/PS1 not tied HIGH | Check wiring |
| All-zeros from BNO085 | Wrong SPI mode (I2C/UART) | Tie PS0+PS1 to 3.3V |
| BNO085 init hangs | INT pin floating | Add 10KΩ pull-up on INT |
| EMG always at 4095 | MyoWare SIG > 3.3V | Power MyoWare from 3.3V |
| EMG always near 0 | MyoWare VCC/GND issue | Check power connections |
| Classifier never changes | Thresholds too high | Tune thresholds in data_types.h |
| Classifier flickers | Thresholds too low | Increase POSTURE_CONFIRM_WINDOW_MS |
