# AI Development Log — Back on Track

*Written from the perspective of the developer (Sparsh). This log documents the AI-assisted development process.*

---

## Prompt Used

I submitted a detailed prompt requesting a full ESP-IDF posture corrector project for the ESP32-S3 using:
- Adafruit BNO085 over SPI (not I2C, not Arduino)
- MyoWare 2.0 over ESP-IDF ADC (not Arduino analogRead)
- Rule-based posture classifier
- FreeRTOS multi-task architecture
- Serial telemetry in CSV or JSON
- Gamification (points, streaks)
- Full documentation

I uploaded the Shield.zip project as a reference for:
- BNO085 SPI pin assignments (GPIO37/38/40/39/5/6, SPI3_HOST)
- SHTP protocol implementation details
- SensorContext_t HAL pattern
- FreeRTOS queue-based acquisition architecture

The MyoWare reference was from SparkFun's code examples on GitHub.

---

## AI Response Summary

The AI (Claude) read the Shield.zip source files directly, specifically:
- `components/sensor_hal/include/sensor_hal.h` — for the SensorContext_t struct and hardware config patterns
- `components/drivers/driver_bno085.c` — for the full SHTP protocol implementation
- `main/main_dual_core.cpp` — for the BNO085 SPI pin assignments and FreeRTOS task structure
- `components/data_types/include/data_types.h` — for the queue message pattern

It generated a complete ESP-IDF project adapted from these references.

---

## Code Generated

The AI generated every file in the project:

**Build system:**
- Root `CMakeLists.txt` and `sdkconfig.defaults`
- Per-component `CMakeLists.txt` files with correct `idf_component_register()` calls

**Sensor HAL:**
- `sensor_hal.h` — extended from Shield with `myoware_adc_config_t`

**Data types:**
- `data_types.h` — new structs: `imu_sample_t`, `emg_sample_t`, `posture_features_t`, `posture_classification_t`, `calibration_t`, `session_stats_t`, queue message types, all pin defines, all threshold defines

**Drivers:**
- `driver_bno085.c/.h` — adapted from Shield's working SHTP implementation, extended to enable the Game Rotation Vector report and convert quaternions to pitch/roll/yaw
- `driver_myoware.c/.h` — new; uses ESP-IDF ADC oneshot driver (not Arduino analogRead), EMA filter, baseline tracking

**Posture:**
- `posture_features.c/.h` — computes deviation from calibration baseline, runs automated calibration loop
- `posture_classifier.c/.h` — rule-based classifier with 6 states, time-based confirmation window, gamification updates

**Telemetry:**
- `telemetry_serial.c/.h` — dual CSV/JSON mode, session stats output

**Main:**
- `main.cpp` — sensor init, calibration sequence, FreeRTOS task creation, 3 tasks (imu_task, emg_task, processing_task)

**Docs:**
- README.md, wiring.md, calibration.md, testing.md, ai_log.md (this file)

---

## Key Assumptions Made by AI

1. **ESP-IDF v5.x** — uses new ADC oneshot API (`adc_oneshot_unit_handle_t`). If using ESP-IDF v4.x, the ADC API is different and `driver_myoware.c` would need to be rewritten.

2. **BNO085 Game Rotation Vector** is available on the Adafruit BNO085 breakout. This report (ID 0x08) gives a quaternion without magnetometer calibration. The AI chose this over the full rotation vector because absolute heading is not needed for posture.

3. **MyoWare powered from 3.3V.** The safety notes in `driver_myoware.h` cover the 5V case.

4. **GPIO4 = ADC1_CH3** on the ESP32-S3. This should be verified against your specific ESP32-S3 board pinout — not all ESP32-S3 boards label GPIOs identically.

5. **No BLE in v1.** BLE is explicitly left as TODO with a detailed comment about how to add it.

6. **Single BNO085 on SPI3_HOST.** If another device needs SPI3, there would be a bus conflict. The user would need to either share the bus (with separate CS pins) or move the BNO085 to SPI2_HOST.

---

## What I Changed Manually

*(This section to be filled in after first hardware test)*

- [ ] Verified GPIO4 = ADC1_CH3 on my specific ESP32-S3 board
- [ ] Adjusted `POSTURE_PITCH_SLOUCH_DEG` after observing my typical posture range
- [ ] Adjusted `POSTURE_EMG_HIGH_THRESHOLD` after baseline calibration session
- [ ] Changed `TELEMETRY_OUTPUT_MODE` to 1 (JSON) for easier logging

---

## What Worked

*(Fill in after hardware testing)*

- [ ] BNO085 SPI init and Product ID response
- [ ] Quaternion to pitch/roll conversion
- [ ] MyoWare ADC reading and EMA smoothing
- [ ] Calibration sequence
- [ ] Classifier state transitions
- [ ] CSV telemetry output
- [ ] FreeRTOS tasks running simultaneously

---

## What Did Not Work

*(Fill in after hardware testing)*

- [ ] ...

---

## Debugging Notes

*(Fill in after hardware testing)*

---

## Final Reflection

*(Fill in after hardware testing)*

This project builds on the Shield reference architecture, which was already verified to work on physical hardware. The BNO085 SHTP implementation in Shield was particularly detailed and well-debugged (the comments about CS toggling, init-phase heartbeats, and FRS draining were very specific). Reusing that implementation and extending it with quaternion parsing was more reliable than starting from scratch.

The MyoWare driver was written from scratch using the ESP-IDF v5.x ADC oneshot API, which is cleaner than the v4.x API. The EMA filter and baseline tracking are conceptually simple but should be tuned per-user.

The rule-based classifier is intentionally simple — it's a starting point, not an end product. The classification data generated during real use (via CSV telemetry) could be used as training data for a TinyML model in the future.
