# Back on Track

Back on Track is ESP-IDF firmware for a wearable posture-awareness prototype. The active application in [`BackOnTrack/`](BackOnTrack/) reads orientation from a BNO085 IMU and an analog MyoWare 2.0 signal, establishes a neutral calibration baseline, and produces posture-related telemetry.

The firmware is a prototype for posture awareness, not a medical device.

## What it does

- Acquires orientation from the active upper-back BNO085 and EMG samples from MyoWare 2.0.
- Calibrates neutral orientation and resting EMG at startup; a BLE command can request recalibration.
- Computes posture features and runs a local rule-based classifier for serial debugging.
- Streams a packed raw-sensor packet over a NimBLE GATT peripheral and accepts BLE commands to calibrate or reset session statistics.
- Emits CSV or JSON serial telemetry, including local classifier and session-statistic fields.

The source configures a second, lower-back BNO085 context, but deliberately disables it at runtime because the bundled BNO08x stack is single-instance. The current firmware therefore runs with one active IMU.

## Confirmed hardware and tools

| Item | Evidence in this repository |
| --- | --- |
| ESP32-S3 development board | `sdkconfig.defaults` targets `esp32s3`; the KiCad schematic names an ESP32-S3-DEVKITC-1-N8R2. |
| BNO085 IMU | `main/main.cpp` initializes the upper BNO085 over SPI; `docs/wiring.md` records its SPI connections. |
| MyoWare 2.0 muscle sensor | `main/main.cpp` initializes its ADC context, and `components/drivers/driver_myoware.c` implements acquisition. |
| Bluetooth Low Energy | `components/ble/` implements a NimBLE GATT peripheral; the ESP-IDF defaults enable NimBLE in peripheral mode. |

Before powering hardware, follow [`BackOnTrack/docs/wiring.md`](BackOnTrack/docs/wiring.md). It documents the BNO085 SPI-mode requirements and the MyoWare ADC connection.

## Firmware architecture

The ESP-IDF project is [`BackOnTrack/`](BackOnTrack/). Its root `CMakeLists.txt` registers these application components:

- `main/main.cpp` initializes NVS, BLE, sensor contexts, calibration, queues, and FreeRTOS tasks.
- `components/sensor_hal` defines the common `SensorContext_t` and hardware-configuration types.
- `components/data_types` owns shared samples, calibration data, thresholds, queue messages, and session statistics.
- `components/drivers` contains the BNO085 SPI driver and MyoWare ADC driver; `components/esp32_BNO08x` is the bundled BNO08x dependency.
- `components/posture` computes features and applies the rule-based classifier.
- `components/ble` builds and notifies the BLE packet and handles control commands.
- `components/telemetry` formats serial CSV or JSON output.

At runtime, `imu_upper_task` and `emg_task` place samples in FreeRTOS queues. `processing_task` drains those queues, sends the current raw sensor packet over BLE, then computes features, classification, session statistics, and serial telemetry. The upper-IMU and EMG tasks are enabled; the lower-IMU task is not launched.

## Build, flash, and demo

Install ESP-IDF with an ESP32-S3 toolchain, then open a terminal in `BackOnTrack/`:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Replace `<PORT>` with the board's serial port, such as `COM3` on Windows. The project defaults configure ESP-IDF for the ESP32-S3 and enable NimBLE.

For the included desktop BLE receiver, install its stated dependency and run it from `BackOnTrack/tools/`:

```bash
python -m pip install bleak
python bot_ble_receiver.py
```

The receiver scans for the advertised device, subscribes to its sensor characteristic, and can log received packets to CSV. See [`BackOnTrack/tools/bot_ble_receiver.py`](BackOnTrack/tools/bot_ble_receiver.py) for its command-line options.

## Credits

- The bundled `components/esp32_BNO08x` library is MIT-licensed and credits Myles Parfeniuk; its license is retained at [`BackOnTrack/components/esp32_BNO08x/LICENSE`](BackOnTrack/components/esp32_BNO08x/LICENSE).
- `components/sensor_hal/include/sensor_hal.h` documents that its sensor-HAL pattern was adapted from Project SHIELD.
