# BNO085 IMU Debugging

Standalone ESP-IDF diagnostic project used to debug BNO085 IMU communication
issues (I2C/SPI setup, sensor report configuration, data parsing) before the
working driver was integrated into `BackOnTrack/components/esp32_BNO08x` and
`BackOnTrack/drivers/driver_bno085.cpp`.

`imu_diag/` went through six iterations while chasing the issue (this
directory's git history preserves each step — see `git log -- debugging/imu_diag`).
The final state here reflects the last working diagnostic build.

Recovered from local debugging scratch work that was never previously
version-controlled or pushed to this repo.
