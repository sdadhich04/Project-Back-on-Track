/**
 * @file imu_diag.h
 * @brief IMU0 hardware diagnostic — public entry point.
 */

#ifndef IMU_DIAG_H
#define IMU_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run all five diagnostic stages in sequence and print a final summary.
 *
 * Stages:
 *   0 — GPIO bit-bang loopback  (MOSI jumpered to MISO)
 *   1 — SPI idle MISO level     (chip held in reset, checks for GND short)
 *   2 — RST pulse + INT timing  (chip power, PS0/PS1, RST wiring)
 *   3 — Raw SHTP advertisement  (MISO connected to BNO085 SDA)
 *   4 — Product ID handshake    (MOSI connected to BNO085 DI, full round-trip)
 *   5 — Live GRV reads          (only if all prior stages pass)
 *
 * Each stage only runs if the previous stage passed. The function does
 * not return — it halts in an infinite loop after printing the summary.
 */
void imu_diag_run(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DIAG_H */
