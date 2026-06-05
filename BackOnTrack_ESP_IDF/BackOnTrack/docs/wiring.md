# Wiring Guide — Back on Track

## BNO085 SPI Wiring

The BNO085 **must** be configured in SPI mode. PS0 and PS1 must both be tied HIGH.

| BNO085 Board Pin | ESP32-S3 GPIO | Direction | Notes |
|------------------|---------------|-----------|-------|
| VIN | 3.3V | Power | |
| GND | GND | Ground | |
| CS | GPIO37 | Output (ESP → BNO) | Active LOW chip select |
| SCL / SCLK | GPIO38 | Output (ESP → BNO) | SPI clock |
| DI / MOSI | GPIO40 | Output (ESP → BNO) | Labeled "DI" on Adafruit board |
| SDA / MISO | GPIO39 | Input (BNO → ESP) | Labeled "SDA" on Adafruit board |
| INT | GPIO5 | Input (BNO → ESP) | **Active LOW data-ready. MUST have 10KΩ pull-up to 3.3V.** |
| RST | GPIO6 | Output (ESP → BNO) | Active LOW reset. Driven by ESP firmware. |
| PS0 | 3.3V | — | **Tie directly to 3.3V for SPI mode.** |
| PS1 | 3.3V | — | **Tie directly to 3.3V for SPI mode.** |

> **IMPORTANT:** If PS0 or PS1 are floating or grounded, BNO085 boots into I2C or UART mode and will not respond to SPI. All-zeros coming back from SPI = PS0/PS1 misconfigured.

> **INT pull-up:** The INT pin is open-drain active-low. Without a pull-up, it floats HIGH and looks permanently asserted. A 10KΩ pull-up from GPIO5 to 3.3V is required. The ESP32-S3 internal pull-up is enabled in software as a backup, but a hardware pull-up is strongly preferred.

---

## MyoWare 2.0 Wiring

| MyoWare 2.0 Pin | ESP32-S3 / Power | Notes |
|-----------------|------------------|-------|
| SIG | GPIO4 (ADC1_CH3) | Analog output. **Max 3.3V at ESP32-S3 ADC input.** |
| VCC | 3.3V | See safety note below |
| GND | GND | Common ground with ESP32-S3 |
| RAW+ | Not connected | Optional raw bipolar EMG (not used in this design) |
| RAW− | Not connected | — |

### Power Supply Note

MyoWare 2.0 can operate from 3.3V to 5.0V. **If you power from 5V, the SIG output can swing up to ~4.7V, which exceeds the ESP32-S3 ADC maximum input of 3.3V and WILL DAMAGE the ADC.**

Options:
- **Recommended:** Power MyoWare from 3.3V (SIG output stays within 3.3V range)
- Alternative: Power from 5V, add a 1:1.5 voltage divider (e.g. 10KΩ + 15KΩ) on SIG before GPIO4

### Safety Note

> **⚠️ Read before applying electrodes:**
> MyoWare 2.0 and ESP32-S3 share a common ground. When the ESP32-S3 is powered via USB from a laptop or PC, the electrode common ground is connected to the laptop's USB ground (and potentially to mains earth).
>
> This is generally safe for surface EMG using **dry snap electrodes** on limbs (forearm, upper trapezius, etc.).
>
> **DO NOT:**
> - Use wet gel electrodes with this non-isolated USB setup
> - Place electrodes near the heart or chest
> - Use needle or intramuscular electrodes
>
> This is a prototype for posture awareness, not a medical EMG system.

---

## Electrode Placement for Posture Monitoring

For shoulder / trapezius tension detection:
- Electrode 1: Upper trapezius, midway between neck and shoulder
- Electrode 2: ~2 cm distal along the same muscle
- Reference: Bony prominence of the acromion (top of shoulder blade)

For lumbar paraspinal monitoring (alternative, not implemented in v1):
- Place along the erector spinae muscles ~3 cm lateral to the spine at L3 level

---

## Wiring Diagram (ASCII)

```
ESP32-S3 Dev Board          Adafruit BNO085
─────────────────           ─────────────────
3.3V  ─────────────────────── VIN
GND   ─────────────────────── GND
GPIO37 ─────────────────────── CS
GPIO38 ─────────────────────── SCL (SCLK)
GPIO40 ─────────────────────── DI  (MOSI)
GPIO39 ─────────────────────── SDA (MISO)
GPIO5  ──┬──────────────────── INT
         │ 10KΩ pull-up
        3.3V
GPIO6  ─────────────────────── RST
3.3V   ─────────────────────── PS0  ← MUST tie HIGH
3.3V   ─────────────────────── PS1  ← MUST tie HIGH

ESP32-S3 Dev Board          MyoWare 2.0
─────────────────           ─────────────────
3.3V  ─────────────────────── VCC
GND   ─────────────────────── GND
GPIO4  ─────────────────────── SIG   (ADC1_CH3)
```
