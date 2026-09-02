# BTS7960 (IBT-2) Dual PWM Engine Driver for ESP-IDF

*Read in other languages: [Português](README.pt-br.md)*

This module provides a modular and optimized static C driver for controlling DC motors using the **BTS7960** high-current H-bridge (commonly sold as the **IBT-2** interface board) on **ESP32-S3** under **ESP-IDF**.

Control is performed in **Dual PWM (Slow-Decay / Active Dynamic Braking)** mode, utilizing the ESP32-S3's native **MCPWM** peripheral for high resolution and switching speed, with native support for thread-safety.

### Key Performance Features:
* **High Resolution:** The timer is configured with an 80 MHz clock resolution (the maximum hardware timer frequency on the ESP32-S3), yielding **4000 steps of resolution** at a 20 kHz frequency.
* **Software Dead-time:** Protects the H-bridge from shoot-through during direction changes with a precise 50 us transition delay (`esp_rom_delay_us(50)`).
* **Flexible Thread-safety:** Thread safety is compile-time selectable using `CONFIG_ENGINE_THREAD_SAFE` via Kconfig/menuconfig.

---

## 🔌 Suggested Pinout (ESP32-S3)

Below is the recommended wiring diagram between the **BTS7960 (IBT-2)** module and the **ESP32-S3 DevKit**:

| IBT-2 Pin (Control Side) | Signal | ESP32-S3 Pin | Description |
| :--- | :--- | :--- | :--- |
| **1 (RPWM)** | Clockwise PWM input | **GPIO 1** | MCPWM Generator 0A |
| **2 (LPWM)** | Counter-clockwise PWM input | **GPIO 2** | MCPWM Generator 0B |
| **3 (R_EN)** | Clockwise Direction Enable | **GPIO 3** | Enable GPIO (Active HIGH, connected to L_EN) |
| **4 (L_EN)** | Counter-clockwise Direction Enable | **GPIO 3** | Enable GPIO (Active HIGH, connected to R_EN) |
| **5 (R_IS)** | Clockwise current sense | **Conditioned GPIO 4** | Optional analog diagnostic/current output |
| **6 (L_IS)** | Counter-clockwise current alarm | *Not connected* | Optional analog output for overcurrent reading |
| **7 (VCC)** | Buffer logic voltage | **3.3V** | Powers the module's input buffer logic (74HC244) |
| **8 (GND)** | Common logic ground | **GND** | Common ground reference connection (Mandatory) |

> [!WARNING]
> **Logic Compatibility (3.3V vs 5V):**
> The IBT-2 module features a CMOS input buffer chip (`74HC244`). If you power the module's **7 (VCC)** pin with 5V, the minimum threshold to recognize a HIGH signal is $3.5\text{V}$, causing failure or unstable behavior since the ESP32-S3 outputs are $3.3\text{V}$. 
> **Powering the module's control VCC pin with 3.3V** natively resolves this issue, adjusting the H-bridge reading threshold to the ESP32-S3's logic voltage.

| IBT-2 Pin (Power) | Function | Connection |
| :--- | :--- | :--- |
| **B+** | Positive power supply | Positive terminal of the motor power supply/battery (6V to 27V DC) |
| **B-** | Power ground | Negative terminal of the motor power supply/battery |
| **M+ / R_OUT** | Positive motor output | DC motor terminal 1 |
| **M- / L_OUT** | Negative motor output | DC motor terminal 2 |

---

## ⚙️ Configuration (menuconfig)

You can configure pins, PWM frequency, and thread-safety graphically by running:
```bash
idf.py menuconfig
```
Under **Component config** -> **Engine Driver Configuration**:
* **`CONFIG_ENGINE_THREAD_SAFE`:** Enable thread-safety (Default: `y`). If disabled, all mutex instructions are compiled out for maximum performance.
* **`CONFIG_ENGINE_PWM_FREQ_HZ`:** Frequency of the PWM signal in Hz (Default: `20000` / 20 kHz).
* **`CONFIG_ENGINE_PIN_RPWM`:** GPIO number used for Forward PWM control (Default: `1`).
* **`CONFIG_ENGINE_PIN_LPWM`:** GPIO number used for Reverse PWM control (Default: `2`).
* **`CONFIG_ENGINE_PIN_ENABLE`:** GPIO tied to both R_EN/L_EN (Default: `3`). It is required to implement the `COAST` state.
* **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`:** Enables continuous ADC1/DMA acquisition of `R_IS`.
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`:** Conditioned ADC1 input (Default: `4`).
* **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`:** Aggregate rate (Default: `25000`), producing 25 samples per 1 ms frame.
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV`:** Calibrated `I_IS` fault-entry threshold (Default: `1500` mV).
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV`:** Hysteretic fault-exit threshold (Default: `1000` mV).

### Optional Current Measurement

The board used by this project has 10 kΩ from `R_IS` to ground. Do not connect
the GPIO directly. Add 10 kΩ in series to the ADC, 1 kΩ from ADC to ground,
100 nF from ADC to ground, and external Schottky clamps to 3.3 V/GND.

Continuous ADC processing runs on Core 0 and performs no blocking conversion in
the control task. Each 1 ms frame exposes the mean and median of 25 samples. The
mean is the primary equivalent-current estimate; the median is complementary
and helps reveal impulses or outliers. At low duty, the median can be zero even
when the mean current is nonzero.

The ADC task also calibrates the latest normal-frame mean and publishes its
PWM-weighted equivalent current as integer milliamperes. Real-time consumers
use `engine_current_sense_get_latest_current_milliamps()` as a lock-free read;
they never invoke ADC calibration from the control path. The value is marked
invalid while a frame contains no normal samples.

`I_IS` also reports BTS7960 faults with a current that is effectively
independent of load current. Samples above the configured threshold are counted
as faults and are not converted to amperes. Snapshots report the fraction of
fault samples, the number of entries into the fault state, and whether it is
still active at the end of the window.

With the 10 kΩ/10 kΩ/1 kΩ network and `k_ILIS=8500`, nominal sensitivity at the
ADC is about 56 mV/A. The wide `k_ILIS` tolerance requires calibration against a
trusted ammeter.

### Bridge states

`engine_driver_set_speed()` applies PWM for nonzero commands. An exact zero
command selects `COAST`: RPWM/LPWM are forced low and R_EN/L_EN are disabled.
`engine_driver_brake()` provides explicit active braking by keeping the enables
active with both PWM pins low. Static levels use the MCPWM force action rather
than relying on the ambiguous compare-equals-zero edge case.

---

## 💻 Usage Example

```c
#include "esp_log.h"
#include "engine_driver.h"

static const char *TAG = "APP";

// Static configuration using Kconfig defaults
static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
};

void app_main(void) {
    ESP_LOGI(TAG, "Initializing H-Bridge motor driver...");
    if (engine_driver_init(&motor) == 0) {
        ESP_LOGI(TAG, "Motor driver initialized successfully.");
        
        // Rotate clockwise at 50% speed
        engine_driver_set_speed(&motor, 50.0f);
    } else {
        ESP_LOGE(TAG, "Failed to initialize motor driver!");
    }
}
```

---

![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Description

This project is part of the **SmartSensing.me** ecosystem and goes beyond the basic examples found on the internet. Here, we apply the real fundamentals of instrumentation engineering and high-performance embedded systems.

Unlike shallow content aimed only at clicks, this repository delivers:
- **Originality:** Original implementations based on nearly 30 years of academic experience.
- **Technical Density:** Professional use of the ESP-IDF, Zephyr RTOS and FreeRTOS frameworks.
- **Didactics:** Documented and structured code for those seeking real technical growth.

> "We transform physical world signals into digital intelligence, without shortcuts."

---

## 🛠️ Technologies and Compatibility
- **Language:** Pure C (C99 or higher) and C++
- **Target Hardware:** ESP32-S3 (and other ESP32 family chips with MCPWM)
- **Environments/RTOS:** ESP-IDF (as a native Component)
- **Build System:** Native CMake

---

## 👤 About the Author

**José Alexandre de França** *Associate Professor in the Department of Electrical Engineering at UEL*

Electrical Engineer with nearly three decades of experience in undergraduate and postgraduate teaching. PhD in Electrical Engineering, researcher in electronic instrumentation, and embedded systems developer. SmartSensing.me is my commitment to raising the level of technological education in Brazil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
