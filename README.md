# BTS7960 (IBT-2) Dual PWM Engine Driver for ESP-IDF

*Read in other languages: [Português](README.pt-br.md)*

This module provides a modular and optimized static C driver for controlling DC motors using the **BTS7960** high-current H-bridge (commonly sold as the **IBT-2** interface board) on **ESP32-S3** under **ESP-IDF**.

Control is performed in **Dual PWM (Slow-Decay / Active Dynamic Braking)** mode, utilizing the ESP32-S3's native **MCPWM** peripheral for high resolution and switching speed, with native support for thread-safety.

### Key Performance Features:
* **High Resolution:** The timer is configured with an 80 MHz clock resolution (the maximum hardware timer frequency on the ESP32-S3), yielding **4000 steps of resolution** at a 20 kHz frequency.
* **Configurable Software Dead-time:** Protects the H-bridge during direction changes with a configurable COAST interval (50 us by default).
* **Flexible Thread-safety:** Thread safety is compile-time selectable using `CONFIG_ENGINE_THREAD_SAFE` via Kconfig/menuconfig.
* **Optional cache-safe control:** The bridge command path can be placed in
  IRAM together with ESP-IDF's MCPWM/GPIO control routines.

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
| **6 (L_IS)** | Counter-clockwise current sense | **Conditioned GPIO 5** | Reverse-current and fault diagnostics |
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
* **`CONFIG_ENGINE_CACHE_SAFE_CONTROL`:** Places `set_speed`, `coast`,
  `brake`, state readback, and their component-side call graph in IRAM. It
  requires thread-safety to be disabled and automatically selects ESP-IDF's
  cache-safe MCPWM/GPIO control options (Default: `n`).
* **`CONFIG_ENGINE_PWM_FREQ_HZ`:** Frequency of the PWM signal in Hz (Default: `20000` / 20 kHz).
* **`CONFIG_ENGINE_PIN_RPWM`:** GPIO number used for Forward PWM control (Default: `1`).
* **`CONFIG_ENGINE_PIN_LPWM`:** GPIO number used for Reverse PWM control (Default: `2`).
* **`CONFIG_ENGINE_PIN_ENABLE`:** GPIO tied to both R_EN/L_EN (Default: `3`). It is required to implement the `COAST` state.
* **`CONFIG_ENGINE_DIRECTION_DEAD_TIME_US`:** Coast interval before a driven direction reversal (Default: `50` us).
* **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`:** Enables continuous ADC1/DMA acquisition of `R_IS` and `L_IS`.
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`:** Conditioned ADC1 input (Default: `4`).
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_L_IS`:** Independently conditioned ADC1 input (Default: `5`).
* **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`:** Per-channel rate (Default: `25000`), producing 25 samples from each input per 1 ms frame.
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV`:** Calibrated `I_IS` fault-entry threshold (Default: `1500` mV).
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV`:** Hysteretic fault-exit threshold (Default: `1000` mV).

### Optional Current Measurement

The board used by this project has 10 kΩ from each `I_IS` output to ground. Do
not connect either GPIO directly and do not tie `R_IS` to `L_IS`. Give each
channel its own 10 kΩ series resistor to the ADC, 1 kΩ from the ADC node to
ground, 100 nF from the ADC node to ground, and external Schottky clamps to
3.3 V/GND.

Continuous ADC processing runs on Core 0 and performs no blocking conversion in
the control task. Each 1 ms frame contains 25 samples from each channel. Fault
hysteresis and diagnostics are independent for `R_IS` and `L_IS`. The task
publishes both calibrated magnitudes plus validity and fault masks. The
application selects `R_IS` during positive drive and `L_IS` during negative
drive, preventing inactive-channel noise from changing the current sign.

The ADC task calibrates both normal-frame means and publishes them as integer
milliamperes through `engine_current_sense_get_latest_measurement()`; ADC
calibration never runs in the control path. The application retains both
channels in the published structure and uses the effective driver state to
determine current direction without discarding information.

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

### Cache-disabled control path

Enable `CONFIG_ENGINE_CACHE_SAFE_CONTROL` only when one deterministic task owns
the driver instance. Besides moving the driver commands to IRAM, the option
selects `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` and
`CONFIG_GPIO_CTRL_FUNC_IN_IRAM`; the MCPWM option also keeps its runtime objects
out of external RAM. A conditional linker fragment additionally moves
`mcpwm_generator_set_force_level()` because ESP-IDF 6.0 does not include that
operation in its control-function IRAM mapping. The caller-owned
`struct engine_config`, caller stack, and the rest of the application call graph
must still reside in internal memory.

Initialization, logging, current-sense acquisition, and current-sense reporting
are outside this guarantee. A direction reversal may also execute the configured
blocking dead time, so cache safety does not imply that every command meets a
particular deadline.

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
    .direction_dead_time_us = CONFIG_ENGINE_DIRECTION_DEAD_TIME_US,
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

## The command is duty, not speed

Despite the historical `engine_driver_set_speed()` name, its argument is a
signed PWM duty command rather than measured or guaranteed speed:

```text
command > 0  -> PWM on RPWM, LPWM forced low
command < 0  -> PWM on LPWM, RPWM forced low
command = 0  -> COAST
```

The driver clamps it to `[-100, +100] %`. Actual speed depends on supply, motor,
load, friction, and back EMF. A speed loop must measure rotation and generate
this command with a controller such as `esp_pid`.

## Electrical bridge states

| API state | R_EN/L_EN | RPWM | LPWM | Expected behavior |
|---|---:|---:|---:|---|
| `COAST` | 0 | forced 0 | forced 0 | Bridge disabled; free-running shaft |
| `BRAKE` | 1 | forced 0 | forced 0 | Active low-side braking |
| positive `DRIVE` | 1 | PWM | forced 0 | Positive torque |
| negative `DRIVE` | 1 | forced 0 | PWM | Negative torque |

COAST and BRAKE are not equivalent. The exact electrical behavior and safety
depend on the real IBT-2 board, motor, and supply. This component does not
measure bus voltage, junction temperature, current limit, or regenerated energy.

## Direction reversal

When a nonzero DRIVE command changes sign, the driver enters COAST, waits
`direction_dead_time_us`, programs the opposite channel, enables the bridge
while both inputs are low, and releases only the selected PWM generator. The
delay uses blocking `esp_rom_delay_us()`; do not configure an excessive value
inside a real-time control task. Dead time reduces transition risk but does not
limit current, torque, or energy.

## Driver lifecycle and API

Fill the configuration fields of `struct engine_config`, call
`engine_driver_init()` once, then use `engine_driver_set_speed()`,
`engine_driver_brake()`, and `engine_driver_coast()`. The structure must remain
alive because it owns MCPWM handles and runtime state. The current component has
no `deinit()` API.

| Function | Purpose | Called internally by |
|---|---|---|
| `engine_driver_init` | Create MCPWM resources and finish in COAST | Nobody |
| `engine_driver_set_speed` | Apply clamped signed PWM; zero means COAST | Nobody |
| `engine_driver_coast` | Force both inputs low and disable R_EN/L_EN | `engine_driver_init` |
| `engine_driver_brake` | Force both inputs low while keeping enables active | Nobody |
| `engine_driver_get_state` | Copy effective mode and direction | Nobody |

`engine_driver_init()` retains its historical return convention: `0` means
success and `-1` means failure. The command/state functions return `esp_err_t`.
Do not pass NaN to `engine_driver_set_speed()`; duty conversion requires a
finite value. A controllable `pin_enable` is required for true COAST.

## Current-sense architecture

With `CONFIG_ENGINE_CURRENT_SENSE_ENABLE=y`,
`engine_current_sense_start()` configures ADC1 continuous mode and DMA to sample
R_IS and L_IS. ISR callbacks only notify a low-priority Core 0 worker. The worker
drains DMA, separates channels, applies independent hysteretic fault states,
excludes fault signaling from normal-current means, calibrates both ADC inputs,
and publishes one coherent result per nominal one-millisecond frame.

The control path reads precomputed integer milliamperes and performs no blocking
ADC conversion or calibration.

## Conditioning each `I_IS` channel

Never connect R_IS and L_IS together. Each output uses its own network:

```text
module I_IS ---- 10 kohm series ----+---- ADC1 GPIO
                                    |
                                    +---- 1 kohm ---- GND
                                    +---- 100 nF ---- GND
                                    `---- Schottky clamps to 3V3/GND
```

The target board also has a 10 kohm resistor from each I_IS output to ground.
Kconfig values must describe the actual board, series, and pulldown resistors.
The capacitor attenuates switching noise but does not replace current limiting
or clamps. Because the BTS7960 current-sense ratio has wide tolerance, compare
the result with a trusted ammeter before using it as a quantitative protection.

## Current direction and faults

`engine_current_sense_get_latest_measurement()` returns separate nonnegative
R_IS/L_IS magnitudes plus validity and fault masks. Combine it with
`engine_driver_get_state()`: use R_IS for positive DRIVE, L_IS for negative
DRIVE, and multiply by effective direction when a signed application value is
needed.

I_IS multiplexes load-current information and fault signaling. A calibrated
ADC voltage at/above `FAULT_ENTER_MV` enters fault state and a voltage at/below
`FAULT_EXIT_MV` leaves it. Samples classified as fault are never converted to
load current. The indication alone does not identify the physical cause; it may
need correlation with voltage, temperature, and the device datasheet.

## Current-sense API

| Function | Consumes data? | Purpose |
|---|:---:|---|
| `engine_current_sense_start` | — | Create ADC/DMA, calibration, worker, callbacks |
| `engine_current_sense_stop` | — | Stop and release acquisition resources |
| `engine_current_sense_get_latest_measurement` | No | Calibrated currents and masks for both channels |
| `engine_current_sense_get_latest_dual_frame` | No | Latest coherent raw dual frame |
| `engine_current_sense_take_dual_snapshot` | Yes | Copy/reset both diagnostic accumulators |
| `engine_current_sense_channel_raw_to_millivolts` | No | Channel-specific calibrated raw conversion |
| `engine_current_sense_adc_to_r_is_millivolts` | No | Reconstruct module-pin voltage |
| `engine_current_sense_adc_to_i_is_milliamperes` | No | Estimate I_IS output current |
| `engine_current_sense_adc_to_amperes` | No | Estimate motor-current magnitude |

The `take_*snapshot()` calls reset their diagnostic windows; latest-frame
getters do not consume data. Complete parameters, returns, callees, and caller
relationships are documented in [`engine_driver.h`](engine_driver.h) and
[`engine_current_sense.h`](engine_current_sense.h). Both `.c` files are divided
into commented functional blocks.

## Breaking-change log

### 2026-09-10 — removal of single-channel current APIs

Four functions previously retained for compatibility with R_IS-only
acquisition were removed:

| Removed function | Replacement |
|---|---|
| `engine_current_sense_take_snapshot()` | `engine_current_sense_take_dual_snapshot()` |
| `engine_current_sense_get_latest_frame()` | `engine_current_sense_get_latest_dual_frame()` |
| `engine_current_sense_get_latest_current_milliamps()` | `engine_current_sense_get_latest_measurement()` combined with `engine_driver_get_state()` |
| `engine_current_sense_raw_to_millivolts()` | `engine_current_sense_channel_raw_to_millivolts()` |

This is a source-incompatible change. It removes interfaces that discarded one
channel or inferred current sign without explicitly considering the bridge's
applied state.

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
