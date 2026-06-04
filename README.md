# BTS7960 (IBT-2) Dual PWM Engine Driver for Zephyr RTOS

*Read in other languages: [Português](README.pt-br.md)*

This module provides a modular and optimized static C driver for controlling DC motors using the **BTS7960** high-current H-bridge (commonly sold as the **IBT-2** interface board).

Control is performed in **Dual PWM (Slow-Decay / Active Dynamic Braking)** mode, utilizing STMicroelectronics Low-Layer (LL) macros for maximum processing and switching speed, with native support for thread-safety.

### Key Performance Features:
* **Native Dynamic Resolution:** The speed input (from `-100.0f` to `100.0f`) is mapped directly to the hardware timer's clock cycles, utilizing the maximum physical precision (no intermediate logical steps limited to 10 or 12 bits).
* **Boot Diagnostics:** During boot, `engine_driver_init` prints the configured physical frequency in Hz and the actual hardware resolution in steps to the console. A warning (`WARNING`) is issued if the resolution drops below 1024 steps to ensure optimal control loop performance.

---

## 🔌 Suggested Pinout

Below is the recommended electrical wiring diagram between the **BTS7960 (IBT-2)** module and the **WeAct STM32G431 Core Board** development board:

| IBT-2 Pin (Control Side) | Signal | STM32G431 Pin | Description |
| :--- | :--- | :--- | :--- |
| **1 (RPWM)** | Clockwise signal input | **PA0** | Timer `TIM2` Channel 1 (PWM switching) |
| **2 (LPWM)** | Counter-clockwise signal input | **PA1** | Timer `TIM2` Channel 2 (PWM switching) |
| **3 (R_EN)** | Clockwise direction Enable | **PA4** | Enable GPIO (Active HIGH) |
| **4 (L_EN)** | Counter-clockwise direction Enable | **PA4** | Enable GPIO (Active HIGH, connected to R_EN) |
| **5 (R_IS)** | Clockwise current alarm | *Not connected* | Optional analog output for overcurrent reading |
| **6 (L_IS)** | Counter-clockwise current alarm | *Not connected* | Optional analog output for overcurrent reading |
| **7 (VCC)** | Buffer logic voltage | **3.3V** | Powers the module's input buffer logic (74HC244) |
| **8 (GND)** | Common logic ground | **GND** | Common ground reference connection (Mandatory) |

> [!WARNING]
> **Logic Compatibility (3.3V vs 5V):**
> The IBT-2 module features a CMOS input buffer chip (`74HC244`). If you power the module's **7 (VCC)** pin with 5V, the minimum threshold to recognize a HIGH signal is $3.5\text{V}$, causing failure or unstable behavior since the STM32 outputs are $3.3\text{V}$. 
> **Powering the module's control VCC pin with 3.3V** natively resolves this issue, adjusting the H-bridge reading threshold to the STM32's logic voltage.

| IBT-2 Pin (Power) | Function | Connection |
| :--- | :--- | :--- |
| **B+** | Positive power supply | Positive terminal of the motor power supply/battery (6V to 27V DC) |
| **B-** | Power ground | Negative terminal of the motor power supply/battery |
| **M+ / R_OUT** | Positive motor output | DC motor terminal 1 |
| **M- / L_OUT** | Negative motor output | DC motor terminal 2 |

---

## ⚙️ Integration into Other Zephyr Projects

To port this driver to another Zephyr RTOS project, follow the steps below:

### Step 1: Copy the Driver Directory
Copy the `engine-driver/` directory into your project's source folder (for example, inside `src/engine-driver/`).

### Step 2: Configure `CMakeLists.txt`
In the root `CMakeLists.txt` of your new project, add the subdirectory and link the static library to your executable (`app`):
```cmake
add_subdirectory(src/engine-driver)
target_link_libraries(app PRIVATE engine_driver)
```

### Step 3: Copy Devicetree Definitions
1. Copy the binding template file `generic-engine.example.yml` (located inside this folder) to your new project's bindings directory, renaming it to `generic-engine.yaml` (usually under `dts/bindings/generic-engine.yaml` or `boards/bindings/generic-engine.yaml`).
2. Add the following configurations to your board's overlay file (e.g., `app.overlay`):
   ```dts
   / {
       engine: engine {
           compatible = "generic-engine";
           pwms = <&pwm2 1 50000 PWM_POLARITY_NORMAL>, /* TIM2 CH1 on PA0 (50us = 20kHz) */
                  <&pwm2 2 50000 PWM_POLARITY_NORMAL>; /* TIM2 CH2 on PA1 (50us = 20kHz) */
           enable-gpios = <&gpioa 4 GPIO_ACTIVE_HIGH>; /* R_EN and L_EN on PA4 */
           status = "okay";
       };
   };

   &timers2 {
       status = "okay";
       pwm2: pwm {
           status = "okay";
           pinctrl-0 = <&tim2_ch1_pa0 &tim2_ch2_pa1>; /* Enables hardware PWM pinout on PA0 and PA1 */
           pinctrl-names = "default";
       };
   };
   ```

### Step 4: Configure `prj.conf` and `Kconfig`
In the `prj.conf` of your new project, enable the required flags:
```kconfig
# Enables the Zephyr PWM subsystem
CONFIG_PWM=y

# Enables Thread-Safe motor driving if necessary (optional)
CONFIG_ENGINE_THREAD_SAFE=y
```
If using the synchronization flag above (`CONFIG_ENGINE_THREAD_SAFE`), remember to declare the corresponding configuration menu in your project's root `Kconfig` file.

---

## 💻 Usage Example

Here is a simple C code example demonstrating how to declare, initialize, and control motor speed:

```c
#include <zephyr/kernel.h>
#include <stdio.h>
#include "engine_driver.h"

// Defines the driver structure based on the nodes created in the Devicetree
static struct engine_config engine = {
    .pwm_fwd = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(engine), 0),
    .pwm_rev = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(engine), 1),
    .enable  = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(engine), enable_gpios, {0}),
};

int main(void) {
    printf("Initializing motor driver...\n");
    
    int ret = engine_driver_init(&engine);
    if (ret < 0) {
        printf("Driver initialization error (%d)\n", ret);
        return ret;
    }
    
    printf("Initialized successfully!\n");

    while (1) {
        // Rotate clockwise at 30% speed
        engine_driver_set_speed(&engine, 30.0f);
        k_msleep(3000);

        // Apply active electronic brake (Slow Decay)
        engine_driver_set_speed(&engine, 0.0f);
        k_msleep(1500);

        // Rotate counter-clockwise at 50% speed
        engine_driver_set_speed(&engine, -50.0f);
        k_msleep(3000);

        // Stop and wait
        engine_driver_set_speed(&engine, 0.0f);
        k_msleep(2000);
    }

    return 0;
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
- **Target Hardware:** Any microcontroller (ESP32, STM32, ARM Cortex, RISC-V, AVR, etc.) or desktop architecture
- **Environments/RTOS:** ESP-IDF (as a native Component), Zephyr RTOS, FreeRTOS, Bare-metal, Desktop (Windows, Linux, macOS)
- **Build System:** Native CMake
- **Simulation:** LTSpice (Sensor modeling and validation)

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
