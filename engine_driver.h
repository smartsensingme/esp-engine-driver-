#ifndef ENGINE_DRIVER_H_
#define ENGINE_DRIVER_H_

#include <stdint.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#if CONFIG_ENGINE_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

struct engine_config {
    /** GPIO pin for Forward direction (RPWM) */
    int pin_fwd;
    /** GPIO pin for Reverse direction (LPWM) */
    int pin_rev;
    /** GPIO pin for H-Bridge Enable control (R_EN/L_EN) */
    int pin_enable;
    /** Target PWM frequency in Hz */
    uint32_t pwm_freq_hz;

    // Internal driver handles
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_gen_handle_t gen_fwd;
    mcpwm_gen_handle_t gen_rev;
    mcpwm_cmpr_handle_t cmp_fwd;
    mcpwm_cmpr_handle_t cmp_rev;

    /** Resolved period of the PWM in timer clock cycles, cached once at initialization. */
    uint32_t period_cycles;
    /** Tracks the last applied direction state (1 for Fwd, -1 for Rev, 0 for Brake) to trigger dead-time. */
    int last_direction;

#if CONFIG_ENGINE_THREAD_SAFE
    /** Mutex to synchronize speed updates across multiple threads. */
    SemaphoreHandle_t mutex;
    StaticSemaphore_t mutex_buffer;
#endif
};

/**
 * @brief Initializes the engine driver pins and configures the MCPWM peripheral.
 * @param engine Pointer to the config structure containing pins and frequency.
 * @return 0 on success, negative error code on failure.
 */
int engine_driver_init(struct engine_config *engine);

/**
 * @brief Sets the speed / duty cycle of the engine.
 * @param engine Pointer to the config structure.
 * @param command Speed from -100.0f to 100.0f.
 */
void engine_driver_set_speed(struct engine_config *engine, float command);

#endif /* ENGINE_DRIVER_H_ */
