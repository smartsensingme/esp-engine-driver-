#ifndef ENGINE_DRIVER_H_
#define ENGINE_DRIVER_H_

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include <stdint.h>

#if CONFIG_ENGINE_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

typedef enum {
  ENGINE_DRIVER_MODE_UNINITIALIZED = 0,
  ENGINE_DRIVER_MODE_COAST,
  ENGINE_DRIVER_MODE_BRAKE,
  ENGINE_DRIVER_MODE_DRIVE,
} engine_driver_mode_t;

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

  /** Resolved period of the PWM in timer clock cycles, cached once at
   * initialization. */
  uint32_t period_cycles;
  /** Tracks the driven direction (1 forward, -1 reverse, 0 coast/brake). */
  int last_direction;
  /** Last hardware mode, used to avoid redundant 1 kHz peripheral writes. */
  engine_driver_mode_t mode;

#if CONFIG_ENGINE_THREAD_SAFE
  /** Mutex to synchronize speed updates across multiple threads. */
  SemaphoreHandle_t mutex;
  StaticSemaphore_t mutex_buffer;
#endif
};

/**
 * @brief Initializes the engine driver pins and configures the MCPWM
 * peripheral.
 * @param engine Pointer to the config structure containing pins and frequency.
 * @return 0 on success, negative error code on failure.
 */
int engine_driver_init(struct engine_config *engine);

/**
 * @brief Sets the signed duty cycle. A zero command selects COAST.
 * @param engine Pointer to the config structure.
 * @param command Speed from -100.0f to 100.0f.
 * @return ESP_OK on success.
 */
esp_err_t engine_driver_set_speed(struct engine_config *engine, float command);

/** Immediately force both PWM pins low and disable R_EN/L_EN. */
esp_err_t engine_driver_coast(struct engine_config *engine);

/** Immediately force both PWM pins low while keeping R_EN/L_EN enabled. */
esp_err_t engine_driver_brake(struct engine_config *engine);

#endif /* ENGINE_DRIVER_H_ */
