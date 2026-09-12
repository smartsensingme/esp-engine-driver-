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

/** Hardware state most recently applied by the bridge driver. */
typedef enum {
  /** Instance has not completed engine_driver_init(). */
  ENGINE_DRIVER_MODE_UNINITIALIZED = 0,
  /** Both PWM inputs low and the shared enable output inactive. */
  ENGINE_DRIVER_MODE_COAST,
  /** Both PWM inputs low and the shared enable output active. */
  ENGINE_DRIVER_MODE_BRAKE,
  /** One PWM input active and the opposite input forced low. */
  ENGINE_DRIVER_MODE_DRIVE,
} engine_driver_mode_t;

/** Read-only snapshot of the effective bridge command state. */
typedef struct {
  /** Last successfully applied COAST, BRAKE, or DRIVE mode. */
  engine_driver_mode_t mode;
  /** +1 for forward DRIVE, -1 for reverse DRIVE, or 0 otherwise. */
  int direction;
} engine_driver_state_t;

/** Configuration and private runtime handles for one motor bridge. */
struct engine_config {
  /** GPIO connected to RPWM, the positive-command PWM input. */
  int pin_fwd;
  /** GPIO connected to LPWM, the negative-command PWM input. */
  int pin_rev;
  /** GPIO connected to both R_EN and L_EN; a negative value means unavailable.
   */
  int pin_enable;
  /** Requested PWM frequency in hertz. */
  uint32_t pwm_freq_hz;
  /** COAST interval inserted before reversing a nonzero command, in us. */
  uint32_t direction_dead_time_us;

  /** MCPWM timer created during initialization. */
  mcpwm_timer_handle_t timer;
  /** MCPWM operator connected to timer. */
  mcpwm_oper_handle_t oper;
  /** MCPWM generator driving RPWM. */
  mcpwm_gen_handle_t gen_fwd;
  /** MCPWM generator driving LPWM. */
  mcpwm_gen_handle_t gen_rev;
  /** Comparator defining the RPWM duty cycle. */
  mcpwm_cmpr_handle_t cmp_fwd;
  /** Comparator defining the LPWM duty cycle. */
  mcpwm_cmpr_handle_t cmp_rev;

  /** Resolved period of the PWM in timer clock cycles, cached once at
   * initialization. */
  uint32_t period_cycles;
  /** Tracks the driven direction (1 forward, -1 reverse, 0 coast/brake). */
  int last_direction;
  /** Last hardware mode, used to avoid redundant 1 kHz peripheral writes. */
  engine_driver_mode_t mode;

#if CONFIG_ENGINE_THREAD_SAFE
  /** Per-instance mutex protecting commands and reported state. */
  SemaphoreHandle_t mutex;
  /** Static mutex storage; initialization performs no heap allocation for it.
   */
  StaticSemaphore_t mutex_buffer;
#endif
};

/**
 * @brief Initialize one bidirectional MCPWM bridge instance in COAST mode.
 *
 * The caller fills the pin, frequency, and reversal-dead-time fields before the
 * call. The function configures the optional shared enable GPIO, creates one
 * timer, operator, two comparators, and two generators, installs edge-aligned
 * PWM actions, starts the timer, creates the optional static mutex, and leaves
 * both half bridges disabled in COAST.
 *
 * This public entry point is called only by application code. No function in
 * this component calls it. Internally it calls force_both_pwm_low() during
 * setup and engine_driver_coast() for the final safe state.
 *
 * @param[in,out] engine Caller-owned configuration/instance structure.
 * @return 0 on success or -1 for invalid configuration, peripheral allocation,
 *         GPIO/MCPWM setup, mutex creation, or inability to enter COAST.
 */
int engine_driver_init(struct engine_config *engine);

/**
 * @brief Apply a signed PWM command, clamped to -100 through +100 percent.
 *
 * Positive values drive RPWM, negative values drive LPWM, and exactly zero
 * selects COAST. The inactive PWM input is always forced low. When a nonzero
 * DRIVE command changes sign, the function first enters COAST, waits the
 * configured direction_dead_time_us, and only then enables the opposite input.
 * A hardware error causes a best-effort transition to COAST.
 *
 * This public entry point is called only by application code. No function in
 * this component calls it. Internally it calls coast_locked(), force_low(),
 * set_enable(), and release_force().
 *
 * @param[in,out] engine Previously initialized bridge instance.
 * @param[in] command Signed requested duty cycle in percent; out-of-range
 * finite values are clamped. The caller must not pass NaN.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance, or the
 *         first GPIO/MCPWM error encountered.
 * @note With CONFIG_ENGINE_CACHE_SAFE_CONTROL enabled, this function and its
 *       complete component-side control call graph execute from IRAM. The
 *       instance, caller stack, and application call graph must also be kept in
 *       internal memory.
 */
esp_err_t engine_driver_set_speed(struct engine_config *engine, float command);

/**
 * @brief Immediately force both PWM inputs low and disable both half bridges.
 *
 * This is the free-running COAST state. It requires a controllable shared
 * enable pin; a configuration with pin_enable below zero returns
 * ESP_ERR_NOT_SUPPORTED when disabling is requested.
 *
 * This public function is called internally only by engine_driver_init().
 * Application code may call it directly. It delegates to coast_locked(), which
 * calls force_both_pwm_low() and set_enable(false).
 *
 * @param[in,out] engine Previously initialized bridge instance.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance,
 *         ESP_ERR_NOT_SUPPORTED without an enable pin, or a GPIO/MCPWM error.
 * @note Included in the optional CONFIG_ENGINE_CACHE_SAFE_CONTROL path.
 */
esp_err_t engine_driver_coast(struct engine_config *engine);

/**
 * @brief Apply active low-side braking with both PWM inputs forced low.
 *
 * Both half bridges remain enabled, selecting the BTS7960 active low-side brake
 * state. This electrically differs from COAST, where the half bridges are
 * disabled and the motor is allowed to freewheel.
 *
 * This public function is called only by application code. No function in this
 * component calls it. It delegates to brake_locked(), which calls
 * force_both_pwm_low() and set_enable(true).
 *
 * @param[in,out] engine Previously initialized bridge instance.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance, or a
 *         GPIO/MCPWM error.
 * @note Included in the optional CONFIG_ENGINE_CACHE_SAFE_CONTROL path.
 */
esp_err_t engine_driver_brake(struct engine_config *engine);

/**
 * @brief Copy the last successfully applied bridge mode and drive direction.
 *
 * This getter performs no peripheral access and does not change the bridge.
 * This public function is called only by application code; no function inside
 * the component calls it.
 *
 * @param[in] engine Previously initialized bridge instance.
 * @param[out] state Destination for the cached state snapshot.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for a null pointer.
 * @note Included in the optional CONFIG_ENGINE_CACHE_SAFE_CONTROL path.
 */
esp_err_t engine_driver_get_state(const struct engine_config *engine,
                                  engine_driver_state_t *state);

#endif /* ENGINE_DRIVER_H_ */
