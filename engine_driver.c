#include "engine_driver.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <inttypes.h>

static const char *TAG = "ENGINE_DRIVER";

#define ENGINE_TIMER_RESOLUTION_HZ 80000000U

#if CONFIG_ENGINE_CACHE_SAFE_CONTROL
#define ENGINE_CONTROL_ATTR IRAM_ATTR
#else
#define ENGINE_CONTROL_ATTR
#endif

/**
 * @brief Override one MCPWM output with a static low level.
 *
 * Called by force_both_pwm_low() and engine_driver_set_speed(). It is private
 * and delegates directly to the MCPWM force-level API.
 */
static esp_err_t ENGINE_CONTROL_ATTR force_low(mcpwm_gen_handle_t generator) {
  /* A held force action overrides timer/comparator-generated PWM edges. */
  return mcpwm_generator_set_force_level(generator, 0, true);
}

/**
 * @brief Remove a generator's software force and resume its PWM actions.
 *
 * Called only by engine_driver_set_speed() when entering DRIVE. It is private
 * and delegates directly to the MCPWM force-level API.
 */
static esp_err_t ENGINE_CONTROL_ATTR
release_force(mcpwm_gen_handle_t generator) {
  /* Force level -1 releases the override; hold_on=true applies synchronously.
   */
  return mcpwm_generator_set_force_level(generator, -1, true);
}

/**
 * @brief Force RPWM and LPWM low, returning the first MCPWM error.
 *
 * Called by coast_locked(), brake_locked(), and engine_driver_init(). It calls
 * force_low() for the forward generator and then for the reverse generator.
 */
static esp_err_t ENGINE_CONTROL_ATTR
force_both_pwm_low(struct engine_config *engine) {
  /* Stop forward PWM before touching reverse PWM; propagate the first error. */
  esp_err_t err = force_low(engine->gen_fwd);
  if (err == ESP_OK) {
    err = force_low(engine->gen_rev);
  }
  return err;
}

/**
 * @brief Drive the shared R_EN/L_EN GPIO to the requested logical state.
 *
 * Called by coast_locked(), brake_locked(), and engine_driver_set_speed(). A
 * missing enable pin can support enable requests only; disabling then returns
 * ESP_ERR_NOT_SUPPORTED because true COAST cannot be guaranteed.
 */
static esp_err_t ENGINE_CONTROL_ATTR
set_enable(const struct engine_config *engine, bool enabled) {
  /* Treat a negative GPIO as a deliberately unavailable enable connection. */
  if (engine->pin_enable < 0) {
    return enabled ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
  }
  return gpio_set_level(engine->pin_enable, enabled ? 1U : 0U);
}

/**
 * @brief Enter COAST while the caller already owns the optional mutex.
 *
 * Called by engine_driver_coast() and engine_driver_set_speed(). It calls
 * force_both_pwm_low() and set_enable(false), then updates cached state only
 * when both hardware operations succeed.
 */
static esp_err_t ENGINE_CONTROL_ATTR
coast_locked(struct engine_config *engine) {
  /* Idempotence block: avoid redundant peripheral writes at control-loop rate.
   */
  if (engine->mode == ENGINE_DRIVER_MODE_COAST) {
    return ESP_OK;
  }

  /* Hardware block: remove both input drives before disabling the half bridges.
   */
  esp_err_t err = force_both_pwm_low(engine);
  esp_err_t enable_err = set_enable(engine, false);

  /* State block: no direction remains active even if a hardware call failed. */
  engine->last_direction = 0;
  if (err == ESP_OK && enable_err == ESP_OK) {
    engine->mode = ENGINE_DRIVER_MODE_COAST;
  }
  return err != ESP_OK ? err : enable_err;
}

/**
 * @brief Enter active BRAKE while the caller owns the optional mutex.
 *
 * Called only by engine_driver_brake(). It calls force_both_pwm_low() and
 * set_enable(true), selecting active low-side braking on the intended module.
 */
static esp_err_t ENGINE_CONTROL_ATTR
brake_locked(struct engine_config *engine) {
  /* Idempotence block: avoid rewriting an already active static state. */
  if (engine->mode == ENGINE_DRIVER_MODE_BRAKE) {
    return ESP_OK;
  }

  /* Hardware block: both inputs low and both enables high select low-side
   * brake. */
  esp_err_t err = force_both_pwm_low(engine);
  if (err == ESP_OK) {
    err = set_enable(engine, true);
  }

  /* State block: braking has no forward/reverse driven direction. */
  engine->last_direction = 0;
  if (err == ESP_OK) {
    engine->mode = ENGINE_DRIVER_MODE_BRAKE;
  }
  return err;
}

int engine_driver_init(struct engine_config *engine) {
  /* Interface block: validate required pins and a nonzero requested frequency.
   */
  if (engine == NULL || engine->pin_fwd < 0 || engine->pin_rev < 0 ||
      engine->pwm_freq_hz == 0U) {
    return -1;
  }

  /* Enable-GPIO block: when present, configure the shared R_EN/L_EN connection
   * as an output and begin with both half bridges disabled. */
  esp_err_t err;
  if (engine->pin_enable >= 0) {
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << engine->pin_enable),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io_conf);
    if (err != ESP_OK || gpio_set_level(engine->pin_enable, 0) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure Enable pin");
      return -1;
    }
  }

  /* Timing-state block: derive integer PWM period ticks and reset cached mode.
   */
  engine->period_cycles = ENGINE_TIMER_RESOLUTION_HZ / engine->pwm_freq_hz;
  engine->last_direction = 0;
  engine->mode = ENGINE_DRIVER_MODE_UNINITIALIZED;
  if (engine->period_cycles == 0U) {
    return -1;
  }

  /* Diagnostic block: report actual tick resolution and warn when duty
   * granularity is low for the selected PWM frequency. */
  ESP_LOGI(TAG, "Dual MCPWM initializing at %" PRIu32 " Hz.",
           engine->pwm_freq_hz);
  ESP_LOGI(TAG, "Hardware Resolution: %" PRIu32 " steps.",
           engine->period_cycles);
  if (engine->period_cycles < 1024U) {
    ESP_LOGW(TAG, "PWM resolution is only %" PRIu32 " steps.",
             engine->period_cycles);
  }

  /* Timer block: create one edge-aligned up-counting timer at 80 MHz. */
  const mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
      .resolution_hz = ENGINE_TIMER_RESOLUTION_HZ,
      .period_ticks = engine->period_cycles,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  err = mcpwm_new_timer(&timer_config, &engine->timer);
  if (err != ESP_OK) {
    return -1;
  }

  /* Operator block: create the shared operator and connect it to the timer. */
  const mcpwm_operator_config_t operator_config = {.group_id = 0};
  err = mcpwm_new_operator(&operator_config, &engine->oper);
  if (err != ESP_OK ||
      mcpwm_operator_connect_timer(engine->oper, engine->timer) != ESP_OK) {
    return -1;
  }

  /* Comparator block: allocate independent forward/reverse duty thresholds and
   * initialize both compare values to zero. */
  const mcpwm_comparator_config_t compare_config = {
      .flags.update_cmp_on_tez = true,
  };
  err = mcpwm_new_comparator(engine->oper, &compare_config, &engine->cmp_fwd);
  if (err == ESP_OK) {
    err = mcpwm_new_comparator(engine->oper, &compare_config, &engine->cmp_rev);
  }
  if (err != ESP_OK ||
      mcpwm_comparator_set_compare_value(engine->cmp_fwd, 0U) != ESP_OK ||
      mcpwm_comparator_set_compare_value(engine->cmp_rev, 0U) != ESP_OK) {
    return -1;
  }

  /* Generator block: bind the forward and reverse MCPWM outputs to their GPIOs.
   */
  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = engine->pin_fwd,
  };
  err = mcpwm_new_generator(engine->oper, &generator_config, &engine->gen_fwd);
  generator_config.gen_gpio_num = engine->pin_rev;
  if (err == ESP_OK) {
    err =
        mcpwm_new_generator(engine->oper, &generator_config, &engine->gen_rev);
  }
  if (err != ESP_OK) {
    return -1;
  }

  /* Waveform block: each generator goes high at timer zero and low at its own
   * compare event, producing edge-aligned active-high PWM. */
  err = mcpwm_generator_set_action_on_timer_event(
      engine->gen_fwd, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    MCPWM_TIMER_EVENT_EMPTY,
                                                    MCPWM_GEN_ACTION_HIGH));
  if (err == ESP_OK) {
    err = mcpwm_generator_set_action_on_compare_event(
        engine->gen_fwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       engine->cmp_fwd, MCPWM_GEN_ACTION_LOW));
  }
  if (err == ESP_OK) {
    err = mcpwm_generator_set_action_on_timer_event(
        engine->gen_rev, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      MCPWM_TIMER_EVENT_EMPTY,
                                                      MCPWM_GEN_ACTION_HIGH));
  }
  if (err == ESP_OK) {
    err = mcpwm_generator_set_action_on_compare_event(
        engine->gen_rev,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       engine->cmp_rev, MCPWM_GEN_ACTION_LOW));
  }
  if (err != ESP_OK || force_both_pwm_low(engine) != ESP_OK) {
    return -1;
  }

  /* Start block: enable the configured timer and run it continuously. */
  err = mcpwm_timer_enable(engine->timer);
  if (err == ESP_OK) {
    err = mcpwm_timer_start_stop(engine->timer, MCPWM_TIMER_START_NO_STOP);
  }
  if (err != ESP_OK) {
    return -1;
  }

#if CONFIG_ENGINE_THREAD_SAFE
  /* Synchronization block: create a per-instance mutex in caller-owned storage.
   */
  engine->mutex = xSemaphoreCreateMutexStatic(&engine->mutex_buffer);
  if (engine->mutex == NULL) {
    return -1;
  }
#endif

  /* Safe-state block: complete initialization only after true COAST succeeds.
   */
  err = engine_driver_coast(engine);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "A controllable Enable pin is required for COAST mode");
    return -1;
  }

  ESP_LOGI(TAG, "Engine Driver initialized in COAST mode.");
  return 0;
}

esp_err_t ENGINE_CONTROL_ATTR
engine_driver_coast(struct engine_config *engine) {
  /* Interface block: require a previously initialized instance. */
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_ENGINE_THREAD_SAFE
  /* Serialization block: keep the hardware transition and cached state atomic
   * relative to other application commands. */
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

  /* Transition block: use the internal variant because the mutex is held. */
  esp_err_t err = coast_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
  /* Release block: publish the completed transition to other callers. */
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}

esp_err_t ENGINE_CONTROL_ATTR
engine_driver_brake(struct engine_config *engine) {
  /* Interface block: require a previously initialized instance. */
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_ENGINE_THREAD_SAFE
  /* Serialization block: protect hardware and cached mode as one operation. */
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

  /* Transition block: use the internal variant because the mutex is held. */
  esp_err_t err = brake_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
  /* Release block: publish the completed transition to other callers. */
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}

esp_err_t ENGINE_CONTROL_ATTR engine_driver_get_state(
    const struct engine_config *engine, engine_driver_state_t *state) {
  /* Interface block: both source and snapshot destination are required. */
  if (engine == NULL || state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_ENGINE_THREAD_SAFE
  /* Snapshot-lock block: mode and direction must come from one command state.
   */
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

  /* Copy block: expose only the public state, not MCPWM implementation handles.
   */
  *state = (engine_driver_state_t){
      .mode = engine->mode,
      .direction = engine->last_direction,
  };
#if CONFIG_ENGINE_THREAD_SAFE
  /* Release block: finish the coherent state snapshot. */
  xSemaphoreGive(engine->mutex);
#endif
  return ESP_OK;
}

esp_err_t ENGINE_CONTROL_ATTR
engine_driver_set_speed(struct engine_config *engine, float command) {
  /* Interface block: a bridge instance is required. */
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Saturation block: limit requested duty to the realizable percentage range.
   */
  if (command > 100.0f) {
    command = 100.0f;
  } else if (command < -100.0f) {
    command = -100.0f;
  }

#if CONFIG_ENGINE_THREAD_SAFE
  /* Serialization block: protect all hardware changes and state publication. */
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

  /* Zero-command block: exact zero intentionally maps to COAST, not BRAKE. */
  if (command == 0.0f) {
    esp_err_t err = coast_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
    xSemaphoreGive(engine->mutex);
#endif
    return err;
  }

  /* Command-decomposition block: separate sign/direction from positive duty and
   * convert percentage to timer ticks. Full scale remains one tick below period
   * because compare equal to period cannot produce the intended falling edge.
   */
  int direction = command > 0.0f ? 1 : -1;
  float duty_percent = command > 0.0f ? command : -command;
  uint32_t pulse_cycles =
      (uint32_t)(duty_percent * (float)engine->period_cycles / 100.0f);
  if (pulse_cycles >= engine->period_cycles) {
    pulse_cycles = engine->period_cycles - 1U;
  }

  /* Reversal block: remove drive, disable the bridge, and wait the configured
   * interval before selecting the opposite torque direction. */
  esp_err_t err = ESP_OK;
  if (engine->mode == ENGINE_DRIVER_MODE_DRIVE &&
      direction != engine->last_direction) {
    err = coast_locked(engine);
    if (err == ESP_OK) {
      if (engine->direction_dead_time_us > 0U) {
        esp_rom_delay_us(engine->direction_dead_time_us);
      }
    }
  }

  /* Channel-selection block: bind the command sign to one generator/comparator
   * while identifying the opposite generator that must stay forced low. */
  mcpwm_gen_handle_t active_generator =
      direction > 0 ? engine->gen_fwd : engine->gen_rev;
  mcpwm_gen_handle_t inactive_generator =
      direction > 0 ? engine->gen_rev : engine->gen_fwd;
  mcpwm_cmpr_handle_t active_comparator =
      direction > 0 ? engine->cmp_fwd : engine->cmp_rev;
  bool entering_drive = engine->mode != ENGINE_DRIVER_MODE_DRIVE ||
                        engine->last_direction != direction;

  /* Preparation block: when entering DRIVE, first guarantee the inactive input
   * is low; then update the active comparator while output remains controlled.
   */
  if (err == ESP_OK && entering_drive) {
    err = force_low(inactive_generator);
  }
  if (err == ESP_OK) {
    err = mcpwm_comparator_set_compare_value(active_comparator, pulse_cycles);
  }

  /* Enable/release block: with both PWM pins initially low, enable the half
   * bridges and release only the selected generator to its programmed PWM. */
  if (err == ESP_OK && entering_drive) {
    /* Enable with both PWM pins low, then release only the selected channel. */
    err = force_low(active_generator);
    if (err == ESP_OK) {
      err = set_enable(engine, true);
    }
    if (err == ESP_OK) {
      err = release_force(active_generator);
    }
  }

  /* Publication/recovery block: cache only a successful DRIVE state. On any
   * peripheral error, make a best-effort transition to the safer COAST state.
   */
  if (err == ESP_OK) {
    engine->last_direction = direction;
    engine->mode = ENGINE_DRIVER_MODE_DRIVE;
  } else {
    (void)coast_locked(engine);
  }

#if CONFIG_ENGINE_THREAD_SAFE
  /* Release block: allow the next command after state and hardware agree. */
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}
