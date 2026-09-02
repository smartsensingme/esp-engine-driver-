#include "engine_driver.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include <inttypes.h>

static const char *TAG = "ENGINE_DRIVER";

#define ENGINE_TIMER_RESOLUTION_HZ 80000000U
#define ENGINE_DIRECTION_DEAD_TIME_US 50U

static esp_err_t force_low(mcpwm_gen_handle_t generator) {
  return mcpwm_generator_set_force_level(generator, 0, true);
}

static esp_err_t release_force(mcpwm_gen_handle_t generator) {
  return mcpwm_generator_set_force_level(generator, -1, true);
}

static esp_err_t force_both_pwm_low(struct engine_config *engine) {
  esp_err_t err = force_low(engine->gen_fwd);
  if (err == ESP_OK) {
    err = force_low(engine->gen_rev);
  }
  return err;
}

static esp_err_t set_enable(const struct engine_config *engine, bool enabled) {
  if (engine->pin_enable < 0) {
    return enabled ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
  }
  return gpio_set_level(engine->pin_enable, enabled ? 1U : 0U);
}

static esp_err_t coast_locked(struct engine_config *engine) {
  if (engine->mode == ENGINE_DRIVER_MODE_COAST) {
    return ESP_OK;
  }
  /* Remove drive immediately before disabling both BTS7960 half bridges. */
  esp_err_t err = force_both_pwm_low(engine);
  esp_err_t enable_err = set_enable(engine, false);
  engine->last_direction = 0;
  if (err == ESP_OK && enable_err == ESP_OK) {
    engine->mode = ENGINE_DRIVER_MODE_COAST;
  }
  return err != ESP_OK ? err : enable_err;
}

static esp_err_t brake_locked(struct engine_config *engine) {
  if (engine->mode == ENGINE_DRIVER_MODE_BRAKE) {
    return ESP_OK;
  }
  /* Both IN pins low with both INH pins high selects active low-side brake. */
  esp_err_t err = force_both_pwm_low(engine);
  if (err == ESP_OK) {
    err = set_enable(engine, true);
  }
  engine->last_direction = 0;
  if (err == ESP_OK) {
    engine->mode = ENGINE_DRIVER_MODE_BRAKE;
  }
  return err;
}

int engine_driver_init(struct engine_config *engine) {
  if (engine == NULL || engine->pin_fwd < 0 || engine->pin_rev < 0 ||
      engine->pwm_freq_hz == 0U) {
    return -1;
  }

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

  engine->period_cycles = ENGINE_TIMER_RESOLUTION_HZ / engine->pwm_freq_hz;
  engine->last_direction = 0;
  engine->mode = ENGINE_DRIVER_MODE_UNINITIALIZED;
  if (engine->period_cycles == 0U) {
    return -1;
  }

  ESP_LOGI(TAG, "Dual MCPWM initializing at %" PRIu32 " Hz.",
           engine->pwm_freq_hz);
  ESP_LOGI(TAG, "Hardware Resolution: %" PRIu32 " steps.",
           engine->period_cycles);
  if (engine->period_cycles < 1024U) {
    ESP_LOGW(TAG, "PWM resolution is only %" PRIu32 " steps.",
             engine->period_cycles);
  }

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

  const mcpwm_operator_config_t operator_config = {.group_id = 0};
  err = mcpwm_new_operator(&operator_config, &engine->oper);
  if (err != ESP_OK ||
      mcpwm_operator_connect_timer(engine->oper, engine->timer) != ESP_OK) {
    return -1;
  }

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

  err = mcpwm_timer_enable(engine->timer);
  if (err == ESP_OK) {
    err = mcpwm_timer_start_stop(engine->timer, MCPWM_TIMER_START_NO_STOP);
  }
  if (err != ESP_OK) {
    return -1;
  }

#if CONFIG_ENGINE_THREAD_SAFE
  engine->mutex = xSemaphoreCreateMutexStatic(&engine->mutex_buffer);
  if (engine->mutex == NULL) {
    return -1;
  }
#endif

  err = engine_driver_coast(engine);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "A controllable Enable pin is required for COAST mode");
    return -1;
  }

  ESP_LOGI(TAG, "Engine Driver initialized in COAST mode.");
  return 0;
}

esp_err_t engine_driver_coast(struct engine_config *engine) {
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif
  esp_err_t err = coast_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}

esp_err_t engine_driver_brake(struct engine_config *engine) {
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif
  esp_err_t err = brake_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}

esp_err_t engine_driver_set_speed(struct engine_config *engine, float command) {
  if (engine == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (command > 100.0f) {
    command = 100.0f;
  } else if (command < -100.0f) {
    command = -100.0f;
  }

#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

  if (command == 0.0f) {
    esp_err_t err = coast_locked(engine);
#if CONFIG_ENGINE_THREAD_SAFE
    xSemaphoreGive(engine->mutex);
#endif
    return err;
  }

  int direction = command > 0.0f ? 1 : -1;
  float duty_percent = command > 0.0f ? command : -command;
  uint32_t pulse_cycles =
      (uint32_t)(duty_percent * (float)engine->period_cycles / 100.0f);
  if (pulse_cycles >= engine->period_cycles) {
    pulse_cycles = engine->period_cycles - 1U;
  }

  esp_err_t err = ESP_OK;
  if (engine->mode == ENGINE_DRIVER_MODE_DRIVE &&
      direction != engine->last_direction) {
    err = coast_locked(engine);
    if (err == ESP_OK) {
      esp_rom_delay_us(ENGINE_DIRECTION_DEAD_TIME_US);
    }
  }

  mcpwm_gen_handle_t active_generator =
      direction > 0 ? engine->gen_fwd : engine->gen_rev;
  mcpwm_gen_handle_t inactive_generator =
      direction > 0 ? engine->gen_rev : engine->gen_fwd;
  mcpwm_cmpr_handle_t active_comparator =
      direction > 0 ? engine->cmp_fwd : engine->cmp_rev;
  bool entering_drive = engine->mode != ENGINE_DRIVER_MODE_DRIVE ||
                        engine->last_direction != direction;

  if (err == ESP_OK && entering_drive) {
    err = force_low(inactive_generator);
  }
  if (err == ESP_OK) {
    err = mcpwm_comparator_set_compare_value(active_comparator, pulse_cycles);
  }
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
  if (err == ESP_OK) {
    engine->last_direction = direction;
    engine->mode = ENGINE_DRIVER_MODE_DRIVE;
  } else {
    (void)coast_locked(engine);
  }

#if CONFIG_ENGINE_THREAD_SAFE
  xSemaphoreGive(engine->mutex);
#endif
  return err;
}
