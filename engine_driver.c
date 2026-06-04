#include "engine_driver.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "ENGINE_DRIVER";

int engine_driver_init(struct engine_config *engine) {
    if (engine->pin_fwd < 0 || engine->pin_rev < 0) {
        ESP_LOGE(TAG, "Invalid PWM pins configured: FWD=%d, REV=%d", engine->pin_fwd, engine->pin_rev);
        return -1;
    }

    esp_err_t err;

    // 1. Initialize H-Bridge Enable pin (active high)
    if (engine->pin_enable >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << engine->pin_enable),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure Enable pin: %s", esp_err_to_name(err));
            return -1;
        }
        // Initially keep output disabled
        gpio_set_level(engine->pin_enable, 0);
    }

    // 2. Calculate hardware period cycles using 80 MHz timer clock resolution
    uint32_t mcpwm_timer_clock_hz = 80000000; // 80 MHz
    uint32_t period_ticks = mcpwm_timer_clock_hz / engine->pwm_freq_hz;
    engine->period_cycles = period_ticks;
    engine->last_direction = 0;

    // Report configuration and check hardware resolution
    ESP_LOGI(TAG, "Dual MCPWM initializing at %lu Hz.", engine->pwm_freq_hz);
    ESP_LOGI(TAG, "Hardware Resolution: %lu steps.", engine->period_cycles);

    if (engine->period_cycles < 1024) {
        ESP_LOGW(TAG, "WARNING: PWM frequency is high or timer clock is low.");
        ESP_LOGW(TAG, "         Resolution is only %lu steps (below recommended 1024 / 10 bits).",
                 engine->period_cycles);
    }

    // 3. Initialize MCPWM Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M, // PLL 160MHz as group clock source
        .resolution_hz = mcpwm_timer_clock_hz,  // 80 MHz maximum timer clock resolution
        .period_ticks = period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    err = mcpwm_new_timer(&timer_config, &engine->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(err));
        return -1;
    }

    // 4. Initialize MCPWM Operator
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    err = mcpwm_new_operator(&operator_config, &engine->oper);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(err));
        return -1;
    }

    // 5. Connect Operator to Timer
    err = mcpwm_operator_connect_timer(engine->oper, engine->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect operator to timer: %s", esp_err_to_name(err));
        return -1;
    }

    // 6. Create Comparators
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };
    err = mcpwm_new_comparator(engine->oper, &compare_config, &engine->cmp_fwd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create FWD comparator: %s", esp_err_to_name(err));
        return -1;
    }
    err = mcpwm_new_comparator(engine->oper, &compare_config, &engine->cmp_rev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create REV comparator: %s", esp_err_to_name(err));
        return -1;
    }

    // Initialize compare values to 0 (duty cycle 0%)
    mcpwm_comparator_set_compare_value(engine->cmp_fwd, 0);
    mcpwm_comparator_set_compare_value(engine->cmp_rev, 0);

    // 7. Create Generators
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = engine->pin_fwd,
    };
    err = mcpwm_new_generator(engine->oper, &generator_config, &engine->gen_fwd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create FWD generator: %s", esp_err_to_name(err));
        return -1;
    }

    generator_config.gen_gpio_num = engine->pin_rev;
    err = mcpwm_new_generator(engine->oper, &generator_config, &engine->gen_rev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create REV generator: %s", esp_err_to_name(err));
        return -1;
    }

    // 8. Set generator actions to output PWM
    // FWD (RPWM): High on timer empty (0), Low on compare A (cmp_fwd) match
    err = mcpwm_generator_set_action_on_timer_event(engine->gen_fwd,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return -1;
    err = mcpwm_generator_set_action_on_compare_event(engine->gen_fwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, engine->cmp_fwd, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) return -1;

    // REV (LPWM): High on timer empty (0), Low on compare B (cmp_rev) match
    err = mcpwm_generator_set_action_on_timer_event(engine->gen_rev,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return -1;
    err = mcpwm_generator_set_action_on_compare_event(engine->gen_rev,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, engine->cmp_rev, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) return -1;

    // 9. Enable and Start Timer
    err = mcpwm_timer_enable(engine->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable MCPWM timer: %s", esp_err_to_name(err));
        return -1;
    }
    err = mcpwm_timer_start_stop(engine->timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(err));
        return -1;
    }

    // 10. Thread Safety Setup
#if CONFIG_ENGINE_THREAD_SAFE
    engine->mutex = xSemaphoreCreateMutexStatic(&engine->mutex_buffer);
    if (engine->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create static mutex");
        return -1;
    }
#endif

    // Turn enable line HIGH to enable H-bridge driver outputs (active high)
    if (engine->pin_enable >= 0) {
        gpio_set_level(engine->pin_enable, 1);
    }

    // Initial state: Electronic Brake (Both PWM channels = 0%, Enable = HIGH)
    engine_driver_set_speed(engine, 0.0f);

    ESP_LOGI(TAG, "Engine Driver successfully initialized.");
    return 0;
}

void engine_driver_set_speed(struct engine_config *engine, float command) {
#if CONFIG_ENGINE_THREAD_SAFE
    xSemaphoreTake(engine->mutex, portMAX_DELAY);
#endif

    // Clamp command to [-100.0, 100.0]
    if (command > 100.0f)  command = 100.0f;
    if (command < -100.0f) command = -100.0f;

    int direction = 0;
    float duty_percent = 0.0f;

    if (command > 0.0f) {
        direction = 1;
        duty_percent = command;
    } else if (command < 0.0f) {
        direction = -1;
        duty_percent = -command;
    }

    // Software Dead-time on direction change to prevent shoot-through
    if (direction != engine->last_direction && engine->last_direction != 0) {
        // Safe transition: Disable both outputs (PWM = 0)
        mcpwm_comparator_set_compare_value(engine->cmp_fwd, 0);
        mcpwm_comparator_set_compare_value(engine->cmp_rev, 0);
        
        // Delay of ~50 us
        esp_rom_delay_us(50);
    }

    // Map duty percentage directly to hardware timer clock cycles
    uint32_t pulse_cycles = (uint32_t)((duty_percent * (float)engine->period_cycles) / 100.0f);

    if (direction == 1) {
        // Forward: RPWM (gen_fwd) active, LPWM (gen_rev) = 0
        mcpwm_comparator_set_compare_value(engine->cmp_rev, 0);
        mcpwm_comparator_set_compare_value(engine->cmp_fwd, pulse_cycles);
    } 
    else if (direction == -1) {
        // Reverse: LPWM (gen_rev) active, RPWM (gen_fwd) = 0
        mcpwm_comparator_set_compare_value(engine->cmp_fwd, 0);
        mcpwm_comparator_set_compare_value(engine->cmp_rev, pulse_cycles);
    } 
    else {
        // Active Slow Decay Brake: RPWM = 0, LPWM = 0 (and Enable is HIGH)
        mcpwm_comparator_set_compare_value(engine->cmp_fwd, 0);
        mcpwm_comparator_set_compare_value(engine->cmp_rev, 0);
    }

    engine->last_direction = direction;

#if CONFIG_ENGINE_THREAD_SAFE
    xSemaphoreGive(engine->mutex);
#endif
}
