#include "sdkconfig.h"

#if CONFIG_ENGINE_CURRENT_SENSE_ENABLE

#include "engine_current_sense.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include <stdatomic.h>

#define CURRENT_SENSE_CORE_ID 0
#define CURRENT_SENSE_TASK_PRIORITY 2
#define CURRENT_SENSE_TASK_STACK_SIZE 4096U
#define CURRENT_SENSE_FRAME_RATE_HZ 1000U
#define CURRENT_SENSE_SAMPLES_PER_FRAME                                        \
  (CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ / CURRENT_SENSE_FRAME_RATE_HZ)
#define CURRENT_SENSE_FRAME_BYTES                                              \
  (CURRENT_SENSE_SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES)
#define CURRENT_SENSE_STORE_BYTES (CURRENT_SENSE_FRAME_BYTES * 4U)

_Static_assert(CURRENT_SENSE_SAMPLES_PER_FRAME > 0,
               "ADC frame must contain at least one sample");
_Static_assert(CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ %
                       CURRENT_SENSE_FRAME_RATE_HZ ==
                   0,
               "ADC sample rate must produce an integer number of samples per "
               "millisecond");

typedef struct {
  uint64_t raw_sum;
  uint64_t normal_raw_sum;
  uint32_t samples;
  uint32_t normal_samples;
  uint32_t fault_samples;
  uint32_t fault_entries;
  uint32_t raw_minimum;
  uint32_t raw_maximum;
  bool fault_active;
  uint32_t invalid_results;
  uint32_t read_errors;
} current_sense_accumulator_t;

static const char *TAG = "BTS7960_IS";
static adc_continuous_handle_t adc_handle;
static adc_cali_handle_t calibration_handle;
static adc_unit_t adc_unit;
static adc_channel_t adc_channel;
static TaskHandle_t current_sense_task_handle;
static portMUX_TYPE data_lock = portMUX_INITIALIZER_UNLOCKED;
static current_sense_accumulator_t accumulator;
static engine_current_sense_frame_t latest_frame;
static atomic_uint pool_overflows;
static uint32_t fault_enter_raw;
static uint32_t fault_exit_raw;
static bool fault_active;

static void reset_accumulator(void) {
  accumulator = (current_sense_accumulator_t){
      .raw_minimum = UINT32_MAX,
  };
}

static bool IRAM_ATTR conversion_done_callback(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *event,
    void *user_data) {
  (void)handle;
  (void)event;
  (void)user_data;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(current_sense_task_handle,
                         &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

static bool IRAM_ATTR pool_overflow_callback(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *event,
    void *user_data) {
  (void)handle;
  (void)event;
  (void)user_data;
  atomic_fetch_add_explicit(&pool_overflows, 1U, memory_order_relaxed);
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(current_sense_task_handle,
                         &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

static void sort_raw_values(uint32_t *values, uint32_t count) {
  for (uint32_t index = 1; index < count; index++) {
    uint32_t value = values[index];
    uint32_t insert_at = index;
    while (insert_at > 0U && values[insert_at - 1U] > value) {
      values[insert_at] = values[insert_at - 1U];
      insert_at--;
    }
    values[insert_at] = value;
  }
}

static uint32_t raw_median(uint32_t *values, uint32_t count) {
  sort_raw_values(values, count);
  if ((count & 1U) != 0U) {
    return values[count / 2U];
  }
  uint32_t upper = values[count / 2U];
  uint32_t lower = values[count / 2U - 1U];
  return lower + (upper - lower) / 2U;
}

static esp_err_t calibrated_raw_threshold(int target_mv,
                                          uint32_t *raw_threshold) {
  if (calibration_handle == NULL || raw_threshold == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t lower = 0U;
  uint32_t upper = (1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U;
  int upper_mv = 0;
  esp_err_t err =
      adc_cali_raw_to_voltage(calibration_handle, (int)upper, &upper_mv);
  if (err != ESP_OK || upper_mv < target_mv) {
    return ESP_ERR_INVALID_ARG;
  }

  while (lower < upper) {
    uint32_t middle = lower + (upper - lower) / 2U;
    int middle_mv = 0;
    err = adc_cali_raw_to_voltage(calibration_handle, (int)middle, &middle_mv);
    if (err != ESP_OK) {
      return err;
    }
    if (middle_mv < target_mv) {
      lower = middle + 1U;
    } else {
      upper = middle;
    }
  }
  *raw_threshold = lower;
  return ESP_OK;
}

static void accumulate_frame(const adc_continuous_data_t *samples,
                             uint32_t sample_count) {
  uint32_t raw_values[CURRENT_SENSE_SAMPLES_PER_FRAME];
  uint64_t raw_sum = 0;
  uint64_t normal_raw_sum = 0;
  uint32_t valid_count = 0;
  uint32_t normal_count = 0;
  uint32_t frame_fault_samples = 0;
  uint32_t frame_fault_entries = 0;
  uint32_t raw_minimum = UINT32_MAX;
  uint32_t raw_maximum = 0;
  uint32_t invalid_results = 0;

  for (uint32_t index = 0; index < sample_count; index++) {
    const adc_continuous_data_t *sample = &samples[index];
    if (!sample->valid || sample->unit != adc_unit ||
        sample->channel != adc_channel ||
        valid_count >= CURRENT_SENSE_SAMPLES_PER_FRAME) {
      invalid_results++;
      continue;
    }

    raw_values[valid_count++] = sample->raw_data;
    raw_sum += sample->raw_data;

    if (!fault_active && sample->raw_data >= fault_enter_raw) {
      fault_active = true;
      frame_fault_entries++;
    } else if (fault_active && sample->raw_data <= fault_exit_raw) {
      fault_active = false;
    }
    if (fault_active) {
      frame_fault_samples++;
    } else {
      normal_raw_sum += sample->raw_data;
      normal_count++;
    }
    if (sample->raw_data < raw_minimum) {
      raw_minimum = sample->raw_data;
    }
    if (sample->raw_data > raw_maximum) {
      raw_maximum = sample->raw_data;
    }
  }

  uint32_t median = valid_count > 0U ? raw_median(raw_values, valid_count) : 0U;

  portENTER_CRITICAL(&data_lock);
  accumulator.raw_sum += raw_sum;
  accumulator.normal_raw_sum += normal_raw_sum;
  accumulator.samples += valid_count;
  accumulator.normal_samples += normal_count;
  accumulator.fault_samples += frame_fault_samples;
  accumulator.fault_entries += frame_fault_entries;
  accumulator.fault_active = fault_active;
  accumulator.invalid_results += invalid_results;
  if (valid_count > 0U) {
    if (raw_minimum < accumulator.raw_minimum) {
      accumulator.raw_minimum = raw_minimum;
    }
    if (raw_maximum > accumulator.raw_maximum) {
      accumulator.raw_maximum = raw_maximum;
    }
  }
  latest_frame = (engine_current_sense_frame_t){
      .sequence = latest_frame.sequence + 1U,
      .samples = valid_count,
      .raw_average = valid_count > 0U ? (uint32_t)(raw_sum / valid_count) : 0U,
      .raw_median = median,
      .raw_minimum = valid_count > 0U ? raw_minimum : 0U,
      .raw_maximum = valid_count > 0U ? raw_maximum : 0U,
      .normal_samples = normal_count,
      .normal_raw_average =
          normal_count > 0U ? (uint32_t)(normal_raw_sum / normal_count) : 0U,
      .fault_samples = frame_fault_samples,
      .fault_entries = frame_fault_entries,
      .fault_active = fault_active,
      .invalid_results = invalid_results,
  };
  portEXIT_CRITICAL(&data_lock);
}

static void current_sense_task(void *argument) {
  (void)argument;
  uint8_t raw_frame[CURRENT_SENSE_FRAME_BYTES];
  adc_continuous_data_t parsed[CURRENT_SENSE_SAMPLES_PER_FRAME];

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (true) {
      uint32_t bytes_read = 0;
      esp_err_t err = adc_continuous_read(adc_handle, raw_frame,
                                          sizeof(raw_frame), &bytes_read, 0);
      if (err == ESP_ERR_TIMEOUT) {
        break;
      }
      if (err != ESP_OK) {
        portENTER_CRITICAL(&data_lock);
        accumulator.read_errors++;
        portEXIT_CRITICAL(&data_lock);
        break;
      }

      uint32_t parsed_count = 0;
      err = adc_continuous_parse_data(adc_handle, raw_frame, bytes_read, parsed,
                                      &parsed_count);
      if (err != ESP_OK) {
        portENTER_CRITICAL(&data_lock);
        accumulator.read_errors++;
        portEXIT_CRITICAL(&data_lock);
        continue;
      }
      accumulate_frame(parsed, parsed_count);
    }
  }
}

static void cleanup_resources(void) {
  if (adc_handle != NULL) {
    adc_continuous_stop(adc_handle);
  }
  if (current_sense_task_handle != NULL) {
    vTaskDelete(current_sense_task_handle);
    current_sense_task_handle = NULL;
  }
  if (calibration_handle != NULL) {
    adc_cali_delete_scheme_curve_fitting(calibration_handle);
    calibration_handle = NULL;
  }
  if (adc_handle != NULL) {
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
  }
}

esp_err_t engine_current_sense_start(void) {
  if (adc_handle != NULL || current_sense_task_handle != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = adc_continuous_io_to_channel(
      CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS, &adc_unit, &adc_channel);
  if (err != ESP_OK || adc_unit != ADC_UNIT_1) {
    return ESP_ERR_INVALID_ARG;
  }

  adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size = CURRENT_SENSE_STORE_BYTES,
      .conv_frame_size = CURRENT_SENSE_FRAME_BYTES,
      .flags.flush_pool = true,
  };
  err = adc_continuous_new_handle(&handle_config, &adc_handle);
  if (err != ESP_OK) {
    return err;
  }

  adc_digi_pattern_config_t pattern = {
      .atten = ADC_ATTEN_DB_12,
      .channel = adc_channel,
      .unit = adc_unit,
      .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
  };
  adc_continuous_config_t continuous_config = {
      .pattern_num = 1,
      .adc_pattern = &pattern,
      .sample_freq_hz = CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
  };
  err = adc_continuous_config(adc_handle, &continuous_config);
  if (err != ESP_OK) {
    cleanup_resources();
    return err;
  }

  adc_cali_curve_fitting_config_t calibration_config = {
      .unit_id = adc_unit,
      .chan = adc_channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
  };
  err = adc_cali_create_scheme_curve_fitting(&calibration_config,
                                             &calibration_handle);
  if (err != ESP_OK) {
    cleanup_resources();
    return err;
  }

  if (CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV >=
      CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV) {
    cleanup_resources();
    return ESP_ERR_INVALID_ARG;
  }
  err = calibrated_raw_threshold(CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV,
                                 &fault_enter_raw);
  if (err == ESP_OK) {
    err = calibrated_raw_threshold(CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV,
                                   &fault_exit_raw);
  }
  if (err != ESP_OK) {
    cleanup_resources();
    return err;
  }

  reset_accumulator();
  latest_frame = (engine_current_sense_frame_t){0};
  fault_active = false;
  atomic_store_explicit(&pool_overflows, 0U, memory_order_relaxed);
  BaseType_t task_created = xTaskCreatePinnedToCore(
      current_sense_task, "r_is_adc", CURRENT_SENSE_TASK_STACK_SIZE, NULL,
      CURRENT_SENSE_TASK_PRIORITY, &current_sense_task_handle,
      CURRENT_SENSE_CORE_ID);
  if (task_created != pdPASS) {
    cleanup_resources();
    return ESP_ERR_NO_MEM;
  }

  const adc_continuous_evt_cbs_t callbacks = {
      .on_conv_done = conversion_done_callback,
      .on_pool_ovf = pool_overflow_callback,
  };
  err = adc_continuous_register_event_callbacks(adc_handle, &callbacks, NULL);
  if (err == ESP_OK) {
    err = adc_continuous_start(adc_handle);
  }
  if (err != ESP_OK) {
    cleanup_resources();
    return err;
  }

  ESP_LOGI(TAG,
           "R_IS on GPIO %d: ADC1 channel %d at %d samples/s, fault "
           "hysteresis=%d/%d mV",
           CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS, adc_channel,
           CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ,
           CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV,
           CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV);
  return ESP_OK;
}

void engine_current_sense_stop(void) { cleanup_resources(); }

bool engine_current_sense_take_snapshot(
    engine_current_sense_snapshot_t *snapshot) {
  if (snapshot == NULL || adc_handle == NULL) {
    return false;
  }

  current_sense_accumulator_t captured;
  portENTER_CRITICAL(&data_lock);
  captured = accumulator;
  reset_accumulator();
  portEXIT_CRITICAL(&data_lock);

  *snapshot = (engine_current_sense_snapshot_t){
      .samples = captured.samples,
      .raw_average = captured.samples > 0U
                         ? (uint32_t)(captured.raw_sum / captured.samples)
                         : 0U,
      .raw_minimum = captured.samples > 0U ? captured.raw_minimum : 0U,
      .raw_maximum = captured.samples > 0U ? captured.raw_maximum : 0U,
      .normal_samples = captured.normal_samples,
      .normal_raw_average =
          captured.normal_samples > 0U
              ? (uint32_t)(captured.normal_raw_sum / captured.normal_samples)
              : 0U,
      .fault_samples = captured.fault_samples,
      .fault_entries = captured.fault_entries,
      .fault_active = captured.fault_active,
      .invalid_results = captured.invalid_results,
      .pool_overflows =
          atomic_exchange_explicit(&pool_overflows, 0U, memory_order_relaxed),
      .read_errors = captured.read_errors,
  };
  return captured.samples > 0U;
}

bool engine_current_sense_get_latest_frame(
    engine_current_sense_frame_t *frame) {
  if (frame == NULL || adc_handle == NULL) {
    return false;
  }

  portENTER_CRITICAL(&data_lock);
  *frame = latest_frame;
  portEXIT_CRITICAL(&data_lock);
  return frame->sequence != 0U && frame->samples > 0U;
}

esp_err_t engine_current_sense_raw_to_millivolts(uint32_t raw,
                                                 int *millivolts) {
  if (calibration_handle == NULL || millivolts == NULL || raw > INT32_MAX) {
    return ESP_ERR_INVALID_STATE;
  }
  return adc_cali_raw_to_voltage(calibration_handle, (int)raw, millivolts);
}

float engine_current_sense_adc_to_r_is_millivolts(int adc_millivolts) {
  return (float)adc_millivolts *
         ((float)CONFIG_ENGINE_CURRENT_SENSE_SERIES_RESISTOR_OHMS +
          (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS) /
         (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS;
}

static float adc_transimpedance_ohms(void) {
  const float board_resistance =
      (float)CONFIG_ENGINE_CURRENT_SENSE_BOARD_RESISTOR_OHMS;
  const float divider_resistance =
      (float)CONFIG_ENGINE_CURRENT_SENSE_SERIES_RESISTOR_OHMS +
      (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS;
  const float parallel_resistance = (board_resistance * divider_resistance) /
                                    (board_resistance + divider_resistance);
  const float adc_transimpedance =
      parallel_resistance *
      (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS /
      divider_resistance;
  return adc_transimpedance;
}

float engine_current_sense_adc_to_i_is_milliamperes(int adc_millivolts) {
  return (float)adc_millivolts / adc_transimpedance_ohms();
}

float engine_current_sense_adc_to_amperes(int adc_millivolts) {
  return engine_current_sense_adc_to_i_is_milliamperes(adc_millivolts) *
         (float)CONFIG_ENGINE_CURRENT_SENSE_RATIO / 1000.0f;
}

#endif /* CONFIG_ENGINE_CURRENT_SENSE_ENABLE */
