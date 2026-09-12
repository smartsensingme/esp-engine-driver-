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
#define CURRENT_SENSE_SAMPLES_PER_CHANNEL_FRAME                                \
  (CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ / CURRENT_SENSE_FRAME_RATE_HZ)
#define CURRENT_SENSE_TOTAL_SAMPLES_PER_FRAME                                  \
  (CURRENT_SENSE_SAMPLES_PER_CHANNEL_FRAME * ENGINE_CURRENT_SENSE_CHANNEL_COUNT)
#define CURRENT_SENSE_TOTAL_SAMPLE_HZ                                          \
  (CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ * ENGINE_CURRENT_SENSE_CHANNEL_COUNT)
#define CURRENT_SENSE_FRAME_BYTES                                              \
  (CURRENT_SENSE_TOTAL_SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES)
#define CURRENT_SENSE_STORE_BYTES (CURRENT_SENSE_FRAME_BYTES * 4U)

_Static_assert(CURRENT_SENSE_SAMPLES_PER_CHANNEL_FRAME > 0,
               "ADC frame must contain at least one sample per channel");
_Static_assert(CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ %
                       CURRENT_SENSE_FRAME_RATE_HZ ==
                   0,
               "ADC sample rate must produce an integer number of samples "
               "per channel and millisecond");

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
static const int sense_gpio[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {
    [ENGINE_CURRENT_SENSE_CHANNEL_R_IS] = CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS,
    [ENGINE_CURRENT_SENSE_CHANNEL_L_IS] = CONFIG_ENGINE_CURRENT_SENSE_GPIO_L_IS,
};
static adc_continuous_handle_t adc_handle;
static adc_cali_handle_t calibration_handle[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static adc_unit_t adc_unit;
static adc_channel_t adc_channel[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static TaskHandle_t current_sense_task_handle;
static portMUX_TYPE data_lock = portMUX_INITIALIZER_UNLOCKED;
static current_sense_accumulator_t
    accumulator[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static engine_current_sense_frame_t
    latest_frame[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static engine_current_sense_measurement_t latest_measurement;
static atomic_uint pool_overflows;
static uint32_t fault_enter_raw[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static uint32_t fault_exit_raw[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
static bool fault_active[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
/* One task owns these buffers; static storage keeps them off its stack. */
static uint8_t raw_frame[CURRENT_SENSE_FRAME_BYTES];
static adc_continuous_data_t
    parsed_frame[CURRENT_SENSE_TOTAL_SAMPLES_PER_FRAME];

const char *
engine_current_sense_channel_name(engine_current_sense_channel_t channel) {
  /* Mapping block: keep labels stable because telemetry uses them in logs. */
  switch (channel) {
  case ENGINE_CURRENT_SENSE_CHANNEL_R_IS:
    return "R_IS";
  case ENGINE_CURRENT_SENSE_CHANNEL_L_IS:
    return "L_IS";
  default:
    return "UNKNOWN_IS";
  }
}

/**
 * @brief Clear both long-window accumulators to their empty sentinel state.
 *
 * Called by engine_current_sense_start() and
 * engine_current_sense_take_dual_snapshot(). It is private and calls no other
 * component function.
 */
static void reset_accumulators(void) {
  /* Reset block: UINT32_MAX lets the first valid sample replace the minimum. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    accumulator[channel] = (current_sense_accumulator_t){
        .raw_minimum = UINT32_MAX,
    };
  }
}

/**
 * @brief Wake the worker when an ADC conversion frame becomes available.
 *
 * Registered by engine_current_sense_start() and called by the ADC driver in
 * ISR context. It notifies current_sense_task() without parsing data in the
 * ISR.
 */
static bool IRAM_ATTR conversion_done_callback(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *event,
    void *user_data) {
  /* Callback metadata is not needed; one global acquisition instance exists. */
  (void)handle;
  (void)event;
  (void)user_data;
  BaseType_t higher_priority_task_woken = pdFALSE;

  /* Notification block: unblock the Core 0 worker and request a yield when a
   * higher-priority task was awakened. */
  vTaskNotifyGiveFromISR(current_sense_task_handle,
                         &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

/**
 * @brief Count a DMA pool overflow and wake the draining worker.
 *
 * Registered by engine_current_sense_start() and called by the ADC driver in
 * ISR context. It publishes an atomic diagnostic counter and notifies
 * current_sense_task().
 */
static bool IRAM_ATTR pool_overflow_callback(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *event,
    void *user_data) {
  /* Callback metadata is unused by the single configured acquisition. */
  (void)handle;
  (void)event;
  (void)user_data;
  atomic_fetch_add_explicit(&pool_overflows, 1U, memory_order_relaxed);

  /* Wake the worker so it drains any data still available after overflow. */
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(current_sense_task_handle,
                         &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

/**
 * @brief Sort a short frame of raw ADC values in ascending order.
 *
 * Called only by raw_median(). Insertion sort is used because each channel has
 * a small, fixed one-millisecond sample set and no allocation is required.
 */
static void sort_raw_values(uint32_t *values, uint32_t count) {
  /* Insertion block: grow a sorted prefix by shifting larger values right. */
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

/**
 * @brief Calculate the integer median of a nonempty raw ADC frame.
 *
 * Called only by accumulate_frame(). It calls sort_raw_values() in place and
 * averages the central pair for an even sample count.
 */
static uint32_t raw_median(uint32_t *values, uint32_t count) {
  /* Ordering block: the caller provides writable frame-local storage. */
  sort_raw_values(values, count);

  /* Selection block: return one center or the overflow-safe midpoint of two. */
  if ((count & 1U) != 0U) {
    return values[count / 2U];
  }
  uint32_t upper = values[count / 2U];
  uint32_t lower = values[count / 2U - 1U];
  return lower + (upper - lower) / 2U;
}

/**
 * @brief Translate an ADC hardware channel number to R_IS or L_IS index.
 *
 * Called only by accumulate_frame(). It scans the two channels configured by
 * engine_current_sense_start() and returns -1 for an unrelated result.
 */
static int sense_channel_index(adc_channel_t channel) {
  /* Lookup block: compare against the resolved ADC channel table. */
  for (size_t index = 0; index < ENGINE_CURRENT_SENSE_CHANNEL_COUNT; index++) {
    if (adc_channel[index] == channel) {
      return (int)index;
    }
  }
  return -1;
}

/**
 * @brief Find the first raw ADC code reaching a calibrated millivolt threshold.
 *
 * Called by engine_current_sense_start() for the enter/exit threshold of both
 * channels. It performs a binary search through the channel-specific
 * curve-fitting calibration function.
 */
static esp_err_t
calibrated_raw_threshold(engine_current_sense_channel_t channel, int target_mv,
                         uint32_t *raw_threshold) {
  /* Interface block: require a valid channel, calibration handle, and output.
   */
  if (channel >= ENGINE_CURRENT_SENSE_CHANNEL_COUNT ||
      calibration_handle[channel] == NULL || raw_threshold == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Range block: verify that the calibrated ADC full scale reaches the target.
   */
  uint32_t lower = 0U;
  uint32_t upper = (1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U;
  int upper_mv = 0;
  esp_err_t err = adc_cali_raw_to_voltage(calibration_handle[channel],
                                          (int)upper, &upper_mv);
  if (err != ESP_OK || upper_mv < target_mv) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Search block: calibration is monotonic, so bisect until lower is the first
   * raw code whose calibrated voltage is at least target_mv. */
  while (lower < upper) {
    uint32_t middle = lower + (upper - lower) / 2U;
    int middle_mv = 0;
    err = adc_cali_raw_to_voltage(calibration_handle[channel], (int)middle,
                                  &middle_mv);
    if (err != ESP_OK) {
      return err;
    }
    if (middle_mv < target_mv) {
      lower = middle + 1U;
    } else {
      upper = middle;
    }
  }

  /* Publication block: expose the raw-domain threshold used in every frame. */
  *raw_threshold = lower;
  return ESP_OK;
}

/**
 * @brief Classify one parsed DMA frame and publish raw per-channel statistics.
 *
 * Called only by current_sense_task(). It calls sense_channel_index() and
 * raw_median(), updates hysteretic fault state, accumulates the long diagnostic
 * window, and publishes coherent latest-frame records under data_lock.
 */
static void accumulate_frame(
    const adc_continuous_data_t *samples, uint32_t sample_count,
    uint32_t normal_raw_average[ENGINE_CURRENT_SENSE_CHANNEL_COUNT],
    bool normal_valid[ENGINE_CURRENT_SENSE_CHANNEL_COUNT],
    uint32_t *frame_fault_mask) {
  /* Frame-storage block: allocate independent fixed-capacity statistics for
   * R_IS and L_IS on the worker-task stack. */
  uint32_t raw_values[ENGINE_CURRENT_SENSE_CHANNEL_COUNT]
                     [CURRENT_SENSE_SAMPLES_PER_CHANNEL_FRAME];
  uint64_t raw_sum[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint64_t normal_raw_sum[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t valid_count[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t normal_count[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t frame_fault_samples[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t frame_fault_entries[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t raw_minimum[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
  uint32_t raw_maximum[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t median[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};
  uint32_t invalid_results[ENGINE_CURRENT_SENSE_CHANNEL_COUNT] = {0};

  /* Initialization block: establish per-channel empty-frame sentinels and clear
   * output validity before parsing any DMA result. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    raw_minimum[channel] = UINT32_MAX;
    normal_valid[channel] = false;
    normal_raw_average[channel] = 0U;
  }
  *frame_fault_mask = 0U;

  /* Parsing block: route each valid ADC1 result to its logical sense channel,
   * enforcing the configured per-frame capacity. */
  for (uint32_t index = 0; index < sample_count; index++) {
    const adc_continuous_data_t *sample = &samples[index];
    int channel = sample->valid && sample->unit == adc_unit
                      ? sense_channel_index(sample->channel)
                      : -1;
    if (channel < 0) {
      invalid_results[ENGINE_CURRENT_SENSE_CHANNEL_R_IS]++;
      continue;
    }
    if (valid_count[channel] >= CURRENT_SENSE_SAMPLES_PER_CHANNEL_FRAME) {
      invalid_results[channel]++;
      continue;
    }

    /* Raw-statistics block: store samples for the median and update sum. */
    uint32_t raw = sample->raw_data;
    raw_values[channel][valid_count[channel]++] = raw;
    raw_sum[channel] += raw;

    /* Fault-classification block: apply separate entry/exit thresholds to avoid
     * rapid state changes when I_IS voltage lies near the boundary. */
    if (!fault_active[channel] && raw >= fault_enter_raw[channel]) {
      fault_active[channel] = true;
      frame_fault_entries[channel]++;
    } else if (fault_active[channel] && raw <= fault_exit_raw[channel]) {
      fault_active[channel] = false;
    }
    if (fault_active[channel]) {
      frame_fault_samples[channel]++;
    } else {
      normal_raw_sum[channel] += raw;
      normal_count[channel]++;
    }

    /* Extrema block: retain the complete valid range, including fault samples.
     */
    if (raw < raw_minimum[channel]) {
      raw_minimum[channel] = raw;
    }
    if (raw > raw_maximum[channel]) {
      raw_maximum[channel] = raw;
    }
  }

  /* Median block: calculate only for channels represented in this frame. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    if (valid_count[channel] > 0U) {
      median[channel] = raw_median(raw_values[channel], valid_count[channel]);
    }
  }

  /* Publication block: update long-window accumulators and both latest-frame
   * records in one critical section so readers cannot mix frame sequences. */
  portENTER_CRITICAL(&data_lock);
  uint32_t next_sequence = latest_frame[0].sequence + 1U;
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    current_sense_accumulator_t *total = &accumulator[channel];
    total->raw_sum += raw_sum[channel];
    total->normal_raw_sum += normal_raw_sum[channel];
    total->samples += valid_count[channel];
    total->normal_samples += normal_count[channel];
    total->fault_samples += frame_fault_samples[channel];
    total->fault_entries += frame_fault_entries[channel];
    total->fault_active = fault_active[channel];
    total->invalid_results += invalid_results[channel];
    if (valid_count[channel] > 0U) {
      if (raw_minimum[channel] < total->raw_minimum) {
        total->raw_minimum = raw_minimum[channel];
      }
      if (raw_maximum[channel] > total->raw_maximum) {
        total->raw_maximum = raw_maximum[channel];
      }
    }

    latest_frame[channel] = (engine_current_sense_frame_t){
        .sequence = next_sequence,
        .samples = valid_count[channel],
        .raw_average = valid_count[channel] > 0U
                           ? (uint32_t)(raw_sum[channel] / valid_count[channel])
                           : 0U,
        .raw_median = median[channel],
        .raw_minimum = valid_count[channel] > 0U ? raw_minimum[channel] : 0U,
        .raw_maximum = valid_count[channel] > 0U ? raw_maximum[channel] : 0U,
        .normal_samples = normal_count[channel],
        .normal_raw_average =
            normal_count[channel] > 0U
                ? (uint32_t)(normal_raw_sum[channel] / normal_count[channel])
                : 0U,
        .fault_samples = frame_fault_samples[channel],
        .fault_entries = frame_fault_entries[channel],
        .fault_active = fault_active[channel],
        .invalid_results = invalid_results[channel],
    };

    /* Conversion-input block: export only normal-sample means for later current
     * conversion; fault-current signaling must never be interpreted as load. */
    if (normal_count[channel] > 0U) {
      normal_raw_average[channel] =
          (uint32_t)(normal_raw_sum[channel] / normal_count[channel]);
      normal_valid[channel] = true;
    }

    /* Fault-mask block: mark a channel if fault occurred or remains active. */
    if (frame_fault_samples[channel] > 0U || fault_active[channel]) {
      *frame_fault_mask |= ENGINE_CURRENT_SENSE_CHANNEL_BIT(channel);
    }
  }
  portEXIT_CRITICAL(&data_lock);
}

/**
 * @brief Round a floating-point current in amperes to signed milliamperes.
 *
 * Called by publish_current_measurement() for both channel magnitudes. It is
 * private and rounds halves away from zero.
 */
static int32_t rounded_milliamps(float current_amperes) {
  /* Scaling/rounding block: add the sign-appropriate half before truncation. */
  float current_milliamps = current_amperes * 1000.0f;
  return current_milliamps >= 0.0f ? (int32_t)(current_milliamps + 0.5f)
                                   : (int32_t)(current_milliamps - 0.5f);
}

/**
 * @brief Calibrate normal channel means and publish current-domain results.
 *
 * Called only by current_sense_task(). It calls
 * engine_current_sense_adc_to_amperes() and rounded_milliamps(), publishes both
 * magnitudes coherently.
 */
static void publish_current_measurement(
    const uint32_t normal_raw_average[ENGINE_CURRENT_SENSE_CHANNEL_COUNT],
    const bool normal_valid[ENGINE_CURRENT_SENSE_CHANNEL_COUNT],
    uint32_t frame_fault_mask) {
  /* Result-initialization block: begin with no valid converted channels and
   * copy the frame's already classified fault mask. */
  engine_current_sense_measurement_t measurement = {
      .fault_mask = frame_fault_mask,
  };

  /* Calibration block: convert each normal raw mean with its own eFuse-derived
   * calibration handle, then apply the external-network/current-ratio model. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    if (!normal_valid[channel]) {
      continue;
    }
    int adc_millivolts = 0;
    if (adc_cali_raw_to_voltage(calibration_handle[channel],
                                (int)normal_raw_average[channel],
                                &adc_millivolts) != ESP_OK) {
      continue;
    }
    float magnitude = engine_current_sense_adc_to_amperes(adc_millivolts);
    measurement.channel_milliamps[channel] = rounded_milliamps(magnitude);
    measurement.valid_mask |= ENGINE_CURRENT_SENSE_CHANNEL_BIT(channel);
  }

  /* Dual-publication block: attach the latest raw-frame sequence and atomically
   * replace the coherent per-channel measurement under data_lock. */
  portENTER_CRITICAL(&data_lock);
  measurement.sequence = latest_frame[0].sequence;
  latest_measurement = measurement;
  portEXIT_CRITICAL(&data_lock);
}

/**
 * @brief Drain ADC DMA frames and run all non-ISR current processing.
 *
 * Created by engine_current_sense_start() and awakened by both ADC callbacks.
 * It calls accumulate_frame() and publish_current_measurement() for every
 * parsed frame and records read/parse failures in both long-window
 * accumulators.
 */
static void current_sense_task(void *argument) {
  /* The task uses component-global acquisition state and needs no argument. */
  (void)argument;

  /* Service loop: sleep without polling until an ISR signals available data. */
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Drain loop: consume every currently available DMA block before sleeping.
     */
    while (true) {
      uint32_t bytes_read = 0;
      esp_err_t err = adc_continuous_read(adc_handle, raw_frame,
                                          sizeof(raw_frame), &bytes_read, 0);
      if (err == ESP_ERR_TIMEOUT) {
        /* A zero-timeout read returning empty means the DMA pool is drained. */
        break;
      }
      if (err != ESP_OK) {
        /* Read-error block: report the acquisition failure for both channels.
         */
        portENTER_CRITICAL(&data_lock);
        for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
             channel++) {
          accumulator[channel].read_errors++;
        }
        portEXIT_CRITICAL(&data_lock);
        break;
      }

      uint32_t parsed_count = 0;

      /* Parse block: convert the raw byte buffer to validated ADC result
       * records. */
      err = adc_continuous_parse_data(adc_handle, raw_frame, bytes_read,
                                      parsed_frame, &parsed_count);
      if (err != ESP_OK) {
        /* Parse-error block: retain diagnostics and continue draining later
         * data. */
        portENTER_CRITICAL(&data_lock);
        for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
             channel++) {
          accumulator[channel].read_errors++;
        }
        portEXIT_CRITICAL(&data_lock);
        continue;
      }

      uint32_t normal_raw_average[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
      bool normal_valid[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
      uint32_t frame_fault_mask = 0U;

      /* Processing block: classify raw data, then convert and publish currents.
       */
      accumulate_frame(parsed_frame, parsed_count, normal_raw_average,
                       normal_valid, &frame_fault_mask);
      publish_current_measurement(normal_raw_average, normal_valid,
                                  frame_fault_mask);
    }
  }
}

/**
 * @brief Stop and release every partially or fully created ADC resource.
 *
 * Called by engine_current_sense_stop() and every failure path in
 * engine_current_sense_start(). It stops ADC, deletes the worker, deletes both
 * calibration schemes, and deinitializes DMA.
 */
static void cleanup_resources(void) {
  /* Acquisition block: stop DMA before deleting the task that drains it. */
  if (adc_handle != NULL) {
    adc_continuous_stop(adc_handle);
  }
  if (current_sense_task_handle != NULL) {
    vTaskDelete(current_sense_task_handle);
    current_sense_task_handle = NULL;
  }

  /* Calibration block: release every channel handle that was created. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    if (calibration_handle[channel] != NULL) {
      adc_cali_delete_scheme_curve_fitting(calibration_handle[channel]);
      calibration_handle[channel] = NULL;
    }
  }

  /* Handle block: release the continuous ADC driver last. */
  if (adc_handle != NULL) {
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
  }
}

esp_err_t engine_current_sense_start(void) {
  /* Lifecycle block: this singleton acquisition may be started only once. */
  if (adc_handle != NULL || current_sense_task_handle != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* GPIO-resolution block: map both configured GPIOs to distinct ADC1 channels.
   */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    adc_unit_t unit = ADC_UNIT_1;
    esp_err_t err = adc_continuous_io_to_channel(sense_gpio[channel], &unit,
                                                 &adc_channel[channel]);
    if (err != ESP_OK || unit != ADC_UNIT_1 ||
        (channel > 0U && adc_channel[channel] == adc_channel[0])) {
      return ESP_ERR_INVALID_ARG;
    }
    adc_unit = unit;
  }

  /* DMA-handle block: reserve four frames of storage and flush old data on pool
   * pressure so the worker favors current measurements over stale ones. */
  adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size = CURRENT_SENSE_STORE_BYTES,
      .conv_frame_size = CURRENT_SENSE_FRAME_BYTES,
      .flags.flush_pool = true,
  };
  esp_err_t err = adc_continuous_new_handle(&handle_config, &adc_handle);
  if (err != ESP_OK) {
    return err;
  }

  /* Pattern block: alternate both ADC1 channels at 12 dB attenuation and the
   * target's maximum digital-controller bit width. */
  adc_digi_pattern_config_t patterns[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    patterns[channel] = (adc_digi_pattern_config_t){
        .atten = ADC_ATTEN_DB_12,
        .channel = adc_channel[channel],
        .unit = adc_unit,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
  }
  adc_continuous_config_t continuous_config = {
      .pattern_num = ENGINE_CURRENT_SENSE_CHANNEL_COUNT,
      .adc_pattern = patterns,
      .sample_freq_hz = CURRENT_SENSE_TOTAL_SAMPLE_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
  };

  /* Continuous-mode block: apply the dual-channel pattern and aggregate rate.
   */
  err = adc_continuous_config(adc_handle, &continuous_config);
  if (err != ESP_OK) {
    cleanup_resources();
    return err;
  }

  /* ADC-calibration block: create a curve-fitting handle for each physical
   * channel so raw-code differences are calibrated independently. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = adc_unit,
        .chan = adc_channel[channel],
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    err = adc_cali_create_scheme_curve_fitting(&calibration_config,
                                               &calibration_handle[channel]);
    if (err != ESP_OK) {
      cleanup_resources();
      return err;
    }
  }

  /* Fault-threshold block: validate hysteresis ordering, then convert
   * configured millivolt thresholds to channel-specific calibrated raw codes
   * once. */
  if (CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV >=
      CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV) {
    cleanup_resources();
    return ESP_ERR_INVALID_ARG;
  }
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    err = calibrated_raw_threshold((engine_current_sense_channel_t)channel,
                                   CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV,
                                   &fault_enter_raw[channel]);
    if (err == ESP_OK) {
      err = calibrated_raw_threshold((engine_current_sense_channel_t)channel,
                                     CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV,
                                     &fault_exit_raw[channel]);
    }
    if (err != ESP_OK) {
      cleanup_resources();
      return err;
    }
  }

  /* Publication-reset block: clear frames, accumulators, fault latches, current
   * values, validity, and overflow counts before the worker can run. */
  reset_accumulators();
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    latest_frame[channel] = (engine_current_sense_frame_t){0};
    fault_active[channel] = false;
  }
  latest_measurement = (engine_current_sense_measurement_t){0};
  atomic_store_explicit(&pool_overflows, 0U, memory_order_relaxed);

  /* Worker block: create the low-priority DMA-processing task pinned to Core 0.
   */
  BaseType_t task_created = xTaskCreatePinnedToCore(
      current_sense_task, "is_adc", CURRENT_SENSE_TASK_STACK_SIZE, NULL,
      CURRENT_SENSE_TASK_PRIORITY, &current_sense_task_handle,
      CURRENT_SENSE_CORE_ID);
  if (task_created != pdPASS) {
    cleanup_resources();
    return ESP_ERR_NO_MEM;
  }

  /* Callback/start block: register minimal ISR notifications, then start DMA.
   */
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

  /* Diagnostic block: report resolved pins/channels, rates, and hysteresis. */
  ESP_LOGI(TAG,
           "R_IS GPIO %d/ADC1_CH%d, L_IS GPIO %d/ADC1_CH%d: %d samples/s "
           "each (%d aggregate), fault hysteresis=%d/%d mV",
           sense_gpio[ENGINE_CURRENT_SENSE_CHANNEL_R_IS],
           adc_channel[ENGINE_CURRENT_SENSE_CHANNEL_R_IS],
           sense_gpio[ENGINE_CURRENT_SENSE_CHANNEL_L_IS],
           adc_channel[ENGINE_CURRENT_SENSE_CHANNEL_L_IS],
           CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ, CURRENT_SENSE_TOTAL_SAMPLE_HZ,
           CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV,
           CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV);
  return ESP_OK;
}

void engine_current_sense_stop(void) {
  /* Cleanup block: the helper tolerates resources that were never started. */
  cleanup_resources();
}

/**
 * @brief Convert one captured internal accumulator to its public
 * representation.
 *
 * Called only by engine_current_sense_take_dual_snapshot(). It calculates means
 * safely for empty windows and attaches the consumed global overflow count.
 */
static engine_current_sense_snapshot_t
make_snapshot(const current_sense_accumulator_t *captured, uint32_t overflows) {
  /* Construction block: use zero for undefined empty-window extrema/averages.
   */
  return (engine_current_sense_snapshot_t){
      .samples = captured->samples,
      .raw_average = captured->samples > 0U
                         ? (uint32_t)(captured->raw_sum / captured->samples)
                         : 0U,
      .raw_minimum = captured->samples > 0U ? captured->raw_minimum : 0U,
      .raw_maximum = captured->samples > 0U ? captured->raw_maximum : 0U,
      .normal_samples = captured->normal_samples,
      .normal_raw_average =
          captured->normal_samples > 0U
              ? (uint32_t)(captured->normal_raw_sum / captured->normal_samples)
              : 0U,
      .fault_samples = captured->fault_samples,
      .fault_entries = captured->fault_entries,
      .fault_active = captured->fault_active,
      .invalid_results = captured->invalid_results,
      .pool_overflows = overflows,
      .read_errors = captured->read_errors,
  };
}

bool engine_current_sense_take_dual_snapshot(
    engine_current_sense_dual_snapshot_t *snapshot) {
  /* Lifecycle/interface block: require running acquisition and an output. */
  if (snapshot == NULL || adc_handle == NULL) {
    return false;
  }

  /* Atomic-capture block: copy and reset both accumulators under one critical
   * section so their diagnostic windows have the same boundary. */
  current_sense_accumulator_t captured[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
  portENTER_CRITICAL(&data_lock);
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    captured[channel] = accumulator[channel];
  }
  reset_accumulators();
  portEXIT_CRITICAL(&data_lock);

  /* Overflow-consumption block: attach and reset the ISR counter for this
   * window. */
  uint32_t overflows =
      atomic_exchange_explicit(&pool_overflows, 0U, memory_order_relaxed);
  bool have_samples = false;

  /* Conversion block: build public snapshots and report whether either had
   * data. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    snapshot->channel[channel] = make_snapshot(&captured[channel], overflows);
    have_samples |= captured[channel].samples > 0U;
  }
  return have_samples;
}

bool engine_current_sense_get_latest_dual_frame(
    engine_current_sense_dual_frame_t *frame) {
  /* Lifecycle/interface block: require running acquisition and an output. */
  if (frame == NULL || adc_handle == NULL) {
    return false;
  }

  /* Snapshot block: copy both channel frames under the publication spinlock. */
  portENTER_CRITICAL(&data_lock);
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    frame->channel[channel] = latest_frame[channel];
  }
  portEXIT_CRITICAL(&data_lock);

  /* Validity block: both channels must belong to a published, populated frame.
   */
  return frame->channel[ENGINE_CURRENT_SENSE_CHANNEL_R_IS].sequence != 0U &&
         frame->channel[ENGINE_CURRENT_SENSE_CHANNEL_R_IS].samples > 0U &&
         frame->channel[ENGINE_CURRENT_SENSE_CHANNEL_L_IS].samples > 0U;
}

bool engine_current_sense_get_latest_measurement(
    engine_current_sense_measurement_t *measurement) {
  /* Lifecycle/interface block: require running acquisition and an output. */
  if (measurement == NULL || adc_handle == NULL) {
    return false;
  }

  /* Snapshot block: copy sequence, currents, and masks as one critical section.
   */
  portENTER_CRITICAL(&data_lock);
  *measurement = latest_measurement;
  portEXIT_CRITICAL(&data_lock);

  /* Sequence zero denotes that the worker has not yet published a frame. */
  return measurement->sequence != 0U;
}

esp_err_t engine_current_sense_channel_raw_to_millivolts(
    engine_current_sense_channel_t channel, uint32_t raw, int *millivolts) {
  /* Interface/state block: require a started calibration handle, valid channel,
   * representable signed ADC code, and output destination. */
  if (channel >= ENGINE_CURRENT_SENSE_CHANNEL_COUNT ||
      calibration_handle[channel] == NULL || millivolts == NULL ||
      raw > INT32_MAX) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Calibration block: apply the eFuse/curve-fitting model for this ADC input.
   */
  return adc_cali_raw_to_voltage(calibration_handle[channel], (int)raw,
                                 millivolts);
}

float engine_current_sense_adc_to_r_is_millivolts(int adc_millivolts) {
  /* Divider-inversion block: reconstruct the module-pin voltage from the
   * configured series and pulldown resistor ratio. */
  return (float)adc_millivolts *
         ((float)CONFIG_ENGINE_CURRENT_SENSE_SERIES_RESISTOR_OHMS +
          (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS) /
         (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS;
}

/**
 * @brief Calculate effective ADC volts per ampere sourced by I_IS.
 *
 * Called only by engine_current_sense_adc_to_i_is_milliamperes(). It models the
 * on-module resistor loaded by the complete external divider and then the
 * fraction appearing across the ADC pulldown.
 */
static float adc_transimpedance_ohms(void) {
  /* Parallel-load block: the external divider loads the fitted board resistor.
   */
  const float board_resistance =
      (float)CONFIG_ENGINE_CURRENT_SENSE_BOARD_RESISTOR_OHMS;
  const float divider_resistance =
      (float)CONFIG_ENGINE_CURRENT_SENSE_SERIES_RESISTOR_OHMS +
      (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS;
  const float parallel_resistance = (board_resistance * divider_resistance) /
                                    (board_resistance + divider_resistance);

  /* Divider block: only the pulldown fraction of that loaded voltage reaches
   * ADC. */
  const float adc_transimpedance =
      parallel_resistance *
      (float)CONFIG_ENGINE_CURRENT_SENSE_PULLDOWN_RESISTOR_OHMS /
      divider_resistance;
  return adc_transimpedance;
}

float engine_current_sense_adc_to_i_is_milliamperes(int adc_millivolts) {
  /* Ohm's-law block: mV/ohm is numerically equal to mA. */
  return (float)adc_millivolts / adc_transimpedance_ohms();
}

float engine_current_sense_adc_to_amperes(int adc_millivolts) {
  /* Scale block: multiply sense-output mA by k_ILIS and convert mA to A. */
  return engine_current_sense_adc_to_i_is_milliamperes(adc_millivolts) *
         (float)CONFIG_ENGINE_CURRENT_SENSE_RATIO / 1000.0f;
}

#endif /* CONFIG_ENGINE_CURRENT_SENSE_ENABLE */
