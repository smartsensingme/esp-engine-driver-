#ifndef ENGINE_CURRENT_SENSE_H_
#define ENGINE_CURRENT_SENSE_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Physical BTS7960 diagnostic/current-sense output. */
typedef enum {
  /** Right-half-bridge R_IS input, conventionally positive current. */
  ENGINE_CURRENT_SENSE_CHANNEL_R_IS = 0,
  /** Left-half-bridge L_IS input, conventionally negative current. */
  ENGINE_CURRENT_SENSE_CHANNEL_L_IS,
  /** Number of channels; also used to size public arrays. */
  ENGINE_CURRENT_SENSE_CHANNEL_COUNT,
} engine_current_sense_channel_t;

/** Raw ADC statistics accumulated since the previous dual snapshot. */
typedef struct {
  /** Total valid ADC results included in the window. */
  uint32_t samples;
  /** Mean of all valid raw ADC results, including fault-state samples. */
  uint32_t raw_average;
  /** Smallest valid raw ADC result in the window. */
  uint32_t raw_minimum;
  /** Largest valid raw ADC result in the window. */
  uint32_t raw_maximum;
  /** Samples classified as normal current rather than fault signaling. */
  uint32_t normal_samples;
  /** Mean raw ADC result over normal_samples only. */
  uint32_t normal_raw_average;
  /** Samples observed while the hysteretic fault state was active. */
  uint32_t fault_samples;
  /** Number of transitions from normal into the fault state. */
  uint32_t fault_entries;
  /** Fault state at the end of the captured window. */
  bool fault_active;
  /** Parsed DMA results rejected for invalid unit/channel/count. */
  uint32_t invalid_results;
  /** ADC DMA pool-overflow events since the previous snapshot. */
  uint32_t pool_overflows;
  /** Continuous-read or parse failures observed by the worker task. */
  uint32_t read_errors;
} engine_current_sense_snapshot_t;

/** Statistics for one channel in the latest nominal one-millisecond frame. */
typedef struct {
  /** Monotonic frame identifier shared by R_IS and L_IS. */
  uint32_t sequence;
  /** Valid ADC samples for this channel in the frame. */
  uint32_t samples;
  /** Mean of all valid raw samples, including fault-state samples. */
  uint32_t raw_average;
  /** Median of all valid raw samples. */
  uint32_t raw_median;
  /** Smallest valid raw sample. */
  uint32_t raw_minimum;
  /** Largest valid raw sample. */
  uint32_t raw_maximum;
  /** Samples classified as normal current. */
  uint32_t normal_samples;
  /** Mean raw value over normal samples only. */
  uint32_t normal_raw_average;
  /** Samples observed while fault signaling was active. */
  uint32_t fault_samples;
  /** Normal-to-fault transitions during this frame. */
  uint32_t fault_entries;
  /** Hysteretic fault state at the frame end. */
  bool fault_active;
  /** DMA results rejected while constructing this frame. */
  uint32_t invalid_results;
} engine_current_sense_frame_t;

/** Coherent R_IS/L_IS accumulators for one diagnostic window. */
typedef struct {
  /** Statistics indexed by engine_current_sense_channel_t. */
  engine_current_sense_snapshot_t channel[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
} engine_current_sense_dual_snapshot_t;

/** Coherent R_IS/L_IS statistics for the latest one-millisecond frame. */
typedef struct {
  /** Frame data indexed by engine_current_sense_channel_t. */
  engine_current_sense_frame_t channel[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
} engine_current_sense_dual_frame_t;

/** Convert a channel enum to its bit in validity and fault masks. */
#define ENGINE_CURRENT_SENSE_CHANNEL_BIT(channel) (1U << (channel))

/** Latest calibrated magnitudes and diagnostics for one coherent ADC frame. */
typedef struct {
  /** Frame identifier corresponding to the latest dual frame. */
  uint32_t sequence;
  /** Per-channel nonnegative current magnitudes in milliamperes. */
  int32_t channel_milliamps[ENGINE_CURRENT_SENSE_CHANNEL_COUNT];
  /** Bit set for each channel successfully converted to current. */
  uint32_t valid_mask;
  /** Bit set for each channel that signaled a fault during/after the frame. */
  uint32_t fault_mask;
} engine_current_sense_measurement_t;

/**
 * @brief Start dual-channel continuous ADC acquisition and publication.
 *
 * The function maps configured GPIOs to distinct ADC1 channels, creates and
 * configures the continuous ADC/DMA handle, creates one calibration handle per
 * channel, converts fault thresholds from millivolts to raw counts, resets all
 * state, starts a Core 0 worker task, registers callbacks, and starts ADC.
 *
 * This public function is called only by application code. No function inside
 * the component calls it. It internally calls calibrated_raw_threshold(),
 * reset_accumulators(), cleanup_resources() on partial failures, and registers
 * conversion_done_callback() and pool_overflow_callback().
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already started,
 *         ESP_ERR_INVALID_ARG for incompatible pins/thresholds, ESP_ERR_NO_MEM
 *         if task creation fails, or an ADC/calibration driver error.
 */
esp_err_t engine_current_sense_start(void);

/**
 * @brief Stop acquisition and release all current-sense resources.
 *
 * This public function is called only by application code. It delegates to the
 * private cleanup_resources() helper and is safe when acquisition is stopped.
 */
void engine_current_sense_stop(void);

/**
 * @brief Atomically copy and reset both diagnostic-window accumulators.
 *
 * The copied statistics cover all completed frames since the preceding take.
 * Calling this function consumes the window counters. This public function is
 * called only by application code. It calls reset_accumulators() and
 * make_snapshot().
 *
 * @param[out] snapshot Destination for coherent R_IS/L_IS statistics.
 * @return True when either channel contains at least one sample; false for a
 *         null destination, stopped acquisition, or an empty window.
 */
bool engine_current_sense_take_dual_snapshot(
    engine_current_sense_dual_snapshot_t *snapshot);

/**
 * @brief Atomically copy the latest complete R_IS/L_IS frame.
 *
 * This non-consuming public getter is called only by application code. It
 * calls no other function in the component.
 *
 * @param[out] frame Destination for coherent per-channel frame statistics.
 * @return True after both channels have samples in a nonzero sequence; false
 *         for a null destination, stopped acquisition, or unavailable frame.
 */
bool engine_current_sense_get_latest_dual_frame(
    engine_current_sense_dual_frame_t *frame);

/**
 * @brief Copy calibrated per-channel currents and validity/fault masks.
 *
 * The worker publishes one coherent measurement per parsed frame. Current
 * values are nonnegative channel magnitudes; the application combines them
 * with engine_driver_get_state() to interpret effective current direction.
 * This public getter is called only by application code and performs no ADC
 * conversion or calibration in the caller.
 *
 * @param[out] measurement Destination for the latest measurement.
 * @return True after at least one frame has been published; false for a null
 *         destination or stopped/unavailable acquisition.
 */
bool engine_current_sense_get_latest_measurement(
    engine_current_sense_measurement_t *measurement);

/**
 * @brief Convert one channel's raw ADC code to calibrated ADC-pin millivolts.
 *
 * This public function is called only by application code. It delegates to the
 * ESP-IDF curve-fitting calibration handle created for the selected channel.
 *
 * @param[in] channel R_IS or L_IS channel.
 * @param[in] raw Native ADC result to convert.
 * @param[out] millivolts Destination for calibrated voltage at the ADC pin.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE for invalid/uninitialized
 *         arguments, or an ESP-IDF ADC calibration error.
 */
esp_err_t engine_current_sense_channel_raw_to_millivolts(
    engine_current_sense_channel_t channel, uint32_t raw, int *millivolts);

/**
 * @brief Return the stable human-readable label for a sense channel.
 *
 * This public utility is called only by application/diagnostic code. It calls
 * no other component function and always returns a valid static string.
 *
 * @param[in] channel Channel enum to name.
 * @return "R_IS", "L_IS", or "UNKNOWN_IS".
 */
const char *
engine_current_sense_channel_name(engine_current_sense_channel_t channel);

/**
 * @brief Reconstruct module I_IS-pin voltage from ADC-node voltage.
 *
 * This pure public conversion models the configured series/pulldown divider.
 * It is called only by application code and calls no component function.
 *
 * @param[in] adc_millivolts Calibrated voltage measured at the ADC node.
 * @return Reconstructed I_IS module-pin voltage in millivolts.
 */
float engine_current_sense_adc_to_r_is_millivolts(int adc_millivolts);

/**
 * @brief Convert ADC-node voltage to BTS7960 I_IS output current.
 *
 * This pure public conversion divides voltage by the effective transimpedance
 * of the module resistor and external network. It calls
 * adc_transimpedance_ohms().
 *
 * @param[in] adc_millivolts Calibrated voltage measured at the ADC node.
 * @return Estimated current sourced by I_IS, in milliamperes.
 */
float engine_current_sense_adc_to_i_is_milliamperes(int adc_millivolts);

/**
 * @brief Convert ADC-node voltage to nominal motor-current magnitude.
 *
 * This pure public conversion calls
 * engine_current_sense_adc_to_i_is_milliamperes() and multiplies by the
 * configured nominal BTS7960 load-to-sense ratio.
 *
 * @param[in] adc_millivolts Calibrated voltage measured at the ADC node.
 * @return Estimated nonnegative motor-current magnitude in amperes.
 */
float engine_current_sense_adc_to_amperes(int adc_millivolts);

#ifdef __cplusplus
}
#endif

#endif /* ENGINE_CURRENT_SENSE_H_ */
