#ifndef ENGINE_CURRENT_SENSE_H_
#define ENGINE_CURRENT_SENSE_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Raw ADC statistics accumulated since the previous diagnostic snapshot. */
typedef struct {
  uint32_t samples;
  uint32_t raw_average;
  uint32_t raw_minimum;
  uint32_t raw_maximum;
  uint32_t normal_samples;
  uint32_t normal_raw_average;
  uint32_t fault_samples;
  uint32_t fault_entries;
  bool fault_active;
  uint32_t invalid_results;
  uint32_t pool_overflows;
  uint32_t read_errors;
} engine_current_sense_snapshot_t;

/** Statistics for the latest completed ADC frame (nominally one millisecond).
 */
typedef struct {
  uint32_t sequence;
  uint32_t samples;
  uint32_t raw_average;
  uint32_t raw_median;
  uint32_t raw_minimum;
  uint32_t raw_maximum;
  uint32_t normal_samples;
  uint32_t normal_raw_average;
  uint32_t fault_samples;
  uint32_t fault_entries;
  bool fault_active;
  uint32_t invalid_results;
} engine_current_sense_frame_t;

/** Start ADC1 continuous conversion, DMA, and the Core 0 accumulator task. */
esp_err_t engine_current_sense_start(void);

/** Stop and release every current-sense acquisition resource. */
void engine_current_sense_stop(void);

/** Copy and reset the long diagnostic-window accumulator. */
bool engine_current_sense_take_snapshot(
    engine_current_sense_snapshot_t *snapshot);

/** Copy the latest complete frame without consuming it. */
bool engine_current_sense_get_latest_frame(engine_current_sense_frame_t *frame);

/**
 * Copy the latest calibrated, PWM-weighted equivalent current.
 *
 * The ADC task updates this value once per completed 1 ms frame. Reading it is
 * lock-free and does not run ADC calibration in the caller.
 */
bool engine_current_sense_get_latest_current_milliamps(int32_t *milliamps);

/** Convert a raw ADC result to calibrated millivolts at the ADC pin. */
esp_err_t engine_current_sense_raw_to_millivolts(uint32_t raw, int *millivolts);

/** Reconstruct the voltage at the module R_IS pin from the divider voltage. */
float engine_current_sense_adc_to_r_is_millivolts(int adc_millivolts);

/** Convert ADC-pin voltage to the current sourced by the I_IS pin. */
float engine_current_sense_adc_to_i_is_milliamperes(int adc_millivolts);

/** Convert ADC-pin voltage to nominal equivalent current. */
float engine_current_sense_adc_to_amperes(int adc_millivolts);

#ifdef __cplusplus
}
#endif

#endif /* ENGINE_CURRENT_SENSE_H_ */
