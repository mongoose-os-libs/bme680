/*
 * Copyright (c) 2019 Deomid "rojer" Ryabkov
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdbool.h>

#include "mgos_event.h"

#include "bsec_interface.h"

#define MGOS_EV_BME680_BASE MGOS_EVENT_BASE('B', '6', '8')
enum mgos_bme680_event {
  MGOS_EV_BME680_BSEC_OUTPUT =
      MGOS_EV_BME680_BASE, /* ev_data: struct mgos_bsec_output */
};

struct mgos_bsec_output {
  bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
  uint8_t num_outputs;  // Actual number of outputs.
  // Outputs pre-parsed for convenience.
  // Thise that are present will have non-zero timestamp.
  bsec_output_t iaq;   // BSEC_OUTPUT_IAQ
  bsec_output_t co2;   // BSEC_OUTPUT_CO2_EQUIVALENT
  bsec_output_t voc;   // BSEC_OUTPUT_BREATH_VOC_EQUIVALENT
  bsec_output_t temp;  // BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE
  bsec_output_t rh;    // BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
  bsec_output_t ps;    // BSEC_OUTPUT_RAW_PRESSURE
};

// Load BSEC library configuration from a file.
bsec_library_return_t mgos_bsec_set_configuration_from_file(const char *file);

// Load BSEC library state from a file.
bsec_library_return_t mgos_bsec_set_state_from_file(const char *file);

// Save BSEC library state to a file.
bsec_library_return_t mgos_bsec_save_state_to_file(const char *file);

// Set temperature heat compensation (BSEC_INPUT_HEATSOURCE) value.
void mgos_bsec_set_input_heat_source_value(float value);

// Set sample rate for the IAQ sensors.
bsec_library_return_t mgos_bsec_set_iaq_sample_rate(float sr);

// Set sample rate for the temperature sensor.
bsec_library_return_t mgos_bsec_set_temp_sample_rate(float sr);

// Set sample rate for the humidity sensor.
bsec_library_return_t mgos_bsec_set_rh_sample_rate(float sr);

// Set sample rate for the pressure sensor.
bsec_library_return_t mgos_bsec_set_rh_sample_rate(float sr);

// Start sensor update loop. Should be called after desired outputs are
// requested via bsec_update_subscription();
bool mgos_bsec_start(void);

// Returns the global instance of the sensor structure.
// If bme680.bsec.enable is set, this will always return NULL so as not to
// interfere with the library-driven measurements.
struct bme680_dev *mgos_bme680_get_global(void);
