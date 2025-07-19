#include "../config.h"
#include "../hardware/ADS131M04.h"
#include "apps.hpp"
#include "console.h"
#include <Arduino.h>
#include <NeoPixelConnect.h>
#include <math.h>

#define TRIGGER_OUT_PIN 5

ADS131M04 adc;
NeoPixelConnect led(NEOPIXEL_PIN, 1);

// Note: channel_enable is now managed through g_config.channel_enable[]

// Baseline values for weight comparison
int32_t baseline_weights[4] = {0, 0, 0, 0}; // Baseline weight values for each channel

// Trigger locking mechanism variables
int32_t triggered_baseline_weights[4] = {0, 0, 0, 0}; // Locked baseline values when triggered
int32_t triggered_threshold = 0;                      // Locked threshold when triggered
bool is_trigger_locked = false;                       // Flag to indicate if trigger is locked

// Output control variables
bool output_is_active = false;             // Current output state
uint32_t output_active_start_time = 0;     // Time when output turned active (in microseconds)
#define OUTPUT_MIN_DURATION_MS 100  // Minimum active output duration in milliseconds
uint32_t threshold_exceed_count = 0; // Counter for consecutive threshold exceeds
bool baseline_initialized = false;   // Flag to track if baseline has been updated at least once

// Efficient sliding window max/min using monotonic deque
typedef struct {
    int32_t *values;   // Store actual values
    uint32_t *indices; // Store indices for age tracking
    uint32_t front;    // Front pointer
    uint32_t rear;     // Rear pointer
    uint32_t capacity; // Maximum capacity
    uint32_t size;     // Current size
} monotonic_deque_t;

typedef struct {
    int32_t *data;
    int32_t *sort_data;
    uint32_t size;
    int64_t sum;
    int32_t mean;
    int32_t mid;
    int32_t max;
    int32_t min;
    uint32_t index;

    // Efficient max/min tracking
    monotonic_deque_t max_deque; // Monotonic decreasing deque for max
    monotonic_deque_t min_deque; // Monotonic increasing deque for min
    uint32_t data_index;         // Global data index for age tracking
    bool is_filled;              // Flag to indicate if window has been filled at least once
} window_data_t;

window_data_t window_data[4];

// Monotonic deque operations
void deque_init(monotonic_deque_t *deque, uint32_t capacity)
{
    deque->values = (int32_t *)calloc(capacity, sizeof(int32_t));
    deque->indices = (uint32_t *)calloc(capacity, sizeof(uint32_t));
    deque->front = 0;
    deque->rear = 0;
    deque->capacity = capacity;
    deque->size = 0;
}

void deque_free(monotonic_deque_t *deque)
{
    free(deque->values);
    free(deque->indices);
}

bool deque_empty(monotonic_deque_t *deque) { return deque->size == 0; }

int32_t deque_front_value(monotonic_deque_t *deque) { return deque->values[deque->front]; }

uint32_t deque_front_index(monotonic_deque_t *deque) { return deque->indices[deque->front]; }

int32_t deque_back_value(monotonic_deque_t *deque)
{
    uint32_t back_pos = (deque->rear - 1 + deque->capacity) % deque->capacity;
    return deque->values[back_pos];
}

void deque_push_back(monotonic_deque_t *deque, int32_t value, uint32_t index)
{
    deque->values[deque->rear] = value;
    deque->indices[deque->rear] = index;
    deque->rear = (deque->rear + 1) % deque->capacity;
    deque->size++;
}

void deque_pop_front(monotonic_deque_t *deque)
{
    if (deque->size > 0) {
        deque->front = (deque->front + 1) % deque->capacity;
        deque->size--;
    }
}

void deque_pop_back(monotonic_deque_t *deque)
{
    if (deque->size > 0) {
        deque->rear = (deque->rear - 1 + deque->capacity) % deque->capacity;
        deque->size--;
    }
}

// Efficient sliding window max using monotonic decreasing deque
void update_sliding_max(window_data_t *wd, int32_t new_value)
{
    // Remove elements outside window (only when window is filled)
    if (wd->data_index >= wd->size) {
        uint32_t oldest_valid_index = wd->data_index - wd->size;
        while (!deque_empty(&wd->max_deque) &&
               deque_front_index(&wd->max_deque) <= oldest_valid_index) {
            deque_pop_front(&wd->max_deque);
        }
    }

    // Maintain monotonic decreasing property
    while (!deque_empty(&wd->max_deque) && deque_back_value(&wd->max_deque) <= new_value) {
        deque_pop_back(&wd->max_deque);
    }

    // Add new element
    deque_push_back(&wd->max_deque, new_value, wd->data_index);

    // Update max value
    wd->max = deque_empty(&wd->max_deque) ? new_value : deque_front_value(&wd->max_deque);
}

// Efficient sliding window min using monotonic increasing deque
void update_sliding_min(window_data_t *wd, int32_t new_value)
{
    // Remove elements outside window (only when window is filled)
    if (wd->data_index >= wd->size) {
        uint32_t oldest_valid_index = wd->data_index - wd->size;
        while (!deque_empty(&wd->min_deque) &&
               deque_front_index(&wd->min_deque) <= oldest_valid_index) {
            deque_pop_front(&wd->min_deque);
        }
    }

    // Maintain monotonic increasing property
    while (!deque_empty(&wd->min_deque) && deque_back_value(&wd->min_deque) >= new_value) {
        deque_pop_back(&wd->min_deque);
    }

    // Add new element
    deque_push_back(&wd->min_deque, new_value, wd->data_index);

    // Update min value
    wd->min = deque_empty(&wd->min_deque) ? new_value : deque_front_value(&wd->min_deque);
}

int compare(const void *a, const void *b) { return (*(int64_t *)a - *(int64_t *)b); }

void window_data_init(window_data_t *wd, uint32_t size)
{
    wd->sum = 0;
    wd->mean = 0;
    wd->index = 0;
    wd->mid = 0;
    wd->size = size;
    wd->data_index = 0;
    wd->is_filled = false;
    wd->data = (int32_t *)calloc(size, sizeof(*wd->data));
    wd->sort_data = (int32_t *)calloc(size, sizeof(*wd->sort_data));

    // Initialize efficient max/min tracking
    deque_init(&wd->max_deque, size);
    deque_init(&wd->min_deque, size);
}

void window_data_cleanup(window_data_t *wd)
{
    if (wd->data) {
        free(wd->data);
        wd->data = NULL;
    }
    if (wd->sort_data) {
        free(wd->sort_data);
        wd->sort_data = NULL;
    }

    // Clean up efficient max/min tracking
    deque_free(&wd->max_deque);
    deque_free(&wd->min_deque);
}

void window_data_add(window_data_t *wd, int64_t data)
{
    // Only subtract old value when window is filled
    if (wd->data_index >= wd->size) {
        wd->sum -= wd->data[wd->index];
    }

    wd->data[wd->index] = data;
    wd->sum += data;

    // Calculate mean based on actual number of data points
    if (wd->data_index < wd->size) {
        // Window not filled yet, use actual count
        wd->mean = wd->sum / (wd->data_index + 1);
    } else {
        // Window is filled, use full window size
        wd->mean = wd->sum / wd->size;
    }

    wd->index = (wd->index + 1) % wd->size;

    // Update max/min efficiently - O(1) amortized
    update_sliding_max(wd, data);
    update_sliding_min(wd, data);
    wd->data_index++;

    // Check if window has been filled at least once
    if (!wd->is_filled && wd->data_index >= wd->size) {
        wd->is_filled = true;
    }
}

// This function is now O(n) but rarely needed since max/min are updated in real-time
void window_data_max_min(window_data_t *wd)
{
    int32_t max, min;
    int i;

    max = wd->data[0];
    min = wd->data[0];
    for (i = 1; i < wd->size; i++) {
        if (wd->data[i] > max) max = wd->data[i];
        if (wd->data[i] < min) min = wd->data[i];
    }
    wd->max = max;
    wd->min = min;
}

void window_data_mid(window_data_t *wd)
{
    memcpy(wd->sort_data, wd->data, wd->size * sizeof(*wd->sort_data));
    qsort(wd->sort_data, wd->size, sizeof(*wd->sort_data), compare);
    wd->mid = wd->sort_data[wd->size / 2];
}

int32_t calc_slope(window_data_t *wd)
{
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_xy = 0;
    int64_t sum_xx = 0;
    int32_t slope = 0;
    int i;

    for (i = 0; i < wd->size; i++) {
        sum_x += i;
        sum_y += wd->data[i];
        sum_xy += i * wd->data[i];
        sum_xx += i * i;
    }

    slope = (wd->size * sum_xy - sum_x * sum_y) / (wd->size * sum_xx - sum_x * sum_x);

    return slope;
}

int32_t window_data_square_mean(window_data_t *wd)
{

    int32_t sum_squared_diff = 0;
    int32_t square_mean = 0;
    int i;

    // Calculate sum of squared differences from mean
    for (i = 0; i < wd->size; i++) {
        int32_t diff = wd->data[i] - wd->mean;
        sum_squared_diff += diff * diff;
    }

    // Calculate standard deviation
    // Using integer square root approximation for embedded systems
    if (wd->size > 0) {
        square_mean = sum_squared_diff >> 10;
    }

    return square_mean;
}

int64_t volt_to_weight(int64_t value)
{
    int64_t weight;
    weight = value * g_config.sensor_max_weight /
             (ADC_REF_VOLT * g_config.sensor_sensitivity / 1000000) / g_config.adc_pga;

    if (g_config.sensor_reverse) {
        return -weight;
    } else {
        return weight;
    }
}

int64_t adc_to_volt(int64_t value)
{
    int64_t volt;
    int64_t con = ADC_MAX_VALUE / ADC_REF_VOLT * 2;
    volt = value / con;

    return volt;
}

void loadcell_task_function()
{
    adcOutput adc_ret;
    int volts[4] = {0};
    int weights[4] = {0};
    int weights_diff[4] = {0};
    int weights_diff_sum = 0;
    bool any_channel_triggered = false;
    int std_dev[4] = {0};
    int last_time = micros();
    int current_time = 0;

    while (1) {
        if (!adc.isDataReady()) {
            continue;
        }
        current_time = micros();
        adc_ret = adc.readADC();
        any_channel_triggered = false; // Reset before processing new data

        // Process data for all four channels
        volts[0] = adc_to_volt(adc_ret.ch0);
        volts[1] = adc_to_volt(adc_ret.ch1);
        volts[2] = adc_to_volt(adc_ret.ch2);
        volts[3] = adc_to_volt(adc_ret.ch3);

        weights_diff_sum = 0;
        for (int i = 0; i < 4; i++) {
            weights[i] = volt_to_weight(volts[i]);
            window_data_add(&window_data[i], weights[i]);
            weights_diff[i] =
                weights[i] - baseline_weights[i]; // Compare with baseline instead of mean
            // Only add to sum if channel is enabled
            if (g_config.channel_enable[i]) {
                weights_diff_sum += weights_diff[i];
                if (abs(weights_diff[i]) > g_config.trigger_threshold) {
                    any_channel_triggered = true;
                }
            }
        }

        // Check if baseline should be updated
        int enabled_channel_count = 0;
        for (int i = 0; i < 4; i++) {
            if (g_config.channel_enable[i]) {
                enabled_channel_count++;
            }
        }

        // Update baseline if ALL enabled channels' max-min differences are below threshold
        if (enabled_channel_count > 0) {
            // Check if all enabled channels have filled their windows at least once
            bool all_windows_filled = true;
            bool all_channels_stable = true;

            for (int i = 0; i < 4; i++) {
                if (g_config.channel_enable[i]) {
                    // Check if window is filled
                    if (!window_data[i].is_filled) {
                        all_windows_filled = false;
                        break;
                    }

                    // Check if this channel's max-min difference is below threshold
                    int32_t channel_max_min_diff = window_data[i].max - window_data[i].min;
                    if (channel_max_min_diff >= g_config.stability_threshold) {
                        all_channels_stable = false;
                        break;
                    }
                }
            }

            // Only update baseline if all windows are filled and ALL channels are stable
            if (all_windows_filled && all_channels_stable) {
                for (int i = 0; i < 4; i++) {
                    if (g_config.channel_enable[i]) {
                        baseline_weights[i] = window_data[i].mean;
                    }
                }
                // Mark baseline as initialized after first update
                baseline_initialized = true;
            }
        }

        // Output control with minimum active duration, count threshold and hysteresis
        if (!baseline_initialized) {
            // Keep blue LED until baseline is initialized
            led.neoPixelFill(0, 0, 128, true);
        } else {
            bool is_triggered = false;
            if (g_config.trigger_mode == TRIGGER_MODE_SUM) {
                is_triggered = (weights_diff_sum > g_config.trigger_threshold);
            } else { // TRIGGER_MODE_ANY
                is_triggered = any_channel_triggered;
            }

            if (is_triggered) {
                // Threshold exceeded, increment counter
                threshold_exceed_count++;

                // Only activate output if count threshold is reached
                if (threshold_exceed_count >= g_config.output_consecutive_count && !output_is_active) {
                    // Transition to active and lock the trigger
                    led.neoPixelFill(128, 0, 0, true);
                    output_is_active = true;
                    digitalWrite(TRIGGER_OUT_PIN, HIGH);
                    output_active_start_time = current_time;

                    // Lock the trigger: record current baseline and threshold
                    if (g_config.trigger_lock) {
                        is_trigger_locked = true;
                        triggered_threshold = g_config.trigger_threshold;
                        for (int i = 0; i < 4; i++) {
                            triggered_baseline_weights[i] = baseline_weights[i];
                        }
                    }
                }
            } else if (g_config.trigger_lock && is_trigger_locked) {
                // In trigger locked state, use locked baseline for comparison
                int weights_diff_sum_locked = 0;
                bool any_channel_triggered_locked = false;

                for (int i = 0; i < 4; i++) {
                    if (g_config.channel_enable[i]) {
                        int weights_diff_locked = weights[i] - triggered_baseline_weights[i];
                        weights_diff_sum_locked += weights_diff_locked;
                        if (abs(weights_diff_locked) > (triggered_threshold - g_config.output_hysteresis_threshold)) {
                            any_channel_triggered_locked = true;
                        }
                    }
                }

                bool is_below_hysteresis = false;
                if (g_config.trigger_mode == TRIGGER_MODE_SUM) {
                    is_below_hysteresis = (weights_diff_sum_locked < (triggered_threshold - g_config.output_hysteresis_threshold));
                } else { // TRIGGER_MODE_ANY
                    // For 'any' mode, if ANY enabled channel is still triggered (above hysteresis), we don't release the lock.
                    // The lock is released only when ALL enabled channels are below the hysteresis threshold.
                    
                    // We need to check if ANY channel is still *above* the threshold.
                    // The logic was checking if any channel was *below* which is not quite right for releasing the lock.
                    // Let's re-evaluate. The lock should release when the trigger condition is no longer met.
                    // The trigger condition *was* `any_channel_triggered_locked`.
                    // So we should check if `any_channel_triggered_locked` is now false.
                    
                    // Re-checking the logic for 'any' mode with hysteresis
                    any_channel_triggered_locked = false; // Reset before check
                    for (int i = 0; i < 4; i++) {
                        if (g_config.channel_enable[i]) {
                            int weights_diff_locked = weights[i] - triggered_baseline_weights[i];
                            // A channel is considered 'active' if its diff is still above the release threshold
                            if (abs(weights_diff_locked) >= (triggered_threshold - g_config.output_hysteresis_threshold)) {
                                 any_channel_triggered_locked = true;
                                 break; // Found one still active, no need to check others
                            }
                        }
                    }
                    is_below_hysteresis = !any_channel_triggered_locked;
                }

                // Check if below locked threshold with hysteresis
                if (is_below_hysteresis) {
                    // Below locked hysteresis threshold, reset counter and unlock trigger
                    threshold_exceed_count = 0;
                    is_trigger_locked = false;

                    // Should be green, but check minimum red duration
                    if (output_is_active) {
                        // Check if minimum red duration has passed
                        uint32_t red_duration_us = current_time - output_active_start_time;
                        if (red_duration_us >= (g_config.output_min_duration_ms * 1000)) {
                            // Minimum duration passed, can turn green
                            led.neoPixelFill(0, 128, 0, true);
                            output_is_active = false;
                            digitalWrite(TRIGGER_OUT_PIN, LOW);
                        }
                        // If minimum duration not passed, keep red LED (do nothing)
                    } else {
                        // Already green, keep green
                        led.neoPixelFill(0, 128, 0, true);
                    }
                }
                // If still above locked threshold, maintain current state
            } else { // Not triggered and not in lock
                 bool is_below_hysteresis = false;
                if (g_config.trigger_mode == TRIGGER_MODE_SUM) {
                    is_below_hysteresis = (weights_diff_sum < (g_config.trigger_threshold - g_config.output_hysteresis_threshold));
                } else { // TRIGGER_MODE_ANY
                    // In 'any' mode, we are not triggered, so we are by definition below the hysteresis threshold.
                    is_below_hysteresis = !any_channel_triggered;
                }

                if(is_below_hysteresis) {
                    // Below hysteresis threshold, reset counter
                    threshold_exceed_count = 0;

                    // Should be inactive, but check minimum output duration
                    if (output_is_active) {
                        // Check if the minimum output duration has been reached
                        uint32_t active_duration_us = current_time - output_active_start_time;
                        if (active_duration_us >= (g_config.output_min_duration_ms * 1000)) {
                            // The minimum duration has been reached, can turn off the output
                            led.neoPixelFill(0, 128, 0, true);
                            output_is_active = false;
                            digitalWrite(TRIGGER_OUT_PIN, LOW);
                        }
                        // If the minimum duration has not been reached, keep the output active (do nothing)
                    } else {
                        // Already inactive, keep green
                        led.neoPixelFill(0, 128, 0, true);
                    }
                }
            }
        }
        char log_buf[128];
        char *ptr = log_buf;
        int enabled_sum = 0;
        ptr += sprintf(ptr, "loadcell:");
        for (int i = 0; i < 4; i++) {
            if (g_config.channel_enable[i]) {
                ptr += sprintf(ptr, "%d,", weights[i]);
                ptr += sprintf(ptr, "%d,", weights_diff[i]);
            }
        }
        sprintf(ptr, "%d\n", weights_diff_sum);
        sprintf(ptr, "%d\n", output_is_active);
        debug_log("%s", log_buf);
        last_time = current_time;
    }
}

void init_loadcell_task()
{
    // Initialize ADC
    adc.begin(ADS131_CLK_PIN, ADS131_SCK_PIN, ADS131_MISO_PIN, ADS131_MOSI_PIN, ADS131_CS_PIN,
              ADS131_DRDY_PIN, ADS131_RST_PIN);

    led.neoPixelFill(0, 0, 128, true);

    pinMode(TRIGGER_OUT_PIN, OUTPUT);
    digitalWrite(TRIGGER_OUT_PIN, LOW);

    // Initialize output state
    output_is_active = false;
    output_active_start_time = 0;
    threshold_exceed_count = 0;
    baseline_initialized = false;

    // Initialize trigger locking mechanism
    is_trigger_locked = false;
    triggered_threshold = 0;
    for (int i = 0; i < 4; i++) {
        triggered_baseline_weights[i] = 0;
    }

    // Configure all four channels
    uint16_t pga_setting;
    switch (g_config.adc_pga) {
        case 1: pga_setting = CHANNEL_PGA_1; break;
        case 2: pga_setting = CHANNEL_PGA_2; break;
        case 4: pga_setting = CHANNEL_PGA_4; break;
        case 8: pga_setting = CHANNEL_PGA_8; break;
        case 16: pga_setting = CHANNEL_PGA_16; break;
        case 32: pga_setting = CHANNEL_PGA_32; break;
        case 64: pga_setting = CHANNEL_PGA_64; break;
        case 128: pga_setting = CHANNEL_PGA_128; break;
        default: pga_setting = CHANNEL_PGA_64; break; // Default fallback
    }
    
    adc.setOsr(OSR_1024);
    for (int i = 0; i < 4; i++) {
        adc.setInputChannelSelection(i, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
        adc.setChannelPGA(i, pga_setting);
        adc.setChannelEnable(i, true);
        window_data_init(&window_data[i], g_config.window_size);
        // Initialize baseline weights to 0 (will be updated automatically during operation)
        baseline_weights[i] = 0;
    }

}

// Console task functions
void init_console_task() { console_init(); }

void console_task_function() { console_task(); }
