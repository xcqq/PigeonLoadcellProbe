#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Console buffer sizes
#define CONSOLE_BUFFER_SIZE 128
#define CONSOLE_MAX_ARGS 10
#define CONSOLE_MAX_ARG_LEN 32

// Enum for trigger mode
typedef enum {
    TRIGGER_MODE_SUM,       // Trigger on sum of weights
    TRIGGER_MODE_ANY,       // Trigger on any channel
    TRIGGER_MODE_SUM_OR_ANY // Trigger on sum or any channel
} trigger_mode_t;

// Console command structure
typedef struct {
    const char* name;
    const char* description;
    void (*handler)(int argc, char* argv[]);
} console_command_t;

// Configuration structure
typedef struct {
    uint32_t magic; // Magic number for EEPROM data validation
    uint32_t version; // Configuration version for compatibility checking
    
    // Sensor configuration
    uint32_t sensor_max_weight;
    uint32_t sensor_sensitivity;
    bool sensor_reverse;
    
    // ADC configuration
    uint8_t adc_pga;
    uint16_t adc_sample_rate;
    
    // Filter configuration
    int filter_window_size;
    int slope_window_size;
    
    // Edge detection configuration
    int window_size;
    int threshold_min;
    int threshold_max;
    int threshold_step;
    int trigger_threshold;  // Weight difference from baseline to trigger detection
    
    // Baseline configuration
    int stability_threshold;  // Max-min difference in window to consider signal stable for baseline update
    
    // Output control configuration
    int output_consecutive_count;      // Number of consecutive threshold meets for output trigger
    int output_hysteresis_threshold;   // Hysteresis threshold for output trigger/release
    uint32_t output_min_duration_ms;   // Minimum output duration in milliseconds
    
    // Channel enable control
    bool channel_enable[4];

    // Trigger lock control
    bool trigger_lock;

    // Trigger mode
    trigger_mode_t trigger_mode;

    // Kalman filter configuration
    float kalman_process_noise;     // Process noise covariance (Q)
    float kalman_measurement_noise; // Measurement noise covariance (R)

} runtime_config_t;

// Global configuration variable
extern runtime_config_t g_config;

// Runtime debug control variables (not saved to EEPROM)
extern bool g_debug_enabled;
extern bool g_verbose_output;

// Public function prototypes
void console_init(void);
void console_task(void);
void console_print(const char* format, ...);
void console_println(const char* format, ...);
void debug_log(const char* format, ...);
void debug_log_verbose(const char* format, ...);

// Function to reinitialize Kalman filters with new parameters
void reinitialize_kalman_filters(void);

// Kalman filter auto-tuning function
void kalman_autotune_update(int32_t measurements[4]);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_H