#ifndef CONFIG_H
#define CONFIG_H

/* -----------------------------pin config ----------------------------- */
// ADS131M04 ADC pins
#define ADS131_CLK_PIN  13
#define ADS131_SCK_PIN  10
#define ADS131_MISO_PIN 8
#define ADS131_MOSI_PIN 11
#define ADS131_CS_PIN   9
#define ADS131_DRDY_PIN 12
#define ADS131_RST_PIN  7

// NeoPixel LED pin
#define NEOPIXEL_PIN    16

// Serial communication

/* -----------------------------pressure sensor configure ----------------------------- */
#define SENSOR_MAX_WEIGHT 10000UL
#define SENSOR_SENSITIVITY 1000UL // uv/g
// #define SENSOR_REVERSE

/* -----------------------------ADC configure ----------------------------- */
#define ADC_MAX_VALUE (1 << 24 - 1)
#define ADC_REF_VOLT 1200000UL // uv
#define ADC_PGA 64UL // just for reference
#define ADC_SAMPLE_RATE 8000 // Options: 8000, 4000, 1000

/* -----------------------------filter configure ----------------------------- */
#define FILTER_WINDOW_SIZE 20
#define SLOPE_WINDOW_SIZE 5

/* -----------------------------edge detection configure ----------------------------- */
#define WINDOW_SIZE 4000
#define THRESHOLD_MIN 500 / 10
#define THRESHOLD_MAX 500
#define THRESHOLD_STEP 500 / 10

#define BASE_WEIGHT_UPDATE_THRESHOLD 100

/* -----------------------------baseline configure ----------------------------- */
#define BASELINE_UPDATE_THRESHOLD 100  // Update baseline when avg(max-min) of enabled channels < this value

/* -----------------------------output control configure ----------------------------- */
#define OUTPUT_CONSECUTIVE_COUNT_THRESHOLD 5  // Number of consecutive threshold exceeds required for output trigger
#define OUTPUT_HYSTERESIS_VALUE 50            // Hysteresis value for output trigger/release (must be less than trigger threshold)
#define OUTPUT_MIN_DURATION_MS 100            // Minimum duration for the output to stay active

#endif