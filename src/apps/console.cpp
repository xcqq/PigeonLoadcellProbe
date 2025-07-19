#include "console.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "../config.h"
#include <EEPROM.h>

#define CONFIG_MAGIC 0xDEADBEEF
#define CONFIG_VERSION 1  // Increment this when config structure changes
#define EEPROM_SIZE sizeof(runtime_config_t)

// Runtime debug control variables (not saved to EEPROM)
bool g_debug_enabled = false;
bool g_verbose_output = false;

// Global runtime configuration with default values
runtime_config_t g_config = {
    .magic = CONFIG_MAGIC,
    .version = CONFIG_VERSION,
    
    // Sensor configuration
    .sensor_max_weight = SENSOR_MAX_WEIGHT,
    .sensor_sensitivity = SENSOR_SENSITIVITY,
    .sensor_reverse = false,
    
    // ADC configuration
    .adc_pga = ADC_PGA,
    
    // Filter configuration
    .filter_window_size = FILTER_WINDOW_SIZE,
    .slope_window_size = SLOPE_WINDOW_SIZE,
    
    // Edge detection configuration
    .window_size = WINDOW_SIZE,
    .threshold_min = THRESHOLD_MIN,
    .threshold_max = THRESHOLD_MAX,
    .threshold_step = THRESHOLD_STEP,
    .trigger_threshold = BASE_WEIGHT_UPDATE_THRESHOLD,
    
    // Baseline configuration
    .stability_threshold = BASELINE_UPDATE_THRESHOLD,
    
    // Output control configuration
    .output_consecutive_count = OUTPUT_CONSECUTIVE_COUNT_THRESHOLD,
    .output_hysteresis_threshold = OUTPUT_HYSTERESIS_VALUE,
    .output_min_duration_ms = OUTPUT_MIN_DURATION_MS,
    
    // Channel enable control
    .channel_enable = {true, true, true, false}
};

// Console state
static char console_buffer[CONSOLE_BUFFER_SIZE];
static int console_buffer_pos = 0;
static bool console_echo = true;

// Forward declarations
static void cmd_help(int argc, char* argv[]);
static void cmd_config(int argc, char* argv[]);
static void cmd_debug(int argc, char* argv[]);
static void cmd_status(int argc, char* argv[]);
static void cmd_reset(int argc, char* argv[]);
static void cmd_channel(int argc, char* argv[]);
static void cmd_save(int argc, char* argv[]);
static void cmd_restore_default(int argc, char* argv[]);

// Command table
static const console_command_t commands[] = {
    {"help", "Show available commands", cmd_help},
    {"cfg", "Get/set configuration parameters", cmd_config},
    {"debug", "Control debug output (on/off/verbose)", cmd_debug},
    {"status", "Show system status", cmd_status},
    {"reset", "Reset system configuration to defaults", cmd_reset},
    {"ch", "Enable/disable channels (ch <0-3> <on/off>)", cmd_channel},
    {"save", "Save current configuration to EEPROM", cmd_save},
    {"restore_default", "Restore all configuration to default values and save", cmd_restore_default},
    {NULL, NULL, NULL} // End marker
};

// Prototypes for internal functions
static void load_config_from_eeprom();
static void save_config_to_eeprom();
static void set_default_config();

// Utility functions
static void console_print_prompt(void) {
    Serial.print("loadcell> ");
}

static int parse_args(char* input, char* argv[], int max_args) {
    int argc = 0;
    char* token = strtok(input, " \t\n\r");
    
    while (token != NULL && argc < max_args) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\n\r");
    }
    
    return argc;
}

static const console_command_t* find_command(const char* name) {
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(commands[i].name, name) == 0) {
            return &commands[i];
        }
    }
    return NULL;
}

// Command implementations
static void cmd_help(int argc, char* argv[]) {
    console_println("Available commands:");
    for (int i = 0; commands[i].name != NULL; i++) {
        console_println("  %-12s - %s", commands[i].name, commands[i].description);
    }
    console_println("");
    console_println("Configuration parameters (use 'config <param> [value]'):");
    console_println("  sensor_max_weight    - Maximum sensor weight (g) [1-100000]");
    console_println("  sensor_sensitivity   - Sensor sensitivity (uV/g) [1-10000]");
    console_println("  sensor_reverse       - Reverse sensor polarity (true/false)");
    console_println("  adc_pga             - ADC PGA gain [1,2,4,8,16,32,64,128]");
    console_println("  trigger_threshold   - Weight difference from baseline to trigger detection [1-10000]");
    console_println("  stability_threshold - Max-min difference in window for stable baseline update [1-1000]");
    console_println("  output_consecutive_count - Consecutive threshold meets for output trigger [1-100]");
    console_println("  output_hysteresis_threshold - Hysteresis threshold for output trigger/release [1-1000]");
    console_println("  output_min_duration_ms - Minimum output duration (ms) [10-10000]");
    console_println("  save                - Save current configuration to persistent memory");
    console_println("");
    console_println("Debug log output format (when debug is enabled):");
    console_println("  loadcell: <weight1>,<diff1>,<weight2>,<diff2>,<weight3>,<diff3>,<weight4>,<diff4>,<total_diff>,<output_active>");
    console_println("  Where:");
    console_println("    - weight1-4: Current weight values for channels 0-3 (in grams)");
    console_println("    - diff1-4: Weight differences from baseline for channels 0-3 (in grams)");
    console_println("    - total_diff: Sum of weight differences from all enabled channels");
    console_println("    - output_active: Output trigger state (1=active, 0=inactive)");
    console_println("  Note: Only enabled channels are included in the output");
}

static void cmd_config(int argc, char* argv[]) {
    if (argc == 1) {
        // Show all configuration
        console_println("Current configuration:");
        console_println("  config_version       = %u", g_config.version);
        console_println("  sensor_max_weight    = %u", g_config.sensor_max_weight);
        console_println("  sensor_sensitivity   = %u", g_config.sensor_sensitivity);
        console_println("  sensor_reverse       = %s", g_config.sensor_reverse ? "true" : "false");
        console_println("  adc_pga             = %u", g_config.adc_pga);
        console_println("  trigger_threshold   = %d", g_config.trigger_threshold);
        console_println("  stability_threshold = %d", g_config.stability_threshold);
        console_println("  output_consecutive_count = %u", g_config.output_consecutive_count);
        console_println("  output_hysteresis_threshold = %u", g_config.output_hysteresis_threshold);
        console_println("  output_min_duration_ms = %u", g_config.output_min_duration_ms);
        return;
    }
    
    if (argc == 2) {
        // Show specific parameter
        const char* param = argv[1];
        if (strcmp(param, "sensor_max_weight") == 0) {
            console_println("sensor_max_weight = %u", g_config.sensor_max_weight);
        } else if (strcmp(param, "sensor_sensitivity") == 0) {
            console_println("sensor_sensitivity = %u", g_config.sensor_sensitivity);
        } else if (strcmp(param, "sensor_reverse") == 0) {
            console_println("sensor_reverse = %s", g_config.sensor_reverse ? "true" : "false");
        } else if (strcmp(param, "adc_pga") == 0) {
            console_println("adc_pga = %u", g_config.adc_pga);
        } else if (strcmp(param, "trigger_threshold") == 0) {
            console_println("trigger_threshold = %d", g_config.trigger_threshold);
        } else if (strcmp(param, "stability_threshold") == 0) {
            console_println("stability_threshold = %d", g_config.stability_threshold);
        } else if (strcmp(param, "output_consecutive_count") == 0) {
            console_println("output_consecutive_count = %u", g_config.output_consecutive_count);
        } else if (strcmp(param, "output_hysteresis_threshold") == 0) {
            console_println("output_hysteresis_threshold = %u", g_config.output_hysteresis_threshold);
        } else if (strcmp(param, "output_min_duration_ms") == 0) {
            console_println("output_min_duration_ms = %u", g_config.output_min_duration_ms);
        } else {
            console_println("Unknown parameter: %s", param);
        }
        return;
    }
    
    if (argc == 3) {
        // Set parameter
        const char* param = argv[1];
        const char* value = argv[2];
        
        if (strcmp(param, "sensor_max_weight") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 0 || val > 100000) {
                console_println("Invalid value: must be 0-100000");
                return;
            }
            g_config.sensor_max_weight = (uint32_t)val;
            console_println("sensor_max_weight set to %u", g_config.sensor_max_weight);
        } else if (strcmp(param, "sensor_sensitivity") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 1 || val > 10000) {
                console_println("Invalid value: must be 1-10000");
                return;
            }
            g_config.sensor_sensitivity = (uint32_t)val;
            console_println("sensor_sensitivity set to %u", g_config.sensor_sensitivity);
        } else if (strcmp(param, "sensor_reverse") == 0) {
            g_config.sensor_reverse = (strcmp(value, "true") == 0 || strcmp(value, "1") == 0);
            console_println("sensor_reverse set to %s", g_config.sensor_reverse ? "true" : "false");
        } else if (strcmp(param, "adc_pga") == 0) {
            const int valid_pga[] = {1, 2, 4, 8, 16, 32, 64, 128};
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0') {
                console_println("Invalid value: must be a number");
                return;
            }
            
            bool valid = false;
            for (int i = 0; i < 8; i++) {
                if (val == valid_pga[i]) {
                    valid = true;
                    break;
                }
            }
            
            if (!valid) {
                console_println("Invalid PGA value: must be 1, 2, 4, 8, 16, 32, 64, or 128");
                return;
            }
            
            g_config.adc_pga = (uint8_t)val;
            console_println("adc_pga set to %u", g_config.adc_pga);
            console_println("Note: Changes to adc_pga require system restart to take effect");
        } else if (strcmp(param, "trigger_threshold") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 1 || val > 10000) {
                console_println("Invalid value: must be 1-10000");
                return;
            }
            g_config.trigger_threshold = (int)val;
            console_println("trigger_threshold set to %d", g_config.trigger_threshold);
        } else if (strcmp(param, "stability_threshold") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 1 || val > 1000) {
                console_println("Invalid value: must be 1-1000");
                return;
            }
            g_config.stability_threshold = (int)val;
            console_println("stability_threshold set to %d", g_config.stability_threshold);
        } else if (strcmp(param, "output_consecutive_count") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 1 || val > 100) {
                console_println("Invalid value: must be 1-100");
                return;
            }
            g_config.output_consecutive_count = (int)val;
            console_println("output_consecutive_count set to %d", g_config.output_consecutive_count);
        } else if (strcmp(param, "output_hysteresis_threshold") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 1 || val > 1000) {
                console_println("Invalid value: must be 1-1000");
                return;
            }
            g_config.output_hysteresis_threshold = (int)val;
            console_println("output_hysteresis_threshold set to %d", g_config.output_hysteresis_threshold);
        } else if (strcmp(param, "output_min_duration_ms") == 0) {
            char* endptr;
            long val = strtol(value, &endptr, 10);
            if (*endptr != '\0' || val < 10 || val > 10000) {
                console_println("Invalid value: must be 10-10000");
                return;
            }
            g_config.output_min_duration_ms = (uint32_t)val;
            console_println("output_min_duration_ms set to %u", g_config.output_min_duration_ms);
        } else {
            console_println("Unknown parameter: %s", param);
        }
        return;
    }
    
    console_println("Usage: config [parameter] [value]");
}

static void cmd_debug(int argc, char* argv[]) {
    if (argc == 1) {
        console_println("Debug: %s, Verbose: %s", 
                       g_debug_enabled ? "ON" : "OFF",
                       g_verbose_output ? "ON" : "OFF");
        return;
    }
    
    const char* cmd = argv[1];
    if (strcmp(cmd, "on") == 0) {
        g_debug_enabled = true;
        console_println("Debug output enabled");
    } else if (strcmp(cmd, "off") == 0) {
        g_debug_enabled = false;
        console_println("Debug output disabled");
    } else if (strcmp(cmd, "verbose") == 0) {
        g_verbose_output = !g_verbose_output;
        console_println("Verbose output %s", g_verbose_output ? "enabled" : "disabled");
    } else {
        console_println("Usage: debug <on|off|verbose>");
    }
}

static void cmd_status(int argc, char* argv[]) {
    console_println("System Status:");
    console_println("  Uptime: %u ms", millis());
    console_println("  Free heap: %u bytes", rp2040.getFreeHeap());
    console_println("  CPU frequency: %u MHz", rp2040.f_cpu() / 1000000);
    console_println("  Debug: %s", g_debug_enabled ? "ON" : "OFF");
    console_println("  Channels: CH0=%s CH1=%s CH2=%s CH3=%s",
                    g_config.channel_enable[0] ? "ON" : "OFF",
                    g_config.channel_enable[1] ? "ON" : "OFF",
                    g_config.channel_enable[2] ? "ON" : "OFF",
                    g_config.channel_enable[3] ? "ON" : "OFF");
}

static void cmd_reset(int argc, char* argv[]) {
    console_println("Resetting configuration to defaults...");
    set_default_config();
    save_config_to_eeprom();
    console_println("Configuration reset and saved.");
}

static void cmd_channel(int argc, char* argv[]) {
    if (argc == 1) {
        console_println("Channel status:");
        for (int i = 0; i < 4; i++) {
            console_println("  CH%d: %s", i, g_config.channel_enable[i] ? "ON" : "OFF");
        }
        return;
    }
    
    if (argc != 3) {
        console_println("Usage: channel <0-3> <on|off>");
        return;
    }
    
    int channel = atoi(argv[1]);
    if (channel < 0 || channel > 3) {
        console_println("Invalid channel number: %d (must be 0-3)", channel);
        return;
    }
    
    const char* state = argv[2];
    if (strcmp(state, "on") == 0 || strcmp(state, "1") == 0) {
        g_config.channel_enable[channel] = true;
        console_println("Channel %d enabled", channel);
    } else if (strcmp(state, "off") == 0 || strcmp(state, "0") == 0) {
        g_config.channel_enable[channel] = false;
        console_println("Channel %d disabled", channel);
    } else {
        console_println("Invalid state: %s (use on/off)", state);
    }
}

static void cmd_save(int argc, char* argv[]) {
    save_config_to_eeprom();
}

static void cmd_restore_default(int argc, char* argv[]) {
    console_println("Restoring all configuration to default values and saving...");
    set_default_config();
    save_config_to_eeprom();
    console_println("Configuration restored and saved.");
}

// Console API functions
void console_init(void) {
    console_buffer_pos = 0;
    memset(console_buffer, 0, sizeof(console_buffer));

    EEPROM.begin(EEPROM_SIZE);
    load_config_from_eeprom();
    
    Serial.println("");
    Serial.println("LoadCell Console CLI");
    Serial.println("Type 'help' for available commands");
    console_print_prompt();
}

void console_task(void) {
    // Non-blocking console processing
    while (Serial.available()) {
        char c = Serial.read();
        
        if (console_echo) {
            Serial.write(c);
        }
        
        if (c == '\r') {
            // Ignore carriage return, we will handle everything on line feed
            continue;
        }

        if (c == '\n') {
            if (console_buffer_pos > 0) {
                console_buffer[console_buffer_pos] = '\0';
                
                // Parse and execute command
                char* argv[CONSOLE_MAX_ARGS];
                int argc = parse_args(console_buffer, argv, CONSOLE_MAX_ARGS);
                
                if (argc > 0) {
                    const console_command_t* cmd = find_command(argv[0]);
                    if (cmd) {
                        cmd->handler(argc, argv);
                    } else {
                        console_println("Unknown command: %s (type 'help' for available commands)", argv[0]);
                    }
                }
                
                console_buffer_pos = 0;
                console_print_prompt();
            } else {
                console_print_prompt();
            }
        } else if (c == '\b' || c == 127) { // Backspace
            if (console_buffer_pos > 0) {
                console_buffer_pos--;
                Serial.print(" \b"); // Erase character
            }
        } else if (c >= 32 && c < 127) { // Printable characters
            if (console_buffer_pos < CONSOLE_BUFFER_SIZE - 1) {
                console_buffer[console_buffer_pos++] = c;
            }
        }
    }
}

void console_print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.print(buffer);
    va_end(args);
}

void console_println(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.println(buffer);
    va_end(args);
}

void debug_log(const char* format, ...) {
    if (!g_debug_enabled) return;
    
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.println(buffer);
    va_end(args);
}

void debug_log_verbose(const char* format, ...) {
    if (!g_debug_enabled || !g_verbose_output) return;
    
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.println(buffer);
    va_end(args);
}

// --- Configuration Management ---

static void set_default_config() {
    g_config.magic = CONFIG_MAGIC;
    g_config.version = CONFIG_VERSION;
    g_config.sensor_max_weight = SENSOR_MAX_WEIGHT;
    g_config.sensor_sensitivity = SENSOR_SENSITIVITY;
    g_config.sensor_reverse = false;
    g_config.adc_pga = ADC_PGA;
    g_config.filter_window_size = FILTER_WINDOW_SIZE;
    g_config.slope_window_size = SLOPE_WINDOW_SIZE;
    g_config.window_size = WINDOW_SIZE;
    g_config.threshold_min = THRESHOLD_MIN;
    g_config.threshold_max = THRESHOLD_MAX;
    g_config.threshold_step = THRESHOLD_STEP;
    g_config.trigger_threshold = BASE_WEIGHT_UPDATE_THRESHOLD;
    g_config.stability_threshold = BASELINE_UPDATE_THRESHOLD;
    g_config.output_consecutive_count = OUTPUT_CONSECUTIVE_COUNT_THRESHOLD;
    g_config.output_hysteresis_threshold = OUTPUT_HYSTERESIS_VALUE;
    g_config.output_min_duration_ms = OUTPUT_MIN_DURATION_MS;
    g_config.channel_enable[0] = true;
    g_config.channel_enable[1] = true;
    g_config.channel_enable[2] = true;
    g_config.channel_enable[3] = false;
}

static void load_config_from_eeprom() {
    runtime_config_t temp_config;
    EEPROM.get(0, temp_config);

    if (temp_config.magic == CONFIG_MAGIC && temp_config.version == CONFIG_VERSION) {
        memcpy(&g_config, &temp_config, sizeof(runtime_config_t));
        console_println("Configuration loaded from EEPROM.");
    } else {
        if (temp_config.magic != CONFIG_MAGIC) {
            console_println("No valid configuration found in EEPROM, loading defaults.");
        } else {
            console_println("Configuration version mismatch (stored: %u, expected: %u), resetting to defaults.", 
                          temp_config.version, CONFIG_VERSION);
        }
        set_default_config();
        save_config_to_eeprom();
    }
}

static void save_config_to_eeprom() {
    EEPROM.put(0, g_config);
    if (EEPROM.commit()) {
        console_println("Configuration saved to EEPROM.");
    } else {
        console_println("ERROR: Failed to save configuration to EEPROM.");
    }
}