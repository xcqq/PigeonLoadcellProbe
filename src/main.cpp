#include <Arduino.h>
#include "apps/apps.hpp"
#include "apps/console.h"
#include "config.h"

// Core 0: Loadcell processing
void setup()
{
    Serial.begin(115200);
    init_loadcell_task();
}

void loop()
{
    loadcell_task_function();
}

// Core 1: Console processing
void setup1(){
    delay(1000); // Wait for core 0 to initialize
    init_console_task();
}

void loop1(){
    console_task_function();
}