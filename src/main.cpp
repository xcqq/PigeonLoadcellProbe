#include <Arduino.h>
#include "apps/apps.hpp"
#include "config.h"


void setup()
{
    Serial.begin();
    init_loadcell_task();
}

void loop()
{
    loadcell_task_function();
}

void setup1(){

}
void loop1(){

}