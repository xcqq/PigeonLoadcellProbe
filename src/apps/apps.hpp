#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Loadcell task functions
void init_loadcell_task();
void loadcell_task_function();
bool loadcell_task_step(); // Non-blocking single step processing

// Console functions
void init_console_task();
void console_task_function();

#ifdef __cplusplus
}
#endif
