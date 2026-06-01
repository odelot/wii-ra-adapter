// Increase loopTask stack from the Arduino default (8KB) to 32KB.
// Must live in a .cpp file — NOT in the .ino — because the Arduino IDE
// auto-inserts forward declarations after any function definition it finds
// in .ino files, which would push prototypes before the Led/LedMode type
// definitions and cause compile errors.
#include <Arduino.h>
SET_LOOP_TASK_STACK_SIZE(32 * 1024);
