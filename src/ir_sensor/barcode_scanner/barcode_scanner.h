#ifndef barcode_scanner_H
#define barcode_scanner_H

#include "FreeRTOS.h"
#include "queue.h" // Include FreeRTOS queue handling for QueueHandle_t


/* Macros */
// Barcode 39
#define TOTAL_CHAR 44                     // Total number of characters encoded by Barcode 39 representation
#define CODE_LENGTH 9                     // Length of each barcode's binary representation
#define DELIMIT_CHAR '*'                  // Delimiter used as a start/stop character before actual data reading
#define DELIMIT_CODE "010010100"          // Binary representation of delimit character
#define DELIMIT_REVERSED_CODE "001010010" // Reversed binary representation of delimit character
#define ERROR_CHAR '#'                    // Error character

// Sensors
#define BTN_PIN 20                       // Maker kit button pin
#define IR_SENSOR_PIN 26                 // IR sensor pin

#define MOVING_AVG_WINDOW 2         // Define window size for moving average
#define CONTRAST_THRESHOLD 1.4f     // Contrast threshold in volts for detecting black vs. white


void vBarcodeTask(void *pvParameters);
void vButtonTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vInitializeTasks(void);
void init_button();

// extern char barcode_char;              // Character variable to store scanned and parsed barcode character
// extern volatile bool scanning_allowed; // Boolean to indicate when scanning is allowed
// extern bool start_scan;                // Boolean to indicate barcode is scanning
// External queue declaration for server-bound messages
extern QueueHandle_t xServerQueue;

#endif