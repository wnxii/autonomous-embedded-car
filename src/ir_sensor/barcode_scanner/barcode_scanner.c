#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "barcode_scanner.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>
#include "../wifi/client_server_socket/client_server_socket.h"
#include "../line_following/line_following.h"

// Global variable to keep track of barcode scan
volatile bool error_scanning = false;
volatile bool scan_started = false;

/* Global Variables */
bool reverse_scan = false;                   // Boolean to check whether current scan direction is reversed or not
bool start_scan = false;                     // Boolean to store current scan status, used to ignore initial change in state
bool first_black_detected = false;           // Flag to indicate when the first black bar is detected
uint64_t last_state_change_time = 0;         // Variable to store the last time where the state changed (microseconds), used for measuring the time it takes to scan each bar
uint64_t scanned_timings[CODE_LENGTH] = {0}; // Array to store the time it took to scan each bar
uint16_t count_scanned_bar = 0;              // Count of number of bars scanned
uint16_t count_scanned_char = 0;             // Count of number of characters scanned, used to get target character between delimiters
char scanned_code[CODE_LENGTH + 1] = "";     // String to store scanned barcode binary representation
char barcode_char = ERROR_CHAR;
bool last_state_black = false;               // Last detected state (black or white)
bool ignored_first_gap = false;              // Variables to track if gaps have been ignored after the first two characters
bool ignored_second_gap = false;

uint16_t barcode_adc_buffer[MOVING_AVG_WINDOW] = {0};  // ADC buffer for moving average
uint8_t adc_buffer_index = 0;                      // Tracks the position in the ADC buffer

// FreeRTOS Task Handles
TaskHandle_t xBarcodeTaskHandle = NULL;

// Threshold variables
static uint32_t barcode_contrast_threshold = 1200;  // Initial threshold (mid-range)


// Function to initialize the ADC for the IR sensor (analog contrast detection)
void init_adc() {
    adc_gpio_init(IR_SENSOR_PIN);  // Initialize GPIO for analog input
}


// Moving average for ADC readings
float get_barcode_moving_average_adc() {
    adc_select_input(0);
    uint16_t adc_value = adc_read();
    barcode_adc_buffer[adc_buffer_index] = adc_value;
    adc_buffer_index = (adc_buffer_index + 1) % MOVING_AVG_WINDOW;

    // Calculate the average
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum += barcode_adc_buffer[i];
    }
    return (uint16_t)(sum / MOVING_AVG_WINDOW);  // Convert to volts
}

// Function to reset barcode
void reset_barcode()
{
    // Reset number of bars scanned
    count_scanned_bar = 0;

    // Reset scanned code
    strcpy(scanned_code, "");

    // Reset array of scanned timings
    for (uint16_t i = 0; i < CODE_LENGTH; i++)
    {
        scanned_timings[i] = 0;
    }

    count_scanned_bar = 0;
    strcpy(scanned_code, "");
    for (uint16_t i = 0; i < CODE_LENGTH; i++) {
        scanned_timings[i] = 0;
    }
    start_scan = false;
    reverse_scan = false;
    barcode_char = ERROR_CHAR;
    count_scanned_char = 0;
    first_black_detected = false;
    ignored_first_gap = false;
    ignored_second_gap = false;
    scan_started = false;
    error_scanning = false;
}



// Function to parse scanned bars
char parse_scanned_bars()
{    
    // char displayMessage[50];
    // Initialise array of indexes
    uint16_t indexes[CODE_LENGTH] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

    // Bubble sort the indexes array based on the values in scanned_timings
    for (uint16_t i = 0; i < CODE_LENGTH - 1; ++i)
    {
        for (uint16_t j = 0; j < CODE_LENGTH - i - 1; ++j)
        {
            if (scanned_timings[indexes[j]] < scanned_timings[indexes[j + 1]])
            {
                // Swap indexes if the value at j is less than the value at j + 1
                uint16_t temp = indexes[j];
                indexes[j] = indexes[j + 1];
                indexes[j + 1] = temp;
            }
        }
    }

    // Generate the final binary representation string (initialise all characters to 0, narrow bars)
    for (uint16_t i = 0; i < CODE_LENGTH; ++i)
    {
        scanned_code[i] = '0';
    }
    // Null-terminate the string
    scanned_code[CODE_LENGTH] = '\0';

    // Set the top 3 indexes (top 3 timings) to 1, wide bars
    for (uint16_t i = 0; i < 3; ++i)
    {
        scanned_code[indexes[i]] = '1';
    }

    // Initialise the decoded character
    char decoded_char = ERROR_CHAR;

    /*
        NOTE: Each character in Barcode 39 is encoded using 5 black bars, 4 white bars, and 3 wide bars. To represent each of the
        44 unique characters, a binary representation is used, whereby 1 indicates a wide bar, and 0 indicates a narrow bar.
        The binary representation does not capture any information on the colour of the bar (whether it is black or white).
    */
    // Initialise array used to store each barcode character
    char array_char[TOTAL_CHAR] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
                                   'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                   'Y', 'Z', '_', '.', '$', '/', '+', '%', ' '};

    // Initialise array used to store binary representation of each character
    char *array_code[TOTAL_CHAR] = {"000110100", "100100001", "001100001", "101100000", "000110001", "100110000", "001110000",
                                    "000100101", "100100100", "001100100", "100001001", "001001001", "101001000", "000011001",
                                    "100011000", "001011000", "000001101", "100001100", "001001100", "000011100", "100000011",
                                    "001000011", "101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
                                    "001000110", "000010110", "110000001", "011000001", "111000000", "010010001", "110010000",
                                    "011010000", "010000101", "110000100", "010101000", "010100010", "010001010", "000101010",
                                    "011000100"};

    // Initialise array used to store the reversed binary representation of each character
    char *array_reverse_code[TOTAL_CHAR] = {"001011000", "100001001", "100001100", "000001101", "100011000", "000011001",
                                            "000011100", "101001000", "001001001", "001001100", "100100001", "100100100",
                                            "000100101", "100110000", "000110001", "000110100", "101100000", "001100001",
                                            "001100100", "001110000", "110000001", "110000100", "010000101", "110010000",
                                            "010010001", "010010100", "111000000", "011000001", "011000100", "011010000",
                                            "100000011", "100000110", "000000111", "100010010", "000010011", "000010110",
                                            "101000010", "001000011", "000101010", "010001010", "010100010", "010101000",
                                            "001000110"};

    // Check if parsing for delimit character
    if (count_scanned_char == 1 || count_scanned_char == 3)
    {
        // Check for a matching delimit character
        if (strcmp(scanned_code, DELIMIT_CODE) == 0)
        {
            // Update decoded character
            decoded_char = DELIMIT_CHAR;
            // match = true;
        }
        else if (strcmp(scanned_code, DELIMIT_REVERSED_CODE) == 0)
        {
            // Update decoded character
            decoded_char = DELIMIT_CHAR;
            // match = true;
            // Update scan direction
            reverse_scan = true;
        }
    }
    else
    { // Parsing for character
        // Check scan direction
        if (!reverse_scan)
        {
            // Loop through all possible binary representations for a matching decoded character
            for (int i = 0; i < TOTAL_CHAR; i++)
            {
                if (strcmp(scanned_code, array_code[i]) == 0)
                {
                    // Update decoded character and immediately break out of loop
                    decoded_char = array_char[i];
                    // match = true;
                    break;
                }
            }
        }
        // Reversed scan direction
        else
        {
            // Loop through all possible reverse binary representations for a matching decoded character
            for (int i = 0; i < TOTAL_CHAR; i++)
            {
                if (strcmp(scanned_code, array_reverse_code[i]) == 0)
                {
                    // Update decoded character and immediately break out of loop
                    decoded_char = array_char[i];
                    // match = true;
                    break;
                }
            }
        }
    }

    // Return decoded character to caller
    return decoded_char;
}

// Read and process barcode using ADC
void vBarcodeTask(void *pvParameters) {
    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);
    char message[200]; // Message buffer
    // bool current_state_black = false;
    while(true) {
        if (get_autonomous_running() == false)
            continue;

        uint16_t current_ir_value = get_barcode_moving_average_adc(); // Get averaged ADC value
        bool current_state_black = current_ir_value >= barcode_contrast_threshold;

        if (!first_black_detected && !start_scan && current_state_black) {
            first_black_detected = true;
            last_state_change_time = time_us_64();
            last_state_black = true;
            start_scan = true;
            scan_started = true;
            continue;
        }

        if (start_scan && first_black_detected && current_state_black != last_state_black) {
            uint64_t current_time = time_us_64();
            uint64_t time_diff = current_time - last_state_change_time;
            
            // Reset barcode if timing between bars does not make sense
            if (time_diff > 3000000) {
                reset_barcode();
                continue;
            }
            
            // Ignore gap only once after the first and second characters
            if ((count_scanned_char == 1 && !ignored_first_gap) || (count_scanned_char == 2 && !ignored_second_gap)) {
                last_state_change_time = current_time;  // Update time for the next transition
                last_state_black = current_state_black;

                // Set flags to indicate gaps have been ignored
                if (count_scanned_char == 1) ignored_first_gap = true;
                else if (count_scanned_char == 2) ignored_second_gap = true;

                continue;  // Skip processing this timing as it's the inter-character gap

            }
            
            
            scanned_timings[count_scanned_bar] = time_diff;
            last_state_change_time = current_time;
            count_scanned_bar++;

            if (count_scanned_bar == CODE_LENGTH) {
                count_scanned_char++;
                char scanned_char = parse_scanned_bars();
                if (scanned_char != ERROR_CHAR) {
                    switch (count_scanned_char) {
                        case 1:
                            if (scanned_char != DELIMIT_CHAR) {
                                // printf("BARCODE: Error - No starting delimiter. Resetting...\n");
                                // snprintf(message, sizeof(message), "BARCODE: Error - No starting delimiter. Resetting...\n");
                                // xQueueSend(xServerQueue, &message, portMAX_DELAY); // Send message to display task
                                reset_barcode();
                                error_scanning = true;
                                continue;
                            }
                            break;
                        case 2:
                            barcode_char = scanned_char;
                            break;
                        case 3:
                            if (scanned_char != DELIMIT_CHAR) {
                                error_scanning = true;
                                reset_barcode();
                            } else {
                                snprintf(message, sizeof(message), "BARCODE: Barcode Successfully Decoded: %c\n", barcode_char);
                                xQueueSend(xServerQueue, &message, portMAX_DELAY); // Send message to display task
                                scan_started = false;
                            }
                            reset_barcode();
                            continue;
                        default:
                            break;
                    }
                } else {
                    snprintf(message, sizeof(message), "BARCODE: Error - Invalid character detected. Resetting...\n");
                    xQueueSend(xServerQueue, &message, portMAX_DELAY); // Send message to display task
                    reset_barcode();
                }
                count_scanned_bar = 0;
                memset(scanned_timings, 0, sizeof(scanned_timings));
            }
            last_state_black = current_state_black;
        }

        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Function to initialize button with interrupt for reset
void init_button() {
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_set_pulls(BTN_PIN, true, false); // Pull-up resistor (Active-Low)
}

// Initialize FreeRTOS Tasks
void init_barcode(void) {
    init_adc();
    // Initialize button for resetting the barcode
    init_button();
    xTaskCreate(vBarcodeTask, "Barcode Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 4, &xBarcodeTaskHandle);
}