/*
 * Author: Joshua Chua
 * Description: Main file for the project
 */
#include "magnetometer_another.h"

/**
 * magnetometer_task()
 * --------
 * Purpose: vTask to be ran by FreeRTOS.
 *          Initialises the magnometer
 *          Reads the magnometer every 2 seconds depending on the direction facing
 *          Accelerometer not used
 * Arguments: params from vTask
 * Return: 1 as completed
 */
void magnetometer_task(void* params){
    
    //Initialise the Magnetometer
    i2c_init_pins();
    magnetometer_init();

    while (1)
    {
        int16_t read_magnetometer_data[3];
        
        reading_magnetometer(read_magnetometer_data);

        float direction = calculate_angle(read_magnetometer_data[0], read_magnetometer_data[1]);

        printf("Direction: %.2f C\n", direction);

        sleep_ms(1000);
    }
}

/**
 * i2c_init_pins()
 * --------
 * Purpose: Initializes I2C Pins
 * Arguments: None
 * Return: None
 */
void i2c_init_pins()
{
    i2c_init(I2C_CHANNEL, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

}

/**
 * magnetometer_init()
 * --------
 * Purpose: Initializes the magnetometer. Further Code explanation below based on the manual
 * Arguments: None
 * Return: None
 */
void magnetometer_init()
{
    //Before Configuring the Sensor, we need to know the register values (Reference Page 23 of the datasheet)
    /*  Register 0x00 = Configuration A: The data rate specifies how frequently the sensor provides measurements. 0x10 means 15Hz according to the datasheet
    *   Register 0x01 = Configuration B: The gain (sensitivity) of the magnetometer. The gain setting determines the measurement range and sensitivity of the sensor.
    *                                   Set to 0x20 as the lowest sensitivity which is 1.3 Gauss
    *   Register 0x02 = Mode Configuration: Set to continuous conversion mode as 0x00
    *   The other Registers: Stores the LSB and MSB of the X, Y and Z Data
    */

    //So, we just need to configure the fist few registers of the Magnetnometer
    //config_data = {Register Address A, ConfigA, Register Address B, Config B, Mode Register, Continuous_config}
    uint8_t config_sampling_data[2] = {SAMPLE_CONFIG_ADDR, 0x1C};
    i2c_write_blocking(I2C_CHANNEL, MAGNETOMETER_ADDR, config_sampling_data, 2, false);

    uint8_t config_gain_data[2] =  {GAIN_CONFIG_ADDR, 0x20};
    i2c_write_blocking(I2C_CHANNEL, MAGNETOMETER_ADDR, config_gain_data, 2, false);

    uint8_t config_mode_data[2] = {MODE_CONFIG_ADDR, 0x00};
    i2c_write_blocking(I2C_CHANNEL, MAGNETOMETER_ADDR, config_mode_data, 2, false);
}

/**
 * accelerometer_init()
 * --------
 * Purpose: Initializes the accelerometer.
 * Arguments: None
 * Return: None
 */
void accelerometer_init()
{
    //Similar to the magnetometer, we need to know the register values (Reference Page 22 of the datasheet)
    // Register 0x20 = Modfy Power mode and enabled Axis at page 24
    uint8_t config_data[2] = {0x20, 0x27};
    i2c_write_blocking(I2C_CHANNEL, ACCELEROMETER_ADDR, config_data, 2, false);

}

/**
 * reading _magnometer()
 * --------
 * Purpose: Reads the x, y and z axis and storing it into the array
 * Arguments: int16_t* result which is the array to store the results
 * Return: true
 */
bool reading_magnetometer(int16_t* result)
{
    uint8_t data[6]; //Use to store the data collected
    uint8_t starting_addr = 0x03; //0x03 is the Data Output X MSB register address. Tells the sensor I want to read from there
    i2c_write_blocking(I2C_CHANNEL, MAGNETOMETER_ADDR, &starting_addr, 1, true); 
    i2c_read_blocking(I2C_CHANNEL, MAGNETOMETER_ADDR, data, 6, false); //Read Register 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 where is X, Z and Y respectively

    // In the data sheet. the data is stored as
    // {x_axis_high_8bits, x_axis_low_8bits, z_axis_high_8bits, z_axis_low_8bits, y_axis_high_8bits, y_axis_low_8bits}
    // Hence, we need to bit shift the low bits by 8 and OR bitwise OR with the low bits
    // Stored in 2's complement, hence I never use unsigned integer 
    result[0] = (data[0] << 8 | data[1]); // x-axis
    result[2] = (data[2] << 8 | data[3]); // z-axis
    result[1] = (data[4] << 8 | data[5]); // y-axis

    return true;
}

/**
 * reading_accelerometer()
 * --------
 * Purpose: Reads the x, y and z accelerated and storing it into the array
 * Arguments: int16_t* result which is the array to store the results
 * Return: true
 */
bool reading_accelerometer(int16_t* result)
{
    int8_t data[6]; //Use to store the data collected
    uint8_t starting_addr = 0x28; //0x03 is the Data Output X MSB register address. Tells the sensor I want to read from there
    i2c_write_blocking(I2C_CHANNEL, ACCELEROMETER_ADDR, &starting_addr, 1, true); 
    i2c_read_blocking(I2C_CHANNEL, ACCELEROMETER_ADDR, data, 6, false); //Read Register 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D where is X, Z and Y respectively

    //Similar to the Magnetometer, however, this time the bits are arranged, low than high
    // {x_axis_high_8bits, x_axis_low_8bits, y_axis_high_8bits, y_axis_low_8bits, z_axis_high_8bits, z_axis_low_8bits}
    result[0] = (data[1] << 8) | data[0]; // x-axis
    result[1] = (data[3] << 8) | data[2]; // y-axis
    result[2] = (data[5] << 8) | data[4]; // z-axis

    return true;
}

/**
 * read_angle_difference()
 * --------
 * Purpose: Reads the difference in angle from the current heading and initial heading
 * Arguments: float curr_heading
 * Return: difference of the current and initial heading
 */
float read_angle_difference(float curr_heading)
{
    static float prev_heading = 0; 

    float difference = curr_heading - prev_heading;

    prev_heading = curr_heading;

    return difference;

}

/**
 * calculate_angle()
 * --------
 * Purpose: Using the Arctangent function formula to calculate the angle
 * Arguments: int16_t of the x and y axis
 * Return: the angle it is facing base on a compass
 */
float calculate_angle(int16_t x, int16_t y) { 

    // Calculate the magnetic heading in degrees 
    float heading = (atan2(y, x) * RADIANS_TO_DEGREES); 

    if (heading < 0) { 
        heading += 360.0; 
    }

    return heading; 
}

int main()
{
    magnetometer_task(NULL);
}