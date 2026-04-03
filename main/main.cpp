#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "VL53L0X.h"

#define I2C_SDA_PIN (gpio_num_t)22
#define I2C_SCL_PIN (gpio_num_t)20
#define I2C_PWR_PIN (gpio_num_t)7
#define I2C_PULLUP_PIN (gpio_num_t)8

// --- Calibration Settings ---
#define CALIBRATION_OFFSET_MM 20 // Subtract 20mm from the reading
#define SENSOR_POLL_MS 25        // ~40Hz update rate

extern "C" void app_main(void)
{
    printf("\n--- HIGH SPEED TOF STARTUP ---\n");

    // Power Gates
    gpio_reset_pin(I2C_PWR_PIN);
    gpio_set_direction(I2C_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PWR_PIN, 1);
    gpio_reset_pin(I2C_PULLUP_PIN);
    gpio_set_direction(I2C_PULLUP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PULLUP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    VL53L0X sensor(I2C_NUM_0);
    sensor.i2cMasterInit(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400kHz for speed

    if (!sensor.init())
    {
        printf("Init Failed!\n");
        return;
    }

    // SPEED TWEAK: Set timing budget to 20ms (High Speed Mode)
    // The VL53L0X minimum is roughly 20000 microseconds.
    sensor.setTimingBudget(20000);

    printf("High Speed Mode Engaged (20ms budget)...\n");

    while (1)
    {
        uint16_t raw_dist = 0;

        if (sensor.read(&raw_dist))
        {
            // Apply offset but don't let it go below zero
            int32_t calibrated_dist = (int32_t)raw_dist - CALIBRATION_OFFSET_MM;
            if (calibrated_dist < 0)
                calibrated_dist = 0;

            printf("Distance: %ld mm (Raw: %u)\n", calibrated_dist, raw_dist);
        }
        else
        {
            printf("Target Lost\n");
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}