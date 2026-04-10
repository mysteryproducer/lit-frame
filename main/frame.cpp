/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <chrono>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include "timer.hpp"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "vl53l8cx_api.h"

using namespace std;
using namespace std::chrono_literals;

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO (APP_LEDC_OUTPUT_GPIO) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

#define I2C_MASTER_SCL_IO (gpio_num_t)CONFIG_APP_I2C_SCL_GPIO               //!< gpio number for I2C master clock 
#define I2C_MASTER_SDA_IO (gpio_num_t)CONFIG_APP_I2C_SDA_GPIO               //< gpio number for I2C master

static i2c_master_bus_handle_t  bus_handle;
static i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
//    .intr_priority = 0,
//    .trans_queue_depth = 0,
    .flags = {
        .enable_internal_pullup = true,
//        .allow_pd = true
    },
};
static VL53L8CX_Configuration 	Dev;			// Sensor configuration 
static VL53L8CX_ResultsData 	Results;		// Results data from VL53L8CX 

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 (rev < 3.0) targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = CONFIG_APP_LEDC_OUTPUT_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // Set duty to 0%
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void init_pin(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_pullup_en(pin);
    gpio_pulldown_dis(pin);
    gpio_set_level(pin, 1);
}

static esp_err_t init_vl53l8cx(const i2c_master_bus_config_t *i2c_mst_config, i2c_master_bus_handle_t *bus_handle)
{
    //Define the i2c device configuration
    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L8CX_DEFAULT_I2C_ADDRESS >> 1, // The API expects a 7-bit address, so we shift the default address
            .scl_speed_hz = VL53L8CX_MAX_CLK_SPEED
    };

    /*********************************/
    /*      Customer platform        */
    /*********************************/

    Dev.platform.bus_config = *i2c_mst_config;

    //Register the device
    if (i2c_master_bus_add_device(*bus_handle, &dev_cfg, &Dev.platform.handle) != ESP_OK) {
        printf("VL53L8CX device registration failed\n");
        return ESP_FAIL;
    }

    /* (Optional) Reset sensor */
    Dev.platform.reset_gpio = GPIO_NUM_5;
    if (VL53L8CX_Reset_Sensor(&(Dev.platform)) != ESP_OK) {
        printf("VL53L8CX reset failed\n");
    }

    /* (Optional) Set a new I2C address if the wanted address is different
    * from the default one (filled with 0x20 for this example).
    */
    //status = vl53l8cx_set_i2c_address(&Dev, 0x20);


    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L8CX sensor connected */
    uint8_t isAlive = 0;
    esp_err_t status = vl53l8cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        printf("VL53L8CX not detected at requested address (status: %i)\n", status);
        return (status == ESP_OK)?ESP_FAIL:status;
    }

    /* (Mandatory) Init VL53L8CX sensor */
    status = vl53l8cx_init(&Dev);
    if(status)
    {
        printf("VL53L8CX ULD Loading failed\n");
        return status;
    }

    printf("VL53L8CX ULD ready ! (Version : %s)\n",
           VL53L8CX_API_REVISION);


    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = vl53l8cx_start_ranging(&Dev);
    return status;
}

static int16_t read_min() {
    /* Use polling function to know when a new measurement is ready.
        * Another way can be to wait for HW interrupt raised on PIN A1
        * (INT) when a new measurement is ready */
    uint8_t isReady = 0;
    esp_err_t status = vl53l8cx_check_data_ready(&Dev, &isReady);

    if(isReady)
    {
        vl53l8cx_get_ranging_data(&Dev, &Results);

        /* As the sensor is set in 4x4 mode by default, we have a total
            * of 16 zones to print. For this example, only the data of first zone are
            * print */
//        printf("Print data no : %3u\n", Dev.streamcount);
        int16_t distance_mm = Results.distance_mm[0];
        for(int i = 1; i < 16; i++)
        {
//            printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
//                    i,
//                    Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i]
                distance_mm = min(distance_mm, Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
        }
//        printf("\n");
        return distance_mm;
    }
    return -1;
}

extern "C" void app_main(void)
{
    bool lidar_ok = false;
    ESP_LOGI("Frame","Starting i2c (scl: %d, sda: %d)", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_LOGI("Frame","Starting vl53l8cx");
    lidar_ok = init_vl53l8cx(&i2c_mst_config, &bus_handle) == ESP_OK;
    ESP_LOGI("Frame","Starting LED controller");
    ledc_init();
    
    std::function<bool ()> read_task_fn = []()
    {
        int16_t distance_mm = read_min();
        if (distance_mm < 0) {
//            ESP_LOGE("Frame", "Failed to read distance from VL53L8CX");
            return false;
        }
        int16_t pwm_duty = distance_mm * 8192 / 2000; // Map distance to duty cycle (assuming max distance is 2000mm)
        if (pwm_duty > 0) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pwm_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        }
        // we don't want to stop, so return false
        return false;
    };

    // Set duty to 0%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    espp::Timer timer({.period = 20ms,
                        .callback = read_task_fn,
                        .task_config =
                            {
                                .name = "VL53L8CX",
                                .stack_size_bytes{4 * 1024},
                            },
                        .log_level = espp::Logger::Verbosity::INFO
                    });
    // timer.start();
    // while (true)
    // {
    //     std::this_thread::sleep_for(100ms);
    //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (i*100) % 8192));
    //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    // }

    for (int i=0;i<20000000;++i)
    {
        // if (!lidar_ok) {
        //     ESP_LOGE("Frame", "VL53L8CX initialization failed, skipping reading");
        //     std::this_thread::sleep_for(1s);
        //     continue;
        // }
        // int16_t distance_mm = read_min();
        // int16_t pwm_duty = distance_mm * 8192 / 2000; // Map distance to duty cycle (assuming max distance is 2000mm)
        // if (pwm_duty > 0) {
        //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pwm_duty));
        //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        // }
        std::this_thread::sleep_for(20ms);
    }

}