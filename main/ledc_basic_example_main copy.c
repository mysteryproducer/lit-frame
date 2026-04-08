/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "vl53l.hpp"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 (rev < 3.0) targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
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

void app_main(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    //gpio_num_t testPin = GPIO_NUM_4;
}

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "VL53L4CX example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (vl53l)
  {
    logger.info("Starting example!");

    //! [vl53l example]
    // make the i2c we'll use to communicate
    static constexpr auto i2c_port = I2C_NUM_0;
    static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
    static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
    static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
    logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);
    logger.info("I2C clock speed: {} Hz", i2c_clock_speed);
    espp::I2c i2c({.port = i2c_port,
                   .sda_io_num = i2c_sda,
                   .scl_io_num = i2c_scl,
                   .sda_pullup_en = GPIO_PULLUP_ENABLE,
                   .scl_pullup_en = GPIO_PULLUP_ENABLE,
                   .clk_speed = i2c_clock_speed});

    // make the actual test object
    espp::Vl53l vl53l(
        espp::Vl53l::Config{.device_address = espp::Vl53l::DEFAULT_ADDRESS,
                            .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                               std::placeholders::_2, std::placeholders::_3),
                            .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                              std::placeholders::_2, std::placeholders::_3),
                            .log_level = espp::Logger::Verbosity::WARN});

    std::error_code ec;
    // set the timing budget to 10ms, which must be shorter than the
    // inter-measurement period. We'll log every 20ms so this guarantees we get
    // new data every time
    if (!vl53l.set_timing_budget_ms(10, ec)) {
      logger.error("Failed to set inter measurement period: {}", ec.message());
      return;
    }
    // set the inter-measurement period to 10ms, so we should be sure to get new
    // data each measurement
    if (!vl53l.set_inter_measurement_period_ms(10, ec)) {
      logger.error("Failed to set inter measurement period: {}", ec.message());
      return;
    }
    // tell it to start ranging
    if (!vl53l.start_ranging(ec)) {
      logger.error("Failed to start ranging: {}", ec.message());
      return;
    }

    // make the task which will read the vl53l
    fmt::print("%time (s), distance (m)\n");
    auto read_task_fn = [&vl53l]() {
      auto now = esp_timer_get_time();
      static auto start = now;
      float elapsed = (float)(now - start) / 1e6;
      std::error_code ec;
      // wait for the data to be ready
      while (!vl53l.is_data_ready(ec)) {
        std::this_thread::sleep_for(1ms);
      }
      // clear the interrupt so we can get another reading
      if (!vl53l.clear_interrupt(ec)) {
        logger.error("Failed to clear interrupt: {}", ec.message());
        return false;
      }
      auto meters = vl53l.get_distance_meters(ec);
      if (ec) {
        logger.error("Failed to get distance: {}", ec.message());
        return false;
      }
      fmt::print("{:.3f}, {:.3f}\n", elapsed, meters);
      // we don't want to stop, so return false
      return false;
    };

    espp::Timer timer({.period = 20ms,
                       .callback = read_task_fn,
                       .task_config =
                           {
                               .name = "VL53L4CX",
                               .stack_size_bytes{4 * 1024},
                           },
                       .log_level = espp::Logger::Verbosity::INFO});
    //! [vl53l example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}