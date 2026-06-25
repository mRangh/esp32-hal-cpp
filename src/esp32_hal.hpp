/*
 * ============================================================================
 * @file        esp32_hal.hpp
 * @brief       HARDWARE DRIVERS - handling hardware for ESP32 (Espressif-IDF)
 *
 * @author      Marco Antônio Ranghetti
 * @github      github.com/mRangh
 * @email       marcoantonioranghetti@gmail.com
 * @academic    d2026008956@unifei.edu.br
 *
 * @version     1.0.0
 * @date        2026-06-22
 * @license     Apache License 2.0
 * ============================================================================
 */

#ifndef ESP32_HAL_HPP
#define ESP32_HAL_HPP

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include <string>
#include <cstdio>

class DigitalInput {

    public:

        virtual ~DigitalInput() = default;
        DigitalInput(int port, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);
        virtual bool read();

    private:

        gpio_num_t _pin;
        gpio_pull_mode_t _pull_mode;

        void init();
};

class AnalogInput {

    public:

        virtual ~AnalogInput() = default;
        AnalogInput(int port);
        virtual int read();

    private:

        gpio_num_t _pin;
        static adc_oneshot_unit_handle_t _adc_handle;
        adc_channel_t _adc_channel;
        static bool _unit_initialized;

        void init();
};

class Output {

    public:

        Output(int port);
        Output& write(bool src);

    private:

        gpio_num_t _pin;

        void init();
};

class Switch : public DigitalInput {

    public:

        Switch(int port, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);
};

class Button : public DigitalInput {

    public:

        Button(int port, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);

        bool toggle();

    private:

        TickType_t _last_debounce_time = 0;
        bool _switch_state = 0;
        bool _last_state = 0;
        bool _actual_state = 0;
};

class LM393 : public DigitalInput {

    public:

        LM393(int port, gpio_pull_mode_t pull = GPIO_FLOATING);

        bool read() override;
}

class Servo {

    public:

        Servo(int port, ledc_channel_t ch = LEDC_CHANNEL_0);
        Servo& move(int angle);

    protected:

        int _current_angle;

    private:

        gpio_num_t _pin;
        ledc_channel_t _channel;
        static bool _timer_initialized;

        void init();
};

class Potentiometer : public AnalogInput {

    public:

        Potentiometer(int port);
};

class Ultrasonic {

    public:

        ~Ultrasonic();
        Ultrasonic(int trig_port, int echo_port);
        float read_cm();

    private:

        gpio_num_t _trig_pin;
        gpio_num_t _echo_pin;
        SemaphoreHandle_t _echo_semaphore;
        volatile uint64_t _echo_start;
        volatile uint64_t _echo_duration;

        static void IRAM_ATTR _gpio_isr_handler(void* arg);
        static bool _isr_service_installed;

        void init();
};

struct mfrc_522_config{
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t spics;
    gpio_num_t reset;
};
class MFRC_522{

    public:

        MFRC_522(const mfrc_522_config& config);
        bool check();
        std::string read_uid();
        void stop_reading();

    private:

        const mfrc_522_config _config;
        spi_device_handle_t _spi_handle;

        static bool _spi_bus_initialized;

        void write_regist(uint8_t reg, uint8_t value);
        uint8_t read_regist(uint8_t reg);
        void execute(uint8_t command);
        esp_err_t init();
};

#endif
