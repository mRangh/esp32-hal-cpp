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

#ifdef __cplusplus

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "rom/ets_sys.h"
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>

class DigitalInput {

    public:

        gpio_num_t pin;
        gpio_pull_mode_t pull_mode;
        bool value;

        virtual ~DigitalInput() = default;
        DigitalInput(int port, gpio_pull_mode_t pull = GPIO_PULLDOWN_ONLY);
        void init();
        virtual int read();
};

class AnalogInput {
    
    public:
    
        gpio_num_t pin;
        int value;
        adc_oneshot_unit_handle_t adc_handle;
        adc_channel_t adc_channel;

        virtual ~AnalogInput() = default;
        AnalogInput(int port);
        void init();
        virtual int read();
};

class Output {

    public:

        gpio_num_t pin;
        bool value;

        Output(int port);
        void init();
        Output& write(bool src);
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

        TickType_t last_debounce_time = 0;
        bool switch_state = 0;
        bool last_state = 0;
        bool actual_state = 0;
};

class Servo {

    public:

        gpio_num_t pin;
        ledc_channel_t channel;
        int current_angle;

        Servo(int port, ledc_channel_t ch = LEDC_CHANNEL_0);
        Servo& move(int angle);
};

class Potentiometer : public AnalogInput {

    public:

        Potentiometer(int port);      
};

class Ultrasonic {

    public:
    
        Ultrasonic(int trig_port, int echo_port);
        void init() ;
        float read_cm();
    
    private:

        gpio_num_t trig_pin;
        gpio_num_t echo_pin;
        uint32_t timeout;
};

#endif
#endif
