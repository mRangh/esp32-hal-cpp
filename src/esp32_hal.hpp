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

#define CommandReg    0x01
#define FIFODataReg   0x09
#define FIFOLevelReg  0x0A
#define BitFramingReg 0x0D
#define ModeReg       0x14
#define TxControlReg  0x14
#define VersionReg    0x37

#define PCD_RECAL_IDLE  0x00
#define PCD_TRANSCEIVE  0x0C
#define PICC_REQIDL     0x26

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
        esp_err_t init();
        bool check();
        std::string read_uid();
        void stop_reading();
    
    private:

        const mfrc_522_config _config;
        spi_device_handle_t _spi_handle;

        void write_regist(uint8_t reg, uint8_t value);
        uint8_t read_regist(uint8_t reg);
        void execute(uint8_t command);
};

#endif
#endif
