/*
 * ============================================================================
 * @file        esp32_hal.cpp
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

#include "esp32_hal.hpp"

DigitalInput::DigitalInput(int port, gpio_pull_mode_t pull){
    pin = (gpio_num_t)port;
    pull_mode = pull;
    value = 0;
}

void DigitalInput::init(){
    gpio_config_t io_conf    = {};
    io_conf.intr_type        = GPIO_INTR_DISABLE;
    io_conf.mode             = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask     = (1ULL << pin);

    if (pull_mode == GPIO_PULLUP_ONLY){
        io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    } else if (pull_mode == GPIO_PULLDOWN_ONLY) {
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    } else {
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    } gpio_config(&io_conf);
}

int DigitalInput::read(){
    return gpio_get_level(pin);
}

AnalogInput::AnalogInput(int port){
    pin = (gpio_num_t)port;
    value = 0;
    adc_handle = nullptr;
}

void AnalogInput::init(){
    if (adc_handle != nullptr) return;
    if (pin == GPIO_NUM_32) adc_channel = ADC_CHANNEL_4;
    else if (pin == GPIO_NUM_33) adc_channel = ADC_CHANNEL_5;
    else if (pin == GPIO_NUM_34) adc_channel = ADC_CHANNEL_6;
    else if (pin == GPIO_NUM_35) adc_channel = ADC_CHANNEL_7;
    else adc_channel = ADC_CHANNEL_0;

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_12;
    config.atten = ADC_ATTEN_DB_12; 
    adc_oneshot_config_channel(adc_handle, adc_channel, &config);
}

int AnalogInput::read(){
    int raw_value = 0;
    adc_oneshot_read(adc_handle, adc_channel, &raw_value);
    value = raw_value;
    return value;
}

Output::Output(int port){
    pin = (gpio_num_t)port;
    value = false;
    init();
}

void Output::init(){
    gpio_config_t io_conf = {};
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask  = (1ULL << pin);
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

Output& Output::write(bool src){
    value = src;
    gpio_set_level(pin, value);
    return *this;
}

Switch::Switch(int port, gpio_pull_mode_t pull)
    : DigitalInput(port, pull) {}

Button::Button(int port, gpio_pull_mode_t pull)
    : DigitalInput(port, pull) {}

bool Button::toggle(){
    actual_state = read();
    TickType_t current_time = xTaskGetTickCount();
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);
    if (actual_state == 1 && last_state == 0) {
        if (current_time - last_debounce_time > debounce_delay) {
            switch_state = !switch_state;
            last_debounce_time = current_time;
        }
    }
    last_state = actual_state;
    value = switch_state;
    return value;
}

Servo::Servo(int port, ledc_channel_t ch) {
    pin = (gpio_num_t)port;
    channel = ch;
    current_angle = 0;

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode          = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num           = LEDC_TIMER_0;
    ledc_timer.duty_resolution     = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz             = 50;
    ledc_timer.clk_cfg             = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = channel;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.gpio_num       = pin;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

Servo& Servo::move(int angle){
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    current_angle = angle;

    uint32_t duty = 205 + ((angle * (1024 - 205))/180);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

    return *this;
}

Potentiometer::Potentiometer(int port) : AnalogInput(port) {}

Ultrasonic::Ultrasonic(int trig_port, int echo_port)
    : trig_pin((gpio_num_t)trig_port), echo_pin((gpio_num_t)echo_port), timeout(12000) {}

void Ultrasonic::init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask  = (1ULL << trig_pin);
    gpio_config(&io_conf);
    gpio_set_level(trig_pin, 0);

    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << echo_pin);
    gpio_config(&io_conf);
}

float Ultrasonic::read_cm() {
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    uint64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() - start_time > timeout) return 999.0f;
    }

    uint64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() - echo_start > timeout) return 999.0f;
    }
    uint64_t echo_duration = esp_timer_get_time() - echo_start;

    return (float)echo_duration * 0.0343f / 2.0f;
}
