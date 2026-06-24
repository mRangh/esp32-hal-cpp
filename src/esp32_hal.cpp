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

MFRC_522::MFRC_522(const mfrc_522_config& config) : _config(config), _spi_handle(nullptr) {}

esp_err_t MFRC_522::init() {
    esp_err_t ret;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num    = _config.mosi;
    buscfg.miso_io_num    = _config.miso;
    buscfg.sclk_io_num    = _config.sclk;
    buscfg.quadwp_io_num  = -1;
    buscfg.quadhd_io_num  = -1;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz  = 4 * 1000 * 1000;
    devcfg.mode            = 0;
    devcfg.spics_io_num    = _config.spics;
    devcfg.queue_size      = 7;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi_handle);
    if (ret != ESP_OK) return ret;

    gpio_reset_pin(_config.reset);
    gpio_set_direction(_config.reset, GPIO_MODE_OUTPUT);
    gpio_set_level(_config.reset, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    write_regist(CommandReg, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    write_regist(ModeReg, 0x3D);
    uint8_t tx_ctrl = read_regist(TxControlReg);
    if ((tx_ctrl & 0x03) != 0x03) {
        write_regist(TxControlReg, tx_ctrl | 0x03);
    }

    ESP_LOGI(TAG, "MFRC522 Initialized. Version: 0x%02X", read_regist(VersionReg));
    return ESP_OK;
}

void MFRC_522::write_regist(uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = (reg << 1) & 0x7E;
    data[1] = value;

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = data;
    spi_device_polling_transmit(_spi_handle, &t);
}

uint8_t MFRC_522::read_regist(uint8_t reg) {
    uint8_t tx_data[2] = { static_cast<uint8_t>(((reg << 1) & 0x7E) | 0x80), 0 };
    uint8_t rx_data[2] = {0};

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    spi_device_polling_transmit(_spi_handle, &t);

    return rx_data[1];
}

void MFRC_522::execute(uint8_t command) {
    write_regist(CommandReg, command);
}

bool MFRC_522::check() {
    execute(PCD_RECAL_IDLE);
    
    write_regist(FIFOLevelReg, 0x80); 
    
    write_regist(BitFramingReg, 0x07);
    
    write_regist(FIFODataReg, PICC_REQIDL);
    
    execute(PCD_TRANSCEIVE);
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    uint8_t n = read_regist(FIFOLevelReg);
    
    return (n > 0);
}

std::string MFRC_522::read_uid() {
    char uid_str[32];
    uint8_t uid_bytes[4] = {0x00, 0x00, 0x00, 0x00};
    
    uint8_t n = read_regist(FIFOLevelReg);
    
    if (n >= 4) {
        for (int i = 0; i < 4; i++) {
            uid_bytes[i] = read_regist(FIFODataReg);
        }
    } else {
        return "";
    }

    snprintf(uid_str, sizeof(uid_str), "%02X%02X%02X%02X", 
             uid_bytes[0], uid_bytes[1], uid_bytes[2], uid_bytes[3]);
             
    return std::string(uid_str);
}

void MFRC_522::stop_reading() {
    execute(PCD_RECAL_IDLE);
}
