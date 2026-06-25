# ESP32 HAL C++ Library

A lightweight, object-oriented C++ hardware abstraction layer for ESP32, built on top of **ESP-IDF**. This library provides reusable classes for digital and analog I/O, PWM servo control, ultrasonic distance sensing, and MFRC522 RFID reader support.

---

## Features

- Object-oriented C++ API with self-contained peripheral classes
- Digital input and output support
- Analog input support via ADC1 (12-bit)
- Debounced button toggle behavior
- PWM servo control using LEDC
- HC-SR04-compatible ultrasonic sensor support
- MFRC522 RFID reader support over SPI
- Automatic GPIO and peripheral initialization in constructors

---

## Requirements

- ESP32 board
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.x or later
- C++11 or later

---

## Installation

Copy `src/esp32_hal.cpp` and `src/esp32_hal.hpp` into your project's `main/`, `components/`, or `lib/` directory, then include it:

```cpp
#include <esp32_hal.hpp>
```

Ensure your `CMakeLists.txt` registers the component and requires the ESP-IDF components used by the library:

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc freertos esp_timer spi_master esp_log
)
```

---

## API Overview

### `DigitalInput`
Configures a GPIO pin as a digital input with optional pull resistor.

```cpp
DigitalInput input(GPIO_NUM_4, GPIO_PULLUP_ONLY);
int state = input.read();
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `port` | `int` | GPIO pin number |
| `pull` | `gpio_pull_mode_t` | `GPIO_PULLDOWN_ONLY`, `GPIO_PULLUP_ONLY`, or `GPIO_FLOATING`(default) |

---

### `AnalogInput`
Reads a 12-bit ADC value from ADC1.

```cpp
AnalogInput sensor(GPIO_NUM_34);
int raw = sensor.read();
```

Supported ADC pins: `GPIO_NUM_32 ~ 39`.

---

### `Output`
Configures a GPIO pin as a digital output.

```cpp
Output led(GPIO_NUM_2);
led.write(true);
```

`write()` returns a reference to the object, allowing method chaining.

```cpp
led.write(true).write(false);
```

---

### `Switch`
A semantic subclass of `DigitalInput` for on/off switch inputs.

```cpp
Switch sw(GPIO_NUM_5);
int value = sw.read();
```

---

### `Button`
A debounced digital input with toggle semantics. Adds **debounced toggle** logic — each press flips the internal state, with a 50 ms debounce delay.

```cpp
Button btn(GPIO_NUM_14);
btn.init();

// Call repeatedly in a loop:
bool toggled = btn.toggle(); // flips on each clean press
```

---

### `Servo`
Controls a standard PWM servo motor using the ESP-IDF LEDC peripheral (50 Hz, 13-bit resolution).

```cpp
Servo servo(GPIO_NUM_18, LEDC_CHANNEL_0);
servo.move(0);   // 0°
servo.move(90);  // 90°
servo.move(180); // 180°
```

`move()` clamps the angle to `[0, 180]` and returns `*this` for chaining:
```cpp
servo.move(45).move(135);
```

> **Note:** Multiple servos must use different `ledc_channel_t` values.

---

### `Potentiometer`
A semantic subclass of `AnalogInput` for potentiometer inputs.

```cpp
Potentiometer pot(GPIO_NUM_32);
int value = pot.read(); // 0 - 4095
```

---

### `Ultrasonic`
Reads distance in centimeters from an HC-SR04-compatible ultrasonic sensor. Returns `999.0f` on timeout.

```cpp
Ultrasonic sonar(GPIO_NUM_12, GPIO_NUM_13);
float distance = sonar.read_cm();
```
Default timeout: `30000 µs` (~5 m range).

---

### `MFRC_522`
Basic MFRC522 RFID reader support over SPI.

```cpp
mfrc_522_config config = {
    .mosi = GPIO_NUM_23,
    .miso = GPIO_NUM_19,
    .sclk = GPIO_NUM_18,
    .spics = GPIO_NUM_5,
    .reset = GPIO_NUM_22
};

MFRC_522 reader(config);
if (reader.check()) {
    std::string uid = reader.read_uid();
}
reader.stop_reading();
```

---

## Example

```cpp
#include <esp32_hal.hpp>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main() {
    Button btn(GPIO_NUM_14);
    Output led(GPIO_NUM_2);
    Servo servo(GPIO_NUM_18, LEDC_CHANNEL_0);
    Potentiometer pot(GPIO_NUM_32);
    Ultrasonic sonar(GPIO_NUM_12, GPIO_NUM_13);

    // Initialization for MFRC522
    mfrc_522_config rfid_cfg = {
        .mosi  = GPIO_NUM_23,
        .miso  = GPIO_NUM_19,
        .sclk  = GPIO_NUM_18,
        .spics = GPIO_NUM_5,
        .reset = GPIO_NUM_22
    };
    MFRC_522 rfid(rfid_cfg);
    
    if (rfid.init() != ESP_OK) {
        printf("Failed to initialize MFRC_522\n");
    }

    while (true) {
        // Toggle LED based on button press
        led.write(btn.toggle());

        // Read potentiometer and move servo
        int angle = (pot.read() * 180) / 4095;
        servo.move(angle);

        // Read ultrasonic sensor
        float dist = sonar.read_cm();
        printf("Distance: %.2f cm\n", dist);

        // Check for RFID cards
        if (rfid.check()) {
            std::string uid = rfid.read_uid();
            if (!uid.empty()) {
                printf("Card Detected! UID: %s\n", uid.c_str());
            }
            rfid.stop_reading();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## Build

Add the library source files to your component and include the required ESP-IDF dependencies.

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc freertos esp_timer spi_master esp_log
)
```

---

## License

Licensed under the [Apache License 2.0](LICENSE).

---

## Contributing

Contributions are welcome. Please open an issue or pull request for bug fixes, new peripheral drivers, or improvements.
