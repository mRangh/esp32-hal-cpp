# ESP32 - Hardware Abstaction Layer

A lightweight, object-oriented C++ hardware abstraction library for ESP32, built on top of **ESP-IDF**. Provides clean, reusable driver classes for common peripherals — digital I/O, analog inputs, PWM servos, and ultrasonic sensors — with minimal boilerplate.

---

## Features

- Object-oriented C++ API (classes, inheritance, method chaining)
- Supports digital inputs/outputs, analog reading, servo control, and ultrasonic distance sensing
- Built-in debounce logic for buttons
- Configurable pull-up/pull-down resistors

---

## Requirements

- ESP32 board (tested with ESP32 DevKit)
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.x or later
- C++11 or later

---

## Installation

Copy `esp32_hal.cpp` and `esp32_hal.hpp` into your project's `main/`, `components/` or `lib/` directory, then include it:

```cpp
#include <esp32_hal.hpp>
```

Make sure your `CMakeLists.txt` links the required ESP-IDF components:

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_adc
)
```

---

## Classes

### `DigitalInput`
Configures a GPIO pin as a digital input with optional pull-up or pull-down resistor.

```cpp
DigitalInput btn(GPIO_NUM_4, GPIO_PULLUP_ONLY);
btn.init();
int state = btn.read(); // returns 0 or 1
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `port` | `int` | GPIO pin number |
| `pull` | `gpio_pull_mode_t` | `GPIO_PULLDOWN_ONLY` (default), `GPIO_PULLUP_ONLY`, or `GPIO_FLOATING` |

---

### `AnalogInput`
Reads a 12-bit ADC value (0–4095) from a GPIO pin using `ADC_UNIT_1`.

```cpp
AnalogInput sensor(GPIO_NUM_34);
sensor.init();
int raw = sensor.read(); // returns 0–4095
```

Supported pins: `GPIO_NUM_32`, `GPIO_NUM_33`, `GPIO_NUM_34`, `GPIO_NUM_35`.

---

### `Output`
Configures a GPIO pin as a digital output. Automatically initializes on construction.

```cpp
Output led(GPIO_NUM_2);
led.write(true);  // HIGH
led.write(false); // LOW
```

`write()` returns a reference to itself, enabling method chaining:
```cpp
led.write(true).write(false);
```

---

### `Switch`
Extends `DigitalInput`. Represents a simple on/off switch. Same API as `DigitalInput`.

```cpp
Switch sw(GPIO_NUM_5);
sw.init();
bool state = sw.read();
```

---

### `Button`
Extends `DigitalInput`. Adds **debounced toggle** logic — each press flips the internal state, with a 50 ms debounce delay.

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
Extends `AnalogInput`. Reads a potentiometer's wiper position as a raw 12-bit value.

```cpp
Potentiometer pot(GPIO_NUM_32);
pot.init();
int raw = pot.read(); // 0–4095
```

---

### `Ultrasonic`
Reads distance in centimeters using an **HC-SR04** (or compatible) ultrasonic sensor. Returns `999.0` on timeout.

```cpp
Ultrasonic sensor(GPIO_NUM_12, GPIO_NUM_13); // trig, echo
sensor.init();
float distance = sensor.read_cm();
```

Default timeout: `12000 µs` (~2 m range).

---

## Example

```cpp
#include "hardware_drivers.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main() {
    Button btn(GPIO_NUM_14);
    btn.init();

    Output led(GPIO_NUM_2);
    Servo servo(GPIO_NUM_18, LEDC_CHANNEL_0);

    Potentiometer pot(GPIO_NUM_32);
    pot.init();

    Ultrassonic sonar(GPIO_NUM_12, GPIO_NUM_13);
    sonar.init();

    while (true) {
        led.write(btn.toggle());

        int angle = (pot.read() * 180) / 4095;
        servo.move(angle);

        float dist = sonar.read_cm();
        printf("Distance: %.2f cm\n", dist);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## License

Licensed under the [Apache License 2.0](LICENSE).

---

## Contributing

Contributions are welcome. Please open an issue or pull request for bug fixes, new peripheral drivers, or improvements.