# esp_stepper

`esp_stepper` is a lightweight non-blocking STEP/DIR/ENA motor engine for ESP-IDF.

Current scope:

- TB6600-style drivers
- trapezoidal acceleration
- asynchronous background motion
- one GPTimer per motor
- optional AT control

## Purpose

The component is meant to be the base motion engine for external drivers that expose:

- `STEP`
- `DIR`
- `ENABLE`

The first target is `TB6600`, but the public API stays generic enough for other step/dir drivers.

## Current model

- one motion command at a time per motor
- non-blocking execution in the background
- soft stop and hard stop
- optional mechanical conversion in `mm`
- optional position limit

## Public API

- `esp_stepper_init(log_enabled, at_enabled)`
- `esp_stepper_deinit()`
- `esp_stepper_is_initialized()`
- `esp_stepper_max_motors()`
- `esp_stepper_add_motor()`
- `esp_stepper_remove_motor()`
- `esp_stepper_set_max_speed()`
- `esp_stepper_set_acceleration()`
- `esp_stepper_set_steps_per_mm()`
- `esp_stepper_set_max_position()`
- `esp_stepper_move()`
- `esp_stepper_move_to()`
- `esp_stepper_move_mm()`
- `esp_stepper_move_to_mm()`
- `esp_stepper_stop()`
- `esp_stepper_set_enabled()`
- `esp_stepper_get_position()`
- `esp_stepper_is_running()`
- `esp_stepper_get_status()`

## Notes

- motion units are `steps` by default
- `mm` helpers require `steps_per_mm > 0`
- `max_position` clamps absolute targets to `0..max`
- the current MVP does not queue moves
- starting a new move while one is already running returns `ESP_ERR_INVALID_STATE`

## TB6600 baseline

Recommended initial values for `TB6600`-style drivers:

- `step_active_high = true`
- `dir_positive_high = true`
- `enable_active_level = false`
- `auto_enable = true`
- `step_pulse_us = 4`
- `dir_setup_us = 20`

## Minimal example

```c
ESP_ERROR_CHECK(esp_stepper_init(false, false));

const esp_stepper_motor_config_t x_cfg = {
    .alias = "X",
    .step_gpio_num = 17,
    .dir_gpio_num = 18,
    .enable_gpio_num = 19,
    .step_active_high = true,
    .dir_positive_high = true,
    .enable_active_level = false,
    .auto_enable = true,
    .step_pulse_us = 4,
    .dir_setup_us = 20,
    .dir_hold_us = 20,
};

ESP_ERROR_CHECK(esp_stepper_add_motor(&x_cfg));
ESP_ERROR_CHECK(esp_stepper_set_steps_per_mm("X", 91.43f));
ESP_ERROR_CHECK(esp_stepper_set_max_position("X", 109716));
ESP_ERROR_CHECK(esp_stepper_set_max_speed("X", 20000.0f));
ESP_ERROR_CHECK(esp_stepper_set_acceleration("X", 30000.0f));
ESP_ERROR_CHECK(esp_stepper_move_mm("X", 10.0f));
```
