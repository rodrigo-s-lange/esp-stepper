# Examples

## Basic TB6600 axis

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

## Runtime + AT

```c
ESP_ERROR_CHECK(esp_at_init(false));
ESP_ERROR_CHECK(esp_runtime_init(false, true));
ESP_ERROR_CHECK(esp_runtime_enable(ESP_RUNTIME_MODULE_STEPPER, false, true));
```

Then:

```text
AT+STEP=ADD,X,17,18,19
AT+STEP=X,SPMM,91.43
AT+STEP=X,MAXMM,1200
AT+STEP=X,SPEED,20000
AT+STEP=X,ACCEL,30000
AT+STEP=X,MOVEMM,10
AT+STEP?
```
