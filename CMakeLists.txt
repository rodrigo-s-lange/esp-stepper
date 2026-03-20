idf_component_register(
    SRCS "esp_stepper.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_at esp_driver_gptimer esp_driver_gpio freertos esp_timer
)
