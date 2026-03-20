#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for one STEP/DIR/ENA motor output.
 */
typedef struct {
    const char *alias;          /**< Motor alias used by API and AT commands (max 7 chars). */
    int step_gpio_num;          /**< STEP output GPIO. */
    int dir_gpio_num;           /**< DIR output GPIO. */
    int enable_gpio_num;        /**< ENABLE output GPIO, or -1 when unused. */
    bool step_active_high;      /**< STEP active level, usually true for TB6600. */
    bool dir_positive_high;     /**< DIR level that represents positive movement. */
    bool enable_active_level;   /**< ENABLE active level for the external driver. */
    bool auto_enable;           /**< Assert ENABLE automatically during movement. */
    uint32_t step_pulse_us;     /**< STEP high pulse width in microseconds. */
    uint32_t dir_setup_us;      /**< Delay after changing DIR before first STEP. */
    uint32_t dir_hold_us;       /**< Reserved for future DIR hold timing. */
} esp_stepper_motor_config_t;

/**
 * @brief Runtime status snapshot for one motor.
 */
typedef struct {
    bool used;                  /**< Motor slot is allocated. */
    bool running;               /**< Motor is currently moving. */
    bool enabled;               /**< ENABLE output is active. */
    int32_t position;           /**< Current position in steps. */
    int32_t target_position;    /**< Target position in steps. */
    int32_t max_position;       /**< Maximum allowed position in steps, 0 when unlimited. */
    float current_speed_sps;    /**< Current estimated speed in steps/s. */
    float max_speed_sps;        /**< Configured maximum speed in steps/s. */
    float acceleration_sps2;    /**< Configured acceleration in steps/s^2. */
    float steps_per_mm;         /**< Mechanical conversion factor used by mm helpers. */
} esp_stepper_status_t;

/**
 * @brief Initialize the stepper engine.
 *
 * @param log_enabled Enable internal component logs.
 * @param at_enabled  Register `AT+STEP` and `AT+STEP?`.
 */
esp_err_t esp_stepper_init(bool log_enabled, bool at_enabled);

/**
 * @brief Deinitialize the stepper engine and release all motors.
 */
esp_err_t esp_stepper_deinit(void);

/**
 * @brief Report whether the engine is initialized.
 */
bool esp_stepper_is_initialized(void);

/**
 * @brief Return the maximum number of motors supported by this build.
 */
size_t esp_stepper_max_motors(void);

/**
 * @brief Add one motor driven by STEP/DIR/ENA.
 */
esp_err_t esp_stepper_add_motor(const esp_stepper_motor_config_t *config);

/**
 * @brief Remove one motor and release its resources.
 */
esp_err_t esp_stepper_remove_motor(const char *alias);

/**
 * @brief Return true if a motor with this alias exists.
 */
bool esp_stepper_has_motor(const char *alias);

/**
 * @brief Set the maximum speed in steps per second.
 */
esp_err_t esp_stepper_set_max_speed(const char *alias, float steps_per_second);

/**
 * @brief Set the acceleration in steps per second squared.
 */
esp_err_t esp_stepper_set_acceleration(const char *alias, float steps_per_second_squared);

/**
 * @brief Set the mechanical conversion factor in steps per millimeter.
 */
esp_err_t esp_stepper_set_steps_per_mm(const char *alias, float steps_per_mm);

/**
 * @brief Set the maximum allowed absolute position in steps.
 *
 * Use `0` to remove the limit.
 */
esp_err_t esp_stepper_set_max_position(const char *alias, int32_t max_position_steps);

/**
 * @brief Start a relative move in steps.
 */
esp_err_t esp_stepper_move(const char *alias, int32_t delta_steps);

/**
 * @brief Start an absolute move in steps.
 */
esp_err_t esp_stepper_move_to(const char *alias, int32_t target_position);

/**
 * @brief Start a relative move in millimeters.
 *
 * Requires a valid `steps_per_mm` value.
 */
esp_err_t esp_stepper_move_mm(const char *alias, float delta_mm);

/**
 * @brief Start an absolute move in millimeters.
 *
 * Requires a valid `steps_per_mm` value.
 */
esp_err_t esp_stepper_move_to_mm(const char *alias, float target_mm);

/**
 * @brief Stop a motor.
 *
 * @param hard True for immediate stop, false for decelerated stop.
 */
esp_err_t esp_stepper_stop(const char *alias, bool hard);

/**
 * @brief Manually set the ENABLE output.
 */
esp_err_t esp_stepper_set_enabled(const char *alias, bool enabled);

/**
 * @brief Fetch the current position in steps.
 */
esp_err_t esp_stepper_get_position(const char *alias, int32_t *out_position);

/**
 * @brief Return true if the motor is currently moving.
 */
bool esp_stepper_is_running(const char *alias);

/**
 * @brief Fetch a runtime status snapshot.
 */
esp_err_t esp_stepper_get_status(const char *alias, esp_stepper_status_t *out_status);

#ifdef __cplusplus
}
#endif
