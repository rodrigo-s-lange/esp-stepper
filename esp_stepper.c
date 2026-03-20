#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_at.h"
#include "esp_log.h"
#include "esp_stepper.h"

#define STEPPER_MAX_MOTORS           4
#define STEPPER_ALIAS_MAX_LEN        7
#define STEPPER_TIMER_RESOLUTION_HZ  1000000UL
#define STEPPER_DEFAULT_SPEED_SPS    1000.0f
#define STEPPER_DEFAULT_ACCEL_SPS2   1000.0f
#define STEPPER_MIN_SPEED_SPS        1.0f

static const char *TAG = "esp_stepper";

typedef struct {
    bool used;
    bool timer_started;
    bool running;
    bool stop_requested;
    bool hard_stop_requested;
    bool pulse_active;
    bool enabled;
    bool step_active_high;
    bool dir_positive_high;
    bool enable_active_level;
    bool auto_enable;
    int step_gpio_num;
    int dir_gpio_num;
    int enable_gpio_num;
    char alias[STEPPER_ALIAS_MAX_LEN + 1];
    uint32_t step_pulse_us;
    uint32_t dir_setup_us;
    uint32_t dir_hold_us;
    int32_t position;
    int32_t target_position;
    int32_t max_position_steps;
    int dir_sign;
    float current_speed_sps;
    float max_speed_sps;
    float acceleration_sps2;
    float steps_per_mm;
    gptimer_handle_t timer;
} stepper_motor_t;

static bool s_init_done = false;
static bool s_log_enabled = false;
static bool s_at_enabled = false;
static stepper_motor_t s_motors[STEPPER_MAX_MOTORS] = {0};
static portMUX_TYPE s_stepper_mux = portMUX_INITIALIZER_UNLOCKED;

#define STEP_LOGI(...) do { if (s_log_enabled) ESP_LOGI(TAG, __VA_ARGS__); } while (0)
#define STEP_LOGW(...) do { if (s_log_enabled) ESP_LOGW(TAG, __VA_ARGS__); } while (0)
#define STEP_LOGE(...) do { if (s_log_enabled) ESP_LOGE(TAG, __VA_ARGS__); } while (0)

static bool alias_is_valid(const char *alias)
{
    if (alias == NULL || alias[0] == '\0') return false;
    size_t len = strlen(alias);
    if (len == 0 || len > STEPPER_ALIAS_MAX_LEN) return false;
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)alias[i];
        if (!(isalnum(c) || c == '_' || c == '-')) return false;
    }
    return true;
}

static bool ci_equals(const char *a, const char *b)
{
    if (a == NULL || b == NULL) return false;
    while (*a != '\0' && *b != '\0') {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static char *trim_ws(char *s)
{
    if (s == NULL) return NULL;
    while (*s != '\0' && isspace((unsigned char)*s)) s++;
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[n - 1] = '\0';
        n--;
    }
    return s;
}

static bool parse_i32(const char *text, int32_t *out_value)
{
    if (text == NULL || out_value == NULL) return false;
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || *end != '\0') return false;
    if (value < INT32_MIN || value > INT32_MAX) return false;
    *out_value = (int32_t)value;
    return true;
}

static bool parse_float_value(const char *text, float *out_value)
{
    if (text == NULL || out_value == NULL) return false;
    char *end = NULL;
    float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;
    *out_value = value;
    return true;
}

static int32_t round_to_i32(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static bool steps_from_mm(stepper_motor_t *motor, float mm, int32_t *out_steps)
{
    if (motor == NULL || out_steps == NULL) return false;
    if (motor->steps_per_mm <= 0.0f) return false;
    float steps_f = mm * motor->steps_per_mm;
    if (steps_f < (float)INT32_MIN || steps_f > (float)INT32_MAX) return false;
    *out_steps = round_to_i32(steps_f);
    return true;
}

static esp_err_t validate_target_position(stepper_motor_t *motor, int32_t target_position)
{
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->max_position_steps > 0 && (target_position < 0 || target_position > motor->max_position_steps)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static stepper_motor_t *find_motor(const char *alias)
{
    if (alias == NULL) return NULL;
    for (size_t i = 0; i < STEPPER_MAX_MOTORS; i++) {
        if (s_motors[i].used && strcmp(s_motors[i].alias, alias) == 0) {
            return &s_motors[i];
        }
    }
    return NULL;
}

static stepper_motor_t *find_free_motor(void)
{
    for (size_t i = 0; i < STEPPER_MAX_MOTORS; i++) {
        if (!s_motors[i].used) return &s_motors[i];
    }
    return NULL;
}

static inline void set_step_level(stepper_motor_t *motor, bool active)
{
    int level = active ? (motor->step_active_high ? 1 : 0) : (motor->step_active_high ? 0 : 1);
    gpio_set_level(motor->step_gpio_num, level);
}

static inline void set_dir_level(stepper_motor_t *motor, int dir_sign)
{
    int positive_level = motor->dir_positive_high ? 1 : 0;
    int level = (dir_sign >= 0) ? positive_level : !positive_level;
    gpio_set_level(motor->dir_gpio_num, level);
}

static inline void set_enable_level(stepper_motor_t *motor, bool enabled)
{
    if (motor->enable_gpio_num < 0) {
        motor->enabled = enabled;
        return;
    }
    gpio_set_level(motor->enable_gpio_num, enabled ? (motor->enable_active_level ? 1 : 0)
                                                    : (motor->enable_active_level ? 0 : 1));
    motor->enabled = enabled;
}

static uint32_t clamp_interval_us(uint32_t value, uint32_t pulse_us)
{
    if (value <= pulse_us + 1U) return pulse_us + 2U;
    return value;
}

static uint32_t initial_interval_us(stepper_motor_t *motor)
{
    float accel = motor->acceleration_sps2;
    if (accel < 1.0f) accel = 1.0f;
    float interval = sqrtf(2.0f / accel) * 1000000.0f;
    if (interval < (float)(motor->step_pulse_us + 2U)) interval = (float)(motor->step_pulse_us + 2U);
    return (uint32_t)interval;
}

static uint32_t compute_next_interval_us(stepper_motor_t *motor)
{
    int32_t remaining = motor->target_position - motor->position;
    float accel = motor->acceleration_sps2;
    float speed = motor->current_speed_sps;
    float max_speed = motor->max_speed_sps;

    if (remaining == 0) {
        motor->current_speed_sps = 0.0f;
        return 0U;
    }

    if (accel < 1.0f) accel = 1.0f;
    if (max_speed < STEPPER_MIN_SPEED_SPS) max_speed = STEPPER_MIN_SPEED_SPS;

    if (speed < STEPPER_MIN_SPEED_SPS) {
        uint32_t interval = initial_interval_us(motor);
        speed = 1000000.0f / (float)interval;
        if (speed > max_speed) speed = max_speed;
        motor->current_speed_sps = speed;
        return interval;
    }

    float dt = 1.0f / speed;
    float stopping_steps = (speed * speed) / (2.0f * accel);
    bool decel = motor->stop_requested || ((float)labs((long)remaining) <= stopping_steps + 1.0f);

    if (decel) {
        speed -= accel * dt;
        if (speed < STEPPER_MIN_SPEED_SPS) speed = STEPPER_MIN_SPEED_SPS;
    } else {
        speed += accel * dt;
        if (speed > max_speed) speed = max_speed;
    }

    motor->current_speed_sps = speed;
    return clamp_interval_us((uint32_t)(1000000.0f / speed), motor->step_pulse_us);
}

static void stop_motor_now(stepper_motor_t *motor)
{
    motor->running = false;
    motor->stop_requested = false;
    motor->hard_stop_requested = false;
    motor->pulse_active = false;
    motor->timer_started = false;
    motor->current_speed_sps = 0.0f;
    set_step_level(motor, false);
    (void)gptimer_stop(motor->timer);
    (void)gptimer_set_raw_count(motor->timer, 0);
    (void)gptimer_set_alarm_action(motor->timer, NULL);
    if (motor->auto_enable) {
        set_enable_level(motor, false);
    }
}

static bool on_stepper_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    stepper_motor_t *motor = (stepper_motor_t *)user_ctx;
    gptimer_alarm_config_t next_alarm = {0};

    portENTER_CRITICAL_ISR(&s_stepper_mux);

    if (!motor->running) {
        portEXIT_CRITICAL_ISR(&s_stepper_mux);
        return false;
    }

    if (motor->hard_stop_requested) {
        stop_motor_now(motor);
        portEXIT_CRITICAL_ISR(&s_stepper_mux);
        return false;
    }

    if (!motor->pulse_active) {
        if (motor->position == motor->target_position) {
            stop_motor_now(motor);
            portEXIT_CRITICAL_ISR(&s_stepper_mux);
            return false;
        }

        set_step_level(motor, true);
        motor->pulse_active = true;
        next_alarm.alarm_count = edata->alarm_value + motor->step_pulse_us;
        (void)gptimer_set_alarm_action(timer, &next_alarm);
        portEXIT_CRITICAL_ISR(&s_stepper_mux);
        return false;
    }

    set_step_level(motor, false);
    motor->pulse_active = false;
    motor->position += motor->dir_sign;

    if (motor->position == motor->target_position) {
        stop_motor_now(motor);
        portEXIT_CRITICAL_ISR(&s_stepper_mux);
        return false;
    }

    uint32_t full_interval_us = compute_next_interval_us(motor);
    if (full_interval_us == 0U) {
        stop_motor_now(motor);
        portEXIT_CRITICAL_ISR(&s_stepper_mux);
        return false;
    }

    uint32_t low_time_us = (full_interval_us > motor->step_pulse_us) ? (full_interval_us - motor->step_pulse_us) : 1U;
    next_alarm.alarm_count = edata->alarm_value + low_time_us;
    (void)gptimer_set_alarm_action(timer, &next_alarm);

    portEXIT_CRITICAL_ISR(&s_stepper_mux);
    return false;
}

static esp_err_t create_timer(stepper_motor_t *motor)
{
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = STEPPER_TIMER_RESOLUTION_HZ,
        .intr_priority = 0,
    };
    esp_err_t err = gptimer_new_timer(&timer_cfg, &motor->timer);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_stepper_alarm,
    };
    err = gptimer_register_event_callbacks(motor->timer, &cbs, motor);
    if (err != ESP_OK) {
        (void)gptimer_del_timer(motor->timer);
        motor->timer = NULL;
        return err;
    }

    err = gptimer_enable(motor->timer);
    if (err != ESP_OK) {
        (void)gptimer_del_timer(motor->timer);
        motor->timer = NULL;
        return err;
    }

    return ESP_OK;
}

static void release_motor(stepper_motor_t *motor)
{
    if (motor == NULL || !motor->used) return;

    portENTER_CRITICAL(&s_stepper_mux);
    stop_motor_now(motor);
    portEXIT_CRITICAL(&s_stepper_mux);

    if (motor->timer != NULL) {
        (void)gptimer_disable(motor->timer);
        (void)gptimer_del_timer(motor->timer);
    }

    if (motor->enable_gpio_num >= 0) {
        gpio_reset_pin(motor->enable_gpio_num);
    }
    gpio_reset_pin(motor->dir_gpio_num);
    gpio_reset_pin(motor->step_gpio_num);

    *motor = (stepper_motor_t){0};
}

static esp_err_t start_move_locked(stepper_motor_t *motor, int32_t target_position)
{
    if (motor->running) return ESP_ERR_INVALID_STATE;
    esp_err_t range_err = validate_target_position(motor, target_position);
    if (range_err != ESP_OK) return range_err;
    if (target_position == motor->position) return ESP_OK;

    motor->target_position = target_position;
    motor->dir_sign = (target_position > motor->position) ? 1 : -1;
    motor->stop_requested = false;
    motor->hard_stop_requested = false;
    motor->pulse_active = false;
    motor->current_speed_sps = 0.0f;

    set_dir_level(motor, motor->dir_sign);
    set_step_level(motor, false);
    if (motor->auto_enable) {
        set_enable_level(motor, true);
    }

    uint32_t first_interval = initial_interval_us(motor);
    if (first_interval < motor->dir_setup_us) {
        first_interval = motor->dir_setup_us;
    }

    gptimer_alarm_config_t alarm = {
        .alarm_count = first_interval,
    };

    esp_err_t err = gptimer_set_raw_count(motor->timer, 0);
    if (err != ESP_OK) return err;
    err = gptimer_set_alarm_action(motor->timer, &alarm);
    if (err != ESP_OK) return err;
    err = gptimer_start(motor->timer);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    motor->timer_started = true;
    motor->running = true;
    motor->enabled = (motor->enable_gpio_num < 0) ? true : motor->enabled;
    STEP_LOGI("%s move start pos=%ld target=%ld", motor->alias, (long)motor->position, (long)target_position);
    return ESP_OK;
}

static esp_err_t register_at_commands(void);
static void unregister_at_commands(void);
static void handle_at_step_query(const char *param);
static void handle_at_step(const char *param);

esp_err_t esp_stepper_init(bool log_enabled, bool at_enabled)
{
    if (s_init_done) return ESP_ERR_INVALID_STATE;
    if (at_enabled && !esp_at_is_initialized()) return ESP_ERR_INVALID_STATE;

    s_log_enabled = log_enabled;
    s_at_enabled = at_enabled;
    s_init_done = true;

    if (s_at_enabled) {
        esp_err_t err = register_at_commands();
        if (err != ESP_OK) {
            s_init_done = false;
            s_log_enabled = false;
            s_at_enabled = false;
            return err;
        }
    }

    STEP_LOGI("initialized");
    return ESP_OK;
}

esp_err_t esp_stepper_deinit(void)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;

    unregister_at_commands();
    for (size_t i = 0; i < STEPPER_MAX_MOTORS; i++) {
        release_motor(&s_motors[i]);
    }

    s_log_enabled = false;
    s_at_enabled = false;
    s_init_done = false;
    return ESP_OK;
}

bool esp_stepper_is_initialized(void)
{
    return s_init_done;
}

size_t esp_stepper_max_motors(void)
{
    return STEPPER_MAX_MOTORS;
}

esp_err_t esp_stepper_add_motor(const esp_stepper_motor_config_t *config)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (config == NULL || !alias_is_valid(config->alias)) return ESP_ERR_INVALID_ARG;
    if (config->step_gpio_num < 0 || config->dir_gpio_num < 0) return ESP_ERR_INVALID_ARG;
    if (config->step_pulse_us == 0U) return ESP_ERR_INVALID_ARG;
    if (find_motor(config->alias) != NULL) return ESP_ERR_INVALID_STATE;

    stepper_motor_t *motor = find_free_motor();
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    gpio_config_t io_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    io_cfg.pin_bit_mask = 1ULL << config->step_gpio_num;
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) return err;

    io_cfg.pin_bit_mask = 1ULL << config->dir_gpio_num;
    err = gpio_config(&io_cfg);
    if (err != ESP_OK) return err;

    if (config->enable_gpio_num >= 0) {
        io_cfg.pin_bit_mask = 1ULL << config->enable_gpio_num;
        err = gpio_config(&io_cfg);
        if (err != ESP_OK) return err;
    }

    *motor = (stepper_motor_t){
        .used = true,
        .step_gpio_num = config->step_gpio_num,
        .dir_gpio_num = config->dir_gpio_num,
        .enable_gpio_num = config->enable_gpio_num,
        .step_active_high = config->step_active_high,
        .dir_positive_high = config->dir_positive_high,
        .enable_active_level = config->enable_active_level,
        .auto_enable = config->auto_enable,
        .step_pulse_us = config->step_pulse_us,
        .dir_setup_us = config->dir_setup_us,
        .dir_hold_us = config->dir_hold_us,
        .max_speed_sps = STEPPER_DEFAULT_SPEED_SPS,
        .acceleration_sps2 = STEPPER_DEFAULT_ACCEL_SPS2,
        .steps_per_mm = 1.0f,
    };
    strncpy(motor->alias, config->alias, sizeof(motor->alias) - 1U);
    motor->alias[sizeof(motor->alias) - 1U] = '\0';

    set_step_level(motor, false);
    set_dir_level(motor, 1);
    set_enable_level(motor, false);

    err = create_timer(motor);
    if (err != ESP_OK) {
        release_motor(motor);
        return err;
    }

    STEP_LOGI("motor %s added step=%d dir=%d ena=%d", motor->alias, motor->step_gpio_num, motor->dir_gpio_num, motor->enable_gpio_num);
    return ESP_OK;
}

esp_err_t esp_stepper_remove_motor(const char *alias)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    release_motor(motor);
    return ESP_OK;
}

bool esp_stepper_has_motor(const char *alias)
{
    return find_motor(alias) != NULL;
}

esp_err_t esp_stepper_set_max_speed(const char *alias, float steps_per_second)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (steps_per_second < STEPPER_MIN_SPEED_SPS) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    if (motor->running) return ESP_ERR_INVALID_STATE;
    motor->max_speed_sps = steps_per_second;
    return ESP_OK;
}

esp_err_t esp_stepper_set_acceleration(const char *alias, float steps_per_second_squared)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (steps_per_second_squared < 1.0f) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    if (motor->running) return ESP_ERR_INVALID_STATE;
    motor->acceleration_sps2 = steps_per_second_squared;
    return ESP_OK;
}

esp_err_t esp_stepper_set_steps_per_mm(const char *alias, float steps_per_mm)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (steps_per_mm <= 0.0f) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    if (motor->running) return ESP_ERR_INVALID_STATE;
    motor->steps_per_mm = steps_per_mm;
    return ESP_OK;
}

esp_err_t esp_stepper_set_max_position(const char *alias, int32_t max_position_steps)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (max_position_steps < 0) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    if (motor->running) return ESP_ERR_INVALID_STATE;
    motor->max_position_steps = max_position_steps;
    if (motor->position > max_position_steps) {
        motor->position = max_position_steps;
    }
    if (motor->target_position > max_position_steps) {
        motor->target_position = max_position_steps;
    }
    return ESP_OK;
}

esp_err_t esp_stepper_move(const char *alias, int32_t delta_steps)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    portENTER_CRITICAL(&s_stepper_mux);
    esp_err_t err = start_move_locked(motor, motor->position + delta_steps);
    portEXIT_CRITICAL(&s_stepper_mux);
    return err;
}

esp_err_t esp_stepper_move_to(const char *alias, int32_t target_position)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    portENTER_CRITICAL(&s_stepper_mux);
    esp_err_t err = start_move_locked(motor, target_position);
    portEXIT_CRITICAL(&s_stepper_mux);
    return err;
}

esp_err_t esp_stepper_move_mm(const char *alias, float delta_mm)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    int32_t delta_steps = 0;
    if (!steps_from_mm(motor, delta_mm, &delta_steps)) return ESP_ERR_INVALID_STATE;
    return esp_stepper_move(alias, delta_steps);
}

esp_err_t esp_stepper_move_to_mm(const char *alias, float target_mm)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    int32_t target_steps = 0;
    if (!steps_from_mm(motor, target_mm, &target_steps)) return ESP_ERR_INVALID_STATE;
    return esp_stepper_move_to(alias, target_steps);
}

esp_err_t esp_stepper_stop(const char *alias, bool hard)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    portENTER_CRITICAL(&s_stepper_mux);
    if (hard) {
        motor->hard_stop_requested = true;
    } else if (motor->running) {
        motor->stop_requested = true;
    }
    portEXIT_CRITICAL(&s_stepper_mux);
    return ESP_OK;
}

esp_err_t esp_stepper_set_enabled(const char *alias, bool enabled)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    if (motor->running && !enabled) return ESP_ERR_INVALID_STATE;
    set_enable_level(motor, enabled);
    return ESP_OK;
}

esp_err_t esp_stepper_get_position(const char *alias, int32_t *out_position)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (out_position == NULL) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;
    portENTER_CRITICAL(&s_stepper_mux);
    *out_position = motor->position;
    portEXIT_CRITICAL(&s_stepper_mux);
    return ESP_OK;
}

bool esp_stepper_is_running(const char *alias)
{
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return false;
    return motor->running;
}

esp_err_t esp_stepper_get_status(const char *alias, esp_stepper_status_t *out_status)
{
    if (!s_init_done) return ESP_ERR_INVALID_STATE;
    if (out_status == NULL) return ESP_ERR_INVALID_ARG;
    stepper_motor_t *motor = find_motor(alias);
    if (motor == NULL) return ESP_ERR_NOT_FOUND;

    portENTER_CRITICAL(&s_stepper_mux);
    *out_status = (esp_stepper_status_t){
        .used = motor->used,
        .running = motor->running,
        .enabled = motor->enabled,
        .position = motor->position,
        .target_position = motor->target_position,
        .max_position = motor->max_position_steps,
        .current_speed_sps = motor->current_speed_sps,
        .max_speed_sps = motor->max_speed_sps,
        .acceleration_sps2 = motor->acceleration_sps2,
        .steps_per_mm = motor->steps_per_mm,
    };
    portEXIT_CRITICAL(&s_stepper_mux);
    return ESP_OK;
}

static void print_motor_status(stepper_motor_t *motor)
{
    esp_stepper_status_t st = {0};
    if (esp_stepper_get_status(motor->alias, &st) != ESP_OK) return;

    AT(C "  %s" W " : run=%s ena=%s pos=%ld target=%ld max=%ld v=%.1f vmax=%.1f a=%.1f spmm=%.2f",
       motor->alias,
       st.running ? G "TRUE" W : R "FALSE" W,
       st.enabled ? "TRUE" : "FALSE",
       (long)st.position,
       (long)st.target_position,
       (long)st.max_position,
       (double)st.current_speed_sps,
       (double)st.max_speed_sps,
       (double)st.acceleration_sps2,
       (double)st.steps_per_mm);
}

static esp_err_t register_at_commands(void)
{
    esp_err_t err = esp_at_register_cmd_example("AT+STEP?", handle_at_step_query, "AT+STEP?");
    if (err != ESP_OK) return err;
    err = esp_at_register_cmd_example("AT+STEP", handle_at_step, "AT+STEP=ADD,X,12,13,14");
    if (err != ESP_OK) {
        (void)esp_at_unregister_cmd("AT+STEP?");
        return err;
    }
    err = esp_at_set_help_visible("AT+STEP", false);
    if (err != ESP_OK) {
        (void)esp_at_unregister_cmd("AT+STEP");
        (void)esp_at_unregister_cmd("AT+STEP?");
        return err;
    }
    return ESP_OK;
}

static void unregister_at_commands(void)
{
    if (!s_at_enabled || !esp_at_is_initialized()) return;
    (void)esp_at_unregister_cmd("AT+STEP?");
    (void)esp_at_unregister_cmd("AT+STEP");
}

static void handle_at_step_query(const char *param)
{
    (void)param;
    unsigned used_count = 0;
    for (size_t i = 0; i < STEPPER_MAX_MOTORS; i++) {
        if (s_motors[i].used) used_count++;
    }

    AT(C "Stepper:");
    AT(C "  init          : " W "%s", s_init_done ? "TRUE" : "FALSE");
    AT(C "  used          : " W "%u / %u", used_count, (unsigned)STEPPER_MAX_MOTORS);
    for (size_t i = 0; i < STEPPER_MAX_MOTORS; i++) {
        if (s_motors[i].used) {
            print_motor_status(&s_motors[i]);
        }
    }
    AT(C "Usage:");
    AT(C "  AT+STEP=ADD,<ALIAS>,<STEP>,<DIR>,<ENA|-1>");
    AT(C "  AT+STEP=<ALIAS>,SPEED,<steps_s>");
    AT(C "  AT+STEP=<ALIAS>,ACCEL,<steps_s2>");
    AT(C "  AT+STEP=<ALIAS>,SPMM,<steps_per_mm>");
    AT(C "  AT+STEP=<ALIAS>,MAX,<max_steps>");
    AT(C "  AT+STEP=<ALIAS>,MAXMM,<max_mm>");
    AT(C "  AT+STEP=<ALIAS>,MOVE,<delta>");
    AT(C "  AT+STEP=<ALIAS>,MOVETO,<pos>");
    AT(C "  AT+STEP=<ALIAS>,MOVEMM,<delta_mm>");
    AT(C "  AT+STEP=<ALIAS>,GOTO,<pos_mm>");
    AT(C "  AT+STEP=<ALIAS>,STOP");
    AT(C "  AT+STEP=<ALIAS>,ESTOP");
    AT(C "  AT+STEP=<ALIAS>,ENA,ON|OFF");
}

static void handle_at_step(const char *param)
{
    if (param == NULL || param[0] == '\0') {
        AT(R "ERROR: use AT+STEP=ADD,... or AT+STEP=<ALIAS>,...");
        return;
    }

    char work[128];
    if (strlen(param) >= sizeof(work)) {
        AT(R "ERROR: comando muito longo");
        return;
    }
    strncpy(work, param, sizeof(work) - 1U);
    work[sizeof(work) - 1U] = '\0';

    char *tokens[5] = {0};
    int token_count = 0;
    char *ctx = NULL;
    char *tok = strtok_r(work, ",", &ctx);
    while (tok != NULL && token_count < 5) {
        tokens[token_count++] = trim_ws(tok);
        tok = strtok_r(NULL, ",", &ctx);
    }

    if (token_count == 5 && ci_equals(tokens[0], "ADD")) {
        int32_t step_gpio = 0;
        int32_t dir_gpio = 0;
        int32_t ena_gpio = -1;
        if (!alias_is_valid(tokens[1]) || !parse_i32(tokens[2], &step_gpio) || !parse_i32(tokens[3], &dir_gpio) || !parse_i32(tokens[4], &ena_gpio)) {
            AT(R "ERROR: argumentos invalidos");
            return;
        }
        esp_stepper_motor_config_t cfg = {
            .alias = tokens[1],
            .step_gpio_num = (int)step_gpio,
            .dir_gpio_num = (int)dir_gpio,
            .enable_gpio_num = (int)ena_gpio,
            .step_active_high = true,
            .dir_positive_high = true,
            .enable_active_level = false,
            .auto_enable = true,
            .step_pulse_us = 4,
            .dir_setup_us = 20,
            .dir_hold_us = 20,
        };
        esp_err_t err = esp_stepper_add_motor(&cfg);
        if (err != ESP_OK) {
            AT(R "ERROR: add failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (token_count < 2) {
        AT(R "ERROR: sintaxe invalida");
        return;
    }

    const char *alias = tokens[0];
    const char *action = tokens[1];

    if (ci_equals(action, "STOP")) {
        esp_err_t err = esp_stepper_stop(alias, false);
        if (err != ESP_OK) {
            AT(R "ERROR: stop failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "ESTOP")) {
        esp_err_t err = esp_stepper_stop(alias, true);
        if (err != ESP_OK) {
            AT(R "ERROR: estop failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "ENA") && token_count == 3) {
        bool enable = ci_equals(tokens[2], "ON") || ci_equals(tokens[2], "1") || ci_equals(tokens[2], "TRUE");
        bool disable = ci_equals(tokens[2], "OFF") || ci_equals(tokens[2], "0") || ci_equals(tokens[2], "FALSE");
        if (!enable && !disable) {
            AT(R "ERROR: use ENA,ON|OFF");
            return;
        }
        esp_err_t err = esp_stepper_set_enabled(alias, enable);
        if (err != ESP_OK) {
            AT(R "ERROR: ena failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (token_count != 3) {
        AT(R "ERROR: sintaxe invalida");
        return;
    }

    if (ci_equals(action, "SPEED")) {
        float value = 0.0f;
        if (!parse_float_value(tokens[2], &value)) {
            AT(R "ERROR: velocidade invalida");
            return;
        }
        esp_err_t err = esp_stepper_set_max_speed(alias, value);
        if (err != ESP_OK) {
            AT(R "ERROR: speed failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "ACCEL")) {
        float value = 0.0f;
        if (!parse_float_value(tokens[2], &value)) {
            AT(R "ERROR: aceleracao invalida");
            return;
        }
        esp_err_t err = esp_stepper_set_acceleration(alias, value);
        if (err != ESP_OK) {
            AT(R "ERROR: accel failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "SPMM")) {
        float value = 0.0f;
        if (!parse_float_value(tokens[2], &value)) {
            AT(R "ERROR: steps_per_mm invalido");
            return;
        }
        esp_err_t err = esp_stepper_set_steps_per_mm(alias, value);
        if (err != ESP_OK) {
            AT(R "ERROR: spmm failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "MAX")) {
        int32_t pos = 0;
        if (!parse_i32(tokens[2], &pos)) {
            AT(R "ERROR: max invalido");
            return;
        }
        esp_err_t err = esp_stepper_set_max_position(alias, pos);
        if (err != ESP_OK) {
            AT(R "ERROR: max failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "MAXMM")) {
        stepper_motor_t *motor = find_motor(alias);
        float value = 0.0f;
        int32_t pos = 0;
        if (motor == NULL) {
            AT(R "ERROR: motor invalido");
            return;
        }
        if (!parse_float_value(tokens[2], &value) || !steps_from_mm(motor, value, &pos)) {
            AT(R "ERROR: max_mm invalido");
            return;
        }
        esp_err_t err = esp_stepper_set_max_position(alias, pos);
        if (err != ESP_OK) {
            AT(R "ERROR: maxmm failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "MOVE")) {
        int32_t delta = 0;
        if (!parse_i32(tokens[2], &delta)) {
            AT(R "ERROR: delta invalido");
            return;
        }
        esp_err_t err = esp_stepper_move(alias, delta);
        if (err != ESP_OK) {
            AT(R "ERROR: move failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "MOVETO")) {
        int32_t pos = 0;
        if (!parse_i32(tokens[2], &pos)) {
            AT(R "ERROR: pos invalido");
            return;
        }
        esp_err_t err = esp_stepper_move_to(alias, pos);
        if (err != ESP_OK) {
            AT(R "ERROR: moveto failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "MOVEMM")) {
        float value = 0.0f;
        if (!parse_float_value(tokens[2], &value)) {
            AT(R "ERROR: delta_mm invalido");
            return;
        }
        esp_err_t err = esp_stepper_move_mm(alias, value);
        if (err != ESP_OK) {
            AT(R "ERROR: movemm failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    if (ci_equals(action, "GOTO")) {
        float value = 0.0f;
        if (!parse_float_value(tokens[2], &value)) {
            AT(R "ERROR: pos_mm invalido");
            return;
        }
        esp_err_t err = esp_stepper_move_to_mm(alias, value);
        if (err != ESP_OK) {
            AT(R "ERROR: goto failed (%s)", esp_err_to_name(err));
            return;
        }
        AT(G "OK");
        return;
    }

    AT(R "ERROR: acao invalida");
}
