#include "hal.h"
#include <stdlib.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define PWM_BITS LEDC_TIMER_10_BIT
#define SERVO_BITS LEDC_TIMER_14_BIT
#define SERVO_CH 3

static int dir_actual[NJ + 1];          /* -1, 0, +1 por motor (el índice NJ es el motor 3) */
static bool stby_estado = false;

void hal_stby(bool on)
{
    gpio_set_level(PIN_STBY, on ? 1 : 0);
    stby_estado = on;
}

bool hal_stby_get(void) { return stby_estado; }

static void ledc_duty(int ch, int d)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

void hal_motor_free(int j)
{
    ledc_duty(j, 0);
    gpio_set_level(JC[j].in1, 0);
    gpio_set_level(JC[j].in2, 0);
    dir_actual[j] = 0;
}

void hal_motor_brake(int j)
{
    gpio_set_level(JC[j].in1, 1);
    gpio_set_level(JC[j].in2, 1);
    ledc_duty(j, PWM_MAX);
    dir_actual[j] = 0;
}

void hal_all_free(void)
{
    for (int j = 0; j < NJ; j++) hal_motor_free(j);
}

/* Al invertir el sentido: primero PWM = 0, luego IN1/IN2, luego el PWM nuevo. */
void hal_motor_drive(int j, int duty)
{
    duty *= JC[j].motor_sign;
    int d = abs(duty);
    if (d > PWM_MAX) d = PWM_MAX;
    if (d == 0) { hal_motor_free(j); return; }
    int dir = duty > 0 ? 1 : -1;
    if (dir != dir_actual[j]) {
        ledc_duty(j, 0);
        gpio_set_level(JC[j].in1, dir > 0 ? 1 : 0);
        gpio_set_level(JC[j].in2, dir > 0 ? 0 : 1);
        dir_actual[j] = dir;
    }
    ledc_duty(j, d);
}

bool hal_fc_raw(int j, int side)
{
    return gpio_get_level(side ? JC[j].fc_max : JC[j].fc_min) == FC_PRESIONADO_NIVEL;
}

void hal_tool_free(void)
{
    ledc_duty(NJ, 0);
    gpio_set_level(PIN_M3_IN1, 0);
    gpio_set_level(PIN_M3_IN2, 0);
    dir_actual[NJ] = 0;
}

void hal_tool_drive(int duty)
{
    duty *= TOOL_MOTOR_SIGN;
    int d = abs(duty);
    if (d > PWM_MAX) d = PWM_MAX;
    if (d == 0) { hal_tool_free(); return; }
    int dir = duty > 0 ? 1 : -1;
    if (dir != dir_actual[NJ]) {
        ledc_duty(NJ, 0);
        gpio_set_level(PIN_M3_IN1, dir > 0 ? 1 : 0);
        gpio_set_level(PIN_M3_IN2, dir > 0 ? 0 : 1);
        dir_actual[NJ] = dir;
    }
    ledc_duty(NJ, d);
}

void hal_servo_set_deg(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > SERVO_GRADOS_MAX) deg = SERVO_GRADOS_MAX;
    const float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / SERVO_GRADOS_MAX;
    const float periodo_us = 1e6f / SERVO_HZ;
    ledc_duty(SERVO_CH, (int)(us / periodo_us * ((1 << SERVO_BITS) - 1)));
}

void hal_init(void)
{
    /* 1) STBY primero: drivers apagados antes de tocar cualquier otra cosa */
    gpio_config_t st = { .pin_bit_mask = 1ULL << PIN_STBY, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&st);
    hal_stby(false);

    /* 2) pines IN1/IN2 de los 3 canales (el motor 3 queda en 0) */
    uint64_t mask = (1ULL << PIN_M3_IN1) | (1ULL << PIN_M3_IN2);
    for (int j = 0; j < NJ; j++) mask |= (1ULL << JC[j].in1) | (1ULL << JC[j].in2);
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(PIN_M3_IN1, 0);
    gpio_set_level(PIN_M3_IN2, 0);
    for (int j = 0; j < NJ; j++) { gpio_set_level(JC[j].in1, 0); gpio_set_level(JC[j].in2, 0); }

    /* 3) PWM 20 kHz, 10 bits. Canales 0 y 1 = motores 1 y 2; canal 2 = motor 3 */
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_BITS, .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    gpio_num_t pwm_pins[3] = { JC[0].pwm, JC[1].pwm, PIN_M3_PWM };
    for (int c = 0; c < 3; c++) {
        ledc_channel_config_t cc = {
            .gpio_num = pwm_pins[c], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)c,
            .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&cc));
    }

    /* 4) finales de carrera: GPIO 34-39 no tienen pull interno; la placa trae R30-R33 */
    uint64_t fm = 0;
    for (int j = 0; j < NJ; j++) fm |= (1ULL << JC[j].fc_min) | (1ULL << JC[j].fc_max);
    gpio_config_t fc = { .pin_bit_mask = fm, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE };
    gpio_config(&fc);

    /* 5) servo en J13: temporizador propio de 50 Hz. Arranca con el ángulo cerrado para no soltar un tornillo. */
    ledc_timer_config_t ts = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = SERVO_BITS, .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ts));
    ledc_channel_config_t cs = {
        .gpio_num = PIN_SERVO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)SERVO_CH,
        .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cs));
    hal_servo_set_deg(SERVO_CERRADO_DEG);
}
