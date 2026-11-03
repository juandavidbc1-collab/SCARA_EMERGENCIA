/* Capa de hardware: PWM + dirección (TB6612), servo y finales de carrera. Sin encoders. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void    hal_init(void);                 /* lo PRIMERO que hace es poner STBY = 0 */
void    hal_stby(bool on);
bool    hal_stby_get(void);
void    hal_motor_drive(int j, int duty);  /* duty con signo en [-PWM_MAX, PWM_MAX]; u>0 ya incluye motor_sign */
void    hal_motor_free(int j);          /* IN1 = IN2 = 0: motor libre */
void    hal_motor_brake(int j);         /* IN1 = IN2 = 1: freno */
void    hal_all_free(void);
void    hal_tool_drive(int duty);       /* motor 3: duty con signo */
void    hal_tool_free(void);
void    hal_servo_set_deg(float deg);   /* servo de J13: 0 a SERVO_GRADOS_MAX */
bool    hal_fc_raw(int j, int side);    /* side 0 = MIN, 1 = MAX; true = presionado (sin antirrebote) */
