/**
 * @file    common.h
 * @brief   Common types, macros, and definitions for EV ADAS System
 * @platform STM32F103C8T6 (Blue Pill)
 * @version  v1.0  April 2026
 */

#ifndef COMMON_H
#define COMMON_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* â”€â”€â”€ Utility Macros â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define CLAMP(x, lo, hi)   ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ARRAY_SIZE(a)       (sizeof(a) / sizeof((a)[0]))

/* â”€â”€â”€ Drive Modes â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define DRIVE_MODE_ECO      0
#define DRIVE_MODE_NORMAL   1
#define DRIVE_MODE_SPORT    2

/* â”€â”€â”€ Vehicle States â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef enum {
    STATE_PARKED  = 0,
    STATE_READY   = 1,
    STATE_DRIVING = 2,
    STATE_REGEN   = 3,
    STATE_FAULT   = 4,
} VehicleState_t;

/* â”€â”€â”€ Fault Flags (bit-field) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define FAULT_NONE   0x00
#define FAULT_OT     0x01   /* Motor over-temperature */
#define FAULT_SOC    0x02   /* Battery critically low  */
#define FAULT_COL    0x04   /* Collision critical       */
#define FAULT_SEN    0x08   /* Sensor timeout           */
#define FAULT_COM    0x10   /* UART comm timeout        */

/* â”€â”€â”€ External handle declarations (defined in main.c) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

/* â”€â”€â”€ GPIO pin aliases â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
/* HC-SR04 */
#define FRONT_TRIG_PIN   GPIO_PIN_0
#define FRONT_TRIG_PORT  GPIOB
#define FRONT_ECHO_PIN   GPIO_PIN_1
#define FRONT_ECHO_PORT  GPIOB

#define LEFT_TRIG_PIN    GPIO_PIN_2
#define LEFT_TRIG_PORT   GPIOB
#define LEFT_ECHO_PIN    GPIO_PIN_3
#define LEFT_ECHO_PORT   GPIOB

#define RIGHT_TRIG_PIN   GPIO_PIN_4
#define RIGHT_TRIG_PORT  GPIOB
#define RIGHT_ECHO_PIN   GPIO_PIN_5
#define RIGHT_ECHO_PORT  GPIOB

/* LEDs */
#define LED_COLLISION_PIN   GPIO_PIN_8
#define LED_COLLISION_PORT  GPIOB
#define LED_BLINDSPOT_L_PIN GPIO_PIN_9
#define LED_BLINDSPOT_PORT  GPIOB
#define LED_BLINDSPOT_R_PIN GPIO_PIN_10
#define LED_FAULT_PIN       GPIO_PIN_11
#define LED_FAULT_PORT      GPIOB

/* Motor FAULT contactor */
#define FAULT_PIN           GPIO_PIN_11
#define FAULT_GPIO_PORT     GPIOB

/* â”€â”€â”€ Buzzer tone types â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef enum {
    BUZZER_OFF    = 0,
    BUZZER_SINGLE = 1,   /* Advisory   */
    BUZZER_DOUBLE = 2,   /* Warning    */
    BUZZER_RAPID  = 3,   /* Critical   */
} BuzzerTone_t;

#endif /* COMMON_H */
