#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f0xx_hal.h"
#include "main.h"

#define INVALID_IRQ		(-99)

#define ADC_PIN_NUM		7

#define ADC_INPUT_3		{ .port = GPIOA, .pin = GPIO_PIN_3 }
#define ADC_INPUT_4		{ .port = GPIOA, .pin = GPIO_PIN_4 }
#define ADC_INPUT_5		{ .port = GPIOA, .pin = GPIO_PIN_5 }
#define ADC_INPUT_6		{ .port = GPIOA, .pin = GPIO_PIN_6 }
#define ADC_INPUT_7		{ .port = GPIOA, .pin = GPIO_PIN_7 }
#define ADC_INPUT_8		{ .port = GPIOB, .pin = GPIO_PIN_0 }
#define ADC_INPUT_9		{ .port = GPIOB, .pin = GPIO_PIN_1 }

#define TICK_TO_MS(t) 	((t) * (uint32_t)HAL_GetTickFreq())
#define MS_TO_TICK(ms) 	((ms) / (uint32_t)HAL_GetTickFreq())

#define BOOT_SERIAL_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define BOOT_SERIAL_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define BOOT_DEV_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define BOOT_DEV_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define BOOT_FUSE_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define BOOT_FUSE_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define MBZ_PWR_OK_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define MBZ_PWR_OK_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

#define BOARD_G_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define BOARD_G_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

#define BOARD_R_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define BOARD_R_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

#define LED_STT_G_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define LED_STT_G_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define LED_STT_B_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define LED_STT_B_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED_STT_R_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define LED_STT_R_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

#define MBZ_STT_B_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define MBZ_STT_B_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define MBZ_STT_R_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define MBZ_STT_R_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define MBZ_STT_G_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define MBZ_STT_G_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc;

typedef struct {
	uint16_t pin;
	GPIO_TypeDef *port;
} PinInfo;

typedef enum {
	OPMODE_INVALID = 0,
	OPMODE_AUTO,
	OPMODE_CONTROL,
	OPMODE_NUM
} OperationMode;

typedef enum {
	QA_PASSED = 0,
	QA_VOLTAGES_FAILED_BUT_ACCEPTED,
	QA_DEVICE_NOT_FOUND,
	QA_DEVICE_NOT_POWERED,
	QA_VOLTAGES_FAILED,
	QA_POWER_FAILED,
} QaStatus;

#endif
