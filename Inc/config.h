#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f0xx_hal.h"

/* Buzzer configurations */
#define CONFIG_BUZZER_GPIO_PIN			GPIO_PIN_2
#define CONFIG_BUZZER_GPIO_PORT			GPIOA
#define CONFIG_BUZZER_GPIO_AF_TIMER		GPIO_AF0_TIM15
#define CONFIG_BUZZER_TIMER				TIM15
#define CONFIG_BUZZER_CHANNEL			TIM_CHANNEL_2
#define CONFIG_BUZZER_DUTY				50

/* Button engine configurations */

#define CONFIG_DELAY_POWER_ON_MS		1000
#define CONFIG_DELAY_RUN_QA_MS			1000

#define CONFIG_SEND_REPORT_TIMEOUT		1000

#define CONFIG_ADC_SAMPLE_TIME			ADC_SAMPLETIME_239CYCLES_5

#define CONFIG_CHECK_SHORT_ENABLE		0

/* Configurations for debug purposes */
#define CONFIG_CALIB_MODE					1
#define CONFIG_ALLOW_POWER_WITHOUT_TARGET	0

#endif
