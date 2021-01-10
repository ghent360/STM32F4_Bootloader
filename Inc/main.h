/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define ST_ENABLE_Pin LL_GPIO_PIN_0
#define ST_ENABLE_GPIO_Port GPIOC
#define SD_DETECT_Pin LL_GPIO_PIN_3
#define SD_DETECT_GPIO_Port GPIOD

#define X_MIN_Pin LL_GPIO_PIN_4
#define X_MIN_GPIO_Port GPIOC
#define Y_MIN_Pin LL_GPIO_PIN_5
#define Y_MIN_GPIO_Port GPIOC
#define Z_MIN_Pin LL_GPIO_PIN_0
#define Z_MIN_GPIO_Port GPIOB
#define X_MAX_Pin LL_GPIO_PIN_1
#define X_MAX_GPIO_Port GPIOB
#define Y_MAX_Pin LL_GPIO_PIN_14
#define Y_MAX_GPIO_Port GPIOC
#define Z_MAX_Pin LL_GPIO_PIN_7
#define Z_MAX_GPIO_Port GPIOE
#define U_MIN_Pin LL_GPIO_PIN_8
#define U_MIN_GPIO_Port GPIOE
#define V_MIN_Pin LL_GPIO_PIN_9
#define V_MIN_GPIO_Port GPIOE
#if 0
#define PWM1_Pin LL_GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin LL_GPIO_PIN_15
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin LL_GPIO_PIN_4
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin LL_GPIO_PIN_8
#define PWM4_GPIO_Port GPIOB
#define PWM5_Pin LL_GPIO_PIN_9
#define PWM5_GPIO_Port GPIOB
#define PWM6_Pin LL_GPIO_PIN_7
#define PWM6_GPIO_Port GPIOC
#define PWM7_Pin LL_GPIO_PIN_5
#define PWM7_GPIO_Port GPIOE
#define PWM8_Pin LL_GPIO_PIN_6
#define PWM8_GPIO_Port GPIOE
#define PWM9_Pin LL_GPIO_PIN_15
#define PWM9_GPIO_Port GPIOD
#endif

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
