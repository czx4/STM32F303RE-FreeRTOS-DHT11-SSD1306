#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f3xx_hal.h"

void Error_Handler(void);

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB


#define DHT_PIN GPIO_PIN_0
#define DHT_PORT GPIOA 
#define DHT_EXTI EXTI0_IRQn
#define EXTI_HANDLER EXTI0_IRQHandler

#define TIM_HANDLE htim6
#define TIM_IRQ_HANDLER TIM6_DAC_IRQHandler

#define I2C_HANDLE hi2c1
#define I2C_EV_IRQ_HANDLER I2C1_EV_IRQHandler

#define DMA_TX_HANDLE hdma_i2c1_tx
#define DMA_IRQ DMA1_Channel6_IRQn
#define DMA_IRQ_HANDLER DMA1_Channel6_IRQHandler

#define OLED_WIDTH 128
#define oled_addr (0x3c << 1)


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
