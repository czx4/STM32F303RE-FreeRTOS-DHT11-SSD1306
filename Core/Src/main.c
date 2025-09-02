#include "main.h"
#include "ssd1306_font.h"
#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include <string.h>
#include <stdio.h>

#define OLED_WIDTH 128
#define oled_addr (0x3c << 1)

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
TIM_HandleTypeDef htim6;
TaskHandle_t DHTHandle;

volatile SemaphoreHandle_t i2c_sem;
volatile SemaphoreHandle_t dht_sem;
SemaphoreHandle_t read_ready;

uint8_t temp,humi,new_temp,new_humi;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void EXTI0_IRQHandler(void);

void i2c_print(void * arg);
void dht11_read(void * arg);
void draw_letter(uint8_t *i2c_buf,char c);
void draw_string(uint8_t *i2c_buf,char *str_buf);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();

  i2c_sem=xSemaphoreCreateBinary();
  xSemaphoreGive(i2c_sem);

  dht_sem=xSemaphoreCreateBinary();
  xSemaphoreGive(dht_sem);

  read_ready=xSemaphoreCreateBinary();
  xSemaphoreGive(read_ready);
  xSemaphoreTake(read_ready,0);

  xTaskCreate(dht11_read,"readdht11",256,NULL,0,&DHTHandle);
  xTaskCreate(i2c_print,"printtoscreen",256,NULL,1,NULL);
  
  vTaskStartScheduler();
  while (1);
}

void i2c_print(void * argument)
{
  uint8_t init_oled[]={
	  0x00,      // control byte = command
	  0xAE,      // Display OFF
	  0x20, 0x00,// Set Memory Addressing Mode to Horizontal
	  0xB0,      // Set Page Start Address for Page Addressing Mode
	  0xC0,      // COM Output Scan Direction remapped
	  0x00,      // low column address
	  0x10,      // high column address
	  0x40,      // start line address
	  0x81, 0x7F,// contrast control
	  0xA0,      // segment remap
	  0xA6,      // normal display
	  0xA8, 0x3F,// multiplex ratio(1 to 64)
	  0xA4,      // output RAM to display
	  0xD3, 0x00,// display offset
	  0xD5, 0x80,// display clock divide ratio/oscillator frequency
	  0xD9, 0xF1,// pre-charge period
	  0xDA, 0x12,// COM pins hardware configuration
	  0xDB, 0x40,// vcomh deselect level
	  0x8D, 0x14,// charge pump
	  0xAF       // Display ON
	};
	char buffer[100]={0};
	uint8_t i2c_buf[40]={0};
  uint8_t set_cursor[] = {
    0x00, // command
    0xB0, // Page 0
    0x00, // Lower column start address = 0
    0x10  // Higher column start address = 0
  };

	if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
		HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr, init_oled,sizeof(init_oled));
	}
  if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
		HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr,set_cursor, sizeof(set_cursor));
	}
	if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
		HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr,icons, sizeof(icons));
	}
  for(;;)
  {
    if(xSemaphoreTake(read_ready,portMAX_DELAY)==pdTRUE){
      if(new_humi!=humi){
      humi=new_humi;
			memset(i2c_buf,0,sizeof(i2c_buf));
			i2c_buf[0]=0x40;
			snprintf(buffer,sizeof(buffer), "%u %%",humi);
			set_cursor[1]=0xB6;
      set_cursor[2] = 0x04;
      set_cursor[3] = 0x12;
			draw_string(&i2c_buf[1], buffer);
			if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
				HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr, set_cursor, sizeof(set_cursor));
			}
			if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
				HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr, i2c_buf, sizeof(i2c_buf));
			}
    }
    if(new_temp!=temp){
      temp=new_temp;
			if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
				memset(i2c_buf, 0, sizeof(i2c_buf));
				i2c_buf[0] = 0x40;
	  		snprintf(buffer, sizeof(buffer), "%u%cC", temp,127);
				draw_string(&i2c_buf[1], buffer);
				set_cursor[1] = 0xB2;
        set_cursor[2] = 0x04;
        set_cursor[3] = 0x12;
				HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr, set_cursor, sizeof(set_cursor));
			}
			if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
				HAL_I2C_Master_Transmit_DMA(&hi2c1, oled_addr, i2c_buf, sizeof(i2c_buf));
			}
    }
			if(xSemaphoreTake(i2c_sem,portMAX_DELAY)==pdTRUE){
				xSemaphoreGive(i2c_sem);
			}
		}
			xSemaphoreGive(dht_sem);
  }

}

void dht11_read(void * argument)
{
	uint8_t data[5]={0};
	uint8_t time;

	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_PIN_0;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  for(;;)
  {
	if(xSemaphoreTake(dht_sem,portMAX_DELAY)==pdTRUE){
	memset(data,0,5*sizeof(data[0]));
	__HAL_TIM_SET_COUNTER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6,20000);
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim6);
	}
	if(xSemaphoreTake(dht_sem,portMAX_DELAY)==pdTRUE){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, GPIO_PIN_SET);
		__HAL_TIM_SET_AUTORELOAD(&htim6,40);
	}
	if(xSemaphoreTake(dht_sem,portMAX_DELAY)==pdTRUE){
		__HAL_TIM_SET_AUTORELOAD(&htim6,65535);
		GPIO_InitTypeDef GPIO_InitStruct={0};
		GPIO_InitStruct.Pin=GPIO_PIN_0;
		GPIO_InitStruct.Mode=GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull=GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	for(uint8_t data_index=0;data_index<40;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){//rising
			__HAL_TIM_SET_COUNTER(&htim6,0);
		}
		else{//falling
			time=__HAL_TIM_GET_COUNTER(&htim6);
			data[data_index/8]<<=1;
			if(time>50){
				data[data_index/8]|=1;
			}
			data_index++;
		}
	}

	__HAL_TIM_SET_COUNTER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6,50);
	if(xSemaphoreTake(dht_sem,portMAX_DELAY)==pdTRUE){
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6,0);
    __HAL_TIM_SET_AUTORELOAD(&htim6,65535);
    GPIO_InitTypeDef GPIO_InitStruct={0};
    GPIO_InitStruct.Pin=GPIO_PIN_0;
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, GPIO_PIN_SET);
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]){
      new_humi=data[0];
      new_temp=data[2];
    }
  }
	xSemaphoreGive(read_ready);
  vTaskDelay(50/portTICK_PERIOD_MS); //added for stability
  }
}

void draw_letter(uint8_t *i2c_buf,char c){
	for(int i=0;i<5;i++){
		i2c_buf[i]=font5x8[c-32][i];

	}
}
void draw_string(uint8_t *i2c_buf,char *str_buf){
	uint32_t num=0;
	while(num!=strlen(str_buf)){
		draw_letter(&i2c_buf[num*5],str_buf[num]);
		num++;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(hi2c->Instance==I2C1){
	  BaseType_t higher=pdFALSE;
	  xSemaphoreGiveFromISR(i2c_sem,&higher);
	  portYIELD_FROM_ISR(higher);
  }
}

void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void HAL_GPIO_EXTI_Callback(uint16_t pin){
	if(pin==GPIO_PIN_0){
		BaseType_t higher=pdFALSE;
    vTaskNotifyGiveFromISR(DHTHandle,&higher);
		portYIELD_FROM_ISR(higher);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6){
		BaseType_t higher=pdFALSE;
		xSemaphoreGiveFromISR(dht_sem,&higher);
		portYIELD_FROM_ISR(higher);
	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{


  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}


static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
