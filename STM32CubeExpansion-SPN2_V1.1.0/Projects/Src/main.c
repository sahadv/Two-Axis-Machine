#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"

// these headers contain motor call def'ns
#include "params.h"
#include "xnucleoihm02a1.h"

#define LIM_SWITCH_H_L GPIO_PIN_8 // PA8
#define LIM_SWITCH_H_R GPIO_PIN_0 // PA0
#define LIM_SWITCH_V_T GPIO_PIN_5 // PA10
#define LIM_SWITCH_V_B GPIO_PIN_1 // PA1
#define DEFAULT_SPEED  5

__IO uint16_t uhADCxConvertedValue = 0;
uint8_t lim_switch_H_L_state = RESET;
uint8_t lim_switch_H_R_state = RESET;
uint8_t lim_switch_V_T_state = RESET;
uint8_t lim_switch_V_B_state = RESET;
eL6470_DirId_t direction_1 = L6470_DIR_FWD_ID;
eL6470_DirId_t direction_2 = L6470_DIR_FWD_ID;

static void Error_Handler(void);
uint16_t Read_ADC(void);

void lim_switch_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // H_L
    GPIO_InitStruct.Pin = LIM_SWITCH_H_L;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    // H_R
    GPIO_InitStruct.Pin = LIM_SWITCH_H_R;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/*
 *     // V_T
 *     GPIO_InitStruct.Pin = LIM_SWITCH_V_T;
 *     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 *     GPIO_InitStruct.Pull = GPIO_PULLUP; // TODO
 *     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
 *     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 *
 *     HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
 *     HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 *
 *     // V_B
 *     GPIO_InitStruct.Pin = LIM_SWITCH_V_B;
 *     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 *     GPIO_InitStruct.Pull = GPIO_PULLUP; // TODO
 *     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
 *     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 *
 *     HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
 *     HAL_NVIC_EnableIRQ(EXTI1_IRQn);
 */
}

void EXTI9_5_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_L) != RESET) {
      __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_H_L);
  }
}

void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_R) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_H_R);
    }

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    BSP_L6470_BusySynchEventManager();
  }
}

void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_V_T) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_V_T);
    }

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    BSP_EmergencyStop();
  }//
}

void EXTI1_IRQHandler(void)
{   
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_V_B) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_V_B);
    }

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		BSP_L6470_FlagEventManager();
  }
}

int main(void)
{
    /* NUCLEO board initialization */
    /* Init for UART, ADC, GPIO and SPI */
    NUCLEO_Board_Init();

    /* X-NUCLEO-IHM02A1 initialization */
    BSP_Init();

    // TODO include? leftovers from MICROSTEPPING_MOTOR_USART_EXAMPLE
    /* Fill the L6470_DaisyChainMnemonic structure */
    Fill_L6470_DaisyChainMnemonic();
    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();

    /*
     * initialize limit switch pins with interrupt handlers
     */
    lim_switch_init();
    
    uint8_t board_id = 0;
    uint8_t device_id_1 = 0;
    uint8_t device_id_2 = 1;
    uint32_t speed_1, speed_2;

    // test program that moves motors back and forth, checking for limits when needed
    while (1) {
        // USART_CheckAppCmd();  // read motor commands
        // run motors

        if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_L) == SET) {
          speed_1 = 0;
          USART_Transmit(&huart2, "HL SET\r\n");
        } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_L) == RESET) {
          speed_1 = 10000;
          USART_Transmit(&huart2, "HL RESET\r\n");
        }

       if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_R) == SET) {
          speed_1 = 0;
         USART_Transmit(&huart2, "HR SET\r\n");
       } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_R) == RESET) {
           speed_1 = 10000;
         USART_Transmit(&huart2, "HR RESET\r\n");
       }

       /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_T) == SET) { */
           /* speed_2 = 0; */
         /* USART_Transmit(&huart2, "VT SET\r\n"); */
       /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_T) == RESET) { */
           /* speed_2 = 10000; */
         /* USART_Transmit(&huart2, "VT RESET\r\n"); */
       /* } */
/*  */
       /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_B) == SET) { */
           /* speed_2 = 0; */
         /* USART_Transmit(&huart2, "VB SET\r\n"); */
       /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_B) == RESET) { */
           /* speed_2 = 10000; */
         /* USART_Transmit(&huart2, "VB RESET\r\n"); */
       /* } */
       
       BSP_L6470_Run(0,0,0,speed_1);
       BSP_L6470_Run(0,1,0,speed_2);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  This function return the ADC conversion result.
  * @retval The number into the range [0, 4095] as [0, 3.3]V.
  */
uint16_t Read_ADC(void)
{
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  
  return HAL_ADC_GetValue(&HADC);
}

