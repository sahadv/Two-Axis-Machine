#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"

// these headers contain motor call def'ns
#include "params.h"
#include "xnucleoihm02a1.h"

#define LIM_SWITCH_H_L GPIO_PIN_8 // PA8
#define LIM_SWITCH_H_R GPIO_PIN_0 // PA0
#define LIM_SWITCH_V_T GPIO_PIN_10 // PA10
#define LIM_SWITCH_V_B GPIO_PIN_1 // PA1
#define DEFAULT_SPEED  10000

#define NUCLEO_USE_USART

__IO uint16_t uhADCxConvertedValue = 0;
eL6470_DirId_t direction_h = L6470_DIR_FWD_ID;
eL6470_DirId_t direction_v = L6470_DIR_FWD_ID;
int8_t motor_h_id = 0;
int8_t motor_v_id = 1;

static void Error_Handler(void);
uint16_t Read_ADC(void);

void lim_switch_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // H_L
    GPIO_InitStruct.Pin = LIM_SWITCH_H_L;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    // H_R
    GPIO_InitStruct.Pin = LIM_SWITCH_H_R;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    // V_T
    GPIO_InitStruct.Pin = LIM_SWITCH_V_T;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    // V_B
    GPIO_InitStruct.Pin = LIM_SWITCH_V_B;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI9_5_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_L) != RESET) {
        BSP_L6470_HardStop(0, motor_h_id);
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_H_L);
    }
}

void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_R) != RESET) {
        BSP_L6470_HardStop(0, motor_h_id);
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_H_R);
    }

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        BSP_L6470_BusySynchEventManager();
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_V_T) != RESET) {
        BSP_L6470_HardStop(0, motor_v_id);
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_V_T);
    }

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
        BSP_EmergencyStop();
    }
}

void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_V_B) != RESET) {
        BSP_L6470_HardStop(0, motor_v_id);
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_V_B);
    }

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        BSP_L6470_FlagEventManager();
    }
}


/*
 * static void ADCx_Init(void)
 * {
 *   if(HAL_ADC_GetState(&hnucleo_Adc) == HAL_ADC_STATE_RESET)
 *   {
 *     [> ADC Config <]
 *     hnucleo_Adc.Instance                   = NUCLEO_ADCx;
 *     hnucleo_Adc.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4; [> (must not exceed 36MHz) <]
 *     hnucleo_Adc.Init.Resolution            = ADC_RESOLUTION12b;
 *     hnucleo_Adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
 *     hnucleo_Adc.Init.ContinuousConvMode    = DISABLE;
 *     hnucleo_Adc.Init.DiscontinuousConvMode = DISABLE;
 *     hnucleo_Adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
 *     hnucleo_Adc.Init.EOCSelection          = EOC_SINGLE_CONV;
 *     hnucleo_Adc.Init.NbrOfConversion       = 1;
 *     hnucleo_Adc.Init.DMAContinuousRequests = DISABLE;    **************************************************
 *
 *     ADCx_MspInit(&hnucleo_Adc);
 *     HAL_ADC_Init(&hnucleo_Adc);
 *   }
 * }
 */
void ADC_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /* GPIO Ports Clock Enable */
  __GPIOB_CLK_ENABLE();

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  // hadc1.Init.ClockPrescaler = ;
  // hadc1.Init.Resolution = ;
  // hadc1.Init.ScanConvMode = DISABLE;    **************************************************
  // hadc1.Init.ContinuousConvMode = ;
  // hadc1.Init.DiscontinuousConvMode = ;
  // hadc1.Init.ExternalTrigConvEdge = ;
  // hadc1.Init.DataAlign = ;
  // hadc1.Init.NbrOfConversion = ;
  // hadc1.Init.DMAContinuousRequests = ;
  // hadc1.Init.EOCSelection = ;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;		/* Currently set to input pin PB0, adjust as needed */
  sConfig.Rank = 1;
  // sConfig.SamplingTime = ;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

int main(void)
{
    /* NUCLEO board initialization */
    /* Init for UART, ADC, GPIO and SPI */
    NUCLEO_Board_Init();

    /* X-NUCLEO-IHM02A1 initialization */
    BSP_Init();

    /* Fill the L6470_DaisyChainMnemonic structure */
    Fill_L6470_DaisyChainMnemonic();
    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();

    /* ADC_Init(); */

    /*
     * initialize limit switch pins with interrupt handlers
     */
    lim_switch_init();

    uint32_t speed_h, speed_v;

    // test program that moves motors back and forth, checking for limits when needed
    while (1) {
        USART_CheckAppCmd();  // read motor commands

        /*
         * run motors
         */

        /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_L) == SET) { */
            /* speed_h = 0; */
            /* [> USART_Transmit(&huart2, "HL SET\r\n"); <] */
        /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_L) == RESET) { */
            /* speed_h = DEFAULT_SPEED; */
            /* [> USART_Transmit(&huart2, "HL RESET\r\n"); <] */
        /* } */
/*  */
        /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_R) == SET) { */
            /* speed_h = 0; */
            /* [> USART_Transmit(&huart2, "HR SET\r\n"); <] */
        /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_R) == RESET) { */
            /* speed_h = DEFAULT_SPEED; */
            /* [> USART_Transmit(&huart2, "HR RESET\r\n"); <] */
        /* } */
/*  */
        /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_T) == SET) { */
            /* speed_v = 0; */
            /* [> USART_Transmit(&huart2, "VT SET\r\n"); <] */
        /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_T) == RESET) { */
            /* speed_v = DEFAULT_SPEED; */
            /* [> USART_Transmit(&huart2, "VT RESET\r\n"); <] */
        /* } */
/*  */
        /* if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_B) == SET) { */
            /* speed_v = 0; */
            /* [> USART_Transmit(&huart2, "VB SET\r\n"); <] */
        /* } else if (HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_B) == RESET) { */
            /* speed_v = DEFAULT_SPEED; */
            /* [> USART_Transmit(&huart2, "VB RESET\r\n"); <] */
        /* } */
/*  */
        /* BSP_L6470_Run(0, motor_h_id, direction_h, speed_h); */
        /* BSP_L6470_Run(0, motor_v_id, direction_v, speed_v); */
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

