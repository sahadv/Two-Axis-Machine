#include <string.h>
#include <stdlib.h>

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
#define DEFAULT_SPEED_H  15000
#define DEFAULT_SPEED_V  20000

#define NUCLEO_USE_USART

#define ADC_H_PIN GPIO_PIN_0 // PC0
#define POT_H_ACTUAL_MIN 94
#define POT_H_ACTUAL_MAX 3366
#define MOTOR_H_MIN -40000
#define MOTOR_H_MAX 40000
#define MOTOR_H_DEADBAND 10000

/* #define USING_V_AXIS */
#ifdef USING_V_AXIS
#define ADC_V_PIN GPIO_PIN_0 // TODO change to something else if doing bonus
#define POT_V_ACTUAL_MIN 94
#define POT_V_ACTUAL_MAX 3366
#define MOTOR_V_MIN -40000
#define MOTOR_V_MAX 40000
#define MOTOR_V_DEADBAND 10000
#endif

__IO uint16_t uhADCxConvertedValue = 0;
eL6470_DirId_t direction_h = L6470_DIR_FWD_ID;
eL6470_DirId_t direction_v = L6470_DIR_FWD_ID;
int8_t motor_h_id = 0;
int8_t motor_v_id = 1;

int32_t speed_h = DEFAULT_SPEED_H;
int32_t speed_v = DEFAULT_SPEED_V;

ADC_HandleTypeDef adc1, adc2;

static void Error_Handler(void);
uint16_t Read_ADC(ADC_HandleTypeDef *hadc);

uint8_t buf[15];

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
    
    // ADC
    GPIO_InitStruct.Pin = ADC_H_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#ifdef USING_V_AXIS
    GPIO_InitStruct.Pin = ADC_V_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif
}

void EXTI9_5_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_L) != RESET) {
        BSP_L6470_HardStop(0, motor_h_id);
        speed_h = 0;
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_H_L);
    }
}

void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_H_R) != RESET) {
        BSP_L6470_HardStop(0, motor_h_id);
        speed_h = 0;
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
        speed_v = 0;
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
        speed_v = 0;
        __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_V_B);
    }

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        BSP_L6470_FlagEventManager();
    }
}

void ADC_Init(void) {
    ADC_ChannelConfTypeDef sConfig;

    __GPIOB_CLK_ENABLE();

    adc1.Instance = ADC1;
    adc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    adc1.Init.Resolution = ADC_RESOLUTION_12B;
    adc1.Init.ScanConvMode = ENABLE;
    adc1.Init.ContinuousConvMode = ENABLE;
    adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc1.Init.NbrOfConversion = 2;
    adc1.Init.DMAContinuousRequests = ENABLE;
    adc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&adc1);
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 1;
    HAL_ADC_ConfigChannel(&adc1, &sConfig);

#ifdef USING_V_AXIS
    adc2.Instance = ADC1;
    adc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    adc2.Init.Resolution = ADC_RESOLUTION_12B;
    adc2.Init.ScanConvMode = ENABLE;
    adc2.Init.ContinuousConvMode = ENABLE;
    adc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc2.Init.NbrOfConversion = 2;
    adc2.Init.DMAContinuousRequests = ENABLE;
    adc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&adc2);
    sConfig.Channel = ADC_CHANNEL_10;   // TODO if bonus then change channel to something else
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 1;
    HAL_ADC_ConfigChannel(&adc2, &sConfig);
#endif
}

int32_t map(int32_t val, int32_t lo, int32_t hi, int32_t new_lo, int32_t new_hi) {
    return (float) (val - lo) / (hi - lo) * (new_hi - new_lo) + new_lo;
}

int main(void)
{
    /* NUCLEO board initialization */
    /* Init for UART, ADC, GPIO and SPI */
    NUCLEO_Board_Init();

    /* X-NUCLEO-IHM02A1 initialization */
    BSP_Init();

    Fill_L6470_DaisyChainMnemonic();
    Motor_Param_Reg_Init();

    ADC_Init();

    lim_switch_init();

    // set speeds again after interrupts triggered
    speed_h = DEFAULT_SPEED_H;
    speed_v = DEFAULT_SPEED_V;
  
    while (1) {
        speed_h = map(Read_ADC(&adc1),
                      POT_H_ACTUAL_MIN,
                      POT_H_ACTUAL_MAX,
                      MOTOR_H_MIN,
                      MOTOR_H_MAX);
        if (abs(speed_h) < MOTOR_H_DEADBAND) {
            speed_h = 0;
        }
        direction_h = (speed_h < 0) ? L6470_DIR_REV_ID : L6470_DIR_FWD_ID;
        BSP_L6470_Run(0, motor_h_id, direction_h, abs(speed_h));

        /* memset(buf, 0, sizeof(buf)); */
        /* sprintf(buf, "motor val: %d\r\n", speed_h); */
        /* USART_Transmit(&huart2, buf); */

#ifdef USING_V_AXIS
        speed_v = map(Read_ADC(&adc2),
                      POT_V_ACTUAL_MIN,
                      POT_V_ACTUAL_MAX,
                      MOTOR_V_MIN,
                      MOTOR_V_MAX);
        if (abs(speed_v) < MOTOR_V_DEADBAND) {
            speed_v = 0;
        }
        direction_v = (speed_v < 0) ? L6470_DIR_REV_ID : L6470_DIR_FWD_ID;
        BSP_L6470_Run(0, motor_v_id, direction_v, abs(speed_v));
#endif
    }
}

static void Error_Handler(void) {
    /* Turn LED2 on */
    BSP_LED_On(LED2);
    while (1) { }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
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
uint16_t Read_ADC(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);

    return HAL_ADC_GetValue(hadc);
}

