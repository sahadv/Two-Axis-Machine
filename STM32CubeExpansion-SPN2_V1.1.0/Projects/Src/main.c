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
#define LIM_SWITCH_V_T GPIO_PIN_5 // PB5
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
#define ADC_V_PIN GPIO_PIN_1 // PC1
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

ADC_HandleTypeDef adc;

static void Error_Handler(void);
uint16_t Read_ADC(ADC_HandleTypeDef *hadc);

uint8_t buf[40];

void lim_switch_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    // H_L
    GPIO_InitStruct.Pin = LIM_SWITCH_H_L;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // H_R
    GPIO_InitStruct.Pin = LIM_SWITCH_H_R;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // V_T
    GPIO_InitStruct.Pin = LIM_SWITCH_V_T;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // V_B
    GPIO_InitStruct.Pin = LIM_SWITCH_V_B;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
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

void ADC_Init(void) {
    ADC_ChannelConfTypeDef sConfig;

    adc.Instance = ADC1;
    adc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.ScanConvMode = ENABLE;
    adc.Init.ContinuousConvMode = ENABLE;
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.NbrOfConversion = 2;
    adc.Init.DMAContinuousRequests = ENABLE;
    adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&adc);
  
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 1;
    HAL_ADC_ConfigChannel(&adc, &sConfig);

#ifdef USING_V_AXIS
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = 2;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 1;
    HAL_ADC_ConfigChannel(&adc, &sConfig);
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
        uint16_t adc1_val = Read_ADC(&adc);
        int switch_hr = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_R);
        int switch_hl = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_H_L);
        /* speed_h = map(Read_ADC(&adc), */
                      /* POT_H_ACTUAL_MIN, */
                      /* POT_H_ACTUAL_MAX, */
                      /* MOTOR_H_MIN, */
                      /* MOTOR_H_MAX); */
        speed_h = SpeedFilteringWithADCValue(adc1_val);
        direction_h = (speed_h < 0) ? L6470_DIR_REV_ID : L6470_DIR_FWD_ID;
        if (abs(speed_h) < MOTOR_H_DEADBAND
            || (direction_h == L6470_DIR_FWD_ID && switch_hr != RESET)
            || (direction_h == L6470_DIR_REV_ID && switch_hl != RESET)) {
            speed_h = 0;
        }
        BSP_L6470_Run(0, motor_h_id, direction_h, abs(speed_h));

#ifdef USING_V_AXIS
        uint16_t adc2_val = Read_ADC(&adc);
        int switch_vt = HAL_GPIO_ReadPin(GPIOB, LIM_SWITCH_V_T);
        int switch_vb = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_V_B);
        /* speed_v = map(Read_ADC(&adc2), */
                      /* POT_V_ACTUAL_MIN, */
                      /* POT_V_ACTUAL_MAX, */
                      /* MOTOR_V_MIN, */
                      /* MOTOR_V_MAX); */
        speed_v = SpeedFilteringWithADCValue(adc2_val);
        direction_v = (speed_v < 0) ? L6470_DIR_REV_ID : L6470_DIR_FWD_ID;
        if (abs(speed_v) < MOTOR_V_DEADBAND
            || (direction_v == L6470_DIR_FWD_ID && switch_vt != RESET)
            || (direction_v == L6470_DIR_REV_ID && switch_vb != RESET)) {
            speed_v = 0;
        }
       // BSP_L6470_Run(0, motor_v_id, direction_v, abs(speed_v));
#endif

        memset(buf, 0, sizeof(buf));
#ifdef USING_V_AXIS
        sprintf(buf, "adc1: %5d | motor h: %5d | left: %d | right: %d |---| "
                "adc2: %5d | motor v: %5d | top: %d | bot: %d\r\n",
                adc1_val, speed_h, switch_hl, switch_hr,
                adc2_val, speed_v, switch_vt, switch_vb);
#else
        sprintf(buf, "adc1: %5d | motor h: %5d | left: %d | right: %d\r\n",
                adc1_val, speed_h, switch_hl, switch_hr);
#endif
        USART_Transmit(&huart2, buf);
    }
}

int32_t SpeedFilteringWithADCValue(int32_t adc) {
  if (adc < 150) return -20000;
  else if (adc > 3000) return 20000;
  else return 0;
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

