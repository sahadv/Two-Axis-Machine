/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"

// these headers contain motor call def'ns
#include "params.h"
#include "xnucleoihm02a1.h"

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
/* #define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example */
// #define RISING_EDGE_POLLING_5_3
/* #define RISING_EDGE_INTERRUPT_5_4 */
#define TWO_AXIS_MACHINE
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
/* #elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE))) */
  /* #error "Please select an option!" */
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */
    
    /* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;


/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler(void);
uint16_t Read_ADC(void);

void Test_Limit_Switch(void) {
    // TODO refactor
    // lim switch
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TODO input lag since print statements
    char buf[10];
    sprintf(buf, "%d\n\r", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
    HAL_UART_Transmit(&huart2, buf, 5, 10);
}

void Test_Led(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
}

void Rising_Edge_Polling(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    // exercise 5 polling/interrupt gpio
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // led
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

    GPIO_PinState cur = GPIO_PIN_RESET, prev = GPIO_PIN_RESET;

    while (1) {
        cur = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

        if (prev == GPIO_PIN_RESET && cur == GPIO_PIN_SET) { // rising edge
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
            // HAL_Delay(1);    // or i++ for finer control
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
        }

        prev = cur;
    }
}

void Rising_Edge_Interrupt(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    // exercise 5 polling/interrupt gpio
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // led
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Tests(void) {
    while (1) {
#ifdef TEST_SWITCH
        Test_Limit_Switch();
#endif

#ifdef TEST_LED
        Test_Led();
#endif

#ifdef RISING_EDGE_POLLING_5_3
        Rising_Edge_Polling();
#endif

#ifdef RISING_EDGE_INTERRUPT_5_4
        Rising_Edge_Interrupt();
#endif
    }
}

/**
  * @brief The FW main module
  */
int main(void)
{
    /* NUCLEO board initialization */
    /* Init for UART, ADC, GPIO and SPI */
    NUCLEO_Board_Init();

    /* X-NUCLEO-IHM02A1 initialization */
    BSP_Init();

#ifdef NUCLEO_USE_USART
    /* Transmit the initial message to the PC via UART */
    USART_TxWelcomeMessage();
#endif

/* #define MICROSTEPPING_MOTOR_USART_EXAMPLE */
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
    /* Perform a batch commands for X-NUCLEO-IHM02A1 */
    MicrosteppingMotor_Example_01();

    /* Infinite loop */
    while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
    /* Fill the L6470_DaisyChainMnemonic structure */
    Fill_L6470_DaisyChainMnemonic();

    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();

    /* Infinite loop */
    while (1)
    {
#ifdef TEST_MOTOR
        /* Check if any Application Command for L6470 has been entered by USART */
        USART_CheckAppCmd();
#else
        uint16_t myADCVal;
        myADCVal = Read_ADC();
        USART_Transmit(&huart2, " ADC Read: ");
        USART_Transmit(&huart2, num2hex(myADCVal, WORD_F));
        USART_Transmit(&huart2, " \n\r");
#endif      
    }
#elif defined(TWO_AXIS_MACHINE)
// TODO decide pins
#define LIM_SWITCH_1_1 GPIO_PIN_8
#define LIM_SWITCH_1_2 GPIO_PIN_9
#define LIM_SWITCH_2_1 GPIO_PIN_10
#define LIM_SWITCH_2_2 GPIO_PIN_11
#define DEFAULT_SPEED  5
    // 

    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();

    /*
     * initialize limit switch pins
     */

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = LIM_SWITCH_1_1;   // TODO get real pins for lim switches
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // TODO do we need to pull the GPIO pins?
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // TODO are switches on GPIOA?

    GPIO_InitStruct.Pin = LIM_SWITCH_1_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // TODO are switches on GPIOA?

    GPIO_InitStruct.Pin = LIM_SWITCH_2_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // TODO are switches on GPIOA?

    GPIO_InitStruct.Pin = LIM_SWITCH_2_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // TODO are switches on GPIOA?

    /* GPIO_PinState cur = GPIO_PIN_RESET, prev = GPIO_PIN_RESET; */
    // to check pin: cur = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_2_2);

    StepperMotorBoardHandle_t *StepperMotorBoardHandle;
    MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
    uint8_t board_id = EXPBRD_ID(0);    // TODO find actual id
    uint8_t device_id_1 = L6470_ID(0);  // TODO find actual id
    uint8_t device_id_2 = L6470_ID(1);  // TODO find actual id
    int direction_1, direction_2;
    uint32_t speed_1, speed_2;
    GPIO_PinState lim_switch_1_1_state[2];
    GPIO_PinState lim_switch_1_2_state[2];
    GPIO_PinState lim_switch_2_1_state[2];
    GPIO_PinState lim_switch_2_2_state[2];

    // TODO figure out what this block of code really does
    for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
    {
        StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
        MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
        StepperMotorBoardHandle->Config(MotorParameterDataSingle);
    }

    // test program that moves motors back and forth, checking for limits when needed
    while (1) {
        // update switch states
        lim_switch_1_1_state[1] = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_1_1);
        lim_switch_1_2_state[1] = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_1_2);
        lim_switch_2_1_state[1] = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_2_1);
        lim_switch_2_2_state[1] = HAL_GPIO_ReadPin(GPIOA, LIM_SWITCH_2_2);

        // TODO might need to use interrupts

        // stop motor if needed
        // TODO confirm directions of motor relative to switches
        if ((lim_switch_1_1_state[1] == GPIO_PIN_SET && direction_1 == L6470_DIR_FWD_ID)
            || (lim_switch_1_2_state[1] == GPIO_PIN_SET && direction_1 == L6470_DIR_REV_ID)) {
            speed_1 = 0;
        } else {
            speed_1 = DEFAULT_SPEED;    // TODO confirm what a reasonable speed is
        }
        if ((lim_switch_2_1_state[1] == GPIO_PIN_SET && direction_2 == L6470_DIR_FWD_ID)
            || (lim_switch_2_2_state[1] == GPIO_PIN_SET && direction_2 == L6470_DIR_REV_ID)) {
            speed_2 = 0;
        } else {
            speed_2 = DEFAULT_SPEED;
        }

        // on a rising edge, switch direction for each motor
        
        // switch direction for axis 1
        // TODO check if value is SET or RESET when the switch is pressed; i.e. are we checking for a rising edge or falling edge?
        if (lim_switch_1_1_state[0] == GPIO_PIN_RESET && lim_switch_1_1_state[1] == GPIO_PIN_SET) {
            direction_1 = !L6470_DIR_FWD_ID;
        }
        if (lim_switch_1_2_state[0] == GPIO_PIN_RESET && lim_switch_1_2_state[1] == GPIO_PIN_SET) {
            direction_1 = !L6470_DIR_REV_ID;
        }
        
        // switch direction for axis 2
        // TODO check if value is SET or RESET when the switch is pressed; i.e. are we checking for a rising edge or falling edge?
        if (lim_switch_2_1_state[0] == GPIO_PIN_RESET && lim_switch_2_1_state[1] == GPIO_PIN_SET) {
            direction_2 = !L6470_DIR_FWD_ID;
        }
        if (lim_switch_2_2_state[0] == GPIO_PIN_RESET && lim_switch_2_2_state[1] == GPIO_PIN_SET) {
            direction_2 = !L6470_DIR_REV_ID;
        }

        // run motors
        StepperMotorBoardHandle->Command->Run(board_id, device_id_1, direction_1, speed_1);
        StepperMotorBoardHandle->Command->Run(board_id, device_id_2, direction_2, speed_2);

        // update old switch states
        lim_switch_1_1_state[0] = lim_switch_1_1_state[1];
        lim_switch_1_2_state[0] = lim_switch_1_2_state[1];
        lim_switch_2_1_state[0] = lim_switch_2_1_state[1];
        lim_switch_2_2_state[0] = lim_switch_2_2_state[1];
    }

#else
    Tests();
#endif
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

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
