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
  
  //#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
/* #define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example */
// #define RISING_EDGE_POLLING_5_3
/* #define RISING_EDGE_INTERRUPT_5_4 */
#define TWO_AXIS_MACHINE
  
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


#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
/* #elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE))) */
  /* #error "Please select an option!" */
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

#ifdef TWO_AXIS_MACHINE
#define LIM_SWITCH_1_1 GPIO_PIN_8 // PA8
#define LIM_SWITCH_1_2 GPIO_PIN_0 // PA0
#define LIM_SWITCH_2_1 GPIO_PIN_5 // PA10
#define LIM_SWITCH_2_2 GPIO_PIN_1 // PA1
#define DEFAULT_SPEED  5
#endif

/**
  * @}
  */ /* End of ExampleTypes */
    
    /* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;
uint8_t lim_switch_1_1_state = RESET;
uint8_t lim_switch_1_2_state = RESET;
uint8_t lim_switch_2_1_state = RESET;
uint8_t lim_switch_2_2_state = RESET;
eL6470_DirId_t direction_1 = L6470_DIR_FWD_ID;
eL6470_DirId_t direction_2 = L6470_DIR_FWD_ID;

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
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // TODO could be falling
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

#ifdef TESTS
// lab 2 5.3 interrupt thing
void EXTI9_5_IRQHandler(void) {
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
  {
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
						// HAL_Delay(1);	// or i++ for finer control
		// int i = 0;
		 //for ( ; i < 10000; i++) {}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
		
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    // BSP_EmergencyStop();
  }
}
#elif defined(TWO_AXIS_MACHINE)
void lim_switch_1_1_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LIM_SWITCH_1_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void lim_switch_1_2_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LIM_SWITCH_1_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
void lim_switch_2_1_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LIM_SWITCH_2_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void lim_switch_2_2_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LIM_SWITCH_2_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // TODO
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/*void EXTI9_5_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(LIM_SWITCH_1_1) != RESET) {
      direction_1 = !L6470_DIR_FWD_ID;
      __HAL_GPIO_EXTI_CLEAR_IT(LIM_SWITCH_1_1);
  }
}

void EXTI0_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    BSP_L6470_BusySynchEventManager();
  }
}

void EXTI15_10_IRQHandler(void)
{
  direction_2 = !L6470_DIR_FWD_ID;
  
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    BSP_EmergencyStop();
  }//
}

void EXTI1_IRQHandler(void)
{   
  direction_2 = !L6470_DIR_REV_ID;
  
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		BSP_L6470_FlagEventManager();
  }
}*/

#endif

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
    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();

    /*
     * initialize limit switch pins with interrupt handlers
     */
    /*lim_switch_1_1_init();
    lim_switch_1_2_init();
    lim_switch_2_1_init();
    lim_switch_2_2_init();*/
    
    StepperMotorBoardHandle_t *StepperMotorBoardHandle;
    MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
    uint8_t board_id = 0;
    uint8_t device_id_1 = 0;
    uint8_t device_id_2 = 1;
    uint32_t speed_1, speed_2;

    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board_id);
    MotorParameterDataSingle = MotorParameterDataGlobal+(board_id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);

    speed_1 = speed_2 = 1000;
    
    // test program that moves motors back and forth, checking for limits when needed
    while (1) {
        // stop motor if needed
        // TODO confirm directions of motor relative to switches
        /*if ((lim_switch_1_1_state == GPIO_PIN_SET && direction_1 == L6470_DIR_FWD_ID)
            || (lim_switch_1_2_state == GPIO_PIN_SET && direction_1 == L6470_DIR_REV_ID)) {
            speed_1 = 0;
        } else {
            speed_1 = DEFAULT_SPEED;    // TODO confirm what a reasonable speed is
        }
        if ((lim_switch_2_1_state == GPIO_PIN_SET && direction_2 == L6470_DIR_FWD_ID)
            || (lim_switch_2_2_state == GPIO_PIN_SET && direction_2 == L6470_DIR_REV_ID)) {
            speed_2 = 0;
        } else {
            speed_2 = DEFAULT_SPEED;
        }*/

        // run motors
        StepperMotorBoardHandle->Command->Run(board_id, device_id_1, direction_1, speed_1);
        StepperMotorBoardHandle->Command->Run(board_id, device_id_2, direction_2, speed_2);
    }

#elif defined(TESTS)
    Tests();
#else
#error "define a macro"
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
