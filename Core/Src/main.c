/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PASS_LENGTH 8
#define MAX_WRONG_INPUTS 3
#define GREEN_LED_ID 13
#define YELLOW_LED_ID 14
#define RED_LED_ID 15
#define BUFFER_SIZE 64
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 char pass[PASS_LENGTH] = "asdfghjk";

 uint32_t currentInput = 0;
 uint32_t wrongInputs = 0;

 _Bool interruptMode = 0;
 _Bool txBusy = 0;

 typedef struct {
     uint8_t data[BUFFER_SIZE];
     uint16_t head;
     uint16_t tail;
     uint16_t count;
 } FIFO_Buffer;

 FIFO_Buffer rxBuffer;
 FIFO_Buffer txBuffer;
 uint8_t rxByte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// static void MX_GPIO_Init(void);
// static void MX_USART6_UART_Init(void);
void txBufferStart(FIFO_Buffer *buf);
void processInputChar(char c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void resetInput(uint32_t* currentInput, uint32_t* wrongInputs) {
	*currentInput = 0;
	*wrongInputs = 0;
}

void fifo_init(FIFO_Buffer *buf) {
  buf->head = 0;
  buf->tail = 0;
  buf->count = 0;
}

_Bool fifo_put(FIFO_Buffer *buf, uint8_t val) {
    if(buf->count >= BUFFER_SIZE) return 0;
    buf->data[buf->head++] = val;
    if(buf->head >= BUFFER_SIZE) buf->head = 0;
    buf->count++;
    return 1;
}

_Bool fifo_get(FIFO_Buffer *buf, uint8_t *val) {
    if(buf->count == 0) return 0;
    *val = buf->data[buf->tail++];
    if(buf->tail >= BUFFER_SIZE) buf->tail = 0;
    buf->count--;
    return 1;
}
void fifo_clear(FIFO_Buffer *buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->count = 0;
}

void startUartIT(void) {
    fifo_init(&rxBuffer);
    HAL_UART_Receive_IT(&huart6, &rxByte, 1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    fifo_put(&rxBuffer, rxByte);
    HAL_UART_Receive_IT(&huart6, &rxByte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    uint8_t c;
    if (huart->Instance == USART6) {
        if (fifo_get(&txBuffer, &c)) {
            HAL_UART_Transmit_IT(&huart6, &c, 1);
        } else {
            txBusy = 0;
        }
    }
}

void stopUartIT(void) {
    HAL_UART_Abort(&huart6);
    fifo_clear(&rxBuffer);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char in;
  uint32_t timeoutLimit = 2000;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  if(interruptMode) {
      char msg[] = "Interrupt mode ON\r\n";
      for (size_t i = 0; i < strlen(msg); i++) {
    	  fifo_put(&txBuffer, msg[i]);
      }
      txBufferStart(&txBuffer);
  } else {
      char msg[] = "Polling mode ON\r\n";
      HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  static _Bool lastButtonState = 0;
	  _Bool currentButtonState = 1;

	   if (currentButtonState && !lastButtonState) {
	      interruptMode = !interruptMode;

		  if (interruptMode) {
			  startUartIT();
			  char msg[] = "\r\nSwitched to Interrupt mode\r\n";
			  for (size_t i = 0; i < strlen(msg); i++) {
			      fifo_put(&txBuffer, msg[i]);
			  }
			  txBufferStart(&txBuffer);
		  } else {
			  stopUartIT();
			  char msg[] = "\r\nSwitched to Polling mode\r\n";
			  HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
		  }
	  }
	  lastButtonState = currentButtonState;

	  if(interruptMode) {
		  uint8_t c;
		  if (fifo_get(&rxBuffer, &c)) {
			  fifo_put(&txBuffer, c);
			  txBufferStart(&txBuffer);
	          processInputChar(c);
		  }
	  } else {
		  HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, (uint8_t*)&in, 1, timeoutLimit);
		  if (status == HAL_OK) {
			  HAL_UART_Transmit(&huart6, (uint8_t*)&in, 1, 100);
			  processInputChar(in);
		  } else if (status == HAL_TIMEOUT && (currentInput || wrongInputs)) {
			  resetInput(&currentInput, &wrongInputs);
		  }
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void txBufferStart(FIFO_Buffer *buf) {
    uint8_t c;
    if(!txBusy && fifo_get(buf, &c)) {
        txBusy = 1;
        HAL_UART_Transmit_IT(&huart6, &c, 1);
    }
}

void processInputChar(char c) {
    if (c >= 'A' && c <= 'Z') {
        c += 'a' - 'A';
    }
    if (c < 'a' || c > 'z') return;

    if (c != pass[currentInput]) {
        wrongInputs++;
        if (wrongInputs == MAX_WRONG_INPUTS) {
            resetInput(&currentInput, &wrongInputs);
        } else {
        }
    } else {
        if (currentInput == PASS_LENGTH - 1) {
            resetInput(&currentInput, &wrongInputs);
        } else {
            currentInput++;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
