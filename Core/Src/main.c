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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 UART_HandleTypeDef huart6;
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


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  uint32_t idleStartTime;
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
	  _Bool currentButtonState = isPressed();

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
			  toggleLed(YELLOW_LED_ID, 1000);
			  resetInput(&currentInput, &wrongInputs);
		  }
	  }
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
    if (c == '+') {
    	if (interruptMode) {
            newStartPasswordChange();
            return;
    	}
        startPasswordChange();
        return;
    }
    if (c < 'a' || c > 'z') return;

    if (c != pass[currentInput]) {
        wrongInputs++;
        if (wrongInputs == MAX_WRONG_INPUTS) {
            toggleLed(RED_LED_ID, 2500);
            resetInput(&currentInput, &wrongInputs);
        } else {
            toggleLed(RED_LED_ID, 250);
        }
    } else {
        if (currentInput == PASS_LENGTH - 1) {
            toggleLed(GREEN_LED_ID, 2500);
            resetInput(&currentInput, &wrongInputs);
        } else {
            currentInput++;
            toggleLed(YELLOW_LED_ID, 250);
        }
    }
}

void startPasswordChange() {
    char newPass[PASS_LENGTH] = {0};
    char c;
    uint32_t index = 0;

    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\nEnter new password:", 22, 100);

    while (1) {
    	if (interruptMode) {
    		fifo_get(&rxBuffer, &c);
    		c = tolower((unsigned char)c);
			if ((c >= 'a' && c <= 'z') && index < PASS_LENGTH) {
				newPass[index++] = c;
				HAL_UART_Transmit(&huart6, (uint8_t*)&c, 1, 100);
			} else if (c == '\r' || c == '\n' || index == PASS_LENGTH) {
				break;
			}
    	} else {
			if (HAL_UART_Receive(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
				c = tolower((unsigned char)c);
				if ((c >= 'a' && c <= 'z') && index < PASS_LENGTH) {
					newPass[index++] = c;
					HAL_UART_Transmit(&huart6, (uint8_t*)&c, 1, 100);
				} else if (c == '\r' || c == '\n' || index == PASS_LENGTH) {
					break;
				}
			}
    	}
    }

    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\nActivate new password? (y/n)\r\n", 32, 100);

    while (1) {
    	if (interruptMode) {
    		fifo_get(&rxBuffer, &c);
    		c = tolower((unsigned char)c);
			if (c == 'y') {
				memcpy(pass, newPass, PASS_LENGTH);
				HAL_UART_Transmit(&huart6, (uint8_t*)"Password changed.\r\n", 19, 100);
				break;
			} else if (c == 'n') {
				HAL_UART_Transmit(&huart6, (uint8_t*)"Password not changed.\r\n", 25, 100);
				break;
			}
    	} else {
            if (HAL_UART_Receive(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
                c = tolower((unsigned char)c);
                if (c == 'y') {
                    memcpy(pass, newPass, PASS_LENGTH);
                    HAL_UART_Transmit(&huart6, (uint8_t*)"Password changed.\r\n", 19, 100);
                    break;
                } else if (c == 'n') {
                    HAL_UART_Transmit(&huart6, (uint8_t*)"Password not changed.\r\n", 25, 100);
                    break;
                }
            }
    	}
    }
}
void newStartPasswordChange() {
    char newPass[PASS_LENGTH] = {0};
    char c;
    uint32_t index = 0;

    if (interruptMode) {
    	char msg[] = "\r\nEnter new password:";
    	for (size_t i = 0; i < strlen(msg); i++) {
    		fifo_put(&txBuffer, (uint8_t)msg[i]); // добавляем каждый символ в буфер
    	}
    	txBufferStart(&txBuffer);
//        HAL_UART_Transmit_IT(&huart6, (uint8_t*)"\r\nEnter new password:", 22);
    } else {
        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\nEnter new password: ", 22, 100);
    }

    // Ввод нового пароля
    while (index < PASS_LENGTH) {
        _Bool gotChar = 0;

        if (interruptMode) {
            gotChar = fifo_get(&rxBuffer, (uint8_t*)&c);
        } else {
            if (HAL_UART_Receive(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
                gotChar = 1;
            }
        }

        if (!gotChar) continue; // символа ещё нет

        c = tolower((unsigned char)c);

        if (c >= 'a' && c <= 'z') {
            newPass[index++] = c;
            if (interruptMode) {
            	fifo_put(&txBuffer, (uint8_t)c);
            	txBufferStart(&txBuffer);
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)&c, 1);
            } else {
                HAL_UART_Transmit(&huart6, (uint8_t*)&c, 1, 100);
            }
        } else if (c == '\r' || c == '\n') {
            break;
        }
    }

    // Подтверждение пароля
    if (interruptMode) {
    	char msg[] = "\r\nActivate new password? (y/n)\r\n";
		for (size_t i = 0; i < strlen(msg); i++) {
			fifo_put(&txBuffer, (uint8_t)msg[i]);
		}
		txBufferStart(&txBuffer);
//        HAL_UART_Transmit_IT(&huart6, (uint8_t*)"\r\nActivate new password? (y/n)\r\n", 32);
    } else {
        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\nActivate new password? (y/n)\r\n", 32, 100);
    }

    while (1) {
        _Bool gotChar = 0;

        if (interruptMode) {
            gotChar = fifo_get(&rxBuffer, (uint8_t*)&c);
        } else {
            if (HAL_UART_Receive(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
                gotChar = 1;
            }
        }

        if (!gotChar) continue;

        c = tolower((unsigned char)c);
        if (c == 'y') {
            memcpy(pass, newPass, PASS_LENGTH);
            if (interruptMode) {
            	char msg[] = "Password changed.\r\n";
            	for (size_t i = 0; i < strlen(msg); i++) {
            		fifo_put(&txBuffer, (uint8_t)msg[i]); // добавляем каждый символ в буфер
            	}
            	txBufferStart(&txBuffer);
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)"Password changed.\r\n", 19);

            } else {
                HAL_UART_Transmit(&huart6, (uint8_t*)"Password changed.\r\n", 19, 100);
            }
            break;
        } else if (c == 'n') {
            if (interruptMode) {
            	char msg[] = "Password not changed.\r\n";
            	for (size_t i = 0; i < strlen(msg); i++) {
            		fifo_put(&txBuffer, (uint8_t)msg[i]); // добавляем каждый символ в буфер
            	}
            	txBufferStart(&txBuffer);
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)"Password not changed.\r\n", 25);
            } else {
                HAL_UART_Transmit(&huart6, (uint8_t*)"Password not changed.\r\n", 25, 100);
            }
            break;
        }
    }
}

//void startPasswordChangeIT() {
//    char newPass[PASS_LENGTH] = {0};
//    char c;
//    uint32_t index = 0;
//
//    HAL_UART_Transmit_IT(&huart6, (uint8_t*)"\r\nEnter new password:\r\n", 22, 100);
//
//    while (1) {
//        if (HAL_UART_Receive_IT(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
//            c = tolower((unsigned char)c);
//            if ((c >= 'a' && c <= 'z') && index < PASS_LENGTH) {
//                newPass[index++] = c;
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)&c, 1, 100); // эхо
//            } else if (c == '\r' || c == '\n' || index == PASS_LENGTH) {
//                break;
//            }
//        }
//    }
//
//    HAL_UART_Transmit_IT(&huart6, (uint8_t*)"\r\nActivate new password? (y/n)\r\n", 32, 100);
//
//    while (1) {
//        if (HAL_UART_Receive_IT(&huart6, (uint8_t*)&c, 1, 5000) == HAL_OK) {
//            c = tolower((unsigned char)c);
//            if (c == 'y') {
//                memcpy(pass, newPass, PASS_LENGTH);
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)"Password changed.\r\n", 19, 100);
//                break;
//            } else if (c == 'n') {
//                HAL_UART_Transmit_IT(&huart6, (uint8_t*)"Password not changed.\r\n", 25, 100);
//                break;
//            }
//        }
//    }
//}
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
