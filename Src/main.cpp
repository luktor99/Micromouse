/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <threads.h>

int main(void) {
	// Initialize all the hardware
	hardware_init();

	// Prepare the OLED menu
	menu_inflate();

	// Create the threads
	createThreads();

	// Start scheduler
	osKernelStart();

	// We should never get here as control is now taken by the scheduler
	// Infinite loop
	for(;;);
}

extern "C" {
	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
		if(huart == &huart3) {
			// flash TX LED
			//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			LED1_GPIO_Port->BSRR = (uint32_t)LED1_Pin;
			osSemaphoreRelease(uart3tx_semaphore_id);
		}
	}

	void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
		if(hi2c == &hi2c1) {
			osSemaphoreRelease(i2c1_semaphore_id);
		} else if(hi2c == &hi2c2) {
			osSemaphoreRelease(i2c2_semaphore_id);
		} else if(hi2c == &hi2c3) {
			osSemaphoreRelease(i2c3_semaphore_id);
		}
	}

	void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
		if(hi2c == &hi2c1) {
			osSemaphoreRelease(i2c1_semaphore_id);
		} else if(hi2c == &hi2c2) {
			osSemaphoreRelease(i2c2_semaphore_id);
		} else if(hi2c == &hi2c3) {
			osSemaphoreRelease(i2c3_semaphore_id);
		}
	}

	void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
		if(hi2c == &hi2c1) {
			osSemaphoreRelease(i2c1_semaphore_id);
		} else if(hi2c == &hi2c2) {
			osSemaphoreRelease(i2c2_semaphore_id);
		} else if(hi2c == &hi2c3) {
			osSemaphoreRelease(i2c3_semaphore_id);
		}
	}

	void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
		if(hi2c == &hi2c1) {
			osSemaphoreRelease(i2c1_semaphore_id);
		} else if(hi2c == &hi2c2) {
			osSemaphoreRelease(i2c2_semaphore_id);
		} else if(hi2c == &hi2c3) {
			osSemaphoreRelease(i2c3_semaphore_id);
		}
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if(htim == &htim9) {
			osSignalSet(motionControlTaskHandle, 1);
		}
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
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
