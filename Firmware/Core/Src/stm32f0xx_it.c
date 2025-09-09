/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

volatile uint8_t g_tx_index;   // 송신할 데이터의 인덱스
volatile uint8_t g_requested_register; // 마스터가 요청한 레지스터 주소
extern volatile uint8_t tx_length;
extern volatile uint8_t* tx_buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
 */
void I2C1_IRQHandler(void)
{
	/* USER CODE BEGIN I2C1_IRQn 0 */
	// NACK (Not Acknowledge) 감지: 마스터가 수신을 거부함 (전송 종료 신호)
	if (LL_I2C_IsActiveFlag_NACK(I2C1))
	{
		// NACK 플래그를 클리어하고, 다음 통신을 위해 상태를 초기화.
		LL_I2C_ClearFlag_NACK(I2C1);
	}
	// 주소 일치 (Address Match) 인터럽트
	if (LL_I2C_IsActiveFlag_ADDR(I2C1))
	{
		LL_I2C_ClearFlag_ADDR(I2C1);
		// 마스터가 읽기를 시작하면(슬레이브는 송신), 송신 인덱스를 초기화.
		if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
		{
			g_tx_index = 0;
			// 다음 전송을 위해 TXIS 인터럽트를 다시 활성화
			LL_I2C_EnableIT_TX(I2C1);
		}
	}

	// 송신 버퍼가 비어 있음 (Transmit Interrupt Status)
	if (LL_I2C_IsActiveFlag_TXIS(I2C1))
	{
		// 마스터가 0x10 레지스터를 요청했고, 아직 보낼 데이터가 남았다면
		if ((g_requested_register == 0x10) && (g_tx_index < tx_length))
		{
			LL_I2C_TransmitData8(I2C1, tx_buffer[g_tx_index]);
			g_tx_index++;
		}
		else
		{
			// 보낼 데이터가 없으면, TXIS 인터럽트를 비활성화하여 무한 루프 방지
			LL_I2C_DisableIT_TX(I2C1);
		}
	}
	// STOP 감지됨
	if (LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		LL_I2C_ClearFlag_STOP(I2C1);

		// 다음 통신을 위해 모든 상태 변수들을 깨끗하게 초기화
		g_tx_index = 0;
		g_requested_register = 0;

		LL_I2C_DisableIT_TX(I2C1);
	}

	// 수신 버퍼에 데이터 있음 (Receive Data Register Not Empty)
	if (LL_I2C_IsActiveFlag_RXNE(I2C1))
	{
		// 마스터가 보낸 레지스터 주소를 읽음
		g_requested_register = LL_I2C_ReceiveData8(I2C1);
	}

	/* USER CODE END I2C1_IRQn 0 */
	/* USER CODE BEGIN I2C1_IRQn 1 */

	/* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
