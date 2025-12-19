/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 	1024
#define SAMPLE_RATE_HZ	1000000.0f

#define CZT_N			1024U
#define CZT_M			128U
#define CZT_L			2048U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LEN];
volatile uint8_t uart_busy = 0;
volatile uint8_t pending_tx = 0;
volatile uint8_t fft_ready = 0;

uint16_t* fft_input_ptr = NULL;
uint8_t* next_tx_ptr;
uint16_t next_tx_len;

// --- CZT working buffers ---
static float czt_x[CZT_N];
static float czt_fft_buf[2*CZT_L];
static float czt_kernel_buf[2*CZT_L];
static float czt_out_real[CZT_M];
static float czt_out_imag[CZT_M];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void czt_fft(const float *x,
             uint32_t n,
             uint32_t m,
             float w_real, float w_imag,
             float a_real, float a_imag,
             float *out_real,
             float *out_imag);

void run_czt_on_adc_block(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    if (fft_ready)
	    {
	        fft_ready = 0;
	        char buf[128];
			int len = snprintf(buf, sizeof(buf), "\n=== ADC Block ===\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
			HAL_ADC_Stop_DMA(&hadc1);
			for (uint32_t i = 0; i < CZT_N; i++) {
				len = snprintf(buf, sizeof(buf),
					   "adc[%lu] = %u\n",
					   (unsigned long)i,
					   adc_buf[i]);
				HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
			}
			run_czt_on_adc_block();
	        while(1);
	        //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
	    }
    /* USER CODE BEGIN 3 */
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 169;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void czt_fft(const float *x,
             uint32_t n,
             uint32_t m,
             float w_real, float w_imag,
             float a_real, float a_imag,
             float *out_real,
             float *out_imag)
{
    const uint32_t N = n;
    const uint32_t M = m;
    const uint32_t L = CZT_L;

    // 1) y[n] = x[n] * a^{-n} * w^{n^2/2}
    for (uint32_t i = 0; i < L; i++) {
    	czt_fft_buf[2*i]   = 0.0f;
    	czt_fft_buf[2*i+1] = 0.0f;
    }

    float a_arg  = atan2f(a_imag, a_real);
    float w_arg  = atan2f(w_imag, w_real);

    for (uint32_t n_idx = 0; n_idx < N; n_idx++)
    {
        float rn = (float)n_idx;

        float ang = (-rn * a_arg) + (0.5f * rn * rn * w_arg);

        float c_real = arm_cos_f32(ang);
        float c_imag = arm_sin_f32(ang);

        czt_fft_buf[2*n_idx]   = x[n_idx] * c_real;
        czt_fft_buf[2*n_idx+1] = x[n_idx] * c_imag;
    }

    char buf[64];
	int len = snprintf(buf, sizeof(buf), "\n=== stage0 input ===\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

	for (uint32_t i = 0; i < N; i++)
	{
		char buf[128];
		int len = snprintf(buf, sizeof(buf),
						   "x[%lu] = %.6f\n",
						   (unsigned long)i,
						   x[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
	}

	len = snprintf(buf, sizeof(buf), "\n=== stage1 ===\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

	// 디버깅 Stage 1: y[n]
	for (uint32_t i = 0; i < N; i++)
	{
		len = snprintf(buf, sizeof(buf),
						   "y[%lu] = %.6f %.6f\n",
						   (unsigned long)i,
						   czt_fft_buf[2*i],
						   czt_fft_buf[2*i+1]);
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
	}

    // 2) convolution kernel v
    for (uint32_t i = 0; i < L; i++) {
    	czt_kernel_buf[2*i]   = 0.0f;
    	czt_kernel_buf[2*i+1] = 0.0f;
    }

    for (uint32_t k = 0; k < M; k++)
    {
        float rk = (float)k;
        float ang = -0.5f * rk * rk * w_arg;

        czt_kernel_buf[2*k]     = arm_cos_f32(ang);
        czt_kernel_buf[2*k+1] = arm_sin_f32(ang);
    }

    for (uint32_t k = 1; k < N; k++)
    {
        float rk = (float)k;
        float ang = -0.5f * rk * rk * w_arg;

        czt_kernel_buf[2*(L - k)]     = arm_cos_f32(ang);
        czt_kernel_buf[2*(L - k) + 1] = arm_sin_f32(ang);

    }

    len = snprintf(buf, sizeof(buf), "\n=== stage2 ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    // 디버깅 Stage 2: v[k] 일부 UART 출력 (k=0~9)
    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "v[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           czt_kernel_buf[2*i],
                           czt_kernel_buf[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    len = snprintf(buf, sizeof(buf), "\n=== stage2 ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    // 디버깅 Stage 2: v[k] 일부 UART 출력 (k=0~9)
    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "v[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           czt_v_real[i],
                           czt_v_imag[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // 3) FFT(y), FFT(v)
    arm_cfft_radix2_instance_f32 fft_inst;
    arm_cfft_radix2_init_f32(&fft_inst, L, 0, 1);

    arm_cfft_radix2_f32(&fft_inst, czt_fft_buf);
    arm_cfft_radix2_f32(&fft_inst, czt_kernel_buf);

    len = snprintf(buf, sizeof(buf), "\n=== stage3: FFT(Y), FFT(V) ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "Y[%lu] = %.6f %.6f , V[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           czt_fft_buf[2*i], czt_fft_buf[2*i+1],
                           (unsigned long)i,
                           czt_kernel_buf[2*i], czt_kernel_buf[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // Multiply G = Y * V
    for (uint32_t i = 0; i < L; i++)
    {
        float Yr = czt_fft_buf[2*i];
        float Yi = czt_fft_buf[2*i+1];
        float Vr = czt_kernel_buf[2*i];
        float Vi = czt_kernel_buf[2*i+1];

        czt_fft_buf[2*i]   = Yr * Vr - Yi * Vi;
        czt_fft_buf[2*i+1] = Yr * Vi + Yi * Vr;
    }

    len = snprintf(buf, sizeof(buf), "\n=== stage3: G = Y*V ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "G[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           czt_fft_buf[2*i],
                           czt_fft_buf[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "G[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           czt_G[2*i],
                           czt_G[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // IFFT(G)
    arm_cfft_radix2_init_f32(&fft_inst, L, 1, 1);
    arm_cfft_radix2_f32(&fft_inst, czt_fft_buf);

    len = snprintf(buf, sizeof(buf), "\n=== stage3: IFFT(G) ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                            "g[%lu] = %.6f %.6f\n",
                            (unsigned long)i,
                            czt_fft_buf[2*i],
                            czt_fft_buf[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }


    len = snprintf(buf, sizeof(buf), "\n=== stage3: IFFT(G) ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                            "g[%lu] = %.6f %.6f\n",
                            (unsigned long)i,
                            czt_G[2*i],
                            czt_G[2*i+1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }


    // 4) Final chirp and output
    for (uint32_t k = 0; k < M; k++)
    {
        float rk = (float)k;
        float ang = 0.5f * rk * rk * w_arg;

        float c_real = arm_cos_f32(ang);
        float c_imag = arm_sin_f32(ang);

        float Gr = czt_fft_buf[2*k];
        float Gi = czt_fft_buf[2*k+1];

        out_real[k] = Gr * c_real - Gi * c_imag;
        out_imag[k] = Gr * c_imag + Gi * c_real;
    }

    len = snprintf(buf, sizeof(buf), "\n=== stage4 ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "X[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           out_real[i],
                           out_imag[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

}

    len = snprintf(buf, sizeof(buf), "\n=== stage4 ===\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < L; i++)
    {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
                           "X[%lu] = %.6f %.6f\n",
                           (unsigned long)i,
                           out_real[i],
                           out_imag[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

}

void run_czt_on_adc_block(void)
{
    if (fft_input_ptr == NULL)
            return;

    float mean = 0.0f;
    for (uint32_t i = 0; i < CZT_N; i++)
        mean += (float)fft_input_ptr[i];

    mean /= (float)CZT_N;

    for (uint32_t i = 0; i < CZT_N; i++)
        czt_x[i] = (float)fft_input_ptr[i] - mean;

    float f_center = 457000.0f;
    float span     = 200.0f;
    float f_start  = f_center - span * 0.5f;   // 456900 Hz
    float f_end    = f_center + span * 0.5f;   // 457100 Hz

    float W_ang = -2.0f * PI * (f_end - f_start) / (CZT_M * SAMPLE_RATE_HZ);
    float A_ang =  2.0f * PI * f_start / SAMPLE_RATE_HZ;

    float W_real = arm_cos_f32(W_ang);
    float W_imag = arm_sin_f32(W_ang);
    float A_real = arm_cos_f32(A_ang);
    float A_imag = arm_sin_f32(A_ang);

<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                        "\n=== stage0: W,A ===\n"
                        "W = %.9f %.9f\n"
                        "A = %.9f %.9f\n",
                        W_real, W_imag,
                        A_real, A_imag);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
    // 3) CZT
    czt_fft(czt_x, CZT_N, CZT_M, W_real, W_imag, A_real, A_imag,
            czt_out_real, czt_out_imag);

    // 4) peak identify
    float max_val = 0.0f;
    uint32_t max_idx = 0;

    for (uint32_t k = 0; k < CZT_M; k++)
    {
        float re = czt_out_real[k];
        float im = czt_out_imag[k];
        float m  = sqrtf(re*re + im*im);

        if (m > max_val)
        {
            max_val = m;
            max_idx = k;
        }
    }

}
/*
void UART_TryStartTx(void)
{
    if (!uart_busy && pending_tx) {
        uart_busy = 1;
        pending_tx = 0;
        HAL_UART_Transmit_IT(&huart1, next_tx_ptr, next_tx_len);
    }
}*/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
    	fft_input_ptr = &adc_buf[0];

    	/*
        next_tx_ptr = (uint8_t*)adc_buf;
        next_tx_len = (ADC_BUF_LEN / 2) * 2; // half-buffer (bytes)
        pending_tx = 1;

        UART_TryStartTx();*/
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
    	//fft_input_ptr = &adc_buf[512];
    	fft_ready = 1;
    	/*
    	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        next_tx_ptr = (uint8_t*)&adc_buf[ADC_BUF_LEN / 2];
        next_tx_len = (ADC_BUF_LEN / 2) * 2;
        pending_tx = 1;

        UART_TryStartTx();*/
    }
}
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_busy = 0;
        UART_TryStartTx();
    }
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
