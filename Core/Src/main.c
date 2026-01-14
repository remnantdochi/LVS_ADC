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
#define ADC_BUF_LEN 	2048U
#define SAMPLE_RATE_HZ	25000.71f

#define CZT_N			1024U
#define CZT_M			256U
#define CZT_L			2048U

#define CENTER_FREQ_HZ 	457000.0f
#define SPAN_HZ			200.0f

#define PRINT
#define LVS_MAGIC 0x3053564CU //'LVS0'
#define UART_TX_RING_SIZE 16384U
#define UART_TX_DMA_CHUNK 256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LEN];
volatile uint8_t adc_ready = 0;
volatile uint16_t* czt_input_ptr = NULL;

// --- CZT working buffers ---
static float czt_x[CZT_N];
static float czt_fft_buf[2*CZT_L];

static float czt_out_real[CZT_M];
static float czt_out_imag[CZT_M];

static float pre_chirp_table[2 * CZT_N];
static float post_chirp_table[2 * CZT_M];
static float kernel_fft_precalc[2 * CZT_L];

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint32_t seq;
	uint32_t time; //czt processing time in ms
	uint32_t dropADC; //dropped ADC blocks
	uint32_t dropFrame; //dropped frame
	float	mag[CZT_M];
} lvs_frame_t;

static lvs_frame_t tx_frame;
static volatile uint32_t frame_seq = 0;
static volatile uint32_t uart_adc_drop_blocks = 0;
static volatile uint32_t czt_busy = 0;

static uint8_t uart_tx_ring[UART_TX_RING_SIZE];
static volatile uint32_t uart_tx_w = 0;
static volatile uint32_t uart_tx_r = 0;
static volatile uint8_t uart_tx_in_progress = 0;

static uint8_t uart_tx_dma_buf[UART_TX_DMA_CHUNK];
static volatile uint32_t uart_tx_drop_frames = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void init_czt_constants(void);
void czt_fft(const float *x, float *out_real, float *out_imag);
void run_czt_on_adc_block(void);

static void uart_queue_tx(const uint8_t *buf, uint32_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_czt_constants(void)
{
    float f_center = CENTER_FREQ_HZ;
    float span     = SPAN_HZ;
    float f_start  = f_center - span * 0.5f;
    float f_end    = f_center + span * 0.5f;

    float W_ang = -2.0f * PI * (f_end - f_start) / (CZT_M * SAMPLE_RATE_HZ);
    float A_ang =  2.0f * PI * f_start / SAMPLE_RATE_HZ;

    // use arm_math sin/cos since it is more accurate than CORDIC and done only once

    // 1) Pre-chirp table creation
    for (uint32_t n = 0; n < CZT_N; n++) {
        float rn = (float)n;
        float ang = (-rn * A_ang) + (0.5f * rn * rn * W_ang);

        pre_chirp_table[2*n]   = arm_cos_f32(ang); // Real
        pre_chirp_table[2*n+1] = arm_sin_f32(ang); // Imag
    }

    // 2) Post-chirp table creation
    for (uint32_t k = 0; k < CZT_M; k++) {
        float rk = (float)k;
        float ang = 0.5f * rk * rk * W_ang;

        post_chirp_table[2*k]   = arm_cos_f32(ang);
        post_chirp_table[2*k+1] = arm_sin_f32(ang);
    }

    // 3) Kernel creation and precompute FFT
    // First, create time-domain kernel in temporary buffer
    memset(kernel_fft_precalc, 0, 2 * CZT_L * sizeof(float));

    // Kernel front part
    for (uint32_t k = 0; k < CZT_M; k++) {
        float rk = (float)k;
        float ang = -0.5f * rk * rk * W_ang;
        kernel_fft_precalc[2*k]     = arm_cos_f32(ang);
        kernel_fft_precalc[2*k+1]   = arm_sin_f32(ang);
    }
    // Kernel remaining part
    for (uint32_t k = 1; k < CZT_N; k++) {
        float rk = (float)k;
        float ang = -0.5f * rk * rk * W_ang;
        kernel_fft_precalc[2*(CZT_L - k)]     = arm_cos_f32(ang);
        kernel_fft_precalc[2*(CZT_L - k) + 1] = arm_sin_f32(ang);
    }

    // Kernel FFT precomputation
    arm_cfft_radix2_instance_f32 fft_inst;
    arm_cfft_radix2_init_f32(&fft_inst, CZT_L, 0, 1);
    arm_cfft_radix2_f32(&fft_inst, kernel_fft_precalc);
}

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
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  init_czt_constants();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (adc_ready)
	  {
			adc_ready = 0;
#ifdef PRINT
      
			//HAL_ADC_Stop_DMA(&hadc1);

			czt_busy = 1;
			uint32_t c0 = DWT->CYCCNT;
			run_czt_on_adc_block();
			uint32_t c1 = DWT->CYCCNT;
			czt_busy = 0;

			uint32_t cycles = c1-c0;

			tx_frame.magic = LVS_MAGIC;
			tx_frame.seq = frame_seq++;
			tx_frame.time = cycles / 170000.0f; //170Mhz -> ms
			tx_frame.dropADC = uart_adc_drop_blocks;
			tx_frame.dropFrame = uart_tx_drop_frames;

			for (uint32_t k = 0; k < CZT_M; k++)
			{
				float mag = sqrtf(
				czt_out_real[k]*czt_out_real[k] +
				czt_out_imag[k]*czt_out_imag[k]);
				tx_frame.mag[k] = mag;
			}
      	  	uart_queue_tx((const uint8_t*)&tx_frame, sizeof(tx_frame));
			//while(1);
#else
			run_czt_on_adc_block();
#endif

	  }
    /* USER CODE END WHILE */

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
  htim6.Init.Prescaler = 67;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
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
  huart1.Init.BaudRate = 2000000;
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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

void czt_fft(const float *x, float *out_real, float *out_imag)
{
	// 1) y[n] = x[n] * a^{-n} * w^{n^2/2}
	arm_cmplx_mult_real_f32(pre_chirp_table, x, czt_fft_buf, CZT_N);
	memset(&czt_fft_buf[2*CZT_N], 0, (CZT_L - CZT_N) * 2 * sizeof(float));

	// 2) v[n] is initialized in kernel_fft_precalc (FFT done already)

	// 3) FFT(y), FFT(v)
	arm_cfft_radix2_instance_f32 fft_inst;
	arm_cfft_radix2_init_f32(&fft_inst, CZT_L, 0, 1);
	arm_cfft_radix2_f32(&fft_inst, czt_fft_buf);
	//kernel_fft_precalc already FFTed

	// Multiply G = Y * V
	arm_cmplx_mult_cmplx_f32(czt_fft_buf, kernel_fft_precalc, czt_fft_buf, CZT_L);

	// IFFT(G)
	arm_cfft_radix2_init_f32(&fft_inst,CZT_L, 1, 1);
	arm_cfft_radix2_f32(&fft_inst, czt_fft_buf);

	// 4) Final chirp and output
	float *ptr_src = czt_fft_buf;
	const float *ptr_chirp = post_chirp_table;
	//post kernal is precomputed

	for (uint32_t k = 0; k < CZT_M; k++)
	{
	  float Gr = *ptr_src++;
	  float Gi = *ptr_src++;

	  float c_real = *ptr_chirp++;
	  float c_imag = *ptr_chirp++;

	  out_real[k] = Gr * c_real - Gi * c_imag;
	  out_imag[k] = Gr * c_imag + Gi * c_real;
	}
}


void run_czt_on_adc_block(void)
{
    uint16_t *src_buf = czt_input_ptr;

    float mean = 0.0f;
    for (uint32_t i = 0; i < CZT_N; i++)
        mean += (float)src_buf[i];

    mean /= (float)CZT_N;

    for (uint32_t i = 0; i < CZT_N; i++)
        czt_x[i] = (float)src_buf[i] - mean;

    // 3) CZT
    czt_fft(czt_x, czt_out_real, czt_out_imag);

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
      if (czt_busy || adc_ready) 
      {
        uart_adc_drop_blocks++;
        return;
      }
    	czt_input_ptr = &adc_buf[0];
    	adc_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
      if (czt_busy || adc_ready) 
      {
        uart_adc_drop_blocks++;
        return;
      }
    	czt_input_ptr = &adc_buf[CZT_N];
    	adc_ready = 1;
    }
}

//last slot is unused to distinguish full/empty
static inline uint32_t uart_ring_used(void)
{
  uint32_t w = uart_tx_w, r = uart_tx_r;
  return (w >= r) ? (w - r) : (UART_TX_RING_SIZE - (r - w));
}
static inline uint32_t uart_ring_free(void)
{
  return (UART_TX_RING_SIZE - 1u) - uart_ring_used();
}

static void uart_kick_tx(void)
{
  if (uart_tx_in_progress) return;
  if (uart_tx_r == uart_tx_w) return; //empty

  //read from ring into chunk
  uint32_t n = 0;
  while ((n < UART_TX_DMA_CHUNK) && (uart_tx_r != uart_tx_w)) {
    uart_tx_dma_buf[n++] = uart_tx_ring[uart_tx_r];
    uart_tx_r = (uart_tx_r + 1u) % UART_TX_RING_SIZE;
  }
  if (n == 0) return;

  uart_tx_in_progress = 1;
  if (HAL_UART_Transmit_DMA(&huart1, uart_tx_dma_buf, (uint16_t)n) != HAL_OK) {
    uart_tx_r = (uart_tx_r + UART_TX_RING_SIZE - n) % UART_TX_RING_SIZE;
    uart_tx_in_progress = 0;
  }
}

static void uart_queue_tx(const uint8_t *buf, uint32_t len)
{
  if (!buf || !len) return;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t free = uart_ring_free();
  if (free < len) {
    uart_tx_drop_frames ++;
    if (!primask) __enable_irq();
    return;
  }

  //copy into ring
  for (uint32_t i = 0; i < len; i++) {
    uart_tx_ring[uart_tx_w] = buf[i];
    uart_tx_w = (uart_tx_w + 1u) % UART_TX_RING_SIZE;
  }

  if (!primask) __enable_irq();

  uart_kick_tx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uart_tx_in_progress = 0;
    uart_kick_tx();
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
