/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIR_PIN          GPIO_PIN_1
#define DIR_GPIO_PORT    GPIOB
#define PWM_MAX_CMD      1000     /* 协议 pwm 上下界（千分比，[-1000,1000]） */
#define PWM_ARR          3599U    /* TIM1 ARR；f_PWM = 72MHz/(PSC+1)/(ARR+1) = 72e6/1/3600 = 20kHz */
#define STREAM_PERIOD_MS 5U       /* 200Hz 状态流 */
#define CMD_TIMEOUT_MS   100U     /* 命令看门狗：超过此时长无有效 P 命令则停机 */
#define MAX_CART_COUNT   1565     /* 编码器越界阈值 = 轨道半程 0.181 m / 1.154e-4 m/count（编码器无 x4 倍频，直尺实测标定），与 bridge.py 的 TRACK_HALF_LENGTH 保持一致 */
#define ADC_OVERSAMPLE   32       /* ADC 过采样次数：同一 5ms 窗口内平均降噪，√N 倍，不引入相位滞后 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t         s_pwm_cmd = 0;     /* PC 下发的有符号指令 [-1000,1000] */
static uint32_t        s_last_cmd_tick = 0; /* 最近一次收到有效 P 命令的 tick */
static int32_t         s_cart_count = 0;  /* 编码器 32 位累加位置，不回绕 */
static uint16_t        s_prev_cnt = 0;    /* 上次采样 TIM2 CNT，用于算 delta */
static char            rx_line[64];       /* 命令行缓冲 */
static volatile uint8_t rx_ready = 0;     /* ISR 置位表示 rx_line 有一完整行待消费 */
static uint8_t         rx_len = 0;        /* rx_line 当前长度（ISR 维护） */
static uint8_t         rx_byte;           /* HAL_UART_Receive_IT 的单字节缓冲 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void send_state(int32_t count, uint16_t adc);
static void apply_motor(void);
static void parse_line(char *s);
static int  int32_to_dec(char *buf, int32_t v);
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
  uint32_t last_stream;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc1);

  s_prev_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  apply_motor();                       /* 上电默认 P 0：电机停 */

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  last_stream = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last_stream >= STREAM_PERIOD_MS) {
      uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
      uint16_t adc;
      uint32_t adc_acc = 0;
      int i;
      last_stream = HAL_GetTick();
      s_cart_count += (int16_t)(cnt - s_prev_cnt);   /* int16 回绕自动取 mod 65536 */
      s_prev_cnt = cnt;
      /* ADC 过采样平均：同一 5ms 窗口内采 N 次取平均，√N 倍降噪，不引入相位滞后 */
      for (i = 0; i < ADC_OVERSAMPLE; i++) {
        while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) == RESET) { /* 等新一次转换完成 */ }
        adc_acc += HAL_ADC_GetValue(&hadc1);
      }
      adc = (uint16_t)(adc_acc / ADC_OVERSAMPLE);
      send_state(s_cart_count, adc);

      HAL_NVIC_DisableIRQ(USART1_IRQn);
      if (rx_ready) {
        parse_line(rx_line);
        rx_ready = 0;
      }
      HAL_NVIC_EnableIRQ(USART1_IRQn);
    }

    /* 命令看门狗：PC 停止下发（bridge 崩溃 / USB 断开）则停机 */
    if (s_pwm_cmd != 0 && (HAL_GetTick() - s_last_cmd_tick) > CMD_TIMEOUT_MS) {
      s_pwm_cmd = 0;
      apply_motor();
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ---- 有符号 int32 转十进制字符串，返回写入字符数（不含终止符）---- */
static int int32_to_dec(char *buf, int32_t v)
{
  char tmp[11];
  int i = 0, n = 0;
  uint32_t u;
  if (v < 0) {
    buf[n++] = '-';
    u = 0U - (uint32_t)v;            /* 无符号取负，规避 INT32_MIN 的有符号溢出 UB */
  } else {
    u = (uint32_t)v;
  }
  if (u == 0U) {
    tmp[i++] = '0';
  }
  while (u > 0U) {
    tmp[i++] = (char)('0' + (u % 10U));
    u /= 10U;
  }
  while (i > 0) {
    buf[n++] = tmp[--i];
  }
  return n;
}

/* ---- 发送 "S <cart_count> <pole_adc>\n" ---- */
static void send_state(int32_t count, uint16_t adc)
{
  char line[24];
  int n = 0;
  line[n++] = 'S';
  line[n++] = ' ';
  n += int32_to_dec(&line[n], count);
  line[n++] = ' ';
  n += int32_to_dec(&line[n], (int32_t)adc);
  line[n++] = '\n';
  HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, 5);  /* 一行≤19字节≈1.7ms；短超时避免主机不读时阻塞主循环 */
}

/* ---- 按 s_pwm_cmd 设置 DIR + PWM 占空比（per-mille 缩放到 ARR，纯寄存器写）---- */
static void apply_motor(void)
{
  int32_t mag;
  uint32_t ccr;

  /* 位置钳制：越过轨道端头后禁止继续往外推（允许往回纠偏）。
     前提：上电时小车停在轨道中间，s_cart_count 从上电起累加，零位≈轨道中心。 */
  if (s_cart_count > MAX_CART_COUNT && s_pwm_cmd > 0) {
    s_pwm_cmd = 0;
  } else if (s_cart_count < -MAX_CART_COUNT && s_pwm_cmd < 0) {
    s_pwm_cmd = 0;
  }

  mag = (s_pwm_cmd >= 0) ? s_pwm_cmd : -s_pwm_cmd;                /* |pwm|, 0..PWM_MAX_CMD */
  ccr = (uint32_t)mag * PWM_ARR / PWM_MAX_CMD;                    /* per-mille -> CCR(0..ARR) */
  if (s_pwm_cmd >= 0) {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);    /* +x 方向 */
  } else {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);      /* -x 方向 */
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

/* ---- 解析 "P <int>"，钳位 [-1000,1000] 并驱动电机；非法行丢弃，保持上一条 ---- */
static void parse_line(char *s)
{
  int32_t v = 0;
  int neg = 0;
  char *p;

  if (s[0] != 'P') {
    return;
  }
  p = s + 1;
  while (*p == ' ' || *p == '\t') {
    p++;
  }
  if (*p == '-') {
    neg = 1;
    p++;
  } else if (*p == '+') {
    p++;
  }
  if (*p < '0' || *p > '9') {
    return;                          /* 无数字：丢弃，保持上一条命令 */
  }
  while (*p >= '0' && *p <= '9') {
    v = v * 10 + (*p - '0');
    if (v > 1000000L) {
      v = 1000000L;                  /* 防累加溢出 */
    }
    p++;
  }
  if (neg) {
    v = -v;
  }
  if (v < -PWM_MAX_CMD) {
    v = -PWM_MAX_CMD;
  } else if (v > PWM_MAX_CMD) {
    v = PWM_MAX_CMD;
  }
  s_pwm_cmd = (int16_t)v;
  s_last_cmd_tick = HAL_GetTick();
  apply_motor();
}

/* ---- USART1 RX 单字节中断回调：逐字节组装命令行 ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    uint8_t b = rx_byte;
    if (!rx_ready) {
      if (b == '\n' || b == '\r') {
        if (rx_len > 0U) {
          rx_line[rx_len] = '\0';
          rx_ready = 1;              /* 通知主循环消费 */
        }
        rx_len = 0;
      } else if (rx_len < (uint8_t)(sizeof(rx_line) - 1U)) {
        rx_line[rx_len++] = (char)b;
      }
      /* else: 行超长，丢弃后续字节直到换行 */
    }
    /* else: 主循环尚未消费上一行，丢弃本字节（有界丢弃，不破坏当前行） */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  /* 续挂下一次接收 */
  }
}

/* ---- UART 错误（overrun/framing 等）：重置行缓冲并续挂 RX，避免接收卡死 ---- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    rx_len = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

/* ---- USART1 中断入口。
   注意：若将来在 CubeMX 启用 USART1 全局中断并重新生成，
   stm32f1xx_it.c 会生成同名函数，届时需删除此处定义，改用那边的。 ---- */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
