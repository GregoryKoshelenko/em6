// Lesson 29 (2.8) - STM32 intro: LED reacts to a button (STM32 HAL)
// Board: Nucleo-L031K6. Based on Wokwi example 367244067477216257
// Simulation: https://wokwi.com/projects/481315097601800193

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stm32l0xx_hal.h>

// ST Nucleo green user LED LD3 (PB3 / D13)
#define LED_GPIO_Port           GPIOB
#define LED_Pin                 GPIO_PIN_3
#define LED_PORT_CLK_ENABLE     __HAL_RCC_GPIOB_CLK_ENABLE

// Push button on D2 (PA12), other leg to GND -> pressed = GPIO_PIN_RESET
#define BTN_GPIO_Port           GPIOA
#define BTN_Pin                 GPIO_PIN_12
#define BTN_PORT_CLK_ENABLE     __HAL_RCC_GPIOA_CLK_ENABLE

#define BLINK_SLOW_MS           500U   // button released
#define BLINK_FAST_MS           100U   // button held
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_RX_Pin GPIO_PIN_15

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_USART2_UART_Init(void);

void osSystickHandler(void)
{
}

void initGPIO()
{
  GPIO_InitTypeDef GPIO_Config;

  GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Config.Pull = GPIO_NOPULL;
  GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_Config.Pin = LED_Pin;

  LED_PORT_CLK_ENABLE();
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_Config);

  GPIO_Config.Pin  = BTN_Pin;
  GPIO_Config.Mode = GPIO_MODE_INPUT;
  GPIO_Config.Pull = GPIO_PULLUP;

  BTN_PORT_CLK_ENABLE();
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_Config);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  initGPIO();
  MX_USART2_UART_Init();

  printf("Hold the button to blink faster\n");

  uint32_t lastToggle = HAL_GetTick();
  uint8_t  wasPressed = 0;

  while (1)
  {
    const uint8_t pressed =
        HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET;

    if (pressed != wasPressed)
    {
      printf(pressed ? "Button pressed: fast blink\n" : "Button released: slow blink\n");
      wasPressed = pressed;
    }

    // Non-blocking version of HAL_GPIO_TogglePin() + HAL_Delay() from the lesson
    const uint32_t period = pressed ? BLINK_FAST_MS : BLINK_SLOW_MS;
    if (HAL_GetTick() - lastToggle >= period)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      lastToggle = HAL_GetTick();
    }
  }

  return 0;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
    in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
    @brief USART2 Initialization Function
    @param None
    @retval None
*/
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15     ------> USART2_RX
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = VCP_TX_Pin | VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_USART2_CLK_ENABLE();
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}


// The following makes printf() write to USART2:

#define STDOUT_FILENO   1
#define STDERR_FILENO   2

int _write(int file, uint8_t *ptr, int len)
{
  switch (file)
  {
    case STDOUT_FILENO:
      HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
      break;

    case STDERR_FILENO:
      HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
      break;

    default:
      return -1;
  }

  return len;
}
