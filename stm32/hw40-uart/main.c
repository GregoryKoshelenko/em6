// Lesson 40 (module 4.1): UART link STM32 (Nucleo-L031K6) <-> ESP32-S3, STM32 side.
//  - "T" received from ESP32 -> toggle the STM32 LED, reply "L0"/"L1" (done in class)
//  - STM32 button press -> send "T" -> ESP32 toggles its LED (homework)
//
// Line protocol, 8N1, every message ends with '\n':
//   "T" toggle your LED | "L0"/"L1" ack with the new LED state | "#..." log, ignored
//
// Link = USART2. In Wokwi it is the VCP (PA2/PA15) wired to the Serial Monitor,
// so the monitor plays the ESP32 side (type T + Enter). On the real board wire
// ESP GPIO17 (TX) -> STM32 RX, ESP GPIO18 (RX) <- STM32 TX, GND <-> GND.

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

#define VCP_TX_Pin              GPIO_PIN_2
#define VCP_RX_Pin              GPIO_PIN_15

#define LINK_BAUD               115200U  // must match the ESP32 side
#define DEBOUNCE_MS             50U
#define LINE_LEN                16U

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_USART2_UART_Init(void);

void osSystickHandler(void)
{
}

static void initGPIO(void)
{
  GPIO_InitTypeDef GPIO_Config = {0};

  GPIO_Config.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_Config.Pull  = GPIO_NOPULL;
  GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Config.Pin   = LED_Pin;
  LED_PORT_CLK_ENABLE();
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_Config);

  GPIO_Config.Pin  = BTN_Pin;
  GPIO_Config.Mode = GPIO_MODE_INPUT;
  GPIO_Config.Pull = GPIO_PULLUP;
  BTN_PORT_CLK_ENABLE();
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_Config);
}

// ---- Button: polled debounce, one event per press ----
typedef struct {
  uint8_t  lastRaw;
  uint8_t  stable;
  uint32_t changedAt;
} Button;

static uint8_t buttonPressed(Button *b, uint32_t now)
{
  const uint8_t raw = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET;
  if (raw != b->lastRaw) { b->lastRaw = raw; b->changedAt = now; }
  if (now - b->changedAt >= DEBOUNCE_MS && raw != b->stable)
  {
    b->stable = raw;
    return b->stable;
  }
  return 0;
}

// ---- Non-blocking line reader ----
typedef struct {
  char    buf[LINE_LEN];
  uint8_t len;
  uint8_t overflow;
} LineReader;

// Returns 1 when a full line is in r->buf.
static uint8_t lineFeed(LineReader *r, char c)
{
  if (c == '\r') return 0;
  if (c == '\n')
  {
    r->buf[r->len] = '\0';
    const uint8_t ready = r->len > 0 && !r->overflow;
    r->len = 0;
    r->overflow = 0;
    return ready;
  }
  if (r->len < LINE_LEN - 1) r->buf[r->len++] = c;
  else r->overflow = 1;                      // drop the over-long line
  return 0;
}

// Reads one byte if the RX register has data; never waits.
static uint8_t linkReadByte(uint8_t *byte)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);  // overrun: drop and go on
  if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) return 0;
  *byte = (uint8_t)(huart2.Instance->RDR & 0xFF);
  return 1;
}

static void handleLine(const char *msg)
{
  if (msg[0] == '#') return;                             // peer log
  if (strcmp(msg, "T") == 0)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    const uint8_t on = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET;
    printf(on ? "L1\n" : "L0\n");                        // ack with the new state
  }
  else if (msg[0] == 'L' && (msg[1] == '0' || msg[1] == '1') && msg[2] == '\0')
  {
    printf("# peer LED is %s\n", msg[1] == '1' ? "ON" : "OFF");
  }
  else
  {
    printf("# unknown command: %s\n", msg);
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  initGPIO();
  MX_USART2_UART_Init();

  printf("# STM32 ready: button -> T, T -> toggle LED\n");

  Button     button = {0};
  LineReader reader = {0};

  while (1)
  {
    if (buttonPressed(&button, HAL_GetTick())) printf("T\n");

    uint8_t byte;
    while (linkReadByte(&byte))
    {
      if (lineFeed(&reader, (char)byte)) handleLine(reader.buf);
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
  huart2.Init.BaudRate = LINK_BAUD;
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
