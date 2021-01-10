#include "main.h"
#include "fatfs.h"
#include "delay.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2   /* Start @ of user Flash area */
#define FLASH_END_ADDR          (ADDR_FLASH_SECTOR_11 + 0x00020000)

SD_HandleTypeDef hsd;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
//static void MX_USART1_UART_Init(void);
static void panic();
static void signal(int u_blink, int v_blink);
static void Jump_to_app();
static uint8_t flash_firmware(FIL*, uint32_t size);
static uint8_t erase_flash(uint32_t address, uint32_t size);
static uint8_t flash_buffer(uint32_t address, uint32_t size);
static uint8_t verify_firmware(FIL*, uint32_t size);
static uint8_t verify_buffer(uint32_t address, uint32_t size);
extern uint8_t sd_use_4b_mode;

#if 0
void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}
#endif

static FATFS fs;
static FIL fw_file;
static FILINFO info;

int main(void)
{
  uint8_t res;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SDIO_SD_Init();
  //MX_USART1_UART_Init();
  MX_FATFS_Init();
  DWT_Delay_Init();
  if (f_mount(&fs, "", 1) != FR_OK) {
    Jump_to_app();
  }
  if (f_stat("firmware.bin", &info) != FR_OK) {
    Jump_to_app();
  }
  if (f_open(&fw_file, "firmware.bin", FA_READ) != FR_OK) {
    Jump_to_app();
  }
  res = flash_firmware(&fw_file, info.fsize);
  if (res != 0) {
    f_close(&fw_file);
    panic();
  }
  res = verify_firmware(&fw_file, info.fsize);
  f_close(&fw_file);
  if (res != 0) {
    panic();
  }
  f_unlink("0:/firmware.cur");
  f_rename("0:/firmware.bin", "0:/firmware.cur");
  Jump_to_app();
}

static void panic()
{
  while(1)
  {
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    DWT_Delay_ms(150);
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(150);
    LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    DWT_Delay_ms(150);
    LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(150);
  }
}

static void signal(int u_blink, int v_blink)
{
  if (u_blink < 0) {
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
  }
  if (v_blink < 0) {
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
  }
  for(;u_blink>0;u_blink--) {
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    DWT_Delay_ms(150);
    LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    DWT_Delay_ms(150);
  }
  for(;v_blink>0;v_blink--) {
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(150);
    LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(150);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  sd_use_4b_mode = 0;
}

#if 0
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
}
#endif

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, Y_MAX_Pin|ST_ENABLE_Pin|X_MIN_Pin|Y_MIN_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, Z_MIN_Pin|X_MAX_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOE, Z_MAX_Pin|U_MIN_Pin|V_MIN_Pin);

  /**/
  GPIO_InitStruct.Pin = PWM7_Pin|PWM8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Y_MAX_Pin|X_MIN_Pin|Y_MIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ST_ENABLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ST_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Z_MIN_Pin|X_MAX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Z_MAX_Pin|U_MIN_Pin|V_MIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWM9_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(PWM9_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWM6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(PWM6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWM1_Pin|PWM2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWM3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(PWM3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWM4_Pin|PWM5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  panic();
}

static int32_t GetSector(uint32_t Address)
{
  int32_t sector = 0;
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else
  {
    sector = -1;
  }

  return sector;
}

static uint32_t io_buffer[16384/4];
static uint8_t flash_firmware(FIL* file, uint32_t size) {
  uint32_t fw_prog_addr = FLASH_USER_START_ADDR;
  UINT buffer_size;
  uint8_t res;
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return 1;
  }
  LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
  LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
  res = erase_flash(FLASH_USER_START_ADDR, size);
  LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
  LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
  if (res != 0) {
    HAL_FLASH_Lock();
    for (buffer_size = 5; buffer_size; buffer_size--) {
      signal(2, res);
      DWT_Delay_ms(2000);
    }
    return 2;
  };
  while (size) {
    buffer_size = sizeof(io_buffer);
    if (buffer_size > size) {
      buffer_size = size;
    }
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    if (f_read(file, io_buffer, buffer_size, &buffer_size) != FR_OK) {
      HAL_FLASH_Lock();
      LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
      LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
      for (buffer_size = 5; buffer_size; buffer_size--) {
        signal(3, 3);
        DWT_Delay_ms(2000);
      }
      return 3;
    }
    LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    res = flash_buffer(fw_prog_addr, buffer_size / 4);
    if (res != 0) {
      HAL_FLASH_Lock();
      LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
      LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
      for (buffer_size = 5; buffer_size; buffer_size--) {
        signal(4, res);
        DWT_Delay_ms(2000);
      }
      return 4;
    }
    LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    fw_prog_addr += buffer_size;
    size -= buffer_size;
  }
  HAL_FLASH_Lock();
  return 0;
}

static uint8_t verify_firmware(FIL* file, uint32_t size) {
  uint32_t fw_prog_addr = FLASH_USER_START_ADDR;
  UINT buffer_size;
  uint8_t res;
  if (f_lseek(file, 0) != FR_OK) {
    for (buffer_size = 5; buffer_size; buffer_size--) {
      signal(5, res);
      DWT_Delay_ms(2000);
    }
    return 5;
  }
  while (size) {
    buffer_size = sizeof(io_buffer);
    if (buffer_size > size) {
      buffer_size = size;
    }
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    if (f_read(file, io_buffer, buffer_size, &buffer_size) != FR_OK) {
      LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
      LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
      for (buffer_size = 5; buffer_size; buffer_size--) {
        signal(3, 3);
        DWT_Delay_ms(2000);
      }
      return 3;
    }
    LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    res = verify_buffer(fw_prog_addr, buffer_size / 4);
    if (res != 0) {
      LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
      LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
      for (buffer_size = 5; buffer_size; buffer_size--) {
        signal(4, res);
        DWT_Delay_ms(2000);
      }
      return 4;
    }
    LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    fw_prog_addr += buffer_size;
    size -= buffer_size;
  }
  for (buffer_size = 6; buffer_size; buffer_size--) {
    LL_GPIO_ResetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    LL_GPIO_ResetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(250);
    LL_GPIO_SetOutputPin(U_MIN_GPIO_Port, U_MIN_Pin);
    DWT_Delay_ms(125);
    LL_GPIO_SetOutputPin(V_MIN_GPIO_Port, V_MIN_Pin);
    DWT_Delay_ms(125);
  }
  return 0;
}

static FLASH_EraseInitTypeDef EraseInitStruct;

static uint8_t erase_flash(uint32_t address, uint32_t size) {
  uint32_t fw_prog_end = address + size;
  int8_t start_sector = GetSector(address);
  int8_t end_sector = GetSector(fw_prog_end);
  uint32_t error;
  if (start_sector < 0 || end_sector < 0) {
    return 1;
  }
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = start_sector;
  EraseInitStruct.NbSectors = end_sector - start_sector + 1;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &error) != HAL_OK) {
    return 2;
  }
  return 0;
}

static uint8_t flash_buffer(uint32_t address, uint32_t size) {
  for (uint32_t idx = 0; idx < size; idx++) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, address, io_buffer[idx]) != HAL_OK) {
      return 1;
    }
    address += 4;
  }
  return 0;
}

static uint8_t verify_buffer(uint32_t address, uint32_t size) {
  for (uint32_t idx = 0; idx < size; idx++) {
    if (*(uint32_t*)address != io_buffer[idx]) {
      return 1;
    }
    address += 4;
  }
  return 0;
}

static void Jump_to_app() {
  uint32_t* fw_start = ((uint32_t*)FLASH_USER_START_ADDR);
  DWT_Delay_DeInit();
  HAL_SD_DeInit(&hsd);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_RCC_DeInit();
  //__disable_irq();

  NVIC->ICER[ 0 ] = 0xFFFFFFFF ;
  NVIC->ICER[ 1 ] = 0xFFFFFFFF ;
  NVIC->ICER[ 2 ] = 0xFFFFFFFF ;

  NVIC->ICPR[ 0 ] = 0xFFFFFFFF ;
  NVIC->ICPR[ 1 ] = 0xFFFFFFFF ;
  NVIC->ICPR[ 2 ] = 0xFFFFFFFF ;

  SysTick->CTRL = 0;
  SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk ;
  SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk |
                   SCB_SHCSR_BUSFAULTENA_Msk |
                   SCB_SHCSR_MEMFAULTENA_Msk );
  uint32_t sp = fw_start[0];
  uint32_t pc = fw_start[1];
  SCB->VTOR = FLASH_USER_START_ADDR;
  __asm__ __volatile__(
      "msr msp, %0\n\t"
      "msr psp, %0\n\t"
      "bx %1\n\t"
      : /* no output */
      : "r" (sp), "r" (pc)
      : /* no clobber list */);
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
}
#endif /* USE_FULL_ASSERT */
