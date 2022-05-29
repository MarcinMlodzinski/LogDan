/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "iwdg.h"
#include "lcd.h"
#include "quadspi.h"
#include "rtc.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "lfs.h"
#include "stm32l476g_discovery_qspi.h"
#include "lsm303c.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

volatile JOYState_TypeDef joy_state = JOY_NONE;
volatile FlagStatus KeyPressed = RESET;
// FlagStatus JoyInitialized = RESET;
// FlagStatus IddInitialized = RESET;
// FlagStatus LcdInitialized = RESET;
// FlagStatus LedInitialized = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int block_device_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int block_device_erase(const struct lfs_config *c, lfs_block_t block);
int block_device_sync(const struct lfs_config *c);

void initFS();
void readFile(lfs_t *lfs, lfs_file_t *file, const char *path, void *buffer, lfs_size_t size);
void writeFile(lfs_t *lfs, lfs_file_t *file, const char *path, const void *buffer, lfs_size_t size);
int16_t GetTemperature();
void initMagneto();
float convertRegDataToTemperature(int16_t value);
float getTemperatureCelsius();
void menu(int *menu_position, char *text, int *menu_select, int *i);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read = block_device_read,
    .prog = block_device_prog,
    .erase = block_device_erase,
    .sync = block_device_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = 4096,
    .block_count = 4096,
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_Clear();
  BSP_QSPI_Init();
  initFS();
  initMagneto();
  float int_part = 0;
  float fract_part = 0;
  uint32_t last_ms = HAL_GetTick();
  uint32_t last_ms_save = last_ms;
  uint32_t now = last_ms;
  uint32_t delay_500ms = 500;
  int i = 0;
  char text[60];
  char display[6];
  RTC_TimeTypeDef RtcTime;
  RTC_DateTypeDef RtcDate;

  int menu_position = 0;
  int menu_select = 0;

  char fs_ok[] = "FS OK";
  char fs_check[5];
  char record_file_name[10];
  int number_of_records = 0;

  readFile(&lfs, &file, "fs_check", &fs_check, sizeof(fs_check));

  if (fs_check == fs_ok)
  {
    readFile(&lfs, &file, "number_of_records", &number_of_records, sizeof(number_of_records));
  }
  else
  {
    writeFile(&lfs, &file, "number_of_records", &number_of_records, sizeof(number_of_records));
    writeFile(&lfs, &file, "fs_check", &fs_ok, sizeof(fs_ok));
  }

  int read_record_number = number_of_records;

  BSP_LCD_GLASS_DisplayString((uint8_t *)fs_check);
  sprintf(text, "Uzyj joysticka aby nawigowac      ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    //	  LCDBarDemo();
    //	  LCDCharDemo();
    //	  LCDStringDemo();

    if (KeyPressed)
    {
      menu(&menu_position, text, &menu_select, &i);
    }

    if (i <= strlen(text))
    {
      if (now - last_ms >= delay_500ms)
      {
        last_ms = now;

        switch (menu_select)
        {
        case 1:
          HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
          HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);
          fract_part = modff(getTemperatureCelsius(), &int_part);
          sprintf((char *)text, "Date %02d-%02d-20%02d Time %02d-%02d-%02d Temperature %d %01d      ",
                  RtcDate.Date, RtcDate.Month, RtcDate.Year, RtcTime.Hours, RtcTime.Minutes, RtcTime.Seconds, (int)int_part, (int)(fract_part * 10.0));
          break;
        case 2:

          break;
        case 3:

          break;
        case 4:

          break;
        default:
          switch (menu_position)
          {
          case 1:
            sprintf(text, "Wyswietl aktualny odczyt      ");
            break;
          case 2:
            sprintf(text, "Wyswietl historie      ");
            break;
          case 3:
            sprintf(text, "Ustaw date i godzine      ");
            break;
          case 4:
            sprintf(text, "Ustaw czas pomiedzy zapisami      ");
            break;
          }
          break;
        }

        for (int j = i - 6; j < i; j++)
        {
          if (j < 0)
          {
            display[j - i + 6] = text[strlen(text) + j];
          }
          else
          {
            display[j - i + 6] = text[j];
          }
        }
        i++;
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString((uint8_t *)display);
        BSP_LCD_GLASS_DisplayBar(LCD_BAR_3 >> (menu_position - 1));
      }

      //      if (now - last_ms_save >= 120 * delay_500ms)
      //      {
      //        last_ms_save = now;
      //        number_of_records++;
      //        sprintf(record_file_name, "record%d", number_of_records);
      //        writeFile(&lfs, &file, record_file_name, &text, sizeof(text));
      //      }
    }
    else
    {
      i = 0;
    }

    HAL_IWDG_Refresh(&hiwdg);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin & (DOWN_JOY_PIN | UP_JOY_PIN | SEL_JOY_PIN | RIGHT_JOY_PIN | LEFT_JOY_PIN)) != RESET)
  {
    KeyPressed = SET;

    switch (GPIO_Pin)
    {
    case DOWN_JOY_PIN:
      joy_state = JOY_DOWN;
      break;
    case UP_JOY_PIN:
      joy_state = JOY_UP;
      break;
    case SEL_JOY_PIN:
      joy_state = JOY_SEL;
      break;
    case RIGHT_JOY_PIN:
      joy_state = JOY_RIGHT;
      break;
    case LEFT_JOY_PIN:
      joy_state = JOY_LEFT;
      break;
    }
  }
}

int block_device_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  BSP_QSPI_Read((uint8_t *)buffer, (block * c->block_size + off), size);
  return 0;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
  BSP_QSPI_Write((uint8_t *)buffer, (block * c->block_size + off), size);
  return 0;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
  BSP_QSPI_Erase_Block(block * c->block_size);
  return 0;
}

int block_device_sync(const struct lfs_config *c)
{
  return 0;
}

void initFS()
{
  if (lfs_mount(&lfs, &cfg))
  {
    lfs_format(&lfs, &cfg);
    lfs_mount(&lfs, &cfg);
  }
}

void readFile(lfs_t *lfs, lfs_file_t *file, const char *path, void *buffer, lfs_size_t size)
{
  lfs_file_open(lfs, file, path, LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_read(lfs, file, buffer, size);
  lfs_file_close(lfs, file);
}

void writeFile(lfs_t *lfs, lfs_file_t *file, const char *path, const void *buffer, lfs_size_t size)
{
  lfs_file_open(lfs, file, path, LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_write(lfs, file, buffer, size);
  lfs_file_close(lfs, file);
}

int16_t GetTemperature()
{
  int8_t buffer[2];
  buffer[0] = MAGNETO_IO_Read(LSM303C_TEMP_OUT_L_M);
  buffer[1] = MAGNETO_IO_Read(LSM303C_TEMP_OUT_H_M);

  int16_t temp = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0]);
  return temp;
}

void initMagneto()
{
  MAGNETO_IO_Init();
  MAGNETO_IO_Write(LSM303C_CTRL_REG1_M, LSM303C_MAG_TEMPSENSOR_ENABLE | LSM303C_MAG_OM_XY_ULTRAHIGH | LSM303C_MAG_ODR_40_HZ);
  MAGNETO_IO_Write(LSM303C_CTRL_REG2_M, LSM303C_MAG_FS_16_GA | LSM303C_MAG_REBOOT_DEFAULT | LSM303C_MAG_SOFT_RESET_DEFAULT);
  MAGNETO_IO_Write(LSM303C_CTRL_REG3_M, 0x84);
  MAGNETO_IO_Write(LSM303C_CTRL_REG4_M, LSM303C_MAG_OM_Z_ULTRAHIGH | LSM303C_MAG_BLE_LSB);
  MAGNETO_IO_Write(LSM303C_CTRL_REG5_M, LSM303C_MAG_BDU_CONTINUOUS);
}

float convertRegDataToTemperature(int16_t value)
{
  int32_t temp = 0;
  float result;
  temp = 32768 + (int32_t)value;
  result = (((float)temp / 65535) * 120) - 40;
  return result;
}

float getTemperatureCelsius()
{
  float result = 0;
  result = convertRegDataToTemperature(GetTemperature());
  return result;
}

void menu(int *menu_position, char *text, int *menu_select, int *i)
{
  KeyPressed = RESET;
  int menu_entities = 4;

  switch (joy_state)
  {
  case JOY_UP:
    if ((*menu_select) == 0)
    {
      (*menu_position)--;

      if ((*menu_position) < 1)
      {
        (*menu_position) = menu_entities;
      }

      (*i) = 0;
    }
    break;

  case JOY_DOWN:
    if ((*menu_select) == 0)
    {
      (*menu_position)++;

      if ((*menu_position) > menu_entities)
      {
        (*menu_position) = 1;
      }

      (*i) = 0;
    }
    break;

  case JOY_LEFT:
    (*menu_select) = 0;
    (*i) = 0;
    break;

  case JOY_SEL:
    (*menu_select) = (*menu_position);
    (*i) = 0;
    break;

  default:
    break;
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
