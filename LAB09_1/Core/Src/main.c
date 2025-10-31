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

#include "stdio.h"
#include "stm32f3xx_hal.h"
#include "string.h"
#include "math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//task 1 defs


#define WHO_AM_I_REG     0x0F

uint8_t who_am_i = 0;
char msg[50];
#define LSM303AGR_ADDR   (0x19 << 1)
#define LSM303AGR_ADDR_WRITE 0x32
#define LSM303AGR_ADDR_READ  0x33

#define CTRL_REG1      0x20
#define CTRL_REG1_VAL  0b00001111
#define WHO_AM_I_A      0x0F
#define CTRL_REG1_A     0x20
#define CTRL_REG4_A     0x23
#define OUT_X_L_A       0x28
#define RAD_TO_DEG      57.2958
#define I3G4250D_CS_PORT   GPIOE
#define I3G4250D_CS_PIN    GPIO_PIN_3
#define I3G4250D_WHO_AM_I  0x0F
#define I3G4250D_CTRL_REG1 0x20
#define I3G4250D_CTRL_REG4 0x23
#define I3G4250D_OUT_X_L   0x28
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

typedef struct {
    int16_t rawX, rawY, rawZ;
    float accX, accY, accZ;
    float offsetX, offsetY, offsetZ;
} LSM303_Data;

LSM303_Data lsmData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {

  float acc_rot;
  float gyro_rot;

} rot_angle_struct_t;

rot_angle_struct_t rot_angle_struct;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
int16_t gx, gy, gz;

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I3G4250D_Init(void)
{
    uint8_t data[2];

    // CTRL_REG1: normal mode, enable all axes
    data[0] = I3G4250D_CTRL_REG1; // register address
    data[1] = 0x0F;               // normal mode, all axes on
    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_SET);

    // CTRL_REG4: scale ±250 dps
    data[0] = I3G4250D_CTRL_REG4; 
    data[1] = 0x00;
    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_SET);
}



// Global offsets for gyro
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Call once at startup to calibrate gyro
void I3G4250D_Calibrate(void)
{
    const int samples = 200;  // More samples = better average
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t gx_raw, gy_raw, gz_raw;
    uint8_t buf[7];

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating gyro...\r\n", 21, HAL_MAX_DELAY);

    for (int i = 0; i < samples; i++)
    {
        buf[0] = 0x28 | 0xC0;  // OUT_X_L + auto-increment
        HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, buf, buf, 7, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_SET);

        gx_raw = (int16_t)(buf[2] << 8 | buf[1]);
        gy_raw = (int16_t)(buf[4] << 8 | buf[3]);
        gz_raw = (int16_t)(buf[6] << 8 | buf[5]);

        sumX += gx_raw;
        sumY += gy_raw;
        sumZ += gz_raw;

        HAL_Delay(5);
    }

    gx_offset = sumX / samples;
    gy_offset = sumY / samples;
    gz_offset = sumZ / samples;

    // Small deadband noise threshold (ignore ±2 raw counts)
    if (gx_offset > -2 && gx_offset < 2) gx_offset = 0;
    if (gy_offset > -2 && gy_offset < 2) gy_offset = 0;
    if (gz_offset > -2 && gz_offset < 2) gz_offset = 0;


}

// Read gyro axes with offsets applied
void read_gyro_axes(void)
{
    uint8_t buf[7];
    buf[0] = 0x28 | 0xC0;

    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, buf, buf, 7, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, I3G4250D_CS_PIN, GPIO_PIN_SET);

    int16_t gx_raw = (int16_t)(buf[2] << 8 | buf[1]) - gx_offset;
    int16_t gy_raw = (int16_t)(buf[4] << 8 | buf[3]) - gy_offset;
    int16_t gz_raw = (int16_t)(buf[6] << 8 | buf[5]) - gz_offset;

    float gx_dps = gx_raw * 0.00875f;
    float gy_dps = gy_raw * 0.00875f;
    float gz_dps = gz_raw * 0.00875f;
    // rot_angle_struct.gyro_rot = gy_dps;

 char uartBuf[80];
    snprintf(uartBuf, sizeof(uartBuf), "GYRO: %.2f, %.2f, %.2f\r\n", gx_dps, gy_dps, gz_dps);
    HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
}




void LSM303_Init(void)
{
    uint8_t data;

    // 1. Enable all axes (X, Y, Z), 100 Hz, normal mode
    data = 0x57;  // 0101 0111 = 100Hz, all axes enabled
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR,
                      CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, HAL_MAX_DELAY);

    // 2. Disable high-pass filters
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR,
                      0x21, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, HAL_MAX_DELAY);

    // 3. Enable block data update, ±2g full-scale, little endian
    data = 0x80;  // BDU=1
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR,
                      CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, HAL_MAX_DELAY);

    // 4. Enable auto-increment for multiple reads (CTRL_REG6_A)
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR,
                      0x25, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, HAL_MAX_DELAY);

    HAL_Delay(200);
}


// Read accelerometer
#define LSM303_SCALE_2G 0.001f  // 1 mg/LSB

void LSM303_ReadAccel(void)
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADDR, OUT_X_L_A | 0x80,
                     I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    // Combine bytes (little-endian: low byte first)
    lsmData.rawX = (int16_t)(data[1] << 8 | data[0]);
    lsmData.rawY = (int16_t)(data[3] << 8 | data[2]);
    lsmData.rawZ = (int16_t)(data[5] << 8 | data[4]);

    // Convert to m/s² (assuming ±2g range, 1 LSB = 0.000598 m/s²)
    lsmData.accX = lsmData.rawX * 0.000598 - lsmData.offsetX;
    lsmData.accY = lsmData.rawY * 0.000598 - lsmData.offsetY;
    lsmData.accZ = lsmData.rawZ * 0.000598 - lsmData.offsetZ;

    // Store one axis for rotation (example: Y-axis)
    rot_angle_struct.acc_rot = lsmData.accY;
}

void LSM303_Offset(void)
{
    float sumX = 0, sumY = 0;
    int samples = 20;

    for (int i = 0; i < samples; i++)
    {
        LSM303_ReadAccel();
        sumX += lsmData.accX;
        sumY += lsmData.accY;
        HAL_Delay(50);
    }

    lsmData.offsetX = sumX / samples;
    lsmData.offsetY = sumY / samples;
}

void LSM303_Print(void)
{
    char uartBuf[100];
    snprintf(uartBuf, sizeof(uartBuf),
             "ACC: %.2f, %.2f, %.2f\r\n",
             lsmData.accX,
             lsmData.accY,
             lsmData.accZ);
    HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
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
  void gyro_init(void);
  void read_gyro_axes(void);                         // task3
  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
  void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
  /* USER CODE BEGIN Init */
  I3G4250D_Init();
  HAL_Delay(100);
  I3G4250D_Calibrate();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();

  // now initialize gyro
  I3G4250D_Init();
  HAL_Delay(100);

  // then accelerometer
  LSM303_Init();
  HAL_Delay(100);
  LSM303_Offset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    /* USER CODE END WHILE */
    //task 1
  //     HAL_Delay(100); // Wait for sensor startup

  // if (HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADDR, WHO_AM_I_REG,
  //                      I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY) == HAL_OK)
  // {
  //     sprintf(msg, "LSM303AGR WHO_AM_I register = 0x%02X\r\n", who_am_i);
  // }
  // else
  // {
  //     sprintf(msg, "I2C read failed!\r\n");
  // }

  // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    /* USER CODE BEGIN 3 */
     
    while (1)
{
    LSM303_ReadAccel();
    LSM303_Print();
    read_gyro_axes();
    HAL_Delay(200);
}



  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
