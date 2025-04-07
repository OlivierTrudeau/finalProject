/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#define ARM_MATH_CM4
#include <stdio.h>
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_STIMULUS_PORT0 (*(volatile uint32_t *)0xE0000000)
typedef struct {
  float q;
  float r;
  float x;
  float p;
  float k;
} KalmanFilter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_REFINT ((uint16_t*)(uint32_t)0x1FFF75AA)
#define TS_CAL1 ((uint16_t*)(uint32_t)0x1FFF75A8)
#define TS_CAL2 ((uint16_t*)(uint32_t)0x1FFF75CA)
#define TS_CAL1_TEMP ((float)30.0)
#define TS_CAL2_TEMP ((float)130.0)
#define SHAKE_THRESHOLD 1000.0f
#define TEMP_THRESHOLD 30.0f
#define HOLD_TIME_MS 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
I2C_HandleTypeDef hi2c2;
OSPI_HandleTypeDef hospi1;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
KalmanFilter kf_x = {0.01f, 10.0f, 0.0f, 1.0f, 0.0f};
KalmanFilter kf_y = {0.01f, 10.0f, 0.0f, 1.0f, 0.0f};
KalmanFilter kf_z = {0.01f, 10.0f, 0.0f, 1.0f, 0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void change_channel(int i);
float kalman_filter(KalmanFilter *kf, float measurement);
int detect_shake_filtered(int16_t* acc);
void play_tone(uint32_t duration_ms);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_DFSDM1_Init();
  MX_OCTOSPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_I2C2_Init();

  BSP_ACCELERO_Init();

  float ADC_value;
  float vref_plus;
  float V_temp;
  float temp;

  int verified = 0;
  uint32_t temp_hold_start = 0;

  while (1)
  {
    int16_t acc[3];
    BSP_ACCELERO_AccGetXYZ(acc);
    printf("accelerometer -> x: %d, y: %d, z: %d\n", acc[0], acc[1], acc[2]);

    if (!verified && detect_shake_filtered(acc)) {
      printf("Shake detected. Starting temperature verification...\n");
      play_tone(100);
      temp_hold_start = HAL_GetTick();
      uint8_t holding = 1;

      while (holding && (HAL_GetTick() - temp_hold_start < HOLD_TIME_MS)) {
        change_channel(0);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        ADC_value = (float)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        vref_plus = 3.0f * (float)(*V_REFINT)/ADC_value;

        change_channel(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        V_temp = (float)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        temp = (((TS_CAL2_TEMP - TS_CAL1_TEMP)/((float)(*TS_CAL2) - (float)(*TS_CAL1))) * ((V_temp * vref_plus/3.0f)-(float)(*TS_CAL1))) + TS_CAL1_TEMP;
        printf("Temp value: %f C\n", temp);

        if (temp < TEMP_THRESHOLD) {
          holding = 0;
          printf("Finger removed or temp too low. Restart verification.\n");
        }
        HAL_Delay(500);
      }

      if (holding) {
        verified = 1;
        HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_SET);
        printf("Verification complete. Access granted.\n");
        play_tone(200);
      }
    }
    HAL_Delay(1000);
  }
}

float kalman_filter(KalmanFilter *kf, float measurement) {
  kf->p = kf->p + kf->q;
  kf->k = kf->p / (kf->p + kf->r);
  kf->x = kf->x + kf->k * (measurement - kf->x);
  kf->p = (1 - kf->k) * kf->p;
  return kf->x;
}

int detect_shake_filtered(int16_t* acc) {
  float x = kalman_filter(&kf_x, (float)acc[0]);
  float y = kalman_filter(&kf_y, (float)acc[1]);
  float z = kalman_filter(&kf_z, (float)acc[2]);
  float total = fabsf(x) + fabsf(y) + fabsf(z);
  return total > SHAKE_THRESHOLD;
}

void play_tone(uint32_t duration_ms)
{
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
  HAL_Delay(duration_ms);
  HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
}

static void MX_DAC1_Init(void)
{
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  DAC_ChannelConfTypeDef sConfig = {0};
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void change_channel (int i) {
  ADC_ChannelConfTypeDef Config = {0};
  Config.Channel = (i == 0) ? ADC_CHANNEL_VREFINT : ADC_CHANNEL_TEMPSENSOR;
  Config.Rank = ADC_REGULAR_RANK_1;
  Config.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  Config.SingleDiff = ADC_SINGLE_ENDED;
  Config.OffsetNumber = ADC_OFFSET_NONE;
  Config.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &Config) != HAL_OK){
    Error_Handler();
  }
}

int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
