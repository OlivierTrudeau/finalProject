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
#include <math.h>
#define ARM_MATH_CM4
#include <stdio.h>
#include <stdbool.h>
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_STIMULUS_PORT0 (*(volatile uint32_t *)0xE0000000)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Define application states
typedef enum {
  STATE_IDLE,           // Waiting for shake to begin
  STATE_SHAKING,        // Currently detecting shaking
  STATE_TEMPERATURE,    // Reading temperature for 5 seconds
  STATE_VERIFY          // Verifying the readings
} AppState;

// Parameters for shake detection
#define SHAKE_THRESHOLD     500    // Minimum acceleration change to qualify as shake (reduced from 8000)
#define SHAKE_WINDOW_SIZE   10      // Number of samples to keep for shake detection
#define MIN_SHAKE_COUNT     3       // Number of shakes needed to qualify as "true shake" (reduced from 5)
#define TEMP_MEASURE_TIME   5000    // Temperature measurement time in ms

// Variables for shake detection
int16_t prev_acc[3] = {0, 0, 0};         // Previous accelerometer readings
uint32_t shake_time_start = 0;           // When shaking started
uint8_t shake_count = 0;                 // Counter for actual shakes
uint32_t temp_reading_start = 0;         // When temperature reading started
AppState current_state = STATE_IDLE;     // Current application state

// Variables for temperature verification
float temp_readings[5] = {0};            // Store multiple temperature readings
uint8_t temp_reading_index = 0;          // Current temperature reading index

// Kalman filter state for temperature readings
struct kalman_state temp_kalman = {
    .q = 0.01f,   // Process noise covariance
    .r = 0.5f,    // Measurement noise covariance
    .x = 25.0f,   // Initial estimate (room temperature)
    .p = 1.0f,    // Initial error estimate
    .k = 0.0f     // Initial Kalman gain
};

// For temperature change detection
float baseline_temp = 0.0f;  // Baseline temperature

// DAC audio variables
#define SINE_SAMPLES 32
uint16_t sine_wave[SINE_SAMPLES] = {
    2048, 2448, 2832, 3186, 3496, 3751, 3940, 4056,
    4095, 4056, 3940, 3751, 3496, 3186, 2832, 2448,
    2048, 1648, 1264, 910,  600,  345,  156,  40,
    0,    40,   156,  345,  600,  910,  1264, 1648
};

// Sound frequencies in Hz
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// Audio status flag
volatile uint8_t audio_playing = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_I2C2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void change_channel(int i);
bool detect_shake(int16_t* acc);
void blink_led(uint32_t times, uint32_t delay_ms);
void play_shake_detected_sound(void);
void play_countdown_sound(uint32_t seconds_remaining);
void play_success_sound(void);
void update_state_machine(float current_temp);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define V_REFINT ((uint16_t*)(uint32_t)0x1FFF75AA)
#define TS_CAL1 ((uint16_t*)(uint32_t)0x1FFF75A8)
#define TS_CAL2 ((uint16_t*)(uint32_t)0x1FFF75CA)
#define TS_CAL1_TEMP ((float)30.0)
#define TS_CAL2_TEMP ((float) 130.0)

// Function to detect shaking motion
bool detect_shake(int16_t* acc) {
  // Calculate absolute differences from previous reading
  int32_t delta_x = abs((int32_t)acc[0] - (int32_t)prev_acc[0]);
  int32_t delta_y = abs((int32_t)acc[1] - (int32_t)prev_acc[1]);
  int32_t delta_z = abs((int32_t)acc[2] - (int32_t)prev_acc[2]);

//  // Debug every 500ms (controlled by external timer)
//  static uint32_t last_debug = 0;
//  uint32_t now = HAL_GetTick();
//  if (now - last_debug > 500) {
//    printf("Delta values: x=%ld, y=%ld, z=%ld (threshold: %d)\n",
//           delta_x, delta_y, delta_z, SHAKE_THRESHOLD);
//    last_debug = now;
//  }

  // Save current values as previous for next time
  prev_acc[0] = acc[0];
  prev_acc[1] = acc[1];
  prev_acc[2] = acc[2];

  // Look for rapid movement on at least one axis (changed from two axes)
  int movement_axes = 0;
  if (delta_x > SHAKE_THRESHOLD) movement_axes++;
  if (delta_y > SHAKE_THRESHOLD) movement_axes++;
  if (delta_z > SHAKE_THRESHOLD) movement_axes++;

  return (movement_axes >= 1);  // True if significant movement on at least 1 axis (changed from 2)
}

// Function to blink LED for visual feedback
void blink_led(uint32_t times, uint32_t delay_ms) {
  for (uint32_t i = 0; i < times; i++) {
    HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_SET);
    HAL_Delay(delay_ms);
    HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_RESET);
    HAL_Delay(delay_ms);
  }
}

// Function to play a tone at a specific frequency for a duration
void play_tone(uint16_t frequency, uint32_t duration_ms) {
    // Stop any ongoing playback
    if (audio_playing) {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        audio_playing = 0;
        HAL_Delay(5); // Small delay to ensure DMA stops
    }

    printf("Playing tone: %d Hz for %lu ms\n", frequency, duration_ms);

    // Calculate timer period for the requested frequency
    uint32_t timer_period = (SystemCoreClock / (frequency * SINE_SAMPLES));

    // Configure Timer2 period
    htim2.Instance->ARR = timer_period - 1;
    htim2.Instance->PSC = 0; // No prescaler

    // Start the timer
    HAL_TIM_Base_Start(&htim2);

    // Start DAC with DMA in circular mode to output the sine wave
    audio_playing = 1;
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_wave, SINE_SAMPLES, DAC_ALIGN_12B_R);

    // Optional: if you need precise duration, use a timer or non-blocking delay
    HAL_Delay(duration_ms);

    // Stop the DAC and timer
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);
    audio_playing = 0;
}

// Add a function to forcibly stop audio
void stop_audio() {
    if (audio_playing) {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim2);
        audio_playing = 0;
    }
}

// This function is called for audio feedback when a shake is detected
void play_shake_detected_sound(void) {
    stop_audio();
    // Rising tone pattern to indicate shake detection
    play_tone(NOTE_C4, 100);
    play_tone(NOTE_E4, 100);
    play_tone(NOTE_G4, 100);
    play_tone(NOTE_C5, 200);

    // Still blink LED for visual feedback
    blink_led(1, 50);
    stop_audio();

}

// This function is called for audio countdown during temperature reading
void play_countdown_sound(uint32_t seconds_remaining) {
    stop_audio();
    // Different tones based on seconds remaining
    switch(seconds_remaining) {
        case 5:
            play_tone(NOTE_C4, 100);
            break;
        case 4:
            play_tone(NOTE_D4, 100);
            break;
        case 3:
            play_tone(NOTE_E4, 100);
            break;
        case 2:
            play_tone(NOTE_G4, 100);
            break;
        case 1:
            play_tone(NOTE_A4, 100);
            break;
        default:
            play_tone(NOTE_C4, 50);
            break;
    }

    // Still blink LED for visual feedback
    blink_led(1, 50);
    stop_audio();
}

// This function is called for temperature reading success
void play_success_sound(void) {
    stop_audio();
    // Success melody
    play_tone(NOTE_C4, 100);
    play_tone(NOTE_E4, 100);
    play_tone(NOTE_G4, 100);
    play_tone(NOTE_C5, 200);
    play_tone(NOTE_G4, 100);
    play_tone(NOTE_C5, 300);

    // Still blink LED for visual feedback
    blink_led(1, 50);
    stop_audio();
}

// Function to process state machine
void update_state_machine(float current_temp) {
  uint32_t current_time = HAL_GetTick();

  switch (current_state) {
    case STATE_IDLE:
      // In idle state, waiting for shake to be detected
      HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_RESET);
      stop_audio();
      shake_count = 0;
      break;

    case STATE_SHAKING:
      // Detecting continuous shaking
      HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_SET);

      // If user has been shaking for too long without qualifying, reset
      if (current_time - shake_time_start > 3000 && shake_count < MIN_SHAKE_COUNT) {
        current_state = STATE_IDLE;
        shake_count = 0;
        printf("Shake timeout - please try again\n");
        blink_led(2, 200); // Error indication
      }
      // If enough shakes detected, move to temperature reading
      else if (shake_count >= MIN_SHAKE_COUNT) {
        current_state = STATE_TEMPERATURE;
        temp_reading_start = current_time;
        temp_reading_index = 0;
        printf("Shake detected! Please place finger on temperature sensor\n");
        stop_audio();
        play_shake_detected_sound(); // Audio feedback
      }
      break;

    case STATE_TEMPERATURE:
        // Reading temperature for 5 seconds
        HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_SET);

        uint32_t elapsed = current_time - temp_reading_start;
        uint32_t seconds_remaining = (TEMP_MEASURE_TIME - elapsed) / 1000;

        // Capture baseline temperature right at the start
        if (temp_reading_index == 0) {
            baseline_temp = current_temp;
            printf("Baseline temperature: %.2f C\n", baseline_temp);
        }

        if (seconds_remaining < 5 && seconds_remaining >= 0) {
            // Every second, update the countdown
            static uint32_t last_second = 0;
            if (last_second != seconds_remaining) {
                last_second = seconds_remaining;
                printf("Temperature reading: %d seconds remaining\n", (int)seconds_remaining + 1);
                play_countdown_sound(seconds_remaining + 1);

                // Store temperature reading
                if (temp_reading_index < 5) {
                    // Store the temperature value
                    temp_readings[temp_reading_index++] = current_temp;
                    printf("Current temperature: %.2f C (delta: %.2f C)\n",
                           current_temp, current_temp - baseline_temp);
                }
            }
        }

        // When time is up, move to verification
        if (elapsed >= TEMP_MEASURE_TIME) {
            current_state = STATE_VERIFY;
            printf("Temperature reading complete, verifying...\n");
        }
        break;

    case STATE_VERIFY:
        // Process the collected temperature data
        float max_temp = -100.0f;
        for (int i = 0; i < temp_reading_index; i++) {
            if (temp_readings[i] > max_temp) {
                max_temp = temp_readings[i];
            }
        }

        float temp_change = max_temp - baseline_temp;
        printf("Maximum temperature: %.2f C\n", max_temp);
        printf("Temperature change: %.2f C\n", temp_change);

        // Verify based on temperature change (warming from finger)
        if (temp_change >= 1.0f) {
            printf("Verification successful! Detected warming of %.2f C\n", temp_change);
            play_success_sound();
        } else {
            printf("Verification failed - insufficient temperature change\n");
            printf("Please place your finger near the board's temperature sensor\n");
            blink_led(2, 200); // Error indication
        }

        stop_audio();
        shake_count = 0;
        temp_reading_index = 0;
        // Return to idle state
        current_state = STATE_IDLE;
        HAL_GPIO_WritePin(GPIOB, greenLed_Pin, GPIO_PIN_RESET);
        break;
  }
}

int _write(int file, char *ptr, int len)
{
    // Send each character via ITM
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void change_channel(int i) {
	ADC_ChannelConfTypeDef Config = {0};

	if (i){
		Config.Channel = ADC_CHANNEL_TEMPSENSOR;
	}
	else{
		Config.Channel = ADC_CHANNEL_VREFINT;
	}

	Config.Rank = ADC_REGULAR_RANK_1,
	Config.SamplingTime = ADC_SAMPLETIME_640CYCLES_5,
	Config.SingleDiff = ADC_SINGLE_ENDED,
	Config.OffsetNumber = ADC_OFFSET_NONE,
	Config.Offset = 0;


	if (HAL_ADC_ConfigChannel(&hadc1, &Config) != HAL_OK){
		Error_Handler();
	}
}

int KalmanFilterC(float* InputArray, float* OutputArray, struct kalman_state* kstate, int length) {
    for (int i = 0; i < length; i++) { // Iterate
        kstate->p = kstate->p + kstate->q;
        kstate->k = kstate->p/(kstate->p + kstate->r);
        kstate->x = kstate->x + (kstate->k)*(InputArray[i]-kstate->x);
        kstate->p = (1-kstate->k)*kstate->p;
        OutputArray[i] = kstate->x; // Store in output array
        int a = __get_FPSCR();
        if ((a & 268435456) != 0) { // Check for overflow (fixed parentheses)
            printf("Overflow.");
            while (1){}
        }
    }
    return 0; // Return 0 if successful or get stuck in while loop
}

// Improved temperature reading function with Kalman filtering
float read_temperature(void) {
    float ADC_value;
    float vref_plus;
    float V_temp;
    float temp_readings[5];
    float filtered_temp;

    // Take multiple readings for stability
    for (int i = 0; i < 5; i++) {
        // Measure the voltage ref+
        change_channel(0);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        ADC_value = (float)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        vref_plus = 3.0f * (float)(*V_REFINT)/ADC_value;

        // Small delay between readings
        HAL_Delay(5);

        // Measure temperature
        change_channel(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        V_temp = (float)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        // Calculate temperature
        temp_readings[i] = (((TS_CAL2_TEMP - TS_CAL1_TEMP)/((float)(*TS_CAL2) - (float)(*TS_CAL1))) *
                          ((V_temp * vref_plus/3.0f)-(float)(*TS_CAL1))) + TS_CAL1_TEMP;

        // Check for obviously wrong readings
        if (temp_readings[i] < -10.0f || temp_readings[i] > 100.0f) {
            temp_readings[i] = (i > 0) ? temp_readings[i-1] : 25.0f; // Use previous or default
        }
    }

    // Apply Kalman filter to the readings
    float filtered_readings[5];
    KalmanFilterC(temp_readings, filtered_readings, &temp_kalman, 5);

    // Return the last filtered value (most current)
    return filtered_readings[4];
}

// DAC DMA error callback
void HAL_DAC_DMAErrorCallback(DAC_HandleTypeDef *hdac) {
    // Handle DMA errors
    audio_playing = 0;
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);

    // Signal error with LED
    blink_led(5, 50);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_OCTOSPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_I2C2_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //BSP_TSENSOR_Init();
  //BSP_HSENSOR_Init(); //HTS221
  //BSP_MAGNETO_Init(); //LIS3MDL
  // Initialize the accelerometer with default parameters (±2g range)
  BSP_ACCELERO_Init(); //LSM6DSL

  current_state = STATE_IDLE;  // Explicitly set initial state
  shake_count = 0;             // Reset shake count
  temp_reading_index = 0;      // Reset temperature reading index

  // Initialize TIM2 for DAC triggering
  HAL_TIM_Base_Stop(&htim2);

  // Set accelerometer to full scale - this may be required depending on your board
  // The below is a placeholder - you might need to modify based on your specific board API
  uint8_t ctrl = 0;
  // Read current settings, modify only the scale bits, then write back
  // This is just an example - you need to check your board's specific implementation
  // BSP_ACCELERO_Set_FS(LSM6DSL_ACC_FULLSCALE_16G); // Set to ±16g for better sensitivity
  //BSP_GYRO_Init(); //LSM6DSL
  //BSP_PSENSOR_Init(); //LPS22HB

  float ADC_value;
  float vref_plus;
  float V_temp;
  float temp = 0;
  int16_t acc[3] = {0};
  uint32_t lastPrintTime = 0;

  printf("\n\n--- Multi-factor Biometric Verification System ---\n");
  printf("Shake the device firmly to begin verification\n");

  // Blink LED to indicate system is ready
  blink_led(2, 250);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t currentTime = HAL_GetTick();

    // Read accelerometer data
    BSP_ACCELERO_AccGetXYZ(acc);

    // Process accelerometer data for shake detection
    if (current_state == STATE_IDLE) {
      // In idle state, look for the start of shaking
      if (detect_shake(acc)) {
        // First shake detected, transition to shaking state
        current_state = STATE_SHAKING;
        shake_time_start = currentTime;
        shake_count = 1;
        printf("Possible shake detected, continue shaking...\n");
      }
    }
    else if (current_state == STATE_SHAKING) {
      // Already in shaking state, count additional shakes
      if (detect_shake(acc)) {
        shake_count++;
        printf("Shake count: %d/%d\n", shake_count, MIN_SHAKE_COUNT);
      }
    }

    // Always read temperature when in temperature reading state or for verification
    // Always read temperature when in temperature reading state or for verification
    if (current_state == STATE_TEMPERATURE || current_state == STATE_VERIFY) {
        temp = read_temperature();
    }

    // For debugging in idle mode, print sensor values occasionally
    if ((current_state == STATE_IDLE || current_state == STATE_SHAKING) &&
        currentTime - lastPrintTime > 1000) {

        // Read temperature with Kalman filtering for display
        temp = read_temperature();

        printf("Status: %s | Accelerometer: x=%d, y=%d, z=%d | Temp: %.2f C\n",
               current_state == STATE_IDLE ? "IDLE" : "SHAKING",
               acc[0], acc[1], acc[2], temp);

        lastPrintTime = currentTime;
    }

    // Update the state machine with current temperature
    update_state_machine(temp);

    // Small delay to prevent CPU hogging
    HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 2;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|ARD_D4_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|greenLed_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST25DV04K_RF_DISABLE_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin ST25DV04K_GPO_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin
                           ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|ST25DV04K_GPO_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin
                          |ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : blue_button_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = blue_button_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin ARD_D4_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|ARD_D4_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin greenLed_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|greenLed_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin USB_OTG_FS_PWR_EN_Pin ARD_D2_Pin
                           HTS221_DRDY_EXTI15_Pin PMOD_IRQ_EXTI2_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin|LSM6DSL_INT1_EXTI11_Pin|USB_OTG_FS_PWR_EN_Pin|ARD_D2_Pin
                          |HTS221_DRDY_EXTI15_Pin|PMOD_IRQ_EXTI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin PMOD_SPI2_SCK_Pin STSAFE_A110_RESET_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
