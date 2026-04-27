/* ==========================================================
 * File: main.c
 * Project: MRAS Controller for LLC Resonant DC-DC Converter
 * Target: STM32F446RE (Nucleo-F446RE)
 * Clock: 180 MHz
 * Control Sample Time: 1 microsecond (1MHz timer interrupt)
 * Author: MSc Thesis
 * ==========================================================
 *
 * PIN MAPPING:
 * -------------------------------------------------------
 * PA0  - ADC1_IN0  - Vo  (Output Voltage measurement)
 * PA1  - ADC1_IN1  - io  (Output Current measurement)
 * PA4  - DAC_OUT1  - u_control (to VCO frequency input)
 * PB0  - ADC1_IN8  - Vref potentiometer (optional)
 * PC13 - LED       - Heartbeat indicator
 * -------------------------------------------------------
 */

#include "main.h"
#include "MRAS_Controller2.h"
#include <math.h>

/* =========================================================
 * HARDWARE HANDLES
 * ========================================================= */
ADC_HandleTypeDef  hadc1;
DAC_HandleTypeDef  hdac;
TIM_HandleTypeDef  htim2;   /* 1us control interrupt timer */
UART_HandleTypeDef huart2;  /* For debug output to PC */

/* =========================================================
 * SCALING CONSTANTS
 * Adjust these to match your hardware voltage dividers
 * and current sensor ratings
 * ========================================================= */
#define ADC_MAX_COUNT     4095.0f   /* 12-bit ADC */
#define VREF_ADC          3.3f      /* ADC reference voltage */

/* Voltage sensing: assuming divider scales 0-5000V to 0-3.3V */
#define VO_SCALE          (5000.0f / 3.3f)

/* Current sensing: assuming 0-3.3V = 0-30A */
#define IO_SCALE          (30.0f / 3.3f)

/* DAC output scaling: u_control maps to 0-3.3V DAC */
/* VCO input: 0-3.3V maps to 50kHz-68kHz (centre 59kHz) */
#define U_CONTROL_MIN     -1000.0f  /* min expected u_control */
#define U_CONTROL_MAX      1000.0f  /* max expected u_control */
#define DAC_MAX_COUNT      4095.0f

/* Reference voltage setpoint (Volts) */
#define VREF_SETPOINT      4000.0f  /* 4000V output target */

/* =========================================================
 * GLOBAL FLAGS AND BUFFERS
 * ========================================================= */
volatile uint8_t  control_flag  = 0;  /* Set by timer ISR */
volatile uint32_t adc_vo_raw    = 0;
volatile uint32_t adc_io_raw    = 0;
uint32_t          heartbeat_cnt = 0;

/* Logged data for thesis verification (optional) */
#define LOG_SIZE 1000
float log_Vo[LOG_SIZE];
float log_u[LOG_SIZE];
uint32_t log_index = 0;

/* =========================================================
 * FUNCTION PROTOTYPES
 * ========================================================= */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_DAC_Init(void);
void MX_TIM2_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);

float    read_Vo(void);
float    read_io(void);
void     set_VCO(float u_control);
void     debug_print(float Vo, float io, float u);

/* =========================================================
 * MAIN FUNCTION
 * ========================================================= */
int main(void)
{
    /* --- 1. HAL and system init --- */
    HAL_Init();
    SystemClock_Config();   /* 180 MHz */

    /* --- 2. Peripheral init --- */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    /* --- 3. Start DAC --- */
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    /* --- 4. Start ADC --- */
    HAL_ADC_Start(&hadc1);

    /* --- 5. Initialise MRAS controller --- */
    MRAS_Controller2_initialize();

    /* --- 6. Start 1us control timer --- */
    HAL_TIM_Base_Start_IT(&htim2);

    /* --- 7. Debug message --- */
    char msg[] = "MRAS Controller Started\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);

    /* =========================================================
     * MAIN LOOP
     * Control runs in timer ISR every 1us
     * Main loop handles slow tasks: logging, UART debug
     * ========================================================= */
    while (1)
    {
        /* Heartbeat LED every 500ms */
        heartbeat_cnt++;
        if (heartbeat_cnt >= 500000)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            heartbeat_cnt = 0;

            /* Send debug data over UART every 500ms */
            debug_print(
                (float)MRAS_Controller2_U.Vo,
                (float)MRAS_Controller2_U.io,
                (float)MRAS_Controller2_Y.u_control
            );
        }
    }
}

/* =========================================================
 * TIMER INTERRUPT - runs every 1 MICROSECOND
 * This is the core control loop
 * ========================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* --- Read sensors --- */
        float Vo = read_Vo();
        float io = read_io();

        /* --- Feed inputs to MRAS controller --- */
        MRAS_Controller2_U.Vo     = (double)Vo;
        MRAS_Controller2_U.io     = (double)io;
        MRAS_Controller2_U.Vref   = (double)VREF_SETPOINT;
        MRAS_Controller2_U.Verror = (double)(VREF_SETPOINT - Vo);
        MRAS_Controller2_U.Ierror = (double)(0.0f - io); /* io_ref=0 or set your target */

        /* --- Run one MRAS control step --- */
        MRAS_Controller2_step();

        /* --- Send control output to VCO --- */
        set_VCO((float)MRAS_Controller2_Y.u_control);

        /* --- Optional: log data for thesis verification --- */
        if (log_index < LOG_SIZE)
        {
            log_Vo[log_index] = Vo;
            log_u[log_index]  = (float)MRAS_Controller2_Y.u_control;
            log_index++;
        }
    }
}

/* =========================================================
 * READ OUTPUT VOLTAGE
 * Reads ADC channel 0 (PA0), scales to real voltage
 * ========================================================= */
float read_Vo(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    return ((float)raw / ADC_MAX_COUNT) * VREF_ADC * VO_SCALE;
}

/* =========================================================
 * READ OUTPUT CURRENT
 * Reads ADC channel 1 (PA1), scales to real current
 * ========================================================= */
float read_io(void)
{
    /* Switch ADC to channel 1 */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    return ((float)raw / ADC_MAX_COUNT) * VREF_ADC * IO_SCALE;
}

/* =========================================================
 * SET VCO FREQUENCY via DAC output
 * Maps u_control to 0-3.3V DAC output
 * VCO converts voltage to frequency around 59kHz centre
 * ========================================================= */
void set_VCO(float u_control)
{
    /* Clamp u_control to expected range */
    if (u_control > U_CONTROL_MAX) u_control = U_CONTROL_MAX;
    if (u_control < U_CONTROL_MIN) u_control = U_CONTROL_MIN;

    /* Scale to DAC counts 0-4095 */
    float normalised = (u_control - U_CONTROL_MIN) /
                       (U_CONTROL_MAX - U_CONTROL_MIN);
    uint32_t dac_val = (uint32_t)(normalised * DAC_MAX_COUNT);

    /* Write to DAC */
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
                     DAC_ALIGN_12B_R, dac_val);
}

/* =========================================================
 * DEBUG UART PRINT
 * Sends Vo, io, u_control over UART2 to PC terminal
 * ========================================================= */
void debug_print(float Vo, float io, float u)
{
    char buf[80];
    int len = snprintf(buf, sizeof(buf),
        "Vo=%.1f V  io=%.3f A  u=%.4f\r\n", Vo, io, u);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

/* =========================================================
 * SYSTEM CLOCK CONFIG - 180 MHz from 8MHz HSE
 * ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* =========================================================
 * GPIO INIT
 * ========================================================= */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC13 - LED heartbeat output */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* =========================================================
 * ADC1 INIT - PA0=Vo, PA1=io
 * ========================================================= */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    /* Default channel: PA0 = Vo */
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* =========================================================
 * DAC INIT - PA4 = u_control output to VCO
 * ========================================================= */
void MX_DAC_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_DAC_CLK_ENABLE();

    hdac.Instance = DAC;
    HAL_DAC_Init(&hdac);

    sConfig.DAC_Trigger      = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
}

/* =========================================================
 * TIMER 2 INIT - 1us interrupt
 * APB1 clock = 90MHz, Timer clock = 180MHz
 * Prescaler = 179, Period = 0 => 1us tick
 * ========================================================= */
void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 179;   /* 180MHz / (179+1) = 1MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0;     /* Interrupt every count = 1us */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    /* Enable timer interrupt in NVIC */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); /* Highest priority */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* =========================================================
 * UART2 INIT - 115200 baud for debug
 * ========================================================= */
void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart2);
}

/* =========================================================
 * TIMER 2 IRQ HANDLER
 * ========================================================= */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/* =========================================================
 * ERROR HANDLER
 * ========================================================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* Flash LED rapidly to indicate error */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}