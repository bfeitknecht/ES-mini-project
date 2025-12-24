/**
 * @file machine_health.c
 * @brief Machine Health Indicator implementation.
 */
#include "machine_health.h"
#include "arm_math.h"
#include "kiss_fftr.h"
#include "main.h"
#include "power_manager.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#define INPUT_SIZE 2048 // ~125ms of data at 16.45kHz
#define FS         16447
#define FFT_SIZE   2048 // Power of 2 for CMSIS-DSP

// Anomaly Detection Strategy: Band Energy Ratio
#define NORMAL_BAND_START_HZ  0
#define NORMAL_BAND_END_HZ    500
#define PROBLEM_BAND_START_HZ 1000
#define PROBLEM_BAND_END_HZ   8000
#define ANOMALY_RATIO_THRESHOLD 0.5f

/* Private variables ---------------------------------------------------------*/
/** @brief Buffer for raw microphone samples. Static for DMA compatibility. */
static int32_t mic_buffer[INPUT_SIZE];

/** @brief KISS FFT configuration state. */
static kiss_fftr_cfg kiss_cfg = NULL;

/** @brief System clock frequency in Hz. */
static uint32_t cpu_freq;

extern RTC_HandleTypeDef hrtc;

/**
 * @brief Gets the current time from the RTC in milliseconds.
 * @return Time in ms since some epoch (e.g. midnight).
 */
static uint32_t get_rtc_ms(void) {
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    
    uint32_t ms = (sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds) * 1000;
    // Subseconds are counting down from SynchPrediv to 0
    uint32_t sub_ms = ((hrtc.Init.SynchPrediv - sTime.SubSeconds) * 1000) / (hrtc.Init.SynchPrediv + 1);
    return ms + sub_ms;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Performs Real FFT using the KISS FFT library.
 * 
 * @param in_buffer Input time-domain samples (float).
 * @param fft_buffer Output buffer for magnitude spectrum.
 * @param size FFT size.
 */
static void decompose_kiss(float *in_buffer, float *fft_buffer, uint32_t size) {
    static kiss_fft_cpx out_cpx[FFT_SIZE / 2 + 1];
    kiss_fftr(kiss_cfg, in_buffer, out_cpx);

    for (uint32_t i = 0; i < size / 2; i++) {
        fft_buffer[i] = sqrtf(out_cpx[i].r * out_cpx[i].r + out_cpx[i].i * out_cpx[i].i);
    }
    fft_buffer[0] = 0; // Remove DC component
}

/**
 * @brief Performs Real FFT using the CMSIS-DSP library.
 * 
 * @param in_buffer Input time-domain samples (float).
 * @param fft_buffer Output buffer for magnitude spectrum.
 * @param size FFT size.
 */
static void decompose_cmsis(float *in_buffer, float *fft_buffer, uint32_t size) {
    static arm_rfft_fast_instance_f32 S;
    static bool cmsis_initialized = 0;
    if (!cmsis_initialized) {
        arm_rfft_fast_init_f32(&S, size);
        cmsis_initialized = 1;
    }

    arm_rfft_fast_f32(&S, in_buffer, fft_buffer, 0);
    arm_cmplx_mag_f32(fft_buffer, fft_buffer, size / 2);
    
    fft_buffer[0] = 0; // Remove DC component
}

/**
 * @brief Calculates the total energy in a specific frequency band.
 * @param fft_buffer Pointer to the frequency spectrum (magnitudes).
 * @param start_hz Start frequency of the band in Hz.
 * @param end_hz End frequency of the band in Hz.
 * @param size FFT size.
 * @retval Total energy in the band.
 */
static float calculate_band_energy(float *fft_buffer, uint32_t start_hz, uint32_t end_hz, uint32_t size) {
    uint32_t start_bin = (start_hz * size) / FS;
    uint32_t end_bin = (end_hz * size) / FS;

    if (end_bin >= size / 2) {
        end_bin = (size / 2) - 1;
    }

    float energy = 0.0f;
    for (uint32_t i = start_bin; i <= end_bin; i++) {
        // Energy is the sum of squares of magnitudes
        energy += fft_buffer[i] * fft_buffer[i];
    }
    return energy;
}

int toggle_algorithm = 0; // 0: CMSIS, 1: KISS

/**
 * @brief Compares KISS FFT and CMSIS-DSP FFT performance and detects anomalies.
 * 
 * @param in_buffer Input time-domain samples (float).
 * @param fft_buffer Output buffer for magnitude spectrum.
 * @param size FFT size.
 * @return true if energy ratio exceeds threshold, false otherwise.
 */
static bool detect_anomaly(float *in_buffer, float *fft_buffer, uint32_t size) {
    uint32_t start, stop;
    float max_val;
    uint32_t max_idx;
    float res = (float)FS / size;

    const char* algo_name = toggle_algorithm ? "KISS FFT" : "CMSIS FFT";
    if (toggle_algorithm) {
        start = DWT->CYCCNT;
        decompose_kiss(in_buffer, fft_buffer, size);
        stop = DWT->CYCCNT;
    } else {
        start = DWT->CYCCNT;
        decompose_cmsis(in_buffer, fft_buffer, size);
        stop = DWT->CYCCNT;
    }

    arm_max_f32(fft_buffer, size / 2, &max_val, &max_idx);

    printf("INFO: FFT Statistics:\r\n");
    printf("  Algorithm:\t\t%s\r\n", algo_name);
    printf("  Cycles:\t\t%" PRIu32 "\r\n", stop - start);
    printf("  Max Magnitude:\t%.2f\r\n", max_val);
    printf("  Peak Frequency:\t%.1f Hz\r\n", max_idx * res);

    // Anomaly Detection Logic: Energy Ratio
    float normal_energy = calculate_band_energy(fft_buffer, NORMAL_BAND_START_HZ, NORMAL_BAND_END_HZ, size);
    float problem_energy = calculate_band_energy(fft_buffer, PROBLEM_BAND_START_HZ, PROBLEM_BAND_END_HZ, size);
    
    // Protect against inf/nan
    if (isinf(normal_energy) || isnan(normal_energy)) normal_energy = 1e30f;
    if (isinf(problem_energy) || isnan(problem_energy)) problem_energy = 1e30f;

    float ratio = problem_energy / (normal_energy + 1e-6f);
    
    printf("INFO: Detection Metrics:\r\n");
    printf("  Normal Band Energy (0-500Hz):\t\t%.2e\r\n", normal_energy);
    printf("  Problem Band Energy (1k-8kHz):\t%.2e\r\n", problem_energy);
    printf("  Energy Ratio (P/N):\t\t\t%.4f\r\n", ratio);

    return (ratio > ANOMALY_RATIO_THRESHOLD);
}

void reset_led_matrix(void) {
  HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_RESET);
  uint8_t buf[] = {
      0x00,
      0x00,
      0x00,
  };
  HAL_SPI_Transmit(&hspi2, buf, 3, 300);
  HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_RESET);
}

void signal_normal(void) {
    reset_led_matrix();
    uint8_t buf[] = {
        0x00,
        0x0f,
        0xff,
    };

    HAL_SPI_Transmit(&hspi2, buf, 3, 300);
    HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    reset_led_matrix();
}

void signal_error(void) {
    reset_led_matrix();

    uint8_t buf[] = {
        0xff,
        0xf0,
        0x00,
    };

    HAL_SPI_Transmit(&hspi2, buf, 3, 300);  
    HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MATRIX_RCK_GPIO_Port, MATRIX_RCK_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Computes and prints estimated energy and power usage.
 * 
 * @param active_cycles Number of CPU cycles spent in active mode.
 * @param avg_active_cycles Average number of CPU cycles spent in active mode.
 * @param sleep_time_ms Time spent in sleep mode in milliseconds.
 */
static void print_power_metrics(uint32_t active_cycles, uint32_t avg_active_cycles, uint32_t sleep_time_ms) {
    const float vdd = 3.3f;        // Volts
    const float i_run = 8.4e-3f;   // Amps (8.4 mA)
    const float i_sleep = 2.3e-3f; // Amps (2.3 mA)

    float t_active = (float)active_cycles / (float)cpu_freq;
    float t_avg_active = (float)avg_active_cycles / (float)cpu_freq;
    float t_sleep = (float)sleep_time_ms / 1000.0f;
    float t_total = t_avg_active + t_sleep;

    float e_active = vdd * i_run * t_avg_active;
    float e_sleep = vdd * i_sleep * t_sleep;
    float e_total = e_active + e_sleep;

    float p_avg = e_total / t_total;

    printf("INFO: Power Metrics\r\n");
    printf("  Active Cycles:\t%10" PRIu32 "\r\n", active_cycles);
    printf("  Avg Active Cycles:\t%10" PRIu32 "\r\n", avg_active_cycles);
    printf("  Active Time:\t\t%10.3f ms\r\n", t_active * 1000.0f);
    printf("  Avg Active Time:\t%10.3f ms\r\n", t_avg_active * 1000.0f);
    printf("  Sleep Time:\t\t%10.3f ms\r\n", (float)sleep_time_ms);
    printf("  Energy/Cycle:\t\t%10.3f mJ\r\n", e_total * 1000.0f);
    printf("  Avg Power:\t\t%10.3f mW\r\n", p_avg * 1000.0f);
}

/**
 * @brief Main task for Machine Health Detection.
 */
void machine_health_task(void) {
    static float in_buffer[FFT_SIZE];
    static float fft_buffer[FFT_SIZE];
    static uint64_t total_active_cycles_kiss = 0;
    static uint32_t iteration_count_kiss = 0;
    static uint64_t total_active_cycles_cmsis = 0;
    static uint32_t iteration_count_cmsis = 0;
    static uint32_t last_sleep_ms = 5000;

    printf("\r\n");
    printf("Machine Health Detection Started\r\n");
    printf("Sample Rate: %d Hz, Window: %d samples (~%d ms)\r\n", FS, INPUT_SIZE, (INPUT_SIZE * 1000) / FS);
    
    cpu_freq = HAL_RCC_GetSysClockFreq();
    printf("System Clock Frequency: %" PRIu32 " Hz\r\n", cpu_freq);
    
    // Enable the DWT cycle counter:
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Initialize KISS FFT
    if (kiss_cfg == NULL) {
        kiss_cfg = kiss_fftr_alloc(FFT_SIZE, 0, NULL, NULL);
    }

    while (1) {
        DWT->CYCCNT = 0;
        printf("--- ACTIVE MODE ---\r\n");
        
        // Acquire microphone data via DFSDM + DMA
        mic_dma_finished = 0;
        if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, mic_buffer, INPUT_SIZE) != HAL_OK) {
            printf("ERROR: Failed to start DFSDM!\r\n");
            Error_Handler();
        }
        while (!mic_dma_finished) {}
        if (HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0) != HAL_OK) {
            printf("ERROR: Failed to stop DFSDM!\r\n");
            Error_Handler();
        }
        
        // Pre-process: Convert to float and normalize to [-1, 1]
        // DFSDM gain with Sinc4, 38x oversampling, 4x int oversampling is ~8.3e6
        const float normalization_factor = 1.0f / 8388608.0f; 
        memset(in_buffer, 0, sizeof(in_buffer));
        for (int i = 0; i < INPUT_SIZE; i++) {
            in_buffer[i] = (float)mic_buffer[i] * normalization_factor;
        }
        
        // Detect Anomaly
        toggle_algorithm = !toggle_algorithm;
        bool anomaly = detect_anomaly(in_buffer, fft_buffer, FFT_SIZE);
        if (anomaly) {
            printf("WARNING: Anomaly detected!\r\n");
            signal_error();
        } else {
            signal_normal();
        }
        
        uint32_t active_cycles = DWT->CYCCNT;
        uint32_t avg_active_cycles;

        if (toggle_algorithm) { // KISS FFT
            total_active_cycles_kiss += active_cycles;
            iteration_count_kiss++;
            avg_active_cycles = (uint32_t)(total_active_cycles_kiss / iteration_count_kiss);
        } else { // CMSIS FFT
            total_active_cycles_cmsis += active_cycles;
            iteration_count_cmsis++;
            avg_active_cycles = (uint32_t)(total_active_cycles_cmsis / iteration_count_cmsis);
        }

        print_power_metrics(active_cycles, avg_active_cycles, last_sleep_ms);

        // Sleep for 5 seconds
        printf("--- SLEEP MODE ---\r\n");
        uint32_t t1 = get_rtc_ms();
        PM_EnterSleep(5);
        uint32_t t2 = get_rtc_ms();
        
        if (t2 >= t1) {
            last_sleep_ms = t2 - t1;
        } else {
            last_sleep_ms = (86400000 - t1) + t2;
        }
        printf("\r\n\r\n");
    }
}

