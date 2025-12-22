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
#define MAGNITUDE_THRESHOLD         20000000.0f

/* Private variables ---------------------------------------------------------*/
/** @brief Buffer for raw microphone samples. Static for DMA compatibility. */
static int32_t mic_buffer[INPUT_SIZE];

/** @brief KISS FFT configuration state. */
static kiss_fftr_cfg kiss_cfg = NULL;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Performs Real FFT using the KISS FFT library.
 * 
 * @param in_buffer Input time-domain samples (float).
 * @param fft_buffer Output buffer for magnitude spectrum.
 * @param size FFT size.
 */
static void decompose_kiss(float *in_buffer, float *fft_buffer, uint32_t size) {
    kiss_fft_cpx out_cpx[(size / 2) + 1];
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
    static bool decomposition_initialized = 0;
    if (!decomposition_initialized) {
        arm_rfft_fast_init_f32(&S, size);
        decomposition_initialized = 1;
    }

    arm_rfft_fast_f32(&S, in_buffer, fft_buffer, 0);
    arm_cmplx_mag_f32(fft_buffer, fft_buffer, size / 2);
    
    fft_buffer[0] = 0; // Remove DC component
}

/**
 * @brief Compares KISS FFT and CMSIS-DSP FFT performance and detects anomalies.
 * 
 * @param in_buffer Input time-domain samples (float).
 * @param fft_buffer Output buffer for magnitude spectrum.
 * @param size FFT size.
 * @return true if peak magnitude exceeds threshold, false otherwise.
 */
static bool detect_anomaly(float *in_buffer, float *fft_buffer, uint32_t size) {
    uint32_t start, stop;
    float max_val;
    uint32_t max_idx;
    float res = (float)FS / size;

    // 1. KISS FFT (Baseline)
    start = DWT->CYCCNT;
    decompose_kiss(in_buffer, fft_buffer, size);
    stop = DWT->CYCCNT;
    
    arm_max_f32(fft_buffer, size / 2, &max_val, &max_idx);
    printf("KISS FFT:  %10" PRIu32 " cycles, Max Mag: %10.2f at %4.1f Hz\r\n", stop - start, max_val, max_idx * res);


    // 2. CMSIS FFT (Optimized)
    start = DWT->CYCCNT;
    decompose_cmsis(in_buffer, fft_buffer, size);
    stop = DWT->CYCCNT;
    
    arm_max_f32(fft_buffer, size / 2, &max_val, &max_idx);
    printf("CMSIS FFT: %10" PRIu32 " cycles, Max Mag: %10.2f at %4.1f Hz\r\n", stop - start, max_val, max_idx * res);

    // Dump frequency spectrum to UART
    dump_frequency_spectrum(fft_buffer, size / 2, max_idx, FS);

    bool anomaly = (max_val > MAGNITUDE_THRESHOLD);
    return anomaly;
}

/**
 * @brief Main task for Machine Health Detection.
 */
void machine_health_task(void) {
    static float in_buffer[FFT_SIZE];
    static float fft_buffer[FFT_SIZE];

    printf("\r\n");
    printf("Machine Health Detection Started\r\n");
    printf("Sample Rate: %d Hz, Window: %d samples (~%d ms)\r\n", FS, INPUT_SIZE, (INPUT_SIZE * 1000) / FS);

    // Enable the DWT cycle counter:
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Initialize KISS FFT
    if (kiss_cfg == NULL) {
        kiss_cfg = kiss_fftr_alloc(FFT_SIZE, 0, NULL, NULL);
    }

    while (1) {
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

        // Dump raw waveform to UART
        dump_time_waveform(mic_buffer, INPUT_SIZE);
        
        // Pre-process: Convert to float and zero-pad for FFT
        memset(in_buffer, 0, sizeof(in_buffer));
        for (int i = 0; i < INPUT_SIZE; i++) {
            in_buffer[i] = (float)mic_buffer[i];
        }
        
        // Detect Anomaly
        bool anomaly = detect_anomaly(in_buffer, fft_buffer, FFT_SIZE);
        if (anomaly) {
            printf("WARNING: Anomaly detected!\r\n");
            // Flash LED to indicate anomaly
            for (int i = 0; i < 10; i++) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_Delay(100);
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        
        // Sleep for 5 seconds
        printf("--- SLEEP MODE ---\r\n");
        PM_EnterSleep(5);
        printf("\r\n\r\n");
    }
}

/**
 * @brief Dumps the raw time-domain waveform to UART.
 */
void dump_time_waveform(int32_t *buf, size_t len) {
  printf("\r\nWAVEFORM:");
  fflush(stdout);
  for (size_t i = 0; i < len; i++) {
    printf("%s%" PRIi32, i == 0 ? "" : ",", buf[i]);
    fflush(stdout);
  }
  printf("\r\n");
}

/**
 * @brief Dumps the frequency-domain FFT magnitude spectrum to UART.
 */
void dump_frequency_spectrum(float *buf, size_t len, uint32_t max_idx, uint32_t fs) {
  printf("\r\nFFT:%" PRIu32 ",%" PRIu32, max_idx, fs);
  fflush(stdout);
  for (size_t i = 0; i < len; i++) {
    printf(",%f", buf[i]);
    fflush(stdout);
  }
  printf("\r\n");
}
