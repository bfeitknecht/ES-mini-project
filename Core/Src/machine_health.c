/**
 * @file machine_health.c
 * @brief Machine Health Indicator implementation.
 */
#include "machine_health.h"
#include "arm_math.h"
#include "main.h"
#include "power_manager.h"
#include <stdio.h>

#define INPUT_SIZE 128
#define FS         100

static int32_t mic_buffer[INPUT_SIZE];
static float in_buffer[INPUT_SIZE];
static float fft_buffer[INPUT_SIZE];

static bool expensive_decompose = 1;

void machine_health_task(void) {
    printf("\r\n");
    printf("Machine Health Indicator Task Started\r\n");

    while (1) {
        printf("--- ACTIVE MODE: Starting Data Collection ---\r\n");

        // 1. Acquire microphone data
        mic_dma_finished = 0;
        if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, mic_buffer, INPUT_SIZE) != HAL_OK) {
            printf("Error: Failed to start DFSDM!\r\n");
            Error_Handler();
        }

        while (!mic_dma_finished) {
            // Wait for DMA to finish
        }

        if (HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0) != HAL_OK) {
            printf("Error: Failed to stop DFSDM!\r\n");
            Error_Handler();
        }

        // 2. Pre-process: Convert to float
        for (int i = 0; i < INPUT_SIZE; i++) {
            in_buffer[i] = (float)mic_buffer[i];
        }

        // 3. Detect Anomaly
        bool anomaly = detect_anomaly(in_buffer, INPUT_SIZE);

        if (anomaly) {
            printf("STATUS: ANOMALY DETECTED!\r\n");
            // FIX: LEDs doesn't work yet
            // Flash LED to indicate anomaly
            // for (int i = 0; i < 10; i++) {
            //     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            //     HAL_Delay(100);
            // }
            // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Leave it on
        } else {
            // FIX: LED don't work
            // printf("STATUS: NORMAL\r\n");
            // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        
        // 5. Sleep for 5 seconds
        PM_EnterSleep(5);
        }
}

bool detect_anomaly(float *in_buffer, uint32_t size) {
    if (expensive_decompose) {
        printf("INFO: Running expensive manual DFT...\r\n");
        expensive_decompose_spectrum(in_buffer, size);
    } else {
        printf("INFO: Running efficient CMSIS-DSP FFT...\r\n");
        decompose_spectrum(in_buffer, size);
    }

    // Shared anomaly detection logic
    float max_val;
    uint32_t max_idx;
    arm_max_f32(fft_buffer, size / 2, &max_val, &max_idx);

    // Arbitrary threshold for demonstration
    if (max_val > 1000000.0f) { 
        return 1;
    }

    return 0;
}

void decompose_spectrum(float *in_buffer, uint32_t size) {
    // Perform FFT using CMSIS-DSP
    static arm_rfft_fast_instance_f32 S;
    static bool is_init = 0;
    if (!is_init) {
        arm_rfft_fast_init_f32(&S, size);
        is_init = 1;
    }

    arm_rfft_fast_f32(&S, in_buffer, fft_buffer, 0);
    arm_cmplx_mag_f32(fft_buffer, fft_buffer, size / 2);
    fft_buffer[0] = 0; // Remove DC component
}

void expensive_decompose_spectrum(float *in_buffer, uint32_t size) {
    // Naive DFT implementation: O(N^2)
    // This is intentionally slow to demonstrate the difference between 
    // optimized library/hardware implementations and naive software ones.
    
    for (uint32_t k = 0; k < size / 2; k++) {
        float real_sum = 0.0f;
        float imag_sum = 0.0f;
        for (uint32_t n = 0; n < size; n++) {
            float angle = 2.0f * PI * (float)k * (float)n / (float)size;
            real_sum += in_buffer[n] * cosf(angle);
            imag_sum -= in_buffer[n] * sinf(angle);
        }
        fft_buffer[k] = sqrtf(real_sum * real_sum + imag_sum * imag_sum);
    }
    fft_buffer[0] = 0; // Remove DC component
}
