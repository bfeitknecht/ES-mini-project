/**
 * @file machine_health.h
 * @brief Machine Health Indicator logic.
 */
#ifndef MACHINE_HEALTH_H_
#define MACHINE_HEALTH_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Main task for Machine Health Indicator.
 * 
 * Orchestrates the machine health monitoring process:
 * 1. Acquires microphone data via DFSDM+DMA (16.45kHz, 2048 samples).
 * 2. Pre-processes data (float conversion, zero-padding).
 * 3. Performs anomaly detection using KISS FFT and CMSIS-DSP FFT.
 * 4. Reports results and dumps spectrum data via UART.
 * 5. Enters low-power sleep mode between measurements.
 */
void machine_health_task(void);

#endif /* MACHINE_HEALTH_H_ */

