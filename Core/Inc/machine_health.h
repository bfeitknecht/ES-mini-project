/**
 * @file machine_health.h
 * @brief Machine Health Indicator logic.
 */
#ifndef MACHINE_HEALTH_H_
#define MACHINE_HEALTH_H_

#include <stdint.h>
#include <stdbool.h>

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

/**
 * @brief Dumps the raw time-domain waveform to UART.
 * 
 * @param buf Pointer to the buffer containing microphone samples.
 * @param len Number of samples to dump.
 */
void dump_time_waveform(int32_t *buf, size_t len);

/**
 * @brief Dumps the frequency-domain FFT magnitude spectrum to UART.
 * 
 * @param buf Pointer to the buffer containing magnitude values.
 * @param len Number of magnitude bins.
 * @param max_idx Index of the peak frequency bin.
 * @param fs Sampling frequency in Hz.
 */
void dump_frequency_spectrum(float *buf, size_t len, uint32_t max_idx, uint32_t fs);

#endif /* MACHINE_HEALTH_H_ */

