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
 * Performs data acquisition, FFT, anomaly detection, and sleeps.
 */
void machine_health_task(void);

/**
 * @brief Main anomaly detection function.
 * Conditionally calls decompose_spectrum or decompose_spectrum_expensive.
 * @param in_buffer Pointer to the input time-domain buffer.
 * @param size Size of the input buffer.
 * @return true if anomaly detected, false otherwise.
 */
bool detect_anomaly(float *in_buffer, uint32_t size);

/**
 * @brief Process data using efficient CMSIS-DSP FFT.
 * @param in_buffer Pointer to the input time-domain buffer.
 * @param size Size of the input buffer.
 */
void decompose_spectrum(float *in_buffer, uint32_t size);

/**
 * @brief Process data using expensive manual DFT.
 * @param in_buffer Pointer to the input time-domain buffer.
 * @param size Size of the input buffer.
 */
void expensive_decompose_spectrum(float *in_buffer, uint32_t size);

#endif /* MACHINE_HEALTH_H_ */
