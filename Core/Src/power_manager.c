/**
 * @file power_manager.c
 * @brief Power management implementation using RTC.
 */

#include "power_manager.h"
#include "main.h"
#include <stdio.h>

extern RTC_HandleTypeDef hrtc;

void PM_EnterSleep(uint32_t seconds) {
    if (seconds == 0) return;
    // 1. Suspend SysTick to prevent 1ms wakeups
    HAL_SuspendTick();

    // 2. Configure RTC Wakeup Timer
    // Using RTCCLK/16. If LSE = 32.768kHz, RTCCLK/16 = 2048Hz.
    uint32_t wakeup_counter = (seconds * 2048) - 1;
    
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeup_counter, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK) {
        Error_Handler();
    }

    // 3. Enter Sleep Mode
    printf("INFO: sleep for %lu seconds\r\n", seconds);
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    // 4. Deactivate RTC Wakeup Timer after wakeup
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    // 5. Resume SysTick
    HAL_ResumeTick();

    // 6. Log wakeup
    printf("INFO: woke up!\r\n");
}
