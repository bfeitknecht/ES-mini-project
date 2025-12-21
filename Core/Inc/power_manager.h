/**
 * @file power_manager.h
 * @brief Power management utilities for STM32L4.
 */

#ifndef __POWER_MANAGER_H
#define __POWER_MANAGER_H

#include "stm32l4xx_hal.h"

/**
 * @brief Enters Sleep Mode for a specified number of seconds.
 * 
 * This function suspends the SysTick, configures a low-power timer (LPTIM1)
 * to wake up the CPU after the specified duration, and enters Sleep Mode.
 * 
 * @param seconds Duration to sleep in seconds.
 */
void PM_EnterSleep(uint32_t seconds);

#endif /* __POWER_MANAGER_H */
