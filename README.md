# Machine Health Indicator (STM32L476)

This project implements a Machine Health Indicator on an STM32L476RG Nucleo board. It monitors machine health by analyzing acoustic patterns from a microphone using Digital Signal Processing (DSP) techniques.

## System Architecture

The application follows a periodic monitoring cycle:
1. **Data Acquisition**: Samples are collected from a microphone.
2. **Signal Processing**: Time-domain data is converted to the frequency domain.
3. **Anomaly Detection**: The frequency spectrum is analyzed for abnormal patterns.
4. **Reporting**: Results are sent via UART and indicated via on-board LEDs.
5. **Power Management**: The system enters a low-power sleep mode between checks.

## Key Components

### 1. Data Acquisition (DFSDM + DMA)
The system uses the **Digital Filter for Sigma Delta Modulators (DFSDM)** peripheral to interface with a digital microphone (acting as a vibration sensor).
- **DFSDM Configuration**: Configured with a Sinc4 filter and an oversampling ratio to achieve a sampling frequency ($f_s$) of **100 Hz**.
- **DMA**: Direct Memory Access is used to transfer samples from the DFSDM filter to a buffer in RAM without CPU intervention.
- **Synchronization**: The application polls a `volatile bool` flag (`mic_dma_finished`) which is set in the `HAL_DFSDM_FilterRegConvCpltCallback` when the buffer is full.

### 2. Signal Processing
Once a buffer of **100 samples** is collected (representing **1 second** of data), the system performs frequency analysis.
- **Top-level Entry**: `detect_anomaly` acts as the main entry point.
- **Conditional Processing**: Based on a internal flag, it calls either:
    - `decompose_spectrum`: Uses the optimized **ARM CMSIS-DSP RFFT** library (zero-padded to 128).
    - `expensive_decompose_spectrum`: Uses a manual **O(N^2) DFT** implementation for performance comparison.
- **Magnitude Calculation**: The complex FFT/DFT output is converted to a magnitude spectrum.
- **DC Removal**: The 0Hz component is cleared to remove any DC offset.

### 3. Anomaly Detection
The `detect_anomaly` function in [Core/Src/machine_health.c](Core/Src/machine_health.c) implements a threshold-based logic:
- It finds the maximum magnitude in the frequency spectrum.
- If the maximum exceeds a predefined threshold, an anomaly is flagged.
- *Note: This is a placeholder for more sophisticated algorithms.*

### 4. User Interface & Feedback
- **UART Console**: Status messages ("NORMAL" or "ANOMALY DETECTED") and system logs are printed to UART2 (115200 baud).
- **LED Indication**: The on-board LED (LD2/PA5) flashes rapidly when an anomaly is detected and remains on until the next cycle.

### 5. Power Management
To maximize energy efficiency, the system uses a dedicated power management module ([Core/Src/power_manager.c](Core/Src/power_manager.c)):
- **Uninterrupted Sleep**: Instead of waking up every 1ms via SysTick, the system suspends the SysTick interrupt (`HAL_SuspendTick()`) before entering sleep.
- **RTC Wakeup**: The **Real-Time Clock (RTC)** Wakeup Timer is used to fire an interrupt after the desired sleep duration (e.g., 5 seconds).
- **Wakeup Source**: The CPU enters Sleep Mode via `HAL_PWR_EnterSLEEPMode` and remains in a low-power state until the RTC Wakeup interrupt occurs.
- **Restoration**: Upon wakeup, the RTC Wakeup Timer is deactivated, and SysTick is resumed (`HAL_ResumeTick()`).

## Build and Run

### Prerequisites
- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- CMake and Ninja
- STM32L476RG Nucleo Board

### Build Commands
Using CMake presets:
```bash
cmake --preset Debug
cmake --build build/Debug
```

### Flashing
Flash the resulting `.elf` or `.bin` file using OpenOCD, STM32CubeProgrammer, or by dragging it onto the Nucleo's USB drive.
