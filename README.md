# ADC_task01 - STM32F401CCU6 LM35 Temperature Reader

This project reads temperature from an **LM35 sensor** using **STM32F401CCU6 ADC1** and sends raw ADC values over **UART1**. The data can be received by another device like ESP32.

## Features
- Reads analog voltage from LM35 sensor (PA0)
- Converts voltage to temperature
- Sends ADC values via UART1 (PB6 = TX)
- Bare-metal programming (no HAL or CubeMX used)

## Pin Configuration
- **PA0** → LM35 analog output
- **PB6** → UART1 TX (to ESP32 RX)
- **PB7** → UART1 RX (optional)

## Usage
1. Connect LM35 to PA0, GND, and VCC (3.3V or 5V depending on board).
2. Connect PB6 to ESP32 RX pin.
3. Compile and flash code to STM32F401CCU6.
4. ESP32 or PC can read the ADC values via UART.

## Author
Sanjeeviram Lap
