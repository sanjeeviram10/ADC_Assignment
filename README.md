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

# ADC_receiver - ESP32 UART Receiver for LM35

This project runs on **ESP32** and receives ADC values from the STM32 project via UART. It converts the ADC values to temperature in Celsius and blinks an LED if temperature exceeds thresholds.

## Features
- Reads ADC values from STM32 over UART (RX pin 16)
- Converts ADC to voltage, then to temperature
- Blinks onboard LED (GPIO2) if temperature > 40°C
- Simple switch-case to control LED blinking rate

## Pin Configuration
- **GPIO16** → RX (connect to STM32 TX PB6)
- **GPIO2** → LED

## Usage
1. Connect STM32 PB6 → ESP32 RX (GPIO16)
2. Connect LED to GPIO2 (or use onboard LED)
3. Upload code to ESP32
4. Open Serial Monitor to view temperature readings

## Author
Sanjeeviram M

