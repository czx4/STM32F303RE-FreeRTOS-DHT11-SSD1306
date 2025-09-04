# STM32F303RE-FreeRTOS-DHT11-SSD1306

Reads DHT11 sensor data and displays readings on an SSD1306 OLED using I²C with DMA on STM32F303RE under native FreeRTOS without ST wrapper.
Originally generated template with CubeMX without FreeRTOS, then the project was extended with native FreeRTOS and full functionality with HAL.
The Repository doesn't provide .ioc file as it would break everything by regenerating code.

## Features

- Read temperature & humidity from DHT11
- Display data on SSD1306 OLED
- Use I²C with DMA for efficient communication
- Powered by native FreeRTOS on STM32F303RE

## Hardware

- STM32F303RE Nucleo Board
- DHT11 Sensor
- SSD1306 OLED Display
- 10 kΩ resistor that will be used as Pull-up resistor (DHT11 Data Sheet recommends 5kΩ but i found that mine worked more reliable with more)

## Schematics and Photos of Hardware
<img width="500" height="401" alt="schematic F303RE DHT11 SSD1306" src="https://github.com/user-attachments/assets/9de20c8a-b320-41b5-957b-600de3da038e" />
<img width="500" height="400" alt="wide shot F303RE DHT11 SSD1306" src="https://github.com/user-attachments/assets/bea7f3f4-64a2-4649-b933-bb943eb21033" />
<img width="500" height="400" alt="up close shot F303RE DHT11 SSD1306" src="https://github.com/user-attachments/assets/eb9f0ae7-b732-4319-af57-f4a04a15331f" />
