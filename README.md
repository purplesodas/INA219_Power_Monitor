# INA219 Power Monitor:

based on https://github.com/lipton5001/INA219_Power_Monitor (https://www.youtube.com/watch?v=AIoXwwDlH3g&list=PLwP7qH-peVYDnbWy8LLsBfmnyk7JchIrj&ab_channel=darieee)

A few changes from Darie's:  
Arduinto --> STM32F411 (black pill)  
USB Serial --> BLE Bluetooth Serial  
Analog Pin Battery --> 2nd INA219 for battery & power monitor power usage  
EEPROM Save State & Wear Leveling --> External EEPROM, no wear leveling (External EEPROM life is 1,000,000 cycles)

