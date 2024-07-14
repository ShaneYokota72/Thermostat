# Arduino Thermostat
This thermostat utilizes an Arduino-based microcontroller to create a dial-type thermometer that can display both local and remote temperature readings.

## What It Does
The Temperature Monitoring System offers the following features:
- Measures and displays local temperature using a DS18B20 sensor
- Shows temperature on both an LCD display and a servo-controlled dial indicator
- Allows setting a temperature threshold using a rotary encoder
- Provides visual alerts via LED indicators (green for normal, blinking red for warning, solid red for alert)
- Includes an audio alert (buzzer) for high temperature conditions
- Communicates with other units via RS-232 serial interface to share temperature data
- Stores temperature threshold in non-volatile EEPROM memory
- Allows switching between local and remote temperature display


## Thermostat Image
![image](https://github.com/user-attachments/assets/15f0f49a-970f-4647-8029-fe326c573d67)

## Technical Details
- Microcontroller: Arduino (ATmega328P)
- Temperature Sensor: DS18B20
- Display: LCD and Servo-controlled dial
- User Input: Rotary encoder, buttons
- Alerts: LEDs, Buzzer
- Communication: RS-232 serial interface
- Memory: EEPROM for storing settings
