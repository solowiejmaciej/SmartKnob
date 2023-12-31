# SmartKnob
##### ESP32 IoT device with MQTT
## Features:
- Rotary encoder
- Push button
- MQTT client
- NTP client

## How it works:
- Reads rotary encoder value and publishes it to MQTT broker every time it changes
- Reads push button state and publishes it to MQTT broker every time it changes

## Picture
![SmartKnob](https://i.imgur.com/U5KOut6.jpg)

## Circuit
![SmartKnob](https://i.imgur.com/SMTS8Hz.png)


### MQTT Topics:
- `/home/sensors/button` - Button state (0 or 1)
- `/home/sensors/rotary` - Rotary encoder value (0-100)

### Libraries used:
- PubSubClient
- ArduinoJson
- NTPClient
- AiEsp32RotaryEncoder

### Hardware used:
- ESP32 DevKit
- Waveshare Rotary Encoder


###### Developed using PlatformIO

