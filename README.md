# ubme280
Micropython driver for BME280 Sensor.

It has been tested on ESP8266 and ESP32.

The code is intended to be self documenting.

## Quickstart

```python
# REPL on an ESP32
>>> import bme280
>>> from machine import I2C, Pin
>>> i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
>>> bme = bme280.BME280(i2c)
>>> bme.read()
>>> bme.temperature
16.27
>>> bme.pressure
100366.4
>>> bme.humidity
56.88574
```
