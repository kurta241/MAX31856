# MAX38156
pycom LoPy library and example code for Adafruit MAX31856 Universal Thermocouple Amplifier

## Wiring
|**LoPy**  |     | **MAX31856**|
|----------|-----|----------|
|3.3V      | --> |   Vin    | 
|Gnd       | --- |   Gnd    |
|P9  (SDA) | --> |   CS     |
|P10 (SCL) | --> |   SCK    |
|P11 (MOSI)| --> |   SDI    |
|P12 (MISO)| <-- |   SDO    |

CS: 
Use different output Pins for multiple sensors. Pin 'P9' is default chip select output pin.

`tc = MAX31856(avgsel=0x03,tc_type=MAX31856_K_TYPE, cs_pin='P9')`
   
