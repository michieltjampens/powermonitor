# General Info

## Changes for rev 0 -> rev 1
- Moved LPUART1 TX to PA2 because then you have the option to use USART2 instead.
- Fixed pinout of STMOD RX/TX swapped?
- Added EMI filter to I2C to connector
- Move hearbeat from PB5 to PA3
- Added resistors on the uart lines, maybe for filter
- General cleanup of traces/vias and the like

## Troubleshooting
* Measurement for current is max value
  * Check soldering of the shunt resistor
* Meausurement for voltage is way to low
  * No idea, haven't fixed that one yet