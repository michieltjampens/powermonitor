# Versions

## Powerlogger
- Uses STMOD board for programming and uart to usb.
- STM32L010K4, using 85% of flash/ram

## Powerlogger USB
- Changes from Powerlogger:
  - Now use LV3842 as buck
    - Cheaper
	- 1,1MHz instead of 700kHz so smaller inductor
	- Same footprint
	- 600mA instead of 1A, but board uses 50mA max.
  - STM32L010K8 for higher flash size
  - USB-C connector
  - CP2102N for UART to usb
  - FUSB302B for USB-C PD functionality (todo)
  - Replaced cap on outputs with fuses