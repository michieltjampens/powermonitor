# Versions

## Powerlogger 
- State: writing firmware.
- Uses STMOD board for programming and uart to usb.
- STM32L010K4, using 80% of flash/ram.

## Powerlogger USB 
- State: drawing schematic/layout.
- Changes from Powerlogger:
  - Now use LV3842 as buck.
    - cheaper
	- 1,1MHz instead of 700kHz, so smaller inductor
	- same footprint
	- 600mA instead of 1A, but board uses 50mA max.
  - STM32L010K8 for higher flash size to add usb-c pd code.
  - USB-C connector.
  - CP2102N for UART to usb.
  - Added reverse polarity protection.
  - FUSB302B for USB-C PD functionality (todo).
  - Replaced cap on outputs with fuses (todo).