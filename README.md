# Power Monitor

Small board to monitor power consumption of attached devices. 
Working either standalone with data stored on onboard flash or realtime by sending measurements to PC through USB.

## Features
- Up to 4 concurrent devices monitored.
- 30V maximum system voltage, haven't tested the current limit yet.
- 8MB local flash storage
- Onboard programmer/debugger/usb to uart with [STLINK V3MODS](https://www.st.com/en/development-tools/stlink-v3mods.html)
- Long term energy consumption measurement

## Progress
- Currently testing rev0
  - Programming and lpuart via vcp work (but had to swap rx/tx)
  - I2C and PAC1954 work
  - Script written for dcafs that reads, processes and stores voltage,current and energy 

## Todo
- Flash via spi -> store and download data
- PAC Alerts
- RTC
- DMA reading I2C and writing UART

## Repo folders content
### board
Diptrace files, both schematic and layout

### datasheets
Contains the relevant datasheets
- **DC-DC** [MPM3506A](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MPM3506AGQV-Z/document_id/2106/)
- **Flash Mem** [MX25L6433F](https://www.macronix.com/Lists/Datasheet/Attachments/8911/MX25L6433F,%203V,%2064Mb,%20v1.3.pdf)
- **MCU** [STM32L040K](https://www.st.com/resource/en/datasheet/stm32l010k4.pdf)
- **Monitor** [PAC1954](https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/PAC195X-Family-Data-Sheet-DS20006539.pdf)

### dcafs
Contains the files needed to read/process the data from the board with [dcafs](https://github.com/michieltjampens/dcafs).

### firmware
Contains the ST CubeIDE project with bare-metal c code.
