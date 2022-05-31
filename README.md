# TParse

A minimalistic C library to parse text in microcontroller without memory allocation.

Supports:
- Hex arrays
- Hex integer
- Decimal integer
- List of allowed keywords
- Raw text token fetch

## IOBridge
A basic iobridge example project is provided, making extended use of the library.
The project is splitted in:
- A iobridge application for NUCLEO-L476RG;
- A python (3) library to communicate with.

The iobridge goal is to offer an UART driven text interface to bridge to various protocols (raw GPIO, I2C bus, CAN bus, ISO7816-3 T=0). The project will be extended with ADC, OneWire and SPI.

