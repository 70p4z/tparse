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

## Nucleo F767ZI solax pylontech
Sometimes it's harder than it looks to build a solar system with batteries from scratch. Especially if you build it from the electronic engineer point of view.
Voltage are in range, and the datasheets seems like a match. 
BUT it turns out you're missing the whole point. Many players in the ecosystem are not willing for compatibility. And the war moves from electronic to communication protocol to make systems work.

I got into such a refreshing problem when I decided to go solar with a huge amount of auto consumption. I went for modular Pylontech H48050 (high voltage (HV)) based system (SC0500 + 8 H48050 in the end). And tried to connect it with a Solax X1 Hybrid G4.

The very point is that it won't work until you ~handle~ rewrite some part of the protocol on the fly.

The example for nucleo was mainly done to have the communication working, and various tries to allow for power optimisation, or workaround on problems (not respected zero injection).

Anyway, this project is only based on tparse for the ring buffer management (but it started with 2 iobridges until I found a board (f767zi) that handle 2 CAN bus to perform the man-in-the-middle).

The project further grown into a fully fledged reporting system to connect with home assistant (through a http sensor python script running on a raspberry pi zero w).

I'll try to add a design later on to design the daughter board on the nucleo for the various connections (mate box relay, CAN to the inverter, CAN to the battery, I2C for the reporting, serial for the pocket wifi link).

Long story short, the bugfix for the protocol is to only patch the CAN message 0x1877 byte 4 from whatever (IIRC it was 0x01) to 0x83 to make the inverter happily use the BMS (and stop complaining about invalid voltage).
