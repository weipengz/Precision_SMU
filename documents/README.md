# IMEBUILD V2 Project

This board is designed to support current/voltage source measurement units (SMUs) with isolated loads. The idea is to make it as modular as possible, with daughterboards containing sets of 8 IV Sources. The IV-source is essentially an AD5753 chip (16-bit Current and Voltage DAC) with galvanic isolation provided by an ADP1031 chip (3-channel isolated uPMU with seven digital isolators). The AD5753 chip provides software-switched ±24mA current source (with voltage readout) and ±10V voltage source (without current readout). Current readout is implemented by using a 2.49Ω sense resistor and a current-sensing amplifier from TI, followed by a 12-bit ADC.

## Main boards

- Daughterboard card
    + 8 IV sources with sensing units
    + SPI control
    + Efficient power conversion with flyback converters and buck-boost converters (switching regulators).
    + Comprehensive diagnostics including voltage readouts and circuit protection faults.
    + One LED that lights on when AD5753 FAULT pin is asserted.

- Arduino Yun 2 Controller card
    + Simple Arduino Yun 2 breakout to the Backplane Controller connector
    + This card should optionally have a +5V power supply from a microusb port.

- Backplane
    + Supports up to 16 Daughterboard cards
    + Supports one controller card
    + +12V power from a DC barrel jack
    + +5V power regulated from +12V (use isolation or disable this option)
    + +5V power from controller card if +12V not present
