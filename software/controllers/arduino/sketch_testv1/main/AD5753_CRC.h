#include <Arduino.h>
// ensure this library description is only included once
#ifndef AD5753_CRC_h
#define AD5753_CRC_h

// Taken from https://github.com/analogdevicesinc/no-OS/blob/master/ad5758-sdz/ad5758.c

#define AD5753_CRC8_POLY    0x07 // x^8 + x^2 + x^1 + x^0

/**
 * Compute CRC8 checksum.
 * @param data - The data buffer.
 * @param data_size - The size of the data buffer.
 * @return CRC8 checksum.
 */
static uint8_t ad5753_compute_crc8(uint8_t *data, uint8_t data_size)
{
    uint8_t i;
    uint8_t crc = 0;

    while (data_size) {
        for (i = 0x80; i != 0; i >>= 1) {
            if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
                crc <<= 1;
                crc ^= AD5753_CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
        data++;
        data_size--;
    }

    return crc;
}

#endif
