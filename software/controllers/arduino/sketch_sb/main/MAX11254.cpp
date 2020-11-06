/*
MAX11254.h - MAX11254 ADC functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include <SPI.h>
#include "MAX11254.h"

SPISettings max11254_spi_settings (MAX11254_SPI_CLOCK, MSBFIRST, SPI_MODE0);

exit_code max11254_reg_write (spi_device dev, uint8_t reg_address, uint8_t* reg_value, uint8_t reg_size)
{
    ss_select(dev);
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(MAX11254_REG_WRITE(reg_address));
    for(int i=0; i < reg_size; i++){
        SPI.transfer(reg_value[i]);
    }
    SPI.endTransaction();
    ss_deselect(dev);

    return SUCCESS;
}

exit_code max11254_reg_read (spi_device dev, uint8_t reg_address, uint8_t* reg_value, uint8_t reg_size)
{
    ss_select(dev);
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(MAX11254_REG_READ(reg_address));
    for(int i=0; i < reg_size; i++){
        reg_value[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    ss_deselect(dev);

    return SUCCESS;
}

exit_code max11254_reg_read8 (spi_device dev, uint8_t reg_address, uint8_t* reg_value)
{
    return max11254_reg_read(dev, reg_address, reg_value, 1);
}

exit_code max11254_reg_read24 (spi_device dev, uint8_t reg_address, uint32_t* reg_value)
{
    exit_code ret = SUCCESS;
    uint8_t reg_data[3];
    ret |= max11254_reg_read(dev, reg_address, reg_data, 3);
    *reg_value = ((((uint32_t) reg_data[0]) << 16) +
                  (((uint32_t) reg_data[1]) << 8) +
                  (((uint32_t) reg_data[2]) << 0));
    return ret;
}

exit_code max11254_reg_read_status (spi_device dev, uint32_t* reg_value)
{
    return max11254_reg_read24(dev, MAX11254_REG_STAT, reg_value);
}

exit_code max11254_reg_write8 (spi_device dev, uint8_t reg_address, uint8_t reg_value)
{
    return max11254_reg_write(dev, reg_address, &reg_value, 1);
}

exit_code max11254_reg_write8_mask (spi_device dev, uint8_t reg_address, uint8_t mask, uint8_t reg_value)
{
    uint8_t reg_data;
    exit_code ret = SUCCESS;
    ret |= max11254_reg_read(dev, reg_address, &reg_data, 1);
    reg_data &= ~mask;
    reg_data |= reg_value;
    ret |= max11254_reg_write(dev, reg_address, &reg_data, 1);
    return ret;
}

exit_code max11254_command (spi_device dev, uint8_t cmd, bool fast = false)
{
    if(!fast){ss_select(dev);}
    else {ss_fast_select(dev);}
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(cmd);
    SPI.endTransaction();
    ss_deselect(dev);

    return SUCCESS;
}


exit_code max11254_powerdown (spi_device dev)
{
    return max11254_command(dev, MAX11254_POWERDOWN);
}


exit_code max11254_calibrate (spi_device dev, uint8_t cal_type)
{
    /* cal_type denotes the type of calibration performed.
     * 0b00 refers to self-calibration,
     * 0b01 refers to a system offset calibration, and
     * 0b10 refers to a system full-scale calibration
     *
     * You can issue a self-calibration at any time.
     * For the offset calibration, source 0V to the ADC and run it.
     * For the full-scale calibration, source maximum voltage (VREF) and run it.
     */

    exit_code ret = SUCCESS;
    cal_type &= 0b11;

    // The calibration command can only be issued in sequencer mode 1 (0b00)
    uint8_t seq_data;
    ret |= max11254_reg_read8(dev, MAX11254_REG_SEQ, &seq_data);
    if((seq_data & MAX11254_SEQ_MODE_MSK) != MAX11254_SEQ_MODE_MODE(0b00))
    {
        ret |= CAL_FAILURE;
        return ret;
    }

    switch (cal_type) {
        case 0b00 :  // Performs a self-calibration
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL1, MAX11254_CTRL1_CAL_MASK, MAX11254_CTRL1_CAL_MODE(0b00)
                );

            max11254_command(dev, MAX11254_CALIBRATE, true);
            delay(200);  // "A self-calibration requires 200ms to complete."
            break;
        case 0b01 :  // Performs a system-level offset calibration
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL1, MAX11254_CTRL1_CAL_MASK, MAX11254_CTRL1_CAL_MODE(0b01)
                );
            max11254_command(dev, MAX11254_CALIBRATE, true);
            delay(100);  // "A system offset calibration requires 100ms to complete."
            break;
        case 0b10 :  // Performs a system-level full-scale calibration
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL1, MAX11254_CTRL1_CAL_MASK, MAX11254_CTRL1_CAL_MODE(0b10)
                );
            max11254_command(dev, MAX11254_CALIBRATE, true);
            delay(100);  // "A system full-scale calibration requires 100ms to complete."
            break;
        default :
            ret |= FAILURE;
    }
    return ret;
}

exit_code max11254_swreset (spi_device dev)
/* The host can issue a software reset to restore the default
 * state of the MAX11254. A software reset sets the interface
 * registers back into their default states and resets the internal
 * state machines. However, a software reset does not emulate the
 * complete POR or hardware reset sequence.
 *
 * Two SPI transactions are required to issue a software reset:
 * First set CTRL1:PD[1:0] to ‘11’ (RESET).
 * Then issue a conversion command with MODE[1:0] set to ‘01’.
 * To confirm the completion of the reset operation, STAT:PDSTAT
 * and STAT:INRESET must be monitored.
 *
 * Figure 16 shows the state transition for the RESET command
 * and the relative timing of STAT register update. During reset,
 * INRESET = ’1’ and PDSTAT= ‘11’. The SPI interface cannot be
 * written until MAX11254 enters STANDBY state where PDSTAT = ‘10’.
 * To confirm completion of the RESET command, monitor for
 * INRESET = ‘0’ and PDSTAT = ‘10’.0 Table 6 summarizes the
 * maximum delay for reset operation.
 */
{
    uint32_t stat_data;
    exit_code ret = SUCCESS;
    ret |= max11254_reg_write8_mask(dev, MAX11254_REG_CTRL1, MAX11254_CTRL1_PD_MASK, MAX11254_CTRL1_PD_MODE(0b11));  // Set POWERDOWN to reset
    ret |= max11254_powerdown(dev);
    EARLY_RETURN(ret);  // Early stop if there is an issue.

    unsigned long initial_time = millis();
    bool reset_in_progress = true;
    do
    {
        ret |= max11254_reg_read_status(dev, &stat_data);
        reset_in_progress &= (MAX11254_STAT_INRESET_FILTER(stat_data) ||
                              (MAX11254_STAT_PDSTAT_FILTER(stat_data) != 0b10));
        // This should take a maximum of 28 ms
        reset_in_progress &= (millis() - initial_time < 28);
        delay(1);
    } while (reset_in_progress);

    return ret;
}

exit_code max11254_setup (spi_device dev, bool skip_reset)
{
    exit_code ret = SUCCESS;

    if (!skip_reset) ret |= max11254_swreset(dev);

    ret |= max11254_reg_write8(dev, MAX11254_REG_CTRL1,     MAX11254_CTRL1_DEFAULT);
    ret |= max11254_reg_write8(dev, MAX11254_REG_CTRL2,     MAX11254_CTRL2_DEFAULT);
    ret |= max11254_reg_write8(dev, MAX11254_REG_CTRL3,     MAX11254_CTRL3_DEFAULT);
    ret |= max11254_reg_write8(dev, MAX11254_REG_GPIO_CTRL, MAX11254_GPIO_CTRL_DEFAULT);
    ret |= max11254_reg_write8(dev, MAX11254_REG_SEQ,       MAX11254_SEQ_DEFAULT);

    return ret;

}

exit_code max11254_prepare_convert (spi_device dev, uint8_t pga)
{
    exit_code ret = SUCCESS;

    // Set sequencer mode 1
    ret |= max11254_reg_write8_mask(dev, MAX11254_REG_SEQ, MAX11254_SEQ_MODE_MSK, MAX11254_SEQ_MODE_MODE(0b00));
    EARLY_RETURN(ret)

    // Set pga
    switch (pga)
    {
        case 1:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b000) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 2:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b001) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 4:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b010) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 8:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b011) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 16:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b100) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 32:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b101) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 64:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b110) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        case 128:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b111) | MAX11254_CTRL2_PGAEN_MODE(1));
            break;
        default:
            ret |= max11254_reg_write8_mask(dev,
                MAX11254_REG_CTRL2,
                MAX11254_CTRL2_PGA_MASK | MAX11254_CTRL2_PGAEN_MASK,
                MAX11254_CTRL2_PGA_MODE(0b000) | MAX11254_CTRL2_PGAEN_MODE(0));
    }
    return ret;
}

exit_code max11254_convert (spi_device dev, uint8_t channel, uint8_t rate, uint32_t* data)
{
    exit_code ret = SUCCESS;
    channel &= 0x7;

    // Select channel
    ret |= max11254_reg_write8_mask(dev, MAX11254_REG_SEQ, MAX11254_SEQ_MUX_MSK, MAX11254_SEQ_MUX_MODE(channel));
    EARLY_RETURN(ret)  // Early stop if there is an issue.

    // Trigger conversion
    ret |= max11254_command(dev, MAX11254_CONVERT(rate));
    EARLY_RETURN(ret)  // Early stop if there is an issue.

    uint32_t stat_data;
    // wait until STAT:RDY == 1, which indicates data is ready
    unsigned long initial_time = millis();
    unsigned long time_step_ms = 0;
    do
    {
        ret |= max11254_reg_read_status(dev, &stat_data);
        // stop early if 100ms have elapsed
        if (millis() - initial_time > MAX11254_CONVERSION_TIMEOUT_MS)
        {
            return (ret | TIMEOUT);
        }
        delay(time_step_ms);
        time_step_ms += 3; // Linear time step increase
    } while (!MAX11254_STAT_RDY_FILTER(stat_data));

    ret |= max11254_reg_read24(dev, MAX11254_REG_DATA0 + channel, data);
    return ret;
}
