/*
 * File:   lcdfunctions.c
 * Author: Nick
 *
 * Created on April 27, 2025, 12:37 PM
 */


#include <xc.h>
#include "lcdfunctions.h"

int16_t adcToTemperature(uint16_t adc10Bit)
{
    uint16_t temperatureTimesTen = 0;
    uint8_t adc8Bit = (uint8_t)(adc10Bit >> 2);
    uint8_t adcmsbs = (uint8_t)(adc10Bit & 0x0003);
    uint16_t interpolate1 = temperatureConstants[adc8Bit];
    uint16_t interpolate2 = temperatureConstants[adc8Bit + 1];
    temperatureTimesTen = ((interpolate2 - interpolate1) * adcmsbs) / 4 + interpolate1;
    
    return temperatureTimesTen;
}

void displayDigitSegments(uint8_t seg1, uint8_t seg2, uint8_t seg3, uint8_t sign, uint8_t enablecolon, uint8_t enabledp)
{
    LCDDATA0 &= 0x47;
    LCDDATA1 &= 0xEB;
    LCDDATA3 &= 0x47;
    LCDDATA4 &= 0xEB;
    LCDDATA6 &= 0x47;
    LCDDATA7 &= 0xEB;
    LCDDATA9 &= 0x47;
    LCDDATA10 &= 0xEB;
    
    LCDDATA3 |= ((seg1 & 0x01) << 3) | ((seg2 & 0x02) << 3) | ((seg2 & 0x01) << 5) | ((seg3 & 0x01) << 7);
    LCDDATA4 |= ((seg1 & 0x02) << 1) | ((seg3 & 0x02) << 3);
    LCDDATA0 |= ((seg1 & 0x20) >> 2) | ((seg2 & 0x40) >> 2) | (seg2 & 0x20) | ((seg3 & 0x20) << 2);
    LCDDATA1 |= ((seg1 & 0x40) >> 4) | ((seg3 & 0x40) >> 2);
    LCDDATA6 |= ((seg1 & 0x10) >> 1) | ((seg2 & 0x04) << 2) | ((seg2 & 0x10) << 1) | ((seg3 & 0x10) << 3);
    LCDDATA7 |= (seg1 & 0x04) | ((seg3 & 0x04) << 2);
    LCDDATA9 |= (sign << 3) | ((seg2 & 0x08) << 1) | (enablecolon << 5) | (enabledp << 7);
    LCDDATA10 |= ((seg1 & 0x08) >> 1) | ((seg3 & 0x08) << 1);
}

void displayNumber(int16_t value, uint8_t enablecolon, uint8_t enabledp, uint8_t valid)
{
    uint8_t sign = 0;
    uint8_t dig1 = 0;
    uint8_t dig2 = 0;
    uint8_t dig3 = 0;
    uint8_t seg1 = 0;
    uint8_t seg2 = 0;
    uint8_t seg3 = 0;
    if (valid)
    {
        // Calculate sign bit and digit patterns
        sign = value < 0 ? 1 : 0;
        value = abs(value);
        dig1 = (value / 100) % 10;
        dig2 = (value / 10) % 10;
        dig3 = value % 10;
        if (dig1 == 0)
        {
            dig1 = 16;  // blank leading zero
            if (dig2 == 0)
            {
                dig2 = 16;
            }
        }
        seg1 = segmentPatterns[dig1];
        seg2 = segmentPatterns[dig2];
        seg3 = segmentPatterns[dig3];
    }
    else
    {
        seg1 = seg2 = seg3 = segmentPatterns[17];
    }
    
    // Update registers with digit patterns
    displayDigitSegments(seg1, seg2, seg3, sign, enablecolon, enabledp);
}

void displayTemperature(int16_t value, uint8_t enablecolon, uint8_t valid)
{
    // Decimal point handling - for numbers > 99.9, eliminate tenths and don't show dp
    uint8_t enabledp = 0;
    uint8_t sign = 0;
    uint8_t dig1 = 0;
    uint8_t dig2 = 0;
    uint8_t dig3 = 0;
    uint8_t seg1 = 0;
    uint8_t seg2 = 0;
    uint8_t seg3 = 0;
    if (valid)
    {
        if (value <= 999 && value >= -999)
        {
            enabledp = 1;
        }
        else
        {
            value = value / 10;
        }
        // Calculate sign bit and digit patterns
        sign = value < 0 ? 1 : 0;
        value = abs(value);
        dig1 = (value / 100) % 10;
        dig2 = (value / 10) % 10;
        dig3 = value % 10;
        if (dig1 == 0)
        {
            dig1 = 16;  // blank leading zero
        }
        seg1 = segmentPatterns[dig1];
        seg2 = segmentPatterns[dig2];
        seg3 = segmentPatterns[dig3];
    }
    else
    {
        seg1 = seg2 = seg3 = segmentPatterns[17];
    }
    
    // Update registers with digit patterns
    displayDigitSegments(seg1, seg2, seg3, sign, enablecolon, enabledp);
}

void setBacklight(uint8_t enable)
{
    if (enable > 0)
    {
        LATBbits.LATB3 = 1;
    }
    else
    {
        LATBbits.LATB3 = 0;
    }
}