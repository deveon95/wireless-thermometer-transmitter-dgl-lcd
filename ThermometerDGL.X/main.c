 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

#include "mcc_generated_files/system/system.h"
#include "lcdfunctions.h"
#include "mcc_generated_files/timer/delay.h"
#include "mcc_generated_files/adc/adc.h"
#include "mcc_generated_files/nvm/nvm.h"

#define SW1_PRESSED (IO_RB6_GetValue() ^ 1)
#define SW2_PRESSED (IO_RB7_GetValue() ^ 1)

/*
    Main application
*/
uint8_t mode;
#define MODE_IDLE_1 0
#define MODE_IDLE_2 1
#define MODE_SET_CONT_INTRO 2
#define MODE_SET_CONT 3
#define MODE_SET_LRLAT_INTRO 4
#define MODE_SET_LRLAT 5
#define MODE_SET_THERMISTORS_INTRO 6
#define MODE_SET_THERMISTORS 7
#define MODE_SET_ID1_INTRO 8
#define MODE_SET_ID1 9
#define MODE_SET_ID2_INTRO 10
#define MODE_SET_ID2 11
#define MODE_SET_CAL1_INTRO 12
#define MODE_SET_CAL1 13
#define MODE_SET_CAL2_INTRO 14
#define MODE_SET_CAL2 15

#define EEPROM_BYTES_IN_USE 9
#define EEPROM_MAGIC_NUMBER 20

#define CAL_LOWER_LIMIT -31
#define CAL_UPPER_LIMIT 31
#define VALID_THRESHOLD 2

uint8_t thermistors;    // 0 for 1, 1 for 2
uint8_t id1;             // ID of first thermistor
uint8_t id2;
int8_t cal1;
int8_t cal2;
uint8_t backlightCounter;

// Used for inline assembly
volatile uint8_t bitCounter __at(0x20);
volatile uint8_t preamble __at (0x21);
volatile uint8_t startChar __at (0x22);
volatile uint8_t data0 __at (0x23);
volatile uint8_t data1 __at (0x24);
volatile uint8_t data2 __at (0x25);
volatile uint8_t data3 __at (0x26);
volatile uint8_t data4 __at (0x27);
volatile uint8_t data5 __at (0x28);
volatile uint8_t data6 __at (0x29);
volatile uint8_t data7 __at (0x30);
volatile uint8_t extraByte __at(0x31);
volatile uint8_t delayCounter __at(0x32);
uint8_t txData[8] = {0};
uint8_t crc;

/**
 * Load settings from EEPROM into global variables
 */
void loadSettings()
{
    uint8_t checkByte = 0;
    uint8_t eepromData[EEPROM_BYTES_IN_USE] = {0};
    for (uint8_t i = 0; i < EEPROM_BYTES_IN_USE; i++)
    {
        eepromData[i] = (uint8_t)EEPROM_Read((eeprom_address_t)i);
        if (i > 0)
        {
            checkByte += eepromData[i];
        }
    }
    if ((eepromData[0] == checkByte) && (eepromData[1] == EEPROM_MAGIC_NUMBER))
    {
        LCDCST = eepromData[2];
        LCDRL |= eepromData[3] & 0x07;
        thermistors = eepromData[4];
        id1 = eepromData[5];
        id2 = eepromData[6];
        cal1 = eepromData[7];
        cal2 = eepromData[8];
    }
}

/**
 * Save settings to EEPROM from global variables
 */
void saveSettings()
{
    uint8_t checkByte = 0;
    
    NVM_UnlockKeySet(UNLOCK_KEY);
    
    EEPROM_Write((eeprom_address_t)1, (eeprom_data_t)EEPROM_MAGIC_NUMBER);
    checkByte += EEPROM_MAGIC_NUMBER;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)2, (eeprom_data_t)LCDCST);
    checkByte += LCDCST;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)3, (eeprom_data_t)(LCDRL & 0x07));
    checkByte += LCDRL & 0x07;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)4, (eeprom_data_t)thermistors);
    checkByte += thermistors;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)5, (eeprom_data_t)id1);
    checkByte += id1;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)6, (eeprom_data_t)id2);
    checkByte += id2;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)7, (eeprom_data_t)cal1);
    checkByte += cal1;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)8, (eeprom_data_t)cal2);
    checkByte += cal2;
    while(NVM_IsBusy());
    
    EEPROM_Write((eeprom_address_t)0, (eeprom_data_t)checkByte);
    while(NVM_IsBusy());
    
    NVM_UnlockKeyClear();
}

void button1ISR()
{
    DELAY_milliseconds(50);
    if (IO_RB6_GetValue())
    {
        return;
    }
    setBacklight(1);
    backlightCounter = 2;
    switch(mode)
    {
        case MODE_IDLE_1:
        case MODE_IDLE_2:
            mode = MODE_SET_CONT_INTRO;
            break;
            
        case MODE_SET_CONT_INTRO:
        case MODE_SET_CONT:
            mode = MODE_SET_LRLAT_INTRO;
            break;
            
        case MODE_SET_LRLAT_INTRO:
        case MODE_SET_LRLAT:
            mode = MODE_SET_THERMISTORS_INTRO;
            break;
            
        case MODE_SET_THERMISTORS_INTRO:
        case MODE_SET_THERMISTORS:
            mode = MODE_SET_ID1_INTRO;
            break;
            
        case MODE_SET_ID1_INTRO:
        case MODE_SET_ID1:
            if (thermistors)
            {
                mode = MODE_SET_ID2_INTRO;
            }
            else
            {
                mode = MODE_SET_CAL1_INTRO;
            }
            break;
            
        case MODE_SET_ID2_INTRO:
        case MODE_SET_ID2:
            mode = MODE_SET_CAL1_INTRO;
            break;
            
        case MODE_SET_CAL1_INTRO:
        case MODE_SET_CAL1:
            if (thermistors)
            {
                mode = MODE_SET_CAL2_INTRO;
            }
            else
            {
                mode = MODE_IDLE_1;
                saveSettings();
            }
            break;
            
        case MODE_SET_CAL2_INTRO:
        case MODE_SET_CAL2:
            mode = MODE_IDLE_1;
            saveSettings();
            break;
    }
}

void button2ISR()
{
    uint8_t temp;
    DELAY_milliseconds(50);
    if (IO_RB7_GetValue())
    {
        return;
    }
    setBacklight(1);
    backlightCounter = 2;
    switch (mode)
    {
        case MODE_SET_CONT_INTRO:
            mode = MODE_SET_CONT;
            break;
            
        case MODE_SET_CONT:
            LCDCST++;
            break;
            
        case MODE_SET_LRLAT_INTRO:
            mode = MODE_SET_LRLAT;
            break;
            
        case MODE_SET_LRLAT:
            temp = LCDRL & 0x07;
            LCDRL &= 0xf8;
            temp++;
            LCDRL |= temp & 0x07;
            break;
            
        case MODE_SET_THERMISTORS_INTRO:
            mode = MODE_SET_THERMISTORS;
            break;
            
        case MODE_SET_THERMISTORS:
            thermistors ^= 1;
            break;
            
        case MODE_SET_ID1_INTRO:
            mode = MODE_SET_ID1;
            break;
            
        case MODE_SET_ID1:
            // Increment if left button not held down, decrement if it is
            if (IO_RB6_GetValue())
            {
                id1++;
            }
            else
            {
                id1--;
            }
            break;
            
        case MODE_SET_ID2_INTRO:
            mode = MODE_SET_ID2;
            break;
            
        case MODE_SET_ID2:
            // Increment if left button not held down, decrement if it is
            if (IO_RB6_GetValue())
            {
                id2++;
            }
            else
            {
                id2--;
            }
            break;
            
        case MODE_SET_CAL1_INTRO:
            mode = MODE_SET_CAL1;
            break;
            
        case MODE_SET_CAL1:
            // Increment if left button not held down, decrement if it is
            if (IO_RB6_GetValue())
            {
                if (cal1 < CAL_UPPER_LIMIT)
                {
                    cal1++;
                }
                else
                {
                    cal1 = CAL_LOWER_LIMIT;
                }
            }
            else
            {
                if (cal1 > CAL_LOWER_LIMIT)
                {
                    cal1--;
                }
                else
                {
                    cal1 = CAL_UPPER_LIMIT;
                }
            }
            break;
            
        case MODE_SET_CAL2_INTRO:
            mode = MODE_SET_CAL2;
            break;
            
        case MODE_SET_CAL2:
            // Increment if left button not held down, decrement if it is
            if (IO_RB6_GetValue())
            {
                if (cal2 < CAL_UPPER_LIMIT)
                {
                    cal2++;
                }
                else
                {
                    cal2 = CAL_LOWER_LIMIT;
                }
            }
            else
            {
                if (cal2 > CAL_LOWER_LIMIT)
                {
                    cal2--;
                }
                else
                {
                    cal2 = CAL_UPPER_LIMIT;
                }
            }
            break;
    }
}

void calculateCRC()
{
    data0 = txData[0];
    data1 = txData[1];
    data2 = txData[2];
    data3 = txData[3];
    data4 = txData[4];
    data5 = txData[5];
    data6 = txData[6];
    data7 = txData[7];
    bitCounter = 64;
    extraByte = 0;
    asm("banksel _extraByte");
    asm("movlw 0xB3");
asmloop:
    asm("rlf _extraByte & 0x7f, F");
    asm("rlf _data7 & 0x7f, F");
    asm("rlf _data6 & 0x7f, F");
    asm("rlf _data5 & 0x7f, F");
    asm("rlf _data4 & 0x7f, F");
    asm("rlf _data3 & 0x7f, F");
    asm("rlf _data2 & 0x7f, F");
    asm("rlf _data1 & 0x7f, F");
    asm("rlf _data0 & 0x7f, F");
    asm("btfsc STATUS, 0");
    asm("xorwf _data0 & 0x7f, F");
    asm("decfsz _bitCounter & 0x7f, F");
    goto asmloop;
    return;
}

/**
 * Assembly language delay procedure
 * @param delayValue Magic number for delay length
 */
void manchesterDelay(uint8_t delayValue)
{
    delayCounter = delayValue;
    asm("banksel _delayCounter");
    //asm("movlw 0x3D");
    //asm("movwf _delayCounter & 0x7f");
    asm("nop");
    delayLoop:
    asm("decfsz _delayCounter & 0x7f");
    goto delayLoop;
    return;
}

/**
 * Manchester transmission procedure
 * @param data Array of 8 data bytes to transmit
 * @param crc Previously calculated CRC to transmit
 */
void transmitData(uint8_t * data, uint8_t crc)
{
    /*uint8_t txByte;
    for (uint8_t i = 0; i < 11; i++)
    {
        txByte = (i == 0) ? 0 : ((i == 1) ? 9 : ((i == 10) ? crc : data[i - 2]));
        for (uint8_t bitCounter = 0; bitCounter < 8; bitCounter++)
        {
            if ((txByte >> (7 - bitCounter)) & 1)
            {
                // 1 then 0 if bit is 1
                LATCbits.LATC0 = 1;
                manchesterDelay();
                LATCbits.LATC0 = 0;
                manchesterDelay();
            }
            else
            {
                // 0 then 1 if bit is 0
                LATCbits.LATC0 = 0;
                manchesterDelay();
                LATCbits.LATC0 = 1;
                manchesterDelay();
            }
        }
    }
     Above code is too inefficient when compiled so rewritten in assembly
     */
    
    preamble = 0;
    startChar = 0x09;
    data0 = txData[0];
    data1 = txData[1];
    data2 = txData[2];
    data3 = txData[3];
    data4 = txData[4];
    data5 = txData[5];
    data6 = txData[6];
    data7 = txData[7];
    extraByte = crc;
    bitCounter = 88;
    
    asm("banksel _bitCounter");
txLoop:
    asm("rlf _extraByte & 0x7f, F");
    asm("rlf _data7 & 0x7f, F");
    asm("rlf _data6 & 0x7f, F");
    asm("rlf _data5 & 0x7f, F");
    asm("rlf _data4 & 0x7f, F");
    asm("rlf _data3 & 0x7f, F");
    asm("rlf _data2 & 0x7f, F");
    asm("rlf _data1 & 0x7f, F");
    asm("rlf _data0 & 0x7f, F");
    asm("rlf _startChar & 0x7f, F");
    asm("rlf _preamble & 0x7f, F");
    
    if (STATUS & 1)
    {
        // 1 then 0 if bit is 1
        LATCbits.LATC0 = 1;
        manchesterDelay(62);
        LATCbits.LATC0 = 0;
        manchesterDelay(55);
    }
    else
    {
        // 0 then 1 if bit is 0
        LATCbits.LATC0 = 0;
        manchesterDelay(62);
        LATCbits.LATC0 = 1;
        manchesterDelay(55);
    }
    
    asm("banksel _bitCounter");
    asm("decfsz _bitCounter, F");
    goto txLoop;
    
    // Any code in this function below the goto above doesn't get included
    // in the output so calling code should run 'LATCbits.LATC0 = 0;'
    // to avoid radio remaining on
    
    //disassembly line 3710
    return;
}

int main(void)
{
    SYSTEM_Initialize();
    NVM_Initialize();
    // Load the settings from EEPROM
    loadSettings();
    // Configure the two button interrupts
    IO_RB6_SetInterruptHandler(button1ISR);
    IO_RB7_SetInterruptHandler(button2ISR);
    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 

    // Enable the Peripheral Interrupts 
    INTERRUPT_PeripheralInterruptEnable(); 

    // Disable the Peripheral Interrupts 
    //INTERRUPT_PeripheralInterruptDisable(); 

    // To save more power:
    // Enable pullups on all unused IO (only supported on port B on this chip)
    //   Reduces current from 34uA to 27uA
    // Disable brown out detection
    //   Reduces current from 27uA to 21uA
    // Change LCD divider ladder from medium to low power
    //   No significant change and compromises contrast too much
    //   Might work better with smaller LCD
    // Turn off ADC before sleeping
    // Final current: 5-10uA depending on LCDRL setting
    
    
    uint16_t temperature1 = 0;
    uint16_t temperature2 = 0;
    uint16_t temperature1Prev = 0;
    uint16_t temperature2Prev = 0;
    uint16_t temperature1Filt = 0;
    uint16_t temperature2Filt = 0;
    uint16_t temperature1Conv = 0;
    uint16_t temperature2Conv = 0;
    uint8_t temperatureLastTransmitted = 0;
    
    while(1)
    {
        // Turn on ADC
        ADC_Initialize();
        // Turn on thermistor
        IO_RC1_SetHigh();
        // Start conversion
        ADC_SelectChannel(10);
        ADC_StartConversion();
        while (!ADC_IsConversionDone());
        // Process ADC reading to get temperature
        temperature1Prev = temperature1;
        temperature1 = ADC_GetConversionResult();
        if (temperature1Prev == 0)
        {
            temperature1Prev = temperature1;
        }
        temperature1Filt = (temperature1 + temperature1Prev) >> 1;
        temperature1Conv = adcToTemperature(temperature1Filt) + cal1;
        if (thermistors > 0)
        {
            ADC_SelectChannel(8);
            ADC_StartConversion();
            while (!ADC_IsConversionDone());
            // Process ADC reading to get temperature
            temperature2Prev = temperature2;
            temperature2 = ADC_GetConversionResult();
            if (temperature2Prev == 0)
            {
                temperature2Prev = temperature2;
            }
            temperature2Filt = (temperature2 + temperature2Prev) >> 1;
            temperature2Conv = adcToTemperature(temperature2Filt) + cal2;
        }
        // Turn off thermistor
        IO_RC1_SetLow();
        // Turn off ADC channel
        ADCON0 = 0;
        
        // Transmission sequence
        // Only if PIC was woken by WDT so that transmission
        // is not repeated whenever a button is pressed
        if ((STATUS & 0x18) == 0)
        {
            // For one thermistor: Always transmit data for first thermistor
            // For two thermistors: Transmit one per wake and alternate
            if (thermistors == 0 || temperatureLastTransmitted == 1)
            {
                // Generate TX data array for first thermistor
                if (temperature1Filt > VALID_THRESHOLD)
                {
                    // Generate TX data array for first thermistor
                    extraByte = 0;
                    bitCounter = 64;
                    txData[0] = id1;
                    txData[1] = 'C';
                    txData[2] = (uint8_t)(temperature1Conv / 10);
                    txData[3] = (uint8_t)(temperature1Conv % 10);
                    txData[7] = txData[2] + txData[3];
                    calculateCRC();
                    crc = data0;
                    transmitData(txData, crc);
                    LATCbits.LATC0 = 0;
                    temperatureLastTransmitted = 0;
                }
            }
            else
            {
                // Generate TX data array for second thermistor
                if (thermistors > 0 && temperature2Filt > VALID_THRESHOLD)
                {
                    extraByte = 0;
                    bitCounter = 64;
                    txData[0] = id2;
                    txData[1] = 'C';
                    txData[2] = (uint8_t)(temperature2Conv / 10);
                    txData[3] = (uint8_t)(temperature2Conv % 10);
                    txData[7] = txData[2] + txData[3];
                    calculateCRC();
                    crc = data0;
                    transmitData(txData, crc);
                    LATCbits.LATC0 = 0;
                    temperatureLastTransmitted = 1;
                }
            }
        }
        
        // Display temperature
        switch(mode)
        {
            case MODE_IDLE_1:
                displayTemperature(temperature1Conv, 0, temperature1Filt > VALID_THRESHOLD);
                // Turn on thermistor number indicator only if multiple
                // thermistors are enabled because it's redundant otherwise
                LCDDATA4bits.SEG9COM1 = (thermistors > 0) ? 1 : 0;
                LCDDATA10bits.SEG8COM3 = 0;
                // Turn on degC indicator on LCD
                LCDDATA9bits.SEG0COM3 = 1;
                if (thermistors > 0)
                {
                    mode = MODE_IDLE_2;
                }
                break;
            
            case MODE_IDLE_2:
                displayTemperature(temperature2Conv, 0, temperature2Filt > VALID_THRESHOLD);
                //displayTemperature(crc, 0, temperature2 > 0);
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 1;
                // Turn on degC indicator on LCD
                LCDDATA9bits.SEG0COM3 = 1;
                mode = MODE_IDLE_1;
                break;
            
            case MODE_SET_CONT_INTRO:
                displayDigitSegments(0b0111001, 0b0111111, 0b0110111,  0, 0, 0);
                // Turn off degC indicator on LCD
                LCDDATA9bits.SEG0COM3 = 0;
                // Turn off thermistor indicators
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 0;
                break;
            
            case MODE_SET_CONT:
                displayNumber(LCDCST, 0, 0, 1);
                break;
            
            case MODE_SET_LRLAT_INTRO:
                displayDigitSegments(0b0111000, 0b1011111, 0b1111000,  0, 0, 0);
                break;
            
            case MODE_SET_LRLAT:
                displayNumber(LCDRL & 7, 0, 0, 1);
                break;
                
            case MODE_SET_THERMISTORS_INTRO:
            case MODE_SET_THERMISTORS:
                displayDigitSegments(0b1111000, 0b1110100, 0b0000000,  0, 0, 0);
                LCDDATA4bits.SEG9COM1 = 1;
                LCDDATA10bits.SEG8COM3 = thermistors;
                break;
                
            case MODE_SET_ID1_INTRO:
                displayDigitSegments(0b0110000, 0b1011110, 0b0000000,  0, 0, 0);
                LCDDATA4bits.SEG9COM1 = 1;
                LCDDATA10bits.SEG8COM3 = 0;
                break;
                
            case MODE_SET_ID1:
                displayNumber(id1, 0, 0, 1);
                LCDDATA4bits.SEG9COM1 = 1;
                LCDDATA10bits.SEG8COM3 = 0;
                break;
                
            case MODE_SET_ID2_INTRO:
                displayDigitSegments(0b0110000, 0b1011110, 0b0000000,  0, 0, 0);
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 1;
                break;
                
            case MODE_SET_ID2:
                displayNumber(id2, 0, 0, 1);
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 1;
                break;
                
            case MODE_SET_CAL1_INTRO:
                displayDigitSegments(0b0111001, 0b1110111, 0b0111000,  0, 0, 0);
                LCDDATA4bits.SEG9COM1 = 1;
                LCDDATA10bits.SEG8COM3 = 0;
                break;
                
            case MODE_SET_CAL1:
                displayNumber(cal1, 0, 1, 1);
                LCDDATA4bits.SEG9COM1 = 1;
                LCDDATA10bits.SEG8COM3 = 0;
                // Turn on degC indicator on LCD
                LCDDATA9bits.SEG0COM3 = 1;
                break;
                
            case MODE_SET_CAL2_INTRO:
                displayDigitSegments(0b0111001, 0b1110111, 0b0111000,  0, 0, 0);
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 1;
                break;
                
            case MODE_SET_CAL2:
                displayNumber(cal2, 0, 1, 1);
                LCDDATA4bits.SEG9COM1 = 0;
                LCDDATA10bits.SEG8COM3 = 1;
                // Turn on degC indicator on LCD
                LCDDATA9bits.SEG0COM3 = 1;
                break;
        }
        // Watchdog timer is configured in MCC and used to exit sleep
        SLEEP();
        if (backlightCounter > 0)
            backlightCounter--;
        else
            setBacklight(0);
    }    
}