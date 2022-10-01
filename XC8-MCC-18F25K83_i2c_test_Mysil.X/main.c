/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F25K83
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"

uint8_t i2c_send_data[6] = {0x00, 0x00, 0x0A, 0x0B, 0x0C, 0x0D};
uint8_t i2c_read_data[4];
uint8_t i2c_memory_index[2];
uint8_t i2c_memory_config[2];
#define EEPROM_DEVICE_ADDRESS 0x50		/* 0xA0 I2C Addressing Change Undone, Mysil */
/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();
    LED_GreenStatus_SetHigh();
    __delay_ms(1000);

    while (1)
    {
        I2C2_WriteNBytes(EEPROM_DEVICE_ADDRESS, i2c_send_data, sizeof(i2c_send_data));
        __delay_ms(1);
        I2C2_WriteNBytes(EEPROM_DEVICE_ADDRESS, i2c_send_data, 2);
        I2C2_ReadNBytes(EEPROM_DEVICE_ADDRESS, i2c_read_data, sizeof(i2c_read_data));
        __delay_ms(1);
        
		/* 
		 * Try to read from EEPROM Configuration WPR and HAR registers. 
		 */
//		i2c_memory_index[0] = 0x80;
//		i2c_memory_index[1] = 0x00;
//        I2C2_WriteNBytes(EEPROM_DEVICE_ADDRESS, i2c_memory_index, 2);
//        I2C2_ReadNBytes(EEPROM_DEVICE_ADDRESS, i2c_memory_config, 2);
//        __delay_ms(1);
		/* 
		 * Try to read from EEPROM Configuration WPR and HAR registers. 
		 */
//		i2c_memory_index[0] = 0xFF;
//		i2c_memory_index[1] = 0xFE;
//        I2C2_WriteNBytes(EEPROM_DEVICE_ADDRESS, i2c_memory_index, 2);
//        I2C2_ReadNBytes(EEPROM_DEVICE_ADDRESS, i2c_memory_config, sizeof(i2c_memory_config));
//        __delay_ms(1);
        LED_GreenStatus_SetLow();
//        if(i2c_memory_config[0] == 0x04)
        if(i2c_read_data[0] == 0x04)
            LED_RedStatus_SetLow();
        else 
            LED_RedStatus_SetHigh();
    }
}
/**
 End of File
*/