/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F25K83
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
        MPLAB 	          :  MPLAB X 5.45	
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set LED_RedStatus aliases
#define LED_RedStatus_TRIS                 TRISBbits.TRISB0
#define LED_RedStatus_LAT                  LATBbits.LATB0
#define LED_RedStatus_PORT                 PORTBbits.RB0
#define LED_RedStatus_WPU                  WPUBbits.WPUB0
#define LED_RedStatus_OD                   ODCONBbits.ODCB0
#define LED_RedStatus_ANS                  ANSELBbits.ANSELB0
#define LED_RedStatus_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define LED_RedStatus_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define LED_RedStatus_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define LED_RedStatus_GetValue()           PORTBbits.RB0
#define LED_RedStatus_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define LED_RedStatus_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define LED_RedStatus_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define LED_RedStatus_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define LED_RedStatus_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define LED_RedStatus_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define LED_RedStatus_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define LED_RedStatus_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set RB1 procedures
#define RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define RB1_GetValue()              PORTBbits.RB1
#define RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define RB1_SetPullup()             do { WPUBbits.WPUB1 = 1; } while(0)
#define RB1_ResetPullup()           do { WPUBbits.WPUB1 = 0; } while(0)
#define RB1_SetAnalogMode()         do { ANSELBbits.ANSELB1 = 1; } while(0)
#define RB1_SetDigitalMode()        do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set RB2 procedures
#define RB2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define RB2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define RB2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define RB2_GetValue()              PORTBbits.RB2
#define RB2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define RB2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define RB2_SetPullup()             do { WPUBbits.WPUB2 = 1; } while(0)
#define RB2_ResetPullup()           do { WPUBbits.WPUB2 = 0; } while(0)
#define RB2_SetAnalogMode()         do { ANSELBbits.ANSELB2 = 1; } while(0)
#define RB2_SetDigitalMode()        do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set IO_RC1 aliases
#define IO_RC1_TRIS                 TRISCbits.TRISC1
#define IO_RC1_LAT                  LATCbits.LATC1
#define IO_RC1_PORT                 PORTCbits.RC1
#define IO_RC1_WPU                  WPUCbits.WPUC1
#define IO_RC1_OD                   ODCONCbits.ODCC1
#define IO_RC1_ANS                  ANSELCbits.ANSELC1
#define IO_RC1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define IO_RC1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define IO_RC1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define IO_RC1_GetValue()           PORTCbits.RC1
#define IO_RC1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define IO_RC1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define IO_RC1_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define IO_RC1_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define IO_RC1_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define IO_RC1_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define IO_RC1_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define IO_RC1_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set IO_RC2 aliases
#define IO_RC2_TRIS                 TRISCbits.TRISC2
#define IO_RC2_LAT                  LATCbits.LATC2
#define IO_RC2_PORT                 PORTCbits.RC2
#define IO_RC2_WPU                  WPUCbits.WPUC2
#define IO_RC2_OD                   ODCONCbits.ODCC2
#define IO_RC2_ANS                  ANSELCbits.ANSELC2
#define IO_RC2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define IO_RC2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define IO_RC2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define IO_RC2_GetValue()           PORTCbits.RC2
#define IO_RC2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define IO_RC2_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define IO_RC2_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define IO_RC2_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define IO_RC2_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define IO_RC2_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define IO_RC2_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set LED_GreenStatus aliases
#define LED_GreenStatus_TRIS                 TRISCbits.TRISC4
#define LED_GreenStatus_LAT                  LATCbits.LATC4
#define LED_GreenStatus_PORT                 PORTCbits.RC4
#define LED_GreenStatus_WPU                  WPUCbits.WPUC4
#define LED_GreenStatus_OD                   ODCONCbits.ODCC4
#define LED_GreenStatus_ANS                  ANSELCbits.ANSELC4
#define LED_GreenStatus_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define LED_GreenStatus_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define LED_GreenStatus_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define LED_GreenStatus_GetValue()           PORTCbits.RC4
#define LED_GreenStatus_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define LED_GreenStatus_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define LED_GreenStatus_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define LED_GreenStatus_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define LED_GreenStatus_SetPushPull()        do { ODCONCbits.ODCC4 = 0; } while(0)
#define LED_GreenStatus_SetOpenDrain()       do { ODCONCbits.ODCC4 = 1; } while(0)
#define LED_GreenStatus_SetAnalogMode()      do { ANSELCbits.ANSELC4 = 1; } while(0)
#define LED_GreenStatus_SetDigitalMode()     do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set LED_BlueStatus aliases
#define LED_BlueStatus_TRIS                 TRISCbits.TRISC5
#define LED_BlueStatus_LAT                  LATCbits.LATC5
#define LED_BlueStatus_PORT                 PORTCbits.RC5
#define LED_BlueStatus_WPU                  WPUCbits.WPUC5
#define LED_BlueStatus_OD                   ODCONCbits.ODCC5
#define LED_BlueStatus_ANS                  ANSELCbits.ANSELC5
#define LED_BlueStatus_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define LED_BlueStatus_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define LED_BlueStatus_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define LED_BlueStatus_GetValue()           PORTCbits.RC5
#define LED_BlueStatus_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define LED_BlueStatus_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define LED_BlueStatus_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define LED_BlueStatus_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define LED_BlueStatus_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define LED_BlueStatus_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define LED_BlueStatus_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define LED_BlueStatus_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/