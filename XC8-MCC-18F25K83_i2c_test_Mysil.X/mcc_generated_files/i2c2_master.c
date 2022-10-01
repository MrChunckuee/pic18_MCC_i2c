/**
  I2C2 Modified Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c2_master.c

  @Summary
    This is the generated driver implementation file for the I2C2 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for I2C2.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F25K83
        Driver Version    :  1.0.2
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above or later
        MPLAB             :  MPLAB X 5.45
 
  @Revision
  	Clear RSEN bit to avoid corrupted STOP condition signal. Mysil.
    Return I2C1_Status.Error to report eventual transfer problem. Mysil. 
    Do Not clear Error variable before transfer is completed.
    Clear the Error variable when setting up New transfer.
    Initialize I2C2CON1  ACKCNT to Not Acknowledge, NACK when end of Read transfer.
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

#include <xc.h>
#include "i2c2_master.h"
#include "pin_manager.h"

// I2C2 STATES
typedef enum {
    I2C2_IDLE = 0,
    I2C2_SEND_ADR_READ,
    I2C2_SEND_ADR_WRITE,
    I2C2_TX,
    I2C2_RX,
    I2C2_TX_EMPTY,
    I2C2_RX_EMPTY,        
    I2C2_SEND_RESTART_READ,
    I2C2_SEND_RESTART_WRITE,
    I2C2_SEND_RESTART,
    I2C2_SEND_STOP,
    I2C2_RX_ACK,
    I2C2_TX_ACK,
    I2C2_RX_NACK_STOP,
    I2C2_RX_NACK_RESTART,
    I2C2_RESET,
    I2C2_ADDRESS_NACK,
    I2C2_BUS_COLLISION,
    I2C2_BUS_ERROR
} i2c2_fsm_states_t;

// I2C2 Event callBack List
typedef enum {
    I2C2_DATA_COMPLETE = 0,
    I2C2_WRITE_COLLISION,
    I2C2_ADDR_NACK,
    I2C2_DATA_NACK,
    I2C2_TIMEOUT,
    I2C2_NULL
} i2c2_callbackIndex_t;

// I2C2 Status Structure
typedef struct
{
    i2c2_callback_t callbackTable[6];
    void *callbackPayload[6];           //  each callBack can have a payload
    uint16_t time_out;                  // I2C2 Timeout Counter between I2C2 Events.
    uint16_t time_out_value;            // Reload value for the timeouts
    i2c2_address_t address;             // The I2C2 Address
    uint8_t *data_ptr;                  // pointer to a data buffer
    size_t data_length;                 // Bytes in the data buffer
    i2c2_fsm_states_t state;            // Driver State
    i2c2_error_t error;
    unsigned addressNackCheck:2;
    unsigned busy:1;
    unsigned inUse:1;
    unsigned bufferFree:1;

} i2c2_status_t;

static void I2C2_SetCallback(i2c2_callbackIndex_t idx, i2c2_callback_t cb, void *ptr);
static void I2C2_Poller(void);
static inline void I2C2_ClearInterruptFlags(void);
static inline void I2C2_MasterFsm(void);

/* I2C2 interfaces */
static inline bool I2C2_MasterOpen(void);
static inline void I2C2_MasterClose(void);    
static inline uint8_t I2C2_MasterGetRxData(void);
static inline void I2C2_MasterSendTxData(uint8_t data);
static inline void I2C2_MasterSetCounter(uint8_t counter);
static inline uint8_t I2C2_MasterGetCounter();
static inline void I2C2_MasterResetBus(void);
static inline void I2C2_MasterEnableRestart(void);
static inline void I2C2_MasterDisableRestart(void);
static inline void I2C2_MasterStop(void);
static inline bool I2C2_MasterIsNack(void);
static inline void I2C2_MasterSendAck(void);
static inline void I2C2_MasterSendNack(void);
static inline void I2C2_MasterClearBusCollision(void);
static inline bool I2C2_MasterIsRxBufFull(void);
static inline bool I2C2_MasterIsTxBufEmpty(void);
static inline bool I2C2_MasterIsStopFlagSet(void);
static inline bool I2C2_MasterIsCountFlagSet(void);
static inline bool I2C2_MasterIsNackFlagSet(void);
static inline void I2C2_MasterClearStopFlag(void);
static inline void I2C2_MasterClearCountFlag(void);
static inline void I2C2_MasterClearNackFlag(void);

/* Interrupt interfaces */
static inline void I2C2_MasterEnableIrq(void);
static inline bool I2C2_MasterIsIrqEnabled(void);
static inline void I2C2_MasterDisableIrq(void);
static inline void I2C2_MasterClearIrq(void);
static inline void I2C2_MasterWaitForEvent(void);

static i2c2_fsm_states_t I2C2_DO_IDLE(void);
static i2c2_fsm_states_t I2C2_DO_SEND_ADR_READ(void);
static i2c2_fsm_states_t I2C2_DO_SEND_ADR_WRITE(void);
static i2c2_fsm_states_t I2C2_DO_TX(void);
static i2c2_fsm_states_t I2C2_DO_RX(void);
static i2c2_fsm_states_t I2C2_DO_TX_EMPTY(void);
static i2c2_fsm_states_t I2C2_DO_RX_EMPTY(void);
static i2c2_fsm_states_t I2C2_DO_SEND_RESTART_READ(void);
static i2c2_fsm_states_t I2C2_DO_SEND_RESTART_WRITE(void);
static i2c2_fsm_states_t I2C2_DO_SEND_RESTART(void);
static i2c2_fsm_states_t I2C2_DO_SEND_STOP(void);
static i2c2_fsm_states_t I2C2_DO_RX_ACK(void);
static i2c2_fsm_states_t I2C2_DO_TX_ACK(void);
static i2c2_fsm_states_t I2C2_DO_RX_NACK_STOP(void);
static i2c2_fsm_states_t I2C2_DO_RX_NACK_RESTART(void);
static i2c2_fsm_states_t I2C2_DO_RESET(void);
static i2c2_fsm_states_t I2C2_DO_ADDRESS_NACK(void);
static i2c2_fsm_states_t I2C2_DO_BUS_COLLISION(void);
static i2c2_fsm_states_t I2C2_DO_BUS_ERROR(void);

typedef i2c2_fsm_states_t (*i2c2FsmHandler)(void);
const i2c2FsmHandler i2c2_fsmStateTable[] = {
    I2C2_DO_IDLE,
    I2C2_DO_SEND_ADR_READ,
    I2C2_DO_SEND_ADR_WRITE,
    I2C2_DO_TX,
    I2C2_DO_RX,
    I2C2_DO_TX_EMPTY,
    I2C2_DO_RX_EMPTY,
    I2C2_DO_SEND_RESTART_READ,
    I2C2_DO_SEND_RESTART_WRITE,
    I2C2_DO_SEND_RESTART,
    I2C2_DO_SEND_STOP,
    I2C2_DO_RX_ACK,
    I2C2_DO_TX_ACK,
    I2C2_DO_RX_NACK_STOP,
    I2C2_DO_RX_NACK_RESTART,
    I2C2_DO_RESET,
    I2C2_DO_ADDRESS_NACK,
    I2C2_DO_BUS_COLLISION,
    I2C2_DO_BUS_ERROR
};

i2c2_status_t I2C2_Status = {0};

void I2C2_Initialize()
{   IO_RC2_SetHigh();					/* Diagnostic timing, Mysil. */
    //EN disabled; RSEN disabled; S waits for BFRE=1 to start; CSTR Enable clocking; MODE 7-bit address; 
    I2C2CON0 = 0x24;
    //ACKCNT Acknowledge; ACKDT Acknowledge; ACKSTAT ACK received; ACKT 0; RXO 0; TXU 0; CSD Clock Stretching disabled; 
    I2C2CON1 = 0x01;
    I2C2CON1 |= _I2C2CON1_ACKCNT_MASK;	/* NACK when I2C2CNT have been reached. */
	I2C2CON1bits.CSD = 0;				/* Why have Clock Stretching been Disabled? It would just cause trouble. */
	
    //ACNT disabled; GCEN disabled; FME disabled; ABD disabled; SDAHT 300 ns hold time; BFRET 8 I2C Clock pulses; 
    I2C2CON2 = 0x10;
    //CLK MFINTOSC; 
    I2C2CLK = 0x03;
    //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
    I2C2PIR = 0x00;
    //CNTIE disabled; ACKTIE disabled; WRIE disabled; ADRIE disabled; PCIE disabled; RSCIE disabled; SCIE disabled; 
    I2C2PIE = 0x00;
    //BTOIF No bus timout; BCLIF No bus collision detected; NACKIF No NACK/Error detected; BTOIE disabled; BCLIE disabled; NACKIE disabled; 
    I2C2ERR = 0x00;
    //Count register 
    I2C2CNT = 0xFF;
    IO_RC2_SetLow();					/* Diagnostic timing, Mysil. */
    return;
}

i2c2_error_t I2C2_Open(i2c2_address_t address)
{   IO_RC2_SetHigh();					/* Diagnostic timing, Mysil. */
    i2c2_error_t returnValue = I2C2_BUSY;
    
    if(!I2C2_Status.inUse)
    {
        I2C2_Status.address = address;
        I2C2_Status.busy = 0;
        I2C2_Status.inUse = 1;
        I2C2_Status.addressNackCheck = 0;
        I2C2_Status.error = I2C2_NOERR;		/* Reset Error flag from previous transfer. Inserted, Mysil. */
        I2C2_Status.state = I2C2_RESET;
        I2C2_Status.time_out_value = 500; // MCC should determine a reasonable starting value here.
        I2C2_Status.bufferFree = 1;

        // set all the call backs to a default of sending stop
        I2C2_Status.callbackTable[I2C2_DATA_COMPLETE]=I2C2_CallbackReturnStop;
        I2C2_Status.callbackPayload[I2C2_DATA_COMPLETE] = NULL;
        I2C2_Status.callbackTable[I2C2_WRITE_COLLISION]=I2C2_CallbackReturnStop;
        I2C2_Status.callbackPayload[I2C2_WRITE_COLLISION] = NULL;
        I2C2_Status.callbackTable[I2C2_ADDR_NACK]=I2C2_CallbackReturnStop;
        I2C2_Status.callbackPayload[I2C2_ADDR_NACK] = NULL;
        I2C2_Status.callbackTable[I2C2_DATA_NACK]=I2C2_CallbackReturnStop;
        I2C2_Status.callbackPayload[I2C2_DATA_NACK] = NULL;
        I2C2_Status.callbackTable[I2C2_TIMEOUT]=I2C2_CallbackReturnReset;
        I2C2_Status.callbackPayload[I2C2_TIMEOUT] = NULL;
        
        I2C2_MasterClearIrq();
        I2C2_MasterOpen();
        returnValue = I2C2_NOERR;
    }
    return returnValue;
}

i2c2_error_t I2C2_Close(void)
{
    i2c2_error_t returnValue = I2C2_BUSY;
    if(!I2C2_Status.busy)
    {
        I2C2_Status.inUse = 0;
        I2C2_Status.address = 0xff;
        I2C2_MasterClearIrq();
        I2C2_MasterDisableIrq();
        I2C2_MasterClose();
        returnValue = I2C2_Status.error;
    }
    return returnValue;
}

i2c2_error_t I2C2_MasterOperation(bool read)
{
    i2c2_error_t returnValue = I2C2_BUSY;
    if(!I2C2_Status.busy)
    {
        I2C2_Status.busy = true;
        returnValue = I2C2_NOERR;
        I2C2_MasterSetCounter((uint8_t) I2C2_Status.data_length);

        if(read)
        {
            I2C2_Status.state = I2C2_RX;
            I2C2_DO_SEND_ADR_READ();
        }
        else
        {
            I2C2_Status.state = I2C2_TX;
            I2C2_DO_SEND_ADR_WRITE();
        }
        IO_RC2_SetLow();				/* Diagnostic timing, Mysil. */
        I2C2_Poller();
    }
    return returnValue;
}

i2c2_error_t I2C2_MasterRead(void)
{
    return I2C2_MasterOperation(true);
}

i2c2_error_t I2C2_MasterWrite(void)
{
    return I2C2_MasterOperation(false);
}

void I2C2_SetTimeOut(uint8_t timeOutValue)
{
    I2C2_MasterDisableIrq();
    I2C2_Status.time_out_value = timeOutValue;
    I2C2_MasterEnableIrq();
}

void I2C2_SetBuffer(void *buffer, size_t bufferSize)
{
    if(I2C2_Status.bufferFree)
    {
        I2C2_Status.data_ptr = buffer;
        I2C2_Status.data_length = bufferSize;
        I2C2_Status.bufferFree = false;
    }
}

void I2C2_SetDataCompleteCallback(i2c2_callback_t cb, void *ptr)
{
    I2C2_SetCallback(I2C2_DATA_COMPLETE, cb, ptr);
}

void I2C2_SetWriteCollisionCallback(i2c2_callback_t cb, void *ptr)
{
    I2C2_SetCallback(I2C2_WRITE_COLLISION, cb, ptr);
}

void I2C2_SetAddressNackCallback(i2c2_callback_t cb, void *ptr)
{
    I2C2_SetCallback(I2C2_ADDR_NACK, cb, ptr);
}

void I2C2_SetDataNackCallback(i2c2_callback_t cb, void *ptr)
{
    I2C2_SetCallback(I2C2_DATA_NACK, cb, ptr);
}

void I2C2_SetTimeoutCallback(i2c2_callback_t cb, void *ptr)
{
    I2C2_SetCallback(I2C2_TIMEOUT, cb, ptr);
}

static void I2C2_SetCallback(i2c2_callbackIndex_t idx, i2c2_callback_t cb, void *ptr)
{
    if(cb)
    {
        I2C2_Status.callbackTable[idx] = cb;
        I2C2_Status.callbackPayload[idx] = ptr;
    }
    else
    {
        I2C2_Status.callbackTable[idx] = I2C2_CallbackReturnStop;
        I2C2_Status.callbackPayload[idx] = NULL;
    }
}

static void I2C2_Poller(void)
{
    while(I2C2_Status.busy)
    {
        I2C2_MasterWaitForEvent();
		IO_RC2_SetHigh();
        I2C2_MasterFsm();
		IO_RC2_SetLow();
    }
    Nop();			/* Nop Instruction, to be able to place a breakpoint. */
}
static inline void I2C2_MasterFsm(void)
{
    I2C2_ClearInterruptFlags();

    if(I2C2_Status.addressNackCheck && I2C2_MasterIsNack())
    {
        I2C2_Status.state = I2C2_ADDRESS_NACK;
    }
    I2C2_Status.state = i2c2_fsmStateTable[I2C2_Status.state]();
}

static inline void I2C2_ClearInterruptFlags(void)
{
    if(I2C2_MasterIsCountFlagSet())
    {
        I2C2_MasterClearCountFlag();
    }
    else if(I2C2_MasterIsStopFlagSet())
    {
        I2C2_MasterClearStopFlag();
		I2C2_Status.busy = 0;		/* No longer busy, Mysil */
    }
    else if(I2C2_MasterIsNackFlagSet())
    {
        I2C2_MasterClearNackFlag();
    }
}

static i2c2_fsm_states_t I2C2_DO_IDLE(void)
{
    I2C2_Status.busy = false;
/*    I2C1_Status.error = I2C_NOERR;	//	Do Not hide information about eventual failed transfer. Mysil. */
    return I2C2_RESET;
}

static i2c2_fsm_states_t I2C2_DO_SEND_ADR_READ(void)
{
    I2C2_Status.addressNackCheck = 2;
    if(I2C2_Status.data_length ==  1)
    {
        I2C2_DO_RX_EMPTY();
    }
    I2C2_MasterSendTxData((uint8_t) (I2C2_Status.address << 1 | 1));
    return I2C2_RX;
}

static i2c2_fsm_states_t I2C2_DO_SEND_ADR_WRITE(void)
{
    I2C2_Status.addressNackCheck = 2;
    I2C2_MasterSendTxData((uint8_t) (I2C2_Status.address << 1));
    return I2C2_TX;
}

static i2c2_fsm_states_t I2C2_DO_TX(void)
{
    if(I2C2_MasterIsNack())
    {
        switch(I2C2_Status.callbackTable[I2C2_DATA_NACK](I2C2_Status.callbackPayload[I2C2_DATA_NACK]))
        {
            case I2C2_RESTART_READ:
                return I2C2_DO_SEND_RESTART_READ();
            case I2C2_RESTART_WRITE:
                  return I2C2_DO_SEND_RESTART_WRITE();
            default:
            case I2C2_CONTINUE:
            case I2C2_STOP:
				I2C2_Status.error = I2C2_FAIL;	// I2C2_ERROR_DATA_NACK;  	/* Not Acknowledge is Failed transfer, Mysil */
				I2C2_MasterDisableRestart();	/* Make sure to clear RSEN bit. */
                return I2C2_IDLE;
        }
    }
    else if(I2C2_MasterIsTxBufEmpty())
    {
        if(I2C2_Status.addressNackCheck)
        {
            I2C2_Status.addressNackCheck--;
        }
        uint8_t dataTx = *I2C2_Status.data_ptr++;
        i2c2_fsm_states_t retFsmState = (--I2C2_Status.data_length)?I2C2_TX:I2C2_DO_TX_EMPTY();
        I2C2_MasterSendTxData(dataTx);
        return retFsmState;
    }
    else
    {
        return I2C2_TX;
    }
}

static i2c2_fsm_states_t I2C2_DO_RX(void)
{
    if(!I2C2_MasterIsRxBufFull())
    {
        return I2C2_RX;
    }
    if(I2C2_Status.addressNackCheck)
    {
        I2C2_Status.addressNackCheck--;
    }

    if(--I2C2_Status.data_length)
    {
        *I2C2_Status.data_ptr++ = I2C2_MasterGetRxData();
        return I2C2_RX;
    }
    else
    {
        i2c2_fsm_states_t retFsmState = I2C2_DO_RX_EMPTY();
        *I2C2_Status.data_ptr++ = I2C2_MasterGetRxData();
        return retFsmState;
    }
}

static i2c2_fsm_states_t I2C2_DO_TX_EMPTY(void)
{
    I2C2_Status.bufferFree = true;
    switch(I2C2_Status.callbackTable[I2C2_DATA_COMPLETE](I2C2_Status.callbackPayload[I2C2_DATA_COMPLETE]))
    {
        case I2C2_RESTART_READ:
            I2C2_MasterEnableRestart();
            return I2C2_SEND_RESTART_READ;
        case I2C2_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C2_MasterSetCounter((uint8_t) I2C2_Status.data_length + 1);
            return I2C2_TX;
        default:
        case I2C2_STOP:
            I2C2_MasterDisableRestart();
            return I2C2_SEND_STOP;
    }
}

static i2c2_fsm_states_t I2C2_DO_RX_EMPTY(void)
{
    I2C2_Status.bufferFree = true;
    switch(I2C2_Status.callbackTable[I2C2_DATA_COMPLETE](I2C2_Status.callbackPayload[I2C2_DATA_COMPLETE]))
    {
        case I2C2_RESTART_WRITE:
            I2C2_MasterEnableRestart();
            return I2C2_SEND_RESTART_WRITE;
        case I2C2_RESTART_READ:
            I2C2_MasterEnableRestart();
            return I2C2_SEND_RESTART_READ;
        case I2C2_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C2_MasterSetCounter((uint8_t) (I2C2_Status.data_length + 1));
            return I2C2_RX;
        default:
        case I2C2_STOP:
            if(I2C2_Status.state != I2C2_SEND_RESTART_READ)
            {
                I2C2_MasterDisableRestart();
            }
            return I2C2_IDLE;			/* Modified, Mysil. */
//            return I2C2_RESET;
    }
}

static i2c2_fsm_states_t I2C2_DO_SEND_RESTART_READ(void)
{
    I2C2_MasterSetCounter((uint8_t) I2C2_Status.data_length);
    I2C2PIEbits.RSCIE = 1;				/* Inserted, Mysil. */
    return I2C2_DO_SEND_ADR_READ();
}

static i2c2_fsm_states_t I2C2_DO_SEND_RESTART_WRITE(void)
{   __debug_break();					/* Function very seldom used, verify correct function. Mysil. */
    return I2C2_SEND_ADR_WRITE;
}


static i2c2_fsm_states_t I2C2_DO_SEND_RESTART(void)
{
    return I2C2_SEND_ADR_READ;
}

static i2c2_fsm_states_t I2C2_DO_SEND_STOP(void)
{
    I2C2_MasterStop();
    if(I2C2_MasterGetCounter())
    {
        I2C2_MasterSetCounter(0);
        I2C2_MasterSendTxData(0);
    }
    return I2C2_IDLE;
}

static i2c2_fsm_states_t I2C2_DO_RX_ACK(void)
{
    I2C2_MasterSendAck();
    return I2C2_RX;
}

static i2c2_fsm_states_t I2C2_DO_TX_ACK(void)
{
    I2C2_MasterSendAck();
    return I2C2_TX;
}

static i2c2_fsm_states_t I2C2_DO_RX_NACK_STOP(void)
{
    I2C2_MasterSendNack();
    I2C2_MasterStop();
    return I2C2_DO_IDLE();
}

static i2c2_fsm_states_t I2C2_DO_RX_NACK_RESTART(void)
{
    I2C2_MasterSendNack();
    return I2C2_SEND_RESTART;
}

static i2c2_fsm_states_t I2C2_DO_RESET(void)
{
    I2C2_MasterResetBus();
    I2C2_Status.busy = false;
    I2C2_Status.error = I2C2_NOERR;
    return I2C2_RESET;
}
static i2c2_fsm_states_t I2C2_DO_ADDRESS_NACK(void)
{
    I2C2_Status.addressNackCheck = 0;
    I2C2_Status.error = I2C2_FAIL;
    I2C2_Status.busy = false;
    switch(I2C2_Status.callbackTable[I2C2_ADDR_NACK](I2C2_Status.callbackPayload[I2C2_ADDR_NACK]))
    {
        case I2C2_RESTART_READ:
            return I2C2_DO_SEND_RESTART_READ();				/* Inserted, Mysil. */
        case I2C2_RESTART_WRITE:
            return I2C2_DO_SEND_RESTART_WRITE();			/* Modified, Mysil. */
        default:
            return I2C2_RESET;
    }
}

static i2c2_fsm_states_t I2C2_DO_BUS_COLLISION(void)
{
    // Clear bus collision status flag
    I2C2_MasterClearIrq();

    I2C2_Status.error = I2C2_FAIL;
    switch (I2C2_Status.callbackTable[I2C2_WRITE_COLLISION](I2C2_Status.callbackPayload[I2C2_WRITE_COLLISION])) {
    case I2C2_RESTART_READ:
        return I2C2_DO_SEND_RESTART_READ();
    case I2C2_RESTART_WRITE:
        return I2C2_DO_SEND_RESTART_WRITE();
    default:
        return I2C2_DO_RESET();
    }
}

static i2c2_fsm_states_t I2C2_DO_BUS_ERROR(void)
{
    I2C2_MasterResetBus();
    I2C2_Status.busy  = false;
    I2C2_Status.error = I2C2_FAIL;
    return I2C2_RESET;
}

void I2C2_BusCollisionIsr(void)
{
    I2C2_MasterClearBusCollision();
    I2C2_Status.state = I2C2_RESET;
}

i2c2_operations_t I2C2_CallbackReturnStop(void *funPtr)
{
    return I2C2_STOP;
}

i2c2_operations_t I2C2_CallbackReturnReset(void *funPtr)
{
    return I2C2_RESET_LINK;
}

i2c2_operations_t I2C2_CallbackRestartWrite(void *funPtr)
{
    return I2C2_RESTART_WRITE;
}

i2c2_operations_t I2C2_CallbackRestartRead(void *funPtr)
{
    return I2C2_RESTART_READ;
}



/* I2C2 Register Level interfaces */
static inline bool I2C2_MasterOpen(void)
{
    if(!I2C2CON0bits.EN)
    {
        //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
        I2C2PIR = 0x00;
        //CNTIE disabled; ACKTIE disabled; WRIE disabled; ADRIE disabled; PCIE disabled; RSCIE disabled; SCIE disabled; 
        I2C2PIE = 0x00;
        //BTOIF No bus timout; BCLIF No bus collision detected; NACKIF No NACK/Error detected; BTOIE disabled; BCLIE disabled; NACKIE disabled; 
        I2C2ERR = 0x00;
        //Count register 
        I2C2CNT = 0xFF;
        //Enable I2C2
        I2C2CON0bits.EN = 1;
        return true;
    }
    return false;
}

static inline void I2C2_MasterClose(void)
{
    //Disable I2C2
    I2C2CON0bits.EN = 0;
    //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
    I2C2PIR = 0x00;
    //Set Clear Buffer Flag
    I2C2STAT1bits.CLRBF = 1;
}

static inline uint8_t I2C2_MasterGetRxData(void)
{
    return I2C2RXB;
}

static inline void I2C2_MasterSendTxData(uint8_t data)
{
    I2C2TXB  = data;
}

static inline uint8_t I2C2_MasterGetCounter()
{
    return I2C2CNT;
}

static inline void I2C2_MasterSetCounter(uint8_t counter)
{
    I2C2CNT = counter;
}

static inline void I2C2_MasterResetBus(void)
{
    //Disable I2C2
    I2C2CON0bits.EN = 0;
    //Set Clear Buffer Flag
    I2C2STAT1bits.CLRBF = 1;
    //Enable I2C2
    I2C2CON0bits.EN = 1;
}

static inline void I2C2_MasterEnableRestart(void)
{
    //Enable I2C2 Restart
    I2C2CON0bits.RSEN = 1;
}

static inline void I2C2_MasterDisableRestart(void)
{
    //Disable I2C2 Restart
    I2C2CON0bits.RSEN = 0;
}

static inline void I2C2_MasterStop(void)
{
    //Clear Start Bit
    I2C2CON0bits.S = 0;					/* This probably have no effect. */
}

static inline bool I2C2_MasterIsNack(void)
{
    return I2C2CON1bits.ACKSTAT;
}

static inline void I2C2_MasterSendAck(void)
{
    I2C2CON1bits.ACKDT = 0;
}

static inline void I2C2_MasterSendNack(void)
{
    I2C2CON1bits.ACKDT = 1;
}

static inline void I2C2_MasterClearBusCollision(void)
{
    I2C2ERRbits.BCLIF = 0;
    I2C2ERRbits.BTOIF = 0;
    I2C2ERRbits.NACKIF = 0;
}

static inline bool I2C2_MasterIsRxBufFull(void)
{
    return I2C2STAT1bits.RXBF;
}

static inline bool I2C2_MasterIsTxBufEmpty(void)
{
    return I2C2STAT1bits.TXBE;
}

static inline bool I2C2_MasterIsStopFlagSet(void)
{
    return I2C2PIRbits.PCIF;
}

static inline bool I2C2_MasterIsCountFlagSet(void)
{
    return I2C2PIRbits.CNTIF;
}

static inline bool I2C2_MasterIsNackFlagSet(void)
{
    return I2C2ERRbits.NACKIF;
}

static inline void I2C2_MasterClearStopFlag(void)
{
    I2C2PIRbits.PCIF = 0;
}

static inline void I2C2_MasterClearCountFlag(void)
{
    I2C2PIRbits.CNTIF = 0;
}

static inline void I2C2_MasterClearNackFlag(void)
{
    I2C2ERRbits.NACKIF = 0;
}

static inline void I2C2_MasterEnableIrq(void)
{
    I2C2IE   = 1; 	/*    PIE7bits.I2C2IE    = 1;	*/
    I2C2EIE  = 1;	/*    PIE7bits.I2C2EIE   = 1;	*/
	I2C2RXIE = 1;	/*    PIE7bits.I2C2RXIE  = 1;	*/
	I2C2TXIE = 1;	/*    PIE7bits.I2C2TXIE  = 1;	*/

    I2C2PIEbits.PCIE = 1; 
    I2C2PIEbits.CNTIE = 1; 
    I2C2ERRbits.NACKIE = 1; 
}

static inline bool I2C2_MasterIsIrqEnabled(void)
{
//    return (PIE7bits.I2C2RXIE && PIE7bits.I2C2TXIE && PIE7bits.I2C2IE);
	return (I2C2RXIE && I2C2TXIE && I2C2IE);				/* PIC18F27K42 */
}

static inline void I2C2_MasterDisableIrq(void)
{
    I2C2IE   = 0;	/*    PIE7bits.I2C2IE    = 0;	*/
    I2C2EIE  = 0;	/*    PIE7bits.I2C2EIE    = 0;	*/
    I2C2RXIE = 0;	/*    PIE7bits.I2C2RXIE  = 0;	*/
    I2C2TXIE = 0;	/*    PIE7bits.I2C2TXIE  = 0;	*/
    I2C2PIEbits.SCIE = 0;
    I2C2PIEbits.PCIE = 0;
    I2C2PIEbits.CNTIE = 0;
    I2C2PIEbits.ACKTIE = 0;
    I2C2PIEbits.RSCIE = 0;
    I2C2ERRbits.BCLIE = 0;
    I2C2ERRbits.BTOIE = 0;
    I2C2ERRbits.NACKIE = 0;
}

static inline void I2C2_MasterClearIrq(void)
{
    I2C2PIR = 0x00;		I2C2ERR &= 0x0F;
}

static inline void I2C2_MasterWaitForEvent(void)
{
    while(1)
    {
        if (I2C2TXIF)	/*    if(PIR7bits.I2C2TXIF)	*/
        {    
            break;
        }
        if (I2C2RXIF)	/*    if(PIR7bits.I2C2RXIF)	*/
        {  
            break;
        } 
        if(I2C2PIRbits.PCIF)
        {
            break;
        } 
        if(I2C2PIRbits.CNTIF)
        {
            break;
        }
        if(I2C2ERRbits.NACKIF)
        {   __debug_break();
            break;
        }
		if (I2C2ERRbits.BCLIF)			/* Mysil was here, inserted. */
		{	break;
		}
    }
}
