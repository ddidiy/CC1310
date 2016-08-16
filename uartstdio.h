/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _UART_DRIVER_
#define _UART_DRIVER_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <driverlib/uart.h>

/*! Size of the TX and RX FIFOs is 32 items */
#define UARTCC26XX_FIFO_SIZE 32

typedef struct UART_Config      *UART_Handle;

typedef struct UART_Config {
    uint32_t   baseAddr;    /*!< UART Peripheral's base address */
    int        intNum;      /*!< UART Peripheral's interrupt vector */
    uint32_t   powerMngrId; /*!< UART Peripheral's power manager ID */
    uint8_t    txPin;       /*!< UART TX pin */
    uint8_t    rxPin;       /*!< UART RX pin */
    uint8_t    ctsPin;      /*!< UART CTS pin */
    uint8_t    rtsPin;      /*!< UART RTS pin */
} UART_Config;

///*!
// *  @brief    UART status
// *
// *  The UART Status is used to flag the different Receive Errors.
// */
//typedef enum UART_Status {
//    UART_TIMED_OUT     = 0x10,                 /*!< UART timed out */
//    UART_PARITY_ERROR  = UART_RXERROR_PARITY,  /*!< UART Parity error */
//    UART_BRAKE_ERROR   = UART_RXERROR_BREAK,   /*!< UART Break error */
//    UART_OVERRUN_ERROR = UART_RXERROR_OVERRUN, /*!< UART overrun error */
//    UART_FRAMING_ERROR = UART_RXERROR_FRAMING, /*!< UART Framing error */
//    UART_OK            = 0x0                   /*!< UART OK */
//} UART_Status;


typedef enum UART_LEN {
    UART_LEN_5 = 0,        /*!< Data length is 5 */
    UART_LEN_6 = 1,        /*!< Data length is 6 */
    UART_LEN_7 = 2,        /*!< Data length is 7 */
    UART_LEN_8 = 3         /*!< Data length is 8 */
} UART_LEN;

// @brief    UART stop bit settings
typedef enum UART_STOP {
    UART_STOP_ONE = 0,  /*!< One stop bit */
    UART_STOP_TWO = 1   /*!< Two stop bits */
} UART_STOP;


//@brief    UART parity type settings
typedef enum UART_PAR {
    UART_PAR_NONE = 0,  /*!< No parity */
    UART_PAR_EVEN = 1,  /*!< Parity bit is even */
    UART_PAR_ODD  = 2,  /*!< Parity bit is odd */
    UART_PAR_ZERO = 3,  /*!< Parity bit is always zero */
    UART_PAR_ONE  = 4   /*!< Parity bit is always one */
} UART_PAR;


// @brief    UART FIFO threshold
typedef enum UART_FifoThreshold {
    UART_TH_FIFO_1_8 = 4,         /*!< RX FIFO threshold of 1/8 = 4 bytes */
    UART_TH_FIFO_2_8 = 8,         /*!< RX FIFO threshold of 2/8 = 8 bytes */
    UART_TH_FIFO_4_8 = 16,        /*!< RX FIFO threshold of 4/8 = 16 bytes */
    UART_TH_FIFO_6_8 = 24,        /*!< RX FIFO threshold of 6/8 = 24 bytes */
    UART_TH_FIFO_7_8 = 28         /*!< RX FIFO threshold of 7/8 = 28 bytes */
} UART_FifoThreshold;

/*!
 *  @brief      UARTCC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct UART_Params {
    uint32_t          baudRate;         /*!< Baud rate for UART */
    UART_LEN          dataLength;       /*!< Data length for UART */
    UART_STOP         stopBits;         /*!< Stop bits for UART */
    UART_PAR          parityType;       /*!< Parity bit type for UART */
   
} UART_Params;


/* Public interfaces */
void         UART_close(UART_Handle handle);
int          UART_control(UART_Handle handle, unsigned int cmd, void *arg);
UART_Handle UART_open(int index,int baudrate, UART_LEN dataLength,       
            UART_STOP stopBits,UART_PAR parityType);

UART_Handle  UART_set(int index, UART_Params *params);
int          UART_read(UART_Handle handle, void *buffer, size_t size);
int          UART_readPolling(UART_Handle handle, void *buf, size_t size);
void         UART_readCancel(UART_Handle handle);
int          UART_write(UART_Handle handle, const uint8_t *buffer,
                            size_t size);
int          UART_writePolling(UART_Handle handle, const void *buf,
                                   size_t size);
void         UART_writeCancel(UART_Handle handle);
void UART_setting(UART_Handle handle,UART_Params *params);

void UARTRxCBRegister(void (* UARTRxCBFun)(uint8_t c));



#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_uart_UARTCC26XX__include */
