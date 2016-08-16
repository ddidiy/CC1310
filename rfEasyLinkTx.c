/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/* Undefine to not use async mode */
#define RFEASYLINKTX_ASYNC

#define RFEASYLINKTX_TASK_STACK_SIZE    1024
#define RFEASYLINKTX_TASK_PRIORITY      2

#define RFEASYLINKTXPAYLOAD_LENGTH      30

Task_Struct txTask;    /* not static so you can see in ROV */
static Task_Params txTaskParams;
static uint8_t txTaskStack[RFEASYLINKTX_TASK_STACK_SIZE];

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State pinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static Semaphore_Handle txDoneSem;
static Semaphore_Handle rxDoneSem;
void txDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED1 to indicate TX */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED2 to indicate command aborted */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
    }

    Semaphore_post(txDoneSem);
}

UART_Handle uart;
UART_Params uartParams;
int number=0;
EasyLink_TxPacket txPacket = {0};
        
void readCB(UART_Handle handle, void *buf, size_t count){
  ((uint8_t *)buf)[1]=01;
  count = count+0x30;
  UART_write(uart, &count, 1);
}
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
      
    }
    else if(status == EasyLink_Status_Aborted)
    {
        
    }
    else
    {
       
    }

    Semaphore_post(rxDoneSem);
}

static void rfEasyLinkTxFnx(UArg arg0, UArg arg1)
{

    /* Create a semaphore for Async */
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    txDoneSem = Semaphore_create(0, &params, &eb);
    rxDoneSem = Semaphore_create(0, &params, &eb);

    EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    /* Set Freq to 868MHz */
    EasyLink_setFrequency(868000000);
    /* Set output power to 12dBm */
    EasyLink_setRfPwr(12);
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    //uartParams.readMode=UART_MODE_CALLBACK;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uartParams.readTimeout=(1000000 / Clock_tickPeriod);
    //uartParams.readCallback=readCB;
    uart = UART_open(Board_UART0, &uartParams);

    while(1) { 
        number = UART_read(uart, txPacket.payload, RFEASYLINKTXPAYLOAD_LENGTH);
        if(number>0)
        {
          UART_write(uart, txPacket.payload, number);

          txPacket.len = number;
          txPacket.dstAddr[0] = 0xaa;
          int numbersToTransmit = 3;
        
//            EasyLink_transmitAsync(&txPacket, txDoneCb);
//            /* Wait 300ms for Tx to complete */
//            if(Semaphore_pend(txDoneSem, (300000 / Clock_tickPeriod)) == FALSE)
//            {
//                /* TX timed out, abort */
//                if(EasyLink_abort() == EasyLink_Status_Success)
//                {
//                    /*
//                     * Abort will cause the txDoneCb to be called, and the txDoneSem ti
//                     * Be released. So we must consume the txDoneSem
//                     * */
//                   Semaphore_pend(txDoneSem, BIOS_WAIT_FOREVER);
//                }
//            }
          
//            EasyLink_receiveAsync(rxDoneCb, 0);
//
//            /* Wait 300ms for Rx */
//            if(Semaphore_pend(rxDoneSem, (300000 / Clock_tickPeriod)) == FALSE)
//            {
//                /* RX timed out abort */
//                if(EasyLink_abort() == EasyLink_Status_Success)
//                {
//                   /* Wait for the abort */
//                   Semaphore_pend(rxDoneSem, BIOS_WAIT_FOREVER);
//                }
//                numbersToTransmit--;
//                  
//            }
//            else
//            {
//
//                break;
//            }
        }
         
    }
}

void txTask_init(PIN_Handle inPinHandle) {
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = RFEASYLINKTX_TASK_STACK_SIZE;
    txTaskParams.priority = RFEASYLINKTX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, rfEasyLinkTxFnx, &txTaskParams, NULL);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initUART();
    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    if(!pinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    /* Clear LED pins */
    PIN_setOutputValue(pinHandle, Board_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_LED2, 0);

    txTask_init(pinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
