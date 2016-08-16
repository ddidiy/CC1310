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

#include <stdint.h>
#include <stddef.h>
/* driverlib header files */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/uart.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>
#include <driverlib/aon_ioc.h>
#include <uartstdio.h>
//#include "FreeRTOS.h"
//#include "semphr.h"


const UART_Config UART_config[] = {
    {UART0_BASE,
     0,
     PRCM_PERIPH_UART0,
     IOID_9,                   //spi sck
     IOID_7,                    //spi mosi
     IOID_UNUSED,               //spi miso
     IOID_UNUSED,               //spi fss
    
    }
    //{NULL, NULL, NULL},
};


static bool  UART_initIO(UART_Handle handle);

static const uint32_t dataLength[] = {
    UART_CONFIG_WLEN_5, /* UART_LEN_5 */
    UART_CONFIG_WLEN_6, /* UART_LEN_6 */
    UART_CONFIG_WLEN_7, /* UART_LEN_7 */
    UART_CONFIG_WLEN_8  /* UART_LEN_8 */
};

static const uint32_t stopBits[] = {
    UART_CONFIG_STOP_ONE,   /* UART_STOP_ONE */
    UART_CONFIG_STOP_TWO    /* UART_STOP_TWO */
};

static const uint32_t parityType[] = {
    UART_CONFIG_PAR_NONE,   /* UART_PAR_NONE */
    UART_CONFIG_PAR_EVEN,   /* UART_PAR_EVEN */
    UART_CONFIG_PAR_ODD,    /* UART_PAR_ODD */
    UART_CONFIG_PAR_ZERO,   /* UART_PAR_ZERO */
    UART_CONFIG_PAR_ONE     /* UART_PAR_ONE */
};


//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
//xSemaphoreHandle g_pUARTSemaphore;

/*
 *  ================================= Macro ====================================
 *  TODO: Move me
 */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


void (*input_handler)(uint8_t c); 
void UARTRxISR();


/*
 * Function for checking whether flow control is enabled.
 */
static inline bool isFlowControlEnabled(UART_Config const  *uartConfig) {
    return ((uartConfig->ctsPin != IOID_UNUSED) && (uartConfig->rtsPin != IOID_UNUSED));
}


/*
 *  ======== writeData ========
 *  Write and process data to the UART.
 */
static int32_t writeData(UART_Handle handle,const unsigned char *buffer, int32_t size)
{
    int i;
    /* Send characters until FIFO is full or done. */
    //if( xSemaphoreTake( g_pUARTSemaphore, ( TickType_t ) 0 ) == pdTRUE )
    {
        for(i=0;i<size;i++)
        {
            /* Send the next character and increment counts. */
            if (!UARTCharPutNonBlocking(handle->baseAddr,buffer[i]))
                break;
        }
        
    
    //    xSemaphoreGive( g_pUARTSemaphore );
    }
        
    return (size-i);
}

/*
 *  ======== readData ========
 *  Read and process data from the UART.
 */
static int32_t readData(UART_Handle handle,unsigned char *buffer, int32_t size)
{
    int32_t                     readIn;
    unsigned char               *p;
    p = buffer;
    /* Receive chars until empty or done. */
    while (size && (readIn = (int32_t)UARTCharGetNonBlocking(handle->baseAddr)) != -1) {

        /* Update status. */
        *p = (uint8_t)readIn;
        p++;
        size--;
    }
    return (size);
}


/*
 *  ======== writeTxFifoFlush ========
 *  Write cancelled or timed out, the TX FIFO must be flushed out.
 *
 *  This function is called either from writeCancel or when a blocking write
 *  has timed out. The HW does not support a simple API for flushing the TX FIFO
 *  so a workaround is done in SW.
 *
 *  @pre The TX FIFO empty clock must have been started in blocking mode.
 *
 *  @param object         Pointer to UART object
 *  @param hwAttrs        Pointer to UART hwAttrs
 */
//static void writeTxFifoFlush(UARTCC26XX_Object  *object, UARTCC26XX_HWAttrs const  *hwAttrs)
//{
//    /*It is not possible to flush the TX FIFO with simple write to HW, doing workaround:
//     * 0. Disable TX interrupt
//     */
//    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
//    /* 1. Ensure TX IO will stay high when connected to GPIO */
//    PIN_setOutputEnable(hPin, hwAttrs->txPin, 1);
//    PIN_setOutputValue(hPin, hwAttrs->txPin, 1);
//    /* 2. Disconntect tx from IO, and set it as "GPIO" */
//    PINCC26XX_setMux(hPin, hwAttrs->txPin, IOC_PORT_GPIO);
//    /* 3. Disconnect cts */
//    PINCC26XX_setMux(hPin, hwAttrs->ctsPin, IOC_PORT_GPIO);
//    /*4. Wait for TX FIFO to become empty.
//     *    CALLBACK: Idle until the TX FIFO is empty, i.e. no longer busy.
//     *    BLOCKING: Periodically check if TX is busy emptying the FIFO.
//     *              Must be handled at TX FIFO empty clock timeout:
//     *                - the timeout/finish function must check the status
//     */
//    if(object->writeMode == UART_MODE_CALLBACK) {
//        /* Wait until the TX FIFO is empty. CALLBACK mode can be used from
//         * hwi/swi context, so we cannot use semaphore..
//         */
//        while(UARTBusy(hwAttrs->baseAddr));
//    } else { /* i.e. UART_MODE_BLOCKING */
//        /* Pend on semaphore again..(this time forever since we are flushing
//         * TX FIFO and nothing should be able to stop it..
//         */
//        Semaphore_pend(Semaphore_handle(&(object->writeSem)), BIOS_WAIT_FOREVER);
//    }
//    /* 5. Revert to active pins before returning */
//    PINCC26XX_setMux(hPin, hwAttrs->txPin, IOC_PORT_MCU_UART0_TX);
//    if(isFlowControlEnabled(hwAttrs)) {
//        PINCC26XX_setMux(hPin, hwAttrs->ctsPin, IOC_PORT_MCU_UART0_CTS);
//    }
//}


void UART_Params_init(UART_Params *params){
  
   params->baudRate=115200;
   params->dataLength=UART_LEN_8;
   params->parityType = UART_PAR_NONE;
   params->stopBits = UART_STOP_ONE;
  
}



UART_Handle UART_open(int index,int baudrate, UART_LEN dataLength,       
                      UART_STOP stopBits,UART_PAR parityType) 
{
    UART_Params  params;
    
    params.baudRate=baudrate;
    params.dataLength=dataLength;
    params.parityType = parityType;
    params.stopBits = stopBits;
    return UART_set(index, &params);
}


/*!
 *  @brief  Function to initialize the  peripheral specified by the
 *          particular handle. The parameter specifies which mode the UART
 *          will operate.
 *
 *  The function will set a dependency on it power domain, i.e. power up the
 *  module and enable the clock. The IOs are allocated. Neither the RX nor TX
 *  will be enabled, and none of the interrupts are enabled.
 *
 *  @pre    UART controller has been initialized
 *          Calling context: Task
 *
 *  @param  handle        A UART_Handle
 *
 *  @param  params        Pointer to a parameter block, if NULL it will use
 *                        default values
 *
 *  @return A UART_Handle on success or a NULL on an error or if it has been
 *          already opened
 */
UART_Handle UART_set(int index, UART_Params *params)
{
    /* Use union to save on stack allocation */
    UART_Handle handle = (UART_Handle)&(UART_config[index]);

    /* If params are NULL use defaults. */
    if (params == NULL) {
        UART_Params_init(params);
    }
    // Power On devices register
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL)
          !=PRCM_DOMAIN_POWER_ON);
    
    PRCMPeripheralRunEnable(handle->powerMngrId);
    //PRCMPeripheralSleepEnable(handle->powerMngrId);
    //PRCMPeripheralDeepSleepEnable(handle->powerMngrId);
    PRCMLoadSet();
    while(!PRCMLoadGet());
   
    /* Configure IOs, make sure it was successful */
    if(!UART_initIO(handle)) {
        /* Disable UART */
        UARTDisable(handle->baseAddr);
        /* Release power dependency - i.e. potentially power down serial domain. */
        PRCMPowerDomainOff(handle->powerMngrId);
        /* Mark the module as available */
        return (NULL);
    }
    
    /* Initialize the UART hardware module */
    UART_setting(handle,params);
//    if(handle!=NULL)
//      g_pUARTSemaphore = xSemaphoreCreateBinary();

    /* Return the handle */
    
    
    return (handle);
}

static void UARTRxIntEnable()
{
    UARTIntRegister(UART0_BASE, UARTRxISR);
    
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    
}


/*!
 *  @brief  Function to close a given CC26XX UART peripheral specified by the
 *          UART handle.
 *
 *  Will disable the UART, disable all UART interrupts and release the
 *  dependency on the corresponding power domain.
 *
 *  @pre    UARTCC26XX_open() had to be called first.
 *          Calling context: Task
 *
 *  @param  handle  A UART_Handle returned from UART_open()
 *
 *  @sa     UARTCC26XX_open
 */
void UART_close(UART_Handle handle)
{
 
    /* Disable all UART module interrupts. */
    UARTIntDisable(handle->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                      UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                      UART_INT_RX | UART_INT_CTS);
    /* Disable UART */
    UARTDisable(handle->baseAddr);

    /* Release power dependency - i.e. potentially power down serial domain. */
    PRCMPowerDomainOff(handle->powerMngrId);

   
}



/*!
 *  @brief  Function that writes data to a UART
 *
 *  This function initiates an operation to write data to CC26XX UART
 *  controller.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Hwi and Swi (only if using ::UART_MODE_CALLBACK), Task
 *
 *  @param  handle      A UART_Handle returned from UARTCC26XX_open()
 *
 *  @param  buffer      A pointer to buffer containing data to be written
 *
 *  @param  size        The number of bytes in buffer that should be written
 *                      onto the UART.
 *
 *  @return Returns the number of bytes that have been written to the UART,
 *          UART_ERROR on an error.
 *
 */
int UART_write(UART_Handle handle, const uint8_t *buffer, size_t size)
{
    /* Enable TX */
    HWREG(handle->baseAddr + UART_O_CTL) |= UART_CTL_TXE;

    /* Fill up TX FIFO */
    return writeData(handle, buffer,size);
     
}



/*!
 *  @brief  Function for reading from UART interface.
 *
 *  The function will enable the RX, enable all RX interrupts and disallow
 *  chip from going into standby.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Hwi and Swi (only if using ::UART_MODE_CALLBACK), Task
 *
 *  @param  handle A UART handle returned from UARTCC26XX_open()
 *
 *  @param  *buffer  Pointer to read buffer
 *
*  @param  size  Number of bytes to read. If ::UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE
 *                has been set, the read will
 *                return if the reception is inactive for a 32-bit period
 *                (i.e. before all bytes are received).
 *
 *  @return Number of samples read
 *
 *  @sa     UARTCC26XX_open(), UARTCC26XX_readCancel()
 */
int UART_read(UART_Handle handle, void *buffer, size_t size)
{
  

    /* Enable RX */
    HWREG(handle->baseAddr + UART_O_CTL) |= UART_CTL_RXE;

    /* Enable RX interrupts */
    UARTIntEnable(handle->baseAddr, UART_INT_RX | UART_INT_RT |
                  UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);

    return (0);
}


/*
 *  ======== UART_setting ========
 *  This functions initializes the UART hardware module.
 *
 *  @pre    Function assumes that the UART handle is pointing to a hardware
 *          module which has already been opened.
 */
static void UART_setting(UART_Handle handle,UART_Params *params) {

    /* Disable UART function. */
   UARTDisable(handle->baseAddr);

    /* Disable all UART module interrupts. */
    UARTIntDisable(handle->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                      UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                      UART_INT_RX | UART_INT_CTS);

    /* Clear all UART interrupts */
    UARTIntClear(handle->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                    UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                    UART_INT_RX | UART_INT_CTS);

    /* Set the FIFO level to 7/8 empty and 4/8 full. The setting was initially
     * 7/8 full, but has been changed to 4/8 full. Consider implementing the
     * FIFO levels as parameters in struct.
     */
    UARTFIFOLevelSet(handle->baseAddr, UART_FIFO_TX7_8, UART_FIFO_RX1_8);

    /* Configure frame format and baudrate */
    UARTConfigSetExpClk(handle->baseAddr,
                        48000000,
                        params->baudRate,
                        (dataLength[params->dataLength] |
                        stopBits[params->stopBits] |
                        parityType[params->parityType]));

    /* Enable UART FIFOs */
    HWREG(handle->baseAddr + UART_O_LCRH) |= UART_LCRH_FEN;

    /* Enable the UART module */
    HWREG(handle->baseAddr + UART_O_CTL) |= UART_CTL_UARTEN;
    
    UARTRxIntEnable();
    
    UARTEnable(handle->baseAddr);

    /* If Flow Control is enabled, configure hardware controlled flow control */
//    if(isFlowControlEnabled(hwAttrs)) {
//        HWREG(handle->baseAddr + UART_O_CTL) |= (UART_CTL_CTSEN | UART_CTL_RTSEN);
//    }
}

/*
 *  ======== UART_initIO ========
 *  This functions initializes the UART IOs.
 *
 *  @pre    Function assumes that the UART handle is pointing to a hardware
 *          module which has already been opened.
 */
static bool UART_initIO(UART_Handle handle) {
    //IOCPinTypeGpioOutput(handle->txPin);
    //GPIOPinWrite(handle->txPin, 1);
  
    /* Locals */
    IOCPinTypeUart(handle->baseAddr, handle->rxPin, handle->txPin,
               handle->ctsPin, handle->rtsPin);
    /* Success */
    return true;
}



void UARTRxCBRegister(void (* UARTRxCBFun)(uint8_t c))
{
    input_handler = UARTRxCBFun;
}



void UARTRxISR()
{
  uint16_t the_char=0;
  uint32_t flags;
  /* Read out the masked interrupt status */
  flags = UARTIntStatus(UART0_BASE, true);

  /* Clear all UART interrupt flags */
  UARTIntClear(UART0_BASE, (UART_INT_RX | UART_INT_RT));

  if((flags & (UART_INT_RX | UART_INT_RT)) != 0) {
    /*
     * If this was a FIFO RX or an RX timeout, read all bytes available in the
     * RX FIFO.
     */
    while(UARTCharsAvail(UART0_BASE)) {
      the_char = UARTCharGetNonBlocking(UART0_BASE);

      if(input_handler != NULL) {
        input_handler((unsigned char)the_char);
      }
    }
  }
}

