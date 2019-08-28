/**
  UART3 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart3.c

  @Summary 
    This is the generated source file for the UART3 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for UART3. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - pic24-dspic-pic32mm : 1.53.0.1
        Device            :  dsPIC33EP128GM604
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.33
        MPLAB 	          :  MPLAB X v4.05
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
//#include "mcc_generated_files/mcc.h"
#include "uart3.h"
//#include "uart1.h"

#include "spi1.h"
#include "pin_manager.h"

unsigned char Rx3Value;
unsigned int read_uart3_data_flag = 0;
unsigned int read_uart3_dataO_flag = 0;
unsigned int read_uart3_dataN_flag = 0;
unsigned int read_uart3_dataI_flag = 0;
unsigned int read_uart3_start_flag = 0;
unsigned int read_uart3_dataM_flag = 0;
unsigned int read_uart3_dataQ_flag = 0;
unsigned int send_image_flag = 0;
int temp1_index=0;

//uint8_t temp1[1500];


/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

typedef union
{
    struct
    {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
}

UART_BYTEQ_STATUS;

/** UART Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

*/
typedef struct
{
    /* RX Byte Q */
    uint8_t                                      *rxTail ;

    uint8_t                                      *rxHead ;

    /* TX Byte Q */
    uint8_t                                      *txTail ;

    uint8_t                                      *txHead ;

    UART_BYTEQ_STATUS                        rxStatus ;

    UART_BYTEQ_STATUS                        txStatus ;

} UART_OBJECT ;

static UART_OBJECT uart3_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART3_CONFIG_TX_BYTEQ_LENGTH 8
#define UART3_CONFIG_RX_BYTEQ_LENGTH 8

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart3_txByteQ[UART3_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart3_rxByteQ[UART3_CONFIG_RX_BYTEQ_LENGTH] ;

/**
  Section: Driver Interface
*/

void UART3_Initialize(void)
{
    // Set the UART3 module to the options selected in the user interface.

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U3MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U3STA = 0x0;
    // BaudRate = 921600.000; Frequency = 58982400 Hz; BRG 15; 
    U3BRG = 0xF;
   IEC5bits.U3RXIE = 1;

   //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
  
   U3MODEbits.UARTEN = 1;  // enabling UARTEN bit
   U3STAbits.UTXEN = 1; 
 
    
   uart3_obj.txHead = uart3_txByteQ;
   uart3_obj.txTail = uart3_txByteQ;
   uart3_obj.rxHead = uart3_rxByteQ;
   uart3_obj.rxTail = uart3_rxByteQ;
   uart3_obj.rxStatus.s.empty = true;
   uart3_obj.txStatus.s.empty = true;
   uart3_obj.txStatus.s.full = false;
   uart3_obj.rxStatus.s.full = false;
}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3TXInterrupt ( void )
{ 
    if(uart3_obj.txStatus.s.empty)
    {
        IEC5bits.U3TXIE = false;
        return;
    }

    IFS5bits.U3TXIF = false;

    while(!(U3STAbits.UTXBF == 1))
    {

        U3TXREG = *uart3_obj.txHead;

        uart3_obj.txHead++;

        if(uart3_obj.txHead == (uart3_txByteQ + UART3_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart3_obj.txHead = uart3_txByteQ;
        }

        uart3_obj.txStatus.s.full = false;

        if(uart3_obj.txHead == uart3_obj.txTail)
        {
            uart3_obj.txStatus.s.empty = true;
            break;
        }
    }
}



void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3ErrInterrupt ( void )
{
    if ((U3STAbits.OERR == 1))
    {
        U3STAbits.OERR = 0;
    }

    IFS5bits.U3EIF = false;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART3_Read( void)
{
    uint8_t data = 0;

    data = *uart3_obj.rxHead;

    uart3_obj.rxHead++;

    if (uart3_obj.rxHead == (uart3_rxByteQ + UART3_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart3_obj.rxHead = uart3_rxByteQ;
    }

    if (uart3_obj.rxHead == uart3_obj.rxTail)
    {
        uart3_obj.rxStatus.s.empty = true;
    }

    uart3_obj.rxStatus.s.full = false;

    return data;
}


unsigned int UART3_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart3_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART3_Read () ;
        }
    }

    return numBytesRead ;
}



void UART3_Write( const uint8_t byte)
{
    IEC5bits.U3TXIE = false;
    
    *uart3_obj.txTail = byte;

    uart3_obj.txTail++;
    
    if (uart3_obj.txTail == (uart3_txByteQ + UART3_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart3_obj.txTail = uart3_txByteQ;
    }

    uart3_obj.txStatus.s.empty = false;

    if (uart3_obj.txHead == uart3_obj.txTail)
    {
        uart3_obj.txStatus.s.full = true;
    }

    IEC5bits.U3TXIE = true ;
	
}


unsigned int UART3_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart3_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART3_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}


UART3_TRANSFER_STATUS UART3_TransferStatusGet (void )
{
    UART3_TRANSFER_STATUS status = 0;

    if(uart3_obj.txStatus.s.full)
    {
        status |= UART3_TRANSFER_STATUS_TX_FULL;
    }

    if(uart3_obj.txStatus.s.empty)
    {
        status |= UART3_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart3_obj.rxStatus.s.full)
    {
        status |= UART3_TRANSFER_STATUS_RX_FULL;
    }

    if(uart3_obj.rxStatus.s.empty)
    {
        status |= UART3_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART3_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}


uint8_t UART3_Peek(uint16_t offset)
{
    if( (uart3_obj.rxHead + offset) > (uart3_rxByteQ + UART3_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart3_rxByteQ[offset - (uart3_rxByteQ + UART3_CONFIG_RX_BYTEQ_LENGTH - uart3_obj.rxHead)];
    }
    else
    {
      return *(uart3_obj.rxHead + offset);
    }
}


unsigned int UART3_ReceiveBufferSizeGet(void)
{
    if(!uart3_obj.rxStatus.s.full)
    {
        if(uart3_obj.rxHead > uart3_obj.rxTail)
        {
            return(uart3_obj.rxHead - uart3_obj.rxTail);
        }
        else
        {
            return(UART3_CONFIG_RX_BYTEQ_LENGTH - (uart3_obj.rxTail - uart3_obj.rxHead));
        } 
    }
    return 0;
}


unsigned int UART3_TransmitBufferSizeGet(void)
{
    if(!uart3_obj.txStatus.s.full)
    { 
        if(uart3_obj.txHead > uart3_obj.txTail)
        {
            return(uart3_obj.txHead - uart3_obj.txTail);
        }
        else
        {
            return(UART3_CONFIG_TX_BYTEQ_LENGTH - (uart3_obj.txTail - uart3_obj.txHead));
        }
    }
    return 0;
}


bool UART3_ReceiveBufferIsEmpty (void)
{
    return(uart3_obj.rxStatus.s.empty);
}


bool UART3_TransmitBufferIsFull(void)
{
    return(uart3_obj.txStatus.s.full);
}


UART3_STATUS UART3_StatusGet (void)
{
    return U3STA;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3RXInterrupt( void )
{
        Rx3Value = U3RXREG;
//    temp1[temp1_index]=Rx3Value;
        read_uart3_data_flag = 1;
    //temp1_index++;
    //U3TXREG=Rx3Value;
        if(send_image_flag==1)
        {
            U1TXREG=Rx3Value;
//            TMR1_SoftwareCounterClear();
//            TMR1_GetElapsedThenClear();
//            TMR1_Stop();
//            TMR1_Start();
        }
   IFS5bits.U3RXIF = false;
}


void putsUART1_3(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    while(*temp_ptr != '\xff')	 //72us
    {
        while(U3STAbits.UTXBF);  /* wait if the buffer is full */
        U3TXREG = *temp_ptr++;   /* transfer data byte to TX reg */		
//		TEST_IO=!TEST_IO;		
    }  
}

void BT3_Send_String(unsigned char *buffer,unsigned int count)
{
unsigned int i=0;
	for(i=0;i<count;i++)
	{
	while (U3STAbits.UTXBF);
    U3TXREG=buffer[i];
	}
}

/**
  End of File
*/
