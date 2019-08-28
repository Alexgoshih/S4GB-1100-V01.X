/**
  UART4 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart4.c

  @Summary 
    This is the generated source file for the UART4 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for UART4. 
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

#include "uart4.h"

extern unsigned char Car_information_patch_1[33];
extern unsigned char Car_information_patch_2[21];
unsigned char Rx4Value;
unsigned int read_uart4_data_flag = 0;
unsigned int read_uart4_data10_flag = 0;
unsigned int read_uart4_data11_flag = 0;
unsigned int read_uart4_dataK_flag = 0,can_data_flag=0;

unsigned int Car_information_patch_1_flag=0;
unsigned int Car_information_patch_2_flag=0;

unsigned int Car_information_patch_1Start_flag=0;
unsigned int Car_information_patch_2Start_flag=0;

unsigned int Car_information_patch_1Start_index=0;
unsigned int Car_information_patch_2Start_index=0;

extern unsigned int canUnlock_flag;
extern unsigned int send_image_flag;

extern unsigned int toggleWake_flag;
extern unsigned int accToggle_flag;

extern unsigned int g_sensor_toggle_flag;
extern unsigned char send10MFlag;
extern unsigned int startCanUARTtoggle_flag;
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

static UART_OBJECT uart4_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART4_CONFIG_TX_BYTEQ_LENGTH 8
#define UART4_CONFIG_RX_BYTEQ_LENGTH 8

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart4_txByteQ[UART4_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart4_rxByteQ[UART4_CONFIG_RX_BYTEQ_LENGTH] ;

/**
  Section: Driver Interface
*/

void UART4_Initialize(void)
{
    // Set the UART4 module to the options selected in the user interface.

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U4MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U4STA = 0x0;
    // BaudRate = 921600.000; Frequency = 58982400 Hz; BRG 15; 
    U4BRG = 0xF;
   IEC5bits.U4RXIE = 1;

   //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
  
   U4MODEbits.UARTEN = 1;  // enabling UARTEN bit
   U4STAbits.UTXEN = 1; 
 
    
   uart4_obj.txHead = uart4_txByteQ;
   uart4_obj.txTail = uart4_txByteQ;
   uart4_obj.rxHead = uart4_rxByteQ;
   uart4_obj.rxTail = uart4_rxByteQ;
   uart4_obj.rxStatus.s.empty = true;
   uart4_obj.txStatus.s.empty = true;
   uart4_obj.txStatus.s.full = false;
   uart4_obj.rxStatus.s.full = false;
}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4TXInterrupt ( void )
{ 
    if(uart4_obj.txStatus.s.empty)
    {
        IEC5bits.U4TXIE = false;
        return;
    }

    IFS5bits.U4TXIF = false;

    while(!(U4STAbits.UTXBF == 1))
    {

        U4TXREG = *uart4_obj.txHead;

        uart4_obj.txHead++;

        if(uart4_obj.txHead == (uart4_txByteQ + UART4_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart4_obj.txHead = uart4_txByteQ;
        }

        uart4_obj.txStatus.s.full = false;

        if(uart4_obj.txHead == uart4_obj.txTail)
        {
            uart4_obj.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4RXInterrupt( void )
{
    Rx4Value = U4RXREG;
    read_uart4_data_flag = 1;
    
//    if(startCanUARTtoggle_flag==1)
//    {
////        startCanUARTtoggle_flag=0;
//        g_sensor_toggle_flag=1;
//    toggleWake_flag=1;
//    send10MFlag=1;
//    }
    
    if(Car_information_patch_1Start_flag==1)
    {
//        Car_information_patch_1Start_flag=0;
        Car_information_patch_1[Car_information_patch_1Start_index++]=Rx4Value;
        if((Car_information_patch_1Start_index>33))
        {
            Car_information_patch_1Start_flag=0;
            can_data_flag=1;
            UART4_Initialize();
            
                                    if(Car_information_patch_1[26]==0xff)//25
                {
                    canUnlock_flag=1;
                }
                        else{
                                                canUnlock_flag=0;
                        }
            
        }
//                        if(Car_information_patch_1[26]==0xff)//25
//                {
//                    canUnlock_flag=1;
//                }
//                        else{
//                                                canUnlock_flag=0;
//                        }
    }
    
    if(Car_information_patch_1_flag==1)
    {
        Car_information_patch_1_flag = 0;
        if(Rx4Value=='\x1F')
        {
            Car_information_patch_1Start_flag=1;
            Car_information_patch_1[0]='\x10';
            Car_information_patch_1[1]='\x11';
            Car_information_patch_1[2]='\x1F';
            Car_information_patch_1Start_index=3;
        }
        else
        {
            Car_information_patch_1Start_flag=0;
        }
    }
//    else if(Car_information_patch_2_flag==1)
//    {
//        Car_information_patch_2_flag = 0;
//        if(Rx4Value=='\x11')
//        {
//            Car_information_patch_2Start_flag=1;
//            Car_information_patch_2[0]='\x10';
//            Car_information_patch_2[1]='\x11';
//            Car_information_patch_2[2]='\x11';
//            Car_information_patch_2Start_index=3;
//        }
//        else
//        {
//            Car_information_patch_2Start_flag=0;
//        }
//    }
    else
    {
        Car_information_patch_1_flag=0;
//        Car_information_patch_2_flag=0;
    }
    
    if(read_uart4_data10_flag==1)
    {
        read_uart4_data10_flag=0;
        if(Rx4Value=='\x11')
        {
            Car_information_patch_1_flag = 1;
        }
//        else if(Rx4Value=='\x11')
//        {
//            Car_information_patch_2_flag = 1;
//        }
        else 
        {
            Car_information_patch_1_flag = 0;
//            Car_information_patch_2_flag = 0;
        }
    }
    
    if(Rx4Value=='\x10')
    {
        read_uart4_data10_flag = 1;
        read_uart4_data10_flag=read_uart4_data10_flag;
    }
    else 
    {
        read_uart4_data10_flag = 0;
    }

    
    
    IFS5bits.U4RXIF = false;
   
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4ErrInterrupt ( void )
{
    if ((U4STAbits.OERR == 1))
    {
        U4STAbits.OERR = 0;
    }

    IFS5bits.U4EIF = false;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART4_Read( void)
{
    uint8_t data = 0;

    data = *uart4_obj.rxHead;

    uart4_obj.rxHead++;

    if (uart4_obj.rxHead == (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart4_obj.rxHead = uart4_rxByteQ;
    }

    if (uart4_obj.rxHead == uart4_obj.rxTail)
    {
        uart4_obj.rxStatus.s.empty = true;
    }

    uart4_obj.rxStatus.s.full = false;

    return data;
}


unsigned int UART4_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart4_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART4_Read () ;
        }
    }

    return numBytesRead ;
}



void UART4_Write( const uint8_t byte)
{
    IEC5bits.U4TXIE = false;
    
    *uart4_obj.txTail = byte;

    uart4_obj.txTail++;
    
    if (uart4_obj.txTail == (uart4_txByteQ + UART4_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart4_obj.txTail = uart4_txByteQ;
    }

    uart4_obj.txStatus.s.empty = false;

    if (uart4_obj.txHead == uart4_obj.txTail)
    {
        uart4_obj.txStatus.s.full = true;
    }

    IEC5bits.U4TXIE = true ;
	
}


unsigned int UART4_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart4_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART4_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}


UART4_TRANSFER_STATUS UART4_TransferStatusGet (void )
{
    UART4_TRANSFER_STATUS status = 0;

    if(uart4_obj.txStatus.s.full)
    {
        status |= UART4_TRANSFER_STATUS_TX_FULL;
    }

    if(uart4_obj.txStatus.s.empty)
    {
        status |= UART4_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart4_obj.rxStatus.s.full)
    {
        status |= UART4_TRANSFER_STATUS_RX_FULL;
    }

    if(uart4_obj.rxStatus.s.empty)
    {
        status |= UART4_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART4_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}


uint8_t UART4_Peek(uint16_t offset)
{
    if( (uart4_obj.rxHead + offset) > (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart4_rxByteQ[offset - (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH - uart4_obj.rxHead)];
    }
    else
    {
      return *(uart4_obj.rxHead + offset);
    }
}


unsigned int UART4_ReceiveBufferSizeGet(void)
{
    if(!uart4_obj.rxStatus.s.full)
    {
        if(uart4_obj.rxHead > uart4_obj.rxTail)
        {
            return(uart4_obj.rxHead - uart4_obj.rxTail);
        }
        else
        {
            return(UART4_CONFIG_RX_BYTEQ_LENGTH - (uart4_obj.rxTail - uart4_obj.rxHead));
        } 
    }
    return 0;
}


unsigned int UART4_TransmitBufferSizeGet(void)
{
    if(!uart4_obj.txStatus.s.full)
    { 
        if(uart4_obj.txHead > uart4_obj.txTail)
        {
            return(uart4_obj.txHead - uart4_obj.txTail);
        }
        else
        {
            return(UART4_CONFIG_TX_BYTEQ_LENGTH - (uart4_obj.txTail - uart4_obj.txHead));
        }
    }
    return 0;
}


bool UART4_ReceiveBufferIsEmpty (void)
{
    return(uart4_obj.rxStatus.s.empty);
}


bool UART4_TransmitBufferIsFull(void)
{
    return(uart4_obj.txStatus.s.full);
}


UART4_STATUS UART4_StatusGet (void)
{
    return U4STA;
}

void putsUART1_4(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    while(*temp_ptr != '\0')	 //72us
    {
        while(U4STAbits.UTXBF);  /* wait if the buffer is full */
        U4TXREG = *temp_ptr++;   /* transfer data byte to TX reg */		
//		TEST_IO=!TEST_IO;		
    }  
}

void BT4_Send_String(unsigned char *buffer,unsigned int count)
{
unsigned int i=0;
	for(i=0;i<count;i++)
	{
	while (U4STAbits.UTXBF);
    U4TXREG=buffer[i];
	}
}

/**
  End of File
*/
