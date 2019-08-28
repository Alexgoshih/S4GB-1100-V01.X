/**
  UART1 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary 
    This is the generated source file for the UART1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for UART1. 
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

#include "uart1.h"
unsigned char Rx1Value;
unsigned char Colon=0x3a;
bool read_uart1_dataW1_flag = 0;
unsigned int read_uart1_data_flag = 0;
unsigned int read_uart1_dataO_flag = 0;
unsigned int read_uart1_dataK_flag = 0;
unsigned int read_uart1_dataP_flag = 0;
unsigned int read_uart1_dataB_flag = 0;
unsigned int read_uart1_dataN_flag = 0;
unsigned int read_uart1_dataI_flag = 0;
unsigned int read_uart1_start_flag = 0;
unsigned int read_uart1_dataM_flag = 0;
unsigned int read_uart1_dataQ_flag = 0;
unsigned int read_uart1_dataY_flag = 0;
unsigned int read_uart1_dataW_flag = 0;
unsigned int read_uart1_data01_flag = 0;
unsigned int read_uart1_data2A_flag = 0;
unsigned int read_uart1_data3A_flag = 0;
unsigned int read_uart1_data61_flag = 0;
unsigned int read_uart1_data21_flag = 0;
unsigned int read_uart1_data10_flag = 0;
unsigned int read_uart1_data08_flag = 0;
unsigned int read_uart1_dataPlus_flag = 0;
unsigned int request_token_flag = 0;
unsigned int read_uart1_data30_flag = 0;//may overflow using++
unsigned int read_uart1_data301_flag = 0;
unsigned int read_uart1_data62_flag = 0;
unsigned int read_uart1_data03_flag = 0;
unsigned int read_uart1_data66_flag = 0;
unsigned int read_uart1_data51_flag = 0;
unsigned int read_uart1_data55_flag = 0;
unsigned int read_uart1_data14_flag = 0;
unsigned int read_uart1_data93_flag = 0;    
unsigned int read_uart1_data12_flag = 0;    
unsigned int read_uart1_data26_flag = 0;    
unsigned int read_uart1_data28_flag = 0;    
unsigned int notRegistered_flag = 0;
unsigned int tokenCS_error_flag=0;

extern unsigned int get_sensor_flag,request_sensor_flag,get_sensor_CSRflag;

unsigned int token_ready_flag = 0;
unsigned int read_uart1_datatime_ready_flag = 0;
unsigned int request_time_flag = 0;
unsigned int get_time_flag = 0;
unsigned int get_token_flag = 0;
unsigned int token_index = 0, time_index=0;
unsigned int gsensor_index=0,gsensor[7];
extern uint8_t time[6],token[145]; 
uint8_t tokentemp[146],tokenCS,gsensor_cs=0; 
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

static UART_OBJECT uart1_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART1_CONFIG_TX_BYTEQ_LENGTH 8
#define UART1_CONFIG_RX_BYTEQ_LENGTH 8

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH] ;

/**
  Section: Driver Interface
*/

void UART1_Initialize(void)
{
    // Set the UART1 module to the options selected in the user interface.

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1STA = 0x0;
    // BaudRate = 921600.000; Frequency = 58982400 Hz; BRG 15; 
    U1BRG = 0xF;
   IEC0bits.U1RXIE = 1;

   //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
  
   U1MODEbits.UARTEN = 1;  // enabling UARTEN bit
   U1STAbits.UTXEN = 1; 
 
    
   uart1_obj.txHead = uart1_txByteQ;
   uart1_obj.txTail = uart1_txByteQ;
   uart1_obj.rxHead = uart1_rxByteQ;
   uart1_obj.rxTail = uart1_rxByteQ;
   uart1_obj.rxStatus.s.empty = true;
   uart1_obj.txStatus.s.empty = true;
   uart1_obj.txStatus.s.full = false;
   uart1_obj.rxStatus.s.full = false;
}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{ 
    if(uart1_obj.txStatus.s.empty)
    {
        IEC0bits.U1TXIE = false;
        return;
    }

    IFS0bits.U1TXIF = false;

    while(!(U1STAbits.UTXBF == 1))
    {

        U1TXREG = *uart1_obj.txHead;

        uart1_obj.txHead++;

        if(uart1_obj.txHead == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart1_obj.txHead = uart1_txByteQ;
        }

        uart1_obj.txStatus.s.full = false;

        if(uart1_obj.txHead == uart1_obj.txTail)
        {
            uart1_obj.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
        Rx1Value = U1RXREG;
        read_uart1_data_flag = 1;
        if(Rx1Value=='O')
        {
            read_uart1_dataO_flag = 1;
        }
        else if(Rx1Value=='W')
        {
            read_uart1_dataW_flag = 1;
            read_uart1_dataW1_flag = 1;
        }
        else if(Rx1Value=='N')
        {
            read_uart1_dataN_flag = 1;
        }
        else if(Rx1Value=='I')
        {
            read_uart1_dataI_flag = 1;
        }
        else if(Rx1Value=='M')
        {
            read_uart1_dataM_flag = 1;
        }
        else if(Rx1Value=='Q')
        {
            read_uart1_dataQ_flag = 1;
        }
        else if(Rx1Value=='Y')
        {
            read_uart1_dataY_flag = 1;
        }
        else if(Rx1Value=='\x01')
        {
            read_uart1_data01_flag = 1;
        }
        else if(Rx1Value=='\x2A')
        {
            read_uart1_data2A_flag = 1;
        }
        else if(Rx1Value=='\x3A')
        {
            read_uart1_data3A_flag = 1;
        }
        else if(Rx1Value=='\x30')
        {
            read_uart1_data30_flag++;
        }
        else if(Rx1Value=='\x61')
        {
            read_uart1_data61_flag = 1;
        }
        else if(Rx1Value=='\x21')
        {
            read_uart1_data21_flag = 1;
        }
        else if(Rx1Value=='\x10')
        {
            read_uart1_data10_flag = 1;
        }
        else if(Rx1Value=='\x08')
        {
            read_uart1_data08_flag = 1;
        }
        else if(Rx1Value=='K')
        {
            read_uart1_dataK_flag = 1;
        }
        else if(Rx1Value=='P')
        {
            read_uart1_dataP_flag = 1;
        }
        else if(Rx1Value=='B')
        {
            read_uart1_dataB_flag = 1;
        }
        else if(Rx1Value=='\x2B')
        {
            read_uart1_dataPlus_flag = 1;
        }
        else if (Rx1Value=='\x62')
        {
            read_uart1_data62_flag = 1;
        }
        else if (Rx1Value=='\x03')
        {
            read_uart1_data03_flag = 1;
        }
        else if (Rx1Value=='\x66')
        {
            read_uart1_data66_flag = 1;
        }
        else if (Rx1Value=='\x51')
        {
            read_uart1_data51_flag = 1;
        }
        else if (Rx1Value=='\x55')
        {
            read_uart1_data55_flag = 1;
        }
        else if (Rx1Value=='\x14')
        {
            read_uart1_data14_flag = 1;
        }
        else if (Rx1Value==147)
        {
            read_uart1_data93_flag = 1;
        }
                else if (Rx1Value=='\x12')
        {
            read_uart1_data12_flag = 1;
        }
        else if (Rx1Value=='\x26')
        {
            read_uart1_data26_flag = 1;
        }
        else if (Rx1Value=='\x28')
        {
            read_uart1_data28_flag = 1;
        }
        
        if((read_uart1_data10_flag)&&(read_uart1_data21_flag)&&(read_uart1_data93_flag)&&(read_uart1_data01_flag)&&(request_token_flag))//regised token
        {
            request_token_flag = 0;
            token_ready_flag = 1;
            read_uart1_data_flag=0;
//            read_uart1_data21_flag = 0;
//            read_uart1_data10_flag = 0;
//            read_uart1_data30_flag = 0;
//            read_uart1_data01_flag = 0;
//            read_uart1_data61_flag = 0;
//            read_uart1_data08_flag = 0;
//            read_uart1_data93_flag = 0;
            UART1_Flag_reset();
                        notRegistered_flag=0;
        }
        else if((read_uart1_data10_flag)&&(read_uart1_data21_flag)&&(read_uart1_data03_flag)&&(read_uart1_data14_flag)&&(request_token_flag))//not registered
        {
           request_token_flag = 0;
            token_ready_flag = 1;
            read_uart1_data_flag=0;
            read_uart1_data21_flag = 0;
            read_uart1_data10_flag = 0;
            read_uart1_data30_flag = 0;
            read_uart1_data14_flag = 0;
            read_uart1_data61_flag = 0;
            read_uart1_data08_flag = 0;
            notRegistered_flag=1;
            UART1_Flag_reset();
//                Write(W_Addr , 0x24 ,0x02 ) ; //設定Active mode 門檻值0x02
        }
        else if((read_uart1_data10_flag)&&(read_uart1_data21_flag)&&(read_uart1_data03_flag)&&(read_uart1_data12_flag)&&(request_token_flag))//mismach test
        {
           request_token_flag = 0;
            request_token_flag = 1;
            read_uart1_data_flag=0;
            read_uart1_data21_flag = 0;
            read_uart1_data10_flag = 0;
            read_uart1_data30_flag = 0;
            read_uart1_data12_flag = 0;
            read_uart1_data61_flag = 0;
            read_uart1_data08_flag = 0;
            notRegistered_flag=1;
            UART1_Flag_reset();
//                Write(W_Addr , 0x24 ,0x02 ) ; //設定Active mode 門檻值0x02
        }
        else if((read_uart1_data30_flag)&&(read_uart1_data61_flag)&&(read_uart1_data08_flag)&&(request_time_flag))//time
        {
            request_time_flag = 0;
            read_uart1_datatime_ready_flag = 1;
            read_uart1_data_flag=0;
            time_index = 0;
            UART1_Flag_reset();
        }
        else if((request_sensor_flag==1)&&(read_uart1_data10_flag==1)&&(read_uart1_data28_flag==1)&&(read_uart1_data_flag))//gsensor
        {
            read_uart1_data_flag=0;
           request_sensor_flag = 0;
           read_uart1_data10_flag = 0;
           read_uart1_data28_flag = 0;
           gsensor_index=0;
           get_sensor_flag = 1;
           UART1_Flag_reset();
        }
        
        if((token_ready_flag)&&(read_uart1_data_flag))
        {
            read_uart1_data_flag = 0;
            tokentemp[token_index]=Rx1Value;
            if(token_index>=144)
            {
//                read_uart1_dataack_flag = 0;
//                read_uart1_data21_flag = 0;
//                read_uart1_data10_flag = 0;
//                read_uart1_data30_flag = 0;
//                read_uart1_data01_flag = 0;
//                read_uart1_data61_flag = 0;
            read_uart1_data21_flag = 0;
            read_uart1_data10_flag = 0;
            read_uart1_data30_flag = 0;
            read_uart1_data01_flag = 0;
            read_uart1_data61_flag = 0;
            read_uart1_data08_flag = 0;
            read_uart1_data93_flag = 0;
                tokenCS=0;
                token_ready_flag = 0;
                for(token_index=0;token_index<144;token_index++)
                {
                    tokenCS=tokenCS+tokentemp[token_index];
                }
                token_index = 0;     
                tokenCS=tokenCS+0xB5;
                if(tokenCS!=tokentemp[144])
                {
                    tokenCS_error_flag=1;
                }
                else
                {
                    tokenCS_error_flag=0;
                                    get_token_flag=1;
                    for(token_index=0;token_index<144;token_index++)
                    {
                        token[token_index]=tokentemp[token_index];
                    }
                }
                //read_uart1_datatime_ready_flag = 1;
            }
            token_index++;
        }
        else if((read_uart1_datatime_ready_flag)&&(read_uart1_data_flag))
        {
            read_uart1_data_flag = 0;
            time[time_index]=Rx1Value;
            
            if(time_index>=5)
            {
                request_time_flag = 0;
                read_uart1_data21_flag = 0;
                read_uart1_data10_flag = 0;
                read_uart1_data30_flag = 0;
                read_uart1_data01_flag = 0;
                read_uart1_data61_flag = 0;
                time_index = 0;
                read_uart1_datatime_ready_flag = 0;
                get_time_flag=1;
            }
            time_index++;
        }
        else if((get_sensor_flag)&&(read_uart1_data_flag))
        {
            
            read_uart1_data_flag = 0;
            gsensor[gsensor_index]=Rx1Value;
            gsensor_index++;
            if(gsensor_index>=4)
            {
                get_sensor_flag=0;
                unsigned int i=0;
                for(i=0,gsensor_cs=0;i<3;i++)
                {
                    gsensor_cs=gsensor_cs+gsensor[i];
                }
                if((gsensor_cs+0x28)==gsensor[3])
                {
                    get_sensor_CSRflag=1;
                }
                else
                {
                    get_sensor_CSRflag=0;
                }
            }
        }
        
        
        
        
//        if((read_uart1_data30_flag)&(read_uart1_data61_flag)&(read_uart1_datatime_flag))
//        {
//            //read_uart1_datatime_flag = 1;
//            time[time_index]=Rx1Value;
//            time_index++;
//        }
        
    IFS0bits.U1RXIF = false;
   
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void )
{
    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0;
    }

    IFS4bits.U1EIF = false;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART1_Read( void)
{
    uint8_t data = 0;

    data = *uart1_obj.rxHead;

    uart1_obj.rxHead++;

    if (uart1_obj.rxHead == (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart1_obj.rxHead = uart1_rxByteQ;
    }

    if (uart1_obj.rxHead == uart1_obj.rxTail)
    {
        uart1_obj.rxStatus.s.empty = true;
    }

    uart1_obj.rxStatus.s.full = false;

    return data;
}


unsigned int UART1_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart1_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART1_Read () ;
        }
    }

    return numBytesRead ;
}



void UART1_Write( const uint8_t byte)
{
    IEC0bits.U1TXIE = false;
    
    *uart1_obj.txTail = byte;

    uart1_obj.txTail++;
    
    if (uart1_obj.txTail == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart1_obj.txTail = uart1_txByteQ;
    }

    uart1_obj.txStatus.s.empty = false;

    if (uart1_obj.txHead == uart1_obj.txTail)
    {
        uart1_obj.txStatus.s.full = true;
    }

    IEC0bits.U1TXIE = true ;
	
}


unsigned int UART1_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart1_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART1_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}


UART1_TRANSFER_STATUS UART1_TransferStatusGet (void )
{
    UART1_TRANSFER_STATUS status = 0;

    if(uart1_obj.txStatus.s.full)
    {
        status |= UART1_TRANSFER_STATUS_TX_FULL;
    }

    if(uart1_obj.txStatus.s.empty)
    {
        status |= UART1_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart1_obj.rxStatus.s.full)
    {
        status |= UART1_TRANSFER_STATUS_RX_FULL;
    }

    if(uart1_obj.rxStatus.s.empty)
    {
        status |= UART1_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART1_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}


uint8_t UART1_Peek(uint16_t offset)
{
    if( (uart1_obj.rxHead + offset) > (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart1_rxByteQ[offset - (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH - uart1_obj.rxHead)];
    }
    else
    {
      return *(uart1_obj.rxHead + offset);
    }
}


unsigned int UART1_ReceiveBufferSizeGet(void)
{
    if(!uart1_obj.rxStatus.s.full)
    {
        if(uart1_obj.rxHead > uart1_obj.rxTail)
        {
            return(uart1_obj.rxHead - uart1_obj.rxTail);
        }
        else
        {
            return(UART1_CONFIG_RX_BYTEQ_LENGTH - (uart1_obj.rxTail - uart1_obj.rxHead));
        } 
    }
    return 0;
}


unsigned int UART1_TransmitBufferSizeGet(void)
{
    if(!uart1_obj.txStatus.s.full)
    { 
        if(uart1_obj.txHead > uart1_obj.txTail)
        {
            return(uart1_obj.txHead - uart1_obj.txTail);
        }
        else
        {
            return(UART1_CONFIG_TX_BYTEQ_LENGTH - (uart1_obj.txTail - uart1_obj.txHead));
        }
    }
    return 0;
}


bool UART1_ReceiveBufferIsEmpty (void)
{
    return(uart1_obj.rxStatus.s.empty);
}


bool UART1_TransmitBufferIsFull(void)
{
    return(uart1_obj.txStatus.s.full);
}


UART1_STATUS UART1_StatusGet (void)
{
    return U1STA;
}

void putsUART1_2(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    while(*temp_ptr != '\0')	 //72us
    {
        while(U1STAbits.UTXBF);  /* wait if the buffer is full */
        U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */		
//		TEST_IO=!TEST_IO;		
    }  
}

void putsUART1_2package(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    while(*temp_ptr != '\xff')	 //72us
    {
        while(U1STAbits.UTXBF);  /* wait if the buffer is full */
        U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */		
//		TEST_IO=!TEST_IO;		
    }  
}


void putsUART1_2package77(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    while(*temp_ptr != '\x77')	 //72us
    {
        while(U1STAbits.UTXBF);  /* wait if the buffer is full */
        U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */		
//		TEST_IO=!TEST_IO;		
    }  
}

void BT_Send_String(unsigned char *buffer,unsigned int count)
{
unsigned int i=0;
	for(i=0;i<count;i++)
	{
	while (U1STAbits.UTXBF);
    U1TXREG=buffer[i];
	}
}

void UART1_Flag_reset()
{
read_uart1_data_flag = 0;
read_uart1_dataO_flag = 0;
read_uart1_dataK_flag = 0;
read_uart1_dataP_flag = 0;
read_uart1_dataB_flag = 0;
read_uart1_dataN_flag = 0;
read_uart1_dataI_flag = 0;
read_uart1_dataM_flag = 0;
read_uart1_dataQ_flag = 0;
read_uart1_dataY_flag = 0;
read_uart1_dataW_flag = 0;
read_uart1_data01_flag = 0;
read_uart1_data2A_flag = 0;
read_uart1_data3A_flag = 0;
read_uart1_data61_flag = 0;
read_uart1_data21_flag = 0;
read_uart1_data10_flag = 0;
read_uart1_data08_flag = 0;
read_uart1_dataPlus_flag = 0;
read_uart1_data30_flag = 0;//may overflow using++
read_uart1_data301_flag = 0;
read_uart1_data62_flag = 0;
read_uart1_data03_flag = 0;
read_uart1_data66_flag = 0;
read_uart1_data51_flag = 0;
read_uart1_data55_flag = 0;
read_uart1_data14_flag = 0;
read_uart1_data93_flag = 0;    
read_uart1_data12_flag = 0;    
read_uart1_data26_flag = 0;    
}

/**
  End of File
*/
