/**
  UART2 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart2.c

  @Summary 
    This is the generated source file for the UART2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for UART2. 
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

#include "uart2.h"
#include "pin_manager.h"
int g=0;
unsigned char RxValue;
unsigned char Rx2Value;
unsigned int read_uart_data_flag = 0;
unsigned int read_uart_start_flag = 0;

extern unsigned char lon2[11];
extern unsigned char lat2[10];
extern unsigned char speed2[10];
extern unsigned char angle2[6];
extern unsigned int open_socket_error_flag;
extern uint8_t time[6],gpsyear[2],gpsmonth[2],gpsdate[2],gpshour[2],gpsminutes[2],gpssecond[2],gpstime[6];

int gps_package_index=0;
int gps_count=0;
unsigned int read_gps_flag = 0;
unsigned int read_gps_Available_flag = 0;
unsigned int gps_one_sflag = 0;
unsigned char gps_package[70];

unsigned int read_uart2_data_flag = 0;
unsigned int read_uart2_dataMO_flag = 0;
unsigned int read_uart2_dataN_flag = 0;
unsigned int read_uart2_dataG_flag = 0;
unsigned int read_uart2_start_flag = 0;
unsigned int read_uart2_dataM_flag = 0;
unsigned int read_uart2_dataR_flag = 0;
unsigned int read_uart2_dataY_flag = 0;
unsigned int read_uart2_dataC_flag = 0;
unsigned int read_uart2_data01_flag = 0;
unsigned int read_uart2_data0A_flag = 0;
unsigned int read_uart2_data0D_flag = 0;

extern unsigned int send_image_flag;
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

static UART_OBJECT uart2_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART2_CONFIG_TX_BYTEQ_LENGTH 8
#define UART2_CONFIG_RX_BYTEQ_LENGTH 8

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart2_txByteQ[UART2_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart2_rxByteQ[UART2_CONFIG_RX_BYTEQ_LENGTH] ;

/**
  Section: Driver Interface
*/

void UART2_Initialize(void)
{
    // Set the UART2 module to the options selected in the user interface.

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U2MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U2STA = 0x0;
    // BaudRate = 9600; Frequency = 58982400 Hz; BRG 1535; 
    U2BRG = 0x5FF;
   IEC1bits.U2RXIE = 1;

   //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
  
   U2MODEbits.UARTEN = 1;  // enabling UARTEN bit
   U2STAbits.UTXEN = 1; 
 
    
   uart2_obj.txHead = uart2_txByteQ;
   uart2_obj.txTail = uart2_txByteQ;
   uart2_obj.rxHead = uart2_rxByteQ;
   uart2_obj.rxTail = uart2_rxByteQ;
   uart2_obj.rxStatus.s.empty = true;
   uart2_obj.txStatus.s.empty = true;
   uart2_obj.txStatus.s.full = false;
   uart2_obj.rxStatus.s.full = false;
}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2TXInterrupt ( void )
{ 
    if(uart2_obj.txStatus.s.empty)
    {
        IEC1bits.U2TXIE = false;
        return;
    }

    IFS1bits.U2TXIF = false;

    while(!(U2STAbits.UTXBF == 1))
    {

        U2TXREG = *uart2_obj.txHead;

        uart2_obj.txHead++;

        if(uart2_obj.txHead == (uart2_txByteQ + UART2_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart2_obj.txHead = uart2_txByteQ;
        }

        uart2_obj.txStatus.s.full = false;

        if(uart2_obj.txHead == uart2_obj.txTail)
        {
            uart2_obj.txStatus.s.empty = true;
            break;
        }
    }
}
unsigned int ReadUART2(void) __attribute__ ((section (".libperi")));


void _ISR _U2RXInterrupt(void)
{
    RxValue = U2RXREG;
    Rx2Value = U2RXREG;
    read_uart2_data_flag = 1;
        if(Rx2Value=='$')
        {
            read_uart2_dataMO_flag = 1;
        }
        else if(Rx2Value=='N')
        {
            read_uart2_dataN_flag = 1;
        }
        else if(Rx2Value=='G')
        {
            read_uart2_dataG_flag = 1;
        }
        else if(Rx2Value=='M')
        {
            read_uart2_dataM_flag = 1;
        }
        else if(Rx2Value=='R')
        {
            read_uart2_dataR_flag = 1;
        }
        else if(Rx2Value=='Y')
        {
            read_uart2_dataY_flag = 1;
        }
        else if(Rx2Value=='C')
        {
            read_uart2_dataC_flag = 1;
        }
        else if(Rx2Value=='\x01')
        {
            read_uart2_data01_flag = 1;
        }
        else if(Rx2Value=='\x0A')
        {
            read_uart2_data0A_flag = 1;
        }
        else if(Rx2Value=='\x0D')
        {
            read_uart2_data0D_flag = 1;
        }
    if((read_uart2_dataMO_flag)&(read_uart2_dataG_flag)&(read_uart2_dataN_flag)&(read_uart2_dataR_flag)&(read_uart2_dataM_flag)&(read_uart2_dataC_flag))
    {
        read_uart2_dataMO_flag=0;
        read_uart2_dataG_flag=0;
        read_uart2_dataN_flag=0;
        read_uart2_dataR_flag=0;
        read_uart2_dataM_flag=0;
        read_uart2_dataC_flag=0;
        read_uart2_data0D_flag=0;
        read_uart2_data0A_flag=0;
        read_gps_flag=1;
        gps_one_sflag=1;
        gps_package_index=0;
        //gps_count++;
        D10_Toggle();//testPWM//
        
        int g=0;
    if((gps_package[12]=='A')&&(send_image_flag==0))//&&(send_image_flag==0)
    {
        read_gps_Available_flag=1;
        for(g=0;g<9;g++)
        {
            lat2[g]=gps_package[14+g];
        }
        for(g=0;g<10;g++)
        {
            lon2[g]=gps_package[27+g];
        }
        for(g=0;g<3;g++)
        {
            speed2[g]=gps_package[41+g];
        }
        
        if(gps_package[63]=='A')//check length to get the right angle
        {
            for(g=0;g<6;g++)
            {
                angle2[g]=gps_package[47+g];
            }
            for(g=0;g<2;g++)
            {
                gpsdate[g]=gps_package[54+g];
            }
            for(g=0;g<2;g++)
            {
                gpsmonth[g]=gps_package[56+g];
            }
            for(g=0;g<2;g++)
            {
                gpsyear[g]=gps_package[58+g];
            }
            
        }
        else
        {
            
            for(g=0;g<2;g++)
            {
                gpsdate[g]=gps_package[48+g];
            }
            for(g=0;g<2;g++)
            {
                gpsmonth[g]=gps_package[50+g];
            }
            for(g=0;g<2;g++)
            {
                gpsyear[g]=gps_package[52+g];
            }
        }
        
//        for(g=0;g<6;g++)
//        {
//            angle2[g]=gps_package[47+g];
//        }
//        for(g=0;g<2;g++)
//        {
//            gpsdate[g]=gps_package[54+g];
//        }
//        for(g=0;g<2;g++)
//        {
//            gpsmonth[g]=gps_package[56+g];
//        }
//        for(g=0;g<2;g++)
//        {
//            gpsyear[g]=gps_package[58+g];
//        }
                for(g=0;g<2;g++)
        {
            gpshour[g]=gps_package[2+g];
        }
                for(g=0;g<2;g++)
        {
            gpsminutes[g]=gps_package[4+g];
        }
                for(g=0;g<2;g++)
        {
            gpssecond[g]=gps_package[6+g];
        }
        for(g=0;g<6;g++)
        {
            gpstime[g]=0;
        }
        gpstime[0]=gpsyear[0]*10+gpsyear[1]-0x10;
        gpstime[1]=gpsmonth[0]*10+gpsmonth[1]-0x10;
        gpstime[2]=gpsdate[0]*10+gpsdate[1]-0x10;
        gpstime[3]=gpshour[0]*10+gpshour[1]-0x10;
        gpstime[4]=gpsminutes[0]*10+gpsminutes[1]-0x10;
        gpstime[5]=gpssecond[0]*10+gpssecond[1]-0x10;
        
        for(g=0;g<70;g++)
        {
            gps_package[g]=0;
        }
    }
    else if((gps_package[12]=='V')&&(send_image_flag==0))
    {
        read_gps_Available_flag=0;
    }
        
//        if(open_socket_error_flag==1)
//        {
//            D10_Toggle();
//        }
        
        time[5]++;
        if(time[5]>=60)//time
        {
            time[5]=0;
            time[4]++;
        }
        else if(time[4]>=60)
        {
            time[4]=0;
            time[3]++;
        }
        else if(time[3]>=24)
        {
            time[3]=0;
            time[2]++;
        }
        else if(time[2]>=32)
        {
            time[2]=0;
            time[1]++;
        }
        else if(time[1]>=13)
        {
            time[1]=0;
            time[0]++;
        }
        
        
    }
    if((read_uart2_data0D_flag)&(read_uart2_data0A_flag))
    {
        read_uart2_dataMO_flag=0;
        read_uart2_dataG_flag=0;
        read_uart2_dataN_flag=0;
        read_uart2_dataR_flag=0;
        read_uart2_dataM_flag=0;
        read_uart2_dataC_flag=0;
        read_uart2_data0D_flag=0;
        read_uart2_data0A_flag=0;
        read_gps_flag=0;
        //D10_Toggle();
    }
    if(read_gps_flag==1)
    {
        
        //D11_Toggle();
        gps_package[gps_package_index]=Rx2Value;
        gps_package_index++;
//        time[5]++;
//        if(time[5]>=60)//time
//        {
//            time[5]=0;
//            time[4]++;
//        }
//        else if(time[4]>=60)
//        {
//            time[4]=0;
//            time[3]++;
//        }
//        else if(time[3]>=24)
//        {
//            time[3]=0;
//            time[2]++;
//        }
//        else if(time[2]>=32)
//        {
//            time[2]=0;
//            time[1]++;
//        }
//        else if(time[1]>=12)
//        {
//            time[1]=0;
//            time[0]++;
//        }
        
//        if(open_socket_error_flag==1)
//        {
//            D10_Toggle();
//        }
        
        
        //for(g=0;g<6;g++)
        //{
        //    if(time[g]==0)
        //        time[g]=1;
        //}
    }
//    if((gps_package[13]=='A')&(gps_package_index==60))
//    {
//        for(g=0;g<9;g++)
//        {
//            lat2[g]=gps_package[15+g];
//        }
//        for(g=0;g<10;g++)
//        {
//            lon2[g]=gps_package[27+g];
//        }
//        for(g=0;g<3;g++)
//        {
//            speed2[g]=gps_package[40+g];
//        }
//    }
    
	IFS1bits.U2RXIF = 0;	// clear interrupt flag
	//RxValue = ReadUART2() ;
            
 //   if(RxValue=='$')
 //       read_uart_start_flag = 1;
//	read_uart_data_flag = 1;ask for gps
//	WriteUART2(RxValue);

}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2ErrInterrupt ( void )
{
    if ((U2STAbits.OERR == 1))
    {
        U2STAbits.OERR = 0;
    }

    IFS4bits.U2EIF = false;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART2_Read( void)
{
    uint8_t data = 0;

    data = *uart2_obj.rxHead;

    uart2_obj.rxHead++;

    if (uart2_obj.rxHead == (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart2_obj.rxHead = uart2_rxByteQ;
    }

    if (uart2_obj.rxHead == uart2_obj.rxTail)
    {
        uart2_obj.rxStatus.s.empty = true;
    }

    uart2_obj.rxStatus.s.full = false;

    return data;
}


unsigned int UART2_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart2_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART2_Read () ;
        }
    }

    return numBytesRead ;
}



void UART2_Write( const uint8_t byte)
{
    IEC1bits.U2TXIE = false;
    
    *uart2_obj.txTail = byte;

    uart2_obj.txTail++;
    
    if (uart2_obj.txTail == (uart2_txByteQ + UART2_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart2_obj.txTail = uart2_txByteQ;
    }

    uart2_obj.txStatus.s.empty = false;

    if (uart2_obj.txHead == uart2_obj.txTail)
    {
        uart2_obj.txStatus.s.full = true;
    }

    IEC1bits.U2TXIE = true ;
	
}


unsigned int UART2_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart2_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART2_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}


UART2_TRANSFER_STATUS UART2_TransferStatusGet (void )
{
    UART2_TRANSFER_STATUS status = 0;

    if(uart2_obj.txStatus.s.full)
    {
        status |= UART2_TRANSFER_STATUS_TX_FULL;
    }

    if(uart2_obj.txStatus.s.empty)
    {
        status |= UART2_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart2_obj.rxStatus.s.full)
    {
        status |= UART2_TRANSFER_STATUS_RX_FULL;
    }

    if(uart2_obj.rxStatus.s.empty)
    {
        status |= UART2_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART2_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}


uint8_t UART2_Peek(uint16_t offset)
{
    if( (uart2_obj.rxHead + offset) > (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart2_rxByteQ[offset - (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH - uart2_obj.rxHead)];
    }
    else
    {
      return *(uart2_obj.rxHead + offset);
    }
}


unsigned int UART2_ReceiveBufferSizeGet(void)
{
    if(!uart2_obj.rxStatus.s.full)
    {
        if(uart2_obj.rxHead > uart2_obj.rxTail)
        {
            return(uart2_obj.rxHead - uart2_obj.rxTail);
        }
        else
        {
            return(UART2_CONFIG_RX_BYTEQ_LENGTH - (uart2_obj.rxTail - uart2_obj.rxHead));
        } 
    }
    return 0;
}


unsigned int UART2_TransmitBufferSizeGet(void)
{
    if(!uart2_obj.txStatus.s.full)
    { 
        if(uart2_obj.txHead > uart2_obj.txTail)
        {
            return(uart2_obj.txHead - uart2_obj.txTail);
        }
        else
        {
            return(UART2_CONFIG_TX_BYTEQ_LENGTH - (uart2_obj.txTail - uart2_obj.txHead));
        }
    }
    return 0;
}


bool UART2_ReceiveBufferIsEmpty (void)
{
    return(uart2_obj.rxStatus.s.empty);
}


bool UART2_TransmitBufferIsFull(void)
{
    return(uart2_obj.txStatus.s.full);
}


UART2_STATUS UART2_StatusGet (void)
{
    return U2STA;
}



/**
  End of File
*/
