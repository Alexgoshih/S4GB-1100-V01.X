/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.h

  @Summary:
    This is the generated driver implementation file for setting up the
    interrupts using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description:
    This source file provides implementations for PIC24 / dsPIC33 / PIC32MM MCUs interrupts.
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - pic24-dspic-pic32mm : 1.53.0.1
        Device            :  dsPIC33EP128GM604
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.33
        MPLAB             :  MPLAB X v4.05
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
    Section: Includes
*/
#include <xc.h>

/**
    void INTERRUPT_Initialize (void)
*/
void INTERRUPT_Initialize (void)
{
    //    UERI: UART2 Error
    //    Priority: 1
        IPC16bits.U2EIP = 1;
    //    UTXI: UART2 Transmitter
    //    Priority: 1
        IPC7bits.U2TXIP = 1;
    //    URXI: UART2 Receiver
    //    Priority: 1
        IPC7bits.U2RXIP = 1;
    //    UERI: UART3 Error
    //    Priority: 1
        IPC20bits.U3EIP = 1;
    //    UTXI: UART3 Transmitter
    //    Priority: 1
        IPC20bits.U3TXIP = 1;
    //    URXI: UART3 Receiver
    //    Priority: 1
        IPC20bits.U3RXIP = 1;
    //    UERI: UART4 Error
    //    Priority: 1
        IPC21bits.U4EIP = 1;
    //    UTXI: UART4 Transmitter
    //    Priority: 1
        IPC22bits.U4TXIP = 1;
    //    URXI: UART4 Receiver
    //    Priority: 1
        IPC22bits.U4RXIP = 1;
    //    CI: ECAN1 Event
    //    Priority: 1
        IPC8bits.C1IP = 1;
    //    CRXI: ECAN1 Receive Data  Ready
    //    Priority: 1
        IPC8bits.C1RXIP = 1;
    //    UERI: UART1 Error
    //    Priority: 1
        IPC16bits.U1EIP = 1;
    //    UTXI: UART1 Transmitter
    //    Priority: 1
        IPC3bits.U1TXIP = 1;
    //    URXI: UART1 Receiver
    //    Priority: 1
        IPC2bits.U1RXIP = 1;
    //    INT0I: External Interrupt 0
    //    Priority: 1
        IPC0bits.INT0IP = 1;
    //    INT3I: External Interrupt 3
    //    Priority: 1
        IPC13bits.INT3IP = 1;
    //    INT4I: External Interrupt 4
    //    Priority: 1
        IPC13bits.INT4IP = 1;
    //    INT1I: External Interrupt 1
    //    Priority: 1
        IPC5bits.INT1IP = 1;
    //    INT2I: External Interrupt 2
    //    Priority: 1
        IPC7bits.INT2IP = 1;
    //    MICI: I2C1 Master Events
    //    Priority: 1
        IPC4bits.MI2C1IP = 1;
    //    SICI: I2C1 Slave Events
    //    Priority: 1
        IPC4bits.SI2C1IP = 1;
    //    DMA1I: DMA Channel 1
    //    Priority: 1
        IPC3bits.DMA1IP = 1;
    //    TI: Timer 2
    //    Priority: 1
        IPC1bits.T2IP = 1;
    //    TI: Timer 1
    //    Priority: 1
        IPC0bits.T1IP = 1;
}

