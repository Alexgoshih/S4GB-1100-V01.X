/**
  EXT_INT Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    ext_int.c

  @Summary
    This is the generated driver implementation file for the EXT_INT 
    driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description:
    This source file provides implementations for driver APIs for EXT_INT. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - pic24-dspic-pic32mm : 1.53.0.1
        Device            :  dsPIC33EP128GM604
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.33
        MPLAB             :  MPLAB X v4.05
*/

/**
   Section: Includes
 */
#include <xc.h>
#include "ext_int.h"
#include "pin_manager.h"

extern unsigned int g_sensor_toggle_flag;
unsigned int accToggle_flag=0;
unsigned int test_accToggle_flag=0;
extern unsigned int makeBATTERYpackage_flag;
extern unsigned char send10MFlag;
extern unsigned char Car_information_patch_1[33];

extern unsigned int lowBattery_flag;
extern unsigned int toggleWake_flag;

//***User Area Begin->code: Add External Interrupt handler specific headers 

//***User Area End->code: Add External Interrupt handler specific headers

/**
   Section: External Interrupt Handlers
 */
/**
  Interrupt Handler for EX_INT1 - INT1
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT1Interrupt(void)
{
    //***User Area Begin->code: External Interrupt 1***
    send10MFlag=1;
    toggleWake_flag=1;
    accToggle_flag=1;
    test_accToggle_flag=1;
    //***User Area End->code: External Interrupt 1***
    EX_INT1_InterruptFlagClear();
}
/**
  Interrupt Handler for EX_INT2 - INT2
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT2Interrupt(void)
{
    //***User Area Begin->code: External Interrupt 2***
makeBATTERYpackage_flag=1;
            lowBattery_flag=1;
            toggleWake_flag=1;
    //***User Area End->code: External Interrupt 2***
    EX_INT2_InterruptFlagClear();
}
/**
  Interrupt Handler for EX_INT0 - INT0
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT0Interrupt(void)
{
    //***User Area Begin->code: External Interrupt 0***
    send10MFlag=1;
//    CALL_DVR_SetHigh();//12/20
    toggleWake_flag=1;
    g_sensor_toggle_flag=1;
    
    
//    if((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03))
//    {
//            send10MFlag=1;
////    CALL_DVR_SetHigh();//12/20
//    toggleWake_flag=1;
//    g_sensor_toggle_flag=1;
//    }
    
//    CALL_DVR_SetHigh();
    //***User Area End->code: External Interrupt 0***
    EX_INT0_InterruptFlagClear();
}
/**
  Interrupt Handler for EX_INT3 - INT3
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT3Interrupt(void)
{
    //***User Area Begin->code: External Interrupt 3***

    //***User Area End->code: External Interrupt 3***
    EX_INT3_InterruptFlagClear();
}
/**
  Interrupt Handler for EX_INT4 - INT4
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT4Interrupt(void)
{
    //***User Area Begin->code: External Interrupt 4***

    //***User Area End->code: External Interrupt 4***
    EX_INT4_InterruptFlagClear();
}
/**
    Section: External Interrupt Initializers
 */
/**
    void EXT_INT_Initialize(void)

    Initializer for the following external interrupts
    INT1
    INT2
    INT0
    INT3
    INT4
*/
void EXT_INT_Initialize(void)
{
    /*******
     * INT1
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT1_InterruptFlagClear();   
    EX_INT1_PositiveEdgeSet();
    EX_INT1_InterruptEnable();
    /*******
     * INT2
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT2_InterruptFlagClear();   
    EX_INT2_NegativeEdgeSet();
    EX_INT2_InterruptEnable();
    /*******
     * INT0
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT0_InterruptFlagClear();   
    EX_INT0_PositiveEdgeSet();
    EX_INT0_InterruptEnable();
    /*******
     * INT3
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT3_InterruptFlagClear();   
    EX_INT3_PositiveEdgeSet();
    EX_INT3_InterruptEnable();
    /*******
     * INT4
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT4_InterruptFlagClear();   
    EX_INT4_PositiveEdgeSet();
    EX_INT4_InterruptEnable();
}
