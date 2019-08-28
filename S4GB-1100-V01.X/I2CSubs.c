
#include	<I2C.h>
#include 	"I2CSubs.h"
#include "mcc_generated_files/mcc.h"

#define		REPORT_NACK			{	\
									IFS1bits.MI2C1IF = 0 ; \
									StopI2C( ) ; \
									while ( I2CCONbits.PEN ) ;	\
									return -1 ; \
								} 

unsigned char  I2C_ACKPolling(unsigned char CMD)
{
		unsigned char ACK_Result ;

		IdleI2C1( ) ;
		StartI2C1( ) ;

		while ( I2C1CONbits.SEN) ;				// Send START bit ! SEN will be clear automatically once START bits completed 

		IFS1bits.MI2C1IF = 0 ;
		MasterWriteI2C1((CMD & 0xFF)) ;
		while(! IFS1bits.MI2C1IF ) ;				// MI2CIF will be clear after data transmitted completely !

			if ( I2C1STATbits.ACKSTAT ) 		ACK_Result =  1 ;
			else							ACK_Result =  0 ;

		IFS1bits.MI2C1IF = 0 ;
		StopI2C1( ) ;
		while ( I2C1CONbits.PEN ) ;
		IFS1bits.MI2C1IF = 0 ;	

		return		ACK_Result ;
}

void	Init_I2C(void)
{
	unsigned int 	config1 , config2 ;

	config2 = 149;   //143 100k   33 400k
	config1 =	 I2C1_ON & I2C1_IDLE_STOP&I2C1_CLK_REL
				& I2C1_IPMI_DIS & I2C1_7BIT_ADD
				& I2C1_SLW_DIS & I2C1_SM_DIS 
				& I2C1_GCALL_DIS & I2C1_STR_DIS
				& I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS 
				& I2C1_STOP_DIS & I2C1_RESTART_DIS
				& I2C1_START_DIS ;

				ConfigIntI2C1(MI2C1_INT_OFF & MI2C1_INT_PRI_0 ) ;
				OpenI2C1(config1,config2) ;

			//	TRISBbits.TRISC9 = 1 ;
			//	TRISBbits.TRISB8 = 1 ;
 
          //      PORTBbits.RB8 = 0;      
          //      PORTBbits.RB9 = 0;         
   
            //    ODCBbits.ODCB8=1;
 	        //    ODCBbits.ODCB9=1;


	
}

int		Write(unsigned char WriteAdd , unsigned char DataAdd , unsigned char Data  ) 
{
	IdleI2C1( ) ;
	StartI2C1( ) ;

	while ( I2C1CONbits.SEN) ;				// Send START bit ! SEN will be clear automatically once START bits completed 

	IFS1bits.MI2C1IF = 0 ;
	MasterWriteI2C1((WriteAdd)) ;
	while(! IFS1bits.MI2C1IF ) ;				// MI2CIF will be clear after data transmitted completely !

	if ( I2C1STATbits.ACKSTAT ) 		return	255 ; 
	else //ACK						
	{
		IFS1bits.MI2C1IF = 0 ;
		MasterWriteI2C1((DataAdd)) ;				// Mask bit 0 -> Write command 
		while(! IFS1bits.MI2C1IF ) ;

//D10_Toggle();
		if ( I2C1STATbits.ACKSTAT ) 	return	254 ;
		else //ACK						
		{
			IFS1bits.MI2C1IF = 0 ;
			MasterWriteI2C1((Data)) ;					// Set bit 0 for "Read" command
			while(! IFS1bits.MI2C1IF ) ;
	
			if ( I2C1STATbits.ACKSTAT ) 	return	253 ;
			else //ACK						
			{		
				IFS1bits.MI2C1IF = 0 ;		
				StopI2C1( ) ;
				while ( I2C1CONbits.PEN ) ;
				IFS1bits.MI2C1IF = 0 ;	
	
				return	0;
			}
		}
	}
}
unsigned char	Read(unsigned char WriteAdd , unsigned char DataAdd , unsigned char ReadAdd  ) 
{
	unsigned char ACK_Result ;
	unsigned char Temp_Buffer ;

	IdleI2C1( ) ;
	StartI2C1( ) ;
	while ( I2C1CONbits.SEN) ;				// Send START bit ! SEN will be clear automatically once START bits completed 
	IFS1bits.MI2C1IF = 0 ;
	MasterWriteI2C1((WriteAdd)) ;
	while(! IFS1bits.MI2C1IF ) ;				// MI2CIF will be clear after data transmitted completely !

 	if ( I2C1STATbits.ACKSTAT ) 		return	99 ; 
	else //ACK						
	{
		IFS1bits.MI2C1IF = 0 ;
		MasterWriteI2C1((DataAdd)) ;				// Mask bit 0 -> Write command 
		while(! IFS1bits.MI2C1IF ) ;


		if ( I2C1STATbits.ACKSTAT ) 	return	98 ;
		else //ACK						
		{
			IFS1bits.MI2C1IF = 0 ;
			RestartI2C1( ) ;
			while ( I2C1CONbits.RSEN) ;

			IFS1bits.MI2C1IF = 0 ;
			MasterWriteI2C1((ReadAdd)) ;					// Set bit 0 for "Read" command
			while(! IFS1bits.MI2C1IF ) ;


			if ( I2C1STATbits.ACKSTAT ) 	return	97 ;
			else //ACK						
			{
				IFS1bits.MI2C1IF = 0 ;
				Temp_Buffer  = MasterReadI2C1( ) ;

             //   while(! IFS1bits.MI2C1IF ) ;
             //   IFS1bits.MI2C1IF = 0 ;
             //   AckI2C1();
                
				while(! IFS1bits.MI2C1IF ) ;
				
				IFS1bits.MI2C1IF = 0 ;
				NotAckI2C1( ) ;
				while(! IFS1bits.MI2C1IF ) ;

				IFS1bits.MI2C1IF = 0 ;		
				StopI2C1( ) ;
				while ( I2C1CONbits.PEN ) ;
				IFS1bits.MI2C1IF = 0 ;		
				
				return Temp_Buffer;
			}
		}
	}

}	




