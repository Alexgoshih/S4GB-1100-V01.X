/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC24 / dsPIC33 / PIC32MM MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"
#define FCY 58982400UL
#include<libpic30.h>
#include "string.h"                

//#define IOT_DOMAIN      "AT+CIPOPEN=0,\"TCP\",\"kw-4gboxlb.japanwest.cloudapp.azure.com\",18500\r\n"       //142
#define IOT_DOMAIN      "AT+CIPOPEN=0,\"TCP\",\"kw-4gboxlb.japanwest.cloudapp.azure.com\",18600\r\n"       //0116
//#define IOT_DOMAIN      "AT+CIPOPEN=0,\"TCP\",\"www.google.com\",18600\r\n"       //142
//#define IOT_DOMAIN      "AT+CIPOPEN=0,\"TCP\",\"40.70.70.142\",18500\r\n"       //142
//#define auto_mirror_EEP 0x001 	//紀錄後視鏡收折的開關
//#define IOT_DOMAIN      "AT+CIPOPEN=0,\"TCP\",\"kw-4gbox.japanwest.cloudapp.azure.com\",18500\r\n"    //255

unsigned char W_Addr=0xA6;
unsigned char R_Addr=0xA7;
unsigned char Facroty_test_fail_flag=0;
unsigned char canabnormal_flag=0;

void delay( unsigned long i);

uint32_t spi_flash_index=0;

int n=0,j=0,k=0,z=0,test10=1,connectfail=0,testcounter=0;
int wake_flag=0;
int finish_flag=0;
int tokenIsNULL_flag=0;
int sleepGsensorToggle_count=0;
int isABnormalState_flag=0;
uint16_t gx=0x11,gy=0,gz=0,gxh=0,gy0=0,gz0=0,gxl=0,gy1=0,gz1=0,send_TEMP_Index=0;
uint32_t i=0,testimei=0;
uint8_t y1=0,y2=0,temp[256],time[6]={0,0,0,0,0,0},token[145],candata[21],gpsyear[2],gpsmonth[2],gpsdate[2],gpshour[2],gpsminutes[2],gpssecond[2],gpstime[6];
uint8_t can_data[10],spi_flash_H_index,spi_flash_M_index,spi_flash_L_index;
char strx[60];
char stry[60];
char strz[60];
char str_send_TEMP_Index[60];
extern can_flag;
extern unsigned finish1s_flag;
extern unsigned finish10M_flag;
extern unsigned test_finish1M_flag;
extern unsigned int read_gps_Available_flag;
unsigned char NMEA_name[50];
char str[60];
char strSend[60];
unsigned char lat[10];
unsigned char lon[11];
unsigned char lat2[10];
unsigned char lon2[11];
unsigned char speed[6];
unsigned char speed2[3];
unsigned char angle2[6];
unsigned char Car_information_patch_1[33];
unsigned char Car_information_patch_2[21];
unsigned char imei[15];
unsigned char imei2[15];
unsigned char imsi[15];
unsigned char imsi2[15];
unsigned char tempimei[16];
unsigned char tempimsi[16];
unsigned char shortimei[8];
unsigned char shortimsi[8];
unsigned char sendimsiimei[42];
unsigned char sendrssi[14];
unsigned char sendtoken[155];//0810
unsigned char sendtokenACC[157];//0810
unsigned char decodingStatetoken[171];//1214
unsigned char sendexception[12];
unsigned char sendend[11];
unsigned char sendcan[32];
unsigned char sendcan1[45];
unsigned char sendcan2[32];
unsigned char sendASKCAN[14];
unsigned char sendbattery[12];//0810
unsigned char sendgsensor[17];//0810
unsigned char sendgps[35];//0810  31
unsigned char sendgps2[40];//0810  31
unsigned char testMessage[75];
unsigned char testMessage2[10]={'H','E','L','L','O','W','O','R','L','D'};
//unsigned char senddata_buffer[10000];send_TEMP   
unsigned char send_TEMP[1500];
unsigned char  Package_data[36]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x20,0x20,0x20,0x20,0x0D,0x0A,0x20,0x20,0x20,0x20,0x20,0x0D,0xFF};
unsigned char  Test_data[8]={0x30,0x01,0x05,0x7B,0x30,0x01,0x05,0x7B};
unsigned char  Test_data1[4]={0x41,0x54,0x0D,0x0A};



unsigned char rssi[3];
unsigned char send10MFlag=0;
unsigned char toggleWake_flag=1;
unsigned char battery=0x00;
unsigned char test_4G_rest_flag=0;
uCAN1_MSG test,test1;

unsigned int connect_sucessfully_flag=0,open_socket_fail_flag=0,open_socket_error_flag=0,send_spiData_flag=0,dvr_Server_ACK_flag=0,canUnlock_flag=0,makeBATTERYpackage_flag=0,test_canUnlock_flag=0,lowBattery_flag=0;
unsigned int open_Gsensor_flag=0;
unsigned int RE_open_Gsensor_flag=0;
unsigned int clearTimer2CallBack_flag=0,get_sensor_flag=0,request_sensor_flag=0,get_sensor_CSRflag=0;
unsigned int startCanUARTtoggle_flag=0;

extern unsigned int gsensor[7];
unsigned int g_sensor_toggle_flag=0;
unsigned int g_sensor_untoggle_count=0;

//extern temp1;
extern unsigned char RxValue;
extern unsigned char Rx1Value;
extern unsigned int read_uart_data_flag;
extern unsigned int read_uart_start_flag;
extern unsigned int read_uart1_data_flag;
extern unsigned int read_uart1_dataN_flag;
extern unsigned int read_uart1_dataO_flag;
extern unsigned int read_uart1_dataK_flag;
extern unsigned int read_uart1_dataI_flag;
extern unsigned int read_uart1_dataM_flag;
extern unsigned int read_uart1_dataQ_flag;
extern unsigned int read_uart1_dataY_flag;
extern unsigned int read_uart1_dataW_flag;
extern unsigned int read_uart1_dataP_flag;
extern unsigned int read_uart1_dataB_flag;
extern unsigned int read_uart1_dataPlus_flag;
extern bool read_uart1_dataW1_flag;
extern unsigned int read_uart1_data01_flag;
extern unsigned int read_uart1_data10_flag;
extern unsigned int read_uart1_data21_flag;
extern unsigned int read_uart1_data2A_flag;
extern unsigned int read_uart1_data3A_flag;
extern unsigned int read_uart1_data61_flag;
extern unsigned int read_uart1_data62_flag;
extern unsigned int read_uart1_data03_flag;
extern unsigned int read_uart1_data66_flag;
extern unsigned int read_uart1_data51_flag;
extern unsigned int read_uart1_data55_flag;
extern unsigned int read_uart1_data14_flag;
extern unsigned int read_uart1_data30_flag;
extern unsigned int read_uart1_data12_flag;
extern unsigned int request_token_flag;
extern unsigned int request_time_flag;
extern unsigned int read_uart1_start_flag;     
extern unsigned int get_time_flag;
extern unsigned int get_token_flag;
extern unsigned int can_data_flag;
extern unsigned int notRegistered_flag;
extern unsigned int tokenCS_error_flag;

extern unsigned char Rx3Value,Rx4Value;

extern unsigned int read_uart4_data_flag;

extern unsigned int read_uart3_data_flag;

extern unsigned int gps_one_sflag;
unsigned int sipFlash_data_flag,imei_index=0,imsi_index=0;

extern unsigned char gps_package[65];

extern unsigned int token_index,time_index;

extern unsigned int send_image_flag;
extern unsigned int send_data_flag;
extern unsigned int accToggle_flag;
extern unsigned int test_accToggle_flag;

extern int gps_count;
//        unsigned char __attribute__ ((space(eedata))) eeData3[156];//eeprom

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    ECAN1_TransmitEnable();
    ECAN1_ReceiveEnable();
    // When using interrupts, you need to set the Global Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalEnable();
    // Disable the Global Interrupts
    //INTERRUPT_GlobalDisable();
//    spi_flash_H_index=spi_flash_index>>16;
//    spi_flash_M_index=spi_flash_index>>8;
//    spi_flash_L_index=spi_flash_index;
//    TMR1_Start();
//    TMR2_Start();
    open_socket_error_flag=0;
    
    __delay_ms(100);
    Write(W_Addr , 0x24 ,0x05 ) ; //設定Active mode 門檻值0x02
    Write(W_Addr , 0x27 ,0xF0 ) ;
    Write(W_Addr , 0x2E ,0x10 ) ;
    Write(W_Addr , 0x2F ,0x10 ) ;
    Write(W_Addr , 0x2D ,0x08 ) ;
    Write(W_Addr , 0x31 ,0x20 ) ;
    __delay_ms(100);
                        G_sensorReset();
                        
    Car_information_patch_1[26]=0x02;//default lock 
                        
    if((send10MFlag==1)||(ACC_DET_GetValue()==1))
    {
        toggleWake_flag=1;
        wake_flag=1;
    }
    
    while (1)
    {
//        while(1)
//        {
//            if(wake_flag==1)
//            {
//                wake_flag=0;
//                if(ACC_DET_GetValue()==1)
//                {
////                    __delay_ms(3000);
//                }
//                wake();
//            }
//            if(ACC_DET_GetValue()==1)//accon    //ACC_DET_GetValue()==1
//            {
//                           WDT_WatchdogTimerClear();
////            putsUART1_2("AT+CSQ\r\n");//rssi
//                putsUART1_2("AT+VTD?\r\n");//rssi
//                        __delay_ms(100);
//            }
//            else
//            {
//                Sleep1();
//            }
//        }
        
        
        
        SEN_PWR_CTRL_SetHigh();
//        __delay_ms(50);
        
        if((BATT_SEN1_GetValue()==0)&&(BATT_SEN2_GetValue()==0))
        {
            lowBattery_flag=1;
            toggleWake_flag=1;
        }
        if(ACC_DET_GetValue()==1)
        {
            lowBattery_flag=0;
        }
        
        if(++sleepGsensorToggle_count>5)//
        {
            sleepGsensorToggle_count=0;
//            Car_information_patch_1[26]=0x04;
        }
        else if(test_finish1M_flag!=0)
        {
            sleepGsensorToggle_count=0;
        }
    
        
//        if(SEL_MCU_GetValue()==1)//factory test mode 
//        {
//            if(wake_flag==1)
//            {
//                wake_flag=0;
//                wake();
//                for(i=0;(read_uart1_dataB_flag==0)&(i<5000);i++)//prevent stock&get start signal wait 4G module wake up
//                {
//                    __delay_ms(100);
//                }
//            }
//            WDT_WatchdogTimerClear();
//            makeTESTpackage();
//            BT4_Send_String(testMessage,sizeof(testMessage));
//            
//            if(Rx3Value!=0x0)
//            {
//                putsUART1_4("DVR_UART PASS\t");
//            }
//            else
//            {
//                putsUART1_4("DVR_UART FAIL\t");
//            }
//            
//            if(Rx4Value!=0x0)
//            {
//                putsUART1_4("CAN_UART PASS\t");
//            }
//            else
//            {
//                putsUART1_4("CAN_UART FAIL\t");
//            }
//            
//            
//            putsUART1_4("v0.2.0  \r\n");
//            BT3_Send_String(testMessage2,sizeof(testMessage2));
//            __delay_ms(1000);
//            
//        }
        
        if(SEL_MCU_GetValue()==1)//factory test mode 
        {
            __delay_ms(500);
            if(SEL_MCU_GetValue()==1)//factory test mode 
            {
                
                if(ACC_DET_GetValue()==1)
                {
            if(wake_flag==1)
            {
                Facroty_test_fail_flag=0;
                wake_flag=0;
                wake();
                putsUART1_4("waitting 4G");
                for(i=0;(read_uart1_dataB_flag==0)&(i<5000);i++)//prevent stock&get start signal wait 4G module wake up
                {
                    __delay_ms(100);
                    putsUART1_4(".");
                }
                putsUART1_4("\r\n");
                readIMEI2();
                readIMSI();
                readRSSI2();
                readADXL345();
                putsUART1_4("IMEI/IMSI:\t");
                BT4_Send_String(imei,sizeof(imei));
                putsUART1_4("/");
                BT4_Send_String(imsi,sizeof(imsi));
                putsUART1_4("\r\n");
                
                CALL_DVR_SetLow();
                
                if((imei[0]==0)&&(imei[1]==0)&&(imei[2]==0)&&(imei[3]==0)&&(imei[4]==0)&&(imsi[0]==0)&&(imsi[1]==0)&&(imsi[2]==0)&&(imsi[3]==0)&&(imsi[4]==0))
                {
                    putsUART1_4("Ver:v0.2.5\r\n\r\n4G module: NG\r\n");
                    Facroty_test_fail_flag=1;
                }
                else
                {
                    putsUART1_4("Ver:v0.2.5\r\n\r\n4G module: PASS\r\n");
                }
                
                putsUART1_4("\r\n");
                
                if(strx[0]==0x0)
                {
                    putsUART1_4("G-Sensor: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                else
                {
                    putsUART1_4("G-Sensor: PASS\r\n");
                }
                
                putsUART1_4("\r\n");
                
                if(Rx4Value!=0x0)
                {
                    putsUART1_4("CAN_UART: PASS\r\n");
                }
                else
                {
                    putsUART1_4("CAN_UART: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                putsUART1_4("\r\n");
                
                CALL_DVR_SetHigh();
                putsUART1_3("\x20\x02\x05\x01\x01\x02\x09\xA5\xFF");//NEW COMMAND
                
                if(Rx3Value!=0x0)
                {
                    putsUART1_4("DVR_UART: PASS\r\n");
                }
                else
                {
                    putsUART1_4("DVR_UART: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                putsUART1_4("\r\n");
                
                putsUART1_4("Watting G-sensor wakes up");
                
                g_sensor_toggle_flag=0;
                    G_sensorReset();
                for(i=0;(g_sensor_toggle_flag==0)&&(i<100);i++)//prevent stock
                {
                    __delay_ms(100);
                    putsUART1_4(".");
                }
                
                putsUART1_4("\r\n");
                    
                if(g_sensor_toggle_flag==1)
                {
                    putsUART1_4("G-sensor wakes up: PASS\r\n");
                }
                else
                {
                    putsUART1_4("G-sensor wakes up: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                
                
                
                putsUART1_4("\r\n");
                
                putsUART1_4("Watting CAN wakes up");
                
                g_sensor_toggle_flag=0;
                    Write(W_Addr , 0x2E ,0x00 );
                    
                for(i=0;(g_sensor_toggle_flag==0)&&(i<100);i++)//prevent stock
                {
                    __delay_ms(100);
                    putsUART1_4(".");
                }
                
                putsUART1_4("\r\n");    
                
                if(g_sensor_toggle_flag==1)
                {
                    putsUART1_4("Watting CAN wakes up: PASS\r\n");
                }
                else
                {
                    putsUART1_4("Watting CAN wakes up: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                
                EX_INT0_InterruptDisable();
                
                
                
                
                
                
                putsUART1_4("\r\n");
                
                putsUART1_4("Watting Battery Detection");
                for(i=0;((BATT_SEN2_GetValue()==1)||(BATT_SEN1_GetValue()==1))&&(i<100);i++)//prevent stock
                {
                    __delay_ms(100);
                    putsUART1_4(".");
                }
                
                putsUART1_4("\r\n");    
                
                if(BATT_SEN2_GetValue()==0)//battery test 11V
                {
                    putsUART1_4("Battery 10V: PASS\r\n");
                }
                else
                {
                    putsUART1_4("Battery 10V: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                WDT_WatchdogTimerClear();
                
                if(BATT_SEN1_GetValue()==0)//10V
                {
                                    
                    putsUART1_4("Battery 11V: PASS\r\n");
                }
                else
                {
                    putsUART1_4("Battery 11V: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                WDT_WatchdogTimerClear();
                
                
                putsUART1_4("\r\n");
                
                putsUART1_4("Watting GPS.............");
                for(i=0;(gps_package[12]!='A')&&(i<120);i++)//prevent stock
                {
                    __delay_ms(1000);
                    putsUART1_4(".");
                }
                putsUART1_4("\r\n");
                
                if(gps_package[12]=='A')
                {
                    putsUART1_4("GPS: PASS\r\n");
                }
                else
                {
                    putsUART1_4("GPS: NG\r\n");
                                        Facroty_test_fail_flag=1;
                }
                
                putsUART1_4("\r\n***********************************************************************************************\r\n");
                putsUART1_4("***********************************************************************************************\r\n");
                
                if(Facroty_test_fail_flag==1)
                {
                    putsUART1_4("NG\r\n");
                }
                else
                {
                    putsUART1_4("PASS\r\n");
                }
                
                
            }
            
            
                    if((imei[0]==0)&&(imei[1]==0)&&(imei[2]==0)&&(imei[3]==0)&&(imei[4]==0)&&(imsi[0]==0)&&(imsi[1]==0)&&(imsi[2]==0)&&(imsi[3]==0)&&(imsi[4]==0))
                    {
                        __asm__ volatile ("reset");
                    }
            }
                else
                {
                    Sleep1();
                }
                
                
            }
        }
        
        else
        {
//            unsigned char __attribute__ ((space(eedata))) eeData[156];//eeprom
            WDT_WatchdogTimerClear();
            if((send10MFlag==1)||(ACC_DET_GetValue()==1))
            {
                toggleWake_flag=1;
            }
            
            if((wake_flag==1)&&(toggleWake_flag==1))
            {
                wake_flag=0;
//                sleepGsensorToggle_count=0;
//                toggleWake_flag=0;
    //                    testcounter++;
    //        putsUART1_2("\r\n======================================\r\n");
    //            __delay_ms(100);
    //        BT_Send_String(testcounter,sizeof(testcounter));
    //            __delay_ms(100);
    //        putsUART1_2("\r\n======================================\r\n");
//                g_sensor_toggle_flag=1;//======================================================================================================================================10/25
                
                
//                if(g_sensor_toggle_flag==1)
//                {
//                    CALL_DVR_SetHigh();
//                }
                
                
                if(ACC_DET_GetValue()==1)//WAIT CAR COMPUTER
                {
//                    __delay_ms(3000);
                }
                wake();
                canUnlock_flag=0;
                if(accToggle_flag==1)
                {
                spiChipErase();
                }
                WDT_WatchdogtimerSoftwareEnable();//0905
    //             WDT_WatchdogTimerClear();
                 //WDT_WatchdogtimerSoftwareDisable();
    //            putsUART1_2package(eeData);
//                putsUART1_2("\r\n======================================\r\n");
                read_uart1_dataB_flag=0;
                test_canUnlock_flag=1;
                for(i=0;(read_uart1_dataB_flag==0)&&(i<3000);i++)//prevent stock&get start signal wait 4G module wake up
                {
                    __delay_ms(10);
                    if((ACC_DET_GetValue()==0))
                    {   
                        
                        if(read_uart4_data_flag==1)
                        {
//                            read_uart4_data_flag=0;
                            sleepGsensorToggle_count=0;
                        }
                        
                        if(Car_information_patch_1[26]==0xff)//25    Check lock status
                        {
                            canUnlock_flag=1;
                        }
                        else{
                            canUnlock_flag=0;
                        }
                        
                        if(canUnlock_flag==1)
                        {
                            test_canUnlock_flag=0;
                            send10MFlag=0;
                         break;   
                        }
//                        else
//                        {
//                            test_canUnlock_flag=1;
//                        }
                    }
                }
                
//                if(ACC_DET_GetValue()==1)//WAIT CAR COMPUTER
//                {
//                    __delay_ms(3000);
//                }
                
                if(test_canUnlock_flag != 0)
                {
                    do
                    {
                        do
                        {
                            do{
                                sleepGsensorToggle_count=0;
                            tokenCS_error_flag=0;
                            
                        __delay_ms(1000);
                        putsUART1_2("AT+CIPTIMEOUT=3000,3000,3000\r\n");//AT+CDNSGIP=”www.google.com”
                        __delay_ms(1000);
                            
                        WDT_WatchdogTimerClear();
                        makeIMSIIMSIpackage();
                        newIMEIIMSI();
                            }while((ACC_DET_GetValue()==1)&&(get_token_flag==0)&&(notRegistered_flag==0));
                        }while(tokenCS_error_flag==1);
                    }while((open_socket_error_flag==1)&&(ACC_DET_GetValue()==1));

                finish_flag=0;
                }
            }

            
            if((sipFlash_data_flag==1)&&(ACC_DET_GetValue()==1)&&(open_socket_error_flag==0))//send the data stored in the flash
            {
                makeTOKENpackage();
                makeENDpackage();
                makeEXCEPTIONpackageNormal();
    //            sipFlash_data_flag=0;
                sendFlashDataSim();
            }

            if((notRegistered_flag==1)&&(ACC_DET_GetValue()==1))
            {
                    
            }
            else if(ACC_DET_GetValue()==1)//accon
            {
                if(test_accToggle_flag==1)
                {
                    test_accToggle_flag=0;
                    Write(W_Addr , 0x24 ,0x1E );//test 2g
                    G_sensorReset();
                    for(i=0;(i<80)&&(ACC_DET_GetValue()==1);i++)
                    {
                        isABnormalState_flag=0;
                        __delay_ms(1000);
                        WDT_WatchdogTimerClear();
                        putsUART1_2("1S\r\n");
                        if(i==10)
                        {
                            putsUART1_2("10S\r\n");
                        }
                        else if(i==20)
                        {
                            putsUART1_2("20S\r\n");
                        }
                        else if(i==30)
                        {
                            putsUART1_2("30S\r\n");
                        }                        
                        else if(i==40)
                        {
                            putsUART1_2("40S\r\n");
                        } 
                        else if(i==50)
                        {
                            putsUART1_2("50S\r\n");
                        }         
                        else if(i==60)
                        {
                            putsUART1_2("60S\r\n");
                        } 
                        else if(i==70)
                        {
                            putsUART1_2("70S\r\n");
                        }  
                    }
                    g_sensor_toggle_flag=0;
                    G_sensorReset();
                }
                
                isABnormalState_flag=0;
                
                if(gps_count>=5)
                {
                    makeTOKENpackage();
                    makeENDpackage();
                    makeEXCEPTIONpackageNormal();
//                    read_uart4_data_flag=0;

                    gps_count=0;
                    testShortPACKAGE();
                    G_sensorReset();
    //                if(open_socket_error_flag==1)
    //                {
    //                    bufferToFlash();
    //                    sipFlash_data_flag=1;
    //                }
                    
                    if((token[0]==0)&&(token[1]==0)&&(token[2]==0)&&(token[3]==0)&&(token[4]==0)&&(token[5]==0)&&(token[6]==0)&&(token[7]==0)&&(token[8]==0))
                    {
                        __asm__ volatile ("reset");
                    }
                    else if((imei[0]==0)&&(imei[1]==0)&&(imei[2]==0)&&(imei[3]==0)&&(imei[4]==0)&&(imsi[0]==0)&&(imsi[1]==0)&&(imsi[2]==0)&&(imsi[3]==0)&&(imsi[4]==0))
                    {
                        __asm__ volatile ("reset");
                    }
                }
                if (gps_one_sflag==1)
                {
                    gps_one_sflag=0;
                    gps_count++;
                    MakeWrite_All_BufferPackage();
                }
                if(g_sensor_toggle_flag==1)
                {
                    g_sensor_toggle_flag=0;
                    G_sensorReset();
                                    putsUART1_4("4G\r\n");
                    //send Emergency
                    makeTOKENpackage();
                    makeENDpackage();
                    makeEXCEPTIONpackageEmergenceNormal();
                    testShortPACKAGE();
                }
                test_finish1M_flag=0;
            }
            else //gsensor
            {

                if(canUnlock_flag==1)
                {
                    if(g_sensor_toggle_flag==1)
                    {
                        g_sensor_toggle_flag=0;
                        G_sensorReset();
                    }
                }
                
                        if(Car_information_patch_1[26]==0xff)//25    Check lock status
                        {
                            canUnlock_flag=1;
                        }
                        else{
                            canUnlock_flag=0;
                        }
                
            if((token[0]==0)&&(token[1]==0)&&(token[2]==0)&&(token[3]==0)&&(token[4]==0)&&(token[5]==0)&&(token[6]==0)&&(token[7]==0)&&(token[8]==0))//check token
            {
                tokenIsNULL_flag=1;
            }
            else
            {
                tokenIsNULL_flag=0;
            }

                if((g_sensor_toggle_flag==1)&&(rssi[1]!=0x39)&&(notRegistered_flag==0)&&(canUnlock_flag==0)&&((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03)))//gsensor   &&(tokenIsNULL_flag==0)
                {
                    if(Car_information_patch_1[26]==0x00)
                    {
                        putsUART1_2("00000000000000000000000000000000000000000000000000000000000000000000000000");
                        putsUART1_4("00000000000000000000000000000000000000000000000000000000000000000000000000");
                    }
                    else if(Car_information_patch_1[26]==0x01)
                    {
                        putsUART1_2("0101010101010101010101010101010101010101010101010101010101010101010101010101");
                        putsUART1_4("01010101010101010101010101010101001010100101010101001010101010100101010101010");
                    }
                    else if(Car_information_patch_1[26]==0x02)
                    {
                        putsUART1_2("020202020202020202020202020202020202020202020202020202020202020202020202020202");
                        putsUART1_4("222222222222222222222222222222222222222222222222222222222222222222222222222222");
                    }
                    else if(Car_information_patch_1[26]==0x03)
                    {
                        putsUART1_2("03033030303030303030303030303030303030303030303030303030303030303030303030303030303");
                        putsUART1_4("33333333333333333333333333333333333333333333333333333333333333333333333333333333333 ");
                    }
                    else if(Car_information_patch_1[26]==0x04)
                    {
                        putsUART1_2("04040404040404040404040404040404040404040404040404040404040404040404004040404040404");
                    }
                    
                    g_sensor_toggle_flag=0;
                    isABnormalState_flag=1;
                    makeTOKENpackage();
                    makeENDpackage();
//                    makeEXCEPTIONpackageabNormal();
                    if(Car_information_patch_1[26]==0x03)
                    {
                        canabnormal_flag=1;
                        makeEXCEPTIONpackageCANabNormal();
                    }
                    else
                    {
                        canabnormal_flag=0;
                        makeEXCEPTIONpackageabNormal();
                    }
    //                while(1)                                       //unlimate send
    //                {          
    //                    makeEXCEPTIONpackage();                       //unlimate send
    //                    sendDvrphotoSim2();                        //unlimate send
    //                    __delay_ms(500);                          //unlimate send
    //                    WDT_WatchdogTimerClear();                  //unlimate send
    //                }                                              //unlimate send
//                    UART3_Initialize();
                    UART3_Initialize();
                    UART4_Initialize();
                    g_sensor_untoggle_count=0;
                    sendDvrphotoSim();
                    g_sensor_toggle_flag=0;
                    G_sensorReset();

////////////                    while((rssi[1]!=0x39))
////////////                    {
////////////                        WDT_WatchdogTimerClear();
////////////                        if(gps_count>=5)
////////////                        {
////////////                            makeTOKENpackage();
////////////                            makeENDpackage();
////////////                            makeEXCEPTIONpackageabNormal();
////////////                            gps_count=0;
////////////                            testShortPACKAGE();
////////////                            g_sensor_untoggle_count++;
////////////                                            isABnormalState_flag=1;
////////////
////////////                            if(g_sensor_toggle_flag==1)
////////////                            {
////////////                                g_sensor_toggle_flag=0;
////////////                                G_sensorReset();
////////////                                g_sensor_untoggle_count=0;
////////////                            }
////////////                            else
////////////                            {
//////////////                                finish_flag=1;
//////////////                                G_sensorReset();
////////////                                if(g_sensor_untoggle_count>=1)//cycle (5*n+5))
////////////                                {
////////////                                    break;
////////////                                }
////////////                            }
////////////                        }
////////////                        if (gps_one_sflag==1)
////////////                        {
////////////                            gps_one_sflag=0;
////////////                            gps_count++;
////////////                            MakeWrite_All_BufferPackage();
////////////                        }
////////////                    }
                }
                else//check signal to prevent clear data if door status is lock
                {
                    g_sensor_toggle_flag=0;
                }
                finish_flag=1;
            }
            
            if(test_4G_rest_flag>=5)
            {
                test_4G_rest_flag=0;
//                wake_flag==1;
//                toggleWake_flag==1;www.hinet.net
                
                                __delay_ms(1000);
                putsUART1_2("AT+CDNSGIP=\"www.hinet.net\"\r\n");
                read_uart1_dataO_flag = 0;
                read_uart1_dataK_flag = 0;
                __delay_ms(1000);
                
                if((read_uart1_dataO_flag==0)&&(read_uart1_dataK_flag==0))
                {
                    __delay_ms(1000);
                    putsUART1_2("AT+CDNSGIP=\"www.google.com\"\r\n");
                    read_uart1_dataO_flag = 0;
                    read_uart1_dataK_flag = 0;
                    __delay_ms(1000);

                    if((read_uart1_dataO_flag==0)&&(read_uart1_dataK_flag==0))
                    {
                        test_4G_reset();
                        putsUART1_2("********************************************************************************\r\n");
                        putsUART1_2("\t\t RESET____4G\r\n");
                        putsUART1_2("********************************************************************************\r\n");
                    }
                }
            }
            
        
        if((ACC_DET_GetValue()==1)||(g_sensor_toggle_flag==1))//check before sleep
        {
            finish_flag=0;
        }
            
        if((token[0]==0)&&(token[1]==0)&&(token[2]==0)&&(token[3]==0)&&(token[4]==0)&&(token[5]==0)&&(token[6]==0)&&(token[7]==0)&&(token[8]==0))
        {
                        tokenIsNULL_flag=1;
        }
        else
        {
            tokenIsNULL_flag=0;
        }
            
        if((finish_flag==1)&&(notRegistered_flag==0)&&(rssi[1]!=0x39)&&(send10MFlag==1)&&(tokenIsNULL_flag==0)&&1)//10minutes before sleep//open_socket_error_flag&&(rssi[1]!=0x39)
        {
//                            Write(W_Addr , 0x24 ,gsensor[2] );//test 2g
                            
                            
                if(get_sensor_CSRflag==1)
                {
                    get_sensor_flag=0;
                    Write(W_Addr , 0x24 ,gsensor[2] );
                }     
                else
                {
                    Write(W_Addr , 0x24 ,0x05 );
                }
                            
                            
                                UART1_Initialize();
                                
            if(((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03)))
            {
                Write(W_Addr , 0x2E ,0x10 );
            }
            else
            {
                Write(W_Addr , 0x2E ,0x00 );
            }
                               
                                
            putsUART1_2("======================================================================================================================================\r\n");
                        CALL_DVR_SetLow();
                        
                        for(i=0;i<10;i++)
                        {
                            putsUART1_2("AT\r\n");
                            __delay_ms(100);
                        }
//            if(get_sensor_CSRflag==1)
//            {
//                get_sensor_flag=0;
//    //                    gsensor[3]=0x0f;
//                Write(W_Addr , 0x24 ,gsensor[3]+0x03 );
//            }            
            G_sensorReset();
//            send10MFlag=0;//
            clear10Mtimer();
            finish10M_flag=0;
            while((test_finish1M_flag<9))//open_socket_error_flag
            {
                WDT_WatchdogTimerClear();
                    if(gps_count>=5)
                    {
//                        read_uart4_data_flag=0;
                        if(Car_information_patch_1[26]==0xff)//25    Check lock status
                        {
                            canUnlock_flag=1;
                        }
                        else{
                            canUnlock_flag=0;
                        }
                        G_sensorReset();
                        makeTOKENpackage();
                        makeENDpackage();
                        if(isABnormalState_flag==1)
                        {
                            if(canabnormal_flag==1)
                            {
                                makeEXCEPTIONpackageCANabNormal();
                            }
                            else
                            {
                                makeEXCEPTIONpackageabNormal();
                            }
                        }
                        else
                        {
                            makeEXCEPTIONpackageNormal();
                        }
                        gps_count=0;
                        testShortPACKAGE();
//                    if((token[0]==0)&&(token[1]==0)&&(token[2]==0))
//                    {
//                        break;
//                        wake_flag==1;
//                                toggleWake_flag==1;
//                                finish_flag=0;
//                    }
//                        if(rssi[1]==0x39)
//                        {
//                            break;
//                        }
//                        g_sensor_toggle_flag=0;
                    }
                    if (gps_one_sflag==1)
                    {
                        gps_one_sflag=0;
                        gps_count++;
                        CALL_DVR_SetLow();//clear dvr status
                        putsUART1_2("AT\r\n");
                        read_uart1_dataO_flag=0;
                        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                            {
                                __delay_ms(1);
                            }
                        if(read_uart1_dataO_flag==0)
                        {
                            
                        }
                        
                        MakeWrite_All_BufferPackage();
                        
                        
//                        if(((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03)||(Car_information_patch_1[26]==0x00)))
//                        {
//                            open_Gsensor_flag=1;
//                            __delay_ms(5000);
//                            Write(W_Addr , 0x2E ,0x10 );
//                        }
//                        else
//                        {
//                            Write(W_Addr , 0x2E ,0x00 );
//                        }
                        
                        
                        
                        if(((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03)||(Car_information_patch_1[26]==0x00))&&(RE_open_Gsensor_flag==1))
                        {
                            RE_open_Gsensor_flag=0;
                            open_Gsensor_flag=1;
                        }
                        else if((Car_information_patch_1[26]==0xff)||(Car_information_patch_1[26]==0x01))
                        {
//                            Write(W_Addr , 0x2E ,0x00 );
                            Write(W_Addr , 0x24 ,0xff );
                            RE_open_Gsensor_flag=1;
                            open_Gsensor_flag=0;
                        }
                        
                        if(open_Gsensor_flag==1)
                        {
                            __delay_ms(5000);
                            
                            if(get_sensor_CSRflag==1)
                            {
                                get_sensor_flag=0;
                                Write(W_Addr , 0x24 ,gsensor[2] );
                            }     
                            else
                            {
                                Write(W_Addr , 0x24 ,0x05 );
                            }
//                            Write(W_Addr , 0x2E ,0x10 );
                        }

                        
                        
                        
                        
                    }
                                            if((g_sensor_toggle_flag==1)&&((Car_information_patch_1[26]==0x02)||(Car_information_patch_1[26]==0x03)))//don't send photo when in 10m
                                            {
//                                                CALL_DVR_SetHigh();
//                                                __delay_ms(3000);
                                                if(isABnormalState_flag==0)
                                                {
//                                                     read_uart4_data_flag=1;   
                                                    
                                                    __delay_ms(3000);
                                                    break;
                                                }
                                            }
                                            else
                                            {
                                                g_sensor_toggle_flag=0;
                                            }
                                            if (ACC_DET_GetValue()==1)
                                            {
                                                clearTimer2CallBack_flag=1;
                                                break;
                                            }
                
                                            if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                                            {
                                                CheckBatteryStatus();
                                                if(lowBattery_flag==1)
                                                {
                                                    break;
                                                }
                                            }        
                    if(test_4G_rest_flag>=5)
                    {
                        test_4G_rest_flag=0;
        //                wake_flag==1;
        //                toggleWake_flag==1;
                        test_4G_reset();
                        putsUART1_2("********************************************************************************\r\n");
                        putsUART1_2("\t\t RESET____4G\r\n");
                        putsUART1_2("********************************************************************************\r\n");
                    }
                
            }
            if(test_finish1M_flag>=9)
            {
                Car_information_patch_1[26]=0x02;
                if(get_sensor_CSRflag==1)
                {
                    get_sensor_flag=0;
                    Write(W_Addr , 0x24 ,gsensor[2] );
                }     
                else
                {
                    Write(W_Addr , 0x24 ,0x05 );
                }
                G_sensorReset();
                g_sensor_toggle_flag=0;
                Write(W_Addr , 0x2E ,0x10 );
            }
        }
        
//            if((BATT_SEN1_GetValue()==1)&&(BATT_SEN2_GetValue()==1)||(ACC_DET_GetValue()==1))
//            {
//                lowBattery_flag=0;
//            }
            
            if((lowBattery_flag==1))//low battery notification  &&(ACC_DET_GetValue()==0)
            {
                            G_sensorReset();
                lowBattery_flag=0;
                
                __delay_ms(1500);
                
                if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))//check battery status twice
                {
                    __delay_ms(500);
                    if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                    {
                        __delay_ms(3000);
                        if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                        {
                            makeTOKENpackage();
                            makeENDpackage();
                            makeEXCEPTIONpackageabNormal();

                                makeBATTERYpackage();
                                WRITE_TEMP(sendbattery,sizeof(sendbattery));

                            testShortPACKAGE();
                        }
                        else
                        {
                            lowBattery_flag=0;
                        }
                    }
                    else
                    {
                        lowBattery_flag=0;
                    }
                }
                else
                {
                    lowBattery_flag=0;
                }
            }  
            else
            {
                lowBattery_flag=0;
            }
        
        if((ACC_DET_GetValue()==1)||(g_sensor_toggle_flag==1))//check before sleep
        {
            finish_flag=0;
        }
        
        if(finish_flag==1)
        {
//            for(i=0;i<156;i++)
//            {
//                eeData[i]=sendtoken[i];
//            }
//            __delay_ms(1500);
            
            if(get_sensor_CSRflag==1)
            {
//                get_sensor_CSRflag=0;
                get_sensor_flag=0;
    //                    gsensor[3]=0x0f;
                Write(W_Addr , 0x24 ,gsensor[2] );//gsensor[2]
                G_sensorReset();
                
            }            
            
//            if(accToggle_flag==1)
//            {
//                __delay_ms(60000);
//            }
            Sleep1();
        }
    }
        

    }
}
/**
 End of File
*/



void readRSSI2()
{
    
    do{
            for (i=0;i<3;i++)
            {
                    rssi[i]=0;
            }
    read_uart1_data3A_flag=0;
    read_uart1_dataO_flag=0;  
    UART1_Initialize();
    putsUART1_2("AT+CSQ\r\n");//rssi
    for(i=0;(read_uart1_data3A_flag==0)&(i<50000);i++)//prevent stock
    {
        __delay_us(1);
    }
    if(read_uart1_data3A_flag==1)
    {
        read_uart1_data3A_flag=0;
            if(read_uart1_data_flag==1)
            {
                read_uart1_data_flag=0;
            }
        for (i=0,j=0;(i<3)&&(j<10000);j++)
        {
            if(read_uart1_data_flag==1)
            {
                read_uart1_data_flag=0;
                rssi[i]=Rx1Value;
                i++;
            }
        }
        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
        {
            __delay_ms(5);
        }
        
        if(rssi[0]==0x20)//check rssi values
        {
            rssi[0]=0x0;
        }
        
//        if((rssi[1]>>2)!=0x3)
//        {
//            rssi[1]=0x2C;
//        }
//        if((rssi[2]>>2)!=0x3)
//        {
//            rssi[2]=0x2C;
//        }
        
    }
    }while((rssi[1]==0x2C)||(rssi[2]==0x2C));

}

//void readIMEI()
//{
//    do{
//                for (i=0;i<15;i++)
//        {
//                imei[i]=0;
//                imsi2[i]=0;
//        }
//    UART1_Initialize();
//    read_uart1_dataN_flag=0;
//    read_uart1_dataO_flag=0;
//    putsUART1_2("AT+CGSN\r\n");//imei
//    for(i=0;(read_uart1_dataN_flag==0)&(i<50000);i++)//prevent stock
//    {
//        __delay_us(2);
//    }
//    if(read_uart1_dataN_flag==1)
//    {
//        read_uart1_dataN_flag=0;
//        for (i=0,j=0;(i<4)&&(j<10000);j++)//4
//        {
//            if(read_uart1_data_flag==1)
//            {
//                read_uart1_data_flag=0;
//                i++;
//            }
//        }
//        for (i=0,j=0;(i<15)&&(j<10000);j++)
//        {
//            if(read_uart1_data_flag==1)
//            {
//                read_uart1_data_flag=0;
//                imei[i]=Rx1Value;
//                i++;
//            }
//        }
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
//    
//    }
//    read_uart1_dataN_flag=0;
//    read_uart1_dataO_flag=0;
//    putsUART1_2("AT+CGSN\r\n");//imei
//    for(i=0;(read_uart1_dataN_flag==0)&(i<50000);i++)//prevent stock
//    {
//        __delay_us(2);
//    }
//    //imei2
//    if(read_uart1_dataN_flag==1)
//    {
//        read_uart1_dataN_flag=0;
//        for (i=0,j=0;(i<4)&&(j<10000);j++)//4
//        {
//            if(read_uart1_data_flag==1)
//            {
//                read_uart1_data_flag=0;
//                i++;
//            }
//        }
//        for (i=0,j=0;(i<15)&&(j<10000);j++)
//        {
//            if(read_uart1_data_flag==1)
//            {
//                read_uart1_data_flag=0;
//                imei2[i]=Rx1Value;
//                i++;
//            }
//        }
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
//    
//    }
//    for(i=0;i<15;i++)
//    {
//        if(imei[i]!=imei2[i])
//        {
//            imei[14]=0x0;
//        }
//    }
//    }while(imei[14]<0x20);
//
//}

void readIMEI2()
{
    do{
                for (i=0;i<15;i++)
        {
                imei[i]=0;
                imei2[i]=0;
        }
    read_uart1_dataN_flag=0;
    read_uart1_dataO_flag=0;
        UART1_Initialize();
    putsUART1_2("AT+CGSN\r\n");//imei
    imei_index=0;
    for(i=0,imei_index=0,read_uart1_dataO_flag=0;(i<5000)&&(read_uart1_dataO_flag==0);i++)
    {
            if(read_uart1_data_flag==1)
            {
                read_uart1_data_flag=0;
                if(Rx1Value>>4==0x3)
                {
                    imei[imei_index]=Rx1Value;
                    imei_index++;
                }
            }
//                    __delay_us(20);
    }    
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
    //                    __delay_us(20);
            UART1_Initialize();
        putsUART1_2("AT+CGSN\r\n");//imei
            read_uart1_dataO_flag=0;
            imei_index=0;
    for(i=0,imei_index=0,read_uart1_dataO_flag=0;(i<5000)&&(read_uart1_dataO_flag==0);i++)
    {
            if(read_uart1_data_flag==1)
            {
                read_uart1_data_flag=0;
                if(Rx1Value>>4==0x3)
                {
                    imei2[imei_index]=Rx1Value;
                    imei_index++;
                }
            }
//                    __delay_us(20);
    }
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
    for(i=0;i<15;i++)
    {
        if(imei[i]!=imei2[i])
        {
            imei[14]=0x0;
        }
    }
            
            if(imei[14]<0x20)
            {
                __delay_ms(50);
            }
            
    }while(imei[14]<0x20);

}

void readIMSI()
{
    do{
            for (i=0;i<15;i++)
            {
                    imsi[i]=0;
                    imsi2[i]=0;
            }
    read_uart1_dataN_flag=0;
    read_uart1_dataO_flag=0;
        UART1_Initialize();
    putsUART1_2("AT+CIMI\r\n");//imsi
        read_uart1_dataO_flag=0;
//    for(i=0;(Rx1Value<0x30)&&(Rx1Value>0x39)&&(i<50000);i++)//prevent stock
//    {
//        __delay_us(2);
//    }
    for(i=0,imsi_index=0,read_uart1_dataO_flag=0;(i<5000)&&(read_uart1_dataO_flag==0);i++)
    {
            if((Rx1Value>>4==0x3)&&(read_uart1_data_flag==1))
            {
                read_uart1_data_flag=0;
                imsi[imsi_index]=Rx1Value;
                imsi_index++;
            }
//                    __delay_us(20);
    }    
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
            UART1_Initialize();
        putsUART1_2("AT+CIMI\r\n");//imsi
            read_uart1_dataO_flag=0;
    for(i=0,imsi_index=0,read_uart1_dataO_flag=0;(i<5000)&&(read_uart1_dataO_flag==0);i++)
    {
            if((Rx1Value>>4==0x3)&&(read_uart1_data_flag==1))
            {
                read_uart1_data_flag=0;
                imsi2[imsi_index]=Rx1Value;
                imsi_index++;
            }
//                    __delay_us(20);
    }
//        for(i=0;(read_uart1_dataO_flag==0)&(i<100);i++)//prevent stock
//        {
//            __delay_ms(5);
//        }
    for(i=0;i<15;i++)
    {
        if(imsi[i]!=imsi2[i])
        {
            imsi[14]=0x0;
        }
    }
            if(imsi[14]<0x20)
            {
                __delay_ms(50);
            }
    }while(imsi[14]<0x20);

}

void makedecodingStatetokenpackage()//decodingStatetoken
{
    makeshortimeiimsi();
    int i=0;
    for(i=0;i<171;i++)
    {
        decodingStatetoken[i]=0;
    }
    decodingStatetoken[0]=0x10;
                    decodingStatetoken[1]=0x27;
                    decodingStatetoken[2]=0xA8;
                    for(i=0;i<6;i++)//time
                    {    
                        decodingStatetoken[i+3]=time[i];
//                        decodingStatetoken[i+3]='\x77';
                    }
                    for(i=0;i<144;i++)//token id
                    {    
                        decodingStatetoken[i+9]=token[i];
                    }
                    for(i=0;i<8;i++)
                    {    
                        decodingStatetoken[i+153]=shortimei[i];
                    }
                    for(i=0;i<8;i++)
                    {    
                        decodingStatetoken[i+161]=shortimsi[i];
                    }
                    for(i=1;i<=168;i++)//cs
                    {    
                        decodingStatetoken[169]=decodingStatetoken[i]+decodingStatetoken[169];
                    }
                    decodingStatetoken[170]=0x2A;
//                    sendimsiimei[41]='\xff';

}

void makeshortimeiimsi()
{
    
    tempimei[0]=0x30;
    for(i=0;i<15;i++)
    {
        tempimei[i+1]=imei[i];
    }   
    for(i=0;i<9;i++)
    {
        shortimei[i]=((tempimei[2*i+1]-0x30)+(tempimei[2*i]<<4));
    }
    
    tempimsi[0]=0x30;
    for(i=0;i<15;i++)
    {
        tempimsi[i+1]=imsi[i];
    }   
    for(i=0;i<9;i++)
    {
        shortimsi[i]=((tempimsi[2*i+1]-0x30)+(tempimsi[2*i]<<4)); 
    }
    
}

void makeIMSIIMSIpackage()
{
    readIMEI2();
    readIMSI();
    sendimsiimei[40]=0;
    sendimsiimei[0]=0x20;
                    sendimsiimei[1]=0x21;
                    sendimsiimei[2]=0x27;
                    for(i=0;i<6;i++)
                    {    
                        sendimsiimei[i+3]=time[i];
                        sendimsiimei[i+3]='\x77';
                    }
                    for(i=0;i<15;i++)
                    {    
                        sendimsiimei[i+9]=imei[i];
                    }
                    for(i=0;i<15;i++)
                    {    
                        sendimsiimei[i+24]=imsi[i];
                    }
                    for(i=0;i<1;i++)//version
                    {    
                        sendimsiimei[39]=0x03;
                    }
                    for(i=1;i<=39;i++)
                    {    
                        sendimsiimei[40]=sendimsiimei[i]+sendimsiimei[40];
                    }
                    sendimsiimei[41]=0x2A;
//                    sendimsiimei[41]='\xff';

}

void makeRSSIpackage()
{
    readRSSI2();
    for(i=0;i<13;i++)
    {
        sendrssi[i]=0;
    }
//    sendrssi[11]=0;
    sendrssi[0]=0x20;
                    sendrssi[1]=0x53;
                    sendrssi[2]=0xB;//B
                    for(i=0;i<6;i++)
                    {    
                        sendrssi[i+3]=time[i];
                    }
                    for(i=0;i<3;i++)
                    {    
                        sendrssi[i+9]=rssi[i];
                    }
                    for(i=1;i<=11;i++)
                    {    
                        sendrssi[12]=sendrssi[i]+sendrssi[12];
                    }
                    sendrssi[13]=0x2A;
                    //sendrssi[13]='\xff';

}

void makeTOKENpackage()
{
    //readRSSI2();
    for(i=0;i<154;i++)
    {
        sendtoken[i]=0;
    }
//    sendtoken[153]=0;
    sendtoken[0]=0x20;
                    sendtoken[1]=0x31;
                    sendtoken[2]=0x98;
                    for(i=0;i<6;i++)
                    {    
                        sendtoken[i+3]=time[i];
                    }
                    for(i=0;i<144;i++)
                    {    
                        sendtoken[i+9]=token[i];
                    }
                    for(i=1;i<=152;i++)
                    {    
                        sendtoken[153]=sendtoken[i]+sendtoken[153];
                    }
                    sendtoken[154]=0x2A;
//                    sendtoken[155]='\xff';

}


void makeEXCEPTIONpackageCANabNormal()
{
    for(i=0;i<11;i++)
    {
        sendexception[i]=0;
    }
//    sendexception[10]=0;
    sendexception[0]=0x20;
                    sendexception[1]=0x25;
                    sendexception[2]=0x09;
                    for(i=0;i<6;i++)
                    {    
                        sendexception[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendexception[i+9]=0x03;
                    }
                    for(i=1;i<=9;i++)
                    {    
                        sendexception[10]=sendexception[i]+sendexception[10];
                    }
                    sendexception[11]=0x2A;
//                    sendexception[12]='\xff';

}//exception can

void makeEXCEPTIONpackageEmergenceNormal()
{
    for(i=0;i<11;i++)
    {
        sendexception[i]=0;
    }
//    sendexception[10]=0;
    sendexception[0]=0x20;
                    sendexception[1]=0x25;
                    sendexception[2]=0x09;
                    for(i=0;i<6;i++)
                    {    
                        sendexception[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendexception[i+9]=0x02;
                    }
                    for(i=1;i<=9;i++)
                    {    
                        sendexception[10]=sendexception[i]+sendexception[10];
                    }
                    sendexception[11]=0x2A;
//                    sendexception[12]='\xff';

}//emergency help

void makeEXCEPTIONpackageabNormal()
{
    for(i=0;i<11;i++)
    {
        sendexception[i]=0;
    }
//    sendexception[10]=0;
    sendexception[0]=0x20;
                    sendexception[1]=0x25;
                    sendexception[2]=0x09;
                    for(i=0;i<6;i++)
                    {    
                        sendexception[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendexception[i+9]=0x01;
                    }
                    for(i=1;i<=9;i++)
                    {    
                        sendexception[10]=sendexception[i]+sendexception[10];
                    }
                    sendexception[11]=0x2A;
//                    sendexception[12]='\xff';

}//sendexception

void makeEXCEPTIONpackageNormal()
{
    for(i=0;i<11;i++)
    {
        sendexception[i]=0;
    }
//    sendexception[10]=0;
    sendexception[0]=0x20;
                    sendexception[1]=0x25;
                    sendexception[2]=0x09;
                    for(i=0;i<6;i++)
                    {    
                        sendexception[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendexception[i+9]=0x00;
                    }
                    for(i=1;i<=9;i++)
                    {    
                        sendexception[10]=sendexception[i]+sendexception[10];
                    }
                    sendexception[11]=0x2A;
//                    sendexception[12]='\xff';

}//sendexception

void makeTOKENpackageWithACC()   //ACC_DET_GetValue()
{
    
    for(i=0;i<155;i++)
    {
        sendtokenACC[i]=0;
    }
    //readRSSI2();
//    sendtokenACC[154]=0;
    sendtokenACC[0]=0x20;
                    sendtokenACC[1]=0x31;
                    sendtokenACC[2]=0x99;
                    for(i=0;i<6;i++)
                    {    
                        sendtokenACC[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendtokenACC[i+9]=ACC_DET_GetValue();
                    }
                    for(i=0;i<144;i++)
                    {    
                        sendtokenACC[i+10]=token[i];
                    }
                    for(i=1;i<=152;i++)
                    {    
                        sendtokenACC[154]=sendtokenACC[i]+sendtokenACC[154];
                    }
                    sendtokenACC[155]=0x2A;
                    sendtokenACC[156]='\xff';

}

void makeBATTERYpackage()
{
    //readRSSI2();
    if(BATT_SEN1_GetValue()==1)
    {
        battery=0;
    }
    else if (BATT_SEN2_GetValue()==1)
    {
        battery=0x01;
    }
    else
    {
        battery=0x02;
    }
    for(i=0;i<11;i++)
    {
        sendbattery[i]=0;
    }
    for(i=0;i<11;i++)
    {
        sendbattery[i]=0;
    }
    sendbattery[10]=0;
    sendbattery[0]=0x20;
                    sendbattery[1]=0x23;
                    sendbattery[2]=0x9;
                    for(i=0;i<6;i++)
                    {    
                        sendbattery[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendbattery[i+9]=battery;///battery
                    }
                    for(i=1;i<=9;i++)
                    {    
                        sendbattery[10]=sendbattery[i]+sendbattery[10];
                    }
                    sendbattery[11]=0x2A;
                    //sendbattery[12]='\xff';

}

//void makeGPSpackage()
//{
//    //readRSSI2();
//    sendgps[32]=0;
//    sendgps[0]=0x20;
//                    sendgps[1]=0x52;
//                    sendgps[2]=0x0;
//                    sendgps[3]=0x1E;
//                    for(i=0;i<6;i++)
//                    {    
//                        sendgps[i+4]=time[i];
//                    }
//                    for(i=0;i<9;i++)
//                    {    
//                        sendgps[i+10]=lat2[i];
//                    }
//                    for(i=0;i<10;i++)
//                    {    
//                        sendgps[i+19]=lon2[i];
//                    }
//                    for(i=0;i<3;i++)
//                    {    
//                        sendgps[i+29]=speed2[i];
//                    }
//                    for(i=1;i<=31;i++)
//                    {    
//                        sendgps[32]=sendgps[i]+sendgps[32];
//                    }
//                    sendgps[33]=0x2A;
//                    sendgps[34]='\xff';
//
//}

void makeGPSpackage2()
{
    for(i=0;i<39;i++)
    {
        sendgps2[i]=0;
    }
//    sendgps2[32]=0;
    sendgps2[0]=0x20;
                    sendgps2[1]=0x52;
                    sendgps2[2]=0x00;
                    sendgps2[3]=0x24;
                    for(i=0;i<6;i++)
                    {    
                        sendgps2[i+4]=time[i];
                    }
                    for(i=0;i<9;i++)
                    {    
                        sendgps2[i+10]=lat2[i];
                    }
                    for(i=0;i<10;i++)
                    {    
                        sendgps2[i+19]=lon2[i];
                    }
                    for(i=0;i<3;i++)
                    {    
                        sendgps2[i+29]=speed2[i];
                    }
                    for(i=0;i<6;i++)
                    {    
                        sendgps2[i+32]=angle2[i];
                    }
                    for(i=1;i<=37;i++)
                    {    
                        sendgps2[38]=sendgps2[i]+sendgps2[38];
                    }
                    sendgps2[39]=0x2A;
                    //sendgps[31]='\xff';

}

void makeENDpackage()
{
    //readRSSI2();
    for(i=0;i<10;i++)
    {
        sendend[i]=0;
    }
    sendend[9]=0;
    sendend[0]=0x20;
                    sendend[1]=0x32;
                    sendend[2]=0x8;
                    for(i=0;i<6;i++)
                    {    
                        sendend[i+3]=time[i];
                    }
                    for(i=1;i<=8;i++)
                    {    
                        sendend[9]=sendend[i]+sendend[9];
                    }
                    sendend[10]=0x2A;
//                    sendend[11]='\xff';

}

//void makeCANpackage()
//{
//    //readRSSI2();
//    sendcan[30]=0;
//    sendcan[0]=0x20;
//                    sendcan[1]=0x22;
//                    sendcan[2]=0x1D;
//                    for(i=0;i<6;i++)
//                    {    
//                        sendcan[i+3]=time[i];
//                    }
//                    for(i=0;i<21;i++)
//                    {    
//                        sendcan[i+9]=candata[i];
//                    }
//                    for(i=1;i<=29;i++)
//                    {    
//                        sendcan[30]=sendcan[i]+sendcan[30];
//                    }
//                    sendcan[31]=0x2A;
//
//}

void makeCANpackage1()
{
    //readRSSI2();
    for(i=0;i<44;i++)
    {
        sendcan1[i]=0;
    }
//    sendcan1[30]=0;
    sendcan1[0]=0x20;
                    sendcan1[1]=0x22;
                    sendcan1[2]=0x2A;
                    for(i=0;i<6;i++)
                    {    
                        sendcan1[i+3]=time[i];
                    }
                    for(i=0;i<34;i++)
                    {    
                        sendcan1[i+9]=Car_information_patch_1[i];
                    }
                    for(i=1;i<=42;i++)
                    {    
                        sendcan1[43]=sendcan1[i]+sendcan1[43];
                    }
                    sendcan1[44]=0x2A;

}

void makeCANpackage2()
{
    //readRSSI2();
    for(i=0;i<31;i++)
    {
        sendcan2[i]=0;
    }
//    sendcan2[30]=0;
    sendcan2[0]=0x20;
                    sendcan2[1]=0x22;
                    sendcan2[2]=0x1D;
                    for(i=0;i<6;i++)
                    {    
                        sendcan2[i+3]=time[i];
                    }
                    for(i=0;i<21;i++)
                    {    
                        sendcan2[i+9]=Car_information_patch_2[i];
                    }
                    for(i=1;i<=29;i++)
                    {    
                        sendcan2[30]=sendcan2[i]+sendcan2[30];
                    }
                    sendcan2[31]=0x2A;

}

void makeGSENSORpackage()
{
    readADXL345();
    //readRSSI2();
    for(i=0;i<16;i++)
    {
        sendgsensor[i]=0;
    }
    sendgsensor[15]=0;
    sendgsensor[0]=0x20;
                    sendgsensor[1]=0x54;
                    sendgsensor[2]=0xE;
                    for(i=0;i<6;i++)
                    {    
                        sendgsensor[i+3]=time[i];
                    }
                    for(i=0;i<1;i++)
                    {    
                        gxh=Read( W_Addr, 0x33 ,R_Addr);
                        sendgsensor[i+9]=gxh;
                    }
                    for(i=0;i<1;i++)
                    {    
                        gxl=Read( W_Addr, 0x32 ,R_Addr);
                        sendgsensor[i+10]=gxl;
                    }
                    for(i=0;i<1;i++)
                    {    
                        gy0=Read( W_Addr, 0x35 ,R_Addr);
                        sendgsensor[i+11]=gy0;
                    }
                    for(i=0;i<1;i++)
                    {    
                        gy1=Read( W_Addr, 0x34 ,R_Addr);
                        sendgsensor[i+12]=gy1;
                    }
                    for(i=0;i<1;i++)
                    {    
                        gz0=Read( W_Addr, 0x37 ,R_Addr);
                        sendgsensor[i+13]=gz0;
                    }
                    for(i=0;i<1;i++)
                    {    
                        gz1=Read( W_Addr, 0x36 ,R_Addr);
                        sendgsensor[i+14]=gz1;
                    }
                    for(i=1;i<=14;i++)
                    {    
                        sendgsensor[15]=sendgsensor[i]+sendgsensor[15];
                    }
                    sendgsensor[16]=0x2A;
                    //sendgsensor[17]='\x77';//2018/08/07

}


void makeCANASKG()//package ASK G-sensor
{
    //readRSSI2();
    for(i=0;i<14;i++)
    {
        sendASKCAN[i]=0;
    }
//    sendcan2[30]=0;
    sendASKCAN[0]=0x20;
                    sendASKCAN[1]=0x28;
                    sendASKCAN[2]=0x0B;
                    for(i=0;i<6;i++)
                    {    
                        sendASKCAN[i+3]=0xff;
                    }
                    for(i=0;i<1;i++)
                    {    
                        sendASKCAN[i+9]=0x01;
                    }
                    for(i=0;i<2;i++)
                    {    
                        sendASKCAN[i+10]=Car_information_patch_1[29+i];
                    }
                    
                    for(i=1;i<=11;i++)
                    {    
                        sendASKCAN[12]=sendASKCAN[i]+sendASKCAN[12];
                    }
                    sendASKCAN[13]=0x2A;

}


void readADXL345()
{
            gxh=Read( W_Addr, 0x33 ,R_Addr);
            gxl=Read( W_Addr, 0x32 ,R_Addr);
            gy0=Read( W_Addr, 0x35 ,R_Addr);
            gy1=Read( W_Addr, 0x34 ,R_Addr);
            gz0=Read( W_Addr, 0x37 ,R_Addr);
            gz1=Read( W_Addr, 0x36 ,R_Addr);
            gx=gxh<<8;
            gx=gx+gxl;
            sprintf(strx,"%d",gx);
            gy=gy0<<8;
            gy=gy+gy1;
            sprintf(stry,"%d",gy);
            gz=gz0<<8;
            gz=gz+gz1;
            sprintf(strz,"%d",gz);
            
//        strcpy(str,"X=");
//        strcat(str,strx);
//        strcat(str,"\tY=");
//        strcat(str,stry);
//        strcat(str,"\tZ=");
//        strcat(str,strz);
//        strcat(str,"\r\n");
//            putsUART1_2(str);

}

void readADXL345_2()
{
            gxh=Read( W_Addr, 0x33 ,R_Addr);
            gxl=Read( W_Addr, 0x32 ,R_Addr);
            gy0=Read( W_Addr, 0x35 ,R_Addr);
            gy1=Read( W_Addr, 0x34 ,R_Addr);
            gz0=Read( W_Addr, 0x37 ,R_Addr);
            gz1=Read( W_Addr, 0x36 ,R_Addr);
            gx=gxh<<8;
            gx=gx+gxl;
            sprintf(strx,"%d",gx);
            gy=gy0<<8;
            gy=gy+gy1;
            sprintf(stry,"%d",gy);
            gz=gz0<<8;
            gz=gz+gz1;
            sprintf(strz,"%d",gz);
            
        strcpy(str,"X=");
        strcat(str,strx);
        strcat(str,"\tY=");
        strcat(str,stry);
        strcat(str,"\tZ=");
        strcat(str,strz);
        strcat(str,"\r\n");
            putsUART1_2(str);

}

void bufferToFlash()
{
    //unsigned long int numBytesWritten = 0 ;
    
    for (i=0;i<send_TEMP_Index;spi_flash_index++)
    {
        spi_flash_H_index=spi_flash_index>>16;
        spi_flash_M_index=spi_flash_index>>8;
        spi_flash_L_index=spi_flash_index;


        SS1_SetLow();
        SPI1_Exchange8bit(0x06);
        SS1_SetHigh();  
        //__delay_us(10);
        SS1_SetLow();
        SPI1_Exchange8bit(0x02);
        SPI1_Exchange8bit(spi_flash_H_index);
        SPI1_Exchange8bit(spi_flash_M_index);
        SPI1_Exchange8bit(spi_flash_L_index);//send_TEMP
        SPI1_Exchange8bit(send_TEMP[i++]);
//        spi_flash_index++;
        //numBytesWritten++;  
        //spi_flash_index++;
        SS1_SetHigh(); 
        __delay_ms(3);
    }
    send_TEMP_Index=0;
    
}

void sendSpiFlashdata()
{
//    unsigned long int i=0;
    for(i=0; i<spi_flash_index;i++)
    {
        spi_flash_H_index=i>>16;
        spi_flash_M_index=i>>8;
        spi_flash_L_index=i;
        
        SS1_SetLow();
        SPI1_Exchange8bit(0x03);
        SPI1_Exchange8bit(spi_flash_H_index);
        SPI1_Exchange8bit(spi_flash_M_index);
        SPI1_Exchange8bit(spi_flash_L_index);
        U1TXREG=SPI1_Exchange8bit(0xFF);
        SS1_SetHigh(); 
        __delay_us(10);//test
//        __delay_ms(3);     
    }
    spi_flash_index=0;
//    spiChipErase();
}

void spiChipErase()
{
    SS1_SetLow();
    SPI1_Exchange8bit(0x06);
    SS1_SetHigh();  
    __delay_us(10);
    SS1_SetLow();
    SPI1_Exchange8bit(0xC7);
    SS1_SetHigh();  
    __delay_ms(6000);//6000

}

void spiBlockErase()
{
    SS1_SetLow();
    SPI1_Exchange8bit(0x06);
    SS1_SetHigh();  
    __delay_us(10);
    SS1_SetLow();
    SPI1_Exchange8bit(0xD8);
    SPI1_Exchange8bit(0x00);
    SPI1_Exchange8bit(0x00);
    SPI1_Exchange8bit(0xFF);
    SS1_SetHigh();  
    __delay_ms(1000);

}


void testShortPACKAGE()
{
        //D11_SetLow();
        //VBUS_CTRL_SetHigh();
        putsUART1_2("AT+VTD?\r\n");//rssi
        read_uart1_data30_flag=0;
            __delay_ms(500);
            if(read_uart1_data30_flag==1)
            {
                makedecodingStatetokenpackage();
                read_uart1_dataW1_flag = 0;
                open_socket_fail_flag = 0;
                ClearFlag();
                putsUART1_2("AT+CIPMODE=1\r\n");
                __delay_ms(10);
                read_uart1_dataO_flag=0;
                for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                putsUART1_2("AT+CGSOCKCONT=1,\"IP\",\"internet.iot\"\r\n");
                read_uart1_dataO_flag=0;
                for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                UART1_Initialize();
                putsUART1_2("AT+NETOPEN\r\n"); 
                __delay_ms(1);
                read_uart1_dataO_flag=0;
                for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                    {
                        __delay_ms(10);
                    }
                       open_socket_error_flag=0;
                open_socket_fail_flag=0;
                UART1_Initialize();
                putsUART1_2(IOT_DOMAIN); 
                __delay_ms(20);
                read_uart1_data30_flag=0;
                    for(i=0;(read_uart1_data30_flag<2)&&(i<=500);i++)//prevent stuck
                    {
                        __delay_ms(10);
                        if(i>=500)
                        {
                            open_socket_fail_flag=1;
                            open_socket_error_flag=1;
                        }
        //                if(i>=500)
        //                {
        //                    i=0;
        //                    open_socket_fail_flag++;
        //                    UART1_Initialize();
        //                    read_uart1_dataO_flag=0;
        //                    for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
        //                        {
                                //                            __delay_ms(1000);
        //                            putsUART1_2("+++"); 
        //                        }
        //                    putsUART1_2("\r\n");
        //                    putsUART1_2("AT+NETCLOSE\r\n");
        //                    __delay_ms(20);
        //                    putsUART1_2("\r\n");
        //                    read_uart1_dataO_flag=0;
        //                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
        //                    {
        //                        __delay_ms(1);
        //                    }
        //                    putsUART1_2("AT+NETOPEN\r\n"); //
        //                    read_uart1_data30_flag=0;
        //                    for(j=0;(read_uart1_data30_flag==0)&(j<500);j++)//prevent stock
        //                        {
        //                            __delay_ms(10);
        //                        }
        //                    putsUART1_2(IOT_DOMAIN); //
        //                    __delay_ms(1);
        //                    read_uart1_data30_flag=0;
        //                    read_uart1_dataO_flag=0;
        //                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
        //                    {
        //                        __delay_ms(1);
        //                    }
        //                    
        //                }
        //                if(open_socket_fail_flag>=3)
        //                {
        //                    open_socket_fail_flag=0;
        //                    open_socket_error_flag=1;
        //                    putsUART1_2("AT+CIPCLOSE\r\n");
        //                    //UART1_Initialize();
        //                    break;
        //                }
                    }
                if(open_socket_fail_flag==0)
                {
        //            putsUART1_2package(sendtoken);
                BT_Send_String(sendtoken,sizeof(sendtoken));
                    __delay_ms(5);
                            BT_Send_String(sendexception,sizeof(sendexception));
        //            putsUART1_2package(sendexception);
                    __delay_ms(500);
                    //putsUART1_2package(sendimsiimei);
                    WRITE_PACKAGE();
                    __delay_ms(500);
//                    BT_Send_String(decodingStatetoken,sizeof(decodingStatetoken));//check token package
//                    __delay_ms(500);



                    //putsUART1_2package(sendgsensor);
                    //putsUART1_2package(sendgps);
                    BT_Send_String(sendend,sizeof(sendend));
        //            putsUART1_2package(sendend);


                    //check ack
                    read_uart1_data30_flag=0;
                    read_uart1_data62_flag=0;
                    read_uart1_data03_flag=0;
                    read_uart1_data01_flag=0;
                    read_uart1_data66_flag=0;
                    read_uart1_data2A_flag=0;
                    for(i=0;((read_uart1_data30_flag==0)||(read_uart1_data62_flag==0)||(read_uart1_data03_flag==0)||(read_uart1_data2A_flag==0))&&(i<300);i++)//prevent stock
                    {
                        __delay_ms(10);
                    }
                    if((read_uart1_data30_flag==1)&&(read_uart1_data62_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data66_flag==1)&&(read_uart1_data2A_flag==1))
                    {
                        putsUART1_2package77("\x10\x62\x03\x01\x66\x2A\x77");
                        __delay_ms(50);
                    }

        //        __delay_ms(15000);
        //        putsUART1_2("+++"); 
                read_uart1_dataO_flag=0;
                for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
                    {
                        putsUART1_2("+++");
                        for(j=0;(read_uart1_dataO_flag==0)&&(j<100);j++)
                        {
                            __delay_ms(10);
                        }    
        //                __delay_ms(1000);
        //                putsUART1_2("+++"); 
                    }
                read_uart1_dataO_flag=0;
                read_uart1_data_flag=0;

                putsUART1_2("\r\n");
                putsUART1_2("AT+CIPCLOSE=0\r\n");
                        __delay_ms(10);
                read_uart1_dataO_flag=0;
                for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                //D11_SetHigh();
                //VBUS_CTRL_SetLow();
        //        }


                //D11_SetHigh();
                //VBUS_CTRL_SetLow();
                }
                else
                {
                    putsUART1_2("+++"); 
                                    read_uart1_dataO_flag=0;
                        for(j=0;(read_uart1_dataO_flag==0)&(j<1500);j++)//prevent stock
                        {
                            __delay_ms(1);
                        }
                      putsUART1_2("AT+NETCLOSE\r\n");
                                        bufferToFlash();
                            sipFlash_data_flag=1;
                            test_4G_rest_flag++;
        //                                    test_4G_reset();//test reset
                }
            }
            else
            {
                putsUART1_2("+++"); 
                                read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<1500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                  putsUART1_2("AT+NETCLOSE\r\n");
                                    bufferToFlash();
                        sipFlash_data_flag=1;
                        test_4G_rest_flag++;
    //                                    test_4G_reset();//test reset
            }
    
}


void newIMEIIMSI()
{
        read_uart1_dataW1_flag = 0;
        open_socket_fail_flag = 0;
        ClearFlag();
        putsUART1_2("AT+CIPMODE=1\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
    request_token_flag=0;
        ClearFlag();
        putsUART1_2("AT+CGSOCKCONT=1,\"IP\",\"internet.iot\"\r\n");
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+NETOPEN\r\n"); //
        read_uart1_data30_flag=0;
        for(i=0;(read_uart1_data30_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(10);
            }
        open_socket_error_flag=0;
        open_socket_fail_flag=0;
        putsUART1_2(IOT_DOMAIN); //
        __delay_ms(20);
        read_uart1_data30_flag=0;
            for(i=0;(read_uart1_data30_flag<2)&(i<=300);i++)//prevent stock
            {
                __delay_ms(10);
                if(i>=300)
                {
                    i=0;
                    open_socket_fail_flag++;
                    UART1_Initialize();
                    putsUART1_2("\r\n");
                    putsUART1_2("AT+NETCLOSE\r\n");
                    __delay_ms(20);
                    putsUART1_2("\r\n");
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    putsUART1_2("AT+NETOPEN\r\n"); //
                    read_uart1_data30_flag=0;
                    for(j=0;(read_uart1_data30_flag==0)&(j<500);j++)//prevent stock
                        {
                            __delay_ms(10);
                        }
                    putsUART1_2(IOT_DOMAIN); //
                    __delay_ms(20);
                    read_uart1_data30_flag=0;
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    
                }
                if(open_socket_fail_flag>=4)
                {
                    open_socket_fail_flag=0;
                    open_socket_error_flag=1;
                    putsUART1_2("AT+NETCLOSE\r\n");
                    //UART1_Initialize();
                    break;
                }
//                else
//                {
//                    open_socket_error_flag=0;
//                }
            }
        
        if(open_socket_error_flag==0)
        {    
//            putsUART1_2("AT+CIPSEND=0,42\r\n");
//            //__delay_ms(1000);
//            ClearFlag();
//            read_uart1_dataO_flag=0;
//            for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
//                {
//                    __delay_ms(1);
//                }
            //putsUART1_2("\x10\x01\x20\x02");
                //putsUART1_2("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\r\n");
                //read_uart1_data01_flag=0;
                //read_uart1_data10_flag=0;
                //read_uart1_data61_flag=0;
                //read_uart1_data21_flag=0;
                //read_uart1_dataack_flag=0;
                //read_uart1_datatime_flag=0;
//            putsUART1_2package(sendimsiimei);
            read_uart1_data21_flag;
                    read_uart1_data10_flag;
                    read_uart1_data12_flag;
            BT_Send_String(sendimsiimei,sizeof(sendimsiimei));
                //ClearFlag();
                //read_uart1_dataack_flag = 1;
                //read_uart1_datatime_flag = 1;
                //token_index=0;
                //time_index=0;
            token_index=0;
            time_index=0;
            get_time_flag=0;
            notRegistered_flag=0;
            read_uart1_data14_flag=0;
            request_token_flag = 1;
            request_time_flag = 1;   
                ClearFlag();

            for(i=0;(get_time_flag==0)&(i<50);i++)//prevent stock
                {
                    __delay_ms(100);
                }
                ClearFlag();
//                UART1_Flag_reset();
                            request_token_flag = 0;
            request_time_flag = 0;
            //read_uart1_dataO_flag=0;
            //read_uart1_data_flag=0;
                request_sensor_flag=1;
                if(get_time_flag==1)
                {
//                    putsUART1_2package77("\x20\x26\x09\x12\x01\x01\x00\x00\x00\x01\x44\x2a\x77");
                    makeCANASKG();
                    BT_Send_String(sendASKCAN,sizeof(sendASKCAN));
                }
                
                for(i=0;(get_sensor_flag==0)&(i<500);i++)//prevent stock
                {
                    __delay_ms(10);
                }
                request_sensor_flag=0;
                if(get_sensor_CSRflag==1)
                {
//                    get_sensor_CSRflag=0;
                    get_sensor_flag=0;
//                    gsensor[3]=0x0f;
//                    Write(W_Addr , 0x24 ,gsensor[2] );
                }
                else
                {
                    get_sensor_CSRflag=0;
                    get_sensor_flag=0; 
                }
                        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
            {
                putsUART1_2("+++");
                for(j=0;(read_uart1_dataO_flag==0)&&(j<100);j++)
                {
                    __delay_ms(10);
                }    
//                __delay_ms(1000);
//                putsUART1_2("+++"); 
            }

            putsUART1_2("\r\n");
            putsUART1_2("AT+NETCLOSE\r\n");
            request_token_flag = 0;
            read_uart1_dataO_flag=0;
            for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                {
                    __delay_ms(1);
                }
            __delay_ms(1500);
            if(notRegistered_flag==1)
            {
//                    Write(W_Addr , 0x24 ,0xff ) ; //設定Active mode 門檻值0x02
                    EX_INT0_InterruptDisable();
            }
            else
            {
                EX_INT0_InterruptEnable();
            }
        //D11_SetHigh();
        //VBUS_CTRL_SetLow();
        }
}


void sendDvrphotoSim()//make package before send
{
        CALL_DVR_SetHigh();
        read_uart1_dataW1_flag = 0;
        open_socket_fail_flag = 0;
//        CALL_DVR_SetHigh();
        ClearFlag();
        putsUART1_2("AT+CIPMODE=1\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+CGSOCKCONT=1,\"IP\",\"internet.iot\"\r\n");
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+NETOPEN\r\n"); //
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(10);
            }
               open_socket_error_flag=0;
//        read_uart1_dataO_flag=0;
//        putsUART1_2("AT+CIPOPEN=0,\"TCP\",\"40.74.74.255\",18500\r\n"); //
//        __delay_ms(20);
//        read_uart1_data30_flag=0;
//        for(i=0;(read_uart1_data30_flag<2)&(i<500);i++)//prevent stock
//            {
//                __delay_ms(100);
//            }
//        read_uart1_dataO_flag=0;
       
        open_socket_error_flag=0;
        open_socket_fail_flag=0;
                UART1_Initialize();
        putsUART1_2(IOT_DOMAIN); //
        __delay_ms(20);
        read_uart1_data30_flag=0;
            for(i=0;(read_uart1_data30_flag<2)&(i<=500);i++)//prevent stock
            {
                __delay_ms(10);
                if(i>=500)
                {
                    i=0;
                    open_socket_fail_flag++;
                    UART1_Initialize();
                    read_uart1_dataO_flag=0;
                    for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
                        {
                            __delay_ms(1000);
                            putsUART1_2("+++"); 
                        }
                    putsUART1_2("\r\n");
                    putsUART1_2("AT+NETCLOSE\r\n");
                    __delay_ms(20);
                    putsUART1_2("\r\n");
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    putsUART1_2("AT+NETOPEN\r\n"); //
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(10);
            }
               open_socket_error_flag=0;
                    putsUART1_2(IOT_DOMAIN); //
                    __delay_ms(20);
                    read_uart1_data30_flag=0;
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    
                }
                if(open_socket_fail_flag>=5)
                {
                    open_socket_fail_flag=0;
                    open_socket_error_flag=1;
                    putsUART1_2("AT+NETCLOSE\r\n");
                    //UART1_Initialize();
                    break;
                }
//                else
//                {
//                    open_socket_error_flag=0;
//                }
            }
            //__delay_ms(500);
        if(open_socket_error_flag==0)
        {
//            putsUART1_2package(sendtoken);
//            UART1_Initialize();
//            UART3_Initialize();
            CALL_DVR_SetHigh();
            __delay_ms(4000);
            BT_Send_String(sendtoken,sizeof(sendtoken));
            __delay_ms(5);
                    BT_Send_String(sendexception,sizeof(sendexception));
//            __delay_ms(50);//1119/500
//                                                                putsUART1_3("\x20\x01\x03\x01\x04\xA5\xFF");//NEW COMMAND
                                                                    __delay_ms(500);
            CALL_DVR_SetHigh();

//                for(read_uart3_data_flag=0,i=0;(i<4)&&(read_uart3_data_flag==0);i++)//delay time
//                {
//                    CALL_DVR_SetHigh();
////                putsUART1_3("\x20\x02\x04\x01\x01\x06\xA5\xFF");//All  test2018Test_data1
//
//                                putsUART1_3("\x20\x05\x04\x01\xC8\xCD\xA5\xFF");//NEW COMMAND
//                __delay_ms(3500);
//                }
//            __delay_ms(1500);
            send_image_flag=1;
            read_uart3_data_flag=0;
                                            
//                for(read_uart3_data_flag=0,i=0;(i<3)&&(read_uart3_data_flag==0);i++)
//                {
////                putsUART1_3("\x20\x02\x04\x01\x01\x06\xA5\xFF");//All  test2018Test_data1
//                                            putsUART1_3("\x20\x05\x04\x01\x32\x37\xA5\xFF");//All  test2018Test_data1
//                __delay_ms(200);
//                }         
//                               UART3_Initialize();   
                for(read_uart3_data_flag=0,i=0;(i<3)&&(read_uart3_data_flag==0);i++)
                {
                    CALL_DVR_SetHigh();
//                putsUART1_3("\x20\x02\x04\x01\x01\x06\xA5\xFF");//All  test2018Test_data1

                                putsUART1_3("\x20\x02\x05\x01\x01\x02\x09\xA5\xFF");//NEW COMMAND
                __delay_ms(3500);
                }
            
                for(read_uart3_data_flag=0,i=0;(i<3)&&(read_uart3_data_flag==0);i++)
                {
                    putsUART1_3("\x20\x02\x05\x01\x01\x00\x07\xA5\xFF");//NEW COMMAND
                __delay_ms(1500);
                }
            
                                            
            __delay_ms(1500);
            
            for(i=0, finish1s_flag=0;(finish1s_flag==0)&&(i<300);i++)//prevent stock
            {
                __delay_ms(100);
                if(read_uart3_data_flag==1)
                {
                    read_uart3_data_flag=0;
                    finish1s_flag=0;
                    TMR1_Stop();
                    TMR1_SoftwareCounterClear();
                    TMR1_Start();
                }
            }
            
        putsUART1_3("\x20\x01\x02\x00\xA5\xFF");//ACK DVR
        
            send_image_flag=0;
            CALL_DVR_SetLow();
            __delay_ms(500);
            UART1_Initialize();
            putsUART1_2package77("\x20\x51\x00\x09\x12\x01\x01\x00\x00\x00\x00\x6E\x2A\x77");
//            read_uart1_data30_flag=0;
            read_uart1_dataQ_flag=0;
            read_uart1_data03_flag=0;
            read_uart1_data01_flag=0;
            read_uart1_data55_flag=0;
            read_uart1_data2A_flag=0;
//            for(i=0;((read_uart1_data30_flag==0)||(read_uart1_data51_flag==0)||(read_uart1_data03_flag==0)||(read_uart1_data01_flag==0)||(read_uart1_data55_flag==0)||(read_uart1_data2A_flag==0))&&(i<300);i++)//prevent stock
//            {
//                __delay_ms(10);
//            }
            
            for(i=0;(!((read_uart1_dataQ_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data55_flag==1)&&(read_uart1_data2A_flag==1)))&&(i<300);i++)//prevent stock
            {
                __delay_ms(10);
            }
            if((read_uart1_dataQ_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data55_flag==1)&&(read_uart1_data2A_flag==1))
            {
               dvr_Server_ACK_flag=1;
            }
            else
            {
                dvr_Server_ACK_flag=0;
            }
            
            BT_Send_String(sendend,sizeof(sendend));
            ClearFlag();
            read_uart1_data30_flag=0;
            
            for(i=0;(read_uart1_data30_flag==0)&(i<500);i++)//prevent stock wait end
            {
                __delay_ms(10);
            }
            if((read_uart1_data30_flag==1)&&(read_uart1_data62_flag==1))//ACK　ｔｏ　ｃｌｏｕｄ　ｓｅｒｖｅｒ
            {
                putsUART1_2package77("\x10\x62\x03\x01\x66\x2A\x77");
            }
//            __delay_ms(1500);
            //TMR1_Stop();
            
        
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
            {
                putsUART1_2("+++");
                for(j=0;(read_uart1_dataO_flag==0)&&(j<100);j++)
                {
                    __delay_ms(10);
                }    
//                __delay_ms(1000);
//                putsUART1_2("+++"); 
            }
        read_uart1_dataO_flag=0;
        read_uart1_data_flag=0;
        
//        putsUART1_2("\r\n");
//        putsUART1_2("AT+NETCLOSE\r\n");
//        __delay_ms(1500);
        
        putsUART1_2("AT+NETCLOSE\r\n");
        __delay_ms(20);
        putsUART1_2("\r\n");
        read_uart1_dataO_flag=0;
        for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
        {
            __delay_ms(1);
        }
        putsUART1_2("AT+CIPMODE=0\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        //D11_SetHigh()
        }
    
}


void sendDvrphotoSim2()//make package before send
{
        read_uart1_dataW1_flag = 0;
        open_socket_fail_flag = 0;
//        CALL_DVR_SetHigh();
        ClearFlag();
        putsUART1_2("AT+CIPMODE=1\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+CGSOCKCONT=1,\"IP\",\"internet.iot\"\r\n");
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+NETOPEN\r\n"); //
        read_uart1_data30_flag=0;
        for(i=0;(read_uart1_data30_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(10);
            }
//        read_uart1_dataO_flag=0;
//        putsUART1_2("AT+CIPOPEN=0,\"TCP\",\"40.74.74.255\",18500\r\n"); //
//        __delay_ms(20);
//        read_uart1_data30_flag=0;
//        for(i=0;(read_uart1_data30_flag<2)&(i<500);i++)//prevent stock
//            {
//                __delay_ms(100);
//            }
//        read_uart1_dataO_flag=0;
       
        open_socket_error_flag=0;
        open_socket_fail_flag=0;
        putsUART1_2(IOT_DOMAIN); //
        __delay_ms(20);
        read_uart1_data30_flag=0;
            for(i=0;(read_uart1_data30_flag<2)&(i<=500);i++)//prevent stock
            {
                __delay_ms(10);
                if(i>=500)
                {
                    i=0;
                    open_socket_fail_flag++;
                    UART1_Initialize();
                    read_uart1_dataO_flag=0;
                    for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
                        {
                            __delay_ms(1000);
                            putsUART1_2("+++"); 
                        }
                    putsUART1_2("\r\n");
                    putsUART1_2("AT+NETCLOSE\r\n");
                    __delay_ms(20);
                    putsUART1_2("\r\n");
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    putsUART1_2("AT+NETOPEN\r\n"); //
                    read_uart1_data30_flag=0;
                    for(j=0;(read_uart1_data30_flag==0)&(j<500);j++)//prevent stock
                        {
                            __delay_ms(10);
                        }
                    putsUART1_2(IOT_DOMAIN); //
                    __delay_ms(20);
                    read_uart1_data30_flag=0;
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    
                }
                if(open_socket_fail_flag>=5)
                {
                    open_socket_fail_flag=0;
                    open_socket_error_flag=1;
                    putsUART1_2("AT+NETCLOSE\r\n");
                    //UART1_Initialize();
                    break;
                }
//                else
//                {
//                    open_socket_error_flag=0;
//                }
            }
            //__delay_ms(500);
        if(open_socket_error_flag==0)
        {
//            putsUART1_2package(sendtoken);
            BT_Send_String(sendtoken,sizeof(sendtoken));
            __delay_ms(5);
                    BT_Send_String(sendexception,sizeof(sendexception));
            __delay_ms(500);
            
            CALL_DVR_SetHigh();
            send_image_flag=1;
        putsUART1_3("\x20\x02\x04\x01\x01\x06\xA5\xFF");//All  test2018Test_data1
        __delay_ms(1000);
//        TMR1_Stop();
//        TMR1_SoftwareCounterClear();
//        TMR1_Start();
//            finish1s_flag=0;
            //TMR1_Stop();
            //__delay_ms(15000);
//            for(i=0;(finish1s_flag==0)&(i<300);i++)//prevent stock
//            {
//                __delay_ms(100);
//            }
//            TMR1_Stop();
            
            //__delay_ms(15000);
            
            for(i=0, finish1s_flag=0;(finish1s_flag==0)&&(i<300);i++)//prevent stock
            {
                __delay_ms(100);
                if(read_uart3_data_flag==1)
                {
                    read_uart3_data_flag=0;
                    finish1s_flag=0;
                    TMR1_Stop();
                    TMR1_SoftwareCounterClear();
                    TMR1_Start();
                }
            }
            
        putsUART1_3("\x20\x01\x02\x00\xA5\xFF");//ACK DVR
        
            send_image_flag=0;
            __delay_ms(500);
            UART1_Initialize();
            putsUART1_2package77("\x20\x51\x00\x09\x12\x01\x01\x00\x00\x00\x00\x6E\x2A\x77");
//            read_uart1_data30_flag=0;
            read_uart1_dataQ_flag=0;
            read_uart1_data03_flag=0;
            read_uart1_data01_flag=0;
            read_uart1_data55_flag=0;
            read_uart1_data2A_flag=0;
//            for(i=0;((read_uart1_data30_flag==0)||(read_uart1_data51_flag==0)||(read_uart1_data03_flag==0)||(read_uart1_data01_flag==0)||(read_uart1_data55_flag==0)||(read_uart1_data2A_flag==0))&&(i<300);i++)//prevent stock
//            {
//                __delay_ms(10);
//            }
            
            for(i=0;(!((read_uart1_dataQ_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data55_flag==1)&&(read_uart1_data2A_flag==1)))&&(i<5);i++)//prevent stock
            {
                __delay_ms(1000);
                      if((read_uart1_dataQ_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data55_flag==1)&&(read_uart1_data2A_flag==1))
                      {
                          break;
                      }
                putsUART1_2package77("\x20\x51\x00\x09\x12\x01\x01\x00\x00\x00\x00\x6E\x2A\x77");
            }
            if((read_uart1_dataQ_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data55_flag==1)&&(read_uart1_data2A_flag==1))
            {
               dvr_Server_ACK_flag=1;
            }
            else
            {
                dvr_Server_ACK_flag=0;
            }
            
            BT_Send_String(sendend,sizeof(sendend));
            ClearFlag();
            read_uart1_data30_flag=0;
            
            for(i=0;(read_uart1_data30_flag==0)&(i<500);i++)//prevent stock wait end
            {
                __delay_ms(10);
            }
            if((read_uart1_data30_flag==1)&&(read_uart1_data62_flag==1))//ACK　ｔｏ　ｃｌｏｕｄ　ｓｅｒｖｅｒ
            {
                putsUART1_2package77("\x10\x62\x03\x01\x66\x2A\x77");
            }
//            __delay_ms(1500);
            //TMR1_Stop();
            
        
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&&(i<3);i++)//prevent stock
            {
                putsUART1_2("+++");
                for(j=0;(read_uart1_dataO_flag==0)&&(j<100);j++)
                {
                    __delay_ms(10);
                }    
//                __delay_ms(1000);
//                putsUART1_2("+++"); 
            }
        read_uart1_dataO_flag=0;
        read_uart1_data_flag=0;
        
//        putsUART1_2("\r\n");
//        putsUART1_2("AT+NETCLOSE\r\n");
//        __delay_ms(1500);
        
        putsUART1_2("AT+NETCLOSE\r\n");
        __delay_ms(20);
        putsUART1_2("\r\n");
        read_uart1_dataO_flag=0;
        for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
        {
            __delay_ms(1);
        }
        putsUART1_2("AT+CIPMODE=0\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        //D11_SetHigh()
        }
    
}

void sendFlashDataSim()//make package before send
{
        read_uart1_dataW1_flag = 0;
        open_socket_fail_flag = 0;
        ClearFlag();
        putsUART1_2("AT+CIPMODE=1\r\n");
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        putsUART1_2("AT+CGSOCKCONT=1,\"IP\",\"internet.iot\"\r\n");
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        UART1_Initialize();
        putsUART1_2("AT+NETOPEN\r\n"); 
        __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(10);
            }
               open_socket_error_flag=0;
        open_socket_fail_flag=0;
        UART1_Initialize();
        putsUART1_2(IOT_DOMAIN); 
        __delay_ms(1);
        read_uart1_data30_flag=0;
            for(i=0;(read_uart1_data30_flag<2)&&(i<=500);i++)//prevent stock
            {
                __delay_ms(10);
                if(i>=500)
                {
                    i=0;
                    open_socket_fail_flag++;
                    UART1_Initialize();
                    read_uart1_dataO_flag=0;
                    for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
                        {
                            __delay_ms(1000);
                            putsUART1_2("+++"); 
                        }
                    putsUART1_2("\r\n");
                    putsUART1_2("AT+NETCLOSE\r\n");
                    __delay_ms(20);
                    putsUART1_2("\r\n");
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    putsUART1_2("AT+NETOPEN\r\n"); //
                    read_uart1_data30_flag=0;
                    for(j=0;(read_uart1_data30_flag==0)&(j<500);j++)//prevent stock
                        {
                            __delay_ms(10);
                        }
                    putsUART1_2(IOT_DOMAIN); //
                    __delay_ms(1);
                    read_uart1_data30_flag=0;
                    read_uart1_dataO_flag=0;
                    for(j=0;(read_uart1_dataO_flag==0)&(j<500);j++)//prevent stock
                    {
                        __delay_ms(1);
                    }
                    
                }
                if(open_socket_fail_flag>=2)
                {
                    open_socket_fail_flag=0;
                    open_socket_error_flag=1;
                    putsUART1_2("AT+NETCLOSE\r\n");
                    //UART1_Initialize();
                    break;
                }
            }
        
        if(open_socket_error_flag==0)
        {
            //__delay_ms(500);
//            putsUART1_2package(sendtoken);
                BT_Send_String(sendtoken,sizeof(sendtoken));
            __delay_ms(5);
                    BT_Send_String(sendexception,sizeof(sendexception));
            __delay_ms(500);
            
            sendSpiFlashdata();
//                                putsUART1_2("AT+NETCLOSE\r\n");
            //putsUART1_2package(sendgsensor);
            //putsUART1_2package(sendgps);
            __delay_ms(500);
            BT_Send_String(sendend,sizeof(sendend));
            //TMR1_Stop();
            
                   read_uart1_data30_flag=0;
            read_uart1_data62_flag=0;
            read_uart1_data03_flag=0;
            read_uart1_data01_flag=0;
            read_uart1_data66_flag=0;
            read_uart1_data2A_flag=0;
            for(i=0;((read_uart1_data30_flag==0)||(read_uart1_data62_flag==0)||(read_uart1_data03_flag==0)||(read_uart1_data01_flag==0)||(read_uart1_data66_flag==0)||(read_uart1_data2A_flag==0))&&(i<300);i++)//prevent stock
            {
                __delay_ms(10);
            }
            if((read_uart1_data30_flag==1)&&(read_uart1_data62_flag==1)&&(read_uart1_data03_flag==1)&&(read_uart1_data01_flag==1)&&(read_uart1_data66_flag==1)&&(read_uart1_data2A_flag==1))
            {
                putsUART1_2package77("\x10\x62\x03\x01\x66\x2A\x77");
                __delay_ms(50);
            }
        
//        __delay_ms(15000);
//        putsUART1_2("+++"); 
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&&(i<5);i++)//prevent stock
            {
                putsUART1_2("+++");
                for(j=0;(read_uart1_dataO_flag==0)&&(j<100);j++)
                {
                    __delay_ms(10);
                }    
//                __delay_ms(1000);
//                putsUART1_2("+++"); 
            }
        read_uart1_dataO_flag=0;
        read_uart1_data_flag=0;
        
        putsUART1_2("\r\n");
        putsUART1_2("AT+CIPCLOSE=0\r\n");
                __delay_ms(10);
        read_uart1_dataO_flag=0;
        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
            {
                __delay_ms(1);
            }
        sipFlash_data_flag=0;
        spiChipErase();
        //D11_SetHigh();
        }
    
}

void ClearFlag()
{
 read_uart1_data_flag = 0;
 read_uart1_dataO_flag = 0;
 read_uart1_dataN_flag = 0;
 read_uart1_dataI_flag = 0;
 read_uart1_start_flag = 0;
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
 //read_uart1_dataack_flag = 0;
 //read_uart1_datatime_flag = 0;
 read_uart1_data30_flag = 0;
}

void ClearUart1Flag()
{
    read_uart1_data_flag = 0;
    read_uart1_dataO_flag = 0;
    read_uart1_dataK_flag = 0;
    read_uart1_dataP_flag = 0;
    read_uart1_dataB_flag = 0;
    read_uart1_dataN_flag = 0;
    read_uart1_dataI_flag = 0;
    read_uart1_start_flag = 0;
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
    read_uart1_dataPlus_flag = 0;
    read_uart1_data30_flag = 0;
}


void G_sensorReset()
{
    Write(W_Addr , 0x2E ,0x00 );
    Read( W_Addr, 0x30 ,R_Addr);
    Write(W_Addr , 0x2E ,0x10 );
}

void wake()
{
        SYSTEM_Initialize();
    PWR_CTRL_SetHigh(); 
    SEN_PWR_CTRL_SetHigh();//open battery test
    VBUS_CTRL_SetLow();
    GPS_RESET_SetHigh();
    IO_4G_REST_SetLow();
    IO_1V8_CTRL_SetLow();
    IO_4G_PWR_CRL_SetLow();
    SS1_SetHigh();
    
//    IO_24ltest_SetHigh();
    
//        IO_4G_ON_SetLow(); 
//    __delay_ms(500);
//    IO_4G_ON_SetHigh();    //ON SINAL
//    __delay_ms(500);
//    IO_4G_ON_SetLow(); 
//    IO_4G_REST_SetLow(); 
    
        IO_4G_ON_SetLow(); 
    IO_4G_REST_SetLow(); 
    __delay_ms(500);
    IO_4G_ON_SetHigh();    //ON SINAL
    IO_4G_REST_SetHigh(); 
    __delay_ms(500);
    IO_4G_ON_SetLow(); 
    IO_4G_REST_SetLow(); 
    
    _LATC6 = 1;
    _LATC8 = 1;
    _LATC1 = 1;
    _LATB9 = 1;//Set Low
//    SYSTEM_Initialize();
    D10_SetHigh();
    G_sensorReset();
    //D11_SetHigh();
    WDT_WatchdogtimerSoftwareDisable();
    
}

void test_4G_reset()
{
    IO_4G_REST_SetHigh(); 
    __delay_ms(200);
    IO_4G_REST_SetLow(); 
    
    IO_4G_ON_SetLow(); 
    IO_4G_REST_SetLow(); 
    __delay_ms(500);
    IO_4G_ON_SetHigh();    //ON SINAL
    IO_4G_REST_SetHigh(); 
    __delay_ms(500);
    IO_4G_ON_SetLow(); 
    IO_4G_REST_SetLow(); 
}

void Sleep1()
{
    tokenIsNULL_flag=0;
////    Car_information_patch_1[26]=0x0;
//    if(toggleWake_flag!=0)
//    {
//        toggleWake_flag=0;
//        UART1_Flag_reset();
//        G_sensorReset();
//    }
    
    
    UART1_Flag_reset();
                        get_sensor_CSRflag=0;
    send10MFlag=0;
    test_finish1M_flag=0;
    isABnormalState_flag=0;
    canabnormal_flag=0;
    PWR_CTRL_SetLow(); 
    SEN_PWR_CTRL_SetHigh();
    VBUS_CTRL_SetLow();
    GPS_RESET_SetLow();
    IO_4G_REST_SetLow();
    IO_1V8_CTRL_SetLow();
    IO_4G_ON_SetLow(); 
    IO_4G_PWR_CRL_SetLow();
    D10_SetLow();
    D11_SetLow();
    SCK1_SetLow();
    SS1_SetLow();
    SDI1_SetLow();
    CALL_DVR_SetLow();
    SDO1_SetLow();
    
    
        if(BATT_SEN2_GetValue()==1)//higher than 10V
    {
        SEN_PWR_CTRL_SetHigh();
        EX_INT2_InterruptEnable();
        WDT_WatchdogtimerSoftwareEnable();
    }
    else
    {
//        SEN_PWR_CTRL_SetLow();
//        EX_INT2_InterruptDisable();
            
                SEN_PWR_CTRL_SetLow();
        EX_INT2_InterruptDisable();
                EX_INT0_InterruptDisable();
                WDT_WatchdogtimerSoftwareDisable();

    }
    
    if(BATT_SEN1_GetValue()==1)//higher than 11V
    {
        WDT_WatchdogtimerSoftwareDisable();
    }
    
    
//    WDT_WatchdogtimerSoftwareDisable();
       U1MODEbits.UARTEN = 0;  // enabling UARTEN bit
   U1STAbits.UTXEN = 0; 
      U2MODEbits.UARTEN = 0;  // enabling UARTEN bit
   U2STAbits.UTXEN = 0; 
      U3MODEbits.UARTEN = 0;  // enabling UARTEN bit
   U3STAbits.UTXEN = 0; 
         U4MODEbits.UARTEN = 0;  // enabling UARTEN bit
   U4STAbits.UTXEN = 0; 
//   IO_24ltest_SetLow();
    _LATC6 = 0;
    _LATC8 = 0;
    _LATC1 = 0;
    _LATB9 = 0;//Set Low
    
    //close spi
    SPI1CON1 = 0x0;
    SPI1CON2 = 0x0;
    SPI1STAT = 0x0;
    
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
            SEN_PWR_CTRL_SetHigh();
    

//            spiChipErase();
//    G_sensorReset();
    g_sensor_toggle_flag=0;
    ECAN1_sleep();
    // Disable can interrupts 
    IEC2bits.C1IE = 0;
    C1INTE = 0;
    
//    WDT_WatchdogtimerSoftwareEnable();//===========================================================================================================10/25
    wake_flag=1;
    accToggle_flag=0;
//    toggleWake_flag=0;
    lowBattery_flag=0;
    makeBATTERYpackage_flag=1;
//        G_sensorReset();
    toggleWake_flag=0;
    rssi[1]=0;
    startCanUARTtoggle_flag=1;//can uart toggle when sleep
    Sleep();
}

void delay( unsigned long i)
{
	unsigned long j=0;
    for(;j<i;j++)
	{
	}
}


void WRITE_TEMP( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        send_TEMP[send_TEMP_Index++]=buffer[numBytesWritten++];
    }

}


void WRITE_PACKAGE()
{

    unsigned int numBytes = 0;
        unsigned int  writebufferLen = send_TEMP_Index;
        
        send_TEMP_Index=0;
        
        UART1_Initialize();
        while(numBytes < writebufferLen)
        {    
            int bytesToWrite = UART1_TransmitBufferSizeGet();
            numBytes += UART1_WriteBuffer ( send_TEMP+numBytes, bytesToWrite)  ;
        }

}

void CheckBatteryStatus()
{

    __delay_ms(1000);
            if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))//low battery notification  &&(ACC_DET_GetValue()==0)
            {
                __delay_ms(1500);
                
                if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))//check battery status twice
                {
                    __delay_ms(500);
                    if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                    {
                        __delay_ms(2000);
                        if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                        {
                            lowBattery_flag=1;
                        }
                        else
                        {
                            lowBattery_flag=0;
                        }
                    }
                    else
                    {
                        lowBattery_flag=0;
                    }
                }
                else
                {
                    lowBattery_flag=0;
                }
            }  
            else
            {
                lowBattery_flag=0;
            }

}


void MakeWrite_All_BufferPackage()
{

                    makeRSSIpackage();
//                    makeBATTERYpackage();
                    makeGSENSORpackage();
//                    makeGPSpackage2();
//                    makeCANpackage1();
//                    makeCANpackage2();
                    
//                    WRITE_TEMP(sendcan1,sizeof(sendcan1));
//                    WRITE_TEMP(sendcan2,sizeof(sendcan2));
//                    WRITE_TEMP(sendgps2,sizeof(sendgps2));
//                    WRITE_TEMP(sendcan1,sizeof(sendcan1));
                    WRITE_TEMP(sendgsensor,sizeof(sendgsensor));
//                    WRITE_TEMP(sendbattery,sizeof(sendbattery));
                    WRITE_TEMP(sendrssi,sizeof(sendrssi));
                    if(read_uart4_data_flag==1)//can data
                    {
                        read_uart4_data_flag=0;
                        makeCANpackage1();
                        WRITE_TEMP(sendcan1,sizeof(sendcan1));
                    }
                    if(read_gps_Available_flag==1)//gps data
                    {
                        makeGPSpackage2();
                        WRITE_TEMP(sendgps2,sizeof(sendgps2));
                    }
                    if(makeBATTERYpackage_flag==1)//battery data
                    {
                        makeBATTERYpackage_flag=0;//didn't send battery at first time
//                        makeBATTERYpackage();
//                        WRITE_TEMP(sendbattery,sizeof(sendbattery));
                    }
                    

}


void clear10Mtimer()
{
//    clearTimer2CallBack_flag=1;
    TMR2_Initialize();
    TMR2_SoftwareCounterClear();
    finish10M_flag=0;
    TMR2_Stop();
    TMR2_SoftwareCounterClear();
    TMR2_Start();
}
void send10Minutes()
{
            while((finish10M_flag==0))//open_socket_error_flag
            {
                WDT_WatchdogTimerClear();
                    if(gps_count>=5)
                    {
//                        if(Car_information_patch_1[26]==0xff)//25    Check lock status
//                        {
//                            canUnlock_flag=1;
//                        }
//                        else{
//                            canUnlock_flag=0;
//                        }
                        
                        makeTOKENpackage();
                        makeENDpackage();
                        makeEXCEPTIONpackageNormal();
                        gps_count=0;
                        testShortPACKAGE();
//                        if(rssi[1]==0x39)
//                        {
//                            break;
//                        }
                    }
                    if (gps_one_sflag==1)
                    {
                        gps_one_sflag=0;
                        gps_count++;
                        
                        putsUART1_2("AT\r\n");
                        read_uart1_dataO_flag=0;
                        for(i=0;(read_uart1_dataO_flag==0)&(i<500);i++)//prevent stock
                            {
                                __delay_ms(1);
                            }
                        if(read_uart1_dataO_flag==0)
                        {
                            
                        }
                        
                        MakeWrite_All_BufferPackage();
                    }
                                            if((g_sensor_toggle_flag==1)||(ACC_DET_GetValue()==1))
                                            {
                                                CALL_DVR_SetHigh();
//                                                __delay_ms(3000);
                                                break;
                                            }
                
                                            if((BATT_SEN1_GetValue()==0)||(BATT_SEN2_GetValue()==0))
                                            {
                                                CheckBatteryStatus();
                                                if(lowBattery_flag==1)
                                                {
                                                    break;
                                                }
                                            }        
                    if(test_4G_rest_flag>=3)
                    {
                        test_4G_rest_flag=0;
        //                wake_flag==1;
        //                toggleWake_flag==1;
                        test_4G_reset();
                        putsUART1_2("********************************************************************************\r\n");
                        putsUART1_2("\t\t RESET____4G\r\n");
                        putsUART1_2("********************************************************************************\r\n");
                    }
                
            }
}


void makeTESTpackage()
{
    readIMEI2();
    readIMSI();
    readRSSI2();
    readADXL345();
                    for(i=0;i<80;i++)
                    {    
                        testMessage[i]=0;
                    }
    
                    for(i=0;i<15;i++)
                    {    
                        testMessage[i]=imei[i];
                    }
   testMessage[15]=0x09;
                    for(i=0;i<15;i++)
                    {    
                        testMessage[i+16]=imsi[i];
                    }
    testMessage[31]=0x09;
    //g-sensor
    for(i=0;i<4;i++)
    {
        if(strx[i]==0x0)
        {strx[i]=0x20;}
        if(stry[i]==0x0)
        {stry[i]=0x20;}
        if(strz[i]==0x0)
        {strz[i]=0x20;}
        
    }
    
                    for(i=0;i<4;i++)
                    {    
                        testMessage[i+32]=strx[i];
                    }
                    for(i=0;i<4;i++)
                    {    
                        gxl=Read( W_Addr, 0x32 ,R_Addr);
                        testMessage[i+36]=stry[i];
                    }
                    for(i=0;i<4;i++)
                    {    
                        gy0=Read( W_Addr, 0x35 ,R_Addr);
                        testMessage[i+40]=strz[i];
                    }
   testMessage[44]=0x09;
   //GPS
                    for(i=0;i<9;i++)
                    {    
                        testMessage[i+45]=lat2[i];
                    }
                    for(i=0;i<10;i++)
                    {    
                        testMessage[i+54]=lon2[i];
                    }
   testMessage[64]=0x09;
   //rssi
                   for(i=0;i<3;i++)
                    {    
                        testMessage[i+65]=rssi[i];
                    }
   testMessage[68]=0x09;
   testMessage[69]=Rx3Value;//DVR
   testMessage[70]=0x09;
   testMessage[71]=Rx4Value;//CAN
   testMessage[72]=0x09;
   //battery
   if(BATT_SEN1_GetValue()==1)
    {
        testMessage[73]='H';
    }
    else if (BATT_SEN2_GetValue()==1)
    {
        testMessage[73]='M';
    }
    else
    {
        testMessage[73]='L';
    }
   
   testMessage[74]=0x09;
//   testMessage[75]=0x09;
//                    sendimsiimei[41]='\xff';

}


void makeTESTpackage2()
{
    readIMEI2();
    readIMSI();
    readRSSI2();
    readADXL345();
                    for(i=0;i<80;i++)
                    {    
                        testMessage[i]=0;
                    }
    
                    for(i=0;i<15;i++)
                    {    
                        testMessage[i]=imei[i];
                    }
   testMessage[15]=0x09;
                    for(i=0;i<15;i++)
                    {    
                        testMessage[i+16]=imsi[i];
                    }
    testMessage[31]=0x09;
    //g-sensor
    for(i=0;i<4;i++)
    {
        if(strx[i]==0x0)
        {strx[i]=0x20;}
        if(stry[i]==0x0)
        {stry[i]=0x20;}
        if(strz[i]==0x0)
        {strz[i]=0x20;}
        
    }
    
                    for(i=0;i<4;i++)
                    {    
                        testMessage[i+32]=strx[i];
                    }
                    for(i=0;i<4;i++)
                    {    
                        gxl=Read( W_Addr, 0x32 ,R_Addr);
                        testMessage[i+36]=stry[i];
                    }
                    for(i=0;i<4;i++)
                    {    
                        gy0=Read( W_Addr, 0x35 ,R_Addr);
                        testMessage[i+40]=strz[i];
                    }
   testMessage[44]=0x09;
   //GPS
                    for(i=0;i<9;i++)
                    {    
                        testMessage[i+45]=lat2[i];
                    }
                    for(i=0;i<10;i++)
                    {    
                        testMessage[i+54]=lon2[i];
                    }
   testMessage[64]=0x09;
   //rssi
                   for(i=0;i<3;i++)
                    {    
                        testMessage[i+65]=rssi[i];
                    }
   testMessage[68]=0x09;
   testMessage[69]=Rx3Value;//DVR
   testMessage[70]=0x09;
   testMessage[71]=Rx4Value;//CAN
   testMessage[72]=0x09;
   //battery
   if(BATT_SEN1_GetValue()==1)
    {
        testMessage[73]='H';
    }
    else if (BATT_SEN2_GetValue()==1)
    {
        testMessage[73]='M';
    }
    else
    {
        testMessage[73]='L';
    }
   
   testMessage[74]=0x09;
//   testMessage[75]=0x09;
//                    sendimsiimei[41]='\xff';

}