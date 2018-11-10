/*
 * main.c
 *
 *  Created on: 21/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#include <string.h>

#include "System/inc/System.h"
#include "SAPF/inc/SAPF_Control.h"

volatile Uint16 timer0_flag = 0;
volatile Uint16 xInt3_flag = 0;
volatile Uint16 pre_charge_flag = 0;
volatile unsigned char rx_data = 0;

void main(void)
{
    ConfigSystem();

    CpuTimer0Regs.TCR.bit.TSS = 0;    //Start Timer0

    GpioDataRegs.GPBSET.bit.GPIO48 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;

    memset((void*)DC_Voltage, 0, BUFFERSIZE);
    memset((void*)GridVoltage, 0, BUFFERSIZE >> 0x01);

    SciaTx_SendMsg("Starting SAPF System\n");

    ADC1_WakeUpSignal();

    for(;;){
        if(timer0_flag){
            timer0_flag = 0;
            ADC1_StartConversionSignal();
        }
        if(xInt3_flag){
            xInt3_flag = 0;
            ADC1_SaveData();
            SAPFControl();
        }
        if(pre_charge_flag){
            //PreChargeDC_Link();
        }
    }
}
