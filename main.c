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
volatile Uint16 pre_charge_flag = 0;                //-> optional
volatile unsigned char rx_data = 0;
volatile int16 enable = 0;
volatile Uint16 error = ALL_OK;                     //-> optional
volatile Uint16 detectedError = ALL_OK;             //-> optional

void main(void)
{
    ConfigSystem();

    CpuTimer0Regs.TCR.bit.TSS = 0;    //Start Timer0

    GpioDataRegs.GPBSET.bit.GPIO48 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;

    memset((void*)DC_Voltage, 0, BUFFERSIZE);
    memset((void*)Grid_Voltage, 0, BUFFERSIZE >> 0x01);

    ClearStructVariables();
    Calculation_KI_Max();
    ADC1_WakeUpSignal();

    for(;;){
        if(timer0_flag){
            timer0_flag = 0;
            ADC1_StartConversionSignal();
        }
        if(xInt3_flag){
            xInt3_flag = 0;
            ADC1_SaveData();
        }
        //Check if there is no error and the enable triggered by the error is different of -1 (enable value to interrupt the PWM pulses)
        if(error == ALL_OK && enable != -1){
            SAPFControl();
        }
    }
}
