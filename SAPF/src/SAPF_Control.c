/*
 * SAPF_Control.c
 *
 *  Created on: 19/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#include "../inc/SAPF_Control.h"

InputData ADC_Data;

float Vpll = 0.0;
float integral_comp_voltage = 0.0;
//###############################################
//#             User Functions                  #
//###############################################
void SAPFControl(void)
{
    //Uint16 GridDisturbance = 0;
    //float Vpeak = 0;

    Vpll = V_RMS * PhaseLockedLoop(&ADC_Data.Vgrid);

//    if(pre_charge_flag)
//      return;
//
//    float Vpf = PassiveFilterVoltage(&ADC_Data.Ilf);
//    float Avg_DC = DC_LinkAverageVoltage(DC_Voltage, &ADC_Data.Vdc);
//
//    if (Vpeak = getVpeak(&ADC_Data.Vgrid)) {
//        GridDisturbance = !VariableCheck(&Vpeak, V_PEAK);
//    }
//
//    float Vreg = DC_LinkRegulation(GridDisturbance, &Avg_DC);
//    float Vref = CompensationVoltageCalc(&ADC_Data.Vgrid, &Vpll, &Vpf, &Vreg);
//    float Vcomp = VoltageCompensation(&Vref, &ADC_Data.Vf);

    //CLOSED-LOOP
    //float Vref = (float)ADC_Data.Vgrid - Vpll;
    float Vref = Vpll - (float)ADC_Data.Vgrid;
    float Vcomp = 0.0;
    if(isSAPF_Enabled()){
        Vcomp = VoltageCompensation(&Vref, &ADC_Data.Vf);
    }else{
        integral_comp_voltage = 0.0;
    }

    Vcomp *= (float)PWM_OFFSET;
    Vcomp /= (float)ADC_Data.Vdc;
    int16 Vpwm = (int16)Vcomp;

    setPWM(Vpwm);

    DAC_Update(ADC_Data.Vgrid, Vpll, Vref, ADC_Data.Vf, Vpwm, 0, 0, 0);
}

void ADC1_WakeUpSignal(void)
{
    GpioDataRegs.GPASET.bit.GPIO26 = 1;     //Start converter 1
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;   //END start converter 1
}

void ADC1_StartConversionSignal(void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;     //START converter signal ADC 1
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBSET.bit.GPIO63 = 1;       //END start converter 1
}

void ADC1_SaveData(void)
{
    unsigned long entrada_A = 0;
    unsigned long entrada_B = 0;

    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");

    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 0
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBSET.bit.GPIO34 = 1; //read 0

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    ADC_Data.Vgrid = (entrada_A & 0x0000F000) >> 8;     //Read Pins 12, 13, 14, 15
    ADC_Data.Vgrid |= (entrada_A & 0x00F00000) >> 12;   //Read Pins 20, 21, 22, 23
    ADC_Data.Vgrid |= (entrada_A & 0xC0000000) >> 18;   //Read Pins 30, 31
    ADC_Data.Vgrid |= (entrada_B & 0x00000003) << 14;   // Read Pins 32
    ADC_Data.Vgrid >>= 0x04;
    ADC_Data.Vgrid += VGRID_OFFSET;
    ADC_Data.Vgrid = -ADC_Data.Vgrid;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34=1; //read 1

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 2

    ADC_Data.Reserv = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Reserv |= (entrada_A & 0x00F00000) >> 12;
    ADC_Data.Reserv |= (entrada_A & 0xC0000000) >> 18;
    ADC_Data.Reserv |= (entrada_B & 0x00000003) << 14;
    ADC_Data.Reserv >>= 0x04;
    ADC_Data.Reserv += RESERV_OFFSET;
    ADC_Data.Reserv =- ADC_Data.Reserv;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34=1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34=1; //read 3

    ADC_Data.ILoad = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.ILoad |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.ILoad |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.ILoad |=(entrada_B & 0x00000003) << 14;
    ADC_Data.ILoad >>= 0x04;
    ADC_Data.ILoad += ILOAD_OFFSET;
    ADC_Data.ILoad = -ADC_Data.ILoad;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 4

    ADC_Data.Vdc = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Vdc |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Vdc |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Vdc |=(entrada_B & 0x00000003) << 14;
    ADC_Data.Vdc >>= 0x04;
    ADC_Data.Vdc += VDC_OFFSET;
    ADC_Data.Vdc = -ADC_Data.Vdc;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 5

    ADC_Data.Vf = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Vf |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Vf |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Vf |=(entrada_B & 0x00000003) << 14;
    ADC_Data.Vf >>= 0x04;
    ADC_Data.Vf += VF_OFFSET;
    ADC_Data.Vf = -ADC_Data.Vf;
    ADC_Data.Vf >>= 0x02;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 6

    ADC_Data.Ilf = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Ilf |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Ilf |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Ilf |=(entrada_B & 0x00000003) << 14;
    ADC_Data.Ilf >>= 0x04;
    ADC_Data.Ilf += ILF_OFFSET;
    ADC_Data.Ilf = -ADC_Data.Ilf;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    asm("      NOP");   asm("      NOP");   asm("      NOP");
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; //read 7

    ADC_Data.Reserv1 = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Reserv1 |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Reserv1 |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Reserv1 |=(entrada_B & 0x00000003) >> 14;
    ADC_Data.Reserv1 >>= 0x04;
    ADC_Data.Reserv1 += RESERV1_OFFSET;
    ADC_Data.Reserv1 = -ADC_Data.Reserv1;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    ADC_Data.Enable = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Enable |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Enable |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Enable |=(entrada_B & 0x00000003) >> 14;
    ADC_Data.Enable >>= 0x04;
    ADC_Data.Enable += ENABLE_OFFSET;
}

void PreChargeDC_Link(void)
{
    int16 PreChargeRef = PRE_LOAD_REF * (Vpll / V_RMS);

    float VcompPreCharge = PreChargeVoltage_Compensation(&PreChargeRef, &ADC_Data.Vf);

    //float VpwmPreCharge *= VG_VDC_FATOR;
    //VpwmPreCharge *= (float)PWM_OFFSET;
    //VpwmPreCharge /= (float)ADC_Data.Vdc;
    //int16 pwm = (int16)VpwmPreCharge;

    //setPWM(pwm);

    DAC_Update(ADC_Data.Vgrid, Vpll, ADC_Data.Vdc, PreChargeRef, ADC_Data.Vf, 0, 0, 0);

    pre_charge_flag = SoftStart(&ADC_Data.Vdc);
}

int16 setPWM(int16 pwm)
{
    int16 PWM = assert_PWM(pwm);

    EPwm1Regs.CMPA.half.CMPA = -PWM + PWM_OFFSET;   //S1 and S2
    EPwm2Regs.CMPA.half.CMPA = PWM + PWM_OFFSET;    //S3 and S4

    return PWM;
}

int16 assert_PWM(int16 pwm)
{
    if(pwm > PWM_MAX)
        pwm = PWM_MAX;

    if(pwm < PWM_MIN)
        pwm = PWM_MIN;

    return pwm;
}

void DAC_Update(int16 DAC_OUT_A, int16 DAC_OUT_B, int16 DAC_OUT_C, int16 DAC_OUT_D, int16 DAC_OUT_E, int16 DAC_OUT_F, int16 DAC_OUT_G, int16 DAC_OUT_H)
{
    if(DAC_OUT_A > 2000)  DAC_OUT_A = 2000;
    if(DAC_OUT_A < -2000) DAC_OUT_A = -2000;

    if(DAC_OUT_B > 2000)  DAC_OUT_B = 2000;
    if(DAC_OUT_B < -2000) DAC_OUT_B = -2000;

    if(DAC_OUT_C > 2000)  DAC_OUT_C = 2000;
    if(DAC_OUT_C < -2000) DAC_OUT_C = -2000;

    if(DAC_OUT_D > 2000)  DAC_OUT_D = 2000;
    if(DAC_OUT_D < -2000) DAC_OUT_D = -2000;

    if(DAC_OUT_E > 2000)  DAC_OUT_E = 2000;
    if(DAC_OUT_E < -2000) DAC_OUT_E = -2000;

    if(DAC_OUT_F > 2000)  DAC_OUT_F = 2000;
    if(DAC_OUT_F < -2000) DAC_OUT_F = -2000;

    if(DAC_OUT_G > 2000)  DAC_OUT_G = 2000;
    if(DAC_OUT_G < -2000) DAC_OUT_G = -2000;

    if(DAC_OUT_H > 2000)  DAC_OUT_H = 2000;
    if(DAC_OUT_H < -2000) DAC_OUT_H = -2000;

    if(SpiaRegs.SPIFFTX.bit.TXFFST < 6){
        SpiaRegs.SPITXBUF = DAC_OUT_A + DAC_OUT_A_OFFSET;//2048 - 30;
        SpiaRegs.SPITXBUF = 0x1000 + DAC_OUT_B + DAC_OUT_B_OFFSET;//2048 + 11;
        SpiaRegs.SPITXBUF = 0x2000 + DAC_OUT_C + DAC_OUT_C_OFFSET;//2048 + 21;
        SpiaRegs.SPITXBUF = 0x3000 + DAC_OUT_D + DAC_OUT_D_OFFSET;//2048 + 3;
        SpiaRegs.SPITXBUF = 0x4000 + DAC_OUT_E + DAC_OUT_E_OFFSET;//2048 + 27;
        SpiaRegs.SPITXBUF = 0x5000 + DAC_OUT_F + DAC_OUT_F_OFFSET;//2048 + 4;
        SpiaRegs.SPITXBUF = 0x6000 + DAC_OUT_G + DAC_OUT_G_OFFSET;//2048 - 4;
        SpiaRegs.SPITXBUF = 0x7000 + DAC_OUT_H + DAC_OUT_H_OFFSET;//2048 + 10;
    }
}

Uint16 isSAPF_Enabled(void)
{
    return (!ADC_Data.Enable);
}

void ClearIntegralsErrors(void)
{
    integral_comp_voltage = 0.0;
}

//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static float PhaseLockedLoop(const int16* grid)
{
    float w = 0, error_amp = 0, error_phase = 0;

    static float amplitude = 0;
    static float wt = 0;
    static float integral_fase = 0;
    static float integral_amp = 0;

    error_amp = *grid - (amplitude * cos(wt));
    integral_amp += error_amp * cos(wt);
    amplitude = integral_amp * K_AMP_PLL;

    error_phase = error_amp * sin(wt);
    integral_fase += error_phase;

    w = KP_PI_PLL * error_phase + KI_PI_PLL * integral_fase + 2 * PI * FREQ;

    wt += w * TS;

    if(wt > 2 * PI)
        wt = 0.0;

    //return (amplitude * cos(wt));
    return sin(wt - PI / 2);
}

static Uint16 VariableCheck(float* variable, int16 value)
{
    return (((*variable * (1 - RANGE)) < value) && ((*variable * (1 + RANGE)) > value));
}

static int16 PreChargeVoltage_Compensation(const int16* pre_load_ref, const int16* Vfilter)
{
    float preLoad_voltage_comp = 0.0, preLoad_voltage_error = 0.0;
    static float integral_preLoad_voltage = 0.0;

    preLoad_voltage_error = (*pre_load_ref) - (*Vfilter);
    integral_preLoad_voltage += preLoad_voltage_error;

    preLoad_voltage_comp = PRE_LOAD_KP * preLoad_voltage_error + PRE_LOAD_KI * integral_preLoad_voltage;
    //preLoad_voltage_comp *= ((*pll) / V_PEAK);

    return preLoad_voltage_comp;
}

static float PassiveFilterVoltage(const int16* pf_current)
{
    float inductor_voltage = 0.0;
    float resistance_voltage = 0.0;
    float derivate = 0.0;
    static float inductor_current_prev = 0.0;

    derivate = ((*pf_current) - inductor_current_prev) / TS;
    inductor_voltage = LPF * derivate;
    inductor_current_prev = (*pf_current);

    resistance_voltage = RPF * (*pf_current);

    return (inductor_voltage + resistance_voltage);
}

static float DC_LinkAverageVoltage(int16* DC_Voltage, const int16* dc_voltage)
{
    static unsigned int index = 0;
    float average = 0.0;

    if(!isBufferFull(&index)){
        average = (float)getDC_LinkValues(DC_Voltage, dc_voltage, EMPTY_BUFFER) / (float)BUFFERSIZE;
        index++;
    }
    else
        average = (float)getDC_LinkValues(DC_Voltage, dc_voltage, FULL_BUFFER) / (float)BUFFERSIZE;

    return average;
}

static unsigned int isBufferFull(unsigned int* index)
{
    return (*index++ == BUFFERSIZE - 1);
}

static Uint32 getDC_LinkValues(int16* DC_Voltage, const int16* dc_voltage, enum BufferType_t mode)
{
    static Uint32 sum = 0.0;

    switch (mode)
    {
        case EMPTY_BUFFER:
            sum += getFullDC_LinkValues(DC_Voltage, dc_voltage);
        break;
        case FULL_BUFFER:
            sum = DC_LinkContiniousAverageVoltage(DC_Voltage, dc_voltage, &sum);
        break;
        default:
        break;
    }
        return sum;
}

static Uint16 getFullDC_LinkValues(int16* DC_Voltage, const int16* dc_voltage)
{
    static unsigned int index = 0;

    DC_Voltage[index++] = (*dc_voltage);

    return *dc_voltage;
}

static Uint32 DC_LinkContiniousAverageVoltage(int16* DC_Voltage, const int16* dc_voltage, Uint32* sum)
{
    static unsigned int pos = 0;

    *sum = (*sum) - (DC_Voltage[pos]) + (*dc_voltage);

    DC_Voltage[pos++] = (*dc_voltage);

    if(isBufferFull(&pos)){
        pos = 0;
    }

    return (*sum);
}

static float DC_LinkCompensation(const float* dc_voltage, float* pll)
{
    float dc_voltage_comp = 0.0, dc_voltage_error = 0.0;
    static float integral_dc_voltage = 0.0;

    dc_voltage_error = V_REF - (*dc_voltage);
    integral_dc_voltage += dc_voltage_error;

    dc_voltage_comp = KP_PI_DC * dc_voltage_error + KI_PI_DC * integral_dc_voltage;
    dc_voltage_comp *= (*pll / V_RMS);

    return dc_voltage_comp;
}

static float CompensationVoltageCalc(const int16* grid, const float* pll, const float* pf, const float* reg)
{
    float Vharm = (*grid) - (*pll);

    return (Vharm + (*pf) + (*reg));
}

static float VoltageCompensation(float* VcompRef, const int16* filter)
{
    //float LIM = (float)PWM_MAX / (V_COMP_KI * TS);
    float LIM_MAX = KI_LIMIT(V_COMP_KI);
    float LIM_MIN = -LIM_MAX;

    float voltage_comp = 0.0, voltage_comp_error = 0.0;
    //static float integral_comp_voltage = 0.0;

    voltage_comp_error = (*VcompRef) - (*filter);
    integral_comp_voltage += voltage_comp_error;

    voltage_comp = V_COMP_KP * voltage_comp_error + V_COMP_KI * TS * integral_comp_voltage;

    if (integral_comp_voltage > LIM_MAX) {
        integral_comp_voltage = LIM_MAX;
    }else if (integral_comp_voltage < LIM_MIN) {
        integral_comp_voltage = LIM_MIN;
    }

    return voltage_comp;
}

static unsigned int SoftStart(const int16* dc_link_voltage)
{
    return *dc_link_voltage > V_REF;
}

static unsigned int MaxValue(const int16* vector)
{
    Uint16 max = vector[0];
    Uint16 i = 0;

    for (i = 0; i < (BUFFERSIZE >> 0x01); i++){
        if (vector[i] > max){
            max = vector[i];
        }
    }

    return max;
}

static unsigned getVpeak(const int16* signal)
{
    static Uint16 pos = 0;

    GridVoltage[pos] = abs(*signal);

    if (pos == 400) {
        pos = 0;
        return MaxValue(signal);
    }

    return 0;
}

static float DC_LinkRegulation(Uint16 disturbance, const float* average)
{
    float vdc = ADC_Data.Vdc;

    if (!disturbance){
            if (VariableCheck(&vdc, V_REF)){
                return DC_LinkCompensation(average, &Vpll);
            }
            else if (ADC_Data.Vdc < V_REF){
                return (Vpll / V_RMS) * 5;
            }
            else {
                return (-(Vpll / V_RMS) * 10);
            }
    }
    return 0.0;
}

