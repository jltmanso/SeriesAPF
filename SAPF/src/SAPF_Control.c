/*
 * SAPF_Control.c
 *
 *  Created on: 19/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#include "../inc/SAPF_Control.h"

inputData_t ADC_Data;
KI_Limits_t KI_Vdc, KI_PreCharge;
sharedData_t Grid_Data, VDC_Data;

float Vpll_unitary = 0.0;                               //needed for generate a precharging reference
float amplitude = 0.0;                                  //needed for check if the amplitude is syncrhonized
Uint16 new_data = 0;                                    //flag to sinalize the new data from the ADC is availabe

float integral_comp_voltage = 0.0;                      //needed to be cleared when the enable button is disable
float integral_preLoad_voltage = 0.0;                   //needed to be cleared when the enable button is disable
float integral_dc_voltage = 0.0;                        //needed to be cleared when the enable button is disable
//float KP = PRE_CHARGE_KP;
//float KI = PRE_CHARGE_KI;
float KP = V_COMP_KP;
float KI = V_COMP_KI;
//###############################################
//#             User Functions                  #
//###############################################
void SAPFControl(void)
{
    static Uint16 synced = 0;
    float Vreg = 0.0;
    float Vref = 0.0;
    float Vpf = 0.0;
    float Vcomp = 0.0;
    float VpreCharge = 0.0;
    int16 Vpwm = 0;

    if(!isSAPF_Enabled()){
        ClearIntegralsErrors();
    }

    //Only execute the control once the ADC data is update
    //Can be improved a lot
    if(!new_data){
        return;
    }else{
        new_data = 0;
    }

    float Vpll = PhaseLockedLoop(&ADC_Data.Vgrid);
    float Avg_DC = AverageVoltage(&VDC_Data, DC_Voltage, &ADC_Data.Vdc);

//SYNCHRONIZATION METHOD -> Only executes in the early instants
    if(!synced){
        Uint16 Vpeak = peakVoltage(&ADC_Data.Vgrid);
        //Uint16 Vpeak = PeakAverageVoltage(&Grid_Data, Grid_Voltage, &ADC_Data.Vgrid);
        synced = isPLL_Synced(Vpeak, amplitude);
        pre_charge_flag = synced;
    }
//PRE-CHARGE METHOD
    if(pre_charge_flag && isSAPF_Enabled()){
        VpreCharge = -(PRE_CHARGE_REF * Vpll_unitary);
        Vref = VpreCharge;
        Vcomp = PreChargeVoltage_Compensation(&Vref, &ADC_Data.Vf);
        Vpwm = setPWM(Vcomp);
        pre_charge_flag = SoftStart(&ADC_Data.Vdc);
    }
//STEADY-STATE
    if(!pre_charge_flag && synced && isSAPF_Enabled()){
        //VpreCharge = PreChargeRefCalculation(&ADC_Data.Vdc, &Vpll_unitary);
        //Vreg = DC_LinkRegulation(&Avg_DC, &Vpll_unitary);
        //Vpf = PassiveFilterVoltage(&ADC_Data.Ilf);
        Vref = CompensationVoltageCalc(&ADC_Data.Vgrid, &Vpll, &Vpll_unitary, &Vpf, &Vreg, &VpreCharge);
        Vcomp = VoltageCompensation(&Vref, &ADC_Data.Vf);
        Vpwm = setPWM(Vcomp);
    }
    DAC_Update(ADC_Data.Vgrid, Vpll, Vref, ADC_Data.Vf, 0, 0, 0, 0);
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
    //ADC_Data_f.Vgrid = (float)ADC_Data.Vgrid / VGRID_GAIN;

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
    ADC_Data.Reserv = -ADC_Data.Reserv;
    //ADC_Data_f.Reserv = (float)ADC_Data.Reserv / RESERV_GAIN;

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
    ADC_Data.ILoad >>= 0x02;                            //Load Current Sensor Turns (4 turns)
    //ADC_Data_f.ILoad = (float)ADC_Data.ILoad / ILOAD_GAIN;

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
    //ADC_Data_f.Vdc = (float)ADC_Data.Vdc / VDC_GAIN;

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
    //ADC_Data_f.Vf = (float)ADC_Data.Vf / VF_GAIN;

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
    ADC_Data.Ilf >>= 0x02;                            //Passive Filter Current Sensor Turns (4 turns)
    //ADC_Data_f.Ilf = (float)ADC_Data.Ilf / ILF_GAIN;

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
    //ADC_Data_f.Reserv1 = (float)ADC_Data.Reserv1 / RESERV1_GAIN;

    entrada_A = GpioDataRegs.GPADAT.all;
    entrada_B = GpioDataRegs.GPBDAT.all;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    ADC_Data.Enable = (entrada_A & 0x0000F000) >> 8;
    ADC_Data.Enable |=(entrada_A & 0x00F00000) >> 12;
    ADC_Data.Enable |=(entrada_A & 0xC0000000) >> 18;
    ADC_Data.Enable |=(entrada_B & 0x00000003) >> 14;
    ADC_Data.Enable >>= 0x04;
    ADC_Data.Enable += ENABLE_OFFSET;
    //ADC_Data_f.Enable = (float)ADC_DAta.Enable * ENABLE_GAIN;

    //Check Protections
    if((error = CheckProtections()) != ALL_OK){
        Protection();
        detectedError = error;
        enable = -1;
    }
    new_data = 1;
}

int16 setPWM(float Vcompensation)
{
    Vcompensation *= (float)PWM_OFFSET;
    Vcompensation /= (float)ADC_Data.Vdc;

    int16 Vpwm = (int16)Vcompensation;
    int16 PWM = assert_PWM(Vpwm);

    //Unipolar
    //EPwm1Regs.CMPA.half.CMPA = -PWM + PWM_OFFSET;   //S1 and S2
    //EPwm2Regs.CMPA.half.CMPA = PWM + PWM_OFFSET;    //S3 and S4

    //Bipolar
    EPwm1Regs.CMPA.half.CMPA = PWM + PWM_OFFSET;   //S1 and S2
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

Uint16 isSAPF_Enabled(void)
{
    return (ADC_Data.Enable < 0);
}

void ClearStructVariables(void)
{
    ADC_Data.Vgrid = 0;
    ADC_Data.Vf = 0;
    ADC_Data.Enable = 0;
    ADC_Data.ILoad = 0;
    ADC_Data.Ilf = 0;
    ADC_Data.Reserv = 0;
    ADC_Data.Reserv1 = 0;
    ADC_Data.Vdc = 0;

    Grid_Data.index = 0;
    Grid_Data.sum = 0;
    Grid_Data.pos = 0;

    VDC_Data.index = 0;
    VDC_Data.sum = 0;
    VDC_Data.pos = 0;

    KI_Vdc.lim_min = 0.0;
    KI_Vdc.lim_max = 0.0;

    KI_PreCharge.lim_min = 0.0;
    KI_PreCharge.lim_max = 0.0;
}

void Calculation_KI_Max(void)
{
    KI_Vdc.lim_max = KI_LIMIT(KI_PI_DC);
    KI_Vdc.lim_min = -KI_Vdc.lim_max;

    KI_PreCharge.lim_max = KI_LIMIT(PRE_CHARGE_KI);
    KI_PreCharge.lim_min = -KI_PreCharge.lim_max;
}
//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static float PhaseLockedLoop(const int16* grid)
{
    float w = 0.0, error_pll = 0.0, error_amp = 0.0, error_phase = 0.0;

    //static float amplitude = 0;
    static float wt = 0;
    static float integral_phase = 0;
    static float integral_amp = 0;

//    float K_AMP_LIM_MAX = KI_LIMIT(KP_AMP_PLL);
//    float K_AMP_LIM_MIN = -K_AMP_LIM_MAX;
    float KI_PHASE_LIM_MAX = KI_LIMIT(KI_PI_PLL);
    float KI_PHASE_LIM_MIN = -KI_PHASE_LIM_MAX;

    error_pll = *grid - (amplitude * sin(wt));

    error_phase = error_pll * cos(wt);
    error_amp = error_pll * sin(wt);

    integral_phase += error_phase;

    if (integral_phase > KI_PHASE_LIM_MAX) {
        integral_phase = KI_PHASE_LIM_MAX;
    }
    if (integral_phase < KI_PHASE_LIM_MIN) {
         integral_phase = KI_PHASE_LIM_MIN;
    }

    integral_amp += error_amp;

//    if (integral_amp > K_AMP_LIM_MAX) {
//        integral_amp = K_AMP_LIM_MAX;
//    }
//    if (integral_amp < K_AMP_LIM_MIN) {
//        integral_amp = K_AMP_LIM_MIN;
//    }

    amplitude = integral_amp * KP_AMP_PLL * TS;

    w = KP_PI_PLL * error_phase + KI_PI_PLL * integral_phase * TS + 2 * PI * FREQ;

    wt += w * TS;

    if(wt > 2 * PI)
        wt = 0.0;

    Vpll_unitary = sin(wt);

    return (amplitude * Vpll_unitary);
}

static float PreChargeVoltage_Compensation(const float* pre_load_ref, const int16* Vfilter)
{
    float preLoad_voltage_comp = 0.0, preLoad_voltage_error = 0.0;
    //static float integral_preLoad_voltage = 0.0;

    float KI_PRE_CHARGE_MAX = KI_LIMIT(PRE_CHARGE_KI);
    float KI_PRE_CHARGE_MIN = -KI_PRE_CHARGE_MAX;

    preLoad_voltage_error = (*pre_load_ref) - (*Vfilter);
    integral_preLoad_voltage += preLoad_voltage_error;

    if(integral_preLoad_voltage > KI_PRE_CHARGE_MAX) {
        integral_preLoad_voltage = KI_PRE_CHARGE_MAX;
    }
    if(integral_preLoad_voltage < KI_PRE_CHARGE_MIN) {
        integral_preLoad_voltage = KI_PRE_CHARGE_MIN;
    }

    preLoad_voltage_comp = KP * preLoad_voltage_error + KI * integral_preLoad_voltage;
    //preLoad_voltage_comp = PRE_CHARGE_KP * preLoad_voltage_error + PRE_CHARGE_KI *integral_preLoad_voltage;

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

static float AverageVoltage(sharedData_t* data, int16* buffer, const int16* value)
{
    //static Uint16 index = 0;
    float average = 0.0;

    if(!isBufferFull(&data->index)){
        average = (float)getValues(data, buffer, value, EMPTY_BUFFER) / (float)BUFFERSIZE;
        //data->index++;
    }
    else
        average = (float)getValues(data, buffer, value, FULL_BUFFER) / (float)BUFFERSIZE;

    return average;
}

static Uint32 getValues(sharedData_t* data, int16* buffer, const int16* value, enum BufferType_t mode)
{
    //static Uint32 sum = 0.0;

    switch (mode)
    {
        case EMPTY_BUFFER:
            data->sum += getFullValues(data, buffer, value);
        break;
        case FULL_BUFFER:
            data->sum = SlidingWindow(data, buffer, value, &data->sum);
        break;
        default:
        break;
    }
        return data->sum;
}

static Uint16 getFullValues(sharedData_t* data, int16* buffer, const int16* value)
{
    //static unsigned int index = 0;

    buffer[data->index++] = (*value);

    return (*value);
}

static Uint32 SlidingWindow(sharedData_t* data, int16* buffer, const int16* value, Uint32* sum)
{
    //static unsigned int pos = 0;

    //*sum = (*sum) - (buffer[pos]) + (*value);
    data->sum = data->sum - buffer[data->pos] + (*value);

    buffer[data->pos++] = (*value);

    if(isBufferFull(&data->pos)){
        data->pos = 0;
    }

    return (data->sum);
}

static Uint16 isBufferFull(Uint16* index)
{
    return (*index == BUFFERSIZE - 1);
}

static Uint16 isHalfBufferFull(unsigned int* index)
{
    return (*index++ == HALF_BUFFERSIZE - 1);
}

//static float PeakAverageVoltage(sharedData_t* data, int16* buffer, const int16* value)
//{
//    static int16 Vpeak = 0;
//    static Uint16 index = 0;
//
//    int16 abs_value = abs(*value);
//
//    // Check for the highest value of the semicicle of the waveform
//    if(!isHalfBufferFull(&index)){
//        if(abs_value > Vpeak)
//            Vpeak = abs_value;
//    }else {
//        index = 0;
//    }
//    //  Return the average value of the peak waveform
//    return AverageVoltage(data, buffer, &Vpeak);
//}

static Uint16 peakVoltage(const int16* value)
{
    static Uint16 Vpeak = 0;

    int16 abs_value = abs(*value);

    // Check for the highest value of the semicicle of the waveform
    if(abs_value > Vpeak){
        Vpeak = abs_value;
    }

    return Vpeak;
}

static float PreChargeRefCalculation(const int16* dc_voltage, const float* pll)
{
    float pre_charge_ref_error = 0.0;
    float pre_charge_ref_comp = 0.0;

    pre_charge_ref_error = V_REF - (*dc_voltage);

    if(pre_charge_ref_error > PRE_CHARGE_REF){
        pre_charge_ref_error = PRE_CHARGE_REF;
    }
    if(pre_charge_ref_error < (-PRE_CHARGE_REF)){
        pre_charge_ref_error = (-PRE_CHARGE_REF);
    }

    pre_charge_ref_comp = pre_charge_ref_error;
    pre_charge_ref_comp *= -(*pll);

    return pre_charge_ref_comp;
}

static float DC_LinkRegulation(const float* dc_voltage, const float* pll)
{
    float dc_voltage_comp = 0.0;
    float dc_voltage_error = 0.0;
    //static float integral_dc_voltage = 0.0;

    float KI_DC_LIM_MAX = KI_LIMIT(KI_PI_DC);
    float KI_DC_LIM_MIN = -KI_DC_LIM_MAX;

    dc_voltage_error = V_REF - (*dc_voltage);
    integral_dc_voltage += dc_voltage_error;

    //Check if the limits of the integral of the error are accomplished
    if (integral_dc_voltage > KI_DC_LIM_MAX) {
        integral_dc_voltage = KI_DC_LIM_MAX;
    }
    if (integral_dc_voltage < KI_DC_LIM_MIN) {
        integral_dc_voltage = KI_DC_LIM_MIN;
    }

    dc_voltage_comp = KP_PI_DC * dc_voltage_error + KI_PI_DC * TS * integral_dc_voltage;

    dc_voltage_comp *= -(*pll);

    return dc_voltage_comp;
}

static float CompensationVoltageCalc(const int16* grid, const float* pll, const float* pll_unitary, const float* pf, const float* reg, const float * pre_charge)
{
    float true_sinewave = V_PEAK * (*pll_unitary);
    float balance = true_sinewave - (*pll);

    float Vharm = (*pll) - (float)(*grid);

    return Vharm;
    //return (Vharm + (*reg) + balance + (*pre_charge));
    //return (Vharm + (*pf) + (*reg) + balance + (*pre_charge));
}

static float VoltageCompensation(float* VcompRef, const int16* filter)
{
    float KI_VCOMP_LIM_MAX = KI_LIMIT(V_COMP_KI);
    float KI_VCOMP_LIM_MIN = -KI_VCOMP_LIM_MAX;

    float voltage_comp = 0.0, voltage_comp_error = 0.0;
    //static float integral_comp_voltage = 0.0;

    voltage_comp_error = (*VcompRef) - (*filter);
    integral_comp_voltage += voltage_comp_error;

    if (integral_comp_voltage > KI_VCOMP_LIM_MAX) {
        integral_comp_voltage = KI_VCOMP_LIM_MAX;
    }
    if (integral_comp_voltage < KI_VCOMP_LIM_MIN) {
        integral_comp_voltage = KI_VCOMP_LIM_MIN;
    }

    //voltage_comp = KP_COMP * voltage_comp_error + KI_COMP * integral_comp_voltage;
    voltage_comp = KP * voltage_comp_error + KI * /*TS **/ integral_comp_voltage;
    //voltage_comp = V_COMP_KP * voltage_comp_error + V_COMP_KI * integral_comp_voltage;

    return voltage_comp;
}

static unsigned int SoftStart(const int16* dc_link_voltage)
{
    return (*dc_link_voltage < V_REF);
}

static Uint16 variableCheck(float variable, int16 value, float range)
{
    return ((variable * (1 - range)) < value && (variable * (1 + range)) > value);
}

static Uint16 isPLL_Synced(float PLL_amp, Uint16 Vpeak)
{
    return variableCheck(PLL_amp, Vpeak, 0.02);
}

static int16 CheckProtections(void)
{
    if(ADC_Data.Vgrid > VGRID_MAX){
        return CH0_VGRID_OVERVOLTAGE;
    }

//    if(ADC_Data.ILoad > ILOAD_MAX){
//        return CH2_ILOAD_OVERCURRENT;
//    }

    if(ADC_Data.Vdc > VDC_MAX){
        return CH3_VDC_OVERVOLTAGE;
    }

    if(ADC_Data.Vf > VF_MAX){
        return CH4_VF_OVERVOLTAGE;
    }

//    if(ADC_Data.Ilf > ILF_MAX){
//        return CH5_ILF_OVERCURRENT;
//    }

    return ALL_OK;
}

static void Protection(void)
{
    EPwm1Regs.AQCSFRC.bit.CSFA = 1; //Ativa EPWM1A -> S1
    //EPwm1Regs.AQCSFRC.bit.CSFB = 0; //Desativa EPWM1B -> S2

    EPwm2Regs.AQCSFRC.bit.CSFA = 1; //Ativa EPWM2A -> S3
    //EPwm2Regs.AQCSFRC.bit.CSFB = 0; //Desativa EPWM2B -> S4
}

static void ClearIntegralsErrors(void)
{
    integral_comp_voltage = 0.0;
    integral_preLoad_voltage = 0.0;
    integral_dc_voltage = 0.0;
}

static void DAC_Update(int16 DAC_OUT_A, int16 DAC_OUT_B, int16 DAC_OUT_C, int16 DAC_OUT_D, int16 DAC_OUT_E, int16 DAC_OUT_F, int16 DAC_OUT_G, int16 DAC_OUT_H)
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
