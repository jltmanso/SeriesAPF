/*
 * SAPF_Control.h
 *
 *  Created on: 19/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#ifndef SAPF_INC_SAPF_CONTROL_H_
#define SAPF_INC_SAPF_CONTROL_H_

#include <math.h>
#include <stdarg.h>

#include "DSP2833x_Device.h"

//###############################################
//#             General Defines                 #
//###############################################
#define         PI                  3.1415926f
#define         V_RMS               1700.0                       //Digital Systen RMS Voltage
#define         FREQ                50                           //Grid Frequency

#define         FSW                 20000.0f                     //Switching Frequency
#define         FS                  (2 * FSW)                    //Sampling Frequency is double of the PWM Frequency (Unipolar Modulation)
#define         TS                  0.000025f                    //Sampling Period (Ts) = 1 / Fs

#define         ROOT_2              1.414213562                 //sqrt(2)
#define         V_PEAK              (ROOT_2 * V_RMS)            //sqrt(2) * V_RMS = V_PEAK

//###############################################
//#         Phase Locked Loop Defines           #
//###############################################
#define         KP_PI_PLL           0.41f                        //Phase Locked Loop Proporcional Gain
#define         KI_PI_PLL           0.001f                       //Phase Locked Loop Integral Gain
#define         K_AMP_PLL           0.0019f

//###############################################
//#    Passive Filter Compensation Defines      #
//###############################################
#define         LPF                 0.000625f                    //Passive Filter Inductor Value
#define         RPF                 0.5f                         //Passive Filter Resistor Value

//###############################################
//#         DC Link Regulation Defines          #
//###############################################
#define         V_REF               842.0f                       //DC Link Voltage Reference (30v) -> -279 read by the sensor
#define         KP_PI_DC            0.3f                         //DC Link Proporcional Gain
#define         KI_PI_DC            0.0001f                      //DC Link Integral Gain

//###############################################
//#      DeadTimes Compensation Defines         #
//###############################################
#define         DT                  0.000003                    //Deadtime (DT) = 3us = 0.000003s
#define         VD                  (DT * V_REF * FSW)          //Voltage Deviation (DV) = DT * V_REF * FSW

//###############################################
//#   Sliding Average DC Link Voltage Defines   #
//###############################################
#define         BUFFERSIZE          800                         //Size of the buffer to calculate the average DC Link Voltage

//###############################################
//#       Damping Compensation Defines          #
//###############################################
#define         KD                  0.0000065                   //Damping Gain

//###############################################
//#       Pre-Charge Compensation Defines       #
//###############################################
#define         PRE_LOAD_REF         187.0                       //5V sinusoidal wave
#define         PRE_LOAD_KP          0.0008
#define         PRE_LOAD_KI          0.00000009

//###############################################
//#       Voltage Compensation Defines          #
//###############################################
#define         V_COMP_KP           5.8f
#define         V_COMP_KI           6550.0f

//###############################################
//#            DAC Offsets Defines              #
//###############################################
#define         DAC_MAX_VALUE       2048
#define         DAC_OUT_A_OFFSET    (DAC_MAX_VALUE - 30)
#define         DAC_OUT_B_OFFSET    (DAC_MAX_VALUE + 11)
#define         DAC_OUT_C_OFFSET    (DAC_MAX_VALUE + 21)
#define         DAC_OUT_D_OFFSET    (DAC_MAX_VALUE + 4)
#define         DAC_OUT_E_OFFSET    (DAC_MAX_VALUE + 27)
#define         DAC_OUT_F_OFFSET    (DAC_MAX_VALUE + 4)
#define         DAC_OUT_G_OFFSET    (DAC_MAX_VALUE - 4)
#define         DAC_OUT_H_OFFSET    (DAC_MAX_VALUE + 11)

//###############################################
//#            Auxiliary Defines                #
//###############################################
//Grid Voltage Sensor Gain:             4.58
//Load Current Sensor Gain:             45.67
//DC Link Sensor Gain:                  24.98
//Filter Voltage Sensor Gain:           21.1
//Passive Filter Current Sensor Gain:   20.7
#define         RANGE               0.02f                        //4%
#define         PWM_OFFSET          1875
#define         PWM_MAX             1575
#define         PWM_MIN             -1575
//#define         VDC_VG_FATOR        0.183f                      //(4.58 / 24.98) -> slope of the line of both graphs
//#define         VF_VG_FATOR         0.366f//0.218f              //(4.58 / 21)    -> slope of the line of both graphs

//###############################################
//#    Sensors Offsets Equations Defines        #
//###############################################
#define         VGRID_OFFSET         71
#define         RESERV_OFFSET        71
#define         ILOAD_OFFSET         74
#define         VDC_OFFSET           -82
#define         VF_OFFSET            124
#define         ILF_OFFSET           -1
#define         RESERV1_OFFSET        0
#define         ENABLE_OFFSET        -2

//###############################################
//#        Integral MAX Error Defines           #
//###############################################
#define         KI_LIMIT(X)      (X ? (PWM_MAX / (X * TS)) : 0.0)

typedef struct
{
    int Vgrid;                                                  //ADC0: Grid Voltage
    int Reserv;                                                 //ADC1: Reserved Variable space
    int ILoad;                                                  //ADC2: Load Current
    int Vdc;                                                    //ADC3: DC Link Voltage
    int Vf;                                                     //ADC4: SAPF Voltage
    int Ilf;                                                    //ADC5: Passive Filter Current
    int Reserv1;                                                //ADC6: Reserved Variable space
    int Enable;                                                 //ADC7: SAPF Enable
}InputData;

enum BufferType_t{
    EMPTY_BUFFER,                                               //MODE EMPTY: start to fill the empty buffer
    FULL_BUFFER                                                 //MODE FULL: start to update the oldest value with the newest
};

//###############################################
//#     Private Translation Unit Variables      #
//###############################################
static int16 DC_Voltage[BUFFERSIZE];
static int16 GridVoltage[BUFFERSIZE >> 0x01];

extern volatile Uint16 pre_charge_flag;
//###############################################
//#             User Functions                  #
//###############################################
void SAPFControl(void);
void ADC1_StartConversionSignal(void);
void ADC1_WakeUpSignal(void);
void ADC1_SaveData(void);
void PreChargeDC_Link(void);
Uint16 isSAPF_Enabled(void);
void ClearIntegralsErrors(void);

//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static float PhaseLockedLoop(const int16* grid);
static int16 PreChargeVoltage_Compensation(const int16* pre_load_ref, const int16* Vfilter);
static float PassiveFilterVoltage(const int16* pf_current);
static Uint16 VariableCheck(float* variable, int16 value);
static unsigned int isBufferFull(unsigned int* index);
static float DC_LinkAverageVoltage(int16* DC_Voltage, const int16* dc_voltage);
static Uint32 getDC_LinkValues(int16* DC_Voltage, const int16* dc_voltage, enum BufferType_t mode);
static Uint16 getFullDC_LinkValues(int16* DC_Voltage, const int16* dc_voltage);
static Uint32 DC_LinkContiniousAverageVoltage(int16* DC_Voltage, const int16*dc_voltage, Uint32* sum);

static float VoltageCompensation(float* VcompRef, const int16* filter);
static float DC_LinkCompensation(const float* dc_voltage, float* pll);
static float CompensationVoltageCalc(const int16* grid, const float* pll, const float* pf, const float* reg);
static unsigned int MaxValue(const int16* vector);
static unsigned getVpeak(const int16* signal);
static float DC_LinkRegulation(Uint16 disturbance, const float* average);

static unsigned int SoftStart(const int16* dc_voltage);

static int16 assert_PWM(int16 pwm);
static int16 setPWM(int16 pwm);

static void DAC_Update(int16 DAC_OUT_A, int16 DAC_OUT_B, int16 DAC_OUT_C, int16 DAC_OUT_D, int16 DAC_OUT_E, int16 DAC_OUT_F, int16 DAC_OUT_G, int16 DAC_OUT_H);

#endif /* SAPF_INC_SAPF_CONTROL_H_ */
