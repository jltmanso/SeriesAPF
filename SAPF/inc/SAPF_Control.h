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
#include "DSP2833x_Device.h"

//###############################################
//#             General Defines                 #
//###############################################
#define         PI                  3.1415926f
#define         V_RMS               962.0f                       //Digital Systen RMS Voltage
#define         FREQ                50                           //Grid Frequency

#define         FSW                 20000.0f                     //Switching Frequency
#define         FS                  (2 * FSW)                    //Sampling Frequency is double of the PWM Frequency (Unipolar Modulation)
#define         TS                  0.000025f                    //Sampling Period (Ts) = 1 / Fs

#define         ROOT_2              1.414213562                  //sqrt(2)
#define         V_PEAK              (ROOT_2 * V_RMS)             //sqrt(2) * V_RMS = V_PEAK

//###############################################
//#         Phase Locked Loop Defines           #
//###############################################
#define         KP_PI_PLL           0.09f                        //Phase Locked Loop Proporcional Gain
#define         KI_PI_PLL           30.0f                        //Phase Locked Loop Integral Gain
#define         KP_AMP_PLL          40.0f

//###############################################
//#    Passive Filter Compensation Defines      #
//###############################################
#define         LPF                 0.000823f                    //Passive Filter Inductor Value
#define         RPF                 0.253f                       //Passive Filter Resistor Value

//###############################################
//#         DC Link Regulation Defines          #
//###############################################
#define         V_REF               600.0f                       //DC Link Voltage Reference (30V)
#define         KP_PI_DC            16.8f                        //DC Link Proporcional Gain
#define         KI_PI_DC            40.0f                        //DC Link Integral Gain
//###############################################
//#   Sliding Average DC Link Voltage Defines   #
//###############################################
#define         BUFFERSIZE          800                         //Size of the buffer to calculate the average DC Link Voltage
#define         HALF_BUFFERSIZE     (BUFFERSIZE >> 0x01)

//###############################################
//#       Pre-Charge Compensation Defines       #
//###############################################
#define         PRE_CHARGE_REF      213.0f                      //5V sinusoidal wave
#define         PRE_CHARGE_KP       2.5f
#define         PRE_CHARGE_KI       0.1f

//###############################################
//#       Voltage Compensation Defines          #
//###############################################
#define         V_COMP_KP           4.2f
#define         V_COMP_KI           0.05f

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
#define         PWM_OFFSET          1875
#define         PWM_MAX             1575
#define         PWM_MIN             -1575

//###############################################
//#    Sensors Offsets Equations Defines        #
//###############################################
#define         VGRID_OFFSET        45//+32
#define         RESERV_OFFSET       0
#define         ILOAD_OFFSET        71
#define         VDC_OFFSET          77
#define         VF_OFFSET           77//47//83
#define         ILF_OFFSET          -1
#define         RESERV1_OFFSET      0
#define         ENABLE_OFFSET       -5

//###############################################
//#    Sensors Conversion Voltage Defines       #
//###############################################
#define         VGRID_GAIN          4.58f
#define         RESERV_GAIN         0
#define         ILOAD_GAIN          45.67f
#define         VDC_GAIN            25
#define         VF_GAIN             42.4
#define         ILF_GAIN            20.7f
#define         RESERV1_GAIN        0
#define         ENABLE_GAIN         0

//###############################################
//#    Sensors Conversion Voltage Defines       #
//###############################################
#define         VGRID_MAX           1500.0f         //Error at 30% up Vgrid Peak
#define         ILOAD_MAX           228.35f         //Error at 5A
#define         VDC_MAX             1530.0f         //Error at 50V
#define         VF_MAX              1484.0f         //Error at 35V
#define         ILF_MAX             165.6f          //Error at 7A
//###############################################
//#        Integral MAX Error Defines           #
//###############################################
#define         KI_LIMIT(X)      (X ? (PWM_MAX / X) : 0.0)

typedef struct
{
    int16 Vgrid;                                                  //ADC0: Grid Voltage
    int16 Reserv;                                                 //ADC1: Reserved Variable space
    int16 ILoad;                                                  //ADC2: Load Current
    int16 Vdc;                                                    //ADC3: DC Link Voltage
    int16 Vf;                                                     //ADC4: SAPF Voltage
    int16 Ilf;                                                    //ADC5: Passive Filter Current
    int16 Reserv1;                                                //ADC6: Reserved Variable space
    int16 Enable;                                                 //ADC7: SAPF Enable
}inputData_t;

typedef struct
{
    Uint16 index;
    Uint32 sum;
    Uint16 pos;
}sharedData_t;

typedef struct
{
    float lim_min;
    float lim_max;
}KI_Limits_t;

enum BufferType_t{
    EMPTY_BUFFER,                                               //MODE EMPTY: start to fill the empty buffer
    FULL_BUFFER                                                 //MODE FULL: start to update the oldest value with the newest
};

enum SensorError_t{
    CH0_VGRID_OVERVOLTAGE,
    CH1_NOT_USED,
    CH2_ILOAD_OVERCURRENT,
    CH3_VDC_OVERVOLTAGE,
    CH4_VF_OVERVOLTAGE,
    CH5_ILF_OVERCURRENT,
    ALL_OK
};

//###############################################
//#     Private Translation Unit Variables      #
//###############################################
static int16 DC_Voltage[BUFFERSIZE];
static int16 Grid_Voltage[BUFFERSIZE >> 0x01];

extern volatile Uint16 pre_charge_flag;
extern volatile int16 enable;
extern volatile Uint16 error;
extern volatile Uint16 detectedError;
//###############################################
//#             User Functions                  #
//###############################################
void SAPFControl(void);
void ADC1_StartConversionSignal(void);
void ADC1_WakeUpSignal(void);
void ADC1_SaveData(void);
Uint16 isSAPF_Enabled(void);
void ClearStructVariables(void);
void Calculation_KI_Max(void);
//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static float PhaseLockedLoop(const int16* grid);
static float PreChargeVoltage_Compensation(const float* pre_load_ref, const int16* Vfilter);
static float PassiveFilterVoltage(const int16* pf_current);
static Uint16 VariableCheck(float* variable, int16 value);
static void ClearIntegralsErrors(void);
static int16 CheckProtections(void);
//              AVERAGE VOLTAGE                 #
static float AverageVoltage(sharedData_t* data, int16* buffer, const int16* value);
static Uint32 getValues(sharedData_t* data, int16* buffer, const int16* value, enum BufferType_t mode);
static Uint16 getFullValues(sharedData_t* data, int16* buffer, const int16* value);
static Uint32 SlidingWindow(sharedData_t* data, int16* buffer, const int16* value, Uint32* sum);
static Uint16 isBufferFull(Uint16* index);
static Uint16 isHalfBufferFull(unsigned int* index);

static float PeakAverageVoltage(sharedData_t* data, int16* buffer, const int16* value);
static Uint16 peakVoltage(const int16* value);

static float PreChargeRefCalculation(const int16* dc_voltage, const float* pll);
static float VoltageCompensation(float* VcompRef, const int16* filter);
static float DC_LinkRegulation(const float* dc_voltage, const float* pll);
static float CompensationVoltageCalc(const int16* grid, const float* pll, const float* pll_unitary, const float* pf, const float* reg, const float* pre_charge);
static unsigned int MaxValue(const int16* vector);
static unsigned getVpeak(const int16* signal);

static unsigned int SoftStart(const int16* dc_voltage);

static int16 assert_PWM(int16 pwm);
static int16 setPWM(float Vcompensation);

static Uint16 variableCheck(float variable, int16 value, float range);
static Uint16 isPLL_Synced(float PLL_amp, Uint16 Vpeak);
static void Protection(void);

static void DAC_Update(int16 DAC_OUT_A, int16 DAC_OUT_B, int16 DAC_OUT_C, int16 DAC_OUT_D, int16 DAC_OUT_E, int16 DAC_OUT_F, int16 DAC_OUT_G, int16 DAC_OUT_H);

#endif /* SAPF_INC_SAPF_CONTROL_H_ */
