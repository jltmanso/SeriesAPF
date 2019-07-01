/*
 * SAPF_GPIO.c
 *
 *  Created on: 21/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#include "../inc/SAPF_GPIO.h"

void ConfigGPIO(void)
{
    //EALLOW;

    ConfigCable();
    ConfigJ8Cable_IO();
    ConfigJ3Cable_PWM_1();
    ConfigJ7Cable_PWM_2();
    ConfigJ11Cable_ADC();
    ConfigJ4Cable_DAC_SPI();
    ConfigJ2Cable_RS232();

    //EDIS;
}

static void ConfigCable(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPCPUD.bit.GPIO84 = 1;
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = 1;
    GpioCtrlRegs.GPCPUD.bit.GPIO86 = 1;
    GpioCtrlRegs.GPCPUD.bit.GPIO87 = 1;

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    GpioDataRegs.GPCSET.bit.GPIO84 = 1;
    GpioDataRegs.GPCSET.bit.GPIO85 = 1;
    GpioDataRegs.GPCSET.bit.GPIO86 = 1;
    GpioDataRegs.GPCSET.bit.GPIO87 = 1;

    EDIS;
}

static void ConfigJ8Cable_IO(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0;

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    //GpioDataRegs.GPCSET.bit.GPIO84 = 1;
    //GpioDataRegs.GPCSET.bit.GPIO85 = 1;
    //GpioDataRegs.GPCSET.bit.GPIO86 = 1;
    //GpioDataRegs.GPCSET.bit.GPIO87 = 1;

    GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;

    EDIS;
}

static void ConfigJ3Cable_PWM_1(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;      //ePWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;      //ePWM1B
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;      //ePWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;      //ePWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;      //ePWM3A
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;      //ePWM3B

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    //GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    //GpioDataRegs.GPASET.bit.GPIO0 = 1;
    //GpioDataRegs.GPASET.bit.GPIO1 = 1;
    //GpioDataRegs.GPASET.bit.GPIO2 = 1;
    //GpioDataRegs.GPASET.bit.GPIO3 = 1;
    //GpioDataRegs.GPASET.bit.GPIO4 = 1;
    //GpioDataRegs.GPASET.bit.GPIO5 = 1;

    EDIS;
}

static void ConfigJ7Cable_PWM_2(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPAPUD.bit.GPIO6  = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO7  = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO8  = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO9  = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    GpioCtrlRegs.GPADIR.bit.GPIO6  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO7  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO8  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO9  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    //GpioDataRegs.GPASET.bit.GPIO6  = 1;
    //GpioDataRegs.GPASET.bit.GPIO7  = 1;
    //GpioDataRegs.GPASET.bit.GPIO8  = 1;
    //GpioDataRegs.GPASET.bit.GPIO9  = 1;
    //GpioDataRegs.GPASET.bit.GPIO10 = 1;
    //GpioDataRegs.GPASET.bit.GPIO11 = 1;

    EDIS;
}

static void ConfigJ11Cable_ADC(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1; //33 DB0-DCIN_D         (GPIO12)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1; //32 DB1_DCIN_C         (GPIO13)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; //31 DB2_DCIN_B         (GPIO14)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; //30 DB3_DCIN_A         (GPIO15)
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; //29 DB4                (GP�O20)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; //28 DB5-SEL_CD         (GPIO21)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; //27 DB6-SEL_B          (GPIO22)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1; //26 DB7                (GPIO23)
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 1; //23 DB8-DCEN           (GPIO30)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1; //22 DB9-SDI            (GPIO31)
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1; //21 DB10-SLCK          (GPIO32)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 1; //20 DB11-/REFBUFEN     (GPIO33)
    //GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0; //19 DB12-SDO_A       (GPIO24)
    //GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0; //18 DB13-SDO_B       (GPIO25)
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1; //37-40 CONVST_A_B_C_D  (GPIO63)
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0; //12 /RD                (GPIO34)
    //GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0; //35 BUSY-INT         (GPIO62)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1; //10 RESET              (GPIO26)
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0; //13 /CS-/FS (MASTER)   (GPIO49)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0; //23 NC                 (GPIO27)
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0; //13 /CS-/FS (SLAVE)    (GPIO48)

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; //33 DB0-DCIN_D         (GPIO12)
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0; //32 DB1_DCIN_C         (GPIO13)
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0; //31 DB2_DCIN_B         (GPIO14)
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0; //30 DB3_DCIN_A         (GPIO15)
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0; //29 DB4                (GP�O20)
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0; //28 DB5-SEL_CD         (GPIO21)
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0; //27 DB6-SEL_B          (GPIO22)
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0; //26 DB7                (GPIO23)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0; //23 DB8-DCEN           (GPIO30)
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0; //22 DB9-SDI            (GPIO31)
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0; //21 DB10-SLCK          (GPIO32)
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0; //20 DB11-/REFBUFEN     (GPIO33)
    //GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0; //19 DB12-SDO_A         (GPIO24)
    //GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0; //18 DB13-SDO_B         (GPIO25)
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0; //37-40 CONVST_A_B_C_D  (GPIO63)
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; //12 /RD                (GPIO34)
    //GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0; //35 BUSY-INT           (GPIO62)
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0; //10 RESET              (GPIO26)
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; //23 NC                 (GPIO27)
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0; //13 /CS-/FS (MASTER)   (GPIO49)
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0; //13 /CS-/FS (SLAVE)    (GPIO48)

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 0; //33 DB0-DCIN_D         (GPIO12)
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 0; //32 DB1_DCIN_C         (GPIO13)
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0; //31 DB2_DCIN_B         (GPIO14)
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 0; //30 DB3_DCIN_A         (GPIO15)
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0; //29 DB4                (GP�O20)
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0; //28 DB5-SEL_CD         (GPIO21)
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0; //27 DB6-SEL_B          (GPIO22)
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0; //26 DB7                (GPIO23)
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0; //23 DB8-DCEN           (GPIO30)
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 0; //22 DB9-SDI            (GPIO31)
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0; //21 DB10-SLCK          (GPIO32)
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0; //20 DB11-/REFBUFEN     (GPIO33)
    //GpioCtrlRegs.GPADIR.bit.GPIO24 = 0; //19 DB12-SDO_A       (GPIO24)
    //GpioCtrlRegs.GPADIR.bit.GPIO25 = 0; //18 DB13-SDO_B       (GPIO25)
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1; //37-40 CONVST_A_B_C_D  (GPIO63)
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; //12 /RD                (GPIO34)
    //GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0; //35 BUSY-INT         (GPIO62)
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1; //10 RESET              (GPIO26)
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1; //23 NC                 (GPIO27)
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1; //13 /CS-/FS (MASTER)   (GPIO49)
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1; //13 /CS-/FS (SLAVE)    (GPIO48)

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    GpioDataRegs.GPASET.bit.GPIO27 = 1; //23 NC                 (GPIO27)
    GpioDataRegs.GPBSET.bit.GPIO34 = 1; //12 /RD                (GPIO34)
    GpioDataRegs.GPBSET.bit.GPIO63 = 1; //37-40 CONVST_A_B_C_D  (GPIO63)

    EDIS;
}

static void ConfigJ4Cable_DAC_SPI(void)
{
    EALLOW;

    //Enable Pull-Up
    //Disable =  1
    //Enable  =  0
    //GPAPUD -> (GPIO0 to 31)
    //GPBPUD -> (GPIO32 to 63)
    //GPCPUD -> (GPIO64 to 95)
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0; // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0; // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0; // Enable pull-up on GPIO19 (SPISTEA)

    //Select Respective Mux
    //GPAMUX1 -> (GPIO0 to 15)
    //GPAMUX2 -> (GPIO16 to 31)
    //GPBMUX1 -> (GPIO32 to 47)
    //GPBMUX2 -> (GPIO48 to 63)
    //GPCMUX1 -> (GPIO64 to 79)
    //GPCMUX2 -> (GPIO80 to 95)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

    //Select Direction
    //Input  =  0
    //Output =  1
    //GPADIR -> (GPIO0 to 31)
    //GPBDIR -> (GPIO32 to 63)
    //GPCDIR -> (GPIO64 to 95)
    //GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;
    //GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;

    //Enable GPIO
    //Disable = 0;
    //Enable  = 1;
    //GPASET -> (GPIO0 to 31)
    //GPBSET -> (GPIO32 to 63)
    //GPCSET -> (GPIO64 to 95)
    //GpioCtrlRegs.GPASET.bit.GPIO16 = 1;
    //GpioCtrlRegs.GPASET.bit.GPIO18 = 1;
    //GpioCtrlRegs.GPASET.bit.GPIO19 = 1;

    //Sets de Qualification to the GPIO
    //GPIO_Qual_Sync     = 0            //!< Denotes input will be synchronized to SYSCLK
    //GPIO_Qual_Sample_3 = 1            //!< Denotes input is qualified with 3 samples
    //GPIO_Qual_Sample_6 = 2            //!< Denotes input is qualified with 6 samples
    //GPIO_Qual_ASync    = 3            //!< Denotes input is asynchronous
    //
    //GPAQSEL1 -> (GPIO0 to 15)
    //GPAQSEL2 -> (GPIO16 to 31)
    //GPBQSEL1 -> (GPIO32 to 47)
    //GPBQSEL2 -> (GPIO48 to 64)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;

    EDIS;
}

static void ConfigJ2Cable_RS232(void)
{
    EALLOW;

    //Enable internal pull-up for the selected pins
    //Pull-ups can be enabled or disabled disabled by the user.
    //This will enable the pullups for the specified pins.
    //Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    //Enable pull-up for GPIO9  (SCITXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   //Enable pull-up for GPIO14 (SCITXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;     //Enable pull-up for GPIO18 (SCITXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   //Enable pull-up for GPIO22 (SCITXDB)

    //GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   //Enable pull-up for GPIO11 (SCIRXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   //Enable pull-up for GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;     //Enable pull-up for GPIO19 (SCIRXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   //Enable pull-up for GPIO23 (SCIRXDB)

    //
    //Set qualification for selected pins to asynch only
    //This will select asynch (no qualification) for the selected pins.
    //Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3; //Asynch input GPIO11 (SCIRXDB)
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3; // Asynch input GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;   // Asynch input GPIO19 (SCIRXDB)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // Asynch input GPIO23 (SCIRXDB)

    //
    //Configure SCI-B pins using GPIO regs
    //This specifies which of the possible GPIO pins will be SCI functional
    //pins.
    //Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;   //Configure GPIO9 to SCITXDB
    //GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;  //Configure GPIO14 to SCITXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 2;    //Configure GPIO18 to SCITXDB
    //GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;  //Configure GPIO22 to SCITXDB

    //GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;  //Configure GPIO11 for SCIRXDB
    //GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;  //Configure GPIO15 for SCIRXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 2;    //Configure GPIO19 for SCIRXDB
    //GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;  //Configure GPIO23 for SCIRXDB

    EDIS;
}
