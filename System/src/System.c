/*
 * System.c
 *
 *  Created on: 21/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#include "../inc/System.h"

//###############################################
//#         Interruption Functions              #
//###############################################
__interrupt void CpuTimer0Interrupt_isr(void)
{
    timer0_flag = 1;
    //Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ExternalInterrupt3_isr(void)
{
    xInt3_flag = 1;

    GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1;

    //Acknowledge this interrupt to get more from group 12
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

__interrupt void SciaRxFifoInterrupt_isr(void)
{
    rx_data = ScibRx();

    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFFOVF = 1;       //Clear Overflow Flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;    //Clear Interrupt Flag

    //Acknowledge this interrupt to get more from group 9
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}

__interrupt void SciaTxFifoInterrupt_isr(void)
{
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;    //Clear SCI Interrupt Flag
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}

//###############################################
//#             User Functions                  #
//###############################################
void ConfigSystem(void)
{
    //Step 1. Initialize System Control:
    //PLL, WatchDog, enable Peripheral Clocks
    //This example function is found in the DSP2833x_SysCtrl.c file.
    InitSysCtrl();

    //Step 2. Initialize GPIO
    ConfigGPIO();

    //Step 3. Clear all interrupts and initialize PIE vector table:
    //Disable CPU interrupts
    DINT;

    //Initialize PIE control registers to their default state.
    //The default state is all PIE interrupts disabled and flags
    //are cleared.
    //This function is found in the DSP2833x_PieCtrl.c file.
    InitPieCtrl();

    //Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    //Initialize the PIE vector table with pointers to the shell Interrupt
    //Service Routines (ISR).
    //This will populate the entire table, even if the interrupt
    //is not used in this example.  This is useful for debug purposes.
    //The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    //This function is found in DSP2833x_PieVect.c.
    InitPieVectTable();

    EALLOW; //This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &CpuTimer0Interrupt_isr;       //Interrupcao Timer0
    PieVectTable.XINT3 = &ExternalInterrupt3_isr;       //Interrupcao Externa ADC1 (ADS8528)
    PieVectTable.SCIRXINTA = &SciaRxFifoInterrupt_isr;   //SCIB RX Interrupt Handler
    PieVectTable.SCITXINTA = &SciaTxFifoInterrupt_isr;   //SCIB TX Interrupt Handler
    EDIS;   //This is needed to disable write to EALLOW protected registers

    //Enable XINT3 in the PIE: Group 12 interrupt 1
    //Enable int1 which is connected to WAKEINT:
    PieCtrlRegs.PIECTRL.bit.ENPIE   = 1;    //Enable the PIE block
    PieCtrlRegs.PIEIER12.bit.INTx1  = 1;    //Enable PIE Group 12 INT1 (Xint3)
    PieCtrlRegs.PIEIER1.bit.INTx7   = 1;    //Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx3   = 1;    //Enable SCI receive interrupt in PIE group 9
    PieCtrlRegs.PIEIER9.bit.INTx4   = 1;    //Enable SCI transmission interrupt in PIE group 9

    IER |= M_INT1;  //Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    IER |= M_INT12; //Enable CPU int12
    IER |= M_INT9;  //Enable SCI interrupt

    EINT;   //Enable Global Interrupt INTM
    ERTM;   //Enable Global Realtime Interrupt DBGM

    ConfigExtInterrupt();
    ConfigTimer0();
    ConfigSPI_DAC();
    ConfigScia(9600);
    ConfigPWM_1();
    ConfigPWM_2();
}

void SciaTx_SendMsg(unsigned char* msg)
{
    while(*msg++)
        ScibTx(msg);
}

//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static void ConfigExtInterrupt(void)
{
    EALLOW;

    GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 62;  //Xint3 is GPIO62 -> 35 BUSY-INT (GPIO62)

    EDIS;

    //Specify the Interrupt Polarity
    //Positive/Negative Edge  = 0
    //Negative/Positive Edge  = 1
    //Both Edges              = 3
    XIntruptRegs.XINT3CR.bit.POLARITY = 0;

    //Enable External Interrupt 3
    XIntruptRegs.XINT3CR.bit.ENABLE = 1;
}

static void ConfigTimer0(void)
{
    //CPU Timer 0
    //Initialize address pointers to respective timer registers:
    CpuTimer0.RegsAddr = &CpuTimer0Regs;

    //Initialize timer period to maximum:
    //CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;

    //Initialize timer period to maximum:
    //CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;

    CpuTimer0Regs.PRD.all  = 0xEA6; //150Mhz/f HEX
                                    //8F0D180   1s
                                    //47868C0   0.5s
                                    //5DC       10u
                                    //EA6       25u (40 kHz)
                                    //0x208D    50Hz 360 pontos
                                    //0x124F    32kHz
                                    //0x1D4C    20kHz

    //Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all = 0;

    //Make sure timer is stopped:
    CpuTimer0Regs.TCR.bit.TSS = 1; // 1 = Stop timer, 0 = Start/Restart Timer

    //Reload all counter register with period value:
    CpuTimer0Regs.TCR.bit.TRB = 1; // 1 = reload timer

    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0;     // Stop after the next decrement

    //Enable Timer0 Interrupt
    CpuTimer0Regs.TCR.bit.TIE=1; // 0 = Disable/ 1 = Enable Timer Interrupt

    //Reset interrupt counters:
    CpuTimer0.InterruptCount = 0;
}

static void ConfigSPI_DAC(void)
{
    // Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x204F;
    SpiaRegs.SPIFFCT.all=0x0002;

    SpiaRegs.SPICCR.all =0x0007;        //Reset on, rising edge, 16-bit char bits, no loopback
    SpiaRegs.SPICTL.all =0x0006;        //Enable master mode, normal phase,
                                        //Enable talk, and SPI int disabled.
    SpiaRegs.SPIBRR =0x0003;            //SpiaRegs.SPIBRR =0x007F;
    SpiaRegs.SPICCR.all =0x009F;        //Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;       //Set so breakpoints don't disturb xmission
}

static void ConfigScia(Uint16 baudrate)
{

    Uint16 baudrate_clk = LSPCLK /(baudrate * 8) - 1;

    SciaRegs.SCICCR.all = 0x0007;   //1 stop bit
                                    //No loopback
                                    //No parity
                                    //8 char bits
                                    //Async mode
                                    //Idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  //Enable TX
                                    //Enable RX
                                    //Enable internal SCICLK,
                                    //Disable RX ERR, SLEEP, TXWAKE

    SciaRegs.SCICTL2.bit.TXINTENA   = 1;    //TX Interrupt Enable
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;    //RX Interrupt Enable

    // Disable RX ERR, SLEEP, TXWAKE
    //SciaRegs.SCICTL2.bit.all = 0x0003;

    //SciaRegs.SCIHBAUD = 0x0001;   // 9600 baud @LSPCLK = 37.5MHz.
    //SciaRegs.SCILBAUD = 0x00E7;

    SciaRegs.SCIHBAUD = baudrate_clk & 0xFF;
    SciaRegs.SCILBAUD = (baudrate_clk >> 0x08) & 0xFF;

    SciaRegs.SCICCR.bit.LOOPBKENA = 0;      //LoopBack Enable

    //SciaRegs.SCIFFTX.all = 0xE040;
    //SciaRegs.SCIFFRX.all = 0x0021;
    SciaRegs.SCIFFTX.all=0xC028;
    SciaRegs.SCIFFRX.all=0x0028;
    SciaRegs.SCIFFCT.all = 0x00;

    SciaRegs.SCICTL1.all = 0x0023;   // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

static void ConfigPWM_1(void)
{
    //######################################  PWM 1 ######################################
    //TBPRD = fDSP/(fPWMxCLKDIVxHSPCLKDIV)
    EPwm1Regs.TBPRD                 = PRD_VALUE;
    EPwm1Regs.TBPHS.half.TBPHS      = 0;                // Set Phase register to zero
    EPwm1Regs.TBCTR                 = 0;                // Clear Counter

    // Setup Counter Mode
    EPwm1Regs.TBCTL.bit.CTRMODE     = TB_COUNT_UPDOWN;  // Symmetrical mode
    EPwm1Regs.TBCTL.bit.HSPCLKDIV   = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV      = TB_DIV1;
    EPwm1Regs.TBCTL.bit.PHSEN       = TB_DISABLE;       // Master module
    EPwm1Regs.TBCTL.bit.PRDLD       = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL    = TB_CTR_ZERO;      // Sync down-stream module

    // Setup Shadowing Registers
    EPwm1Regs.CMPCTL.bit.SHDWAMODE  = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE  = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE  = CC_CTR_ZERO;      // load on CTR = Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE  = CC_CTR_ZERO;      // load on CTR = Zero

    // Set Actions
    EPwm1Regs.AQCTLA.bit.CAU        = AQ_CLEAR;//AQ_SET;           // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD        = AQ_SET;//AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBU        = AQ_SET;//AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD        = AQ_CLEAR;//AQ_SET;

    EPwm1Regs.DBCTL.bit.OUT_MODE    = DB_FULL_ENABLE;   // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL      = DB_ACTV_HIC;      // Active Hi complementary
    //EPwm1Regs.DBFED                 = 150;              // FED = 50 TBCLKs (1us)
    //EPwm1Regs.DBRED                 = 150;              // RED = 50 TBCLKs (1us)

//    EPwm1Regs.CMPA.half.CMPA        = 1875;             //50% Duty Cycle
//    EPwm1Regs.CMPB                  = 1875;

    //######################################  PWM 2 ######################################
    //TBPRD = fDSP/(2xfPWMxCLKDIVxHSPCLKDIV)
    EPwm2Regs.TBPRD                 = PRD_VALUE;
    EPwm2Regs.TBPHS.half.TBPHS      = 0;                // Set Phase register to zero
    EPwm2Regs.TBCTR                 = 0;                //Clear Counter

    // Setup Counter Mode
    EPwm2Regs.TBCTL.bit.CTRMODE     = TB_COUNT_UPDOWN;  // Symmetrical mode
    EPwm2Regs.TBCTL.bit.HSPCLKDIV   = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV      = TB_DIV1;
    EPwm2Regs.TBCTL.bit.PHSEN       = TB_ENABLE;       // Master module
    EPwm2Regs.TBCTL.bit.PRDLD       = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL    = TB_SYNC_IN;      // Sync down-stream module

    // Setup Shadowing Registers
    EPwm2Regs.CMPCTL.bit.SHDWAMODE  = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE  = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE  = CC_CTR_ZERO;      // load on CTR = Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE  = CC_CTR_ZERO;      // load on CTR = Zero

    // Set Actions
    EPwm2Regs.AQCTLA.bit.CAU        = AQ_SET;           // set actions for EPWM1A
    EPwm2Regs.AQCTLA.bit.CAD        = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU        = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBD        = AQ_SET;

    EPwm2Regs.DBCTL.bit.OUT_MODE    = DB_FULL_ENABLE;   // enable Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL      = DB_ACTV_HIC;      // Active Hi complementary
    //EPwm2Regs.DBFED                 = 150;              // FED = 50 TBCLKs
    //EPwm2Regs.DBRED                 = 150;              // RED = 50 TBCLKs

//    EPwm2Regs.CMPA.half.CMPA        = 1875;             //50% Duty Cycle
//    EPwm2Regs.CMPB                  = 1875;             //50% Duty Cycle

//    //######################################  PWM 3 ######################################
//
//    EPwm3Regs.TBCTL.bit.CLKDIV =  0;            //CLKDIV = 1
//    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;          //HSPCLKDIV = 2
//    EPwm3Regs.TBCTL.bit.CTRMODE = 2;            //Up - down mode
//
//    EPwm3Regs.AQCTLA.all = 0x0060;              //Set ePWM2A on CMPA up
//                                                //Clear ePWM2A on CMPA down
//    //TBPRD = fDSP/(2xfPWMxCLKDIVxHSPCLKDIV)
//    EPwm3Regs.TBPRD = 1875;                //1KHz - PWM signal
//    //EPwm3Regs.CMPA.half.CMPA  = 18750;        //100% --> TBPRD
//
//    //TRising Edge Delay = TTBCLKTBCLKx x DBRED, TBCLK equals to 13.33334 ns  with CLKDIV set to 1 and HSPCLKDIV set to 2
//    EPwm3Regs.DBRED = 225;                      //x microseconds delay ->225 3us
//    EPwm3Regs.DBFED = 225;                      //For rising and falling edge
//    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;           //ePWM3A = RED
//    EPwm3Regs.DBCTL.bit.POLSEL = 2;             //S3=1 inverted signal at ePWM3B
//    EPwm3Regs.DBCTL.bit.IN_MODE = 0;            //ePWM3A = source for RED & FED
}

static void ConfigPWM_2(void)
{
    //######################################  PWM 1 ######################################

    EPwm4Regs.TBCTL.bit.CLKDIV =  0;            //CLKDIV = 1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 1;          //HSPCLKDIV = 2
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;            //Up - down mode

    EPwm4Regs.AQCTLA.all = 0x0060;              //Set ePWM4A on CMPA up
                                                //Clear ePWM4A on CMPA down
    //TBPRD = fDSP/(2xfPWMxCLKDIVxHSPCLKDIV)
    EPwm4Regs.TBPRD = 1875;                //1KHz - PWM signal
    //EPwm4Regs.CMPA.half.CMPA  = 2000;         //100% --> TBPRD

    //TRising Edge Delay = TTBCLKTBCLKx x DBRED, TBCLK equals to 13.33334 ns  with CLKDIV set to 1 and HSPCLKDIV set to 2
    EPwm4Regs.DBRED = 225;                      //x microseconds delay ->225 3us
    EPwm4Regs.DBFED = 225;                      //For rising and falling edge
    EPwm4Regs.DBCTL.bit.OUT_MODE = 3;           //ePWM4A = RED
    EPwm4Regs.DBCTL.bit.POLSEL = 2;             //S3=1 inverted signal at ePWM4B
    EPwm4Regs.DBCTL.bit.IN_MODE = 0;            //ePWM4A = source for RED & FED

    //######################################  PWM 2 ######################################

    EPwm5Regs.TBCTL.bit.CLKDIV =  0;            //CLKDIV = 1
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 1;          //HSPCLKDIV = 2
    EPwm5Regs.TBCTL.bit.CTRMODE = 2;            //Up - down mode

    EPwm5Regs.AQCTLA.all = 0x0060;              //Set ePWM5A on CMPA up
                                                //Clear ePWM5A on CMPA down
    //TBPRD = fDSP/(2xfPWMxCLKDIVxHSPCLKDIV)
    EPwm5Regs.TBPRD = 1875;                //1KHz - PWM signal
    //EPwm5Regs.CMPA.half.CMPA  = 18750;        //100% --> TBPRD

    //TRising Edge Delay = TTBCLKTBCLKx x DBRED, TBCLK equals to 13.33334 ns  with CLKDIV set to 1 and HSPCLKDIV set to 2
    EPwm5Regs.DBRED = 225;                      //x microseconds delay ->225 3us
    EPwm5Regs.DBFED = 225;                      //For rising and falling edge
    EPwm5Regs.DBCTL.bit.OUT_MODE = 3;           //ePWM5A = RED
    EPwm5Regs.DBCTL.bit.POLSEL = 2;             //S3=1 inverted signal at ePWM5B
    EPwm5Regs.DBCTL.bit.IN_MODE = 0;            //ePWM5A = source for RED & FED

    //######################################  PWM 3 ######################################

    EPwm6Regs.TBCTL.bit.CLKDIV =  0;            //CLKDIV = 1
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 1;          //HSPCLKDIV = 2
    EPwm6Regs.TBCTL.bit.CTRMODE = 2;            //Up - down mode

    EPwm6Regs.AQCTLA.all = 0x0060;              //Set ePWM6A on CMPA up
                                                //Clear ePWM6A on CMPA down
    //TBPRD = fDSP/(2xfPWMxCLKDIVxHSPCLKDIV)
    EPwm6Regs.TBPRD = 1875;                //1KHz - PWM signal
    //EPwm6Regs.CMPA.half.CMPA  = 18750;        //100% --> TBPRD

    //TRising Edge Delay = TTBCLKTBCLKx x DBRED, TBCLK equals to 13.33334 ns  with CLKDIV set to 1 and HSPCLKDIV set to 2
    EPwm6Regs.DBRED = 225;                      //x microseconds delay ->225 3us
    EPwm6Regs.DBFED = 225;                      //For rising and falling edge
    EPwm6Regs.DBCTL.bit.OUT_MODE = 3;           //ePWM6A = RED
    EPwm6Regs.DBCTL.bit.POLSEL = 2;             //S3=1 inverted signal at ePWM6B
    EPwm6Regs.DBCTL.bit.IN_MODE = 0;            //ePWM6A = source for RED & FED
}

static unsigned char ScibRx(void)
{
    while(ScibRegs.SCIFFRX.bit.RXFFST != 1);
    return ScibRegs.SCIRXBUF.all;
}

static void ScibTx(unsigned char* character)
{
    while(ScibRegs.SCIFFTX.bit.TXFFST);
    ScibRegs.SCITXBUF = *character;
}
