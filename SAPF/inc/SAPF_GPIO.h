/*
 * SAPF_GPIO.h
 *
 *  Created on: 21/03/2018
 *      Author: Joao Luis Torre Manso
 *      University: Minho
 */

#ifndef SAPF_INC_SAPF_GPIO_H_
#define SAPF_INC_SAPF_GPIO_H_

#include "DSP2833x_Device.h"

//###############################################
//#             User Functions                  #
//###############################################
void ConfigGPIO(void);

//###############################################
//#     Private Translation Unit Functions      #
//###############################################
static void ConfigCable(void);
static void ConfigJ8Cable_IO(void);
static void ConfigJ3Cable_PWM_1(void);
static void ConfigJ7Cable_PWM_2(void);
static void ConfigJ11Cable_ADC(void);
static void ConfigJ4Cable_DAC_SPI(void);
static void ConfigJ2Cable_RS232(void);

#endif /* SAPF_INC_SAPF_GPIO_H_ */
