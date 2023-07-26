/**************************************************************
* FILE:         ADC_drv.c
* DESCRIPTION:  A/D driver for piccolo devices
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#ifndef     __ADC_DRV_H__
#define     __ADC_DRV_H__

#include    "DSP28x_Project.h"

// which PWM module is the trigger source for ADC
#define     ADC_MODUL1      EPwm1Regs

// map the results registers to defines, for easier reconfiguration
#define     ADC_RESULT0     (AdcResult.ADCRESULT0)
#define     VOLTAGE         (AdcResult.ADCRESULT1)
#define     CURRENT         (AdcResult.ADCRESULT2)

/**************************************************************
* initialize ADC
**************************************************************/
extern void ADC_init(void);

/**************************************************************
* waits for the ADC to finish with current sequence
* return: void
**************************************************************/
extern void ADC_wait(void);

#endif /* __ADC_DRV_H__ */
