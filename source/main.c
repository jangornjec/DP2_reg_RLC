/****************************************************************
* FILE:         main.h
* DESCRIPTION:  initialisation code
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#include "main.h"

// Global variables


// Function declaration


/**************************************************************
* main function only executes initialisation code
**************************************************************/

void main(void)
{

	// local variables


	// initialise system clock
    InitSysCtrl();
    
    // default GPIO initialisation
    InitGpio();

    // initialise PIE expansion unit
    InitPieCtrl();

    // populate vector table with dummy interrupt functions
    InitPieVectTable();

    // initialise ADC and PWM
    ADC_init();

    PWM_init();

    // initialise specific GPIO functions
    PCB_init();

    // initialise periodic interrupt function
    PER_int_setup();

    // enable interrupts
    EINT;
    ERTM;

    // start timer, which will trigger ADC and an interrupt
    PWM_start();

    // proceed to background loop code
    BACK_loop();
}   // end of main






