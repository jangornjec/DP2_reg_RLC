/****************************************************************
* FILENAME:     PER_int.c
* DESCRIPTION:  periodic interrupt code
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"
#include    "PID.h"

// variables required for voltage measurement
long    voltage_raw = 0;
long    voltage_offset = 1971;
float   voltage_gain = (3.3/4095.0)*(24.0/10.0);
float   voltage = 0.0;

// variables required for current measurement
long    current_raw = 0;
long    current_offset = 1940;
float   current_gain = (3.3/4095.0)*(1000.0/50.0)*(1.0/5.1);
float   current = 0.0;

// power stage modulation index [-1, +1]
float   mod_index = 0;

// variables for reference value generation and load toggling
float   ref_counter = 0;
float   ref_counter_prd = SAMPLE_FREQ;
float   ref_counter_cmpr = 500;
float   ref_counter_load_on = 350;
float   ref_counter_load_off = 650;

float   ref_value = 0;
float   ref_value_high = 2.5;
float   ref_value_low = 0.25;

// variables for offset calibration 
bool    offset_calib = TRUE;
long    offset_counter = 0;
float   offset_filter = 0.05;

// CPU load estimation
float   cpu_load  = 0.0;
long    interrupt_cycles = 0;

// counter of too long interrupt function executions
int     interrupt_overflow_counter = 0;

/*--------------------------------------
 * variables for the PID controllers
 -------------------------------------*/
float reference = 0.0;

// PID Current
#define Kp_i       1.0f
#define Ki_i     100.0f
#define Kd_i       0.0f
#define tau_i      0.0f
#define limMin_i  -3.3f
#define limMax_i   3.3f
#define T   SAMPLE_TIME
// PID Voltage
#define Kp_u       5.5f
#define Ki_u      15e3f
#define Kd_u     15e-5f
#define tau_u     2e-5f
#define limMin_u  -4.5f
#define limMax_u   4.5f

// PID controllers structures
PIDController PID_current = {Kp_i, Ki_i, Kd_i,
                             tau_i, T,
                             limMin_i, limMax_i};

PIDController PID_voltage = {Kp_u, Ki_u, Kd_u,
                             tau_u, T,
                             limMin_u, limMax_u};


/**************************************************************
* interrupt function
**************************************************************/
#pragma CODE_SECTION(PER_int, "ramfuncs");
void interrupt PER_int(void)
{
    /// local variables

    // acknowledge interrupt within PWM module
    EPwm1Regs.ETCLR.bit.INT = 1;
    // acknowledge interrupt within PIE module
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    // start CPU load stop-watch
    interrupt_cycles = TIC_time;
    TIC_start();

    // get previous CPU load estimate
    cpu_load = (float)interrupt_cycles * ((float)SAMPLE_FREQ/CPU_FREQ);

    // increase and wrap around interrupt counter every 1 second
    interrupt_cnt = interrupt_cnt + 1;
    if (interrupt_cnt >= SAMPLE_FREQ)
    {
        interrupt_cnt = 0;
    }

    // wait for the ADC to finish with conversion
    ADC_wait();

    // at startup calibrate offset in current measurement
    if (offset_calib == TRUE)
    {
        PWM_update(0.0);
        current_raw = CURRENT;
        current_offset = (1.0 - offset_filter) * current_offset + offset_filter * current_raw;

        voltage_raw = VOLTAGE;
        voltage_offset = (1.0 - offset_filter) * voltage_offset + offset_filter * voltage_raw;

        offset_counter = offset_counter + 1;
        if ( offset_counter == (long)5 * SAMPLE_FREQ)
        {
            offset_calib = FALSE;
        }

    }
    // otherwise operate normally
    else
    {
    	// calculate voltage measured from ADC data
        voltage_raw = VOLTAGE - voltage_offset;
        voltage = -voltage_raw * voltage_gain;

        // calculate current measured from ADC data
        current_raw = CURRENT - current_offset;
        current = -current_raw * current_gain;

        // counter for reference value and load toggling
        ref_counter = ref_counter + 1;
        if (ref_counter >= ref_counter_prd)
        {
            ref_counter = 0;
        }

        // generate reference value
        if (ref_counter > ref_counter_cmpr)
        {
            ref_value = ref_value_low;
        }
        else
        {
            ref_value = ref_value_high;
        }

        reference = ref_value;
        

        // place for PID controller
        PIDController_Update(&PID_voltage, reference, voltage);
        PIDController_Update(&PID_current, PID_voltage.out, current);

        mod_index = PID_current.out / 3.3f;

        // set modulation index of "power" stage.
        // Specific duty cycles for each transistor are calculated internally
        PWM_update(mod_index);
        
    }
    
    // store values for display within CCS or GUI
    DLOG_GEN_update();

    /* Test if new interrupt is already waiting.
     * If so, then something is seriously wrong.
     */
    if (EPwm1Regs.ETFLG.bit.INT == TRUE)
    {
        // count number of interrupt overflow events
        interrupt_overflow_counter = interrupt_overflow_counter + 1;

        /* if interrupt overflow event happened more than 10 times
         * stop the CPU
         *
         * Better solution would be to properly handle this event
         * (shut down the power stage, ...)
         */
        if (interrupt_overflow_counter >= 10)
        {
            asm(" ESTOP0");
        }
    }

    // stop the sCPU load stop-watch
    TIC_stop();

}   // end of PWM_int

/**************************************************************
* Function which initialises all required for execution of
* interrupt function
**************************************************************/
void PER_int_setup(void)
{

    // Initialise PID
    PIDController_Init(&PID_current);
    PIDController_Init(&PID_voltage);

    // initialise data logger
    // specify trigger value
    dlog.trig_level = SAMPLE_FREQ - 10;    
    // trigger on positive slope
    dlog.slope = Positive;                 
    // store every  sample
    dlog.downsample_ratio = 1;             
    // Normal trigger mode
    dlog.mode = Normal;                    
    // number of calls to update function
    dlog.auto_time = 100;                  
    // number of calls to update function
    dlog.holdoff_time = 100;               

    dlog.trig = &ref_counter;
    dlog.iptr1 = &voltage;
    dlog.iptr2 = &current;
    dlog.iptr3 = &reference;
    dlog.iptr4 = &current;

    // setup interrupt trigger
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;
    EPwm1Regs.ETCLR.bit.INT = 1;
    EPwm1Regs.ETSEL.bit.INTEN = 1;

    // register the interrupt function
    EALLOW;
    PieVectTable.EPWM1_INT = &PER_int;
    EDIS;

    // acknowledge any spurious interrupts
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    // enable interrupt within PIE
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // enable interrupt within CPU
    IER |= M_INT3;

    // enable interrupt in real time mode
    SetDBGIER(M_INT3);
}






