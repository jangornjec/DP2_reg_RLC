/****************************************************************
* FILENAME:     define.h           
* DESCRIPTION:  file with global define macros
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#ifndef     __DEFINE_H__
#define     __DEFINE_H__

// switching frequency
/*
 * 0.84 CPU load @ 87k
 * ESTOP         @ 87k500
 * Stable        @ 15k
 * Unstable      @ 14k
 * 0.39 CPU load @ 40k
 *
 */
#define     SWITCH_FREQ     40000L

// ratio between switching and sampling frequency
#define     SAMP_PRESCALE   1

// sampling frequency
#define     SAMPLE_FREQ     (SWITCH_FREQ/SAMP_PRESCALE)

// sampling period
#define     SAMPLE_TIME     (1.0/SAMPLE_FREQ)

// CPU speed
#define     CPU_FREQ        80000000L

// grid frequency
#define		GRID_FREQ		50.0

// math constants
#define     SQRT3           1.7320508075688772935274463415059
#define     SQRT2           1.4142135623730950488016887242097
#define     PI              3.1415926535897932384626433832795

// bool type definition
typedef enum {FALSE = 0, TRUE} bool;

// how peripherals behave on debug event
// 0 stop immediately, 1 stop when finished, 2 run free
#define     DEBUG_STOP      0

#endif // end of __DEFINE_H__ definition
