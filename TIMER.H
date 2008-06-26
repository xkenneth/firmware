//	Revision History:
//  001	11/11/03	Original Version
//  002	12/01/06	D Lerner soft timer for motor control. 
//		delete flash timer as it is done per frame not by time
//		rename flag timer to mudpulse timer

#ifndef TIMER_H
#define TIMER_H

#include	"sysdef.h"

//-----------------------------------------------------------------------------
// Timer Rate also determines the baud rate for the UART.
// The Timer Rate must be 3*Baud Rate
// example:
//      Baud Rate = 2400
//      1/(2400*3) = 138.8 usec
//
// This means that the timer irq must be called every 138 usec.
// Since the Timer IRQ is dependant on the Crystal frequency, the
// Baud Rate is also dependant on the Crystal frequency.  The CYCLE_TIME
// in the below equations is based of the Crystal frequency and is defined
// in the sysdef.h file.  If the baud rate changes all you need to do is
// change the baud rate below.  If the Crystal Frequency changes all you
// need to do is change XTAL_FREQ in sysdef.h
//-----------------------------------------------------------------------------
#define BAUD_RATE       2400.0        // bps
#define TIMER_RATE      (1000000.0/(BAUD_RATE*3.0)) // Rate of timer interrupt (in usec)
#define TIMER_COUNT     (unsigned)((TIMER_RATE*1000.0)/(float)CYCLE_TIME)

//-----------------------------------------------------------------------------
// Timer Scale
//      The timers below are dependant on the Crystal frequency defined
//      in sysdef.h. Because timers[] is a ushort and the current
//      TIMER_SCALE results in a 7.  The min time is 1msec.
//
//      Timers below are scaled to 1msec
//-----------------------------------------------------------------------------
#define TIMER_SCALE     (1000.0/TIMER_RATE) // Scale to 1 msec

enum    {GP_TIMER0, MUDPULSE_TIMER, MOTOR_CONTROL_TIMER, MOTOR_VEL_TIMER, MOTOR_WAIT_TIMER, MOTOR_RESTART_TIMER, MOTOR_FREEZE_TIMER, DATE_TIME_TIMER, MOTOR_CYCLE_TIMER, GR_LOG_TIMER, RT_DLY_TIMER, DATA_FRESH_TIMER, DATA_SAVE_TIMER, NUM_TIMERS};

#define ACCESS_TIMERS   extern ulong timers[]
#define SetTimer(a,b)   timers[a]=(ulong)((float)b*TIMER_SCALE)
#define TimerExpired(a) !timers[a]
#define TimerRead(a)    timers[a]


void TimerInit();
void TimerIRQ(int sig);

#endif


