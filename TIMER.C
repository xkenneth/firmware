//Timer interrupt code
//	Revision History:
//  001	11/11/03	Original Version
//  002	12/02/06	D Lerner call soft interrupts with if test for 
//					completion to reduce interrupt overhead.
#include	"timer.h"
#include	"stddef.h"
#include    "swuart.h"
#include    "dataout.h"
#include	"BoardCheckout1.h"
#include	<misc.h>
#include	<signal.h>

ulong	timers[NUM_TIMERS]; // 16 bit down counting timers

void TimerInit()
{	 
	// ----------------------------------- 
	// Set the timer period and enable it. 
	// -----------------------------------

	// timer_set(tperiod,tcount,tscale)
	timer_set(TIMER_COUNT,TIMER_COUNT,0);
	interrupt(SIGTIMER,TimerIRQ);
	timer_on();
}


//extern int quad;
//extern int previous_quad;
extern long long_date_time;

bool Timer_Int_Served=true;

void TimerIRQ(int sig)
//this is the actual timer interrupt code
//it is called by the timer interrupt at 3x serial baud rate
//so the swuart (software uart) can read each IO bit properly
//each item in the timer list is tested for zero and decremented
//the timers are set with counts scaled to 1 millisecond rate
//then each of soft interrupt functions are called
//with test if the soft timer is zero = completed
{
	
	static int k;
	
	// ---------------------------------------
	// Decrement each timer if it's not zero.
	// ---------------------------------------
	for(k=0;k < NUM_TIMERS;k++) {
		
		if(timers[k])
			--timers[k];
			
	}
	
	// Service the UART
    UARTServ();	//serial uart
    PWM_controller();    //PWM output controller
	
    //obtain motor position
	MotorPos();
	
	//count date time counter
	if(TimerExpired(DATE_TIME_TIMER)) {
		
		SetTimer(DATE_TIME_TIMER,1000);
		long_date_time++;
		
	}
	
	//if(!Timer_Int_Served) printf("!");
	
	//run only if the timer interrupt service program has been served completely
	//the scheme is designed to reduce allow the timer interrupt while servicing the program
	
	if(Timer_Int_Served) {
		
		Timer_Int_Served=false;
		
		//enable timer interrupt
		//asm("ar = imask;ar=ar or 0x001;imask=ar;");
		
		//update motor open and shut position
		PositionUpdate();

    	// Service the Data in the mudpulse buffer
		if(TimerExpired(MUDPULSE_TIMER))	//wait till the MUDPULSE_TIMER completes
	    	ServOutput();   // Outputs Data to motor control variables
	    
		else {
			
			// check motor velocity
			if(TimerExpired(MOTOR_VEL_TIMER))	//wait till the MOTOR_VEL_TIMER to complete
	    		Motor_Velocity();   // calculate the current velocity of the motor
	    		
	    	else {
	    		
	    		/*
	    		if(TimerExpired(MOTOR_CYCLE_TIMER)	//wait till the MOTOR_CYCLE_TIMER to complete
	    			MotorCycle();
	    			
	    		else { */
	    		
	    			// Service the motor controller
					if(TimerExpired(MOTOR_CONTROL_TIMER))	//wait till the MOTOR_CONTROL_TIMER completes
					Motor_controller();   // run the motor controller
					
	    		//}
				
	    	}
	    	
		}
		
		Timer_Int_Served=true;
			
	}
	    
	
}


