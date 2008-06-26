/* -------------------------------------------------------------- */
//	Revision History:
//  001 11/11/03	Original Version
//  002 08/03/05	Flag 2 is now needed to drive permenant magnet
//					solenoid. Do not blink it here.
//  003	09/15/05 	Added #define __SIMULATION__ to Backgrd.h to 
//					conditionally compile some wait loops when
//					running in summulation. 
//					REMEMBER TO COMMENT IT OUT FOR REALTIME PGM !!
//	004	01/04/06	in dataout.c change data items sent in raw data frames
//	005	void
//	006 09/14/06	use motor to drive pulser and FM modulation
//					save each frame to flash and don't use flash timer
//	007 12/01/06	use pulse chirp modulation
//  008 03/26/08	Code scheme changed from chirp to interleaved 2 to 5 bar code
//					
//					
//
/* -------------------------------------------------------------- */
/*COMMENT*/
#include	<signal.h>
#include	"stddef.h"
#include	"timer.h"
#include	"Backgrd.h"
#include	"BoardCheckout1.h"
#include	"peterc.h"
#include    "dataout.h"
#include    "RS232.h"

ACCESS_TIMERS;				     
//volatile short count;				   
static short count = 0;
extern unsigned long data_save_rate;
/* -----------------------------------------------------------------------------
// This is the entry point for the program. 
// -----------------------------------------------------------------------------*/

void main()
{
	/* ----------------------- */
	/* Init various components */
	/* ----------------------- */

	TimerInit();
	Startup();
    UARTInit();
	//SetPF4ModeOutput();	//setup for motor control using PF4 for RUN/STOP
    //	printf("hello world \n");
    //SetPF4();
    //ClrPF4();
    LoadCoeff();
    LoadStatus();
    LoadPattern();
    LoadPatternSeq();
    
    //Motor_Rty_Calib();		//calibrate the open and shut position
    //Motor_Lin_Calib();		//calibrate the open and shut position
    Motor_Opt_Calib();		//calibrate the open and shut position
    FindAddr();				//find the beginning address to save the logging data
        
    //Motor_Calib_Shut();		//caliberate shut position
    //printf("SHUT: %d\n",motor_shut_position);
    //Motor_Calib_Open();		//caliberate open position
    //printf("OPEN: %d\n",motor_open_position);
    
	// --------------------------------- 
	// main program loop runs continuously
	// GP_TIMER0 is used to trigger data acq and processing
	// --------------------------------- 
    SetTimer(GP_TIMER0,2000);   /* 2000 msec time when battery is connected */
    SetTimer(DATA_SAVE_TIMER, 2000);	/* wait untill logging data available to save */
    
	while(1) //loop forever
	{
#if __SIMULATION__	
//		if(1)
//		{
		printf("ACQ loop begin \n");
#else

		if(TimerExpired(GP_TIMER0))
		{
#endif
			SetTimer(GP_TIMER0,1000);   /* 1000 msec time between acquisitions */
			
			Motor_Restart();			/* Restart motor if it stalls */

            AcquireData();              /* Read new data */
    	//	RS232Serv();                /* Make Sure RS232 Gets Served */
			UpdateChAvg();              /* Calculate the channel averages */			
            PeterC();                   /* Do additional processing. */
    	//	RS232Serv();                /* Make Sure RS232 Gets Served */
            count=GR_CPS(count);      	/* Do GammaRay processing. */
			count++;
			
	    	Pattern2Frame();          		/* Put data into the pulser buffer array */
	    	
	    	CoeffReset();
	    		    	
		}
				
		if (TimerExpired(DATA_SAVE_TIMER)) {
		
			SetTimer(DATA_SAVE_TIMER, data_save_rate);	/* 60 sec between saving logging data */
			
			SaveData();					/* Save data every 60 sec */
			
		}
				
		RS232Serv();
		
#if __SIMULATION__	
		printf("GP_TIMER0 = %f\n",TimerRead(GP_TIMER0));
		printf("loop end \n");
#endif
	}
}



