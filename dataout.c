//-----------------------------------------------------------------------------
// DATA OUTPUT FILE
//	Revision History:
//  001	11/11/03	Original Version
//  002	3/2/04		P. Masak  Hardcoded outputs for testing/altered timer
//                  rate slightly to get 1 second pulses
//  003	4/21/05  	D.Lerner start work to change pulse output to characterize
//                  pulser maximum operation rate.
//  004	8/03/05		Send data as frequency chirps. This rev only produces 
//					basic chirp test pattern. 
//					Eight pulses per chirp (4 cycles)
//					Chirps are monotonic - no skipped tones.
//					Tones defined using standard musical chromatic scale.
//					Lowest tone is C-4 978 ms per half cycle.
//					Drive board for permenant magnet solenoid uses Flag 1 and 2.
//					Just needs narrow drive pulse to magnetically latch solenoid.
//					Flag 1&2 must never be on at same time or they short the battery.
//	005	08/07/05	Iron core solenoid version has 100% drive pulse width.	
//	006	09/20/05	Removed chirp codes and added MOD7 modlation.
//	007	10/17/05	Removed MOD7 and added very slow FSK and PSK for testing
//	008	11/01/05	Removed FSK. Change PSK to 1 bit per cycle. 
//					Lower pulse rate to provide more robust solenoid drive.
//	009	12/06/05	Fixed error in FormatStartSync()
//					changed PULL_TIME to 200 for longer drive in PM mode
//					TIME_BETWEEN_FRAMES 	15000
//	010	01/04/05	Change order of data items sent in raw data frames.
//
//	011	01/17/05	Support for original PWM drive board dropped.
//					Only new drive board will work properly with following code.
//					This is because software PWM is implimented here.
//					Impliment #define PULL_TIME 
//					Add soft PWM for iron core hold current.
//					Moved inter-frame timing to Data2Frame() so frame data is fresh.
//	012	01/19/05	Change PULL_TIME to 120 ms as coil current saturates in that time.
//					Change PWM to 50% which results in 250 ma coil current.
//	013	01/25/06	Add new frames for short calculated data and variant of raw data
//					lengthen time between frames and set frame sequence to reduce power
//					Correct GR is on avg8 channel.
//					Use GammaRayCPS value from GR_CPS() totalizer
//					added #define to convert azi,inc,G,H to integers in range
//	014	02/02/06	In CloseON() and OpenON():
//					to stop cross conduction in the H bridge added
//						1 ms to PWMstate==0)&&(HOLDremaining<(PWM_ON+1) 
//					Cleanup logic in FormatMudPulses() 
//					Cleanup logic in Data2Frame()
//	015	02/20/06	Increased PULL time to 140 ms
//					Used #defines for PWMstateOFF PWMstateON 
//					PWM to output the most possible of PWM pulses
//	016 03/10/06	Major revamp of modulation method to Pulse Chirps
//					This revamp includes patentable modulation methods.
//					Data will be sent using minimum width positive going mud pulses.
//					This will provide the minimum restriction to mud flow through our tool.
//					Pulses are grouped in "chirps" to improve signal to noise ratio.
//					Rapidly changing data (Gamma Ray and Tool Face) is sent more often.
//					Slow rate data is sent with a subchannel at a slower bit rate.
//					Provision is made for alarm data such as rapid pressure changes.
//					Change ServeOutput() to mark buffer empty when end of buffer data is reached.
//					Gamma Ray in counts per second, 1000 CPS maximum limit.
//	017	06/06/06	feature to control pulse width.
//					Simplify interrupt handler. 
//					Pulse buffer contains packed binary.
//						MSB is FLAG1	MSB-1 is FLAG2	14 LSBs are timer setting
//	019	06/26/06	increase pulse width to one second and adjust 
//					STEPs accordingly.
//					Allow PULL time to be shorter than the one second pulse.
//					Move logic for PULL, Dwell, and merging multiple pulses in same direction
//					from the interrupt handler to the modulator code. Logic for merging pulses 
//					is not robust.
//	020	09/01/06	Changing to motor drive instead of solenoid. 
//					Remove PULL logic as motor does not have this feature.
//					Remove flag macros since motor drive interrupt is in swuart for now.
//					LONGEST_PULSE set default to 1000 for now.
//					PULL_TIME is not used with motor drive, so remove it.
//					AddPulse() merge pulses in same direction for future advanced modulation 
//					like BPSK.
//					Increase mudpulse buffer to 512 for FM modulation
//					Store each frame to FLASH and eliminate the subchannel buffer.
//					CalcLogGR in log10 output scale to 256 unsigned short
//	021	12/01/06	Change modulation method to Pulse Chirps but using motor drive. 
//					Reinstate subchannel buffer.
//					Add Motor speed and position control interrupt. 
//					simulate position encoder for now.
//					PULSE_TIME set default to 500 (milliseconds) for now.
//	022	12/07/06	All digital pulse chirps, 4 bit encoding per chirp, pseudo-random chirp timing.
//					Add assy code for motor STOP and RUN.
//	023	02/02/07	Motor position obtained from the motor controller board. Break was added to slow
//					the motor before stop
//	024 03/09/07	Motor caliberation and corresponding motor position warpping were established for 
//					rotary cup set up.
//	025	04/18/07	Open and shut position reversed in caliberation dynamics
//	026	09/22/07	Rotary cup calibration with pattern code built in 
//	027	10/12/07	Linear cup calibration
//
//
// Description:
//      This file outputs the sensor data formatted into frames
//		one bit at a time
//		through modulating the drive motor
//      and saves each data frame to flash with
//      a time stamp.
//-----------------------------------------------------------------------------
//#include 	"C:\Program Files\Analog Devices\VisualDSP 3.5 16-Bit\218x\include\def218x.h"
//#include	<def218x.h>
//#include <def2191.h> 
#include    "sysdef.h"
#include	"stddef.h"
#include	"Backgrd.h"
#include	"dataout.h"
#include	"timer.h"
#include    "flash.h"
#include	"RS232.h"
#include	"Peterc.h"
//#include 	"swuart.h"
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
ACCESS_TIMERS;
extern 	float 	G,H,azi,inc,Temperature;	//calculated values
//definitions to convert floating point values to useful integers
//in the range of expected values 
#define azimuth 	(unsigned)min(9999.0, azi*10000.0/360.0) //expected value 
#define inclination (unsigned)min(9999.0, inc*10000.0/180.0)	//expected value 
#define G1000 		(int)(G*1000.0)	//expected value 1.000
#define H1000 		(int)(H*1000.0)	//expected value 1.000
//#define GX1000		(int)(Gx*1000.0)
//#define GY1000		(int)(Gy*1000.0)
//#define GZ1000		(int)(Gz*1000.0)
//#define HX1000		(int)(Bx*1000.0)
//#define HY1000		(int)(By*1000.0)
//#define HZ1000		(int)(Bz*1000.0)
#define TEMP		(unsigned)min(9999.0, Temperature*10000.0/500.0)		//temperature in 0.1 degree

extern	float 	G,H,azi,inc,Temperature;	//these seem to be globally accessed
extern  float	Gx,Gy,Gz,Bx,By,Bz;
//extern	short	GammaRayCPS;	//the Gamma Ray in counts per minute
//extern	short	ToolFace;		//calculated ToolFace in 0.1 degree per bit
extern	double	ToolFace;		//calculated ToolFace in 0.1 degree per bit
extern	short	Pressure;		//calculated pressure
extern	short	avg1,avg2,avg3,avg4,avg5,avg6;
extern  short   avg7,avg8,avg9,avg10,avg11;     /* Averages from BoardCheckout1.asm */

extern unsigned short status[NUM_STATUS_CONST];

extern	unsigned GRayCPS;		//gamma ray counts per second
extern  double	Avg_GRayCPS;	//average gamma ray count per second
unsigned GRBin_buf[GRBIN_BUF_SIZE];			//gamma ray spike bin buffer
unsigned GRBin_CPS[GRBIN_BUF_SIZE];			//updated gamma ray bin counts per second

//unsigned short 	seq_count = 0;   // Counts how many times a sequence was saved to flash
//unsigned short	ToolStatus = 0xa5 ;	//future tool status bits
unsigned short	ToolStatus=0;	//future tool status bits
//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Timer Rates
//-----------------------------------------------------------------------------
//#define TIMER_BIT_RATE      978 // (msec) Time length of each bit output - old

// data now saved to flash when each frame is initiated
//#define SAVE_FLASH_TIME    60000    // (msec) Rate at which data gets saved to flash
    // This means the data block will be saved to flash every 1 min.
    // To determine how many hours you can continuously save data before
    // at loops around and starts over writing data use below formula
    //
    // hours = ((0x80000-DATA_FLASH_ADDR)/FLASH_BLOCK_SIZE)*(secs)/3600
    //
    // so:
    //      (0x80000-0x5000)/18 = 27989
    //      (27989*60)/3600 = 466 hours
    
//-----------------------------------------------------------------------------
// Constants
//      eventually we will add a time stamp which will increase the
//      number of bytes saved to flash. It is multiplied by two because
//      the data_seq array is a ushort.
//-----------------------------------------------------------------------------
//#define FLASH_BLOCK_SIZE    NUM_OF_SEQ*2  // Number of bytes saved to flash in one block
//#define MAX_FLASH_SIZE      (0x80000-FLASH_BLOCK_SIZE)  // Size of flash to last block able to store

// Date variable
unsigned short date_time[NUM_OF_DATE];

unsigned long long_date_time=0;

// Routine to update the date and time of the tool
void DateTimeUpdate()
{
	
	unsigned short_time;
	
	long_date_time=((((((ulong)date_time[YEAR]-1)*12+(ulong)date_time[MONTH]-1)*30+(ulong)date_time[DAY]-1)*24+(ulong)date_time[HOUR])*60+(ulong)date_time[MINUTE])*60+(ulong)date_time[SECOND];
	//put the time into a 32 bit word in unit of second
	status[SC_DATE_TIME_H]=short_time=(unsigned)(long_date_time>>16);
	SetStatus(SC_DATE_TIME_H,short_time);
	status[SC_DATE_TIME_L]=short_time=(unsigned)((long_date_time<<16)>>16);
	SetStatus(SC_DATE_TIME_L,short_time);
	
}

// Routine to calculate the current date and time upon the long_date_time
void DateTimeCalc()
{
	ulong year_second=31104000;
	ulong month_second=2592000;
	ulong day_second=86400;
	ulong hour_second=3600;
	ulong minute_second=60;
	
	ulong time_resid=long_date_time;
	
	date_time[YEAR]=time_resid/year_second+1;
	time_resid-=(date_time[YEAR]-1)*year_second;
	
	date_time[MONTH]=time_resid/month_second+1;
	time_resid-=(date_time[MONTH]-1)*month_second;
	
	date_time[DAY]=time_resid/day_second+1;
	time_resid-=(date_time[DAY]-1)*day_second;
	
	date_time[HOUR]=time_resid/hour_second;
	time_resid-=date_time[HOUR]*hour_second;
	
	date_time[MINUTE]=time_resid/minute_second;
	time_resid-=date_time[MINUTE]*minute_second;
	
	date_time[SECOND]=time_resid;
	
}
	
/*
//-----------------------------------------------------------------------------
// DATA SEQUENCE ARRAY
//      There are two arrays.
//      data_seq - holds the actual data of the sequence
//      data_size - holds the size in bits of the data in the sequence
//
//-----------------------------------------------------------------------------
#define SEQ_MONTH       0
#define SEQ_DAY         1
#define SEQ_YEAR        2
#define SEQ_HOUR        3
#define SEQ_MINUTE      4
#define SEQ_SECONDS     5
#define SEQ_FRAME_ID    6
#define SEQ_DIAGNOSIC	7 //future diagnostic check data
#define SEQ_INC         8
#define SEQ_AZI         9
#define SEQ_G           10
#define SEQ_H           11
#define SEQ_GR          12
#define SEQ_TF	        13
#define SEQ_GX	        14
#define SEQ_GY	        15
#define SEQ_GZ	        16
#define SEQ_HX	        17
#define SEQ_HY	        18
#define SEQ_HZ	        19
#define SEQ_TEMP        20
#define NUM_OF_SEQ      21
*/

// Data array !this array must match the array below!
//unsigned short data_seq[NUM_OF_SEQ] = {


unsigned data_seq[NUM_OF_SEQ] = {
    //0,      // Month
    //0,      // Day
    //0,      // Year
    //0,      // Hour
    //0,      // Minute
    0,		//htime
    0,		//ltime
    //0,      // Second
    0x20,   // Frame ID
    0,		// diagnostic
    0,      // inclination
    0,      // azimuth
    0,      // G1000
    0,      // H1000
    0,      // GR (GammaRayCPS)
    0,		// tool face
    0,		// gx raw
    0,		// gy raw
    0,		// gz raw
    0,		// hx raw
    0,		// hy raw
    0,		// hz raw
    0,      // Temperature
    0,		// Pressure
};

unsigned data_qt_seq[NUM_OF_SEQ];		//saved data when the tool is not rotating

unsigned data_log[NUM_OF_LOG] = {
	
	0,		//htime
    0,		//ltime
    //0,      // Second
    //0x20,   // Frame ID
    0,		// diagnostic
    //0,      // inclination
    //0,      // azimuth
    //0,      // G1000
    //0,      // H1000
    //0,      // GR (GammaRayCPS)
    //0,		// tool face
    0,		// gx raw
    0,		// gy raw
    0,		// gz raw
    0,		// hx raw
    0,		// hy raw
    0,		// hz raw
    0,      // Temperature
    //0,		// Pressure
};

unsigned data_static[NUM_OF_SEQ] = {
	
	1000,
    10000,
    0,
    0xa5,	//0xa5
    0x0213,	//inclination
    0x0421,	//azimuth
    0x03e8,
    0x03e7,
    0x0f, 	// GR
    0x7f,		// Tool face calculated 
    0x0123,			//raw gx Ax
    0x4567,			//raw gy Ay
    0x89ab,			//raw gz Az
    0xcdef,			//raw hx Mx
    0x45,			//raw hy My
    0x46,			//raw hz Mz
    0x08,			//temperature
    0x08,			//pressure
    
};

// Bit sizes of above data !this array must match the array above!
static short data_size[NUM_OF_SEQ] = {
    
	//8,      // Month
    //8,      // Day
    //8,      // Year
    //8,      // Hour
    //8,      // Minute
    //8,      // Second
    
    16,      // Htime
    16,      // Ltime
    
    8,      // Frame ID
    8,      // diagnostic
    12,     // inclination  //was 10 in peters code
    12,     // azimuth
    8,      // G1000
    8,      // H1000
    8,      // GR (GammaRayCPS)
    8,		// tool face
   16,		// gx raw
   16,		// gy raw
   16,		// gz raw
   16,		// hx raw
   16,		// hy raw
   16,		// hz raw
    8,      // Temperature
    8,		// Pressure
};


unsigned long data_save_rate;
unsigned long data_fresh_rate;
unsigned gr_log_size;
#define GR_LOG_RATE		(ulong)(data_save_rate/(ulong)gr_log_size)
unsigned gr_log[GR_BUF_SIZE];
unsigned gr_idx=0;

unsigned CalcLogGR(double GRay)
{
	
//#define CPS_MAX 	1000	//GR counts 1000 CPS = full scale
//#define CPS_MAX 	200	//GR counts 500 CPS = full scale
//#define LOG_CPS_MAX	(log10(CPS_MAX))
#define LOG_CPS_MAX	(log10(CPS_MAX/5))  
#define FULL_SCALE_GR 10000.0 //logGR at full scale

	double y = LOG_CPS_MAX;
	double x;	

	//ratio of GRay to full scale, convert to log
	//x = (log10((float)(max(GRay,1)) )) / y;
	//x = log10(max(GRay,1) );
	x = log10(max(GRay,5)/5 );		
	
#if __SIMULATION__
		printf("GRay = %x \tlog10(GRay)ratio = %f \tscaled ushort=%x\n",
		GRay,x,((unsigned short)(x*FULL_SCALE_GR/y)));
#endif

	return( (unsigned)(min(9999.0, x*FULL_SCALE_GR/y)) );	
}

//Definition of boolean flags used for RS232 control

bool Motor_Capture = false;		//flag to stop the motor
bool Sensor_Live;		//flag to send live sensor data through RS232
bool Motor_Freeze=false;	//flag if the battery is too low to run the motor
bool Tool_Rotate=true;		//flag if the tool is rotating
bool Tool_Quiet=false;		//flag if the tool is quiet for data acquiring
bool Data_Fresh=false;		//flag if quiet data is ready to be sent

unsigned long rt_dly_rate;

void CalcToolStatus()
{
	
	unsigned short temp_status=0;
	
	if (Tool_Rotate) temp_status=temp_status|(0x0001<<TS_TOOL_ROTATE);
	if (Tool_Quiet) temp_status=temp_status|(0x0001<<TS_TOOL_QUIET);
	if (Motor_Freeze) temp_status=temp_status|(0x0001<<TS_MOTOR_FREEZE);
	
	ToolStatus=temp_status;
	
}

//store gamma ray data
void FreezeGRData()
{
	
	if(TimerExpired(GR_LOG_TIMER)) {
    	
    	SetTimer(GR_LOG_TIMER, GR_LOG_RATE);
    	gr_log[gr_idx++]=data_seq[SEQ_GR];
    	
    	if(gr_idx>=gr_log_size) gr_idx=0;
    	
    }
    
}

//store log data
void FreezeLogData()
{
	
	data_log[LOG_HTIME]=data_seq[SEQ_HTIME];
	data_log[LOG_LTIME]=data_seq[SEQ_LTIME];
	data_log[LOG_DIAGNOSIC]=data_seq[SEQ_DIAGNOSIC];
	data_log[LOG_GX]=data_seq[SEQ_GX];
	data_log[LOG_GY]=data_seq[SEQ_GY];
	data_log[LOG_GZ]=data_seq[SEQ_GZ];
	data_log[LOG_HX]=data_seq[SEQ_HX];
	data_log[LOG_HY]=data_seq[SEQ_HY];
	data_log[LOG_HZ]=data_seq[SEQ_HZ];
	data_log[LOG_TEMP]=data_seq[SEQ_TEMP];
	
}

//store data acquired when tool is not rotating
void FreezeRtData()
{
	
	int i;
	
	if(Tool_Quiet) {
		
		for (i=0;i<NUM_OF_SEQ;i++) {
		
			data_qt_seq[i]=data_seq[i];
			
		}
		
		if(!Data_Fresh) Data_Fresh=true;
		
	}
	
}
	

void FreezeData()
{ //copy current measurements into data_seq buffer
    // Time Stamp and increment seque-nce count 
    // (it's stored in seconds at the moment)
    
    CalcToolStatus();

    data_seq[SEQ_HTIME]   = long_date_time>>16;
    data_seq[SEQ_LTIME]   = (long_date_time<<16)>>16;
        
    //data_seq[SEQ_FRAME_ID]	= 0; //flag that value is not set
    data_seq[SEQ_DIAGNOSIC]	= ToolStatus;
    data_seq[SEQ_INC]     	= inclination;
    data_seq[SEQ_AZI]     	= azimuth;
    data_seq[SEQ_G]       	= G1000;
    data_seq[SEQ_H]       	= H1000;
    //data_seq[SEQ_GR]      	= GRayCPS; 	// GR
    //data_seq[SEQ_GR]      	= avg8; 	// GR
    //data_seq[SEQ_GR]      	= CalcLogGR(avg8); 	// GR
    data_seq[SEQ_GR]      	= CalcLogGR(Avg_GRayCPS); 	// GR
    //data_seq[SEQ_TF]      	= ToolFace;		// Tool face calculated
    data_seq[SEQ_TF]      	= (unsigned)(min(9999.0, (ToolFace * 10000.0) / 360.0));	// Tool face calculated
    //data_seq[SEQ_TF]      	= (unsigned)ToolFace;		// Tool face calculated
    	    	     	
    data_seq[SEQ_GX]      	= avg7;			//raw gx Ax
    data_seq[SEQ_GY]      	= avg6;			//raw gy Ay
    data_seq[SEQ_GZ]      	= avg5;			//raw gz Az
    data_seq[SEQ_HX]      	= avg4;			//raw hx Mx
    data_seq[SEQ_HY]      	= avg3;			//raw hy My
    data_seq[SEQ_HZ]      	= avg2;			//raw hz Mz
    	
    //data_seq[SEQ_GX]      	= GX1000;			//raw gx Ax
    //data_seq[SEQ_GY]      	= GY1000;			//raw gy Ay
    //data_seq[SEQ_GZ]      	= GZ1000;			//raw gz Az
    //data_seq[SEQ_HX]      	= HX1000;			//raw hx Mx
    //data_seq[SEQ_HY]      	= HY1000;			//raw hy My
    //data_seq[SEQ_HZ]      	= HZ1000;			//raw hz Mz
    //data_seq[SEQ_TEMP]    	= Temperature;		//calibrated tempreature
    data_seq[SEQ_TEMP]    	= TEMP;		//calibrated tempreature
    data_seq[SEQ_PRES]		=0;		//pressure
    
    FreezeGRData();
    FreezeLogData();
    
    FreezeRtData();
}

//-----------------------------------------------------------------------------
// Local Functions
//-----------------------------------------------------------------------------
    
//-----------------------------------------------------------------------------
// FORMAT MUD PULSES ROUTINES
//
// Description:
//	Formats data into the mudpulse circular buffer.
//	Mudpulse buffer contains packed binary.
//		MSB is FLAG1 = CLOSE	MSB-1 is FLAG2 = OPEN
//		14 LSBs are timer setting
//
//	Pulse Chirp encoding is designed for minimum pressure drop in mud pulse telemetry.
//	Detailed documentation is in a seperate document.
//	Short pulses are used to keep the mud valve open as much as possible.
//	The pulses are all the same width, ie the minimum time for a reliable pulse.
//	Data is encoded in pulse STEPs.
//	Two methods of data encoding are used simultaneously to send fast and slow data rates.
//	Fast data is not as accurate, but is sent as often as possible.
//	For an MWD tool the fast data is Gamma Ray counts and tool face.
//	Gamma Ray (GR) is encoded as 8 bit digital data using a log scale compression.
//	Tool Face (TF) is encoded as 8 bit digital data with linear scale.
//	Regular data is encode to the precision required in a sub channel,
//		interleaved with the fast rate data.
//	Chirps have specific numbers of pulses and increasing or decreasing STEPs.
//	The STEPs are specific patterns to avoid harmonics and increase signal to noise ratio.
//	Slow data is sent in binary as a subchannel.
//	Specific chirps identify the data items which will be sent.
//	Chirp STEP patterns represent the binary data.
//
//	Starting a new data frame freezes the subchannel data into local variables
//	This time stamps the data at the frame start time.
//	Starting with an empty buffer, pulse edges are added to the buffer.
//	A frame chirp is sent to identify the start of data and data order. 
//	The first chirp of a frame identifies the frame and data in the frame.
//	The frame chirp is 6 pulses long to mark the start of each frame.
//	There are 16 possible frame types (4 bit code).
//	The next chirp will start at a specific time from the beginning of the frame chirp.
//	The time for the next chirp start is specific to the data sent chosen to be non harmonic.
//	Then 2 data chirps send the first high rate data value.
//	High rate data (GR & TF) are fresh as possible, represented by two chirps.
//	Next a chirp which contains slow rate subchannel data is sent.
//
//	Slow rate binary Data is encoded using various chirps with differing STRETCHs.
//	Binary data is added four bits at a time with each chirp.
//	The last chirp is an xor checksum of the data.
//
//	The actual data encoding is done by STRETCHing the STEP timings.
//	There are 8 amounts of STRETCH which produce 8 unique logaithmic STEP sequences.
//		thus representing 3 bits.
//	The fourth bit is represented by the STEP seuence ascending or decending.
//	
//	Following is information about chromatic tones for reference
//	Twelve tones per octave - called half steps - 2**(1/12) per step
//		one chromatic half step 1.0594630943593 = 2**(1/12)
//		two chromatic half step 1.1224620483094	= 2**(2/12)
//		bit_time = ((bit_time * 1000000) / 1059463); 
//
//	#define TIMER_BIT_RATE_MAX   978    // (msec) C-4 note time = 1.022495 Hz
//	#define TIMER_BIT_RATE_MIN   183    // (msec) fastest the pulser can work
//	
//-----------------------------------------------------------------------------
#define CLOSEFLAG	0x080000000	//flag bit to CLOSE the pulser valve sending a pressure pulse
#define OPENFLAG	0x040000000	//flag bit to OPEN the pulser valve sending a pressure pulse
#define DATABITS	0x03FFFFFFF	//flag bit to OPEN the pulser valve sending a pressure pulse
//#define PULSE_TIME	500				    	// (msec) valve is closed a fixed time
//#define GAP_MIN		PULSE_TIME				//minimum gap
//#define CYCLE_MIN	PULSE_TIME + GAP_MIN	//min cycle time for reference

unsigned pulse_time;	//(msec) time of the motor runing to close position
unsigned code_pulse_time;		//(msec) time of the bar code pulsing time
unsigned code_pulse_diff;		//minimal time difference between the full pulse time and code time

unsigned gear_numerator;
unsigned gear_denominator;
unsigned encoder_scale;

//#define COUNTS_PER_TURN		(GEAR_NUMERATOR*ENCODER_SCALE)		//position counts per out-gear full turn adjusted according to the GEAR_DENOMINATOR
#define COUNTS_PER_TURN		((gear_numerator*encoder_scale)/gear_denominator)		//position counts per out-gear full turn adjusted according to the GEAR_DENOMINATOR

unsigned N_Pulse;		//narrow pulse time
unsigned W_Pulse;		//wide pulse time
unsigned tts;			//time to close
unsigned tto;			//time to open

unsigned code[NUM_OF_CODE]={
	
	0x0006, 		//00110,	0: NNWWN
	0x0011,			//10001,	1: WNNNW
	0x0009,			//01001,	2: NWNNW
	0x0018,			//11000,	3: WWNNN
	0x0005,			//00101,	4: NNWNW
	0x0014,			//10100,	5: WNWNN
	0x000C,			//01100,	6: NWWNN
	0x0003,			//00011,	7: NNNWW
	0x0012,			//10010,	8: WNNWN
	0x000A,			//01010,	9: NWNWN
	//0x0000,			//00000,	SYNC
	
};

unsigned syncNW;		//bit sync indicator narrow (0) or wide (1)
unsigned syncPN;		//bit sync indicator pulse (1) or no pulse (0)
unsigned syncLEN;		//length of the sync

// mudpulse circular buffer for actual pulse times to modulate
//#define MAX_MUDPULSES	128 //must be 2^n, length of mudpulse buffer.
#define MAX_MUDPULSES	256 //must be 2^n, length of mudpulse buffer.
#define MUDPULSE_MASK	MAX_MUDPULSES - 1	//to create circular buffer
unsigned long mudpulse[MAX_MUDPULSES] ={0};	//transmit array of mud pulse times in milliseconds
short mudfetchindex = 0;  	//index to the pulse being transmitted
short mudstoreindex = 0;  	//index to the pulse being stored into the array
short mudpulses = 0; 		//number of pulses in the buffer
	//array has data when mudpulses > 0


	
void PushMudpulse(unsigned long mudpulse_code)
{	mudpulse[mudstoreindex++] = mudpulse_code ; //the PULL pulse
	mudstoreindex = mudstoreindex & MUDPULSE_MASK; //for circular buffer push
	mudpulses++;	//increment the mudpulse counter
}	
	
unsigned long PullMudpulse()
{	unsigned long pulse;
	pulse = mudpulse[mudfetchindex++]; //the PULL pulse
	mudfetchindex = mudfetchindex & MUDPULSE_MASK; //for circular buffer push
	mudpulses--;	//decrement the mudpulse counter
	return(pulse);	//returns the value from the mudpulse buffer
}	
	
unsigned long LastDrivePulse = 0; // fill with the drive flag when filling mudpulse buffer

void AddPulse(unsigned long Pulse_Width, unsigned long DriveFLAG)
	//time value in milliseconds for a pulse
	//DriveFLAG is direction of the pulse
	//Store the pulse into the mud pulse cicular buffer
{	if(Pulse_Width == 0) return; //do nothing if the pulse is zero length
	PushMudpulse(Pulse_Width|DriveFLAG); //Store pulse code to mudpulse buffer
	LastDrivePulse = DriveFLAG;
#if __SIMULATION__
	printf("Pulse_Width=%x \tDriveFLAG=%x \tChirpWidth=%d\n",Pulse_Width,DriveFLAG,ChirpWidth);
#endif
}


void AddCycle(ulong cycle_time)	//adds a regular cycle to the mudpulse array
{

		AddPulse(pulse_time, CLOSEFLAG);	//CLOSE for wide pulse
		AddPulse(cycle_time-pulse_time, OPENFLAG);	//CLOSE for narrow pulse 
				
}

unsigned AddCodePulse(unsigned code_width)		//add a code pulse to the mudpulse buffer
{
	
	unsigned code_time=0;
	
	if(code_width>(code_pulse_time+(tts-tto)+code_pulse_diff)) {	//code with > code pulse time + minimal time difference
	
		AddPulse((code_width-(code_pulse_time+(tts-tto)))/2, OPENFLAG);	//open so that the peak is centered
		code_time+=(code_width-(code_pulse_time+(tts-tto)))/2;
		AddPulse(code_pulse_time+(tts-tto), CLOSEFLAG);
		code_time+=code_pulse_time+(tts-tto);
		AddPulse(code_width-code_time, OPENFLAG);
		
		return(code_width);
		
	}
	
	else {		//add a pulse with code_width
	
		AddPulse(code_width+(tts-tto), CLOSEFLAG);
		
		return(code_width+(tts-tto));
		
	}
	
}

void AddCodeCycle(unsigned short bit_code1, unsigned short bit_code2)	//adds a code cycle to the mudpulse array
{
	
	unsigned cycle_time=0;
	unsigned total_cycle_time=0;
	
	if(bit_code1) {
	
		total_cycle_time+=W_Pulse;
		cycle_time+=AddCodePulse(W_Pulse);	//CLOSE for wide pulse
		
	}
		
	else {
	
		total_cycle_time+=N_Pulse;
		cycle_time+=AddCodePulse(N_Pulse);	//CLOSE for narrow pulse 
		
	}
		
	if(bit_code2)
	
		total_cycle_time+=W_Pulse;	
		
	else
	
		total_cycle_time+=N_Pulse;
		
	AddPulse(total_cycle_time-cycle_time, OPENFLAG);	//OPEN for wide/narrow pulse
		
}

void AddSync()		//function to add sync to the frame sequence
{
	
	int i;
	unsigned short bit_sync1;
	unsigned short bit_sync2;
	
	for (i=0;i<syncLEN;i++) {
		
		bit_sync1=(syncNW&(0x0001<<(syncLEN-i-1)))>>(syncLEN-i-1);
		bit_sync2=(syncPN&(0x0001<<(syncLEN-i-1)))>>(syncLEN-i-1);
		
		if (bit_sync1) {
			
			if (bit_sync2)
			
				AddCodePulse(W_Pulse);
				
			else
			
				AddPulse(W_Pulse, OPENFLAG);
				
		}
		
		else {
			
			if (bit_sync2)
			
				AddCodePulse(N_Pulse);
				
			else
			
				AddPulse(N_Pulse, OPENFLAG);
				
		}
		
	}
	
}
		
				
#define	FRAME_BUFFER_LENGTH 32
char 	FrameBuffer[FRAME_BUFFER_LENGTH] = {0}; //data for frame subchannel
short 	FrameBufferStore = 0;	//index to store new FrameBuffer item
short 	FrameBufferFetch = 0;	//index to fetch next FrameBuffer item
short 	FrameChecksum = 0;	//Frame Checksum accumulator

void AddCode(unsigned code1, unsigned code2, unsigned short code_len)
{
	int i;
	unsigned short bit_code1;
	unsigned short bit_code2;
	
	for (i=0;i<code_len;i++) {
		
		bit_code1=(code1&(0x0001<<(code_len-i-1)))>>(code_len-i-1);
		bit_code2=(code2&(0x0001<<(code_len-i-1)))>>(code_len-i-1);
		
		AddCodeCycle(bit_code1, bit_code2);
		
	}
	
}

void FormatMudPulses() // Encode more data into the mudpulse[] buffer
{
	
	unsigned code1;
	unsigned code2;
	
	if((FrameBufferFetch == 0)&(FrameBufferFetch<FrameBufferStore)) { //a new frame has been formated
	
		AddSync();
	
		while(FrameBufferFetch<FrameBufferStore) {
			
			code1=code[FrameBuffer[FrameBufferFetch++]];
			code2=code[FrameBuffer[FrameBufferFetch++]];
			
			AddCode(code1, code2, CODE_LEN);
			
		}
	
    }
   
}

#define H_RES		1		//high resolution
#define L_RES		0		//low resolution

void AddData(unsigned long data, short digits, short resolution)
// add data to FrameBuffer one nibble at a time
// digits 0 to 9
{
	unsigned long temp_data=data;
	unsigned frame_data=0;
	
	if(!resolution) {	//low resolution
	
		temp_data=temp_data/100;		//down grade resolution by 100
		
	}
	
	switch(digits) //store nibble to FrameBuffer & increment index for each nibble
	{

		case 3:
		
			frame_data=temp_data/100000;
			FrameBuffer[FrameBufferStore++]=frame_data;
			temp_data=temp_data-frame_data*100000;
			
			frame_data=temp_data/10000;
			FrameBuffer[FrameBufferStore++]=frame_data;
			temp_data=temp_data-frame_data*10000;
			
		case 2:
			
			frame_data=temp_data/1000;		
			FrameBuffer[FrameBufferStore++]=frame_data;
			temp_data=temp_data-frame_data*1000;
			
			frame_data=temp_data/100;
			FrameBuffer[FrameBufferStore++]=frame_data;
			temp_data=temp_data-frame_data*100;
			
		case 1:
		
			frame_data=temp_data/10;
			FrameBuffer[FrameBufferStore++]=frame_data;
			temp_data=temp_data-frame_data*10;
			FrameBuffer[FrameBufferStore++]=temp_data;
			
	}
		//printf("FrameBuffer = %d FrameBufferStoreFrame = %d Checksum=%d\n",FrameBuffer[FrameBufferStore-1],FrameBufferStore,FrameChecksum);
}

#define StatID		0
#define AziID		1
#define IncID		2
#define GzID		3
#define HzID		4
#define HGxGyID		5
#define HHxHyID		6
#define TempID		7
#define PresID		8
#define	HTF_ID		9
#define HGR_ID		10
#define LGxGyID		11
#define LHxHyID		12
#define	LTF_ID		13
#define LGR_ID		14
#define TFGR_ID		15

void FormatData(short FrameID, unsigned *data) //formats data to FrameBuffer
{
	FrameBufferStore = 0;	//reset the FrameBufferStore to add fresh data
	FrameBufferFetch = 0;	//reset the FrameBufferFetch to first data item
	FrameChecksum = 0;		//reset the checksum for fresh data
	
	data_seq[SEQ_FRAME_ID]=FrameID;
	
	AddData(data_seq[SEQ_FRAME_ID], 1, H_RES);	//FrameID to the FrameBuffer
	
	switch(FrameID) //select the frame to format
	{
		case StatID: 		AddData(*(data+SEQ_DIAGNOSIC), 2, H_RES); 
							break;

		case AziID:			AddData(*(data+SEQ_AZI), 2, H_RES) ; //azimuth
 							break;
 							
 		case IncID:			AddData(*(data+SEQ_INC), 2, H_RES) ; //inclination
 							break;
 							
 		case GzID: 			AddData(*(data+SEQ_GZ), 3, H_RES) ;	//gz
							break;
							
		case HzID: 			AddData(*(data+SEQ_HZ), 3, H_RES) ;	//hz
							break;
		
		case HGxGyID: 		AddData(*(data+SEQ_GX), 3, H_RES) ;	//high res. gx
							AddData(*(data+SEQ_GY), 3, H_RES) ;	//high res. gy
 							break;
		
		case HHxHyID: 		AddData(*(data+SEQ_HX), 3, H_RES) ;	//high res. hx
							AddData(*(data+SEQ_HY), 3, H_RES) ;	//high res. hy
 							break;
				
		case TempID: 		AddData(*(data+SEQ_TEMP), 2, H_RES) ; //temperature
							break;
							
		case PresID: 		AddData(*(data+SEQ_PRES), 2, H_RES) ; //pressure
							break;
							
		case HTF_ID: 		AddData(*(data+SEQ_TF), 2, H_RES) ; //high res. toolface
							break;
							
		case HGR_ID: 		AddData(*(data+SEQ_GR), 2, H_RES) ; //high res. gamma ray
							break;
							
		case LGxGyID: 		AddData(*(data+SEQ_GX), 2, L_RES) ;	//low res. gx
							AddData(*(data+SEQ_GY), 2, L_RES) ;	//low res. gy
 							break;
		
		case LHxHyID: 		AddData(*(data+SEQ_HX), 2, L_RES) ;	//low res. hx
							AddData(*(data+SEQ_HY), 2, L_RES) ;	//low res. hy
							break;
							
		case LTF_ID: 		AddData(*(data+SEQ_TF), 1, L_RES) ; 	//low res. toolface
							break;
							
		case LGR_ID: 		AddData(*(data+SEQ_GR), 1, L_RES) ; 	//low res. gamma ray
							break;
							
		case TFGR_ID: 		AddData(*(data+SEQ_TF), 1, L_RES) ; 	//low res. toolface
							AddData(*(data+SEQ_GR), 1, L_RES) ; 	//low res. gamma ray
							break;
							
		default:			break;
				
	}
	
}

void StartNewFrame(unsigned FrameNumber, unsigned *data)	// round robbin frame sequencer for starters
{
//static  short   FrameNumber = 0;
    *(data+SEQ_FRAME_ID) = FrameNumber; 
	FormatData(FrameNumber, data);	//formats data to FrameBuffer
	//FrameNumber++;				//round robbin frame sequence
	//if(FrameNumber > HIGHEST_DATA_FRAME) FrameNumber = 0; //reset Frame sequence
}


extern unsigned pattern[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE];
extern unsigned pattern_count[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE];
extern unsigned pattern_idx[NUM_PATTERN_SEQ];
extern unsigned pattern_counter[NUM_PATTERN_SEQ];
extern unsigned pattern_seq[PATTERN_SEQ_BUF_SIZE];
extern unsigned long pattern_seq_rate[PATTERN_SEQ_BUF_SIZE];
extern unsigned ini_pattern_seq_idx;
extern unsigned pattern_seq_idx;

//Test pulses routine
void TestPulses()
{
	
	#define TEST_SHUT_TIME		5000
	#define	TEST_OPEN_TIME		5000
	#define TEST_PULSE_NUM		10
	#define TEST_DECREMENT_STEP	500
	
	int i, j;
	
	for(i=0;i < TEST_PULSE_NUM;i++) {
		
		AddPulse(TEST_SHUT_TIME-i*TEST_DECREMENT_STEP, CLOSEFLAG);	//CLOSE for SHUT_TIME
		
		AddPulse(TEST_OPEN_TIME-i*TEST_DECREMENT_STEP, OPENFLAG);	//OPEN for OPEN_TIME
		
		if (i==TEST_PULSE_NUM-1) {
			
			for(j=0;j<TEST_PULSE_NUM;j++) { 
			
				AddPulse(TEST_SHUT_TIME-i*TEST_DECREMENT_STEP, CLOSEFLAG);	//CLOSE for SHUT_TIME
				
				if (j==TEST_PULSE_NUM-1)
					
					AddPulse(TEST_OPEN_TIME, OPENFLAG);	//OPEN for OPEN_TIME
					
				else
				
					AddPulse(TEST_OPEN_TIME-i*TEST_DECREMENT_STEP, OPENFLAG);	//OPEN for OPEN_TIME
					
			}
			
		}
		
	}
	
}

#define INC_STEP			0.25
//#define MAX_INC				2.0
unsigned max_inc;
//simple pulse scheme based on inclination
void SimplePulses(float data_inc)
{
	
	#define SIMPLE_CYCLE_TIME	2000
	//#define SIMPLE_GAP_TIME		5000
	#define SIMPLE_TOTAL_TIME	60000
	
	//#define SIMPLE_PULSE_NUM	(((float)MAX_INC-inc)/(float)INC_STEP)+13
	#define SIMPLE_PULSE_NUM	(21.0-(data_inc/(float)INC_STEP))  
	#define MAX_PULSE_NUM		25
	
	int i;
	ulong simple_total_time=0;
			
	if(data_inc<(float)max_inc) {
			
		for (i=0;i<(unsigned)SIMPLE_PULSE_NUM;i++) {
				
			AddCycle((ulong)SIMPLE_CYCLE_TIME);
			simple_total_time+=(ulong)SIMPLE_CYCLE_TIME;
				
		}
			
	}
			
	else {
			
		for (i=0;i<MAX_PULSE_NUM;i++) {
				
			AddCycle((ulong)SIMPLE_CYCLE_TIME);
			simple_total_time+=(ulong)SIMPLE_CYCLE_TIME;
				
		}
			
	}
	
	AddPulse((ulong)((ulong)SIMPLE_TOTAL_TIME-simple_total_time), OPENFLAG);
	
}

//frequency modulated pulse scheme based on inclination
void FreqPulses(float data_inc)
{
	
	#define FREQ_GAP_TIME	500
	#define FREQ_GAP_NUM		(data_inc/(float)INC_STEP+3.0)
	#define FREQ_CYCLE_TIME		((ulong)pulse_time+(ulong)FREQ_GAP_TIME*(ulong)FREQ_GAP_NUM)
	#define FREQ_MIN_CYCLE_TIME		(pulse_time+2*FREQ_GAP_TIME)
	
	int i;
			
	if(data_inc<(float)max_inc) {
		
		AddCycle((ulong)FREQ_CYCLE_TIME);
			
	}
			
	else {
							
		AddCycle((ulong)FREQ_MIN_CYCLE_TIME);
			
	}
	
}


/*******decode pulse patterns for Kevin Diao*************/
/*
//decoder pulses for Kevin Diao
unsigned pulse_width=1000;

//Decoding test pulses
void DecodePulses(unsigned pulse_width, int pattern_num)
{
	int gap_num=pattern_num;
	
	#define DECODE_SHUT_TIME	pulse_width
	#define DECODE_OPEN_TIME	((gap_num+2)*pulse_width)
	
	if (pattern_num==2) {
		
		gap_num=3;
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		
	}
	
	if (pattern_num==3) {
		
		gap_num=1;
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		
		gap_num=3;
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		
	}
	
	else {
		
		AddPulse((ulong)DECODE_SHUT_TIME, CLOSEFLAG);
		AddPulse((ulong)DECODE_OPEN_TIME, OPENFLAG);
		
	}	
	
}
*/

unsigned code_seq=0;

//a full code sequence
void CodeSeqPulses()
{
	short i;
	
	AddSync();
	
	for (i=0;i<NUM_OF_CODE;i++) {
		
		AddCode(code[code_seq], code[i], CODE_LEN);
		
	}
	
	code_seq++;
	if (code_seq>=NUM_OF_CODE) code_seq=0;
	
}

#define RUN_TIME		2000
unsigned long stop_time;

//Run scheme
void RunPulses()
{
	
	AddCycle((ulong)RUN_TIME);
	
}
	
//stop scheme
void StopPulses()
{
	//#define STOP_TIME1		30000
	//#define STOP_TIME2		28000
	
	AddPulse((ulong)(stop_time>>1), OPENFLAG);
	RunPulses();
	AddPulse((ulong)(stop_time-(stop_time>>1)-(ulong)RUN_TIME), OPENFLAG);
	
}

//open and shut position of the motor were kept as a nominator according to the GEAR_DENOMINATOR
//This keeps the non-integer gear ratio so that the open and shut position were consistent
long motor_open_position	= 0;	//position to open the flow loop
long motor_neutral_position = 0;	//position that motor is neutral to open				
long motor_shut_position	= 0;	//position to shut the flow loop
long motor_target = 0;		//desired position of the motor
							//motor_target were kept the same unit as motor_position, different from open and shut position														
							
bool Motor_Cycle;		//motor cycling flag

//cyclying between the run and stop mode
void MotorCycle()
{
	
	if(Motor_Cycle) {
		
		if(TimerExpired(MOTOR_CYCLE_TIMER)) {
			
			pattern_seq_idx++;
			if((pattern_seq_idx>=PATTERN_SEQ_BUF_SIZE)|(pattern_seq[pattern_seq_idx]==0x0FFFF)) pattern_seq_idx=0;
			
			SetTimer(MOTOR_CYCLE_TIMER, pattern_seq_rate[pattern_seq_idx]);
	
			mudpulses=0;
			mudfetchindex=mudstoreindex;
			FrameBufferFetch=FrameBufferStore;
			if(motor_target!=motor_open_position) motor_target=motor_open_position;
			
		}
		
	}
	
	else pattern_seq_idx=ini_pattern_seq_idx;		//0 always refer to the primary pattern sequence
	
}

//set certain motor running pattern
void Pattern2Frame()
{
	
	FreezeData();
	
	MotorCycle();
		
	if(mudpulses<=8) {		//mudpulse buffer is almost empty
			
		if(FrameBufferFetch >= FrameBufferStore) {		//fram buffer is all fetched
		
			if(Tool_Rotate|(!TimerExpired(RT_DLY_TIMER))) {
			
				switch (pattern[pattern_seq[pattern_seq_idx]][pattern_idx[pattern_seq[pattern_seq_idx]]]) {
							
					//live frame data
					case PC_REGULAR_FRAME0:		//0: send regular frame 0
						
					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(0, data_qt_seq);
							
					 	}

					 	else StartNewFrame(0,data_seq);
			
						break;
				
					case PC_REGULAR_FRAME1:		//1: send regular frame 1

					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(1, data_qt_seq);
							
					 	}

						break;
				
					case PC_REGULAR_FRAME2:		//2: send regular frame 2

					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(2, data_qt_seq);
							
					 	}
					 	
					 	else StartNewFrame(2,data_seq);

						break;
						
					case PC_REGULAR_FRAME3:		//3: send regular frame 3

					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(3, data_qt_seq);
							
					 	}

						break;
						
					case PC_REGULAR_FRAME4:		//4: send regular frame 4
						
					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(4, data_qt_seq);
							
					 	}
					
						break;
					
					case PC_REGULAR_FRAME5:		//5: send regular frame 5

					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(5, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME6:		//6: send regular frame 6

					 	if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(6, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME7:		//7: send regular frame 7
									
						StartNewFrame(7,data_seq);
					
						break;
					
					case PC_REGULAR_FRAME8:		//8: send regular frame 8
									
						StartNewFrame(8,data_seq);
					
						break;
					
					case PC_REGULAR_FRAME9:		//9: send regular frame 9
					
						if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(9, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME10:		//A: send regular frame 10
									
						StartNewFrame(10,data_seq);
					
						break;
					
					case PC_REGULAR_FRAME11:		//B: send regular frame 11
								
						if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(11, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME12:		//C: send regular frame 12
								
						if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(12, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME13:		//D: send regular frame 13
									
						if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(13, data_qt_seq);
							
					 	}

						break;
					
					case PC_REGULAR_FRAME14:		//E: send regular frame 14
									
						StartNewFrame(14,data_seq);
					
						break;
					
					case PC_REGULAR_FRAME15:		//F: send regular frame 15
									
						if(!TimerExpired(DATA_FRESH_TIMER)) {

							StartNewFrame(15, data_qt_seq);
							
					 	}

						break;
				
					//	static frame data
					case PC_STATIC_FRAME0:		//10: send static frame 0
									
						StartNewFrame(0,data_static);
					
						break;
					
					case PC_STATIC_FRAME1:		//11: send static frame 1
									
						StartNewFrame(1,data_static);
					
						break;
					
					case PC_STATIC_FRAME2:		//12: send static frame 2
									
						StartNewFrame(2,data_static);
					
						break;
					
					case PC_STATIC_FRAME3:		//13: send static frame 3
									
						StartNewFrame(3,data_static);
					
						break;
					
					case PC_STATIC_FRAME4:		//14: send static frame 4
									
						StartNewFrame(4,data_static);
						
						break;
					
					case PC_STATIC_FRAME5:		//15: send static frame 5
									
						StartNewFrame(5,data_static);
						
						break;
					
					case PC_STATIC_FRAME6:		//16: send static frame 6
									
						StartNewFrame(6,data_static);
						
						break;
					
					case PC_STATIC_FRAME7:		//17: send static frame 7
									
						StartNewFrame(7,data_static);
						
						break;
					
					case PC_STATIC_FRAME8:		//18: send static frame 8
									
						StartNewFrame(8,data_static);
						
						break;
					
					case PC_STATIC_FRAME9:		//19: send static frame 9
									
						StartNewFrame(9,data_static);
						
						break;
					
					case PC_STATIC_FRAME10:		//1A: send static frame 10
									
						StartNewFrame(10,data_static);
						
						break;
					
					case PC_STATIC_FRAME11:		//1B: send static frame 11
									
						StartNewFrame(11,data_static);
						
						break;
					
					case PC_STATIC_FRAME12:		//1C: send static frame 12
									
						StartNewFrame(12,data_static);
						
						break;
					
					case PC_STATIC_FRAME13:		//1D: send static frame 13
									
						StartNewFrame(13,data_static);
						
						break;
					
					case PC_STATIC_FRAME14:		//1E: send static frame 14
									
						StartNewFrame(14,data_static);
						
						break;
					
					case PC_STATIC_FRAME15:		//1F: send static frame 15
									
						StartNewFrame(15,data_static);
						
						break;
					
					
					// specific pulse scheme
					case PC_TEST_PULSE:			//20: test pulses
					
						TestPulses();
					
						break;
					
					case PC_SIMPLE_PULSE:		//21:simple pulse scheme
				
						if(!TimerExpired(DATA_FRESH_TIMER)) {
					
							SimplePulses((float)data_qt_seq[SEQ_INC]*180.0/10000.0);
							
						}
						
						else SimplePulses((float)data_seq[SEQ_INC]*180.0/10000.0);
					
						break;
					
					case PC_FREQ_PULSE:			//22: frequency pulse scheme
				
						if(!TimerExpired(DATA_FRESH_TIMER)) {
					
							FreqPulses((float)data_qt_seq[SEQ_INC]*180.0/10000.0);
							
						}
						
						else FreqPulses((float)data_seq[SEQ_INC]*180.0/10000.0);
					
						break;
					
					/*
					
					case PC_DECODE_PULSE0:		//11: test decode pulse pattern 0
				
						DecodePulses(pulse_width, 0);
				
						break;
					
					case PC_DECODE_PULSE1:		//12: test decode pulse pattern 1
				
						DecodePulses(pulse_width, 1);
				
						break;
				
					case PC_DECODE_PULSE2:		//13: test decode pulse pattern 2
				
						DecodePulses(pulse_width, 2);
				
						break;
				
					case PC_DECODE_PULSE3:		//14: test decode pulse pattern 3
				
						DecodePulses(pulse_width, 3);
				
						break;
					
					*/
					
					case PC_CODE_PULSE:		//23: send a certain chirp
					
						AddCode(code[4], code[5], CODE_LEN);
					
						break;
				
					case PC_CODE_SEQ:			//24: send full code sequence
					
						CodeSeqPulses();
					
						break;
					
					// simple stop or run scheme
					case PC_RUN:				//25: conituously running
						
						RunPulses();
					
						break;
					
					case PC_STOP:				//26: stop pulsing
				
						StopPulses();
				
						break;
				
					default:					//skip to the next pattern
		    
						pattern_counter[pattern_seq[pattern_seq_idx]]=0;
					
						break;
						
				}
				
			}
			
			else {
				
				if(mudpulses==0) StopPulses();
				
			}
			
			if(pattern_counter[pattern_seq[pattern_seq_idx]]) pattern_counter[pattern_seq[pattern_seq_idx]]--;
		
			if(pattern_counter[pattern_seq[pattern_seq_idx]]<=0) {		//pattern counter expired
			
				pattern_idx[pattern_seq[pattern_seq_idx]]++;
				
				/*
				//kevin's pattern
				if(pattern[pattern_idx]==0x0FFFF) {
				
					if(pulse_width==1000) pulse_width=500;
					else pulse_width=1000;
					
				}
				*/
				
				//if the end of the pattern buffer, wrap the buffer						
				if((pattern_idx[pattern_seq[pattern_seq_idx]]>=PATTERN_BUF_SIZE)|(pattern[pattern_seq[pattern_seq_idx]][pattern_idx[pattern_seq[pattern_seq_idx]]]==0x0FFFF)) pattern_idx[pattern_seq[pattern_seq_idx]]=0;					
			
				pattern_counter[pattern_seq[pattern_seq_idx]]=pattern_count[pattern_seq[pattern_seq_idx]][pattern_idx[pattern_seq[pattern_seq_idx]]];
			
			}
			
		}
		
		else FormatMudPulses();
		
	}
	
	if(Sensor_Live) Send_Array(data_seq, NUM_OF_SEQ);
	
}
		
/*
//push the mudpulse buffer if necessary
void Data2Frame() 
{
	
	FreezeData();		//always freeze data into the data_seq so that it can be logged into the memory

	if(mudpulses<=8) {
		
		if(Pulse_Test) TestPulses();
		
		else if(Pulse_Simple) SimplePulses();
		else if(Pulse_Freq)	  FreqPulses();
			
		else RegularPulses();
		
	}
	
	if (Sensor_Live) Send_Array(data_seq, NUM_OF_SEQ);		//send live sensor data through RS232
		
}
*/

/*
void Data2Frame() 
// saves data to FLASH once for each frame
// FLASH save overlaps the end of the previous frame since it takes some time
// the mudpulse buffer is refilled if necessary.
{
	//Send_Int((long)GRBin);
	//Send_CRLF();
	
	if(Pulse_Test) {		//use test pulses
	
		if(mudpulses <= 8) TestPulses();
		
	}
	
	else {
		
		FreezeData();
		
		if(FrameBufferFetch >= FrameBufferStore)	//FrameBuffer is empty
		{	//FreezeData();
#if __SIMULATION__
	printf("in Data2Frame StartNewFrame\n");
#endif
			StartNewFrame();
			//SaveDataSeq();
			
			if (Sensor_Frame) Send_Array(data_seq, NUM_OF_SEQ);		//send framed sensor data through RS232
			
		}
		//if the mudpulse buffer is nearing empty format more pulses
		if(mudpulses <= 8) FormatMudPulses();
		
		if (Sensor_Live) Send_Array(data_seq, NUM_OF_SEQ);		//send live sensor data through RS232
	
	}
		
}
*/

//-----------------------------------------------------------------------------
// SERVICE OUTPUT ROUTINES
//
// Description:
//	This function is called by a soft timer interrupt.
//	Outputs the mudpulse[] array a pulse at a time to the motor position controller.
//
// Operation:
//	The routine is initiated by the timer interrupt.
//	Pulse data is read from the mudpulse[] array one pulse at a time 
//	and used to actuate the mud pulser,
//	via the motor positon conroller.
//	Pulse buffer contains packed binary.
//		MSB = CLOSE		MSB-1 = OPEN
//		14 LSBs are timer setting
//	array has data to transmit when mudpulses > 0
//	
//----------------------------------------------------------------------------------
#define POLL_mudpulse 100	//time to wait between polling mudpulse[] buffer for new pulses	

int motor_shut_offset;			//offset of the shut position
int motor_open_offset;			//offset of the open position

void ServOutput()
{	
	
	static unsigned long pulse = 0; //pulse to output from mudpulse[] used as local variable only

	// if the buffer has data then actuate the motor	
	if(mudpulses > 0) //there is data in the buffer
	{ 
		pulse = PullMudpulse() ; //read the next pulse
		SetTimer(MUDPULSE_TIMER,(pulse & DATABITS));	//set timer for the pulse
	   	if((pulse & OPENFLAG) != 0) {			//command motor to OPEN if OPENFLAG is set
	   			
	   		if(!Motor_Capture) {
	   				
	   			motor_target=motor_open_position;
	   				   			   				
	   		}
	   	}
    	else { 
    			
    		if((pulse & CLOSEFLAG) != 0) { 	//command motor to CLOSE if CLOSEFLAG is set
    		
    			if(!Motor_Capture) {
    					
    				motor_target=motor_shut_position;
    		
    			}	
    		}
    	}	
    }
    // else the buffer is empty so just set the timer to poll later	
	else	//change to send sync pulses is buffer is empty 
		SetTimer(MUDPULSE_TIMER,POLL_mudpulse);	//check for a new pulses in POLL_mudpulse ms
			
}							

// ----------------------------------------------------------------------------------------
// Motor position controller
// This code is here because it is in the interrupt handler called most often
// Later it can be moved to its own routine and called some other way.
// Motor position is controlled using an error feedback algorithm
// When there is a difference in the motor_position and the motor_target
// the direction is controlled to move the motor toward the motor_target
// and the speed is set depending on how far the motor must move.

//#define MotorHome	2	//motor is cose enough to home to STOP - to reduce "hunting"
#define PWM_MIN		2	//lowest PWM value allowed
#define PWM_MAX 	15	//highest PWM value allowed
#define MOTOR_CONTROL_RATE	2	//rate to run Motor_controller function in milliseconds
//#define REVERSE_RATE	0.8		//rate to reverse the motor to brake

#define ENCODER_SCALE_FACTOR	1

unsigned MotorHome;

unsigned Speed_Dn;	//# of counts before MotorHome to start slowing down

int	motor_position=0; 	//physical position of the motor

int previous_position=0;	//previous motor position

int reverse = 0;	//reserve flag

int quad = 0;		//value of a quadratic signal
//int previous_quad = 0;	//previous quadratic signal, used to determine if the position has changed
    
unsigned short PWM_setting = 0; 		//PWM ito output for motor speed
unsigned short pre_PWM_setting = 0;		//previous PWM_setting
unsigned short PWM_calib;		//PWM during calibration
unsigned short ini_PWM_calib;	//initial calibration PWM

/***************** caliberation code ******************/

#define MOTOR_CALIB_RATE	10		//rate to increment the step of caliberation in msec
#define MOTOR_WAIT_RATE		5000	//rate to wait to resolve the stalls in msec
#define MOTOR_RESTART_RATE	100		//rate to check if the restart position is reached 
#define MOTOR_VEL_RATE		100			//time rate to check the velocity of the motor in msec

#define MOTOR_STALL		1		//number of counts of position change within MOTOR_STALL_RATE
#define MIN_STALL_COUNT		4.0		//minimal stall count for PWM_MAX
#define MAX_STALL_COUNT		10.0		//maximal stall count for PWM_MIN

ushort stall_counter = MIN_STALL_COUNT;		//counter to determine the stall, 0 when stall detected

#define MotorStalled() (!stall_counter)		//if the stall_counter is zero

//#define CALIB_OPEN_OFFSET	COUNTS_PER_TURN/4		//counts to offset the detected open position
//#define CALIB_SHUT_OFFSET	COUNTS_PER_TURN*3/4		//counts to offset the detected open position

//#define CALIB_TURNS			4		//number of turns to caliberate

//#define START_TURN				10		//number of turns to run the motor before caliberation
									//set such that the initial stop position does not affect the caliberation
bool CalibDONE = false;
bool RestartMOTOR = false;

unsigned velocity = 0;	//velocity of the motor indicated by the motor position change

//int abs(int f);

//routine to calculate the current velocity of the motor and find the minimal velocity
//also check if the motor jam by "stall_counter"
void Motor_Velocity()
{
	
	SetTimer(MOTOR_VEL_TIMER,MOTOR_VEL_RATE);  //set timer to check velocity again
	
	if (MotorStalled()) {
		
		MotorSTOP();
		motor_target=motor_position;
		Motor_Capture=true;		//capture motor so that no more mudpulses can be sent
		RestartMOTOR=true;
		
	}
	
	else {
		
		if ((PWM_setting!=0)&(velocity<=MOTOR_STALL))		//check if the motor jammed
		
			stall_counter--;
			
		else
		
			stall_counter=MAX_STALL_COUNT-(MAX_STALL_COUNT-MIN_STALL_COUNT)*((float)PWM_setting-(float)PWM_MIN)/((float)PWM_MAX-(float)PWM_MIN);		//reset the stall_counter to initial value defined by INI_STALL_COUNTER
			
	}
	
	velocity=0;
	
}


//#define RESTART_STEP		30		//position distance to move the motor to resolve the stall
#define RESTART_COUNT		20		//# of steps to move the motor
unsigned restart_step;		//12		//position distance to move the motor to resolve the stall
ushort PWM_restart=0;		//motor restart PWM
ushort ini_PWM_restart;	//initial motor restart PWM
ushort restart_counter;
#define ExpireRestartCounter()		(!restart_counter)
unsigned short motor_freeze_count;
unsigned short motor_freeze_counter;
#define ExpireMotorFreezeCounter()	(!motor_freeze_counter)

unsigned long motor_freeze_rate;

//void Motor_Rty_Calib();
//void Motor_Lin_Calib();
void Motor_Opt_Calib();

//Routine to wait for a certain amount of time
void Motor_Wait()
{
	
	SetTimer(MOTOR_WAIT_TIMER,MOTOR_WAIT_RATE);
		

	while (!TimerExpired(MOTOR_WAIT_TIMER)) {
			
		asm("nop;");
			
	}
		
}

//routine to resolve stall conditions by slightly turning the motor back and forward to losen the jam
void Motor_Restart()
{
	
	int dir=1;
	
	if(MotorStalled()) {
			
		Motor_Wait();
		
		stall_counter=MIN_STALL_COUNT;
		restart_counter = RESTART_COUNT;
		
		while ((!MotorStalled())&(!ExpireRestartCounter())) {
				
			if (TimerExpired(MOTOR_RESTART_TIMER)) {
					
				SetTimer(MOTOR_RESTART_TIMER,MOTOR_RESTART_RATE);
					
				if (PWM_setting==0) {
			
					dir=(-1)*dir;
					motor_target=motor_target+(long)dir*(long)restart_step;
					restart_counter--;
						
				}
					
			}
				
			else asm("nop;");
			
		}
		
		if(motor_freeze_counter) motor_freeze_counter--;
		
		if(ExpireMotorFreezeCounter()) {
			
			if(!Motor_Freeze) {
				
				PWM_setting=0;
				stall_counter=MIN_STALL_COUNT;
				Motor_Freeze=true;
				RestartMOTOR=false;
				Motor_Capture=false;
				
			}
			
		}
		
		else {
			
			if(TimerExpired(MOTOR_FREEZE_TIMER)) {
				
				SetTimer(MOTOR_FREEZE_TIMER, motor_freeze_rate);
				motor_freeze_counter=motor_freeze_count;
				
			}
		
			if(!MotorStalled()) {
		
				RestartMOTOR=false;
		
				//if(!CalibDONE) Motor_Rty_Calib();
				//if(!CalibDONE) Motor_Lin_Calib();
				if(!CalibDONE) Motor_Opt_Calib();
				else Motor_Capture=false;
			
				PWM_restart=ini_PWM_restart;
		
			}
		
			else {
		
				PWM_restart++;
				if(PWM_restart==PWM_MAX+1) PWM_restart=ini_PWM_restart;
			
			}
			
		}
				
	}
	
}
			
//ushort max_turn=CALIB_TURNS;		//maximal out-gear full turns to caliberate the motor

unsigned load_counter = 0;			//load counter to display velocity for a full turn
unsigned load_buf[LOAD_BUF_SIZE];		//load buffer to store the motor load
//unsigned acc_load_buf[MAX_LOAD_BUF_SIZE];	//accumulated load buffer
unsigned load_pos_buf[LOAD_BUF_SIZE];		//load position buffer to store motor position corresponding to the motor load
//unsigned acc_load_pos_buf[MAX_LOAD_BUF_SIZE];	//accumulated load position buffer

unsigned load_in_idx = 0;			//load buffer input pointer
unsigned load_out_idx = 0;			//load buffer output pointer

unsigned open_load_calib;		//the target calibration load to reach
unsigned shut_load_calib;
unsigned ref_load_calib;		//reference load for calibration

unsigned skip_count;		//initial skip count for calibation
int ini_calib_dir=1;			//calibration motor running direction

int WriteFlash(short ctrl, unsigned data_size, unsigned long flash_addr, unsigned *data);

//routine to calibrate the linear cup in a optimal way
void Motor_Opt_Calib()
{
	
	CalibDONE=false;
	
	int dir=ini_calib_dir;
	unsigned max_load=0;
	unsigned min_load=0xFFFF;
	int max_load_pos=0;
	int calib_pos[2]={0x8000, 0x7FFF,};		//calibration position
	unsigned load_calib[2]={shut_load_calib, open_load_calib};
		
	#define CALIB_COUNT		11		//2 times the actual turns to calibrate count to calibrate the position
	unsigned calib_counter=CALIB_COUNT;
	#define ExpireCalibCounter()		(!calib_counter)
	
	unsigned skip_counter=skip_count;
	#define ExpireSkipCounter()		(!skip_counter)
	
	motor_target=(long)motor_position;
			
	//start regular calibration	
	while ((!ExpireCalibCounter())&(!MotorStalled())) {
		
		motor_target=motor_target+(long)dir*((long)COUNTS_PER_TURN+(long)skip_count);
		load_out_idx=load_in_idx;
		
		max_load=0;
		max_load_pos=0;
		
		skip_counter=skip_count;
		
		while((max_load<load_calib[min(1-dir,1)])&(!MotorStalled())&(labs(motor_target-(long)motor_position)>(long)MotorHome)) {
			
			if(TimerExpired(GP_TIMER0)) {
			
				SetTimer(GP_TIMER0,MOTOR_CALIB_RATE);
				
				while ((load_out_idx!=load_in_idx)&(max_load<load_calib[min(1-dir,1)])) {
					
					if(skip_counter) skip_counter--;
					
					if(ExpireSkipCounter()) {
					
						if(load_buf[load_out_idx]>=max_load) {
						
						 	max_load=load_buf[load_out_idx];
						 	max_load_pos=load_pos_buf[load_out_idx];
						
						}
						
						if(load_buf[load_out_idx]<=min_load) {
							
							min_load=load_buf[load_out_idx];
							
						}
					
					}
					
					load_out_idx++;
					if(load_out_idx>=LOAD_BUF_SIZE) load_out_idx=0;
					
				}
				
			}
			
		}
		
		motor_target=(long)motor_position;
		MotorSTOP();
		
		if(calib_counter!=CALIB_COUNT) {	//skip the first turn
		
			if((long)(calib_pos[min(1-dir,1)]*dir)<(long)(max_load_pos*dir)) {
		
				calib_pos[min(1-dir,1)]=max_load_pos;
				
			}
				
			Send_Array(calib_pos, 2);
			
		}
					
		dir=dir*(-1);
		
		if(calib_counter) calib_counter--;
		
		if((ExpireCalibCounter())&(!MotorStalled())&(PWM_calib<=PWM_MAX)) {
			
			if(min_load>ref_load_calib) {
				
				calib_counter=CALIB_COUNT;
				PWM_calib=min(PWM_MAX, PWM_calib+1);
				calib_pos[0]=0x8000;
				calib_pos[1]=0x7FFF;
				min_load=0xFFFF;
				
			}
			
		}		
		
	}
	
	if(!MotorStalled()) {
	
		//motor_shut_position=(long)calib_pos[0]+(long)motor_shut_offset;
		//motor_open_position=(long)calib_pos[1]+(long)motor_open_offset;
		
		//write the calibration record to the flash for future diagnostic purpose
		WriteFlash(DM_FULL_WORD, sizeof(load_buf), FLASH_LOAD_ADDR, load_buf);
		WriteFlash(DM_FULL_WORD, sizeof(load_pos_buf), FLASH_POS_ADDR, load_pos_buf);
		
		motor_target=(long)calib_pos[1];
		
		Motor_Wait();
		
		if(!MotorStalled()) {
		
			PWM_calib=0;
		
			MotorHome=(unsigned)COUNTS_PER_TURN;
		
			Motor_Wait();
			
			motor_open_position=(long)calib_pos[1]+(long)motor_open_offset;
			motor_neutral_position=(long)motor_position;
			motor_shut_position=motor_neutral_position+(long)(COUNTS_PER_TURN/2)+(long)motor_shut_offset;
			
			MotorHome=status[SC_MOTORHOME];
			PWM_calib=ini_PWM_calib;
	
			CalibDONE=true;
	
			motor_target=motor_open_position;
	
			//write the calibration record to the flash for future diagnostic purpose
			//WriteFlash(DM_FULL_WORD, sizeof(load_buf), FLASH_LOAD_ADDR, load_buf);
			//WriteFlash(DM_FULL_WORD, sizeof(load_pos_buf), FLASH_POS_ADDR, load_pos_buf);
		
			Motor_Capture=false;
			
		}
		
	}
	
	else {
	
		PWM_calib++;
		if(PWM_calib==PWM_MAX+1) PWM_calib=ini_PWM_calib;
		ini_calib_dir=ini_calib_dir*(-1);
		
	}
		
	Send_Calib_Status();
	
}


/*
//routine to calibrate the linear cup
void Motor_Lin_Calib()
{
	
	CalibDONE=false;
	
	int dir=ini_calib_dir;
	unsigned max_load=0;
	unsigned min_load=0xFFFF;
	int max_load_pos=0;
	int calib_pos[2]={0, 0};		//calibration position
	unsigned load_calib[2]={shut_load_calib, open_load_calib};
		
	#define CALIB_COUNT		11		//2 times the actual turns to calibrate count to calibrate the position
	unsigned calib_counter=CALIB_COUNT;
	#define ExpireCalibCounter()		(!calib_counter)
	
	unsigned skip_counter=skip_count;
	#define ExpireSkipCounter()		(!skip_counter)
		
	//start regular calibration	
	while ((!ExpireCalibCounter())&(!MotorStalled())) {
		
		motor_target=(long)motor_position+(long)dir*(long)COUNTS_PER_TURN;
		load_out_idx=load_in_idx;
		
		max_load=0;
		max_load_pos=0;
		
		skip_counter=skip_count;
		
		while((max_load<load_calib[min(1-dir,1)])&(!MotorStalled())&(labs(motor_target-(long)motor_position)>(long)MotorHome)) {
			
			if(TimerExpired(GP_TIMER0)) {
			
				SetTimer(GP_TIMER0,MOTOR_CALIB_RATE);
				
				while ((load_out_idx!=load_in_idx)&(max_load<load_calib[min(1-dir,1)])) {
					
					if(skip_counter) skip_counter--;
					
					if(ExpireSkipCounter()) {
					
						if(load_buf[load_out_idx]>=max_load) {
						
						 	max_load=load_buf[load_out_idx];
						 	max_load_pos=load_pos_buf[load_out_idx];
						
						}
						
						if(load_buf[load_out_idx]<=min_load) {
							
							min_load=load_buf[load_out_idx];
							
						}
					
					}
					
					load_out_idx++;
					if(load_out_idx>=LOAD_BUF_SIZE) load_out_idx=0;
					
				}
				
			}
			
		}
		
		motor_target=(long)motor_position;
		MotorSTOP();
		
		if(calib_counter!=CALIB_COUNT) {	//skip the first turn
				
			if((calib_pos[min(1-dir,1)]*dir)<(max_load_pos*dir)) {
		
				calib_pos[min(1-dir,1)]=max_load_pos;
				
			}
			
		}
					
		dir=dir*(-1);
		
		if(calib_counter) calib_counter--;
		
		if((ExpireCalibCounter())&(!MotorStalled())&(PWM_calib<=PWM_MAX)) {
			
			if(min_load>ref_load_calib) {
				
				calib_counter=CALIB_COUNT;
				PWM_calib=min(PWM_MAX, PWM_calib+1);
				calib_pos[0]=0;
				calib_pos[1]=0;
				min_load=0xFFFF;
				
			}
			
		}		
		
	}
	
	if(!MotorStalled()) {
	
		motor_shut_position=(long)calib_pos[0]+(long)motor_shut_offset;
		motor_open_position=(long)calib_pos[1]+(long)motor_open_offset;
	
		CalibDONE=true;
	
		motor_target=motor_open_position;
		PWM_calib=ini_PWM_calib;
	
		//write the calibration record to the flash for future diagnostic purpose
		WriteFlash(DM_FULL_WORD, sizeof(load_buf), FLASH_LOAD_ADDR, load_buf);
		
		Motor_Capture=false;
		
	}
	
	else {
	
		PWM_calib++;
		if(PWM_calib==PWM_MAX+1) PWM_calib=ini_PWM_calib;
		ini_calib_dir=ini_calib_dir*(-1);
		
	}
		
	Send_Calib_Status();
	
}
*/		

/*
//routine to caliberate rotary set-up of motor
//calibration has 2 steps.
//Step 1: run motor from the highest speed to the target speed
//Step 2: calibrate the open and shut position
void Motor_Rty_Calib()
{
	
	CalibDONE=false;
	
	PWM_calib = ini_PWM_calib;
	
	#define INI_MAX_AVG_LOAD		0		//initial maximal load of the motor	
	#define	INI_MIN_AVG_LOAD		255		//initial minimal load of the motor

	#define LOAD_DIFF_TARGET	50		//initial difference target between the maximal and minimal load
	
	#define MAX_CALIB_TURN	(max_turn+START_TURN+3)		//maximal turns for caliberation
		
	int i=0;
	
	int calib_pos = 0;		//calibrated position
	
	unsigned max_avg_load=0;		//maximal motor load
	unsigned min_avg_load=0;		//minimal motor load
		
	unsigned acc_load_buf[MAX_LOAD_BUF_SIZE];	//accumulated load buffer
	unsigned acc_load_pos_buf[MAX_LOAD_BUF_SIZE];	//accumulated load position buffer
	
	unsigned start_counter;
	unsigned accl_counter;
	
	#define ExpireStartCounter()		(!start_counter)
	#define ExpireAcclCounter()			(!accl_counter)
		
	//Step 1: run motor from highest speed to target speed for calibration
	while ((max_avg_load-min_avg_load<LOAD_DIFF_TARGET)&(!MotorStalled())&(PWM_calib>=PWM_MIN)) {
		
		calib_pos = 0;
		max_avg_load=INI_MAX_AVG_LOAD;
		min_avg_load=INI_MIN_AVG_LOAD;
			
		start_counter=(load_buf_size*START_TURN);
		accl_counter=(load_buf_size*max_turn);
		
		for (i=0;i<MAX_LOAD_BUF_SIZE;i++) {		//clear all load buffers
		
			acc_load_buf[i]=0;
			acc_load_pos_buf[i]=0;
		
		}
		
		motor_target=(long)motor_position+(long)MAX_CALIB_TURN*(long)COUNTS_PER_TURN/(long)(long)gear_denominator;		//set motor target > max_turn so that motor does not stop in the middle	
	
		load_out_idx=load_in_idx;
	
		while ((!ExpireAcclCounter())&(!MotorStalled())) {		//first 2 turn just to start motor to avoid return of maximal load
		
			if(TimerExpired(GP_TIMER0)) {
			
				SetTimer(GP_TIMER0,MOTOR_CALIB_RATE);
			
				while ((load_out_idx!=load_in_idx)&(!ExpireAcclCounter())) {		//read load from buffer
			
					if(start_counter)	start_counter--;
			
					if (ExpireStartCounter()) {		//skip the first two turns
					
						if(accl_counter) accl_counter--;
						
						acc_load_buf[load_out_idx]=acc_load_buf[load_out_idx]+load_buf[load_out_idx];		//accumulate load at 265 positions of a full turn
						acc_load_pos_buf[load_out_idx]=load_pos_buf[load_out_idx];
					
					}
				
					load_out_idx++;
					if(load_out_idx>=load_buf_size)	load_out_idx=0;
				
				}
			
			}
		
			else asm("nop;");
		
		}
		
		if(!MotorStalled()) {		//set new target only when motor is not stalled
	
			motor_target=motor_position;
			MotorSTOP();	//stop the motor when caliberation done
					
			for (i=0;i<load_buf_size-1;i++) {		//calulate the average load from loads of adjacent two points
		
				acc_load_buf[i]=(acc_load_buf[i]+acc_load_buf[i+1])/max_turn;
			
				//find the maximal load position to calibrate the motor open position
				if (acc_load_buf[i]>=max_avg_load) {
			
					max_avg_load=acc_load_buf[i];
					calib_pos=acc_load_pos_buf[i+1];
			
				}
			
				//find the minimal load position
				if (acc_load_buf[i]<=min_avg_load) {
			
					min_avg_load=acc_load_buf[i];
			
				}
		
			}
		
			PWM_calib--;
			
		}
		
	}
	
	
	//Step 2: calibrate the open and shut position
	if(((max_avg_load-min_avg_load>=LOAD_DIFF_TARGET)|(PWM_calib<PWM_MIN))&(!MotorStalled())) {
		
		//find the open position based on the calib_shut_position and max_turn
		//motor_open_position=(long)COUNTS_PER_TURN+(long)calib_pos*(long)GEAR_DENOMINATOR+(long)CALIB_OPEN_OFFSET;
		//motor_shut_position=(long)COUNTS_PER_TURN+(long)calib_pos*(long)GEAR_DENOMINATOR+(long)CALIB_SHUT_OFFSET;
		//motor_target=motor_open_position/(long)GEAR_DENOMINATOR;
		
		motor_open_position=(long)COUNTS_PER_TURN+(long)calib_pos*(long)gear_denominator+(long)CALIB_OPEN_OFFSET;
		motor_shut_position=(long)COUNTS_PER_TURN+(long)calib_pos*(long)gear_denominator+(long)CALIB_SHUT_OFFSET;
		motor_target=motor_open_position/(long)gear_denominator;
		PWM_setting=0;
		
		CalibDONE=true;
		
		//write the calibration record to the flash for future diagnostic purpose
		WriteFlash(DM_FULL_WORD, sizeof(acc_load_buf), FLASH_LOAD_ADDR, acc_load_buf);
		
	}
	
}
*/


/***************** motor position updating code ******************/

#define POSITIVE_WRAP	32752				//7ff0 positive number to wrap
#define	NEGATIVE_WRAP	-32753				//800f negative number to wrap

//wrap the motor position and
//update the open and shut position in the timer
//open and shut position were kept updated all the time
void PositionUpdate()
{
	
	//wrap the motor_position if needed
	//motor_position resets to 0 and all position related variables resets accordingly
	if((motor_position>POSITIVE_WRAP)|(motor_position<NEGATIVE_WRAP)) {
	
		motor_target=motor_target-(long)motor_position;
		previous_position=previous_position-motor_position;
		motor_open_position=motor_open_position-(long)motor_position;
		motor_shut_position=motor_shut_position-(long)motor_position;
		motor_position=0;
		
	}
	
	/*
	//update the open and shut position in the timer
	if(CalibDONE) {		//caliberation provides the initial open and shut position
	
		if (motor_target>(motor_shut_position/(long)gear_denominator)) {
		                                                   
			motor_shut_position=motor_shut_position+(long)COUNTS_PER_TURN;
			
		}
	
		if (motor_target>(motor_open_position/(long)gear_denominator)) {
		
			motor_open_position=motor_open_position+(long)COUNTS_PER_TURN;
		
		}
		
		if (motor_target<=(motor_shut_position-(long)COUNTS_PER_TURN)/(long)gear_denominator) {
			
			motor_shut_position=motor_shut_position-(long)COUNTS_PER_TURN;
			
		}
		
		if (motor_target<=(motor_open_position-(long)COUNTS_PER_TURN)/(long)gear_denominator) {
			
			motor_open_position=motor_open_position-(long)COUNTS_PER_TURN;
			
		}

		
	}
	*/
	
}


/********************* Motor Control code *******************/


void MotorCCW_control()		//control motor to run counter clock wise
{
	if (reverse==0)	{
		
		MotorCCW();
		MotorRUN();
		
	}
		
	else {
		
		MotorCW();
		MotorRUN();
		
	}
		
}

void MotorCW_control()		//control motor to run clock wise
{
	if (reverse==0)	{
		
		MotorCW();
		MotorRUN();
		
	}
		
	else {
		
		MotorCCW();
		MotorRUN();
		
	}
		
}			


/************** Acceleration variables ****************/	
#define UP_OPEN_STEP	1		//# of PWM increase allowed for each time interrupt, toward open
#define UP_SHUT_STEP	1		//# of PWM increase allowed for each time interrupt, toward shut

unsigned up_open_step = UP_OPEN_STEP;
unsigned up_shut_step = UP_SHUT_STEP;

ushort up_open_PWM_count;
ushort up_open_PWM_counter;
#define ExpireUpOpenPWMCounter()	(!up_open_PWM_counter)

ushort up_shut_PWM_count;
ushort up_shut_PWM_counter;
#define ExpireUpShutPWMCounter()	(!up_shut_PWM_counter)

ushort up_open_max_PWM;
ushort up_shut_max_PWM;

ushort ini_up_open_PWM;
ushort ini_up_shut_PWM;

ushort up_open_PWM;
ushort up_shut_PWM;

/************** Deacceleration variables ****************/	
#define DN_OPEN_STEP	1		//# of PWM decrease allowed for each time interrupt, toward open
#define DN_SHUT_STEP	1		//# of PWM decrease allowed for each time interrupt, toward shut

unsigned dn_open_step = DN_OPEN_STEP;
unsigned dn_shut_step = DN_SHUT_STEP;

ushort dn_open_PWM_count;
ushort dn_open_PWM_counter;
#define ExpireDnOpenPWMCounter()	(!dn_open_PWM_counter)

ushort dn_shut_PWM_count;
ushort dn_shut_PWM_counter;
#define ExpireDnShutPWMCounter()	(!dn_shut_PWM_counter)

ushort dn_open_min_PWM;
ushort dn_shut_min_PWM;


void PWM_Up()		//routine to speed up the PWM according to the current PWM allowance
{
	
	if (motor_target==motor_open_position) {		//motor is running toward open position
	
		if(up_open_PWM_counter) up_open_PWM_counter--;
		
		if (ExpireUpOpenPWMCounter()) {
			
			up_open_PWM=min(up_open_max_PWM, up_open_PWM+up_open_step);
			PWM_setting=up_open_PWM;
			up_open_PWM_counter = up_open_PWM_count;
			
		}
		
		up_shut_PWM=ini_up_shut_PWM;
		
	}
		
	else {		//motor is running toward shut position
	
		if(up_shut_PWM_counter) up_shut_PWM_counter--;
		
		if (ExpireUpShutPWMCounter()) {
			
			up_shut_PWM=min(up_shut_max_PWM, up_shut_PWM+up_shut_step);
			PWM_setting=up_shut_PWM;
			up_shut_PWM_counter = up_shut_PWM_count;
			
		}
		
		up_open_PWM=ini_up_open_PWM;
		
	}
	
	dn_open_PWM_counter=dn_open_PWM_count;
	dn_shut_PWM_counter=dn_shut_PWM_count;
		
}

void PWM_Dn()		//routine to slow down the PWM according to the current PWM allowance
{
	
	if (motor_target==motor_open_position) {		//motor is running toward open position
	
		if(dn_open_PWM_counter) dn_open_PWM_counter--;
		
		if (ExpireDnOpenPWMCounter()) {
			
			PWM_setting=max((int)dn_open_min_PWM, (int)pre_PWM_setting-(int)dn_open_step);
			dn_open_PWM_counter = dn_open_PWM_count;
			
		}
		
	}
		
	else {		//motor is running toward shut position
	
		if(dn_shut_PWM_counter) dn_shut_PWM_counter--;
		
		if (ExpireDnShutPWMCounter()) {
			
			PWM_setting=max((int)dn_shut_min_PWM, (int)pre_PWM_setting-(int)dn_shut_step);
			dn_shut_PWM_counter = dn_shut_PWM_count;
			
		}
		
	}
	
	up_open_PWM_counter=up_open_PWM_count;
	up_shut_PWM_counter=up_shut_PWM_count;
	
	up_open_PWM=ini_up_open_PWM;
	up_shut_PWM=ini_up_shut_PWM;
		
}

void Motor_controller()	// run the motor controller
{	

//ClrFlagBit(2);
	//set the next time to run the Motor_controller function
	SetTimer(MOTOR_CONTROL_TIMER, MOTOR_CONTROL_RATE); 
	//calculate the position error
	//position_error = motor_target - (motor_position/ENCODER_SCALE_FACTOR); 
   	//set the motor speed scaled to the position_error
   	//limit to PWM_MAX and PWM_MIN
    //PWM_setting = max(PWM_MIN, (min( (abs(position_error) * gain),PWM_MAX) ) );
    
    //setting the speed of the motor
    
    if(Motor_Freeze) {		//release the motor if battery low
    	    	
    	//MotorSTOP();
    	PWM_setting=0;
    	MotorRUN();
    	
    }
    
    else {
    	
    	if(motor_target!=motor_open_position) MotorHome=status[SC_MOTORHOME];

    	if (labs(motor_target-(long)motor_position)<=(long)MotorHome) {		//motor reach motor home, stop motor
    
     		if(motor_target==motor_open_position) {
     			
     			MotorHome=abs(motor_neutral_position-motor_open_position);
     			
     		}
     			
     			
    		MotorSTOP();
    		//reverse=0;
    		PWM_setting=0;
    		
   		}
   
    	else {
    	
    		if ((!CalibDONE)&(!RestartMOTOR)) PWM_setting=PWM_calib; 		//motor speed when caliberating
   
			else if (RestartMOTOR) PWM_setting=PWM_restart;		//motor in restarting mode, running at full speed
		
			else {

        		if (labs(motor_target-(long)motor_position)<((long)Speed_Dn+(long)MotorHome)) {		//motor need to slow down to reach target
   			
   			 		//PWM_setting=(PWM_MAX-PWM_MIN)*labs((labs(motor_target-motor_position)-MotorHome))/SpeedDn+PWM_MIN;
   			 		//PWM_setting=PWM_MAX-(PWM_MAX-PWM_STOP)*(SpeedDn-labs(motor_position-motor_target))/(SpeedDn-(MotorHome));
    		 		//if the distance between the position and target is smaller than SpeedDn
    			 	//slow down linearly till reach the minimal speed.
    		 
    		 		/*if (PWM_setting<PWM_MAX*REVERSE_RATE) {
    	
    					if ((previous_position-motor_position)*(motor_position-motor_target)>0)
    		
    						reverse=1;
    	
    		 			else reverse=0;
    		
    		 		}*/
    		 	
    		 		PWM_Dn();
    		
   		 		}
   		 	
   				else {
   				
   					//reverse=0;
   					PWM_Up();		//if the distance between the position and target
    								//is between SpeedUp and SpeedDn, try running at maximal speed
    									
   				}
   				
			}
    	 		
    	
    		if	((long)motor_position<motor_target)	MotorCW_control();		//motor runs clockwise toward target
    	
    		else MotorCCW_control();		//motor runs counter clockwise toward target
    		
    	}
    	
    }
    
    previous_position=motor_position;
    pre_PWM_setting=PWM_setting;
}



/****************************** Flash memory programs ********************************/


//Routine to write data[] into flash from the address indicated by flash_addr

unsigned data_buf[DATA_BUF_SIZE];		//data buffer to store the logging data to be saved to flash

unsigned data_counter = 0;		//count of data that can be saved to flash
unsigned long data_save_addr;	//address of data to be saved into flash memory

//unsigned long data_save_rate;

int WriteFlash(short ctrl, unsigned data_size, unsigned long flash_addr, unsigned *data)
{

	int flash_status=1;
	
	unsigned long curr_addr = flash_addr;
        // Current Flash Address that the data is going to be written
    unsigned long  sector_addr;
        // Current Sector address writing too.
    unsigned scratch[FLASH_SECTOR_1M];
        // Scratch buffer
    unsigned  idx = 0;
        // index into the data[] array

    // While we still have more data to save to flash
    while(idx < data_size)
    {   
    	// Read in the sector starting at the sector address
        sector_addr = curr_addr&0xFFFFFF00;
        
        if(sector_addr<=MAX_SECTOR_SIZE) {
        	
        	ReadFlash(DM_FULL_WORD,sizeof(scratch),sector_addr,scratch);
                
        	while(idx < data_size)
        	{   // Store value into scratch buffer
            	scratch[(curr_addr&0xFF)>>1] = *(data+idx);
            	idx++;
            	// Increment address and check if it crosses a sector boundary
            	curr_addr += 2; // Increment by 2 because of the ushort
            	if((curr_addr&0xFF) == 0)
                	break;  // Crosses a boundary so break and send data to flash
        	}

        	// Save Data to flash
        	ProgFlash(ctrl,sizeof(scratch),sector_addr,scratch);
        	
        }
        
        else {
        	
        	flash_status=0;
        	break;
        	
        }
    }
    
    return(flash_status);

}

void EraseData()		//routine to erase the log data from the memory replace them by FFFF
{
	
	int i;
	int flash_status=1;
	unsigned temp_data[FLASH_SECTOR_1M];
	unsigned long temp_addr=FLASH_LOG_ADDR;
	
	char message[13]={'F','l','a','s','h',' ','e','r','a','s','e','d','!'};
	
	for (i=0;i<FLASH_SECTOR_1M;i++) temp_data[i]=0x0ffff;
	
	while(flash_status) {
		
		flash_status=WriteFlash(DM_FULL_WORD,sizeof(temp_data),temp_addr,temp_data);
		
		temp_addr=temp_addr+FLASH_SECTOR_1M*2;
		
	}
	
	data_save_addr=FLASH_LOG_ADDR;
	
	
	Send_Message(13,message);
	Send_Value(data_save_addr);
	
}

void FindAddr()		//find the initial address to save the logging data
{
	
	int i;
	unsigned temp_data[FLASH_SECTOR_1M];
	unsigned flag_counter=0;
	
	data_save_addr = FLASH_LOG_ADDR;
	
	while (data_save_addr<MAX_FLASH_SIZE) {		//address not found or memory not full
	
		if (flag_counter<DATA_SIZE) {		//at least the memory is enough to write one more reocrd
		
			ReadFlash(DM_FULL_WORD,sizeof(temp_data),data_save_addr,temp_data);
		
			for (i=0;i<FLASH_SECTOR_1M;i++) {
			
				if(temp_data[i]==0x0FFFF) flag_counter++;
				else flag_counter=0;
			
			}
			
			data_save_addr+=FLASH_SECTOR_1M*2;
			
		}
		
		else {
			
			data_save_addr-=flag_counter*2;
			data_save_addr-=FLASH_LOG_ADDR;
			data_save_addr=ceil((data_save_addr>>1)/DATA_SIZE);
			data_save_addr=(data_save_addr<<1)*DATA_SIZE+FLASH_LOG_ADDR;
			//make the address to be integrate multiple of the log data length
			//pay attention to the type tranferring between variables
			
			break;
			
		}
		
	}
	
	Send_Value(data_save_addr);
		
}

//-----------------------------------------------------------------------------
// SAVE DATA
//
// Description:
//      This routine saves the sequence data to flash
//
// Notes:
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------

void SaveData()			//rountine to save the logging data into the flash memory on a timely basis
{
		
	//char message[11]={'F','l','a','s','h',' ','f','u','l','l','!'};
	
	unsigned data_save_size=0;
	int i;
		
	if(data_save_addr<MAX_FLASH_SIZE) {
		
		//FreezeLogData();
		
		for (i=0;i<NUM_OF_LOG;i++) {		//store all sensor data
			
			data_buf[data_save_size++]=data_log[i];
			
		}
		
		for (i=0;i<gr_log_size;i++) {		//store all sensor data
			
			data_buf[data_save_size++]=gr_log[i];
			
		}
		
		/*	
		for (i=0;i<GRBIN_BUF_SIZE;i++) {	//store the gamma ray spectrum data
		
			data_buf[data_save_size++]=GRBin_CPS[i];			
		
		}
		*/
	
		WriteFlash(DM_FULL_WORD,data_save_size,(ulong)data_save_addr,data_buf);
		
		data_save_addr=data_save_addr+data_save_size*2;
		
	}
	
	//else Send_Message(11,message);
	
}


/*void SaveData()
{
	static unsigned long curr_addr = FLASH_DATA_ADDR;
        // Current Flash Address that the data is going to be written
    unsigned long  sector_addr;
        // Current Sector address writing too.
    unsigned short scratch[128];
        // Scratch buffer
    unsigned char  idx = 0;
        // index into the data_seq array

    // While we still have more data to send in the seq buffer
    while(idx < NUM_OF_SEQ)
    {   // Read in the sector starting at the sector address
        sector_addr = curr_addr&0xFFFFFF00;
        ReadFlash(DM_FULL_WORD,sizeof(scratch),sector_addr,scratch);
        
        while(idx < NUM_OF_SEQ)
        {   // Store value into scratch buffer
            scratch[(curr_addr&0xFF)>>1] = data_seq[idx++];

            // Increment address and check if it crosses a sector boundary
            curr_addr += 2; // Increment by 2 because of the ushort
            if((curr_addr&0xFF) == 0)
                break;  // Crosses a boundary so break and send data to flash
        }

        // Save Data to flash
        ProgFlash(DM_FULL_WORD,sizeof(scratch),sector_addr,scratch);
    }

    // check if we reached end of flash
    // If reached end of flash than loop around and start over writing data.
    if(curr_addr > MAX_FLASH_SIZE)
        curr_addr = FLASH_DATA_ADDR;
}*/
	

