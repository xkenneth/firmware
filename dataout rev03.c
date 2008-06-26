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
//					gaps accordingly.
//					Allow PULL time to be shorter than the one second pulse.
//					Move logic for PULL, Dwell, and merging multiple pulses in same direction
//					from the interrupt handler to the modulator code. 
//					
//
// The prototype solenoid H bridge driver is slow to turn off so I added a little
// software delay. see rev 014 1 ms.
// the production board should be faster and not require the delay
//
// Description:
//      This file outputs the sequence of data a bit at a time
//      on the Flag1 and Flag2 pins.  It also saves the data to flash with
//      a time stamp.
//-----------------------------------------------------------------------------
#include	"stddef.h"
#include	"Backgrd.h"
#include	"dataout.h"
#include	"timer.h"
#include    "flash.h"
#include    "sysdef.h"
 
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
ACCESS_TIMERS;
extern 	float 	G,H,azi,inc,Temperature;	//calculated values
//definitions to convert floating point values to useful integers
//in the range of expected values 
#define azimuth 	(int)(azi*10.0) //expected value 
#define inclination (int)(inc*10.0)	//expected value 
#define G1000 		(int)(G*1000.0)	//expected value 1.000
#define H1000 		(int)(H*1000.0)	//expected value 1.000
extern	float 	G,H,azi,inc,Temperature;	//these seem to be globally accessed
extern	short	GammaRayCPS;	//the Gamma Ray in counts per minute
extern	short	ToolFace;		//calculated ToolFace in 0.1 degree per bit
extern	short	Pressure;		//calculated pressure
extern	short	avg1,avg2,avg3,avg4,avg5,avg6;
extern  short   avg7,avg8,avg9,avg10,avg11;     /* Averages from BoardCheckout1.asm */
ushort seq_count = 0;   // Counts how many times a sequence was saved to flash

//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------
#define MAX_MUDPULSES		128 //maximum number of mud pulses per data frame.
short mudpulse[MAX_MUDPULSES] ={0};	//transmit array of mud pulse times in milliseconds
short mudfetchindex = 0;  	//index to the pulse being transmitted
short mudstoreindex = 0;  	//index to the pulse being stored into the array
	//array has data to transmit when mudstoreindex > mudfetchindex
	//array is ready for new data when mudstoreindex <= mudfetchindex


//short PMsolenoid = 1;		//0=not Permanent magnet   1=Permanent Magnet

//-----------------------------------------------------------------------------
// Timer Rates
//-----------------------------------------------------------------------------
#define PULL_TIME_MIN		 100	// (msec) Length of pulser drive signal minimum
#define PULL_TIME_DEFAULT	1000 	// (msec) Length of pulser drive signal maximum
short PULL_TIME = PULL_TIME_DEFAULT ; // (msec) Length of pulser drive signal

  //#define MOD48_UNIT_TIME		 33	// (msec) Time length for the MOD48 modulation
	//was 33. this actually measured about 32 time units long.
	//not sure why the time is wrong. 
	//maybe just the timer setting always ends up one tick short
	//maybe the DSP clock rate is off a little

//#define TIMER_BIT_RATE      978 // (msec) Time length of each bit output - old
//#define TIMER_BIT_RATE_MAX  978 // (msec) Max Time length of each bit output
//#define TIMER_BIT_RATE_MIN  160 // (msec) Min Time length of each bit output

#define SAVE_FLASH_TIME    60000    // (msec) Rate at which data gets saved to flash
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
// Flag drive macros
//-----------------------------------------------------------------------------    
// These macros should be changed to fit the specific hardware.
// 
// SetData should set the Data value high.
// ClrData should set the Data value low.
// --------------------------------------------------------------
//#define SetData()       asm(" SET FL1;")
//#define ClrData()       asm(" RESET FL1;")
 
// CloseON FL1 drives the iron solenoid core to close, 
// FL2 is not used for iron solenoid, but does not hurt to drive the FL2 line.
//  nop; is to reduce cross over current in H bridge.
#define CloseON()       asm(" RESET FL2; nop; SET FL1;")
#define CloseOFF()      asm(" RESET FL1;")
#define OpenON()        asm(" RESET FL1; nop; SET FL2;")
#define OpenOFF()       asm(" RESET FL2;")
#define AllOFF()        asm(" RESET FL1; RESET FL2;")

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
#define SEQ_SYNC_WORD   6
#define SEQ_FRAME_ID    7
#define SEQ_INC         8
#define SEQ_AZI         9
#define SEQ_G           10
#define SEQ_H           11
#define SEQ_GR          12
#define SEQ_TEMP        13
#define SEQ_END_FRAME   14
#define NUM_OF_SEQ      15

//-----------------------------------------------------------------------------
// Constants
//      eventually we will add a time stamp which will increase the
//      number of bytes saved to flash. It is multiplied by two because
//      the data_seq array is a ushort.
//-----------------------------------------------------------------------------
#define FLASH_BLOCK_SIZE    NUM_OF_SEQ*2  // Number of bytes saved to flash in one block
#define MAX_FLASH_SIZE      (0x80000-FLASH_BLOCK_SIZE)  // Size of flash to last block able to store

// Data array !this array must match the array below!
static ushort data_seq[NUM_OF_SEQ] = {
    0,      // Month
    0,      // Day
    0,      // Year
    0,      // Hour
    0,      // Minute
    0,      // Second
    0x55,   // Sync Word
    0x20,   // Frame ID
    0,      // inclination
    0,      // azimuth
    0,      // G1000
    0,      // H1000
    0,      // GR (GammaRayCPS)
    0,      // Temperature
    0x000f  // end frame
};

// Bit sizes of above data !this array must match the array above!
static ushort data_seq_size[NUM_OF_SEQ] = {
    8,      // Month
    8,      // Day
    8,      // Year
    8,      // Hour
    8,      // Minute
    8,      // Second
    8,      // Sync Word 
    8,      // Frame ID
    10,     // inclination
    12,     // azimuth
    8,     // G1000
    8,     // H1000
    8,      // GR (GammaRayCPS)
    8,      // Temperature
    4       // end frame
};

//-----------------------------------------------------------------------------
// Local Functions
//-----------------------------------------------------------------------------
    
//-----------------------------------------------------------------------------
// SERVICE OUTPUT ROUTINES
//
// Description:
//	This function is called by a timer interrupt.
//	It drives the direct H bridge solenoid driver board and Permenant magnet pulser
//	Outputs the mudpulse[] array a pulse at a time on the flag pins FL1 and FL2.
//	Indexes through mudpulse[] array using mudfetchindex.
//
// Operation:
//	The routine is initiated by the timer interrupt. It tests for MUDPULSE_TIMER expired.
//	Pulse data is read from the mudpulse[] array one at a time and used to actuate the mud pulser.
//	Pulse buffer contains packed binary.
//		MSB is FLAG1 = CLOSE	MSB-1 is FLAG2 = OPEN
//		14 LSBs are timer setting
	//array has data to transmit when mudstoreindex > mudfetchindex
	//array is ready for new data when mudstoreindex <= mudfetchindex
//	
//----------------------------------------------------------------------------------
#define POLL_mudpulse 100	//time to wait between polling mudpulse[] buffer for new pulses	
#define CLOSEFLAG	0x8000	//flag bit to CLOSE the pulser valve sending a pressure pulse
#define OPENFLAG	0x4000	//flag bit to OPEN the pulser valve sending a pressure pulse
#define DATABITS	0x3FFF	//flag bit to OPEN the pulser valve sending a pressure pulse
						
int motor_target = 0; //desired physical position for the motor
int pulse = 0; //pulse to output from mudpulse[] used as local variable only
void ServOutput()
{
    {
	    // if the buffer is empty just set the timer to poll later	
	    AllOFF();		//can turn off both outputs between pulses
		
	    if(mudstoreindex > mudfetchindex) //there is data in the buffer
	    { 
	    	pulse = mudpulse[mudfetchindex++] ; //read the next pulse and increment index
			SetTimer(MUDPULSE_TIMER,(pulse & DATABITS));	//set timer for the pulse
	    	if((pulse & OPENFLAG) != 0) OpenON();				//PULL OPEN if OPENFLAG is set
	    	else { if((pulse & CLOSEFLAG) != 0) CloseON(); }	//PULL CLOSE if CLOSEFLAG is set
	    }
	    //set timer to poll later	
		else 
			SetTimer(MUDPULSE_TIMER,POLL_mudpulse);	//check for a new pulses in POLL_mudpulse ms
    }
}



//-----------------------------------------------------------------------------
// FORMAT MUD PULSES ROUTINES
//
// Description:
//	Formats data into the mud pulse array.
//	array is ready for new data when mudstoreindex <= mudfetchindex
//	Pulse buffer contains packed binary.
//		MSB is FLAG1 = CLOSE	MSB-1 is FLAG2 = OPEN
//		14 LSBs are timer setting
//
//	Pulse Chirp encoding is designed for minimum pressure drop in mud pulse telemetry.
//	Detailed documentation is in a seperate document.
//	Short pulses are used to keep the mud valve open as much as possible.
//	The pulses are all the same width, ie the minimum time for a reliable pulse.
//	Data is encoded into the gaps between the pulses.
//	Two methods of data encoding are used simultaneously to send fast and slow data rates.
//	Fast data is not as accurate, but is sent as often as possible.
//	For an MWD tool the fast data is Gamma Ray counts and tool face.
//	Gamma Ray (GR) is encoded as analog time using a log scale compression.
//	Tool Face (TF) is encoded as analog time with linear scale.
//	Time is marked with groups of pulses called chirps.
//	Chirps have specific numbers of pulses and increasing or decreasing gaps.
//	Time gaps are specific patterns to avoid harmonics and increase signal to noise ratio.
//	Time calibration is included in the protocol.
//	Slow data is sent in binary as a subchannel.
//	Specific chirps identify the data items which will be sent.
//	Chirp gap patterns represent the binary data.
//
//	Starting a new data frame freezes the subchannel data into local variables
//	This time stamps the data at the frame start time.
//	Starting with an empty buffer, pulse edges are added to the buffer.
//	The first chirp T0 identifies the frame data and start time.
//	Fast data (GR & TF) are fresh as possible, represented by analog time of chirps.
//	GR is a time value marked by the second chirp.
//	TF is a time value marked by the third chirp.
//	The TF chirp is also the time marker for the next GR & TF, T0n+1
//
//	Slow rate binary Data is encoded using various chirps with differing gaps.
//	Binary data is added four bits at a time with each chirp.
//	The last chirp is an xor checksum of the data.
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
#define PULSE_TIME	PULL_TIME_DEFAULT    	// (msec) valve is closed a fixed time
#define GAP_MIN		PULL_TIME_DEFAULT		//minimum gap
#define CYCLE_MIN	PULSE_TIME + GAP_MIN	//min cycle time for reference
#define CYCLE_STEP	1.059463				//one chromatic half step
//#define CYCLE_STEP	1.122462				//one chromatic full step
#define NUM_GAPS	10		// base values for pulse gaps
#define GAP0	((GAP_MIN          ) * CYCLE_MIN) - PULSE_TIME  
#define GAP1	((PULSE_TIME + GAP0) * CYCLE_MIN) - PULSE_TIME //one chromatic full step
#define GAP2	((PULSE_TIME + GAP1) * CYCLE_MIN) - PULSE_TIME
#define GAP3	((PULSE_TIME + GAP2) * CYCLE_MIN) - PULSE_TIME
#define GAP4	((PULSE_TIME + GAP3) * CYCLE_MIN) - PULSE_TIME
#define GAP5	((PULSE_TIME + GAP4) * CYCLE_MIN) - PULSE_TIME
#define GAP6	((PULSE_TIME + GAP5) * CYCLE_MIN) - PULSE_TIME
#define GAP7	((PULSE_TIME + GAP6) * CYCLE_MIN) - PULSE_TIME
#define GAP8	((PULSE_TIME + GAP7) * CYCLE_MIN) - PULSE_TIME
#define GAP9	((PULSE_TIME + GAP8) * CYCLE_MIN) - PULSE_TIME

const long GAP[NUM_GAPS] = 
{
//	gaps required to produce cycles of chromatic full steps
    GAP0,  
    GAP1,
    GAP2,
    GAP3,
    GAP4,
    GAP5,
    GAP6,
    GAP7,
    GAP8,
    GAP9 
};

#define SPEED_STEP_INTERVAL 7 // SPEED_STEP_INTERVAL

#define	FRAME_BUFFER_LENGTH 16
char 	FrameBuffer[FRAME_BUFFER_LENGTH] = {0}; //data for frame subchannel
short 	FrameBufferStore = 0;	//index to store new FrameBuffer item
short 	FrameBufferFetch = 0;	//index to fetch next FrameBuffer item
short 	FrameChecksum = 0;	//Frame Checksum accumulator
short 	ToolStatus = 0xa5;		//future ToolStatus 8 bits

#define T0 			0		//time zero for GammaRay ToolFace reference
#define T_GR_0 		24000	//GR zero
#define T_GR_MAX 	33000	//GR MAX
#define T_GR_range	(T_GR_MAX - T_GR_0)			//GR range
#define T_TF_range	8000			//3600 is 0.1 degree resolution
#define T_TF_0	 	53000	//TF MID
#define T_TF_MIN 	(T_TF_0 - (T_TF_range/2))	//TF MIN
#define T_TF_MAX 	(T_TF_0 + (T_TF_range/2))	//TF MAX
long Tdata = T0;		//holds elapsed time from T0
short LastDrivePulse = 0; // fill with the drive flag when filling mudpulse buffer

long CalcLogGR(short GR)
#define CPS_MAX 1000	//Limit GR counts to 1000 CPS
{
float x,y;	
	//scale 0-7000 ms 0-1000 range T_GR_0
	//ratio of GR to full scale, convert to log, scale to GR milliseconds
	x = max(GR,1);
	x = log(x);
	y = CPS_MAX;
	y = log(y);
	x = x / y;	
		//printf("log(GR)ratio = %f\n",x);
	x = (T_GR_range * x) + T_GR_0;	
		//printf("GR time = %f\n",x);
	return( (long)x );	
}

void AddPulse(short msTime, unsigned short DriveFLAG)
	//time value in milliseconds for a pulse
	//Store the time into the mud pulse array
	//increment the mud pulse array store index to the next element
{	
	if(msTime <= PULL_TIME)  //use one pulse for PULL and second for HOLD
		mudpulse[mudstoreindex++] = (msTime | DriveFLAG) ;
	else //msTime is > PULL_TIME so two pulses are needed
	{	mudpulse[mudstoreindex++] = (PULL_TIME | DriveFLAG) ; //the PULL pulse
		mudpulse[mudstoreindex++] = (msTime - PULL_TIME) ; //the hold portion
	}
	LastDrivePulse = DriveFLAG;
	Tdata = Tdata + msTime; //elapsed time since T0
		//printf("mudpulse = %d Tdata=%d\n",mudpulse[mudstoreindex-1],Tdata);
}

void OPENUntilTime(long msTime)
	//OPEN valve until specific time
	//sends an OPEN pulse which ends at a specific time referenced from T0
	//NOTE attempting to set time earlier than Tdata does nothing
{
	if(msTime > Tdata)  //the time adjustment is positive
	{	// check the last PULL pulse to see if it was an OPEN pulse
		// because there is no need to OPEN again
		if (LastDrivePulse == CLOSEFLAG) 
			AddPulse((msTime - Tdata), OPENFLAG);	//an OPEN pulse
		else AddPulse((msTime - Tdata), 0);	//last drive was OPEN, so just wait
	}
}

void AddCycle(short GapMilliseconds)	//adds a cycle to the mudpulse array
{
	AddPulse(PULSE_TIME, CLOSEFLAG);	//CLOSE for PULSE_TIME
	AddPulse(GapMilliseconds, OPENFLAG);	//OPEN for GAP time
		//Store the resultant times into the mud pulse array
}
		
void Chirp(short cycles, short nibble) //chirp one of 16 representing binary nibble
{
int i;
	if(nibble < 0x8) //ChirpUp
			//ChirpUp(cycles, (nibble & 0x7));	
		for (i = (cycles - 1); (i >= 0); i--) 
			AddCycle(GAP[i] + ((GAP[i] * (nibble & 0x7))/SPEED_STEP_INTERVAL));
	else // ChirpDn
			//ChirpDn(cycles, (nibble & 0x7));
		for (i = 0; (i < cycles); i++) 
			AddCycle(GAP[i] + ((GAP[i] * (nibble & 0x7))/SPEED_STEP_INTERVAL));
}
	
void DataChirp(short cycles) //chirp one of 16 data from FrameBuffer
{
	if(FrameBufferFetch >= FrameBufferStore) //FrameBuffer is empty
		Chirp(cycles, 0); //send data of 0
	else //send data and increment the buffer pointer
		Chirp(cycles, FrameBuffer[FrameBufferFetch++]); 
}

void FormatGR_TF(long GR, long TF)
// add fresh GR and TF to mudpulse buffer with analog time encoding and subchannel data
{
	OPENUntilTime(GR);	//set GammaRayCPS chirp time
	DataChirp(4);	//send GammaRayCPS chirp with data
	OPENUntilTime(TF);	//set ToolFace chirp time
}

void Data2FrameBuffer(short nibble)
// add nibble to FrameBuffer and do checksum
{
	FrameBuffer[FrameBufferStore++] = nibble ;	//store nibble to FrameBuffer & increment index
	FrameChecksum = (FrameChecksum ^ nibble) ;	//XOR the new nibble into the frame checksum
		//printf("FrameBuffer = %d FrameBufferStoreFrame = %d Checksum=%d\n",FrameBuffer[FrameBufferStore-1],FrameBufferStore,FrameChecksum);
	}
	
void AddData(short data, short nibbles)
// add data to FrameBuffer
// nibbles 1 to 4 are the number of 4 bit nibbles to encode
// keep running XOR checksum
{
	switch(nibbles)
	{
		case 4:
			Data2FrameBuffer((data & 0xF000) >>12);	
		case 3:
			Data2FrameBuffer((data & 0x0F00) >>8);	
		case 2:
			Data2FrameBuffer((data & 0x00F0) >>4);	
		case 1:
			Data2FrameBuffer((data & 0x000F) );
	}
}

#define CalID 		00
#define AziIncID	01
#define GzHzID		02
#define GxGyID		03
#define HxHyID		04
#define T_PID		05
//#define HIGHEST_DATA_FRAME T_PID
#define HIGHEST_DATA_FRAME AziIncID

void FormatData(short FrameID) //formats data to FrameBuffer
{
	FrameBufferStore = 0;	//reset the FrameBufferStore to add fresh data
	FrameBufferFetch = 0;	//reset the FrameBufferFetch to first data item
	FrameChecksum = 0;		//reset the checksum for fresh data
	switch(FrameID) //select the frame to format
	{
		case CalID: 		AddData(ToolStatus, 2); 
							break; 
			//chirp at specific time with specific data to calibrate uphole receiver
			//sends time marker chirps at T_GR_MAX and T_TF_0 then GR_TF in normal format
			//sends 8 bits tool status with checksum in subchannel

		case AziIncID:		AddData((azimuth), 3) ; //azimuth
							AddData((inclination), 3) ; //inclination
 							break;
 							
		case GzHzID: 		AddData(avg5, 4) ;	//gz
							AddData(avg2, 4) ;	//hz
							break;
		
		case GxGyID: 		AddData(avg7, 4) ;	//gx
							AddData(avg6, 4) ;	//gy
 							break;
		
		case HxHyID: 		AddData(avg4, 4) ;	//hx
							AddData(avg3, 4) ;	//hy
 							break;
				
		case T_PID: 		AddData((Temperature), 3) ; //temperature
							AddData((Pressure),    3) ; //total magnetic
							break;		
	}
	AddData((FrameChecksum),    1) ; //checksum to data buffer
}

void FormatFrame(FrameID) //create the requested frame in the mudpulse buffer 
{
	Chirp(6, FrameID) ;;	//FormatFrameID
	FormatData(FrameID);	//formats data to FrameBuffer

	if(CalID == FrameID) //calibration frame
		FormatGR_TF(T_GR_MAX, T_TF_0); //with cal time markers
	else FormatGR_TF(CalcLogGR(GammaRayCPS), (T_TF_0+ToolFace)); //with GR TF
		//scale 0-3600 ms -180.0 to +180.0 degree range integer -1800 to +1800
		//scale to 1 millisecond per 0.1 degree in TF range
}

void SetPULL_TIME(FrameNumber) //for testing various PULL times
{
	switch(FrameNumber) //send rotating frame sequence
	{
		case CalID: 	PULL_TIME = PULL_TIME_DEFAULT; break; //default pull time
		case AziIncID:	PULL_TIME = 800; break;
		case GzHzID: 	PULL_TIME = 600; break;
		case GxGyID: 	PULL_TIME = 400; break;
		case HxHyID: 	PULL_TIME = 200; break;	
		case T_PID: 	PULL_TIME = 100; break;	
	}	
}

//for test sequence
void FormatMudPulses()
{
	mudstoreindex = 0;		//reset the mud pulse store index 
	mudfetchindex = 0;		//reset the mud pulse fetch index 
	Tdata = 0;				//reset elapsed time to T0
	int i;
	for( i=1000; i>=100; i=i-100) //loop down every 100 ms increment
	{
		PULL_TIME = i; //set shorter pull time
		AddCycle(PULSE_TIME); //add a cycle 
	}
	OPENUntilTime(Tdata + 5000); //make a gap 5000 ms
}
		

/*
void FormatMudPulses()
// Encode more data into the mudpulse[] buffer
// round robbin frame sequencer for starters
{
static  short   FrameNumber = 0;
	mudstoreindex = 0;		//reset the mud pulse store index 
	mudfetchindex = 0;		//reset the mud pulse fetch index 
	Tdata = 0;				//reset elapsed time to T0
	if(FrameBufferFetch >= FrameBufferStore)	//FrameBuffer is empty
	{
		SetPULL_TIME(FrameNumber);
			//printf("FrameNumber = %d PULL_TIME = %d\n",FrameNumber,PULL_TIME);
		FormatFrame(FrameNumber);	//outputs the T0 chirp as FrameID
		FrameNumber++; //round robbin frame sequence
	}
	else 
	{
		DataChirp(5);
		FormatGR_TF(CalcLogGR(GammaRayCPS), (T_TF_0+ToolFace));
	}

		//printf("FormatMudPulses completed\n");
	 	
	if(FrameNumber > HIGHEST_DATA_FRAME) FrameNumber = 0; //reset Frame sequence
	
					
#if __SIMULATION__
	printf("FormatMudPulses completed\n");
#endif
}
*/
    /* the data is summed, and stored in buffers as follows
    	_avg1 = channel 1 = RTD
    	_avg2 = channel 2 = mag z data
    	_avg3 = channel 3 = mag y data
    	_avg4 = channel 4 = mag x data
    	_avg5 = channel 5 = accel z data
    	_avg6 = channel 6 = accel y data
    	_avg7 = channel 7 = accel x data
    	_avg8 = channel 8 = GR raw value
    	_avg9 = channel 9
    	_avg10 = channel 10
    	_avg11 = channel 11

    	data_seq[SEQ_INC]     = inclination;
    	data_seq[SEQ_AZI]     = azimuth
    	data_seq[SEQ_G]       = G1000;
    	data_seq[SEQ_H]       = H1000;
    	data_seq[SEQ_GR]      = GammaRayCPS; // GR
    	data_seq[SEQ_TEMP]    = Temperature;    	
    	*/

void Data2Frame() {
	//if the mudpulse buffer is empty format more pulses into the buffer
	if(!(mudstoreindex > mudfetchindex)) //there is no data in the buffer
		FormatMudPulses();
}

//-----------------------------------------------------------------------------
// UPDATE OUTPUT
//
// Description:
//      Updates the data sequence array with the current data
//
// Notes:
//      Also saves the info to flash every 1 minutes at the current rate
//
//-----------------------------------------------------------------------------
void UpdateOutput()
{
    // Update Data  // now for flash data store only
    // Save Data To Flash (Time Stamped) and increment sequence count (it's stored in seconds at the moment)
    // Check if it is time to save to flash
    if(TimerExpired(SAVE_FLASH_TIMER))
    {   // Reset Timer
    	data_seq[SEQ_INC]     = inclination;
    	data_seq[SEQ_AZI]     = azimuth;
    	data_seq[SEQ_G]       = G1000;
    	data_seq[SEQ_H]       = H1000;
    	data_seq[SEQ_GR]      = GammaRayCPS; // GR
    	data_seq[SEQ_TEMP]    = Temperature;
    	data_seq[SEQ_MONTH]   = 1;
        data_seq[SEQ_DAY]     = 26;
        data_seq[SEQ_YEAR]    = 6;
        data_seq[SEQ_HOUR]    = 2;
        data_seq[SEQ_MINUTE]  = 25;
        data_seq[SEQ_SECONDS] = ++seq_count;
    	SaveData();
    	SetTimer(SAVE_FLASH_TIMER,SAVE_FLASH_TIME);   // Set Timer for next save flash
    }
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
void SaveData()
{
	static ulong curr_addr = FLASH_DATA_ADDR;
        // Current Flash Address that the data is going to be written
    ulong  sector_addr;
        // Current Sector address writing too.
    ushort scratch[128];
        // Scratch buffer
    uchar  idx = 0;
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
}

