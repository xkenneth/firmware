//** Program peterc.c  ** //
//** Revision History **//
// 001	10.15.03 	Initial Release  
// 002	11.03.03 	Updated coeffs for tool 1
// 003	11.05.03 	Received new code from Joe Allard
// NOTE: There are problems with the declarations of the variables in the
// computation of Mx,Bx,etc. We probably want to be working with signed integers
// 004	01/26/06	Updated coeffs for tool 2
//						put extra parens in PeterC() formulas for readability
//						Add GR_CPS() to make GR readings total per minute
//						typecast in formulas to correct math errors
//						correct azimuth to 0-360 degrees range
//						correct azimuth formula from divide by zero error
// 005	01/26/06	Improved Gamma Ray preamp to stop ringing. Changed GR code to take actual count
//					from GammaRay preamp.
// 006	03/10/06	Reduce Gamma Ray time average window to 30 secinds but keep 
//					counts in Counts Per Minute.
// 007	03/13/06	Change Gamma Ra30y to 30 second window average of counts per second
// note in future to apply cal constant to Temperature measurement
// Azimuth formula needs improvement in accuracy around due east and west
// 008	04/02/07	Updated coeffs for accel, magnet remains the same

#include <stdio.h>
//#include <math_const.h>
#include <math.h>
#include "peterc.h"
#include "flash.h"
#include "sysdef.h"
#include "timer.h"

ACCESS_TIMERS;

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define samples         10
#define PI              3.1415926535897932384626433832795
#define FLASH_HEADER    0xA55A
#define STATUS_HEADER	0xA55A
#define PATTERN_HEADER	0xA55A
#define PATTERN_SEQ_HEADER	0xA55A
//#define FLASH_HEADER_SZ 2       // (16bit short size)
    // Flash header consists of the FLASH_HEADER and COEFF_VERSION

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern	short	avg1,avg2,avg3,avg4,avg5,avg6;
extern  short   avg7,avg8,avg9,avg10,avg11; /* Averages from peterdsp.dsp */
extern	unsigned GRayCPS;		//gamma ray counts per second

//-----------------------------------------------------------------------------
// Coefficients
//      If you want to hard code new factory coefficients into the new code.
//      You must change the coefficients in the const factory_coeff array below.
//      Than you must increment the COEFF_VERSION number.
//      The version number is saved in flash.  It reads this number out of flash
//      at start up.  If the version is different than the one stored in flash,
//      it will reset the coefficients to the factory coefficients.
//-----------------------------------------------------------------------------
#define COEFF_VERSION   9   // !! Must increment if you change the factory_coeff[]
//	order must match the enumeration in the header file
//The tool with accel sensors attached to the chasis use version 2
//The tool with accel sensors attached to the sensor block use version 3

static const ushort factory_coeff[NUM_CALIB_CONST] = 
{

/*
//	first pass cal for tool 1
    32041,  // Gxmean
    11705,   // Gxrange
    29175,  // Gymean
    11777,   // Gyrange
    29284,  // Gzmean
    11817,   // Gzrange
    38656,  // Mxmean
    27892,  // Mxrange
    39637,  // Mymean
    23291,  // Myrange
    38757,  // Mzmean
    29240,  // Mzrange
*/

/*
//	first pass cal for tool ? w/circuit updates
//	version 2 coefficients

    29300,  // Gxmean
    2960,  // Gxrange
    29300,  // Gymean
    2980,  // Gyrange
    29300,  // Gzmean
    2994,  // Gzrange
    39347,  // Mxmean
    10583,  // Mxrange
    39150,  // Mymean
    9150,  // Myrange
    38808,  // Mzmean
    12659,  // Mzrange     
*/

/*
//	first pass cal for tool ? w/circuit updates
//  version 3 coefficient

    52500,  // Gxmean
    5820,  // Gxrange
    52500,  // Gymean
    5940,  // Gyrange
    52500,  // Gzmean
    5810,  // Gzrange
    39347,  // Mxmean
    10583,  // Mxrange
    39150,  // Mymean
    9150,  // Myrange
    38808,  // Mzmean
    12659,  // Mzrange    
*/

/*
//	first pass cal for tool ? w/circuit updates
//  version 4 coefficient

    29134,  // Gxmean
    2990,  // Gxrange
    29357,  // Gymean
    2925,  // Gyrange
    29115,  // Gzmean
    2972,  // Gzrange
    39347,  // Mxmean
    10583,  // Mxrange
    39150,  // Mymean
    9150,  // Myrange
    38808,  // Mzmean
    12659,  // Mzrange  
*/
    
/*
//	06/15/07 shop calibration constant for toool 1, calibrated on stand in shop
//	version 5 coefficient

	29133,  // Gxmean
    2989,  	// Gxrange
    29345,  // Gymean
    2925,  	// Gyrange
    29199,  // Gzmean
    2970,  	// Gzrange
    
    39458,  // Mxmean
    11521,  // Mxrange
    39258,  // Mymean
    9749,  	// Myrange
    39392,  // Mzmean
    12525,  // Mzrange
*/
       
/*    
//	06/18/07 shop calibration constant for toool 2, calibrated on stand in shop
//	version 6 coefficient

	51416,  // Gxmean
    5671,  	// Gxrange
    51932,  // Gymean
    5839,  	// Gyrange
    52102,  // Gzmean
    5804,  	// Gzrange
    
    40144,  // Mxmean
    10253,  // Mxrange
    39770,  // Mymean
    12076,  	// Myrange
    38968,  // Mzmean
    12978,  // Mzrange
*/

/*     
//	06/19/07 field calibration constant for toool 1, calibrated on stand in field
//	version 7 coefficient

	29133,  // Gxmean
    2989,  	// Gxrange
    29345,  // Gymean
    2925,  	// Gyrange
    29199,  // Gzmean
    2970,  	// Gzrange
    
    39507,  // Mxmean
    12136,  // Mxrange
    39268,  // Mymean
    10392,  	// Myrange
    39358,  // Mzmean
    13404,  // Mzrange
*/

/*
//	10/30/07 field calibration constant for toool 1, calibrated on stand in field
//	version 7 coefficient

	29175,  // Gxmean
    2989,  	// Gxrange
    29409,  // Gymean
    2925,  	// Gyrange
    29267,  // Gzmean
    2970,  	// Gzrange
    
    39507,  // Mxmean
    12136,  // Mxrange
    39268,  // Mymean
    10392,  	// Myrange
    39358,  // Mzmean
    13404,  // Mzrange  
*/
  
/*
//	12/10/07 field calibration constant for toool 1, calibrated on stand in field
//	version 7 coefficient

	29175,  // Gxmean
    2989,  	// Gxrange
    29409,  // Gymean
    2925,  	// Gyrange
    29267,  // Gzmean
    2970,  	// Gzrange
    
    39464,  // Mxmean
    11554,  // Mxrange
    38980,  // Mymean
    9874,   // Myrange
    39476,  // Mzmean
    12846,  // Mzrange  
*/

/*
//	03/05/08 field calibration constant for toool 1, calibrated on stand in field
//	version 7 coefficient

	29156,  // Gxmean
    2988,  	// Gxrange
    29407,  // Gymean
    2924,  	// Gyrange
    29278,  // Gzmean
    2971,  	// Gzrange
    
    39423,  // Mxmean
    11756,  // Mxrange
    39043,  // Mymean
    9941,   // Myrange
    39577,  // Mzmean
    13215,  // Mzrange  
*/

/*
//	06/19/07 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51416,  // Gxmean
    5671,  	// Gxrange
    51932,  // Gymean
    5839,  	// Gyrange
    52102,  // Gzmean
    5804,  	// Gzrange
    
    40165,  // Mxmean
    11079,  // Mxrange
    39802,  // Mymean
    13126,  	// Myrange
    39011,  // Mzmean
    13783,  // Mzrange 
*/

/*       
//	10/31/07 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51426,  // Gxmean
    5671,  	// Gxrange
    51774,  // Gymean
    5839,  	// Gyrange
    51978,  // Gzmean
    5804,  	// Gzrange
    
    40165,  // Mxmean
    11079,  // Mxrange
    39802,  // Mymean
    13126,  	// Myrange
    39011,  // Mzmean
    13783,  // Mzrange 
*/

/*
//	11/27/07 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51418,  // Gxmean
    5661,  	// Gxrange
    51732,  // Gymean
    5823,  	// Gyrange
    51905,  // Gzmean
    5788,  	// Gzrange
    
    40165,  // Mxmean
    11079,  // Mxrange
    39802,  // Mymean
    13126,  	// Myrange
    39011,  // Mzmean
    13783,  // Mzrange 
*/

/*
//	12/07/07 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51418,  // Gxmean
    5661,  	// Gxrange
    51732,  // Gymean
    5823,  	// Gyrange
    51905,  // Gzmean
    5788,  	// Gzrange
    
    38939,  // Mxmean
    10794,  // Mxrange
    38349,  // Mymean
    12700,  // Myrange
    38946,  // Mzmean
    13470,  // Mzrange 
*/

/*
//	03/04/08 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51176,  // Gxmean
    5630,  	// Gxrange
    51478,  // Gymean
    5790,  	// Gyrange
    51705,  // Gzmean
    5758,  	// Gzrange
    
    38547,  // Mxmean
    12571,  // Mxrange
    39083,  // Mymean
    10708,  // Myrange
    38893,  // Mzmean
    13486,  // Mzrange 
*/

/*
//	05/21/08 field calibration constant for toool 2, calibrated on stand in field
//	version 8 coefficient

	51135,  // Gxmean
    5632,  	// Gxrange
    51425,  // Gymean
    5780,  	// Gyrange
    51585,  // Gzmean
    5764,  	// Gzrange
    
    38061,  // Mxmean
    13663,  // Mxrange
    39112,  // Mymean
    11586,  // Myrange
    38924,  // Mzmean
    14432,  // Mzrange 
*/

//	04/16/08 field calibration constant for toool 3, calibrated on stand in field
//	version 8 coefficient

	52773,  // Gxmean
    5912,  	// Gxrange
    52407,  // Gymean
    5784,  	// Gyrange
    51876,  // Gzmean
    5802,  	// Gzrange
    
    40243,  // Mxmean
    4669,  // Mxrange
    39779,  // Mymean
    3813,  // Myrange
    39543,  // Mzmean
    6943,  // Mzrange       


/*
//	05/02/08 field calibration constant for toool 4, calibrated on stand in field
//	version 8 coefficient

	52880,  // Gxmean
    5913,  	// Gxrange
    53003,  // Gymean
    5859,  	// Gyrange
    51483,  // Gzmean
    5700,  	// Gzrange
    
    39126,  // Mxmean
    3404,  // Mxrange
    39203,  // Mymean
    4280,  // Myrange
    39192,  // Mzmean
    5588,  // Mzrange       
*/
    
    
/*
//	06/20/07 tempreature sensor calibration for tool 1

	175,	// Tpgain
	6544,	// Tpoffset
*/

		
//	06/21/07 tempreature sensor calibration for tool 2

	2566,	// Tpgain
	5395,	// Tpoffset

	
/*	
//	05/02/08 tempreature sensor calibration for tool 4

	538,	// Tpgain
	16873,	// Tpoffset
*/

};
    
unsigned short coeff[NUM_CALIB_CONST];  // Holds the current coefficients

//-----------------------------------------------------------------------------
// Status constant
//      If you want to hard code new factory coefficients into the new code.
//      You must change the status constant in the const array below.
//      Than you must increment the STATUS_VERSION number.
//      The version number is saved in flash.  It reads this number out of flash
//      at start up.  If the version is different than the one stored in flash,
//      it will reset the coefficients to the coefficients.
//-----------------------------------------------------------------------------
#define STATUS_VERSION   31   // !! Must increment if you change the status constant
//	order must match the enumeration in the header file
//The tool with accel sensors attached to the chasis use version 2
//The tool with accel sensors attached to the sensor block use version 3

static const ushort factory_status[NUM_STATUS_CONST] = 
{

	//0,			//Pulse_Test
	//0,			//Pulse_Simple
	//0,			//Pulse_Freq
	//0,			//Pulse_Static
	0,			//Sensor_Live
	//0,			//Sensor_Frame
	0,			//Motor_Cycle
	0,			//TF_Reset
				
	0x0BF7,		//higher 16 bits of long_date_time 
	0x0E50A,	//lower 16 bits of long_date_time 

	300,		//pulse_time
	300,		//code_pulse_time
	7,			//code_pulse_diff
	500,		//N_Pulse
	1000,		//W_Pulse
	0,			//tts
	0,			//tto
	
	0x0000,		//syncNW
	0x2150,		//syncPN
	14,			//syncLEN
	
	576,		//gear_numerator
	25,			//gear_denominator
	4,			//motor encoder scale factor
	
	0,			//ini_pattern_seq_idx
	
	30,			//rt_buf_size
	30,			//dly_rt_count
	10,			//dly_nrt_count
	
	20,			//RT_GX
	20,			//RT_GY
	20,			//RT_GZ
	20,			//RT_HX
	20,			//RT_HY
	20,			//RT_HZ
	
	20,			//RT_GX
	20,			//RT_GY
	20,			//RT_GZ
	20,			//RT_HX
	20,			//RT_HY
	20,			//RT_HZ
	
	5,			//dly_qt_count
	1,			//dly_nqt_count
	
	15,			//RT_GX
	15,			//RT_GY
	15,			//RT_GZ
	15,			//RT_HX
	15,			//RT_HY
	15,			//RT_HZ
	
	15,			//RT_GX
	15,			//RT_GY
	15,			//RT_GZ
	15,			//RT_HX
	15,			//RT_HY
	15,			//RT_HZ
	
	50,			//TF_G_Angle
	50,			//TF_H_Angle
	0,			//TF_G_Offset
	0,			//TF_H_Offset
	70,			//TF_Threshold
	5,			//max_inc
	
	4,			//gr_log_size
	
	4,			//restart_step
	5,			//ini_PWM_restart
	
	4,			//ini_PWM_calib
	3,			//skip_count
	0xFFFF,		//open_load_calib
	0xFFFF,		//shut_load_calib
	0x0020,		//ref_load_calib
	0,			//motor_open_offset
	0,			//motor_shut_offset
	
	5,			//Speed_Dn
	2,			//MotorHome

	6,			//up_open_pwm_count
	6,			//up_shut_pwm_count
	2,			//dn_open_pwm_count
	2,			//dn_shut_pwm_count
	
	0,			//ini_up_open_PWM
	0,			//ini_up_shut_PWM
	
	5,			//up_open_max_PWM
	5,			//up_shut_max_PWM
	
	0,			//dn_open_min_PWM
	0,			//dn_shut_min_PWM
	
	20,			//motor_freeze_count
	
	0x0004,		//higher 16 bits of the motor_freeze_rate
	0x93E0,		//lower 16 bits of the motor_freeze_rate
	
	0,			//higher 16 bits of stop_time
	30000,		//lower 16 bits of stop time
	0x0004,		//higher 16 bits of rt_dly_rate
	0x93E0,		//lower 16 bits of rt_dly_rate
	0x0009,		//higher 16 bits of data_fresh_rate
	0x27C0,		//lower 16 bits of data_fresh_rate
	0,			//higher 16 bits of data_save_rate
	60000,		//lower 16 bits of data_save_rate

};

unsigned short status[NUM_STATUS_CONST];  // Holds the current coefficients

//-----------------------------------------------------------------------------
// Pattern constant
//      If you want to hard code new factory coefficients into the new code.
//      You must change the status constant in the const array below.
//      Than you must increment the STATUS_VERSION number.
//      The version number is saved in flash.  It reads this number out of flash
//      at start up.  If the version is different than the one stored in flash,
//      it will reset the coefficients to the coefficients.
//-----------------------------------------------------------------------------
#define PATTERN_VERSION   2   // !! Must increment if you change the status constant
//	order must match the enumeration in the header file
//The tool with accel sensors attached to the chasis use version 2
//The tool with accel sensors attached to the sensor block use version 3

static const ushort factory_pattern[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE] = 
{
	
	/*
	PC_REGULAR_FRAME0,
	PC_REGULAR_FRAME1,
	PC_REGULAR_FRAME2,
	PC_REGULAR_FRAME3,
	PC_REGULAR_FRAME4,
	PC_REGULAR_FRAME5,
	*/
	
	/*
	PC_STATIC_FRAME0,
	PC_STATIC_FRAME1,
	PC_STATIC_FRAME2,
	PC_STATIC_FRAME3,
	PC_STATIC_FRAME4,
	PC_STATIC_FRAME5,
	*/
	
	PC_SIMPLE_PULSE,
	PC_STOP,
	
	0x0FFFF,		//end of the pattern

};

unsigned pattern[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE];		//pattern buffer

static const ushort factory_pattern_count[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE] = 
{

	10,
	20,

};

unsigned pattern_count[NUM_PATTERN_SEQ][PATTERN_BUF_SIZE];	//pattern count buffer
unsigned pattern_idx[NUM_PATTERN_SEQ];		//pattern index
unsigned pattern_counter[NUM_PATTERN_SEQ];		//pattern counter


//-----------------------------------------------------------------------------
// Pattern sequence constant
//      If you want to hard code new factory coefficients into the new code.
//      You must change the status constant in the const array below.
//      Than you must increment the STATUS_VERSION number.
//      The version number is saved in flash.  It reads this number out of flash
//      at start up.  If the version is different than the one stored in flash,
//      it will reset the coefficients to the coefficients.
//-----------------------------------------------------------------------------
#define PATTERN_SEQ_VERSION   1   // !! Must increment if you change the status constant
//	order must match the enumeration in the header file
//The tool with accel sensors attached to the chasis use version 2
//The tool with accel sensors attached to the sensor block use version 3

static const ushort factory_pattern_seq[PATTERN_SEQ_BUF_SIZE] = 
{
	
	SEQ_REGULAR,
	SEQ_ALT,
	
	0xFFFF,		//end of pattern sequence
	
};

unsigned pattern_seq[PATTERN_SEQ_BUF_SIZE];		//pattern sequence


static const ushort factory_pattern_seq_rate[PATTERN_SEQ_BUF_SIZE*2] = 
{
	
	0x0009,		//higher 16 bits
	0x27C0,		//lower	16 bits
	
	0x0009,		//higher 16 bits
	0x27C0,		//lower	16 bits
	
};

unsigned long pattern_seq_rate[PATTERN_SEQ_BUF_SIZE];		//time to run a pattern sequence
unsigned ini_pattern_seq_idx;	//initial pattern sequence index
unsigned pattern_seq_idx;	//pattern sequence index

//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------
float 	G,H,azi,inc,Temperature;	//these seem to be globally accessed
//short	GammaRayCPS = 0;	//the Gamma Ray in counts per minute
double	Avg_GRayCPS = 0;	//the average Gamma Ray in counts per minute
double  TF_G = 0;		//gavity tool face
double	TF_G_Total = 0;		//total gravity tool face
double 	TF_H = 0;		//magnetic tool face
double	TF_H_Total = 0;		//total magnetic tool face
double	ToolFace = 0;		//calculated ToolFace
short	Pressure = 0;		//calculated pressure

bool TF_GH = false;		//flag to switch between magnetic tool face and gravity tool face
bool TF_Reset;		//flag to reset tool face offset
//#define INI_TFS_ANGLE	4.5		//initial tool face switch angle
unsigned TF_G_Angle;		//tool face gravity switch angle
unsigned TF_H_Angle;		//tool face magnetic switch angle
unsigned TF_G_Offset;		//offset angle of the gravity tool face
unsigned TF_H_Offset;		//offste angle of the magnetic tool face
unsigned TF_Threshold;		//threshold of gamma ray to calibrate tool face
//-----------------------------------------------------------------------------
// GR_CPS Routine
//-----------------------------------------------------------------------------

#define GRBins 30	//bins in the window to average
#define INI_TOTAL	(30*GRBins)		//initial window_total value

static unsigned GRBin[GRBins] ={30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
							 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
							 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, };	//one bin for each second, fill with 30 background count
unsigned long total_counts = INI_TOTAL;


short GR_CPS(seconds_counter)
// Averages the Gamma Ray pulses keeping the most recent 30 second window
// uses an array to store the count every second
// each second subtract the 30 second old value and add the current value
// NOTE: This function is called once per second by Main(), 
//		
//#define GRBins 30	//bins in the window to average
//#define CPS_MAX 1000	//Limit GR counts to 1000 CPS
//#define CPS_MAX 500	//Limit GR counts to 1000 CPS
{

	//static short GRBin[GRBins] ={30};	//one bin for each second, fill with 30 background count
	static short selected_bin=0;
	//static short window_total = (30 * GRBins);
	
	selected_bin = seconds_counter;
	if (selected_bin >= GRBins) //end of the GRBin[] array
		selected_bin = 0;	//reset the seconds counter
	//subtract the counts which are GRBins old
	total_counts = total_counts - GRBin[selected_bin];
	//store the most recent GR counts in the GRBin[] array
	//the min(a,b) limits counts in any second interval to reduce noise spikes
	//
	//GRBin[selected_bin] = (min(avg8,CPS_MAX));
	GRBin[selected_bin] = (min(GRayCPS,CPS_MAX));
	//add the most recent GR counts
	total_counts = total_counts + GRBin[selected_bin];
	Avg_GRayCPS = ((double)total_counts) / ((double)GRBins); //average to counts per second, time 100 so that every bit counts
	return(selected_bin);
	
}
// old comments from GR preamp running 5V power
	//looks like there is ringing on the counter input
	//3 or 4 rings per pulse
	//the integer divide by 3 eliminates the ringing counts
	//remove the division when tool circuitry is improved

//-----------------------------------------------------------------------------
// PeterC Routine
//-----------------------------------------------------------------------------

float Gx,Gy,Gz,Bx,By,Bz;		//calculated gravity and magnetic parameters

extern bool Tool_Rotate;		//flag to indicate tool rotation
extern bool Tool_Quiet;			//flag to indicate tool stability
extern bool Data_Fresh;
extern unsigned long rt_dly_rate;
extern unsigned long data_fresh_rate;

#define RT_COUNT		1		//rotating count, one axis rotating = tool is rotating
unsigned rt_counter=RT_COUNT;
#define ExpireRtCounter()		(!rt_counter)		//rotating if the counter is expired

unsigned dly_rt_count;
unsigned dly_rt_counter;
#define ExpireDlyRtCounter()	(!dly_rt_counter)		//delayed rotating counter

unsigned dly_nrt_count;
unsigned dly_nrt_counter;
#define ExpireDlyNrtCounter()	(!dly_nrt_counter)		//delayed non-rotating counter


#define QT_COUNT		1		//rotating data count, one axis rotating = data is not ready
unsigned qt_counter=QT_COUNT;
#define ExpireQtCounter()		(!qt_counter)		//data not stable if the counter is expired

unsigned dly_qt_count;
unsigned dly_qt_counter;
#define ExpireDlyQtCounter()	(!dly_qt_counter)		//delayed rotating data counter

unsigned dly_nqt_count;
unsigned dly_nqt_counter;
#define ExpireDlyNqtCounter()	(!dly_nqt_counter)		//delayed non-rotating data counter

//buffer to store sensor values
unsigned gxbin[RTBIN_INS_SIZE];
unsigned gybin[RTBIN_INS_SIZE];
unsigned gzbin[RTBIN_INS_SIZE];

unsigned hxbin[RTBIN_INS_SIZE];
unsigned hybin[RTBIN_INS_SIZE];
unsigned hzbin[RTBIN_INS_SIZE];

unsigned HxBin[RTBIN_SIZE];
unsigned HyBin[RTBIN_SIZE];
unsigned HzBin[RTBIN_SIZE];

unsigned GxBin[RTBIN_SIZE];
unsigned GyBin[RTBIN_SIZE];
unsigned GzBin[RTBIN_SIZE];

unsigned rt_buf_size;		//size of the sensor buffer

unsigned rt_limit[NUM_OF_RT];		//threshold array that determine if an axis is rotating or not, full scale 100
unsigned qt_limit[NUM_OF_RT];	//threshold array that determine if data can be freezed to the buffer
unsigned rt_ins_limit[NUM_OF_RT];		//instantaneous threshold array that determine if an axis is rotating or not, full scale 100
unsigned qt_ins_limit[NUM_OF_RT];	//instantaneous threshold array that determine if data can be freezed to the buffer

unsigned rt_idx=0;

//calclate the average of a array
double ArrayAvg(unsigned *data, unsigned data_size)
{
	int i;
	double array_avg=0.0;
	
	for (i=0;i<data_size;i++) {
		
		array_avg=array_avg+(double)(*(data+i));
		
	}
	
	return(array_avg/(double)data_size);
	
}

//calculate a biased standard diviation of an array
double ArrayStd(unsigned *data, unsigned data_size)
{
	
	int i;
	double array_std=0.0;
	double array_avg=0.0;
	
	array_avg=ArrayAvg(data, data_size);
	
	for (i=0;i<data_size;i++) {
		
		array_std=array_std+((double)(*(data+i))-array_avg)*((double)(*(data+i))-array_avg);
		
	}

	return(sqrt(array_std/(double)data_size));
	
}

//determine if a axis is rotating
unsigned Axis_Rt(unsigned *data, unsigned data_size, unsigned rt_limit_axis, unsigned axis_gain)
{
		
	if(ArrayStd(data, data_size)>=((double)rt_limit_axis*(double)axis_gain)/100.0) {
		
		return(1);
		
	}
	
	else return(0);
	
}

//determine is the tool is rotating
void Tool_Rt()
{
	
	rt_counter=RT_COUNT;
	
	//fast rotating if the instantaneous sensor reading is changing above the threshold
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(gxbin, RTBIN_INS_SIZE, rt_ins_limit[RT_GX], coeff[CC_GXRANGE]/RTBIN_INS_SIZE);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(gybin, RTBIN_INS_SIZE, rt_ins_limit[RT_GY], coeff[CC_GYRANGE]/RTBIN_INS_SIZE);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(gzbin, RTBIN_INS_SIZE, rt_ins_limit[RT_GZ], coeff[CC_GZRANGE]/RTBIN_INS_SIZE);
	
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(hxbin, RTBIN_INS_SIZE, rt_ins_limit[RT_HX], coeff[CC_MXRANGE]/RTBIN_INS_SIZE);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(hybin, RTBIN_INS_SIZE, rt_ins_limit[RT_HY], coeff[CC_MYRANGE]/RTBIN_INS_SIZE);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(hzbin, RTBIN_INS_SIZE, rt_ins_limit[RT_HZ], coeff[CC_MZRANGE]/RTBIN_INS_SIZE);
	
	//slow rotating if the accumulated/averaged sensor reading is changing above threshold
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(GxBin, rt_buf_size, rt_limit[RT_GX], coeff[CC_GXRANGE]);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(GyBin, rt_buf_size, rt_limit[RT_GY], coeff[CC_GYRANGE]);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(GzBin, rt_buf_size, rt_limit[RT_GZ], coeff[CC_GZRANGE]);
	
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(HxBin, rt_buf_size, rt_limit[RT_HX], coeff[CC_MXRANGE]);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(HyBin, rt_buf_size, rt_limit[RT_HY], coeff[CC_MYRANGE]);
	if(rt_counter) rt_counter=rt_counter-Axis_Rt(HzBin, rt_buf_size, rt_limit[RT_HZ], coeff[CC_MZRANGE]);
		
	if(ExpireRtCounter()) {
		
		if(dly_rt_counter) {
			
			dly_rt_counter--;
			if(ExpireDlyRtCounter()&(!Tool_Rotate)) {
				
				Tool_Rotate=true;
				
				if(Data_Fresh) {
					
					SetTimer(DATA_FRESH_TIMER, data_fresh_rate);
					Data_Fresh=false;
					
				}
				
			}
			
		}
		
		dly_nrt_counter=dly_nrt_count;
		
	}
	
	else {
		
		if(dly_nrt_counter) {
			
			dly_nrt_counter--;
			if(ExpireDlyNrtCounter()&Tool_Rotate) {
			
				SetTimer(RT_DLY_TIMER, rt_dly_rate);
				Tool_Rotate=false;
			
			}
			
		}
		
		dly_rt_counter=dly_rt_count;
		
	}
	
	//Send_Value(Tool_Rotate);
	
}

//determine is the tool is quiet enough for data acquiring
void Tool_Qt()
{
	
	qt_counter=QT_COUNT;
	
	//fast rotating if the instantaneous sensor reading is changing above the threshold
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(gxbin, RTBIN_INS_SIZE, qt_limit[RT_GX], coeff[CC_GXRANGE]/RTBIN_INS_SIZE);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(gybin, RTBIN_INS_SIZE, qt_limit[RT_GY], coeff[CC_GYRANGE]/RTBIN_INS_SIZE);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(gzbin, RTBIN_INS_SIZE, qt_limit[RT_GZ], coeff[CC_GZRANGE]/RTBIN_INS_SIZE);
	
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(hxbin, RTBIN_INS_SIZE, qt_limit[RT_HX], coeff[CC_MXRANGE]/RTBIN_INS_SIZE);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(hybin, RTBIN_INS_SIZE, qt_limit[RT_HY], coeff[CC_MYRANGE]/RTBIN_INS_SIZE);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(hzbin, RTBIN_INS_SIZE, qt_limit[RT_HZ], coeff[CC_MZRANGE]/RTBIN_INS_SIZE);
	
	//slow rotating if the accumulated/averaged sensor reading is changing above threshold
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(GxBin, rt_buf_size, qt_limit[RT_GX], coeff[CC_GXRANGE]);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(GyBin, rt_buf_size, qt_limit[RT_GY], coeff[CC_GYRANGE]);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(GzBin, rt_buf_size, qt_limit[RT_GZ], coeff[CC_GZRANGE]);
	
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(HxBin, rt_buf_size, qt_limit[RT_HX], coeff[CC_MXRANGE]);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(HyBin, rt_buf_size, qt_limit[RT_HY], coeff[CC_MYRANGE]);
	if(qt_counter) qt_counter=qt_counter-Axis_Rt(HzBin, rt_buf_size, qt_limit[RT_HZ], coeff[CC_MZRANGE]);
	
	if(!ExpireQtCounter()) {
		
		if(dly_qt_counter) {
			
			dly_qt_counter--;
			if(ExpireDlyQtCounter()&(!Tool_Quiet)) Tool_Quiet=true;
			
		}
		
		dly_nqt_counter=dly_nqt_count;
		
	}
	
	else {
		
		if(dly_nqt_counter) {
			
			dly_nqt_counter--;
			if(ExpireDlyNqtCounter()&Tool_Quiet) Tool_Quiet=false;
			
		}
		
		dly_qt_counter=dly_qt_count;
		
	}
	
	//Send_Value(dly_qt_counter);
	
}

void PeterC(void)
{

	//float Gx,Gy,Gz,Bx,By,Bz,numerator,denominator;
	float numerator,denominator;
	unsigned short i,Ax,Ay,Az,Mx,My,Mz,Tp;
	//copy the raw values into integer variables for computation below
    GxBin[rt_idx]=Ax=avg7;
    GyBin[rt_idx]=Ay=avg6;
    GzBin[rt_idx]=Az=avg5;    /* these values measured by DSP_sensor board */
    HxBin[rt_idx]=Mx=avg4;
    HyBin[rt_idx]=My=avg3;
    HzBin[rt_idx]=Mz=avg2;
    
    rt_idx++;
    if(rt_idx>=rt_buf_size) rt_idx=0;
    
    Tool_Rt();
    Tool_Qt();
    
    //if((!Tool_Rotate)|Tool_Quiet) {
    if(Tool_Quiet) {		//average data when the tool is quiet
    	
    	Ax=(unsigned)ArrayAvg(GxBin, rt_buf_size);
    	Ay=(unsigned)ArrayAvg(GyBin, rt_buf_size);
    	Az=(unsigned)ArrayAvg(GzBin, rt_buf_size);
    	Mx=(unsigned)ArrayAvg(HxBin, rt_buf_size);
    	My=(unsigned)ArrayAvg(HyBin, rt_buf_size);
    	Mz=(unsigned)ArrayAvg(HzBin, rt_buf_size);
    	
    }
    
    //Temperature=avg1;	//future apply cal cosntant
    
    Tp=avg1;
    Temperature=(((float)Tp-(float)coeff[CC_TPOFFSET])*100)/(float)coeff[CC_TPGAIN];
/*
	printf("GXRANGE = %u GXMEAN = %u GX = %u \r\n",coeff[CC_GXRANGE],coeff[CC_GXMEAN],Ax);
	printf("GYRANGE = %u GYMEAN = %u GY = %u \r\n",coeff[CC_GYRANGE],coeff[CC_GYMEAN],Ay);
	printf("GZRANGE = %u GZMEAN = %u GZ = %u \r\n",coeff[CC_GZRANGE],coeff[CC_GZMEAN],Az);
	printf("MXRANGE = %u MXMEAN = %u MX = %u \r\n",coeff[CC_MXRANGE],coeff[CC_MXMEAN],Mx);
	printf("MYRANGE = %u MYMEAN = %u MY = %u \r\n",coeff[CC_MYRANGE],coeff[CC_MYMEAN],My);
	printf("MZRANGE = %u MZMEAN = %u MZ = %u \r\n",coeff[CC_MZRANGE],coeff[CC_MZMEAN],Mz);
//    printf("diff = %u \r\n",(Ax-coeff[CC_GXMEAN]) );
*/
    /* Apply scale factors and offsets */
    Bx=( ( (float)Mx - (float)coeff[CC_MXMEAN] ) / (float)coeff[CC_MXRANGE]);
    By=( ( (float)My - (float)coeff[CC_MYMEAN] ) / (float)coeff[CC_MYRANGE]);
    Bz=( ( (float)Mz - (float)coeff[CC_MZMEAN] ) / (float)coeff[CC_MZRANGE]);
	//correct for flipped sensor polarity
    Bz=-1.0*Bz;

    Gx=( ( (float)Ax - (float)coeff[CC_GXMEAN] ) / (float)coeff[CC_GXRANGE]);
	//correct for flipped sensor polarity
    Gx=-1.0*Gx;
    Gy=( ( (float)Ay - (float)coeff[CC_GYMEAN] ) / (float)coeff[CC_GYRANGE]);
	//correct for flipped sensor polarity
    Gy=-1.0*Gy;
    Gz=( ( (float)Az - (float)coeff[CC_GZMEAN] ) / (float)coeff[CC_GZRANGE]);
	//correct for flipped sensor polarity
    //Gz=-1.0*Gz;
    
    /*
    #define TOTAL_G		1.4		//estimated total g for broken sensor tool
    if (TOTAL_G*TOTAL_G>(Gx*Gx+Gy*Gy)) {
    	
    	Gz=(float)sqrt((float)(TOTAL_G*TOTAL_G)-Gx*Gx-Gy*Gy);
    	
    }
    
    else
    
    	Gz=0;
    */
    
    /* Compute G and H */
    G=sqrt((Gx*Gx)+(Gy*Gy)+(Gz*Gz));
    //G=TOTAL_G;
    H=sqrt((Bx*Bx)+(By*By)+(Bz*Bz));
    /* Compute inclination */
    //inc=(acos(Gz/G))*180/PI;
    inc=(acos(Gz/G))*180/PI;
    
    //Compute azimuth
    numerator=G*((By*Gx)-(Bx*Gy));
    denominator=(Bz*((Gx*Gx)+(Gy*Gy)))-(Gz*((Bx*Gx)+(By*Gy)));
    //protect from divide by zero
    if ((denominator < 0.0001) && (denominator > -0.0001))
    	denominator = 0.0001;
    azi=(atan(numerator/denominator))*180/PI;
    //azi is calculated to range of -180 to +180 degrees, 0=north
    //convert to range 0 to 360, 0=north
    //the positive part of the range is correct 0=north increasing to 180=south
    if (denominator < 0.0) //if denominator is negative (south), convert the range
    	azi = 180.0 + azi;
    	//subtract from half circle to correct
    	//example: -1 (south-1 deg) to +179
    	//         +1 (south+1 deg) to +181 
    if (azi < 0.0) //if azi is negative (west thru north), convert the range
    	azi = 360.0 + azi;
    	//subtract from full circle to correct
    	//example: -1 (north-1 deg) to +359 south
    	
    //tool face calculation
    
    //gravity tool face
    
    if ((inc<(float)TF_H_Angle/10.0)|(inc>360.0-(float)TF_H_Angle/10.0)) TF_GH=true;
    
    else {
    	
    	//magnetic tool face
    	if ((inc>=(float)TF_G_Angle/10.0)&(inc<=360.0-(float)TF_G_Angle/10.0)) TF_GH=false;
    	
    }
    
    
    if ((By<0.0001)&(By>-0.0001)) By=0.0001;
    TF_H=atan2(Bx, By)*180/PI;		//atan2 calculate the tool face from 0 to PI and -PI to 0
    if(TF_H<0) TF_H=360.0+TF_H;
    
    if ((Gy<0.0001)&(Gy>-0.0001)) Gy=0.0001;
    TF_G=atan2(Gx, Gy)*180/PI;
    if(TF_G<0) TF_G=360.0+TF_G;
    
 
    
    if (TF_GH) {	//use magnetic tool face
    	    	
    	ToolFace=TF_H-TF_H_Offset;
    	
    }
    	
    else {		//use the gravity tool face
        
    	ToolFace=TF_G-TF_G_Offset;
    	
    }
    
    if(ToolFace<0) ToolFace=360.0+ToolFace;
    
    
    //Rotating bypass routine, temporary use, should be removed for more sophisticated control
    //when inc>30 degree, keep pulsing
    
    //if(inc>30.0) SetTimer(RT_DLY_TIMER, rt_dly_rate);

/*
//for (i = 0; i < samples; i++)
//	{
	 printf("Ax =%u Ay =%u Az =%u \r\n",Ax,Ay,Az);
	 printf("Gx =%f Gy =%f Gz =%f \r\n",Gx,Gy,Gz);
	 printf("Mx =%u My =%u Mz =%u \r\n",Mx,My,Mz);
	 printf("Bx =%f By =%f Bz =%f \r\n",Bx,By,Bz);
	 printf("G = %f H = %f        \r\n",G,H);
	 printf("inc= %f azi= %f\r\n\n",inc,azi);
	 //printf("azi= %f \r\n",azi);
	//printf("\n");
//	}
*/	
}

//-----------------------------------------------------------------------------
// Load Coefficients Routine
//
// Description:
//      Loads the calibration constant coefficients out of flash.
//
// Notes:
//      The Factory settings has a FLASH_HEADER at the beginning of the
//      memory block.  The first thing I do is read that header and see if
//      it is correct.  If it is NOT correct.  That means either the flash
//      is corrupt or this is a new board.  So I initialize the flash memory
//      with the values in factory_coeff[].
//
//      Also there is a COEFF_VERSION stored in flash after the FLASH_HEADER.
//      The number stored on flash is compared with the COEFF_VERSION defined
//      above.  If this is different, the coeff[] is loaded with factory_coeff[]
//      and the new values stored in flash.
//
//      This routine should only be called once at startup.
//      I am using the initialization values of the array above to
//      load to flash if the header is corrupt.
//
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------
    
void LoadCoeff()
{
    int i;                  // For loop variable
    ushort flash_header;    // flash header returned from flash
    ushort coeff_version;   // Coeff verison returned from flash
    ushort scratch[128];    // Scratch buffer
   
    // Read the header out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_FACTORY_ADDR,&flash_header);
    // Read the coeff version out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_FACTORY_ADDR+2,&coeff_version);
    
    if((flash_header==FLASH_HEADER) && (coeff_version==COEFF_VERSION))   // Load Coefficients
        ReadFlash(DM_FULL_WORD,sizeof(coeff),FLASH_FACTORY_ADDR+(FLASH_HEADER_SZ*2),coeff);
    else
    {   // Flash is corrupt or not initialized so do it.
        ReadFlash(DM_FULL_WORD,sizeof(scratch),FLASH_FACTORY_ADDR,scratch);
        
        // Load Header and then the constants into the scratch pad
        scratch[0] = FLASH_HEADER;
        scratch[1] = COEFF_VERSION;
        for(i=0; i<NUM_CALIB_CONST; i++)
            scratch[i+FLASH_HEADER_SZ] = coeff[i] = factory_coeff[i];
            
        // Save Data to flash
        WriteFlash(DM_FULL_WORD,sizeof(scratch),(ulong)FLASH_FACTORY_ADDR,scratch);
    }
    
    Send_Coeff();
    
}

//-----------------------------------------------------------------------------
// Set Coefficients Routine
//
// Description:
//      Sets the given coefficient and saves it to flash
//
// Arguments:
//      cc_idx - see Calibration Constant enum
//      val    - value to set coefficient too.
//
// Notes:
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------
void SetCoeff(unsigned cc_idx, ushort val)
{
	
	WriteFlash(DM_FULL_WORD, 1, (ulong)(cc_idx*2+FLASH_FACTORY_ADDR+FLASH_HEADER_SZ*2), &val);
	coeff[cc_idx]=val;
	
}

/*void SetCoeff(uchar cc_idx, ushort val)
{
    ushort scratch[128];
        // Scratch buffer

    // Read in entire sector
    ReadFlash(DM_FULL_WORD,sizeof(scratch),FLASH_FACTORY_ADDR,scratch);
    
    // Change desired coeff
    scratch[cc_idx+FLASH_HEADER_SZ] = coeff[cc_idx] = val;
    
    // Save back to flash
    ProgFlash(DM_FULL_WORD,sizeof(scratch),FLASH_FACTORY_ADDR,scratch);
}*/


//-----------------------------------------------------------------------------
// Load Status Constant Routine
//
// Description:
//      Loads the status constant coefficients out of flash.
//-----------------------------------------------------------------------------

//flag variables
extern bool Sensor_Live;
extern bool Motor_Cycle;
extern bool Motor_Capture;

//status variables
extern unsigned long long_date_time;

extern unsigned pulse_time;
extern unsigned code_pulse_time;
extern unsigned code_pulse_diff;
extern unsigned N_Pulse;
extern unsigned W_Pulse;
extern unsigned tto;
extern unsigned tts;

extern unsigned syncNW;
extern unsigned syncPN;
extern unsigned syncLEN;

extern unsigned gear_numerator;
extern unsigned gear_denominator;
extern unsigned encoder_scale;

extern unsigned max_inc;

extern unsigned gr_log_size;

extern unsigned restart_step;
extern unsigned short ini_PWM_restart;
extern unsigned short PWM_restart;

extern unsigned short ini_PWM_calib;
extern unsigned short PWM_calib;
extern unsigned skip_count;
extern unsigned open_load_calib;
extern unsigned shut_load_calib;
extern unsigned ref_load_calib;
extern int motor_open_offset;
extern int motor_shut_offset;

extern unsigned Speed_Dn;
extern unsigned MotorHome;

extern unsigned short up_open_PWM_count;
extern unsigned short up_shut_PWM_count;
extern unsigned short dn_open_PWM_count;
extern unsigned short dn_shut_PWM_count;

extern ushort ini_up_open_PWM;
extern ushort ini_up_shut_PWM;

extern ushort up_open_max_PWM;
extern ushort up_shut_max_PWM;

extern ushort up_open_PWM;
extern ushort up_shut_PWM;

extern ushort dn_open_min_PWM;
extern ushort dn_shut_min_PWM;

extern unsigned short motor_freeze_count;
extern unsigned short motor_freeze_counter;

extern unsigned long motor_freeze_rate;

extern unsigned long stop_time;
extern unsigned long data_save_rate;


void LoadStatus()
{
	
    int i;                  // For loop variable
    ushort status_header;    // flash header returned from flash
    ushort status_version;   // Coeff verison returned from flash
    ushort scratch[FLASH_SECTOR_1M];    // Scratch buffer
   
    // Read the header out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_STATUS_ADDR,&status_header);
    // Read the coeff version out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_STATUS_ADDR+2,&status_version);
    
    if((status_header==STATUS_HEADER) && (status_version==STATUS_VERSION))   // Load Coefficients
        ReadFlash(DM_FULL_WORD,sizeof(status),FLASH_STATUS_ADDR+(STATUS_HEADER_SZ*2),status);
    else
    {   // Flash is corrupt or not initialized so do it.
        ReadFlash(DM_FULL_WORD,sizeof(scratch),FLASH_STATUS_ADDR,scratch);
        
        // Load Header and then the constants into the scratch pad
        scratch[0] = STATUS_HEADER;
        scratch[1] = STATUS_VERSION;
        for(i=0; i<NUM_STATUS_CONST; i++)
            scratch[i+STATUS_HEADER_SZ] = status[i] = factory_status[i];
            
        // Save Data to flash
        WriteFlash(DM_FULL_WORD,FLASH_SECTOR_1M,(ulong)FLASH_STATUS_ADDR,scratch);
    }
    
    //flag variables
    Sensor_Live=status[SC_SENSOR_LIVE];
    Motor_Cycle=status[SC_MOTOR_CYCLE];
    TF_Reset=status[SC_TF_RESET];
    
    //status variables
    long_date_time=((ulong)status[SC_DATE_TIME_H]<<16)|(status[SC_DATE_TIME_L]);
    
    pulse_time=status[SC_PULSE_TIME];
    code_pulse_time=status[SC_CODE_PULSE_TIME];
    code_pulse_diff=status[SC_CODE_PULSE_DIFF];
    N_Pulse=status[SC_N_PULSE];
    W_Pulse=status[SC_W_PULSE];
    tts=status[SC_TTS];
    tto=status[SC_TTO];
    
    syncNW=status[SC_SYNCNW];
    syncPN=status[SC_SYNCPN];
    syncLEN=status[SC_SYNCLEN];
    
    gear_numerator=status[SC_GEAR_NUMERATOR];
    gear_denominator=status[SC_GEAR_DENOMINATOR];
    encoder_scale=status[SC_ENCODER_SCALE];
    
    pattern_seq_idx=ini_pattern_seq_idx=status[SC_INI_PATTERN_SEQ_IDX];
    
    rt_buf_size=status[SC_RT_BUF_SIZE];
    dly_rt_count=status[SC_DLY_RT_COUNT];
    dly_nrt_count=status[SC_DLY_NRT_COUNT];
    
    for (i=0;i<NUM_OF_RT;i++) {
    	
    	rt_limit[RT_GX+i]=status[SC_RT_GX+i];
    	
    }
    
    dly_qt_count=status[SC_DLY_QT_COUNT];
    dly_nqt_count=status[SC_DLY_NQT_COUNT];
    
    for (i=0;i<NUM_OF_RT;i++) {
    	
    	qt_limit[RT_GX+i]=status[SC_QT_GX+i];
    	
    }
    
    for (i=0;i<NUM_OF_RT;i++) {
    	
    	rt_ins_limit[RT_GX+i]=status[SC_RT_INS_GX+i];
    	
    }
    
    for (i=0;i<NUM_OF_RT;i++) {
    	
    	qt_ins_limit[RT_GX+i]=status[SC_QT_INS_GX+i];
    	
    }
    
    TF_G_Angle=status[SC_TF_G_ANGLE];
    TF_H_Angle=status[SC_TF_H_ANGLE];
    
    TF_G_Offset=status[SC_TF_G_OFFSET];
    TF_H_Offset=status[SC_TF_H_OFFSET];
    
    TF_Threshold=status[SC_TF_THRESHOLD];
    max_inc=status[SC_MAX_INC];
    
    gr_log_size=status[SC_GR_LOG_SIZE];
    
    restart_step=status[SC_RESTART_STEP];
    PWM_restart=ini_PWM_restart=status[SC_INI_PWM_RESTART];
    
    PWM_calib=ini_PWM_calib=status[SC_INI_PWM_CALIB];
    skip_count=status[SC_SKIP_COUNT];
    open_load_calib=status[SC_OPEN_LOAD_CALIB];
    shut_load_calib=status[SC_SHUT_LOAD_CALIB];
    ref_load_calib=status[SC_REF_LOAD_CALIB];
    motor_open_offset=(int)status[SC_MOTOR_OPEN_OFFSET];
    motor_shut_offset=(int)status[SC_MOTOR_SHUT_OFFSET];
    
    Speed_Dn=status[SC_SPEED_DN];
    MotorHome=status[SC_MOTORHOME];
    
    up_open_PWM_count=status[SC_UP_OPEN_PWM_COUNT];
    up_shut_PWM_count=status[SC_UP_SHUT_PWM_COUNT];
    dn_open_PWM_count=status[SC_DN_OPEN_PWM_COUNT];
    dn_shut_PWM_count=status[SC_DN_SHUT_PWM_COUNT];
    
    up_open_PWM=ini_up_open_PWM=status[SC_INI_UP_OPEN_PWM];
    up_shut_PWM=ini_up_shut_PWM=status[SC_INI_UP_SHUT_PWM];
    
    up_open_max_PWM=status[SC_UP_OPEN_MAX_PWM];
    up_shut_max_PWM=status[SC_UP_SHUT_MAX_PWM];
    
    dn_open_min_PWM=status[SC_DN_OPEN_MIN_PWM];
    dn_shut_min_PWM=status[SC_DN_SHUT_MIN_PWM];
    
    motor_freeze_counter=motor_freeze_count=status[SC_MOTOR_FREEZE_COUNT];
    
    motor_freeze_rate=((ulong)status[SC_MOTOR_FREEZE_RATE_H]<<16)|(status[SC_MOTOR_FREEZE_RATE_L]);
    
    stop_time=((ulong)status[SC_STOP_TIME_H]<<16)|(status[SC_STOP_TIME_L]);
    rt_dly_rate=((ulong)status[SC_RT_DLY_RATE_H]<<16)|(status[SC_RT_DLY_RATE_L]);
    data_fresh_rate=((ulong)status[SC_DATA_FRESH_RATE_H]<<16)|(status[SC_DATA_FRESH_RATE_L]);
    data_save_rate=((ulong)status[SC_DATA_SAVE_RATE_H]<<16)|(status[SC_DATA_SAVE_RATE_L]);
    
    Send_Status();
    
}

//-----------------------------------------------------------------------------
// Set Status Routine
//
// Description:
//      Sets the given coefficient and saves it to flash
//
// Arguments:
//      cc_idx - see Calibration Constant enum
//      val    - value to set coefficient too.
//
// Notes:
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------
void SetStatus(unsigned sc_idx, ushort val)
{
	
	WriteFlash(DM_FULL_WORD, 1, (ulong)(sc_idx*2+FLASH_STATUS_ADDR+STATUS_HEADER_SZ*2), &val);
	status[sc_idx]=val;
	
}

#define COEFF_RESET_COUNT	10		//# of gamma counts above the threshold
unsigned coeff_reset_counter=COEFF_RESET_COUNT;
#define ExpireCoeffResetCounter()	(!coeff_reset_counter)

#define COEFF_AVG_COUNT		10		//# of tool face readings to average
unsigned coeff_avg_counter=COEFF_AVG_COUNT;
#define ExpireCoeffAvgCounter()		(!coeff_avg_counter)	

//Routine to reset some of the paramenters, tool face offset
void CoeffReset()
{
	if(!TF_Reset) {
		
		Motor_Capture=true;
		
		if(ExpireCoeffResetCounter()) {
			
			TF_G_Total+=TF_G;
			TF_H_Total+=TF_H;
				
			if(coeff_avg_counter) coeff_avg_counter--;
			if(ExpireCoeffAvgCounter()) {
					
				TF_G_Offset=(unsigned)(TF_G_Total/(double)COEFF_AVG_COUNT);
				TF_H_Offset=(unsigned)(TF_H_Total/(double)COEFF_AVG_COUNT);
					
				SetStatus(SC_TF_G_OFFSET, TF_G_Offset);
				SetStatus(SC_TF_H_OFFSET, TF_H_Offset);
					
				TF_G_Total=0;
				TF_H_Total=0;
					
				coeff_reset_counter=COEFF_RESET_COUNT;
				coeff_avg_counter=COEFF_AVG_COUNT;
				
				TF_Reset=true;
				SetStatus(SC_TF_RESET,TF_Reset);
					
				Motor_Capture=false;
				
				Send_Status();
			}
				
		}
		
		else {
		
			if(Avg_GRayCPS>=(double)TF_Threshold) {
			
				if(coeff_reset_counter) coeff_reset_counter--;
				
			}
			
		}
		
	}
	
}
			
			
//-----------------------------------------------------------------------------
// Load Pattern Routine
//
// Description:
//      Loads the pattern constant out of flash.
//
// Notes:
//      The Factory settings has a PATTERN_HEADER at the beginning of the
//      memory block.  The first thing I do is read that header and see if
//      it is correct.  If it is NOT correct.  That means either the flash
//      is corrupt or this is a new board.  So I initialize the flash memory
//      with the values in factory_coeff[].
//
//      Also there is a PATTERN_VERSION stored in flash after the PATTERN_HEADER.
//      The number stored on flash is compared with the PATTERN_VERSION defined
//      above.  If this is different, the coeff[] is loaded with factory_coeff[]
//      and the new values stored in flash.
//
//      This routine should only be called once at startup.
//      I am using the initialization values of the array above to
//      load to flash if the header is corrupt.
//
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------
    
void LoadPattern()
{
    int i;                  // For loop variable
    int j;
    ushort pattern_header;    // flash header returned from flash
    ushort pattern_version;   // Coeff verison returned from flash
    ushort scratch[128];    // Scratch buffer
   
    // Read the header out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_PATTERN_ADDR,&pattern_header);
    // Read the coeff version out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_PATTERN_ADDR+2,&pattern_version);
    
    if((pattern_header==PATTERN_HEADER) && (pattern_version==PATTERN_VERSION)) { // Load patterns
    
    	for (i=0;i<NUM_PATTERN_SEQ;i++) {
    
        	ReadFlash(DM_FULL_WORD,sizeof(pattern[i]),FLASH_PATTERN_ADDR+((PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*i*2)*2),pattern[i]);		//load patterns
        	ReadFlash(DM_FULL_WORD,sizeof(pattern_count[i]),FLASH_PATTERN_ADDR+((PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*(i*2+1))*2),pattern_count[i]);		//load pattern counts
        	
    	}
        
    }
        
    else
    {   // Flash is corrupt or not initialized so do it.
        ReadFlash(DM_FULL_WORD,sizeof(scratch),FLASH_PATTERN_ADDR,scratch);
        
        // Load Header and then the constants into the scratch pad
        scratch[0] = PATTERN_HEADER;
        scratch[1] = PATTERN_VERSION;
        for(i=0; i<NUM_PATTERN_SEQ; i++) {
        	
        	for (j=0; j<PATTERN_BUF_SIZE; j++) {
        		
            	scratch[j+PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*i*2] = pattern[i][j] = factory_pattern[i][j];		//load pattern
        		scratch[j+PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*(i*2+1)] = pattern_count[i][j] = factory_pattern_count[i][j];		//load pattern count
        		
        	}
        
        }
           
        // Save Data to flash
        WriteFlash(DM_FULL_WORD,sizeof(scratch),(ulong)FLASH_PATTERN_ADDR,scratch);
    	
    }
    
    for (i=0;i<NUM_PATTERN_SEQ;i++) {
    	
    	//set the pattern to the first one in the buffer
    	pattern_idx[i]=0;
    	pattern_counter[i]=pattern_count[i][pattern_idx[i]];
    	
    }
    
    Send_Pattern();
    
}

//-----------------------------------------------------------------------------
// Set Pattern Routine
//
// Description:
//      Sets the given coefficient and saves it to flash
//
// Arguments:
//      cc_idx - see Calibration Constant enum
//      val    - value to set coefficient too.
//
// Notes:
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------

void SetPattern(ushort pattern_seq, unsigned pc_idx, ushort p_val, ushort pc_val)
{
	
	pattern[pattern_seq][pc_idx]=p_val;
	pattern_count[pattern_seq][pc_idx]=pc_val;
	
	WriteFlash(DM_FULL_WORD, 1, (ulong)((PATTERN_BUF_SIZE*pattern_seq*2+pc_idx)*2+FLASH_PATTERN_ADDR+PATTERN_HEADER_SZ*2), &p_val);
	WriteFlash(DM_FULL_WORD, 1, (ulong)((PATTERN_BUF_SIZE*(pattern_seq*2+1)+pc_idx)*2+FLASH_PATTERN_ADDR+PATTERN_HEADER_SZ*2), &pc_val);
	
}

//-----------------------------------------------------------------------------
// Load Pattern Sequence Routine
//
// Description:
//      Loads the pattern sequence constant out of flash.
//
// Notes:
//      The Factory settings has a PATTERN_HEADER at the beginning of the
//      memory block.  The first thing I do is read that header and see if
//      it is correct.  If it is NOT correct.  That means either the flash
//      is corrupt or this is a new board.  So I initialize the flash memory
//      with the values in factory_coeff[].
//
//      Also there is a PATTERN_VERSION stored in flash after the PATTERN_HEADER.
//      The number stored on flash is compared with the PATTERN_VERSION defined
//      above.  If this is different, the coeff[] is loaded with factory_coeff[]
//      and the new values stored in flash.
//
//      This routine should only be called once at startup.
//      I am using the initialization values of the array above to
//      load to flash if the header is corrupt.
//
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------
    
void LoadPatternSeq()
{
    int i;                  // For loop variable
    unsigned temp_pattern_seq_rate[PATTERN_SEQ_BUF_SIZE*2];
    ushort pattern_seq_header;    // flash header returned from flash
    ushort pattern_seq_version;   // Coeff verison returned from flash
    ushort scratch[128];    // Scratch buffer
   
    // Read the header out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_PATTERN_SEQ_ADDR,&pattern_seq_header);
    // Read the coeff version out of flash
    ReadFlash(DM_FULL_WORD,1,FLASH_PATTERN_SEQ_ADDR+2,&pattern_seq_version);
    
    if((pattern_seq_header==PATTERN_SEQ_HEADER) && (pattern_seq_version==PATTERN_SEQ_VERSION)) { // Load patterns
    
        ReadFlash(DM_FULL_WORD,sizeof(pattern_seq),FLASH_PATTERN_SEQ_ADDR+PATTERN_SEQ_HEADER_SZ*2,pattern_seq);		//load patterns
        ReadFlash(DM_FULL_WORD,sizeof(temp_pattern_seq_rate),FLASH_PATTERN_SEQ_ADDR+(PATTERN_SEQ_HEADER_SZ+PATTERN_SEQ_BUF_SIZE)*2,temp_pattern_seq_rate);		//load pattern counts
        
    }
        
    else
    {   // Flash is corrupt or not initialized so do it.
        ReadFlash(DM_FULL_WORD,sizeof(scratch),FLASH_PATTERN_SEQ_ADDR,scratch);
        
        // Load Header and then the constants into the scratch pad
        scratch[0] = PATTERN_SEQ_HEADER;
        scratch[1] = PATTERN_SEQ_VERSION;
        	
        for (i=0; i<PATTERN_SEQ_BUF_SIZE; i++) {
        		
            scratch[PATTERN_SEQ_HEADER_SZ+i] = pattern_seq[i] = factory_pattern_seq[i];		//load pattern sequence
        	scratch[PATTERN_SEQ_HEADER_SZ+PATTERN_SEQ_BUF_SIZE+i*2] = temp_pattern_seq_rate[i*2] = factory_pattern_seq_rate[i*2];		//load pattern sequence time
        	scratch[PATTERN_SEQ_HEADER_SZ+PATTERN_SEQ_BUF_SIZE+i*2+1] = temp_pattern_seq_rate[i*2+1] = factory_pattern_seq_rate[i*2+1];		//load pattern sequence time
        
        }
           
        // Save Data to flash
        WriteFlash(DM_FULL_WORD,sizeof(scratch),(ulong)FLASH_PATTERN_SEQ_ADDR,scratch);
    	
    }
    
    for (i=0;i<PATTERN_SEQ_BUF_SIZE;i++) {
    	
    	//set the pattern to the first one in the buffer
    	pattern_seq_rate[i]=((ulong)temp_pattern_seq_rate[i*2]<<16)|(temp_pattern_seq_rate[i*2+1]);
    	
    }
        
    Send_Pattern_Seq();
    
}

//-----------------------------------------------------------------------------
// Set Pattern Sequence Routine
//
// Description:
//      Sets the given coefficient and saves it to flash
//
// Arguments:
//      cc_idx - see Calibration Constant enum
//      val    - value to set coefficient too.
//
// Notes:
//      You have to write an entire sector of data.
//      So I read the entire sector into memory, change the data
//      and write it back.
//-----------------------------------------------------------------------------

void SetPatternSeq(unsigned seq_idx, ushort q_val, ulong seq_val)
{
	
	unsigned val;
	
	pattern_seq[seq_idx]=q_val;
	pattern_seq_rate[seq_idx]=seq_val;
	
	WriteFlash(DM_FULL_WORD, 1, (ulong)(seq_idx*2+FLASH_PATTERN_SEQ_ADDR+PATTERN_SEQ_HEADER_SZ*2), &q_val);
	val=seq_val>>16;
	WriteFlash(DM_FULL_WORD, 1, (ulong)((PATTERN_SEQ_BUF_SIZE+seq_idx*2)*2+FLASH_PATTERN_SEQ_ADDR+PATTERN_SEQ_HEADER_SZ*2), &val);
	val=(seq_val<<16)>>16;
	WriteFlash(DM_FULL_WORD, 1, (ulong)((PATTERN_SEQ_BUF_SIZE+seq_idx*2+1)*2+FLASH_PATTERN_SEQ_ADDR+PATTERN_SEQ_HEADER_SZ*2), &val);
	
}

