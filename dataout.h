//	Revision History:
//  001	11/11/03	Original Version
//  002	12/01/06	D Lerner Add Motor_controller, delete UpdateOutput
#ifndef DATAOUT_H
#define DATAOUT_H

//void    ServOutput();
    // Outputs the next bit in the sequence on the flag pin.
    
//void 	Motor_controller();   // run the motor controller
	// controls the motor position and speed
	
//void    Data2Frame();
    // FormatMudPulses into the transmit array

#endif

#define MotorSTOP()	asm("SET FL0;")	//set the FL0 to not STOP the motor
#define MotorRUN()	asm("RESET FL0;")	//clear the FL0 to not STOP the motor

// assembler calls to control the line used for motor direction
#define MotorCW()		asm(" SET FL2;")
#define MotorCCW()		asm(" RESET FL2;")

//-----------------------------------------------------------------------------
// DATA SEQUENCE ARRAY
//      There are two arrays.
//      data_seq - holds the actual data of the sequence
//      data_size - holds the size in bits of the data in the sequence
//
//-----------------------------------------------------------------------------

/*
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


#define SEQ_HTIME		0		//higher 16 bits of the time
#define SEQ_LTIME   	1		//lower 16 bits of the time
#define SEQ_FRAME_ID    2
#define SEQ_DIAGNOSIC	3	 	//future diagnostic check data
#define SEQ_INC         4
#define SEQ_AZI         5
#define SEQ_G           6
#define SEQ_H           7
#define SEQ_GR          8
#define SEQ_TF	        9
#define SEQ_GX	        10
#define SEQ_GY	        11
#define SEQ_GZ	        12
#define SEQ_HX	        13
#define SEQ_HY	        14
#define SEQ_HZ	        15
#define SEQ_TEMP        16
#define	SEQ_PRES		17
#define NUM_OF_SEQ      18

#define LOG_HTIME		0		//higher 16 bits of the time
#define LOG_LTIME   	1		//lower 16 bits of the time
#define LOG_DIAGNOSIC	2	 	//future diagnostic check data
#define LOG_GX	        3
#define LOG_GY	        4
#define LOG_GZ	        5
#define LOG_HX	        6
#define LOG_HY	        7
#define LOG_HZ	        8
#define LOG_TEMP        9
#define NUM_OF_LOG      10

#define NUM_OF_CODE		10		//bar code from 0 - 9
#define CODE_LEN		5		//number of pulses for a single code

#define TS_TOOL_ROTATE			0
#define	TS_TOOL_QUIET			1
#define TS_MOTOR_FREEZE			2
#define NUM_OF_TS				3

//#define GEAR_NUMERATOR		16224		//nominator of gear ratio
//#define GEAR_DENOMINATOR	245			//denominator of gear ratio

//#define GEAR_NUMERATOR		624		//nominator of gear ratio
//#define GEAR_DENOMINATOR	35			//denominator of gear ratio

//#define ENCODER_SCALE 4	//encoder counts per motor revolution

//#define LOAD_BUF_SIZE		(long)GEAR_NUMERATOR*(long)ENCODER_SCALE/(long)GEAR_DENOMINATOR+1		//buffer size for the motor velocity
#define LOAD_BUF_SIZE		300
//#define LOAD_BUF_SIZE		71		//size of the motor load buffer, it is the counts per full out-gear turn
//#define MAX_LOAD_BUF_SIZE	300		//(ulong)gear_numerator*(ulong)encoder_scale/(ulong)gear_denominator		//buffer size for the motor velocity

#define	GRBIN_BUF_SIZE		20		//size of thegamma ray bin buffer

#define DATA_BUF_SIZE		100		//size of the buffer to store logging data

#define GR_BUF_SIZE		60				//size of the GR recording buffer

#define DATA_SIZE	(NUM_OF_LOG+gr_log_size)

#define YEAR			0
#define MONTH			1
#define DAY				2
#define HOUR			3
#define	MINUTE			4
#define	SECOND			5
#define NUM_OF_DATE		6

//#define INI_YEAR		7		//year from 2000
//#define INI_MONTH		6		//month in the year 1-12
//#define INI_DAY			15		//day in the month 1-30
//#define INI_HOUR		0		//hour in the day 0-23
//#define INI_MINUTE		30		//minute in the hour 0-59
//#define INI_SECOND		0		//second in the minute 0-59
