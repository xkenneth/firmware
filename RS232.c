//-----------------------------------------------------------------------------
// RS232 Communications File
//-----------------------------------------------------------------------------
//
// Rev 01	02/02/06	Loop RS232Serv till Rx buffer is empty to improve comm
// Rev 02	02/19/07	RS232Serv totally revised, motor, pulse, sensor functions are working
//comment4
#include "RS232.h"
#include "swuart.h"
#include "peterc.h"
#include "flash.h"
#include "dataout.h"
#include "timer.h"
#include "boardcheckout1.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern	short	avg1,avg2,avg3,avg4,avg5,avg6;
extern  short   avg7,avg8,avg9,avg10,avg11;     /* Averages from BoardCheckout1.asm */
//extern  ushort  seq_count;   // Counts how many times a sequence was saved to flash

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define MAX_DATA_PER_CMD    7
#define ACKNOWLEDGE         0x5A


/*
//-----------------------------------------------------------------------------
//	COMMAND LIST
//
//		Order is important
//
//		If you add a command that requires more data bytes
//		than MAX_DATA_PER_CMD, you must increase this Constant.
//-----------------------------------------------------------------------------
enum {CMD_NOP=0x0030,				// #0  - NOP
      CMD_GET_SENSOR_AVG,   // #1  - Returns Sesnor avg(1-11)
      CMD_GET_FLASH_BYTE,	// #2  - Returns byte from flash at given address
      CMD_SET_COEFF,		// #3  - Sets the given coefficients
      CMD_SET_DATE_TIME,    // #4  - Sets the given date and time
      CMD_GET_DATA_SEQ,     // #5  - Returns a data sequence at given address
      CMD_GET_NUM_SEQ_SAVED,// #6  - Returns the number of sequences saved to flash
      NUM_OF_COMMANDS		// Total Number Of Commands
     };
*/

//-----------------------------------------------------------------------------
//	Sensor Output Averages
//-----------------------------------------------------------------------------
enum {AVG_1=1,AVG_2,AVG_3,AVG_4,AVG_5,AVG_6,AVG_7,AVG_8,AVG_9,AVG_10,AVG_11};

ACCESS_TIMERS;

#define CMD_BUF_SIZE	100		//size of the command buffer
uchar CMD_Buf[CMD_BUF_SIZE];	//command buffer
int cmd_idx=0;		//command buffer index


uchar LowerUpper(uchar lower_ch)	//transform from lower case to upper case
{
	
	uchar upper_ch;
	
	if ((lower_ch>=CHa)&(lower_ch<=CHz))	//transform from lower case to upper case
		
		upper_ch=lower_ch-0x0020;
		
	else
	
		upper_ch=lower_ch;
		
	return(upper_ch);
		
}


uchar ASC2HEX(uchar asc_ch)		//transform a ASCII charater into a HEX number
{
	uchar hex_ch;
	
	if ((asc_ch>=CHa)&(asc_ch<=CHf))	//transform from lower case ASCII a-f to HEX
		
		hex_ch=asc_ch-0x0057;
		
	else if ((asc_ch>=CHA)&(asc_ch<=CHF))	//transform from upper case ASCII A-F to HEX
		
		hex_ch=asc_ch-0x0037;
			
	else if ((asc_ch>=CH0)&(asc_ch<=CH9))
	
		hex_ch=asc_ch-0x0030;		//transform from ASCII 0-9 numbers to HEX
		
	else
	
		hex_ch=asc_ch;
		
	return(hex_ch);
		
}
	

uchar HEX2ASC(uchar hex_ch)		//transform a HEX number into a ASCII charater
{
	uchar asc_ch;
	
	if ((hex_ch>=0x000A)&(hex_ch<=0x000F))
		
		asc_ch=hex_ch+0x0037;		//transform from HEX number A-F to ASCII code
			
	//else if ((hex_ch>=0x0000)&(hex_ch<=0x0009))
	
	else if (hex_ch<=0x0009)
	
		asc_ch=hex_ch+0x0030;		//transform from HEX number 0-9 to ASCII code
		
	else
	
		asc_ch=hex_ch;
		
	return(asc_ch);
		
}

unsigned long Read_ASC()		//read value from the ASCII input
{
	
	unsigned long hex_val=0;
			
	while (CMD_Buf[cmd_idx]==SP) {		//dump space before the input number
		
		cmd_idx++;
		
	}
	
	while ((CMD_Buf[cmd_idx]!=SP)&(CMD_Buf[cmd_idx]!=CR)) {		//read the input nuber untill space or cr
	
		hex_val=(hex_val<<4)|(ASC2HEX(CMD_Buf[cmd_idx++]));
		
	}
	
	return(hex_val);
	
}

void Send_ASC(uchar asc_val)		//display a ASCII character, defined by asc_val
{
	
	UARTPutChar(HIBYTE(asc_val));
    UARTPutChar(LOBYTE(asc_val));
    
}


void Send_Int(unsigned long int_val)		//display a 32bit integer in HEX format, sign not included
{
	uchar asc_val;
	unsigned med_val;
	
	if(int_val>0x0000FFFF) {
		
		asc_val=HEX2ASC((int_val & 0xF0000000) >>28);
		Send_ASC(asc_val);
		
		asc_val=HEX2ASC((int_val & 0x0F000000) >>24);
		Send_ASC(asc_val);
		
		asc_val=HEX2ASC((int_val & 0x00F00000) >>20);
		Send_ASC(asc_val);
		
		asc_val=HEX2ASC((int_val & 0x000F0000) >>16);
		Send_ASC(asc_val);
		
		med_val=int_val & 0x0000FFFF;
		
	}
	
	else
	
		med_val=int_val;
		
	
	asc_val=HEX2ASC((med_val & 0xF000) >>12);
	Send_ASC(asc_val);
	
	asc_val=HEX2ASC((med_val & 0x0F00) >>8);
	Send_ASC(asc_val);
	
	asc_val=HEX2ASC((med_val & 0x00F0) >>4);
	Send_ASC(asc_val);
	
	asc_val=HEX2ASC(med_val & 0x000F);
	Send_ASC(asc_val);
	
}
	
void Send_CRLF()		//display CR and LF

{

	Send_ASC(CR);	
	Send_ASC(LF);
    
}

void Send_Head()		//display customized headers
{
	
	Send_ASC(RT);
    Send_ASC(RT);
    Send_ASC(SP);
    
}

bool CMD_ESC = false;
short CMD_Counter = 0;

extern long motor_open_position;
extern long motor_neutral_position;					
extern long motor_shut_position;					
extern long	motor_target;
extern int motor_position;

extern unsigned gear_numerator;
extern unsigned gear_denominator;

extern unsigned encoder_scale;

extern unsigned rt_buf_size;
extern unsigned dly_rt_count;
extern unsigned dly_nrt_count;
extern unsigned rt_limit[NUM_OF_RT];
extern unsigned rt_ins_limit[NUM_OF_RT];

extern unsigned dly_qt_count;
extern unsigned dly_nqt_count;
extern unsigned qt_limit[NUM_OF_RT];
extern unsigned qt_ins_limit[NUM_OF_RT];

extern unsigned TF_G_Angle;
extern unsigned TF_H_Angle;
extern unsigned TF_G_Offset;
extern unsigned TF_H_Offset;
extern unsigned TF_Threshold;
extern unsigned max_inc;

extern unsigned gr_log_size;

extern unsigned restart_step;
extern unsigned short ini_PWM_restart;
extern unsigned short PWM_restart;

extern unsigned short ini_PWM_calib;
extern unsigned short PWM_calib;
extern unsigned short skip_count;
extern unsigned open_load_calib;
extern unsigned shut_load_calib;
extern unsigned ref_load_calib;
extern int motor_open_offset;
extern int motor_shut_offset;

extern unsigned Speed_Dn;
extern unsigned MotorHome;

//extern unsigned acc_load_buf[LOAD_BUF_SIZE];
extern unsigned load_buf[LOAD_BUF_SIZE];

extern unsigned GRBin_CPS[GRBIN_BUF_SIZE];

extern unsigned GRayCPS;

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

extern short mudpulses; 

extern unsigned data_seq[NUM_OF_SEQ];
extern unsigned short date_time[NUM_OF_DATE];

extern unsigned long stop_time;

extern unsigned long data_save_addr;
extern unsigned long data_save_rate;
extern unsigned long rt_dly_rate;
extern unsigned long data_fresh_rate;

extern ushort up_open_PWM_count;
extern ushort up_shut_PWM_count;

extern ushort dn_open_PWM_count;
extern ushort dn_shut_PWM_count;

extern ushort ini_up_open_PWM;
extern ushort ini_up_shut_PWM;

extern ushort up_open_max_PWM;
extern ushort up_shut_max_PWM;

extern ushort dn_open_min_PWM;
extern ushort dn_shut_min_PWM;

extern bool Motor_Freeze;

extern unsigned short motor_freeze_count;
extern unsigned short motor_freeze_counter;

extern unsigned long motor_freeze_rate;

extern bool Motor_Capture;
extern bool Sensor_Live;
extern bool Motor_Cycle;
extern bool TF_Reset;

extern unsigned short status[NUM_STATUS_CONST];

extern unsigned pattern_seq[PATTERN_SEQ_BUF_SIZE];
extern unsigned long pattern_seq_rate[PATTERN_SEQ_BUF_SIZE];
extern unsigned ini_pattern_seq_idx;

#define ROW_COUNT		30		//number of numbers to display in a row
unsigned short row_counter = ROW_COUNT;
#define ExpireRowCounter()		(!row_counter)

void Send_Message(unsigned message_size, unsigned *message)
{
	
	int i;
	
	Send_Head();
	
	for (i=0;i<message_size;i++) Send_ASC(*(message+i));
	
	Send_CRLF();
	
}

void Send_Value(ulong value)		//send a single value through RS232
{
	
	if (!CMD_ESC) {
	
		Send_Head();
		Send_Int(value);
		Send_CRLF();
		
	}
	
}

void Send_Array(unsigned *Array, unsigned Array_Size)		//send a array of data through certain format
{

	int i;
	
	row_counter = ROW_COUNT;
	
	Send_Head();
	
	for (i=0; i<Array_Size; i++) {
		
		if (!CMD_ESC) {
		
			Send_Int(*(Array+i));
			Send_ASC(SP);
		
			if (row_counter) row_counter--;
		
			if (ExpireRowCounter()) {
			
				row_counter=ROW_COUNT;
				Send_CRLF();
				Send_ASC(SP);
				Send_ASC(SP);
				Send_ASC(SP);
				
			}
			
		}
		
		else break;
		
	}
	
	Send_CRLF();
	
}

void Send_Time()			//send current time
{
	
	Send_Head();
	
	Send_Int(date_time[YEAR]);
	Send_ASC(SP);
	Send_Int(date_time[MONTH]);
	Send_ASC(SP);
	Send_Int(date_time[DAY]);
	Send_ASC(SP);
	Send_Int(date_time[HOUR]);
	Send_ASC(SP);
	Send_Int(date_time[MINUTE]);
	Send_ASC(SP);
	Send_Int(date_time[SECOND]);
	Send_ASC(SP);
	Send_CRLF();
	
}

void Send_Log_Data()		//send all data in flash starting from FLASH_DATA_ADDR
{
		
	int i;
	unsigned long data_addr = FLASH_LOG_ADDR;		//read only the logging data
	unsigned temp_data[DATA_SIZE];
		
	row_counter = DATA_SIZE;
	
	Motor_Capture=true;
	Sensor_Live=false;
	
	//Send_ASC(CR);
	
	while ((data_addr<MAX_FLASH_SIZE-DATA_SIZE)&(!CMD_ESC)) {
		
		ReadFlash(DM_FULL_WORD,sizeof(temp_data),data_addr,temp_data);
		
		for (i=0;i<DATA_SIZE;i++) {
			
			if(!CMD_ESC) {
			
				Send_Int(temp_data[i]);
				Send_ASC(TAB);
		
				if (row_counter) row_counter--;
		
				if (ExpireRowCounter()) {
			
					row_counter=DATA_SIZE;
					Send_CRLF();			//in Linux mode, minicom
					//Send_ASC(CR);			//in Windows mode, hyper terminal, use internal
											//new line feed in terminal program
					
				}
				
			}
			
			else break;
			
		}
		
		data_addr=data_addr+DATA_SIZE*2;
		
	}
	
	Send_CRLF();
	Send_CRLF();
		
}

void Send_Calib_Status()		//display calibration information
{
	
	Send_Head();
	
	Send_Int(PWM_calib);
	Send_ASC(SP);
	Send_Int(motor_open_position);
	Send_ASC(SP);
	Send_Int(motor_neutral_position);
	Send_ASC(SP);
	Send_Int(motor_shut_position);
	
	Send_CRLF();
	
}

void Send_Calib_Load()		//display calibration load information
{
	int i;
	unsigned temp_load[LOAD_BUF_SIZE];
	unsigned temp_load_pos[LOAD_BUF_SIZE];
	
	ReadFlash(DM_FULL_WORD,LOAD_BUF_SIZE,FLASH_LOAD_ADDR,temp_load);
	ReadFlash(DM_FULL_WORD,LOAD_BUF_SIZE,FLASH_POS_ADDR,temp_load_pos);
	
	/*
	Send_Head();
	
	for (i=0;i<LOAD_BUF_SIZE;i++) {
		
		if(row_counter) row_counter--;
				
		Send_Int(temp_load[i]);
		Send_CRLF();
		
	}
	
	Send_CRLF();
	*/
	
	Send_Array(temp_load, LOAD_BUF_SIZE);
	Send_Array(temp_load_pos, LOAD_BUF_SIZE);
	
}

void Send_Coeff()		//display caliberation coefficients
{
	
	int i;
    
    unsigned temp_coeff[NUM_CALIB_CONST+FLASH_HEADER_SZ];
    
    ReadFlash(DM_FULL_WORD,sizeof(temp_coeff),FLASH_FACTORY_ADDR,temp_coeff);	
    
	Send_Array(temp_coeff, NUM_CALIB_CONST+FLASH_HEADER_SZ);
	
}

void Send_Status()		//display status coefficients
{
	
	int i;
    
    unsigned temp_status[NUM_STATUS_CONST+STATUS_HEADER_SZ];
    
    ReadFlash(DM_FULL_WORD,sizeof(temp_status),FLASH_STATUS_ADDR,temp_status);	
    
	Send_Array(temp_status, NUM_STATUS_CONST+STATUS_HEADER_SZ);
	
}

void Send_Pattern()		//display patterns
{
	
	int i;
    
    unsigned temp_pattern[PATTERN_BUF_SIZE];
    
    for (i=0;i<NUM_PATTERN_SEQ;i++) {
    
    	ReadFlash(DM_FULL_WORD,sizeof(temp_pattern),FLASH_PATTERN_ADDR+(PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*i*2)*2,temp_pattern);	
    
		Send_Array(temp_pattern, PATTERN_BUF_SIZE);
	
		ReadFlash(DM_FULL_WORD,sizeof(temp_pattern),FLASH_PATTERN_ADDR+(PATTERN_HEADER_SZ+PATTERN_BUF_SIZE*(i*2+1))*2,temp_pattern);	
    
		Send_Array(temp_pattern, PATTERN_BUF_SIZE);
    }
	
}

void Send_Pattern_Seq()		//display pattern sequences
{
    
    unsigned temp_pattern_seq[PATTERN_SEQ_BUF_SIZE];
    unsigned temp_pattern_seq_rate[PATTERN_SEQ_BUF_SIZE*2];
    
    ReadFlash(DM_FULL_WORD,sizeof(temp_pattern_seq),FLASH_PATTERN_SEQ_ADDR+PATTERN_SEQ_HEADER_SZ*2,temp_pattern_seq);	
    
	Send_Array(temp_pattern_seq, PATTERN_SEQ_BUF_SIZE);
	
	ReadFlash(DM_FULL_WORD,sizeof(temp_pattern_seq_rate),FLASH_PATTERN_SEQ_ADDR+(PATTERN_SEQ_HEADER_SZ+PATTERN_SEQ_BUF_SIZE)*2,temp_pattern_seq_rate);	
    
	Send_Array(temp_pattern_seq_rate, PATTERN_SEQ_BUF_SIZE*2);
	
}

void CMD_Read()		//read the command out of the serial buffer and store into the command buffer
{
	
	int i=0;
	uchar rx_val=0;
	
	while (rx_val!=CR) {
		
		rx_val = UARTGetChar();
		rx_val = LowerUpper(rx_val);
		CMD_Buf[i++]=rx_val;
		
	}
		
}
	
	
	
//-----------------------------------------------------------------------------
// RS232 SERVICE ROUTINE
//-----------------------------------------------------------------------------
void RS232Serv()
{
	
	unsigned val=0;
	unsigned temp_val=0;
	unsigned alt_val=0;
	unsigned long long_val=0;
	unsigned long addr=0;
	
	if(CMD_Counter>0) {
		
		CMD_Read();
		
		cmd_idx=0;
		
		switch(CMD_Buf[cmd_idx++]) {
				
			/***********************/
			/*Test echo command*****/
			/***********************/
			
			case CHE:		//Echo command
			
				Send_Head();
				
				while (CMD_Buf[cmd_idx]!=CR) {
					
					Send_ASC(CMD_Buf[cmd_idx++]);
					
				}
				
				Send_CRLF();
    		    
				break;
				
			/**********************/
			/*Time set command*****/
			/**********************/
			
			case CHT:
							
				switch(CMD_Buf[cmd_idx++]) {
				
					case CR:		//display current Time
					
						DateTimeCalc();
						Send_Time();
						
						break;
					
					case CHY:		//Time Year set
							
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[YEAR]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
							
						}
								
						break;
						
					case CHM:		//Time Month set
						
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[MONTH]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
							
						}
						
						break;
						
					case CHD:		//Time Day set
							
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[DAY]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
							
						}
						
						break;
						
					case CHH:		//Time Hour set
						
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[HOUR]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
							
						}
						
						break;
						
					case CHN:		//Time miNute set
							
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[MINUTE]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
							
						}
						
						break;
						
					case CHS:		//Time Second set
						
						if (CMD_Buf[cmd_idx]==SP) {
									
							DateTimeCalc();
							date_time[SECOND]=Read_ASC();
							DateTimeUpdate();
							
							Send_Time();
						}
						
						break;
						
					case CHR:		//Time of data save rate
					
						if (CMD_Buf[cmd_idx]==SP) {
									
							data_save_rate=Read_ASC();
							val=(unsigned)(data_save_rate>>16);
							SetStatus(SC_DATA_SAVE_RATE_H,val);
							val=(unsigned)((data_save_rate<<16)>>16);
							SetStatus(SC_DATA_SAVE_RATE_L,val);
							
							Send_Status();
							
						}
						
						break;
						
					case CHT:		//Time of tool roTate delay rate
					
						if (CMD_Buf[cmd_idx]==SP) {
									
							rt_dly_rate=Read_ASC();
							val=(unsigned)(rt_dly_rate>>16);
							SetStatus(SC_RT_DLY_RATE_H,val);
							val=(unsigned)((rt_dly_rate<<16)>>16);
							SetStatus(SC_RT_DLY_RATE_L,val);
							
							Send_Status();
							
						}
						
						break;
						
						
					case CHF:		//Time of data Fresh rate
					
						if (CMD_Buf[cmd_idx]==SP) {
									
							data_fresh_rate=Read_ASC();
							val=(unsigned)(data_fresh_rate>>16);
							SetStatus(SC_DATA_FRESH_RATE_H,val);
							val=(unsigned)((data_fresh_rate<<16)>>16);
							SetStatus(SC_DATA_FRESH_RATE_L,val);
							
							Send_Status();
							
						}
						
						break;
						
					default:
					
						break;
						
				}
				
				break;
				
			/***********************/
			/*read memory command***/
			/***********************/
				
			case CHR:
				
				switch(CMD_Buf[cmd_idx++]) {
				
							
					case CHL:		//Read Logging data from flash
													
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Log_Data();
							
						}
						
						break;
						
					case CHS:		//Read data Save address
																		
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Value(data_save_addr);
							
						}
								
						break;
					
					case CHC:		//Read Coefficients from flash
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Coeff();
							
						}
								
						break;
						
					case CHP:		//Read Pattern from flash
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Pattern();
							
						}
								
						break;
						
					case CHQ:		//Read pattern seQuence from flash
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Pattern_Seq();
							
						}
								
						break;
						
					case CHT:		//Read sTatus constants
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Status();
							
						}
								
						break;
						
					case CHD:		//Read loaD record from flash
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Send_Calib_Load();
							
						}
								
						break;
						
					case CHF:		//Read Flash a word
						
						if (CMD_Buf[cmd_idx]==SP) {
									
							addr=Read_ASC();
							
							ReadFlash(DM_FULL_WORD,1,addr,&val);
							
							Send_Value(val);
							
						}
						
						break;
							
					default:
							
						break;
						
				}
				
				break;
				
			/***********************/
			/*write memory command**/
			/***********************/
				
			case CHW:

				switch(CMD_Buf[cmd_idx++]) {
					
					case CHE:		//Write to Erase the whole flash
						
						if (CMD_Buf[cmd_idx]==CR) {
							
							EraseData();
							
						}
						
						break;
						
					case CHC:		//Write Coefficient to flash	
						
						if (CMD_Buf[cmd_idx]==SP) {
							
							addr=Read_ASC();		//read the address string
							
							if (addr<NUM_CALIB_CONST) {		//address corresponds to the coefficient address
							
								val=Read_ASC();		//read the value string
								
								SetCoeff(addr,val);		//set coefficient value
								
							}
							
							Send_Coeff();
							
						}

						break;
					
					case CHR:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case SP:	//Write Rotation limit constant
							
								addr=Read_ASC();		//read the address string
							
								if (addr<NUM_OF_RT) {		//address corresponds to the rotation coefficient address
							
									rt_limit[RT_GX+addr]=Read_ASC();		//read the value string
								
								
									SetStatus((SC_RT_GX+addr),rt_limit[RT_GX+addr]);	//set rotation coefficient value
								
								}
							
								Send_Status();
								
								break;
								
							case CHI:		//Write Rotation Instantaneous limit
							
								if (CMD_Buf[cmd_idx]==SP) {
							
									addr=Read_ASC();		//read the address string
							
									if (addr<NUM_OF_RT) {		//address corresponds to the rotation coefficient address
							
										rt_ins_limit[RT_GX+addr]=Read_ASC();		//read the value string
								
								
										SetStatus((SC_RT_INS_GX+addr),rt_ins_limit[RT_GX+addr]);	//set rotation coefficient value
							
									}
							
									Send_Status();
							
								}

								break;
								
							default:
							
								break;
								
						}
							

						break;
						
					case CHU:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case SP:	//Write qUiet limit constant
							
								addr=Read_ASC();		//read the address string
							
								if (addr<NUM_OF_RT) {		//address corresponds to the rotation coefficient address
							
									qt_limit[RT_GX+addr]=Read_ASC();		//read the value string
								
								
									SetStatus((SC_QT_GX+addr),qt_limit[RT_GX+addr]);	//set rotation coefficient value
								
								}
							
								Send_Status();
								
								break;
								
							case CHI:		//Write qUiet Instantaneous limit
							
								if (CMD_Buf[cmd_idx]==SP) {
							
									addr=Read_ASC();		//read the address string
							
									if (addr<NUM_OF_RT) {		//address corresponds to the rotation coefficient address
							
										qt_ins_limit[RT_GX+addr]=Read_ASC();		//read the value string
								
								
										SetStatus((SC_QT_INS_GX+addr),qt_ins_limit[RT_GX+addr]);	//set rotation coefficient value
							
									}
							
									Send_Status();
							
								}

								break;
								
							default:
							
								break;
								
						}
							

						break;
						
					case CHP:		//Write Pattern to flash
					
						if (CMD_Buf[cmd_idx]==SP) {
							
							val=Read_ASC();
							
							if (val<NUM_PATTERN_SEQ) {
								
								addr=Read_ASC();		//read the address string
							
								if (addr<PATTERN_BUF_SIZE) {		//address corresponds to the pattern buffer address
							
									temp_val=Read_ASC();		//read the pattern string
								
									if ((temp_val<NUM_PATTERN_CONST)|(temp_val==0x0FFFF)) {
									
										alt_val=Read_ASC();		//read the pattern string
								
										SetPattern(val,addr,temp_val,alt_val);		//set coefficient value
									
									}
									
								}
						
							}
							
							Send_Pattern();
							
						}

						break;
						
					case CHQ:		//Write pattern seQuence to flash
					
						if (CMD_Buf[cmd_idx]==SP) {
								
							addr=Read_ASC();		//read the address string
							
							if (addr<PATTERN_SEQ_BUF_SIZE) {		//address corresponds to the pattern buffer address
							
								val=Read_ASC();		//read the pattern string
								
								if ((val<NUM_PATTERN_SEQ)|(val==0x0FFFF)) {
									
									long_val=Read_ASC();		//read the pattern string
								
									SetPatternSeq(addr,val,long_val);		//set coefficient value
									
								}
						
							}
							
							Send_Pattern_Seq();
							
						}

						break;
						
					case CHF:		//Write Flash a word
						
						if (CMD_Buf[cmd_idx]==SP) {
							
							addr=Read_ASC();		//read the address string
							
							if (addr>=FLASH_FACTORY_ADDR) {		//address corresponds to the flash data address
							
								val=Read_ASC();		//read the value string
								
								WriteFlash(DM_FULL_WORD,1,(ulong)addr,&val);
								
							}
							
							ReadFlash(DM_FULL_WORD,1,addr,&val);
							
							Send_Value(val);
							
						}
						
						break;
					
					default:
							
						break;
						
				}
				
				break;
				
				
			/************************/
			/*Sensor related command*/
			/************************/
			
			case CHS:
				
				switch(CMD_Buf[cmd_idx++]) {
					
					case CR:
					
						Send_Array(data_seq, NUM_OF_SEQ);
						
						break;
						
					case CHG:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CR:		//Sensor Gamma ray bin counts
									
								Send_Value(GRayCPS);
								Send_Array(GRBin_CPS, GRBIN_BUF_SIZE);
									
								break;
								
							case CHB:		//Sensor Gamma ray Buffer log size
						
								if (CMD_Buf[cmd_idx]==SP) {
									
									gr_log_size=Read_ASC();
									SetStatus(SC_GR_LOG_SIZE,gr_log_size);
							
									Send_Status();
							
								}
								
							default:
							
								break;
								
						}
								
						break;
							
					case CHL:		//Sensor Live data
						
						if (CMD_Buf[cmd_idx]==CR) {
									
							Sensor_Live=!Sensor_Live;
							
							SetStatus(SC_SENSOR_LIVE,Sensor_Live);
							
							Send_Status();
							
						}
						
						break;
						
					case CHI:		//Sensor max Inclination
					
						if (CMD_Buf[cmd_idx]==SP) {
									
							max_inc=Read_ASC();
							SetStatus(SC_MAX_INC,max_inc);
							
							Send_Status();
																					
						}
								
						break;
						
					case CHT:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHT:		//Sensor Tool face Threshold
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									TF_Threshold=Read_ASC();
									SetStatus(SC_TF_THRESHOLD,TF_Threshold);
							
									Send_Status();
								
								}
										
								break;
															
							case CHR:		//Sensor Tool face Reset
							
								if (CMD_Buf[cmd_idx]==CR) {
									
									TF_Reset=!TF_Reset;
									SetStatus(SC_TF_RESET,TF_Reset);
							
									Send_Status();
								
								}
										
								break;
							
							case CHG:
					
								switch(CMD_Buf[cmd_idx++]) {
									
									case SP:		//Sensor Tool face Gravity angle
									
										TF_G_Angle=Read_ASC();
										SetStatus(SC_TF_G_ANGLE,TF_G_Angle);
							
										Send_Status();
										
										break;
										
									case CHO:		//Sensor Tool face Gravity angle Offset
									
										if (CMD_Buf[cmd_idx]==SP) {
									
											TF_G_Offset=Read_ASC();
											SetStatus(SC_TF_G_OFFSET,TF_G_Offset);
							
											Send_Status();
								
										}
										
										break;
										
									default:
									
										break;
								
								}
								
								break;
								
							case CHH:
							
								switch(CMD_Buf[cmd_idx++]) {
									
									case SP:		//Sensor Tool face Magnetic angle
									
										TF_H_Angle=Read_ASC();
										SetStatus(SC_TF_H_ANGLE,TF_H_Angle);
							
										Send_Status();
										
										break;
										
									case CHO:		//Sensor Tool face Magnetic angle Offset
									
										if (CMD_Buf[cmd_idx]==SP) {
									
											TF_H_Offset=Read_ASC();
											SetStatus(SC_TF_H_OFFSET,TF_H_Offset);
							
											Send_Status();
								
										}
										
										break;
										
									default:
									
										break;
								
								}
								
								break;
								
							default:
							
								break;
																					
						}
								
						break;
						
					default:
							
						break;
						
				}
				
				break;

				
			/***********************/
			/*Pulse related command*/
			/***********************/
			
			case CHP:
				
				switch(CMD_Buf[cmd_idx++]) {
					
					case CHN:		//Pulse Number display
						
						if (CMD_Buf[cmd_idx]==CR) {
							
							Send_Value(mudpulses);
							
						}

						break;
						
					case CHI:		
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case SP:		//Pulse tIme
															
								pulse_time=Read_ASC();
								SetStatus(SC_PULSE_TIME,pulse_time);
							
								Send_Status();
								
								break;
								
							case CHC:		//Pulse tIme Code
							
								code_pulse_time=Read_ASC();
								SetStatus(SC_CODE_PULSE_TIME,code_pulse_time);
							
								Send_Status();
								
								break;
								
							case CHD:		//Pulse tIme code Difference
							
								code_pulse_diff=Read_ASC();
								SetStatus(SC_CODE_PULSE_DIFF,code_pulse_diff);
							
								Send_Status();
								
								break;
								
							case CHN:		//Pulse tIme Narrow
							
								N_Pulse=Read_ASC();
								SetStatus(SC_N_PULSE,N_Pulse);
							
								Send_Status();
								
								break;
								
							case CHW:		//Pulse tIme Wide
							
								W_Pulse=Read_ASC();
								SetStatus(SC_W_PULSE,W_Pulse);
							
								Send_Status();
								
								break;
								
							case CHS:		//Pulse tIme to Shut
							
								tts=Read_ASC();
								SetStatus(SC_TTS,tts);
							
								Send_Status();
								
								break;
								
							case CHO:		//Pulse tIme to Open
							
								tto=Read_ASC();
								SetStatus(SC_TTO,tto);
							
								Send_Status();
								
								break;
								
							default:
							
								break;
								
						}
								
						break;
						
					case CHY:		
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHN:		//Pulse sYnc Narrow/wide bit
							
								syncNW=Read_ASC();
								SetStatus(SC_SYNCNW,syncNW);
							
								Send_Status();
								
								break;
								
							case CHP:		//Pulse sYnc Pulse/non-pulse bit
							
								syncPN=Read_ASC();
								SetStatus(SC_SYNCPN,syncPN);
							
								Send_Status();
								
								break;
								
							case CHL:		//Pulse sYnc Length
							
								syncLEN=Read_ASC();
								SetStatus(SC_SYNCLEN,syncLEN);
							
								Send_Status();
								
								break;
								
							default:
								
								break;
								
						}
						
						break;
						
					case CHS:		//Pulse Stop time
					
						if (CMD_Buf[cmd_idx]==SP) {
									
							stop_time=Read_ASC();
							val=(unsigned)(stop_time>>16);
							SetStatus(SC_STOP_TIME_H,val);
							val=(unsigned)((stop_time<<16)>>16);
							SetStatus(SC_STOP_TIME_L,val);
							
							Send_Status();
							
						}
						
						break;
						
					case CHQ:		//initial Pattern seQuence index
					
						if (CMD_Buf[cmd_idx]==SP) {
							
							ini_pattern_seq_idx=Read_ASC();
							SetStatus(SC_INI_PATTERN_SEQ_IDX, ini_pattern_seq_idx);
							
							Send_Status();
							
						}
						
						break;
										
					default:
							
						break;
						
				}
				
				break;
			
				
			/***********************/
			/*motor related command*/
			/***********************/
				
			case CHM:
				
				switch(CMD_Buf[cmd_idx++]) {
					
					case CHA:
					
						switch (CMD_Buf[cmd_idx++]) {
							
							case CHS:		//Motor restArt Step
								
								if(CMD_Buf[cmd_idx]==SP) {
								
									restart_step=Read_ASC();
									SetStatus(SC_RESTART_STEP,restart_step);
									
									Send_Status();
									
								}
								
								break;
								
							case CHP:		//Motor restArt sPeed
							
								if(CMD_Buf[cmd_idx]==SP) {
									
									PWM_restart=ini_PWM_restart=Read_ASC();
									SetStatus(SC_INI_PWM_RESTART,ini_PWM_restart);
									
									Send_Status();
									
								}
								
								break;
								
							default:
							
								break;
								
						}
					
						break;
						
					case CHF:
					
						switch (CMD_Buf[cmd_idx++]) {
							
							case CHC:		//Motor Freeze Count
								
								if(CMD_Buf[cmd_idx]==SP) {
								
									motor_freeze_counter=motor_freeze_count=Read_ASC();
									SetStatus(SC_MOTOR_FREEZE_COUNT,motor_freeze_count);
									
									Send_Status();
									
								}
								
								break;
								
							case CHR:		//Motor Freeze Rate
							
								if(CMD_Buf[cmd_idx]==SP) {
									
									motor_freeze_rate=Read_ASC();
									val=(unsigned)(motor_freeze_rate>>16);
									SetStatus(SC_MOTOR_FREEZE_RATE_H,val);
									val=(unsigned)((motor_freeze_rate<<16)>>16);
									SetStatus(SC_MOTOR_FREEZE_RATE_L,val);
									
									Send_Status();
									
								}
								
								break;
								
							case CHO:	//Motor Freeze Off
							
								if(CMD_Buf[cmd_idx]==CR) {
									
									motor_freeze_counter=motor_freeze_count;
									Motor_Freeze=false;
									
								}
								
								break;
									
								
							default:
							
								break;
								
						}
					
						break;
								
					
					case CHL:
						
						switch (CMD_Buf[cmd_idx++]) {
							
							case CR:		//Motor caLibration
							
								//motor_target=motor_position;	
								Motor_Opt_Calib();
								
								break;
								
							case CHP:		//Motor caLibration sPeed
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									PWM_calib=ini_PWM_calib=Read_ASC();
									SetStatus(SC_INI_PWM_CALIB,ini_PWM_calib);
									
									Send_Status();
									
								}
								
								break;
																
							case CHL:		//Motor caLibration reference Load
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									ref_load_calib=Read_ASC();
									SetStatus(SC_REF_LOAD_CALIB,ref_load_calib);
									
									Send_Status();
									
								}
								
								break;
								
							case CHO:		//Motor caLibration Open load
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									open_load_calib=Read_ASC();
									SetStatus(SC_OPEN_LOAD_CALIB,open_load_calib);
									
									Send_Status();
									
								}
								
								break;
								
							case CHS:		//Motor caLibration Shut load
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									shut_load_calib=Read_ASC();
									SetStatus(SC_SHUT_LOAD_CALIB,shut_load_calib);
									
									Send_Status();
									
								}
								
								break;
								
							case CHC:		//Motor caLibration skip Count
							
								if (CMD_Buf[cmd_idx]==SP) {
									
									skip_count=Read_ASC();
									SetStatus(SC_SKIP_COUNT,skip_count);
									
									Send_Status();
									
								}
								
								break;
								
							case CHT:		//Motor caLibration sTatus
							
								if (CMD_Buf[cmd_idx]==CR) {
									
									Send_Calib_Status();
									
								}
								
								break;
								
							default:
							
								break;
																
						}
								
						break;
					
					case CHC:		//Motor Capture command
						
						if (CMD_Buf[cmd_idx]==CR) {
							
							Motor_Capture=true;
						}
								
						break;
						
					case CHD:		//live Motor loaD command
						
						if (CMD_Buf[cmd_idx]==CR) {
							
							Send_Array(load_buf, LOAD_BUF_SIZE);
							
						}
								
						break;
						
					case CHR:		//Motor Release command
						
						if (CMD_Buf[cmd_idx]==CR) {
							
							Motor_Capture=false;
							
						}
							
						break;
						
					case CHP:		//current motor position

						if(CMD_Buf[cmd_idx]==CR) {
												
							Send_Value(motor_position);
							
						}
				
						break;
						
					case CHO:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CR:		//Motor Open
							
								motor_target=motor_open_position;
								
								break;
							
							case CHP:		//Motor Open Position
							
								if(CMD_Buf[cmd_idx]==CR)
							
									Send_Value(motor_open_position);
								
								break;
								
							case CHF:		//Motor Open Offset
							
								if (CMD_Buf[cmd_idx]==SP) {
								
									val=Read_ASC();
									motor_open_position=motor_open_position-(long)motor_open_offset;
									motor_open_offset=(int)val;
									motor_open_position=motor_open_position+(long)motor_open_offset;
									SetStatus(SC_MOTOR_OPEN_OFFSET,(unsigned)motor_open_offset);
									
									Send_Status();
									Send_Value(motor_open_position);
									
								}
								
								break;
								
							default:
							
								break;
								
						}			
									
						break;
					
					case CHS:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CR:		//Motor Shut
							
								motor_target=motor_shut_position;
								
								break;
							
							case CHP:		//Motor Shut Position
							
								if(CMD_Buf[cmd_idx]==CR)
							
									Send_Value(motor_shut_position);
								
								break;
								
							case CHF:		//Motor Shut Offset
							
								if (CMD_Buf[cmd_idx]==SP) {
								
									val=Read_ASC();
									motor_shut_position=motor_shut_position-(long)motor_shut_offset;
									motor_shut_offset=(int)val;
									motor_shut_position=motor_shut_position+(long)motor_shut_offset;
									SetStatus(SC_MOTOR_SHUT_OFFSET,(unsigned)motor_shut_offset);
								
									Send_Status();
									Send_Value(motor_shut_position);
									
								}
								
								break;
								
							default:
							
								break;
								
						}			
									
						break;
						
					case CHT:		//Motor Target command
				
						if (CMD_Buf[cmd_idx]==CR) {
															
							Send_Value(motor_target);
							
						}
							
						break;
						
					case CHN:		//Motor speed dowN variable
					
						if (CMD_Buf[cmd_idx]==SP) {
							
							Speed_Dn=Read_ASC();
							SetStatus(SC_SPEED_DN,Speed_Dn);
							
							Send_Status();
							
						}
						
						break;
						
					case CHH:		//Motor Home
					
						if (CMD_Buf[cmd_idx]==SP) {
							
							MotorHome=Read_ASC();
							SetStatus(SC_MOTORHOME,MotorHome);
							
							Send_Status();
							
						}
						
						break;
							
					case CHE:		//Motor Encoder scale
					
						if (CMD_Buf[cmd_idx]==SP) {
							
							encoder_scale=Read_ASC();;
							SetStatus(SC_ENCODER_SCALE,encoder_scale);
														
							Send_Status();
							
						}
						
						break;
						
					case CHW:		//Motor pWm command
						
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHU:		//Motor speed Up
							
								switch(CMD_Buf[cmd_idx++]) {
															
									case CHO:
									
										switch(CMD_Buf[cmd_idx++]) {
						
											case SP:		//set the up_open_pwm_count rate
									
												up_open_PWM_count=Read_ASC();
												SetStatus(SC_UP_OPEN_PWM_COUNT,up_open_PWM_count);
											
												Send_Status();
												
												break;
												
											case CHI:		//set the ini_up_open_PWM
												
												if (CMD_Buf[cmd_idx]==SP) {
													
													ini_up_open_PWM=Read_ASC();
													SetStatus(SC_INI_UP_OPEN_PWM,ini_up_open_PWM);
											
													Send_Status();
													
												}
												
												break;
												
											case CHM:		//set the up_open_max_PWM
												
												if (CMD_Buf[cmd_idx]==SP) {
													
													up_open_max_PWM=Read_ASC();
													SetStatus(SC_UP_OPEN_MAX_PWM,up_open_max_PWM);
											
													Send_Status();
													
												}
												
												break;
												
											default:
											
												break;
												
										}

								
										break;
								
									case CHS:
						
										switch(CMD_Buf[cmd_idx++]) {
						
											case SP:		//set the up_shut_pwm_count rate
									
												up_shut_PWM_count=Read_ASC();
												SetStatus(SC_UP_SHUT_PWM_COUNT,up_shut_PWM_count);
											
												Send_Status();
												
												break;
												
											case CHI:		//set the ini_up_shut_PWM
												
												if (CMD_Buf[cmd_idx]==SP) {
													
													ini_up_shut_PWM=Read_ASC();
													SetStatus(SC_INI_UP_SHUT_PWM,ini_up_shut_PWM);
											
													Send_Status();
													
												}
												
												break;
												
											case CHM:		//set the up_shut_max_PWM
												
												if (CMD_Buf[cmd_idx]==SP) {
													
													up_shut_max_PWM=Read_ASC();
													SetStatus(SC_UP_SHUT_MAX_PWM,up_shut_max_PWM);
											
													Send_Status();
													
												}
												
												break;
												
											default:
											
												break;
												
										}

								
										break;
								
									default:
									
										break;
									
								}
								
							case CHD:		//Motor speed Down
							
								switch(CMD_Buf[cmd_idx++]) {
															
									case CHO:
						
										switch(CMD_Buf[cmd_idx++]) {
											
											case SP:		//set the dn_open_pwm_count rate
									
												dn_open_PWM_count=Read_ASC();
												SetStatus(SC_DN_OPEN_PWM_COUNT,dn_open_PWM_count);
											
												Send_Status();
												
												break;
												
											case CHN:		//set the dn_open_min_PWM
											
												if (CMD_Buf[cmd_idx]==SP) {
									
													dn_open_min_PWM=Read_ASC();
													SetStatus(SC_DN_OPEN_MIN_PWM,dn_open_min_PWM);
											
													Send_Status();
													
												}
												
											default:
											
												break;

										}
								
										break;
								
									case CHS:
						
										switch(CMD_Buf[cmd_idx++]) {
											
											case SP:		//set the dn_shut_pwm_count rate
									
												dn_shut_PWM_count=Read_ASC();
												SetStatus(SC_DN_SHUT_PWM_COUNT,dn_shut_PWM_count);
											
												Send_Status();
												
												break;
												
											case CHN:		//set the dn_shut_min_PWM
											
												if (CMD_Buf[cmd_idx]==SP) {
									
													dn_shut_min_PWM=Read_ASC();
													SetStatus(SC_DN_SHUT_MIN_PWM,dn_shut_min_PWM);
											
													Send_Status();
													
												}
												
											default:
											
												break;

										}
								
										break;
								
									default:
									
										break;
									
								}
								
							default:
							
								break;
								
						}
						
						break;
							
					case CHG:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHN:		//Motor Gear Numerator
							
								if(CMD_Buf[cmd_idx]==SP) {
									
									val=Read_ASC();
									
									if(val!=0) {
										
										gear_numerator=val;
										SetStatus(SC_GEAR_NUMERATOR,gear_numerator);
																				
										Send_Status();
										
									}
									
								}
								
								break;
								
							case CHD:		//Motor Gear Denominator
							
								if(CMD_Buf[cmd_idx]==SP) {
									
									val=Read_ASC();
									
									if(val!=0) {
										
										gear_denominator=val;
										SetStatus(SC_GEAR_DENOMINATOR,gear_denominator);
																				
										Send_Status();
										
									}
									
								}
								
								break;
								
							default:
							
								break;	
									
						}
						
						break;
						
					case CHY:		//Motor cYcle command
						
						if(CMD_Buf[cmd_idx]==CR) {  //set the cycle motor with the current motor_cycle_rate
							
							Motor_Cycle=!Motor_Cycle;
							SetStatus(SC_MOTOR_CYCLE,Motor_Cycle);
								
							Send_Status();
							
						}	
						
						break;

						
					default:
							
						break;
				
				}
				
				break;
				
				
			/**********************/
			/*Tool related command*/
			/**********************/
				
			case CHO:
			
				switch(CMD_Buf[cmd_idx++]) {
					
					case CHR:		//tOol Rotate count
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHB:		//tOol Rotate Buffer size
					
								rt_buf_size=Read_ASC();
								SetStatus(SC_RT_BUF_SIZE, rt_buf_size);
						
								Send_Status();
						
								break;
							
							case CHT:		//tOol RoTate count
					
								dly_rt_count=Read_ASC();
								SetStatus(SC_DLY_RT_COUNT, dly_rt_count);
						
								Send_Status();
						
								break;
						
							case CHN:		//tOol Non-RoTate count
					
								dly_nrt_count=Read_ASC();
								SetStatus(SC_DLY_NRT_COUNT, dly_nrt_count);
						
								Send_Status();
						
								break;
								
							default:
							
								break;
								
						}
						
						break;
						
					case CHQ:
					
						switch(CMD_Buf[cmd_idx++]) {
							
							case CHT:		//tOol Quiet counT
					
								dly_qt_count=Read_ASC();
								SetStatus(SC_DLY_QT_COUNT, dly_qt_count);
						
								Send_Status();
						
								break;
						
							case CHN:		//tOol Non-Quiet count
					
								dly_nqt_count=Read_ASC();
								SetStatus(SC_DLY_NQT_COUNT, dly_nqt_count);
						
								Send_Status();
						
								break;
								
							default:
							
								break;
								
						}
						
						break;
						
				}
				
				break;
				
			default:
					
				break;

		}
		
		CMD_Counter--;
		
	}
	
	CMD_ESC = false;
	
}
		
		
		
		
	/*static uchar data_buf[MAX_DATA_PER_CMD];// Holds the current data for rx'd command
	static bool ExpectingCMD = true;	   // Boolean For CMD or Data byte
	static uchar data_idx = 0;             // Data Buffer Index
	static uchar curr_cmd = CMD_NOP;	   // Current Command
	uchar  rx_val;                         // Holds Value gotten out of the rx Buffer
    ushort temp_val;
    ulong  temp_long;
    ushort seq[15];
    //uchar  tx_val;
    
    //temp_val=0x0035;
    
    //UARTPutChar(HIBYTE(temp_val));
 	//UARTPutChar(LOBYTE(temp_val));
    
  	// If there are no rx'd values return
    // loop till Rx buffer is processed to improve comm speed
    while(UARTRxReady())
  {      
    // Get Rx'd value out of buffer
    rx_val = UARTGetChar();

    //---------------------------------------------------------------------------
    // Process Rx'd Byte
    //---------------------------------------------------------------------------
    if(ExpectingCMD && (rx_val < NUM_OF_COMMANDS))
    {	// If command is valid
        curr_cmd = rx_val;		// Set Current Command to new Command
        data_idx = 0;			// Clear data buffer
    }
    else if(!ExpectingCMD && (data_idx < MAX_DATA_PER_CMD))
    {	// Valid Data value
    	data_buf[data_idx++] = rx_val;	// Store data byte
    }
    else
    {	// Were out of sync or bad rx_val so reset
    	curr_cmd = CMD_NOP;		// Reset Command
    	data_idx = 0;			// Clear Data Buffer
    	ExpectingCMD = TRUE;	// Reset Expecting Command
    }
    
    //---------------------------------------------------------------------------
    // Process Command
    //---------------------------------------------------------------------------
    
    //printf("%d\t%d\n",rx_val,curr_cmd);
    
    switch(curr_cmd)
    {
        //--------------------------------------------------------------------------
    	// COMMAND NOP - #0
    	//--------------------------------------------------------------------------
    	case CMD_NOP:
    		data_idx = 0;				// Clear Data Buffer
    		ExpectingCMD = TRUE;		// Reset Expecting Command
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND GET SENSOR AVERAGE - #1
    	//
    	//		Returns the sensor average
    	//		data bytes <channel>
    	//
    	//		channel: (1-11)
    	//--------------------------------------------------------------------------
    	case CMD_GET_SENSOR_AVG:
    		if(data_idx >= 1)
    		{
                switch(data_buf[0])
                {
                    case AVG_1:
                        temp_val = avg1;
                        break;
                    case AVG_2:
                        temp_val = avg2;
                        break;
                    case AVG_3:
                        temp_val = avg3;
                        break;
                    case AVG_4:
                        temp_val = avg4;
                        break;
                    case AVG_5:
                        temp_val = avg5;
                        break;
                    case AVG_6:
                        temp_val = avg6;
                        break;
                    case AVG_7:
                        temp_val = avg7;
                        break;
                    case AVG_8:
                        temp_val = avg8;
                        break;
                    case AVG_9:
                        temp_val = avg9;
                        break;
                    case AVG_10:
                        temp_val = avg10;
                        break;
                    case AVG_11:
                        temp_val = avg11;
                        break;
                }
    		    // Send the average
                UARTPutChar(HIBYTE(temp_val));
    		    UARTPutChar(LOBYTE(temp_val));
    		    
    			data_idx = 0;
    			ExpectingCMD = TRUE;
    		}
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND GET FLASH BYTE - #2
    	//
    	//		Returns byte from flash from given address
    	//		data bytes <hi-byte>, <mid-byte>, <low-byte>
    	//
    	//		byte: (24bit address)
    	//--------------------------------------------------------------------------
    	case CMD_GET_FLASH_BYTE:
    		if(data_idx >= 3)
    		{
    			temp_long = data_buf[0];
    			temp_long <<= 8;
    			temp_long |= data_buf[1];
    			temp_long <<= 8;
    			temp_long |= data_buf[2];
    			ReadFlash(DM_FULL_WORD,1,temp_long,&temp_val);
                UARTPutChar(HIBYTE(temp_val));
    		    UARTPutChar(LOBYTE(temp_val));
    		    data_idx = 0;
    			ExpectingCMD = TRUE;
    		}
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND SET COEFFICIENTS - #3
    	//
    	//		Sets the given coefficient
    	//		data bytes <coeff#>, <hi-byte>, <lo-byte>
    	//
    	//		coeff#: (0-11) see Eval Board Operation.doc
    	//      hi-byte, lo-byte: 16bit coeff value
    	//--------------------------------------------------------------------------
    	case CMD_SET_COEFF:
    		if(data_idx >= 3)
    		{
                temp_val = data_buf[1];
                temp_val <<= 8;
                temp_val |= data_buf[2];
    		    SetCoeff(data_buf[0],temp_val);
                UARTPutChar(ACKNOWLEDGE);
    		    data_idx = 0;
    			ExpectingCMD = TRUE;
    		}
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND SET COEFFICIENTS - #4
    	//
    	//		Sets the time and date on the real time clock
    	//		data bytes <month>, <day>, <year>, <hour>, <minute>, <seconds>
    	//
    	//      month:   (1-12)
        //      day:     (1-31)
        //      year:    (0-255) Starting at 2000
        //      hour:    (0-23)
        //      minute:  (0-59)
        //      seconds: (0-59)
    	//--------------------------------------------------------------------------
    	case CMD_SET_DATE_TIME:
    		if(data_idx >= 6)
    		{
    			data_idx = 0;
    			ExpectingCMD = TRUE;
    		}
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND GET DATA SEQUENCE - #5
    	//
    	//		Returns the data sequence at the given address
    	//		data bytes <hi-byte>, <mid-byte>, <low-byte>
    	//
    	//		byte: (24bit address)
    	//--------------------------------------------------------------------------
    	case CMD_GET_DATA_SEQ:
    		if(data_idx >= 3)
    		{
    			temp_long = data_buf[0];
    			temp_long <<= 8;
    			temp_long |= data_buf[1];
    			temp_long <<= 8;
    			temp_long |= data_buf[2];
    			ReadFlash(DM_FULL_WORD,15,temp_long,seq);
    			for(temp_val=0; temp_val<15; temp_val++)
    			{
                    UARTPutChar(HIBYTE(seq[temp_val]));
    		        UARTPutChar(LOBYTE(seq[temp_val]));
    			}
    		    data_idx = 0;
    			ExpectingCMD = TRUE;
    		}
    		break;
    	//--------------------------------------------------------------------------
    	// COMMAND GET NUMBER OF SEQUENCE SAVED - #6
    	//
    	//		Returns the number of sequenced saved to flash
    	//
    	//--------------------------------------------------------------------------
    	case CMD_GET_NUM_SEQ_SAVED:
            UARTPutChar(HIBYTE(seq_count));
	        UARTPutChar(LOBYTE(seq_count));
		    data_idx = 0;
			ExpectingCMD = TRUE;
    		break;
    }
  }//end of while(  ) loop till rx buffer is processed 
  
}*/

