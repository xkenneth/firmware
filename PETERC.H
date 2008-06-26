#ifndef PETERC_H
#define PETERC_H

#include "stddef.h"

//-----------------------------------------------------------------------------
// Calibration Constants
//-----------------------------------------------------------------------------
enum {CC_GXMEAN, CC_GXRANGE, CC_GYMEAN, CC_GYRANGE, CC_GZMEAN, CC_GZRANGE,
      CC_MXMEAN, CC_MXRANGE, CC_MYMEAN, CC_MYRANGE, CC_MZMEAN, CC_MZRANGE,
      CC_TPGAIN, CC_TPOFFSET,	//tempreature slope and intersect
      NUM_CALIB_CONST};
      
//-----------------------------------------------------------------------------
// Status Constants
//-----------------------------------------------------------------------------
enum {SC_SENSOR_LIVE, SC_MOTOR_CYCLE, SC_TF_RESET, SC_DATE_TIME_H, SC_DATE_TIME_L, 
	  SC_PULSE_TIME, SC_CODE_PULSE_TIME, SC_CODE_PULSE_DIFF, SC_N_PULSE, SC_W_PULSE, SC_TTS, SC_TTO,
	  SC_SYNCNW, SC_SYNCPN, SC_SYNCLEN, SC_GEAR_NUMERATOR, SC_GEAR_DENOMINATOR, SC_ENCODER_SCALE, 
	  SC_INI_PATTERN_SEQ_IDX, SC_RT_BUF_SIZE, SC_DLY_RT_COUNT, SC_DLY_NRT_COUNT, SC_RT_GX, SC_RT_GY,
	  SC_RT_GZ, SC_RT_HX, SC_RT_HY, SC_RT_HZ, SC_RT_INS_GX, SC_RT_INS_GY, SC_RT_INS_GZ, SC_RT_INS_HX, 
	  SC_RT_INS_HY, SC_RT_INS_HZ, SC_DLY_QT_COUNT, SC_DLY_NQT_COUNT, SC_QT_GX, SC_QT_GY, SC_QT_GZ, 
	  SC_QT_HX, SC_QT_HY, SC_QT_HZ, SC_QT_INS_GX, SC_QT_INS_GY, SC_QT_INS_GZ, SC_QT_INS_HX, SC_QT_INS_HY, 
	  SC_QT_INS_HZ, SC_TF_G_ANGLE, SC_TF_H_ANGLE, SC_TF_G_OFFSET, SC_TF_H_OFFSET, SC_TF_THRESHOLD, 
	  SC_MAX_INC, SC_GR_LOG_SIZE, SC_RESTART_STEP, SC_INI_PWM_RESTART, SC_INI_PWM_CALIB, SC_SKIP_COUNT, 
	  SC_OPEN_LOAD_CALIB, SC_SHUT_LOAD_CALIB, SC_REF_LOAD_CALIB, SC_MOTOR_OPEN_OFFSET, SC_MOTOR_SHUT_OFFSET, 
	  SC_SPEED_DN, SC_MOTORHOME, SC_UP_OPEN_PWM_COUNT, SC_UP_SHUT_PWM_COUNT, SC_DN_OPEN_PWM_COUNT, 
	  SC_DN_SHUT_PWM_COUNT, SC_INI_UP_OPEN_PWM, SC_INI_UP_SHUT_PWM, SC_UP_OPEN_MAX_PWM, SC_UP_SHUT_MAX_PWM, 
	  SC_DN_OPEN_MIN_PWM, SC_DN_SHUT_MIN_PWM, SC_MOTOR_FREEZE_COUNT, SC_MOTOR_FREEZE_RATE_H, 
	  SC_MOTOR_FREEZE_RATE_L, SC_STOP_TIME_H, SC_STOP_TIME_L, SC_RT_DLY_RATE_H, SC_RT_DLY_RATE_L, 
	  SC_DATA_FRESH_RATE_H, SC_DATA_FRESH_RATE_L, SC_DATA_SAVE_RATE_H, SC_DATA_SAVE_RATE_L, 
	  NUM_STATUS_CONST};
	  
//Sensor Rotation Constant
enum {RT_GX, RT_GY, RT_GZ, RT_HX, RT_HY, RT_HZ, NUM_OF_RT};

//-----------------------------------------------------------------------------
// Pattern Constants
//-----------------------------------------------------------------------------
enum {PC_REGULAR_FRAME0, PC_REGULAR_FRAME1, PC_REGULAR_FRAME2, PC_REGULAR_FRAME3, PC_REGULAR_FRAME4,
	  PC_REGULAR_FRAME5, PC_REGULAR_FRAME6, PC_REGULAR_FRAME7, PC_REGULAR_FRAME8, PC_REGULAR_FRAME9, 
	  PC_REGULAR_FRAME10, PC_REGULAR_FRAME11, PC_REGULAR_FRAME12, PC_REGULAR_FRAME13, PC_REGULAR_FRAME14,
	  PC_REGULAR_FRAME15, PC_STATIC_FRAME0, PC_STATIC_FRAME1, PC_STATIC_FRAME2, PC_STATIC_FRAME3, 
	  PC_STATIC_FRAME4, PC_STATIC_FRAME5, PC_STATIC_FRAME6, PC_STATIC_FRAME7, PC_STATIC_FRAME8,
	  PC_STATIC_FRAME9, PC_STATIC_FRAME10, PC_STATIC_FRAME11, PC_STATIC_FRAME12, PC_STATIC_FRAME13,
	  PC_STATIC_FRAME14, PC_STATIC_FRAME15, PC_TEST_PULSE, PC_SIMPLE_PULSE, PC_FREQ_PULSE, PC_CODE_PULSE,
	  PC_CODE_SEQ, PC_RUN, PC_STOP, NUM_PATTERN_CONST};

//-----------------------------------------------------------------------------
// Pattern Sequence
//-----------------------------------------------------------------------------
enum {SEQ_REGULAR, SEQ_ALT, SEQ_BACKUP, NUM_PATTERN_SEQ};

#define PATTERN_BUF_SIZE	20
#define PATTERN_SEQ_BUF_SIZE	10
        
#define FLASH_HEADER_SZ 2       // (16bit short size)
#define STATUS_HEADER_SZ 2       // (16bit short size)
#define PATTERN_HEADER_SZ 2       // (16bit short size)
#define PATTERN_SEQ_HEADER_SZ 2       // (16bit short size)

#define CPS_MAX 500	//Limit GR counts to 500 CPS

#define RTBIN_INS_SIZE		16
#define RTBIN_SIZE			60

void PeterC();
short GR_CPS();
void LoadCoeff();
void SetCoeff(unsigned cc_idx, ushort val);
void LoadStatus();
void SetStatus(unsigned sc_idx, ushort val);
void CoeffReset();
void LoadPattern();
void SetPattern(ushort pattern_seq, unsigned pc_idx, ushort p_val, ushort pc_val);
void LoadPatternSeq();
void SetPatternSeq(unsigned seq_idx, ushort q_val, ulong seq_val);
#endif
