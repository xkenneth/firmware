// ----------------------------------------- 
// Functions for reading/writing I/O memory
// ----------------------------------------- 
#include	"sysdef.h"
#include	"stddef.h"

.section/code	program;

// ------------------------------------------------------------------------
// 0:ROUTINE - int GetIO_NC(int reg)
//
//	Note: This routine is NOT reenterant. It uses self-modifying code.
// As a result, this function should not be called from interrupt routines.
// ------------------------------------------------------------------------
.global	_GetIO_NC;
_GetIO_NC:
	leaf_entry();
	readfirst(ar);
ClrPFTYPEBit(PF0);		
	i6 = n0_io;
	m5 = 0;
	sr0 = 0xa;
	sr = sr or lshift ar by 4 (LO);
	px = sr0;
	sr = lshift sr0 by -8 (LO);
	ar = setbit 8 of sr0;
	pm(i6,m5) = ar;
	nop;

n0_io:
	ar = io(0);
	leaf_exit();
_GetIO_NC.END:

// --------------------------------------------------------------------------
// 1:ROUTINE - void SetIO_NC(int reg,int val)
//
//	Note: This routine is NOT reenterant. It uses self-modifying code.
// As a result, this function should not be called from interrupt routines.
// --------------------------------------------------------------------------
.global	_SetIO_NC;
_SetIO_NC:
	leaf_entry();
	readfirst(ar);
	readnext(ay1);

	i6 = n1_io;
	m5 = 0;
	sr0 = 0x8005;
	sr = sr or lshift ar by 4 (LO);
	px = sr0;
	sr = lshift sr0 by -8 (LO);
	ar = setbit 8 of sr0;
	pm(i6,m5) = ar;
	nop;

n1_io:
	io(0)=ay1;
	leaf_exit();
_SetIO_NC.END:

// ---------------------------------------------------------------
// 2:ROUTINE -- void NSDelay(ushort nsec);
//								AR
//
//	Delays for the specified number of nsec. The only register
// this function uses is AR.
//
//	Modified registers:
//		AR
//
// ---------------------------------------------------------------
.section/data 	data1;
	regspace(ay1);

.section/code 	program;
#define CYCLE_OVERHEAD 	(CYCLE_TIME*7)
#define CYCLE_LOOP	  	(CYCLE_TIME*2)

.global _NSDelay;
_NSDelay:
	save(ay1);
	ay1 = CYCLE_OVERHEAD;			// Subtract the number of cycles of overhead
	ar = ar-ay1;
	ay1 = CYCLE_LOOP;
	if not ac jump n2_done;			// If the num nsec is less then the overhead, return	
n2_lp:								// Loop until the number of nsec has been reached.
	ar = ar-ay1;					// Each time around the loop subtract two cycle times
	if ac jump n2_lp;				// because the loop takes two cycles.
n2_done:
	restore(ay1);
	rts;
_NSDelay.END:

/*
// ------------------------------------------------------------------------
// 3:ROUTINE - void SetPF4ModeOutput()
//
// ------------------------------------------------------------------------
.global	_SetPF4ModeOutput;
_SetPF4ModeOutput:
	leaf_entry();
	SetPFTYPEBit(PF4);		

	leaf_exit();
_SetPF4ModeOutput.END:

// ------------------------------------------------------------------------
// 4:ROUTINE - void SetPF4()
//
// ------------------------------------------------------------------------
.global	_SetPF4;
_SetPF4:
	leaf_entry();
	SetFlagBit(PF4);		

	leaf_exit();
_SetPF4.END:

// ------------------------------------------------------------------------
// 4:ROUTINE - void ClrPF4()
//
// ------------------------------------------------------------------------
.global	_ClrPF4;
_ClrPF4:
	leaf_entry();
	ClrFlagBit(PF4);		

	leaf_exit();
_ClrPF4.END:

*/

