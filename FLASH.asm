// --------------------------------------------------------------------------
// Routines for reading and writing FLASH memory. 
// --------------------------------------------------------------------------

#include	<def2181.h>;
#include	"flash.h"
#include	"stddef.h"
#include	"Backgrd.h"

#define Wait(v)\
    ar  = v;\
    call WaitDly      
    
// -------------------------------------------------------------------------------
// Locals
// This buffer is used to store data when reading and writing from flash.
// Other parts of the program may use this buffer when sending and receiving
// data. 
//
// WARNING: Do not use this buffer when calling ProgFlash in DM_FULL_WORD mode.
// -------------------------------------------------------------------------------
.section/dm data1;
.var	btemp1,btemp2;
.var    sector_size = 0;
.global _r_flashbuf;
.var	_r_flashbuf[FLASHBUF_SZ];
 
// ----------------------------- 
// Macros
// ----------------------------- 
#define WriteFlashVal(val,addr,ctrl,label)\
    ar = val;\
    dm(btemp1)=ar;\
    ar = addr;\
    dm(BDMA_External_Address)=ar;\
    ar = btemp1;\
    dm(BDMA_Internal_Address)=ar;\
    ar = ctrl;\
    dm(BDMA_Control)=ar;\
    ar = 1;\
    dm(BDMA_Word_Count)=ar;\
label:\
    ar = dm(BDMA_Word_Count);\
    ar = pass ar;\
    if ne jump label

#define ReadFlashVal(addr,ctrl,dsp_addr,label)\
	ar = addr;\
	dm(BDMA_External_Address)=ar;\
	ar = dsp_addr;\
	dm(BDMA_Internal_Address)=ar;\
	ar = ctrl;\
	dm(BDMA_Control)=ar;\
	ar = 1;\
	dm(BDMA_Word_Count)=ar;\
label:\
	ar = dm(BDMA_Word_Count);\
	ar = pass ar;\
	if ne jump label

.section/pm     program;
// ---------------------------------------------------------------------------
// 0:ROUTINE -- void ReadFlash(short ctrl,short num,long addr,void *data);
//
// Reads data from FLASH and stores it in either PM or DM. 
// There are 3 destination memory types:
//
//		PM	(upper 16 bits only)
//		DM	(full 16 bits)
//		DM	(lower 8 bits only)
//
//		If the destination is PM, the BDMA will transfer data to DM and this 
//  routine will copy it to PM.  This is because the BDMA controller has no 
//  provision for transfering only the upper 16-bits of PM. The entire 24 bits 
//  of PM would have been transfered, but this would complicate handling of 
//  flash sectors.
//
//		If the data destination is DM, no copying is done. The BDMA controller 
//  will load the memory directly. The calling parameter "control value" 
//  determines if the DM is 8 or 16 bits.
//
//  Restrictions:
//
//      Data must NOT span a BDMA page (page=0x4000 bytes)
//
//  Internally these registers hold the following values:
//
//      I6 	-- DSP address
//		MR0 -- FLASH address
//		MR1 -- FLASH page (a page is 0x4000 bytes)
//		AY1 -- number of WORDS to transfer. (A word can be 8 or 16 bits depending on 'control value')
//		AR  -- control value (PM_UPPER_WORD, DM_FULL_WORD, or DM_LSB as defined in "flash.h")
//      
// --------------------------------------------------------------------
.global	_ReadFlash,_ReadFlashPM;
_ReadFlash:
_ReadFlashPM:
	leaf_entry();

    // ---------------------------------------------
    // Read stack parameters into proper registers.
    // ---------------------------------------------
    readfirst(ax1);    // ctrl
    readnext(ay0);     // num
	readnext(mr1);     // address (hi)
	readnext(mr0);     // address (lo)
    readnext(si);      // *data 
	i6=si;

	// --------------------------------------------------------------------- 
	// Convert the 32-bit flash address to a 14-bit address and page number. 
	// --------------------------------------------------------------------- 

	sr = lshift mr1 by 2 (HI);
	sr = sr or lshift mr0 by 2 (LO);
	mr1= sr1;							// Flash Page #
	sr = lshift sr0 by -2 (LO);
	mr0= sr0;							// Flash Address

// If the destination is DM, just do the transfer. If it's PM, must transfer and copy 

	ar = tstbit 0 of ax1;
	if eq jump n0_pm;	

// ------------------------ 
// ** Destination is DM ** 
// ------------------------ 

    dm(BDMA_External_Address) = mr0;
    dm(BDMA_Internal_Address) = i6;    
	sr0	= ax1;
    sr = sr or lshift mr1 by 8 (LO);        // Combine BDMA page and control val
    ar = clrbit 2 of sr0;                	// Reading from FLASH
    dm(BDMA_Control) = ar;
    dm(BDMA_Word_Count) = ay0;

// wait for transfer to complete 

n0_wait1:
    ar = dm(BDMA_Word_Count);
    ar = pass ar;
    if ne jump n0_wait1;        
	jump n0_done;

// ----------------------- 
// ** Destination is PM ** 
// ----------------------- 

n0_pm:
	
// Calc the number of words to transfer: num=min(num_words,FLASHBUF_SZ) 			

	ar	= FLASHBUF_SZ;
	none= ar-ay0;
	if ac ar=pass ay0;
	ax0 = ar;							// AX0 = number of words

	dm(BDMA_External_Address) = mr0;
	i1 	= _r_flashbuf;
	dm(BDMA_Internal_Address) = i1;
	sr0 = ax1;							// Combine BDMA page and control val
	sr	= sr or lshift mr1 by 8 (LO);
	ar	= clrbit 2 of sr0;				// Reading from FLASH
	ar	= setbit 0 of ar;				// Force DM dest
	dm(BDMA_Control)=ar;
	dm(BDMA_Word_Count)=ax0;

// wait for transfer to complete 

n0_wait2:
	ar	= dm(BDMA_Word_Count);
	ar	= pass ar;
	if ne jump n0_wait2;
	
// Copy to PM 
	
	m5 	= 1;
	px	= 0;							// Lower 8 bits of PM will get set to zero
	cntr=ax0;
	do n0_copy_lp until ce;
		sr0 = dm(i1,m1);	 	
n0_copy_lp:	pm(i6,m5)=sr0;
	
	ay1 = ax0;							// Inc FLASH address
	ar	= mr0+ay1;
	mr0=ar,ar = ay0-ax0;				// Inc word count
	ay0 = ar;	
	if ne jump n0_pm;					// If count != 0, go around again

n0_done:
	leaf_exit();
_ReadFlashPM.END:
_ReadFlash.END:

// ----------------------------------------------------------------------------
// 2:ROUTINE -- void ProgFlash(short ctrl,short num,long addr,void *data);
//
//      Programs FLASH memory. There are 3 types of memory which can be written
//  to the FLASH: 
//
//		PM_UPPER_WORD	(upper 16 bits only from PM)
//		DM_FULL_WORD    (full 16 bits from DM)
//		DM_LSB	        (lower 8 bits only from DM)
//
//		If the data is from PM, this routine will copy it to DM before writing
//	it to FLASH. This is because the BDMA controller has no provision for 
//  transfering the upper 16-bits of PM. The entire 24 bits of PM would have
//  been transfered, but this would complicate handling of flash sector 
//  programming.
//
//		If the data is from DM and LSB mode, no copying is done. If in FULL mode,
//  data is copied and unpacked to the local buffer before writing to FLASH. In all
//  cases, the FLASH is programmed by using BDMA LSB-only mode. This is to prevent
//  the very small write-inactive pulses between writes that occur in FULL_WORD mode.
//  This is a work-around for not meeting the FLASH wr-inactive pulse widths. It would
//  be more efficent to use FULL word transfers, but the hardware timing issues need
//  to be resolved first.
//
//  Restrictions:
//
//      1) Data must NOT span a BDMA page (page=0x4000 bytes)
//		2) The FLASH address must start on a FLASH_SECTOR_SIZE boundary
//      3) Partial FLASH sectors can be written but the part NOT written will contain 
//		   indeterminate data. 
//      4) Do NOT use 'r_flash_buf' when calling in DM_FULL_WORD mode.
//
//  Internally these registers hold the following values:
//
//      I6  -- DSP address
//		MR0 -- FLASH address
//		MR1 -- FLASH page
//		AY0 -- number of words to transfer
//		AX1 -- control value		(See flash.h for possible values)
//
// ----------------------------------------------------------------------------
.global  _ProgFlash,_ProgFlashPM;
_ProgFlash:
_ProgFlashPM:
	leaf_entry();

	// ---------------------------------------------------------------------------
	// If the sector size is zero, this must be the first call to ProgFlash. In
	// this case the flash type must be read and the sector size set accordingly.
	// ---------------------------------------------------------------------------
	ar = dm(sector_size);
	ar = pass ar;
	if ne jump n2_have_sector_size;
	
	// Set the sector size depending on the flash type.
	call GetFlashInfo;
	ay0 = 0x35;                 // If the flash is a 1Meg flash the return value will be 0x35   
	ar = sr1-ay0;
	ar = FLASH_SECTOR_4M;
	ax0= FLASH_SECTOR_1M;
	if eq ar = pass ax0;
	dm(sector_size) = ar;
	
n2_have_sector_size:
    // ------------------------------------------
    // Read parameters from stack.
    // ------------------------------------------
    readfirst(ax1);             // ctrl
    readnext(ay0);              // num
	readnext(mr1);				// upper addr
	readnext(mr0);				// lower addr
    readnext(si);               // *data (page)
	i6=si;

	// --------------------------------------------------------------------- 
	// Convert the 32-bit flash address to a 14-bit address and page number. 
	// --------------------------------------------------------------------- 

	sr = lshift mr1 by 2 (HI);
	sr = sr or lshift mr0 by 2 (LO);
	mr1= sr1;							// Flash Page #
	sr = lshift sr0 by -2 (LO);
	mr0= sr0;							// Flash Address

// ----------------------------------------------------------------------------- 	
// Main loop:
//		Each time around a maximum of FLASH_SECTOR_SIZE bytes are written to 
// the flash until the transfer is complete.	  										   	
// ----------------------------------------------------------------------------- 	
n2_main_loop:

// Calc the number of words to transfer: num=min(num_words,FLASH_SECTOR_SIZE (in words)) 

	sr0 = dm(sector_size);			// Calc sector size in WORDS
	ar	= tstbit 1 of ax1;
	if ne jump n2_no_div;
	sr	= lshift sr0 by -1 (LO);

n2_no_div:
	ar	= sr0;							// Calc min(num_words,FLASH_SECTOR_SIZE (in_words) 
	none= ar-ay0;	
	if ac ar=pass ay0;
	ax0 = ar;							// AX0 = number of words

    // Check if this is a PM or DM transfer.
	ar	= tstbit 0 of ax1;
	if ne jump n2_dm;

    // Data is in program memory. Only upper 16-bits of PM will be stored.
    // This requires data to be copied to the local buffer and unpacked into
    // MSB,LSB pairs.
	i1	= _r_flashbuf;					// Copy from PM to DM
	m5	= 1;
    ay1 = 0xff;
	cntr=ax0;
	do n2_copy_lp until ce;
		sr0 = pm(i6,m5);		
        ar = sr0 and ay1;               // extract LSByte
        sr = lshift sr0 by 8 (LO);      // extract MSByte
        dm(i1,m1) = sr1;                // store MSByte
n2_copy_lp:	
        dm(i1,m1) = ar;                 // store LSByte

	mx0	= _r_flashbuf;					// DSP address 
	jump	n2_common;

    // Data is in data memory. Check if this is a LSB or FULL transfer.
    // LSB mode requires no data copying, but FULL mode requires data to
    // be unpacked into MSB,LSB pairs.
n2_dm:
    ar = tstbit 1 of ax1;
    if ne jump n2_dm_lsb;

    // Data is Full word so we must unpack to local buffer.
    i1 = _r_flashbuf;
	m5 = 1;
    ay1 = 0xff;
    cntr=ax0;
    do n2_copy_dm_lp until ce;
        sr0 = dm(i6,m5);
        ar = sr0 and ay1;           // extract LSByte
        sr = lshift sr0 by 8 (LO);  // extract MSByte
        dm(i1,m1) = sr1;            // store MSByte
n2_copy_dm_lp:
        dm(i1,m1) = ar;             // store LSByte

    mx0 = _r_flashbuf;              // DSP address
    jump n2_common;

    // Data is LSB format. No copying is required.
n2_dm_lsb:
	mx0	= i6;							// DSP address 
	ay1	= i6;							// Update address for next time around 
	ar	= ax0+ay1;
	i6  = ar;

    // Start the BDMA transfer.
n2_common:

// Enable programming mode 
	WriteFlashVal(0xaa,0x1555,0x107,n2_wait1);
	WriteFlashVal(0x55,0x2aaa,0x007,n2_wait2);
	WriteFlashVal(0xa0,0x1555,0x107,n2_wait3);
	
// Write the data 
	dm(BDMA_External_Address)=mr0;
	dm(BDMA_Internal_Address)=mx0;
	sr0= ax1;							// Combine page# and control value 
	sr = sr or lshift mr1 by 8 (LO);	
	ar = setbit 2 of sr0;				// Set write to FLASH bit 
	ar = setbit 0 of ar;				// Force DM transfer 
    ar = setbit 1 of ar;                // Force LSB transfer
	dm(BDMA_Control) = ar;
    sr0 = ax0;
    ar = tstbit 1 of ax1;               // If FULL word, double the count (i.e. convert to #bytes).
	if ne jump n2_no_shift;
    sr = lshift sr0 by 1 (LO);
n2_no_shift:
	dm(BDMA_Word_Count)=sr0;            // SR0 = byte count

// Adjust pointers and count 
	ay1	= sr0;
	ar	= mr0+ay1;       				// Inc FLASH address
	mr0=ar,ar = ay0-ax0;				// Dec word count
	ay0 = ar;

// Wait for Transfer to finish 
n2_wait4:
	ar	= dm(BDMA_Word_Count);
	ar  = pass ar;
	if ne jump n2_wait4;	
    
// Wait for the FLASH to finish the program cycle 
n2_wait5:	
	ReadFlashVal(0,0x3,btemp1,n2_wait6);
	ReadFlashVal(0,0x3,btemp2,n2_wait7);
	ax0= dm(btemp1);					// Check if bit6 is toggling
	ay1= dm(btemp2);
	ar = ax0 xor ay1;
	ar = tstbit 6 of ar;
	if ne jump n2_wait5;				// If it's toggling, wait

// Check word count and go around again 
	ar	= pass ay0;
	if ne jump n2_main_loop;
	leaf_exit();
_ProgFlashPM.END:
_ProgFlash.END:

// ------------------------------------------------------------- 
// 3:ROUTINE -- void LoadApplication()
// 
//  	This loads the application program and runs it.  
//			Note: This program gets overwritten in the process, 
//			      thus this routine can never return. 			
// -------------------------------------------------------------
.global	_LoadApplication;
_LoadApplication:
	ay1 = 0;								// Load from FLASH address
	dm(BDMA_External_Address) = ay1;	
	ay1 = 0;
	dm(BDMA_Internal_Address) = ay1;

// Combine page and control values 

	sr0 = 0x08;								// btype=0, bcr=1, bdir=0
	sr1	= APPLICATION_PAGE;
	sr	= sr or lshift sr1 by 8 (LO);
	dm(BDMA_Control) = sr0;

// Writing to BDMA_Word_Count will start the transfer 

	ay1 = 32;								// Load first 32 words	
	dm(BDMA_Word_Count) = ay1;	
	idle;									// Waits here until done, then resets
_LoadApplication.END:
	
// ----------------------------------------------------------------------------
// 4:ROUTINE - Local Routine.
//
// Parameters: None
// Returns:
//      SR0 = Lower boot-block lockout (0xff=locked-out, 0xfe=NOT locked-out)
//      SR1 = Device Code (0x35=AT29LV010A, 0xC4 = AT29LV040A) 
// ----------------------------------------------------------------------------
GetFlashInfo:

	WriteFlashVal(0xaa,0x1555,0x107,n4_wait1);
	WriteFlashVal(0x55,0x2aaa,0x007,n4_wait2);
	WriteFlashVal(0x90,0x1555,0x107,n4_wait3);

	Wait(0xffff);
	Wait(0xffff);
	Wait(0xffff);
	Wait(0xffff);

	ReadFlashVal(2,0x3,btemp2,n4_wait4);       // Get lower boot block lockout value

	sr0 = dm(btemp2);

	//ReadFlashVal(0x3ff2,0x303,btemp2,n4_wait5);
	ReadFlashVal(1,0x3,btemp2,n4_wait5);       // Get Device code

	sr1 = dm(btemp2);

	WriteFlashVal(0xaa,0x1555,0x107,n4_wait6);
	WriteFlashVal(0x55,0x2aaa,0x007,n4_wait7);
	WriteFlashVal(0xF0,0x1555,0x107,n4_wait8);

	Wait(0xffff);
	Wait(0xffff);
	Wait(0xffff);
	Wait(0xffff);

	rts;
	
// -----------------------------------------------------------------------------------
// 5:ROUTINE -- WaitDly
//
//  Calling Parameters
//
//      AR	-	Time to delay ( AR*122 ns )
//
//  Return values
//
//      none
//
//  Modified registers
//
//      AR
// -----------------------------------------------------------------------------------
WaitDly:
	nop;
	nop;
#if !__SIMULATION__ //following lines don't seem to work properly in simulator
	ar = ar-1;
	if ne jump WaitDly;
#endif
	rts;
	
		
	
