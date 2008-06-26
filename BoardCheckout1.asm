/* Board Checkout Program for Teledrill DSP */
/* Note that this dsp program requires the following */
/* linker file for testing -- ADSP-2187b.ldf */
/******************************************/
/* Revision History   DSP_Board_Checkout1.dsp */
/* 5/10/03  P. Masak  First working version */
/* 8/9/03   P. Masak  Worked on syncing to C0 */
/* 08/03/05 D. Lerner Add reset for FL2        */
/* 01/25/06 D. Lerner GR is on channel avg8 */
/* 02/02/07	W. Zhou	  PF4 connected to sensor A and PF6 connected to sensor C*/
/* 02/03/07 W. Zhou   Motor position obtained through the two quadratic signal from the motor*/
/********************************************/
/* Functional Description                   */
/* 											*/
/* This routine provides the following functionality */
/* It reads incoming data on 11 channels into the DSP */
/* serial port 0									  */
/* The first frame is a sync word, with a code of 0xC000 */
/* Channels 1-5, repeated 16 times                    */
/* Channels 6-11, repeated 16 times					  */
/* A timer is set to toggle FL1 each 0.4 seconds      */
// I removed set FL1 in timer setup code dl 02/06/06
// I removed set FL1 and FL2 in Startup code dl 02/06/06

#include	"stddef.h"
#include	"Backgrd.h"
#include	"dataout.h"


#define  SPORT1_Autobuf_Ctrl    0x3fef
#define  SPORT1_RFSDIV          0x3ff0
#define  SPORT1_SCLKDIV         0x3ff1
#define  SPORT1_Ctrl_Reg        0x3ff2
#define  SPORT0_Autobuf_Ctrl    0x3ff3
#define  SPORT0_RFSDIV          0x3ff4
#define  SPORT0_SCLKDIV         0x3ff5
#define  SPORT0_Ctrl_Reg        0x3ff6
#define  SPORT0_TX_Channels0    0x3ff7
#define  SPORT0_TX_Channels1    0x3ff8
#define  SPORT0_RX_Channels0    0x3ff9
#define  SPORT0_RX_Channels1    0x3ffa
#define  Tscale_Reg             0x3ffb
#define  Tcount_Reg             0x3ffc
#define  Tperiod_Reg            0x3ffd
#define  DM_Wait_Reg            0x3ffe
#define  Sys_Ctrl_Reg           0x3fff
#define  BDMA_ctrl              0x3fe3
#define  BWCOUNT                0x3fe4
#define  BEAD                   0x3fe2
#define  BIAD                   0X3FE1


/*monitor registers*/
#define FLAG     0x03FDF
#define BYTE     0x03FDE
#define CODES	 0x03FDD
#define store	 0x03FDC
#define byte1	 0x03FDB
#define word1	 0x03FDA
#define byte2	 0x03FD9
#define LATCH	 0x03FD8
#define VERSION	 0x03FD7
#define STACK1	 0x03FD6
#define STACK2	 0x03FD5
#define STACK3	 0x03FD4

#if 0
.section/pm interrupts;			/*------Interrupt vector table------*/
ISR_RESET:          jump start;	RTI; RTI; RTI; 
ISR_IRQ2:           RTI		RTI; RTI; RTI;   
ISR_IRQ1:           RTI;		RTI; RTI; RTI; 
ISR_IRQ0:           RTI;		RTI; RTI; RTI;  
ISR_S0_TRANSMIT:    RTI;		RTI; RTI; RTI;  
ISR_S0_RECEIVE:     jump RxInt;		RTI; RTI; RTI;   
ISR_IRQE:           RTI 		RTI; RTI; RTI;  
ISR_BDMA:           RTI;		RTI; RTI; RTI;  
ISR_S1_TRANSMIT:    RTI;        RTI; RTI; RTI;
ISR_S1_RECEIVE:     RTI;        RTI; RTI; RTI;  
ISR_TIMER:          jump timer_isr;	RTI; RTI; RTI;  
ISR_POWERDOWN:      RTI;		RTI; RTI; RTI;  
#endif


/*this program reads data from ADC channel 1 ,2 ,3 ,4 ,5 & 6*/
// seems to read through sensor channel 11 dl 09Sep05
.SECTION/DATA	data1;

// .VAR on 16 bit processors makes 16 bit "short" mem allocations
// .GLOBAL includes the variables in global name space for the pgm

.var/circ 	input[196];    	/*circular input sample buffer - was 60*/
.var 		rx_loc;
.var		sync_add;
.var		_sampling = 0;
.global		_sampling;
.var 		samp_cntr[1];
.var		location[1];
.var 		_avg1,_avg2,_avg3,_avg4,_avg5,_avg6;
.global		_avg1,_avg2,_avg3,_avg4,_avg5,_avg6;
.var 		_avg7,_avg8,_avg9,_avg10,_avg11;
.global		_avg7,_avg8,_avg9,_avg10,_avg11;

.var		_GRCount = 0;

.var		_GRayCPS = 0;
.global		_GRayCPS;

.extern		_GRBin_buf;
.extern		_GRBin_CPS;

.extern     _motor_position;
.extern		_PWM_setting;
.extern		_quad;
//.extern		_previous_quad;

.extern		_load_counter;
.extern		_load_buf;
.extern		_load_pos_buf;
.extern		_load_in_idx;

.extern		_velocity;

.extern		_gxbin;
.extern		_gybin;
.extern		_gzbin;

.extern		_hxbin;
.extern		_hybin;
.extern		_hzbin;

//.var		_acount = 0;
//.var		_bcount = 0;

    regspace(i1);
    regspace(l1);
    regspace(m2);
    //regspace(m3);

.SECTION/CODE 	program;
// ------------------------------------------------------------------------
// 0:ROUTINE - void	Startup(void);
//							  
//
// ------------------------------------------------------------------------
.global _Startup;
_Startup:  	
 
/*=============== setup the serial ports ==============================*/			

	ax0=b#0000000000000000;   dm(SPORT0_Autobuf_Ctrl) = ax0;
	ax0=255;                  dm(SPORT0_RFSDIV) = ax0;
	ax0=312;				  dm(SPORT0_SCLKDIV) = ax0;  /* divide MCLK by 312 */
	ax0=b#0110000000001111;   dm(SPORT0_Ctrl_Reg) = ax0;  /* internal SCLK0 , external framing, word length=16 bits*/
	ax0=b#0000000000000000;   dm(SPORT0_TX_Channels0) = ax0;
	dm(SPORT0_TX_Channels1) = ax0;
	dm(SPORT0_RX_Channels0) = ax0;
	dm(SPORT0_RX_Channels1) = ax0;
	// don't know why flags are reset set reset ?? DL 08/03/05
    reset FL1;
  	//set FL1;
  	//reset FL1;
    reset FL2;
  	//set FL2;
  	//reset FL2;
  
  
	/***** SPORT ENABLE  ******/
  
  IFC=0x1E;  /* clear any extraneous SPORT interrupts */
  ICNTL=0;	/* interrupt nesting enabled */
  ax0=0x1007;  /* enable SPORT0, Enable BMS*/
  dm(Sys_Ctrl_Reg)=ax0;
      //imask=b#0000100001;   /* enable serial and timer interrupts */
      //imask=b#1000010000;   /* enable IRQ2 and IRQE interrupts */
  
  #if 0
  initialize_data_buffer:
  m0=1;
  m1=6;
  i0=input;  /* point to mem location of data */
  l0=length(input);
  #endif    
  
    // Setup interrupt vector and enable RX0, IRQ2 and IRQE interrupts.
    i6 = jump_inst;
    ax0=pm(i6,m6);				// Set the RX0 interrupt vector to jump
    i6 = 20;
    pm(i6,m6)=ax0;				// to the RxInt routine.
    
    i6 = jump_grint;
    ax0=pm(i6,m6);				// Set the IRQ2 interrupt vector to jump
    i6 = 4;
    pm(i6,m6)=ax0;				// to the GRInt routine.
    
//    i6 = jump_pos;
//    ax0=pm(i6,m6);				// Set the IRQE interrupt vector to jump
//    i6 = 24;
//    pm(i6,m6)=ax0;				// to the PosDownInt routine.
    
    
    
    dis ints;
    nop;
    ar = imask;
    ar = setbit 5 of ar;        // enable SPORT0 receiver
//    ar = setbit 4 of ar;		// enable IRQE
    ar = setbit 9 of ar;		// enable IRQ2
    imask = ar;
    
    ar = icntl;
    ar = setbit 2 of ar;		// set IRQ2 to edge sensitive
    icntl = ar;
    
    ClrPFTYPEBit(PF6);
	ClrPFTYPEBit(PF4);
	ClrPFTYPEBit(PF7);

    nop;
    ena ints;
	
  #if 0
  /* TIMER SETUP */
  ax0=255;
  dm(Tscale_Reg)=ax0;
  ax0=0xFFFF;
  dm(Tcount_Reg)=ax0;
  dm(Tperiod_Reg)=ax0;
  //set FL1; //don't know why flag1 is set in timer setup??
  ena TIMER;
  #endif    

  rts;
_Startup.END:    
  
// ------------------------------------------------------------------------
// 1:ROUTINE - void	RxInt();
//							  
// Receive data interrupt routine 
// ------------------------------------------------------------------------
    // RX0 Interrupt will jump here.    
jump_inst:
	jump	_RxInt;
	nop;
	
	// IRQ2 Interrupt will jump here.
//jump_posup:
//    jump	PosUpInt;
//    nop;
    
    // IRQE Interrupt will jump here.
//jump_pos:
//	jump	PosInt;
//	nop;
	
_RxInt: 
  ena SEC_REG;
  save(i1);
  save(l1);
  		
    ax0=rx0;      // read input 
  
    ay1 = dm(_sampling);       // 0 = done sampling
    none = pass ay1;
    if eq jump n1_done;
    
  i1 = dm(rx_loc);
  l1 = length(input);
  dm(i1,m1)=ax0;  /* Store received data */
  dm(rx_loc) = i1;
  ay0=0xC000;
  ar=ay0-ax0;     /* test if data is a sync word */
  IF ne jump n1_done; /* no sync word detected, finish */

/*  the processor scans for the sync word */
/*  upon detection of the frame sync word, the processor */
/*  starts storing the data in a data buffer, whose index */
/*  is defined by the data buffer input */

  syncloop:  
  ar=input;  /* set index to beginning of circ buffer */  
  dm(rx_loc) = ar;
  
  // Decrement sampling flag
  ar = dm(_sampling);
  ar = ar-1;
  dm(_sampling) = ar;
  
n1_done:  
  
  restore(l1);
  restore(i1);
  dis   SEC_REG;
  rti;
  
#if 0  
  timer_isr:
  toggle Fl1;	/* change state of solenoid drive flag*/
  rti; 
#endif


// IRQ2 interrupt routine void GRInt();
  
// IRQ2 Interrupt will jump here.    
jump_grint:
	jump	_GRInt;
	nop;
	
_GRInt: 
  ena SEC_REG; 
 
  af=pass 0;
 
  //count time bins for each gamma ray spike
  do BinCount until ne;
  
  ar=dm(Prog_Flag_Data);	//PF port values
  ar=tstbit 7 of ar;		//check if FP7 is low
  BinCount: af=af+1;		//increment bin count by 1
  
  ar=pass af;
  ax0=ar;
  ay0=GRBIN_BUF_SIZE;		//size of the GRBin_buf
  ar=ax0-ay0;
  
  IF le jump SaveGRBin;
  
  ax0=ay0; 
  
SaveGRBin:

  save(i1);
  save(m2);

  ay0=_GRBin_buf;		//increment the bin count buffer by 1 at specific locations
  ar=ax0+ay0;
  ar=ar-1;
  i1=ar;
  m2=0;
  
  ar=dm(i1,m2);
  ar=ar+1;
  dm(i1,m2)=ar;
  
  ar=dm(_GRCount);		//increment GRCount by 1 and return;
  ar=ar+1;
  dm(_GRCount)=ar;
  
  restore(m2);
  restore(i1);
  
  dis   SEC_REG;
  
  rti;

  
  
//PosUpInt:

//    ar=dm(_motor_position);
//    ar=ar+1;
//    dm(_motor_position)=ar;
    
//    rti;

/*PosInt:

	ena SEC_REG;

	ax0 = dm(Prog_Flag_Data);
	ay0 = 0x0040;
	ar	= ax0 and ay0;
	
	IF ne jump CW;
	
	IF eq jump CCW;
	
CW:	
	ar=dm(_motor_position);
    ar=ar+1;
    dm(_motor_position)=ar;
    
    dis SEC_REG;
    
    rti;
    
    
CCW: 
	ar=dm(_motor_position);
    ar=ar-1;
    dm(_motor_position)=ar;
    
    dis SEC_REG;
    
    rti;*/
  
// ------------------------------------------------------------------------
// 2:ROUTINE - void	UpdateChAvg();
//							  
//
// ------------------------------------------------------------------------
.global	_UpdateChAvg;
_UpdateChAvg:
    /* compute sums from each channel */
    /* channel 1 */
    
      i1=input;m2=6;
      cntr=16;
      ar=0;
      do average1 until ce;
      	ay1=dm(i1,m2);
      average1: ar=ar+ay1; 
      dm(_avg1)=ar;
    // channel 2
      i1=input+1;m2=6;
      i2=_hzbin;m3=1;
      cntr=16;
      ar=0;
      do average2 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average2: ar=ar+ay1; 
      dm(_avg2)=ar;
     // channel 3
      i1=input+2;m2=6;
      i2=_hybin;m3=1;
      cntr=16;
      ar=0;
      do average3 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average3: ar=ar+ay1; 
      dm(_avg3)=ar;
      // channel 4
      i1=input+3;m2=6;
      i2=_hxbin;m3=1;
      cntr=16;
      ar=0;
      do average4 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average4: ar=ar+ay1; 
      dm(_avg4)=ar;
      // channel 5
      i1=input+4;m2=6;
      i2=_gzbin;m3=1;
      cntr=16;
      ar=0;
      do average5 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average5: ar=ar+ay1; 
      dm(_avg5)=ar;
      // channel 6
      i1=input+95;m2=6;
      i2=_gybin;m3=1;
      cntr=16;
      ar=0;
      do average6 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average6: ar=ar+ay1; 
      dm(_avg6)=ar;
      // channel 7
      i1=input+96;m2=6;
      i2=_gxbin;m3=1;
      cntr=16;
      ar=0;
      do average7 until ce;
      	ay1=dm(i1,m2);
      	dm(i2,m3)=ay1;
      average7: ar=ar+ay1; 
      dm(_avg7)=ar;
      // channel 8
      i1=input+97;m2=6;
      cntr=16;
      ar=0;
      do average8 until ce;
      	ay1=dm(i1,m2);
      average8: ar=ar+ay1; 
      dm(_avg8)=ar;
      // channel 9
      i1=input+98;m2=6;
      cntr=16;
      ar=0;
      do average9 until ce;
      	ay1=dm(i1,m2);
      average9: ar=ar+ay1; 
      dm(_avg9)=ar;
      // channel 10
      i1=input+99;m2=6;
      cntr=16;
      ar=0;
      do average10 until ce;
      	ay1=dm(i1,m2);
      average10: ar=ar+ay1; 
      dm(_avg10)=ar;
      // channel 11
      i1=input+100;m2=6;
      cntr=16;
      ar=0;
      do average11 until ce;
      	ay1=dm(i1,m2);
      average11: ar=ar+ay1; 
      dm(_avg11)=ar;
      /*******************************/
    /* the data is summed, and stored in buffers as follows
    	_avg1 = channel 1 = RTD
    	_avg2 = channel 2 = mag z data
    	_avg3 = channel 3 = mag y data
    	_avg4 = channel 4 = mag x data
    	_avg5 = channel 5 = accel z data
    	_avg6 = channel 6 = accel y data
    	_avg7 = channel 7 = accel x data
    	_avg8 = channel 8 = GR //being tested DL
    	_avg9 = channel 9
    	_avg10 = channel 10 
    	_avg11 = channel 11
    */
    
      //copy the gamma ray bin counts from the GRBin_buf to GRBin_CPS, excuted every one second
      i1=_GRBin_CPS;
      i2=_GRBin_buf;
      cntr=20;			//must keep same as the size of the GRBin_buf
      do copyGRbin until ce;
      m2=0;
      ar=dm(i2,m2);
      m2=1;
      dm(i2,m2)=0;		//clear the GRBin_buf at the same time
      copyGRbin: dm(i1,m2)=ar;
      
      ar=dm(_GRCount);
      dm(_GRayCPS)=ar;		//retreive the gamma ray count from GRCount and store to _GRayCPS
      ar=0;					//called every second
      dm(_GRCount)=ar;

    m2 = 0;				//Restore m2}
    rts;
_UpdateChAvg.END:

// ------------------------------------------------------------------------
// 3:ROUTINE - void	AcquireData();
//							  
//
// ------------------------------------------------------------------------
.global	_AcquireData;
_AcquireData:
	ay1 = input;			// Reset input pointer.
	dm(rx_loc)=ay1;
	ay1 = 2;
	dm(_sampling)=ay1;	// This will start the sampling process.	

	// ---------------------------------------
	// Wait for data acquisition to complete.
	// ---------------------------------------
n3_wait1:
	ay1 = dm(_sampling);
#if !__SIMULATION__ 
	none = pass ay1;
	if ne jump n3_wait1;
#endif
	rts; 
_AcquireData.END: 

// -------------------------------------------------------------------------
// 4:LOCAL ROUTINE: void LoadCount(); 
//
//
// -------------------------------------------------------------------------
LoadCount:  // calculate the current motor velocity through the vel_counter


	ax1=dm(_load_in_idx);
	ay1=_load_buf;
	ay0=_load_pos_buf;
	ar=ax1+ay1;
	i1=ar;
	m2=0;
	
	ar=dm(_load_counter);
	
	dm(i1,m2)=ar;		//update the load buffer
	
	ar=ax1+ay0;
	i1=ar;
	m2=0;
	
	ar=dm(_motor_position);
	
	dm(i1,m2)=ar;		//update the load position buffer
	
	ar=0;
	dm(_load_counter)=ar;
	
	ar=ax1+1;
	ax1=ar;
	
	ay1=LOAD_BUF_SIZE;	//size of load buffer
	
	ar=ax1-ay1;
	
	IF eq jump IdxWrap;
	
	dm(_load_in_idx)=ax1;
	
	rts;

IdxWrap:

	ar=0;
	
	dm(_load_in_idx)=ar;
	
	rts;


// -------------------------------------------------------------------------
// 5:ROUTINE: void MotorPos(); 
//
//
// -------------------------------------------------------------------------
.global _MotorPos;  // obtain the current motor position based on PF4 and PF6 quadratic inputs

_MotorPos:

	ena SEC_REG;
	 
	save(i1);
	save(m2);
	
	ar=dm(_load_counter);		//increment the load counter
	ar=ar+1;
	dm(_load_counter)=ar;
  
  	ax0 = dm(Prog_Flag_Data);	//PF port values
  	ay0 = dm(_quad);			//current quadratic values
  	//dm(_previous_quad)=ay0;		//update the previous qudratic values
  	
  	ay1 = 0x0050;
  	ar = ax0 and ay1;
  	ax0 = ar;
  	
  	ar = ax0 - ay0;
  	
  	IF eq jump EndChk;			//not change in PF4 and PF6
  	
  		IF lt jump JmpDn;		//PF4 or PF6 goes from 1 to 0
  		  		
  			ar = ax0 xor ay0;
  			ax1 = ar;
  			ay1 = 0x0020;
  			ar= ax1 - ay1;
  		
  			IF lt jump PF4Up;	//PF4 goes up from 0 to 1
  			
  				ay1 = 0x0010;
  				ar= ax0 and ay1;
  				
  				IF eq jump CCW;	//motor position change counter clockwise
  				
  					jump CW;	//motor position change clockwise
  					
JmpDn:

			ar = ax0 xor ay0;
  			ax1 = ar;
  			ay1 = 0x0020;
  			ar= ax1 - ay1;
  			
  			IF lt jump PF4Dn;
  				
  				ay1 = 0x0010;
  				ar= ax0 and ay1;
  				
  				IF eq jump CW;
  					
  					jump CCW;
  					

PF4Up:			
  				
  		  		ay1 = 0x0040;
  				ar= ax0 and ay1;
  				
  				IF eq jump CW;
  				
  					jump CCW;
  					
PF4Dn:			
  				
  		  		ay1 = 0x0040;
  				ar= ax0 and ay1;
  				
  				IF eq jump CCW;
  				
  					jump CW;  					
  		
  		
CCW:	
	
	ar=dm(_motor_position);
	ar=ar+1;
	dm(_motor_position)=ar;
	
	jump LoadVel;
	
CW:	
	
	ar=dm(_motor_position);
	ar=ar-1;
	dm(_motor_position)=ar;
	
	jump LoadVel;
	
LoadVel:

	ar=dm(_velocity);
	ar=ar+1;
	dm(_velocity)=ar;
	
	call LoadCount;
	
EndChk:

	dm(_quad)=ax0;
	
	restore(m2);
	restore(i1);
	
	dis SEC_REG;
	
	rts;
	
_MotorPos.END:
	   
