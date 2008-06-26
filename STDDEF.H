//	added support for PFTYPE register control 12/21/06
//	include processor defines 20070111 dl
#ifndef STDDEF_H
#define STDDEF_H

#ifndef __ECC__

#include	<def218x.h>
#if !(defined(__ADSP21000) || defined(__ADSP219X__) || defined(__ADSP218X__))
#  error sysreg.h not configured
#endif


/********************************/
/* Assembly Macro definitions   */
/********************************/

/* Entry and exit macros for 'C' callable asm routines. */
#define leaf_entry()\
    si = m4;m4=i4;m5=-1;dm(i4,m5)=si

#define leaf_exit()\
    i6=m4;si=dm(i6,m6);i4=m4;m4=si;rts

#define nonleaf_entry()\
    si=m4;m4=i4;mr1=toppcstack;m5=-1;\
    dm(i4,m5)=si;dm(i4,m5)=mr1

#define nonleaf_exit()\
    i6=m4;m5=-1;si=dm(i6,m5);mr1=dm(i6,m6);\
    i4=m4;m4=si;i6=mr1;jump(i6)

/* -------------------------------------------- */
/* Macros for reading parameters from the stack */
/* -------------------------------------------- */
#define readfirst(a)\
    m5=1;i6=1;modify(i6,m4);a=dm(i6,m5)

#define readnext(a)\
    a=dm(i6,m5)

#define readparam(a,b)\
	m5=1;i6=b;modify(i6,m4);a=dm(i6,m5)

/* --------------------------------------- */
/* Macros for pushing values on the stack  */
/* --------------------------------------- */
#define pushfirst(a)\
    m5=-1;dm(i4,m5)=a
    
#define pushnext(a)\
    dm(i4,m5)=a

#define pushfirstDAG(a)\
    m5=-1;si=a;dm(i4,m5)=si

#define pushnextDAG(a)\
    si = a;dm(i4,m5)=si

/* ------------------------------------------------------------------------------------ */
/* Macros for poping values from the stack.                                             */
/* NOTE: It is usually more efficient to use the 'getfirst' and 'getnext' macros        */
/* to retreive the values and then use the 'alter' macro to update the stack pointer.   */       
/* ------------------------------------------------------------------------------------ */
#define popfirst(a)\
    m5=1;i6=i4;modify(i6,m5);a=dm(i6,m6);i4=i6

#define popnext(a)\
    modify(i6,m5);a=dm(i6,m6);i4=i6

#define popfirstDAG(a)\
    m5=1;i6=i4;modify(i6,m5);si=dm(i6,m6);a=si;i4=i6

#define popnextDAG(a)\
    modify(i6,m5);si=dm(i6,m6);a=si;i4=i6

/* --------------------------------------------- */
/* Macros for reading values from the stack      */
/* These do NOT modify the stack pointer.        */
/* --------------------------------------------- */
#define getfirst(a)\
    m5=1;i6=1;modify(i6,m4);a=dm(i6,m5)

#define getnext(a)\
    a=dm(i6,m5)

#define getfirstDAG(a)\
    m5=1;i6=1;modify(i6,m4);si=dm(i6,m5);a=si

#define getnextDAG(a)\
    si=dm(i6,m5);a=si

/* ------------------------- */
/* Modifys the stack pointer */
/* ------------------------- */

#define alter(x)\
    m5=x;modify(i4,m5)

/* Macros for reserving space, saving and restoring registers. */
/* These are to be used in environments with no stack or in    */
/* cases where using the stack is not desired.                 */
#define regspace(x)\
	.var    reg_##x

#define save(x)\
    dm(reg_##x) = x

#define restore(x)\
    x = dm(reg_##x)

#define save_araf()\
    dm(reg_ar) = ar;\
    ar = pass af;\
    dm(reg_af) = ar

#define restore_araf()\
    ar = dm(reg_af);\
    af = pass ar;\
    ar = dm(reg_ar)

/* Macros for setting and clearing PFTYPE values */
#define	PF0		0x00
#define	PF1		0x01
#define	PF2		0x02
#define	PF3		0x03
#define	PF4		0x04
#define	PF5		0x05
#define	PF6		0x06
#define	PF7		0x07
#define ClrPFTYPEBit(f)\
    ax0 = dm(Prog_Flag_Comp_Sel_Ctrl);\
    ar = clrbit f of ax0;\
    dm(Prog_Flag_Comp_Sel_Ctrl) = ar

#define SetPFTYPEBit(f)\
    ax0 = dm(Prog_Flag_Comp_Sel_Ctrl);\
    ar = setbit f of ax0;\
    dm(Prog_Flag_Comp_Sel_Ctrl) = ar

/* Macros for setting and clearing FLAG values */
#define ClrFlagBit(f)\
    ax0 = dm(Prog_Flag_Data);\
    ar = clrbit f of ax0;\
    dm(Prog_Flag_Data) = ar

#define SetFlagBit(f)\
    ax0 = dm(Prog_Flag_Data);\
    ar = setbit f of ax0;\
    dm(Prog_Flag_Data) = ar

#define ToggleFlagBit(f)\
    ax0 = dm(Prog_Flag_Data);\
    ar = tglbit f of ax0;\
    dm(Prog_Flag_Data) = ar

/* Delay macros */
#define NSDelay(a)\
    ar=a;\
    call NSDelay_

#else

/************************/
/* C Macro definitions  */
/************************/

#ifndef NULL
#define NULL    0
#endif    
#define TRUE    1
#define FALSE   0

    /* --------------------------------------- */
    /* Short-hand versions of some basic types */
    /* --------------------------------------- */
typedef unsigned short  ushort;
typedef unsigned int    uint;
typedef unsigned long   ulong;
typedef unsigned char   uchar;
typedef char            byte;
typedef unsigned char   ubyte;    

    //-----------------------------------------------------------------------------
    // Macros for accessing 8-bit bytes from a 16-bit word.
    //-----------------------------------------------------------------------------
#define HIBYTE(val)    (uchar)(((ushort)(val) >> 8)&0xff)
#define LOBYTE(val)    (uchar)((ushort)(val)&0xff)

    /* ----------------------------------- */
    /* Standard integer min and max macros */
    /* ----------------------------------- */
#define min(a,b)    (((a)<(b)) ? (a):(b))
#define max(a,b)    (((a)<(b)) ? (b):(a))

    /* -------------------------------------------------------- */
    /* Includes a a file so that it is seen by 'asm' statements */
    /* -------------------------------------------------------- */
#define incasm(f)	asm("#undef __ECC__\n#include \""##f"\"")		

/* ================================================ */
/* Macros for reading and writing for the I/O space */
/* ================================================ */

    /* ---------------------------------------------------------------- */
    /* These must have a constant IO address. They are inline so they   */
    /* can be used anywhere.                                            */
    /* ---------------------------------------------------------------- */
#define GetIO(reg,val)      asm("%0=io("#reg");" : "=e" (val))
#define SetIO(reg,val)      asm("io("#reg")=%0;" :: "e" (val)) 

    /* ---------------------------------------------------------------- */
    /* These are for accessing a non-constant IO address. These are     */
    /* non-reenterant, so they must NOT be called from within interrupt */
    /* routines or from re-enterant functions.                          */
    /* NOTE: These routines use self-modifying code.                    */
    /* ---------------------------------------------------------------- */
ushort  GetIO_NC(ushort reg);
void    SetIO_NC(ushort reg,ushort val);

    /* ---------------------------------------------------------------- */
    /* These are also for accessing non-constant IO address', but are   */
    /* in-line versions. These should be used when called from an       */
    /* interrupt routine or re-enterent code.                           */
    /* NOTE: These routines use self-modifying code.                    */
    /* ---------------------------------------------------------------- */
inline static ushort GetIO_NCIL(ushort reg)
{
    ushort y;
    asm("i6 = inst1_addr;\
	m5 = 0;\
	sr0 = 0xa;\
	sr = sr or lshift %1 by 4 (LO);\
	px = sr0;\
	sr = lshift sr0 by -8 (LO);\
	ar = setbit 8 of sr0;\
	pm(i6,m5) = ar;\
	nop;\
inst1_addr:\
	ar = io(0);\
	%0=ar;" : "=e" (y) : "d" (reg) : "i6","m5","sr0","sr1","px","ar");
    return y;
}

inline static void SetIO_NCIL(ushort reg,ushort val)
{
	asm("i6 = inst_addr;\
	m5 = 0;\
	ay1= %1;\
	sr0 = 0x8005;\
	sr = sr or lshift %0 by 4 (LO);\
	px = sr0;\
	sr = lshift sr0 by -8 (LO);\
	ar = setbit 8 of sr0;\
	pm(i6,m5) = ar;\
	nop;\
inst_addr:\
	io(0)=ay1;" :: "d" (reg), "e" (val) : "i6","m5","sr0","sr1","px","ar","ay1");
}

/* ------------------------------- */
/* Enables/Disables all interrupts */
/* ------------------------------- */

#define EnableInts()    asm("ENA INTS;")
#define DisableInts()   asm("DIS INTS;")


/* Delay Functions */
extern "OldAsmCall" void    NSDelay(ushort nsec);

/*log10 function */
double log10(double f);

/*abs function */
int abs(int f);

/*labs function */
long labs(long f);

/*ceil function */
double ceil(double f);

/*floor function */
double floor(double f);

#endif

#endif

