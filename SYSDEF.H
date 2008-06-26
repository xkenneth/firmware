#ifndef SYSDEF_H
#define SYSDEF_H

/* ------------------------------------------------------------ */
/* System-wide definitions                                      */    
/* ------------------------------------------------------------ */

#define XTAL_FREQ       10000                   /* Crystal freq in Khz */
#define XTAL_PERIOD     (1000000/XTAL_FREQ)     /* Period in nsec */
#define CYCLE_FREQ      (XTAL_FREQ*2)           /* Instruction cycle freq in Khz */
#define CYCLE_TIME      (500000/XTAL_FREQ)      /* Instruction cycle time in nsec */

#define SAMPLE_RATE         44100               /* In Hertz     */

// -------------------------------------------------------
// FLASH ADDRESS
//
// They must start on page boundarys.
// 256 bytes makes up a page or sector 0x100
// -------------------------------------------------------
#define FLASH_CODE_ADDR     0x00000 // Program Code
#define FLASH_FACTORY_ADDR  0x0C900 // Factory Constants and Coefficients

#define FLASH_STATUS_ADDR  0x0CA00 // Factory Constants and Coefficients

#define FLASH_PATTERN_ADDR  0x0CC00 // Factory Constants and Coefficients

#define FLASH_PATTERN_SEQ_ADDR  0x0CF00 // Factory Constants and Coefficients

#define FLASH_DATA_ADDR     0x0D000 // Data sequence storage

#define FLASH_LOAD_ADDR     FLASH_DATA_ADDR // Motor load data storage
#define FLASH_LOAD_SIZE		0x00300 // size of flash to save the load data 384 bytes
#define FLASH_POS_ADDR		FLASH_LOAD_ADDR+FLASH_LOAD_SIZE	//flash to save the load position data
#define FLASH_POS_SIZE		0x00300	//size of flash to save the load position data

#define FLASH_LOG_ADDR		FLASH_POS_ADDR+FLASH_POS_SIZE	// Flash address to save the data_seq data

//#define FLASH_BLOCK_SIZE    NUM_OF_SEQ*2  // Number of bytes saved to flash in one block
//#define FLASH_BLOCK_SIZE    FLASH_SECTOR_1M*2  // Number of bytes saved to flash in one block
//#define MAX_FLASH_SIZE      (0x80000-FLASH_BLOCK_SIZE)  // Size of flash to last block able to store

#define MAX_FLASH_SIZE      0x80000  // Size of flash

#define MAX_SECTOR_SIZE      (MAX_FLASH_SIZE-FLASH_SECTOR_1M*2)  // Size of flash to last block able to store

#endif
