#ifndef FLASH_H
#define FLASH_H

#define FLASH_SECTOR_1M     128     // Sector size for 1M parts
#define FLASH_SECTOR_4M     256     // Sector size for 4M parts
#define FLASH_SECTOR_SIZE   256     /* Sector size in bytes */
#define FLASHBUF_SZ         256     /* Buffer used for FLASH reading and writing */
#define FLASHBDMA_PAGE_SZ   0x4000  /* BDMA page size */
#define APPLICATION_PAGE    1       /* Application always starts on a page boundary */

// ----------------------------------------------------------------
// FLASH read/write type definitions (Note: DM_MSB is not allowed) 
// ----------------------------------------------------------------
#define PM_UPPER_WORD       0x0
#define DM_FULL_WORD        0x1
#define DM_LSB              0x3

#ifndef __ECC__

/* -------------------------------- */
/* Assembly language definitions    */
/* -------------------------------- */


#else

/* ------------------------ */
/* 'C' language definitions */    
/* ------------------------ */
#define ACCESS_FLASH_BUF    extern    ushort  r_flashbuf[]

void ReadFlash(short ctrl,short num,long addr,void *data);
void ReadFlashPM(short ctrl,short num,long addr,void pm *data);
void ProgFlash(short ctrl,short num,long addr,void *data);
void ProgFlashPM(short ctrl,short num,long addr,void pm *data);
void LoadApplication();

#endif
#endif

