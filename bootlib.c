//========================================================================================
//  Copyright Max Fierke (TeenDev)
//  http://www.maxfierke.com
//
//  License: GPLv2
//========================================================================================

#include <nds.h>
#include <stdio.h>
#include <fat.h>
#include <string.h>
#include <sys/stat.h>
#include <malloc.h>
#include <sys/unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include "bootlib.h"

#ifdef __cplusplus
extern "C" {
#endif


//=================================================
#ifdef ARM7
//=================================================


#define FW_READ        0x03

__attribute__((noinline)) static void readFirmware2(uint32 address, uint32 size, uint8 * buffer) {
  uint32 index;

  // Read command
  while (REG_SPICNT & SPI_BUSY);
  REG_SPICNT = SPI_ENABLE | SPI_CONTINUOUS | SPI_DEVICE_NVRAM;
  REG_SPIDATA = FW_READ;
  while (REG_SPICNT & SPI_BUSY);

  // Set the address
  REG_SPIDATA =  (address>>16) & 0xFF;
  while (REG_SPICNT & SPI_BUSY);
  REG_SPIDATA =  (address>>8) & 0xFF;
  while (REG_SPICNT & SPI_BUSY);
  REG_SPIDATA =  (address) & 0xFF;
  while (REG_SPICNT & SPI_BUSY);

  for (index = 0; index < size; index++) {
    REG_SPIDATA = 0;
    while (REG_SPICNT & SPI_BUSY);
    buffer[index] = REG_SPIDATA & 0xFF;
  }
  REG_SPICNT = 0;
}

/*-------------------------------------------------------------------------
resetMemory_ARM7
Clears all of the NDS's RAM that is visible to the ARM7
Written by Darkain.
Modified by Chishm:
 * Added STMIA clear mem loop
--------------------------------------------------------------------------*/
__attribute__((noinline)) static void resetMemory_ARM7 (void)
{
	int i;
	u8 settings1, settings2;

	REG_IME = 0;

	for (i=0x04000400; i<0x04000500; i+=4) {
	  *((u32*)i)=0;
	}
	SOUND_CR = 0;


  for(i=0x040000B0;i<(0x040000B0+0x30);i+=4){
    *((vu32*)i)=0;
  }
  for(i=0x04000100;i<0x04000110;i+=2){
    *((u16*)i)=0;
  }

	//switch to user mode
  __asm volatile(
	"mov r6, #0x1F                \n"
	"msr cpsr, r6                 \n"
	:
	:
	: "r6"
	);

  __asm volatile (
	// clear most of EWRAM - except after 0x023FF800, which has DS settings
	"mov r8, #0x02000000		\n"	// Start address part 1
	"orr r8, r8, #0x8000		\n" // Start address part 2
	"mov r9, #0x02300000		\n" // End address part 1
	"orr r9, r9, #0xff000		\n" // End address part 2
	"orr r9, r9, #0x00800		\n" // End address part 3
	"clear_EXRAM_loop:			\n"
	"stmia r8!, {r0, r1, r2, r3, r4, r5, r6, r7} \n"
	"cmp r8, r9					\n"
	"blt clear_EXRAM_loop		\n"
	:
	:
	: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"
	);

	REG_IE = 0;
	REG_IF = ~0;
	(*(vu32*)(0x04000000-4)) = 0;  //IRQ_HANDLER ARM7 version
	(*(vu32*)(0x04000000-8)) = ~0; //VBLANK_INTR_WAIT_FLAGS, ARM7 version
	REG_POWERCNT = 1;  //turn off power to stuffs

	// Reload DS Firmware settings
	readFirmware2((u32)0x03FE70, 0x1, &settings1);
	readFirmware2((u32)0x03FF70, 0x1, &settings2);

	if (settings1 > settings2) {
		readFirmware2((u32)0x03FE00, 0x70, (u8*)0x027FFC80);
	} else {
		readFirmware2((u32)0x03FF00, 0x70, (u8*)0x027FFC80);
	}
}
void NdsLoaderCheck(void) {
	if(*((vu32*)0x027FFE24) == (u32)0x027FFE04)
	{
		irqDisable (IRQ_ALL);
		*((vu32*)0x027FFE34) = (u32)0x06000000;
		swiSoftReset();
	}
}

void bootndsGBAMPCheck(void) {
	if (*((vu32*)0x027FFE24) == (u32)0x027FFE04) {  // Check for ARM9 reset

        	REG_IME = IME_DISABLE;   // Disable interrupts
        	REG_IF = REG_IF;   // Acknowledge interrupt
        	*((vu32*)0x027FFE34) = (u32)0x08000000;   // Bootloader start address
        	swiSoftReset();   // Jump to boot loader
        }
}

void bootndsM3SCcheck(void) {
	if (*((vu32*)0x027FFE24) == (u32)0x027FFE08) {
		resetMemory_ARM7();

		REG_IME = IME_DISABLE;	// Disable interrupts
		REG_IF = REG_IF;	// Acknowledge interrupt
		*((vu32*)0x027FFE34) = (u32)0x08000000;	// Bootloader start address
		swiSoftReset();	// Jump to boot loader
	}
}

void bootndsCheck(void) {
	if(REG_IPC_FIFO_RX == 17) NdsLoaderCheck();
	if(REG_IPC_FIFO_RX == 18) bootndsGBAMPCheck();
	if(REG_IPC_FIFO_RX == 19) bootndsM3SCcheck();
}

#endif

//=================================================
#ifdef ARM9
//=================================================

#include "load_bin.h"

// --- Chishm's loader --- WORKING
#define LCDC_BANK_C (u16*)0x06840000
#define STORED_FILE_CLUSTER (*(((u32*)LCDC_BANK_C) + 1))
#define INIT_DISC (*(((u32*)LCDC_BANK_C) + 2))
#define WANT_TO_PATCH_DLDI (*(((u32*)LCDC_BANK_C) + 3))

#define STORED_FILE_CLUSTER_OFFSET 4
#define INIT_DISC_OFFSET 8
#define WANT_TO_PATCH_DLDI_OFFSET 12


typedef signed int addr_t;
typedef unsigned char data_t;

#define FIX_ALL	0x01
#define FIX_GLUE	0x02
#define FIX_GOT	0x04
#define FIX_BSS	0x08

enum DldiOffsets {
	DO_magicString = 0x00,			// "\xED\xA5\x8D\xBF Chishm"
	DO_magicToken = 0x00,			// 0xBF8DA5ED
	DO_magicShortString = 0x04,		// " Chishm"
	DO_version = 0x0C,
	DO_driverSize = 0x0D,
	DO_fixSections = 0x0E,
	DO_allocatedSpace = 0x0F,

	DO_friendlyName = 0x10,

	DO_text_start = 0x40,			// Data start
	DO_data_end = 0x44,				// Data end
	DO_glue_start = 0x48,			// Interworking glue start	-- Needs address fixing
	DO_glue_end = 0x4C,				// Interworking glue end
	DO_got_start = 0x50,			// GOT start					-- Needs address fixing
	DO_got_end = 0x54,				// GOT end
	DO_bss_start = 0x58,			// bss start					-- Needs setting to zero
	DO_bss_end = 0x5C,				// bss end

	// IO_INTERFACE data
	DO_ioType = 0x60,
	DO_features = 0x64,
	DO_startup = 0x68,
	DO_isInserted = 0x6C,
	DO_readSectors = 0x70,
	DO_writeSectors = 0x74,
	DO_clearStatus = 0x78,
	DO_shutdown = 0x7C,
	DO_code = 0x80
};

static addr_t readAddr (data_t *mem, addr_t offset) {
	return ((addr_t*)mem)[offset/sizeof(addr_t)];
}

static void writeAddr (data_t *mem, addr_t offset, addr_t value) {
	((addr_t*)mem)[offset/sizeof(addr_t)] = value;
}

static void vramcpy (void* dst, const void* src, int len)
{
	u16* dst16 = (u16*)dst;
	u16* src16 = (u16*)src;

	for ( ; len > 0; len -= 2) {
		*dst16++ = *src16++;
	}
}

static addr_t quickFind (const data_t* data, const data_t* search, size_t dataLen, size_t searchLen) {
	const int* dataChunk = (const int*) data;
	int searchChunk = ((const int*)search)[0];
	addr_t i;
	addr_t dataChunkEnd = (addr_t)(dataLen / sizeof(int));

	for ( i = 0; i < dataChunkEnd; i++) {
		if (dataChunk[i] == searchChunk) {
			if ((i*sizeof(int) + searchLen) > dataLen) {
				return -1;
			}
			if (memcmp (&data[i*sizeof(int)], search, searchLen) == 0) {
				return i*sizeof(int);
			}
		}
	}

	return -1;
}

static const data_t dldiMagicString[] = "\xED\xA5\x8D\xBF Chishm";	// Normal DLDI file
static const data_t dldiMagicLoaderString[] = "\xEE\xA5\x8D\xBF Chishm";	// Different to a normal DLDI file
#define DEVICE_TYPE_DLDI 0x49444C44

extern const u32 _io_dldi;

static bool dldiPatchLoader (data_t *binData, u32 binSize, bool clearBSS)
{
	addr_t memOffset;			// Offset of DLDI after the file is loaded into memory
	addr_t patchOffset;			// Position of patch destination in the file
	addr_t relocationOffset;	// Value added to all offsets within the patch to fix it properly
	addr_t ddmemOffset;			// Original offset used in the DLDI file
	addr_t ddmemStart;			// Start of range that offsets can be in the DLDI file
	addr_t ddmemEnd;			// End of range that offsets can be in the DLDI file
	addr_t ddmemSize;			// Size of range that offsets can be in the DLDI file

	addr_t addrIter;

	data_t *pDH;
	data_t *pAH;

	size_t dldiFileSize = 0;

	// Find the DLDI reserved space in the file
	patchOffset = quickFind (binData, dldiMagicLoaderString, binSize, sizeof(dldiMagicLoaderString));

	if (patchOffset < 0) {
		// does not have a DLDI section
		return false;
	}

	pDH = (data_t*)(((u32*)(&_io_dldi)) -24);
	pAH = &(binData[patchOffset]);

	if (*((u32*)(pDH + DO_ioType)) == DEVICE_TYPE_DLDI) {
		// No DLDI patch
		return false;
	}

	if (pDH[DO_driverSize] > pAH[DO_allocatedSpace]) {
		// Not enough space for patch
		return false;
	}

	dldiFileSize = 1 << pDH[DO_driverSize];

	memOffset = readAddr (pAH, DO_text_start);
	if (memOffset == 0) {
			memOffset = readAddr (pAH, DO_startup) - DO_code;
	}
	ddmemOffset = readAddr (pDH, DO_text_start);
	relocationOffset = memOffset - ddmemOffset;

	ddmemStart = readAddr (pDH, DO_text_start);
	ddmemSize = (1 << pDH[DO_driverSize]);
	ddmemEnd = ddmemStart + ddmemSize;

	// Remember how much space is actually reserved
	pDH[DO_allocatedSpace] = pAH[DO_allocatedSpace];
	// Copy the DLDI patch into the application
	vramcpy (pAH, pDH, dldiFileSize);

	// Fix the section pointers in the header
	writeAddr (pAH, DO_text_start, readAddr (pAH, DO_text_start) + relocationOffset);
	writeAddr (pAH, DO_data_end, readAddr (pAH, DO_data_end) + relocationOffset);
	writeAddr (pAH, DO_glue_start, readAddr (pAH, DO_glue_start) + relocationOffset);
	writeAddr (pAH, DO_glue_end, readAddr (pAH, DO_glue_end) + relocationOffset);
	writeAddr (pAH, DO_got_start, readAddr (pAH, DO_got_start) + relocationOffset);
	writeAddr (pAH, DO_got_end, readAddr (pAH, DO_got_end) + relocationOffset);
	writeAddr (pAH, DO_bss_start, readAddr (pAH, DO_bss_start) + relocationOffset);
	writeAddr (pAH, DO_bss_end, readAddr (pAH, DO_bss_end) + relocationOffset);
	// Fix the function pointers in the header
	writeAddr (pAH, DO_startup, readAddr (pAH, DO_startup) + relocationOffset);
	writeAddr (pAH, DO_isInserted, readAddr (pAH, DO_isInserted) + relocationOffset);
	writeAddr (pAH, DO_readSectors, readAddr (pAH, DO_readSectors) + relocationOffset);
	writeAddr (pAH, DO_writeSectors, readAddr (pAH, DO_writeSectors) + relocationOffset);
	writeAddr (pAH, DO_clearStatus, readAddr (pAH, DO_clearStatus) + relocationOffset);
	writeAddr (pAH, DO_shutdown, readAddr (pAH, DO_shutdown) + relocationOffset);

	if (pDH[DO_fixSections] & FIX_ALL) {
		// Search through and fix pointers within the data section of the file
		for (addrIter = (readAddr(pDH, DO_text_start) - ddmemStart); addrIter < (readAddr(pDH, DO_data_end) - ddmemStart); addrIter++) {
			if ((ddmemStart <= readAddr(pAH, addrIter)) && (readAddr(pAH, addrIter) < ddmemEnd)) {
				writeAddr (pAH, addrIter, readAddr(pAH, addrIter) + relocationOffset);
			}
		}
	}

	if (pDH[DO_fixSections] & FIX_GLUE) {
		// Search through and fix pointers within the glue section of the file
		for (addrIter = (readAddr(pDH, DO_glue_start) - ddmemStart); addrIter < (readAddr(pDH, DO_glue_end) - ddmemStart); addrIter++) {
			if ((ddmemStart <= readAddr(pAH, addrIter)) && (readAddr(pAH, addrIter) < ddmemEnd)) {
				writeAddr (pAH, addrIter, readAddr(pAH, addrIter) + relocationOffset);
			}
		}
	}

	if (pDH[DO_fixSections] & FIX_GOT) {
		// Search through and fix pointers within the Global Offset Table section of the file
		for (addrIter = (readAddr(pDH, DO_got_start) - ddmemStart); addrIter < (readAddr(pDH, DO_got_end) - ddmemStart); addrIter++) {
			if ((ddmemStart <= readAddr(pAH, addrIter)) && (readAddr(pAH, addrIter) < ddmemEnd)) {
				writeAddr (pAH, addrIter, readAddr(pAH, addrIter) + relocationOffset);
			}
		}
	}

	if (clearBSS && (pDH[DO_fixSections] & FIX_BSS)) {
		// Initialise the BSS to 0, only if the disc is being re-inited
		memset (&pAH[readAddr(pDH, DO_bss_start) - ddmemStart] , 0, readAddr(pDH, DO_bss_end) - readAddr(pDH, DO_bss_start));
	}

	return true;
}

bool runNds (const void* loader, u32 loaderSize, u32 cluster, bool initDisc, bool dldiPatchNds)
{
	irqDisable(IRQ_ALL);

	// Direct CPU access to VRAM bank C
	VRAM_C_CR = VRAM_ENABLE | VRAM_C_LCD;
	// Clear VRAM
	memset (LCDC_BANK_C, 0x00, 128 * 1024);
	// Load the loader/patcher into the correct address
	vramcpy (LCDC_BANK_C, loader, loaderSize);

	// Patch the loader with a DLDI for the card
	if (!dldiPatchLoader ((data_t*)LCDC_BANK_C, loaderSize, initDisc)) {
		return false;
	}

	// Set the parameters for the loader
	// STORED_FILE_CLUSTER = cluster;
	writeAddr ((data_t*) LCDC_BANK_C, STORED_FILE_CLUSTER_OFFSET, cluster);
	// INIT_DISC = initDisc;
	writeAddr ((data_t*) LCDC_BANK_C, INIT_DISC_OFFSET, initDisc);
	// WANT_TO_PATCH_DLDI = dldiPatchNds;
	writeAddr ((data_t*) LCDC_BANK_C, WANT_TO_PATCH_DLDI_OFFSET, dldiPatchNds);

	// Give the VRAM to the ARM7
	VRAM_C_CR = VRAM_ENABLE | VRAM_C_ARM7_0x06000000;
	// Reset into a passme loop
	REG_EXMEMCNT |= ARM7_OWNS_ROM | ARM7_OWNS_CARD;
	*((vu32*)0x027FFFFC) = 0;
	*((vu32*)0x027FFE04) = (u32)0xE59FF018;
	*((vu32*)0x027FFE24) = (u32)0x027FFE04;
	REG_IPC_FIFO_TX = 17;
	swiSoftReset();
	return true;
}

bool runNdsFile(const char* filename)
{
	struct stat st;

	if (stat (filename, &st) < 0) {
		return false;
	}

	return runNds (load_bin, load_bin_size, st.st_ino, false, false);
}


// --- M3 stuff --- Taken from MoonShell Source ---  WORKING (Please Confirm)

static void SetM3_EnableCard(void)
{
	// run unlock sequence
	volatile unsigned short tmp ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08E00002 ;
	tmp = *(volatile unsigned short *)0x0800000E ;
	tmp = *(volatile unsigned short *)0x08801FFC ;
	tmp = *(volatile unsigned short *)0x0800104A ;
	tmp = *(volatile unsigned short *)0x08800612 ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08801B66 ;
	tmp = *(volatile unsigned short *)0x08800006 ;
	tmp = *(volatile unsigned short *)0x08000000 ;
}

static void SetM3_EnablePSRAM(void)
{
	// run unlock sequence
	volatile unsigned short tmp ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08E00002 ;
	tmp = *(volatile unsigned short *)0x0800000E ;
	tmp = *(volatile unsigned short *)0x08801FFC ;
	tmp = *(volatile unsigned short *)0x0800104A ;
	tmp = *(volatile unsigned short *)0x08800612 ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08801B66 ;
	tmp = *(volatile unsigned short *)0x08800004 ; // 0=bios?, 4 or c=8MByte?
	tmp = *(volatile unsigned short *)0x08000000 ;

        *(volatile u16*)0x09FFEFFE=0xAA55; // PepsiMan vote: The RAM on the M3 is made writable by writing 0xaa55 to 0x09ffeffe.
}

static void SetM3_EnablePSRAM_notwrite(void)
{
	// run unlock sequence
	volatile unsigned short tmp ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08E00002 ;
	tmp = *(volatile unsigned short *)0x0800000E ;
	tmp = *(volatile unsigned short *)0x08801FFC ;
	tmp = *(volatile unsigned short *)0x0800104A ;
	tmp = *(volatile unsigned short *)0x08800612 ;
	tmp = *(volatile unsigned short *)0x08000000 ;
	tmp = *(volatile unsigned short *)0x08801B66 ;
	tmp = *(volatile unsigned short *)0x08800004 ; // 0=bios?, 4 or c=8MByte?
	tmp = *(volatile unsigned short *)0x08000000 ;
}

#define _REG_WAIT_CR (*(vuint16*)0x04000204)

#include "BootStrap_M3_bin.h"

#define ReadBufCount (16*512)

static void boot_M3(const char *filename)
{

	FILE *fp = fopen(filename, "r");



  u16 *pROM16=(u16*)0x08000000;

  {
    u16 *prb=(u16*)malloc(512);

    fseek(fp,0,SEEK_SET);
    	fread(prb,1,512,fp);
    	fseek(fp,0,SEEK_SET);

    if((prb[2]==0)&&(prb[3]==0)&&(prb[0x10]!=0x4e20)&&(prb[0x11]!=0x5344)&&(prb[0x12]!=0x6c20)){


      u16 *pBootStrap=(u16*)BootStrap_M3_bin;
      u32 BootStrapSize=BootStrap_M3_bin_size;

      SetM3_EnablePSRAM();
      //dmaCopy(pBootStrap,pROM16,BootStrapSize);
	u32 adr = 0;
      for(adr=0;adr<BootStrapSize/2;adr++){
        		pROM16[adr]=pBootStrap[adr];
      }
      SetM3_EnableCard();
      pROM16+=BootStrapSize/2;
    }

    free(prb); prb=NULL;
  }

  {


    u16 *pReadBuf=(u16*)malloc(ReadBufCount);
    u32 ReadCount=0;



    fseek(fp,0,SEEK_SET);

    while(1){


      u32 rs=fread(pReadBuf,1,ReadBufCount,fp);
      ReadCount+=rs;

      SetM3_EnablePSRAM();
      u32 adr = 0;
      	for(adr=0;adr<(rs+1)/2;adr++){
        	pROM16[adr]=pReadBuf[adr];
      	}
      SetM3_EnableCard();
      pROM16+=(rs+1)/2;

      if(rs!=ReadBufCount) break;
    }

    free(pReadBuf); pReadBuf=NULL;
  }


  fclose(fp);

  fatUnmount(PI_DEFAULT);


  SetM3_EnablePSRAM_notwrite();

  {
    u16 KEYS_CUR;

    KEYS_CUR=(~REG_KEYINPUT)&0x3ff;
    while(KEYS_CUR!=0){
      KEYS_CUR=(~REG_KEYINPUT)&0x3ff;
    }
  }

  DC_FlushAll();

  {


    REG_IME = IME_DISABLE;	// Disable interrupts
    REG_EXMEMCNT |= (0x8080);  // ARM7 has access to GBA cart
    *((vu32*)0x027FFE08) = (u32)0xE59FF014;  // ldr pc, 0x027FFE24
    *((vu32*)0x027FFE24) = (u32)0x027FFE08;  // Set ARM9 Loop address (M3/SC)
	REG_IPC_FIFO_TX = 19;
    swiSoftReset();  // Reset
  }

  while(1);
}



#undef _REG_WAIT_CR
#undef ReadBufCount


// --- GBAMP Stuff --- WORKING
void bootndsGBAMP(const char *filename) {

	REG_EXMEMCNT &= ~(0x8080);



 	FILE *handle = fopen(filename, "rb");
 	if(handle < 0) {
 		iprintf("\nLoader has failed!\n");
 	}


	struct stat st;
	u32 cluster;

	stat(filename, &st);

	cluster = st.st_ino;

	fclose(handle);

 	REG_EXMEMCNT |= (0x8080);

 	REG_IME = IME_DISABLE;	// Disable interrupts
 	REG_EXMEMCNT |= (0x8080);  // ARM7 has access to GBA cart
 	*((vu32*)0x027FFFFC) = cluster;  // Start cluster of NDS to load
 	*((vu32*)0x027FFE04) = (u32)0xE59FF018;  // ldr pc, 0x027FFE24
 	*((vu32*)0x027FFE24) = (u32)0x027FFE04;  // Set ARM9 Loop address
	REG_IPC_FIFO_TX = 18;
 	swiSoftReset();  // Reset
}
// -- SC Stuff -- Taken from DSOrganize and MoonShell Sources --- WORKING
#define SSC_Disabled (0)
#define SSC_SDRAM (1)
#define SSC_CF (2)

#define SC_REG_UNLOCK	*(vu16*)(0x09FFFFFE)
static void SetSC_UNLOCK(int SSC)
{
  switch(SSC){
    case SSC_Disabled:
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0x0001;
      SC_REG_UNLOCK = 0x0001;
      break;
    case SSC_SDRAM:
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0x0005;
      SC_REG_UNLOCK = 0x0005;
      break;
    case SSC_CF:
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0xA55A;
      SC_REG_UNLOCK = 0x0003;
      SC_REG_UNLOCK = 0x0003;
  }
}
#undef SC_REG_UNLOCK

#define _REG_WAIT_CR (*(vuint16*)0x04000204)

#include "BootStrap_SC_bin.h"

#define ReadBufCount (16*512)

void boot_SCCF(const char *filename)
{
  	FILE *fp = fopen(filename, "r");

  	vu16 *pROM16=(vu16*)0x08000000;

  	{
   	u16 *prb=(u16*)malloc(8);

    	fseek(fp,0,SEEK_SET);
    	fread(prb,1,8,fp);
    	fseek(fp,0,SEEK_SET);

    	if((prb[2]==0)&&(prb[3]==0)){

      	u16 *pBootStrap=(u16*)BootStrap_SC_bin;
      	u32 BootStrapSize=BootStrap_SC_bin_size;

	u32 adr = 0;
      	for(adr=0;adr<BootStrapSize/2;adr++){
        		pROM16[adr]=pBootStrap[adr];
      	}
      		pROM16+=BootStrapSize/2;
    	}

    		free(prb); prb=NULL;
  	}

  	{

    	u16 *pReadBuf=(u16*)malloc(ReadBufCount);
   	u32 ReadCount=0;

    	fseek(fp,0,SEEK_SET);

	while(1){
      		SetSC_UNLOCK(SSC_CF);
      		u32 rs=fread(pReadBuf,1,ReadBufCount,fp);
      		ReadCount+=rs;

      		SetSC_UNLOCK(SSC_SDRAM);
	  	u32 adr = 0;
      		for(adr=0;adr<(rs+1)/2;adr++){
        		pROM16[adr]=pReadBuf[adr];
      		}
      		pROM16+=(rs+1)/2;

      		if(rs!=ReadBufCount){
        		SetSC_UNLOCK(SSC_CF);
        		break;
      		}
    		}

    			free(pReadBuf); pReadBuf=NULL;
  		}

  		fclose(fp);
  		fatUnmount(PI_DEFAULT);

  		SetSC_UNLOCK(SSC_Disabled);

  		DC_FlushAll();

  		{
    		REG_IME = IME_DISABLE;	// Disable interrupts
    		REG_EXMEMCNT |= (0x8080);  // ARM7 has access to GBA cart
   		*((vu32*)0x027FFE08) = (u32)0xE59FF014;  // ldr pc, 0x027FFE24
   		*((vu32*)0x027FFE24) = (u32)0x027FFE08;  // Set ARM9 Loop address (SCCF)
		REG_IPC_FIFO_TX = 19;
		swiSoftReset();  // Reset
		}

  		while(1);
}
// --- EZ4 Stuff --- Taken from MoonShell --- Working (Please Confirm)
static void EZ4_OpenWrite()
{
//	  WAIT_CR &= ~0x80;
	*(vuint16 *)0x9fe0000 = 0xd200;
	*(vuint16 *)0x8000000 = 0x1500;
	*(vuint16 *)0x8020000 = 0xd200;
	*(vuint16 *)0x8040000 = 0x1500;
	*(vuint16 *)0x9C40000 = 0x1500;
	*(vuint16 *)0x9fc0000 = 0x1500;
}

static void EZ4_CloseWrite()
{
	*(vuint16 *)0x9fe0000 = 0xd200;
	*(vuint16 *)0x8000000 = 0x1500;
	*(vuint16 *)0x8020000 = 0xd200;
	*(vuint16 *)0x8040000 = 0x1500;
	*(vuint16 *)0x9C40000 = 0xd200;
	*(vuint16 *)0x9fc0000 = 0x1500;
//	WAIT_CR |= 0x80;

}

#define EZ4_NandControl_Flash (0)
#define EZ4_NandControl_SD (1)

static void EZ4_SetNandControl(uint16  control)
{
	*(vuint16 *)0x9fe0000 = 0xd200;
	*(vuint16 *)0x8000000 = 0x1500;
	*(vuint16 *)0x8020000 = 0xd200;
	*(vuint16 *)0x8040000 = 0x1500;
	*(vuint16 *)0x9400000 = control;
	*(vuint16 *)0x9fc0000 = 0x1500;
}

static inline void SetEZ4_open(void)
{return;
  EZ4_OpenWrite();
  EZ4_SetNandControl(EZ4_NandControl_SD);
}

static inline void SetEZ4_close(void)
{return;
  EZ4_CloseWrite();
  EZ4_SetNandControl(EZ4_NandControl_SD);
}

#include "BootStrap_M3_bin.h"

#define ReadBufCount (16*512)

static void boot_EZ4(const char *filename)
{
  FILE *fp = fopen(filename, "r");

  u16 *pROM16=(u16*)0x08000000;

  {
    u16 *prb=(u16*)malloc(512);

    fseek(fp,0,SEEK_SET);
    fread(prb,1,512,fp);
    fseek(fp,0,SEEK_SET);

    if((prb[2]==0)&&(prb[3]==0)&&(prb[0x10]!=0x4e20)&&(prb[0x11]!=0x5344)&&(prb[0x12]!=0x6c20)){


      u16 *pBootStrap=(u16*)BootStrap_M3_bin;
      u32 BootStrapSize=BootStrap_M3_bin_size;

      SetEZ4_close();
      u32 adr = 0;
      	for(adr=0;adr<BootStrapSize/2;adr++){
        		pROM16[adr]=pBootStrap[adr];
      	}
      SetEZ4_open();
      pROM16+=BootStrapSize/2;
    }

    free(prb); prb=NULL;
  }

  {


    u16 *pReadBuf=(u16*)malloc(ReadBufCount);
    u32 ReadCount=0;



    fseek(fp,0,SEEK_SET);

    while(1){


      u32 rs=fread(pReadBuf,1,ReadBufCount,fp);
      ReadCount+=rs;

      SetEZ4_close();
        u32 adr = 0;
      	for(adr=0;adr<(rs+1)/2;adr++){
        	pROM16[adr]=pReadBuf[adr];
      	}
      SetEZ4_open();
      pROM16+=(rs+1)/2;

      if(rs!=ReadBufCount) break;
    }

    free(pReadBuf); pReadBuf=NULL;
  }


  fclose(fp);

  fatUnmount(PI_DEFAULT);


  SetEZ4_close();

  {
    u16 KEYS_CUR;

    KEYS_CUR=(~REG_KEYINPUT)&0x3ff;
    while(KEYS_CUR!=0){
      KEYS_CUR=(~REG_KEYINPUT)&0x3ff;
    }
  }

  DC_FlushAll();

  {


    REG_IME = IME_DISABLE;	// Disable interrupts
    REG_EXMEMCNT |= (0x8080);  // ARM7 has access to GBA cart
    *((vu32*)0x027FFE08) = (u32)0xE59FF014;  // ldr pc, 0x027FFE24
    *((vu32*)0x027FFE24) = (u32)0x027FFE08;  // Set ARM9 Loop address (M3/SC)
    REG_IPC_FIFO_TX = 19;
    swiSoftReset();  // Reset
  }

  while(1);
}



#undef _REG_WAIT_CR
#undef ReadBufCount


void bootndsSC(const char *filename) {
	boot_SCCF(filename);
}

void bootndsM3(const char *filename) {
	boot_M3(filename);
}

void bootndsEZ4(const char *filename) {
	boot_EZ4(filename);
}

void bootnds(const char *filename) {

	fatInitDefault();
	struct stat st;
	stat("/.", &st);
	int device_id = st.st_dev;


	if(device_id==1178816589) // GBAMP
        {
		bootndsGBAMP(filename);
        }

	if(device_id==1178813267) // SuperCard CF
        {
		bootndsSC(filename);
        }
	if(device_id==1146307411) // SuperCard SD
	{
		bootndsSC(filename);
	}

	if(device_id==1178809165) // M3 CF
	{
		bootndsM3(filename);
	}
	if(device_id==1146303309) // M3 SD
	{
		bootndsM3(filename);
	}
	if(device_id==1146313285) // EZ4
	{
		bootndsEZ4(filename);
	}
	/*if(device_id==1396982611) // SuperCard DS ONE
	{
		//bootndsSCONE(filename);
	}

	if(device_id==1179923538) // R4 and M3S
	{
		//bootndsR4M3S(filename);
	}*/
	else {
		runNdsFile(filename);
	}
}
#endif

#ifdef __cplusplus
}
#endif

