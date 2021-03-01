// WSPR transmitter for the Raspberry Pi. See accompanying README
// file for a description on how to use this code.

// License:
//   This program is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program.  If not, see <http://www.gnu.org/licenses/>.

// ha7ilm: added RPi2 support based on a patch to PiFmRds by Cristophe
// Jacquet and Richard Hirst: http://git.io/vn7O9

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <malloc.h>
#include <time.h>
#include <sys/time.h>
#include <getopt.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <pthread.h>
#include <sys/timex.h>

#ifdef __cplusplus
extern "C" {
#include "mailbox.h"
}
#endif /* __cplusplus */


// Note on accessing memory in RPi:
//
// There are 3 (yes three) address spaces in the Pi:
// Physical addresses
//   These are the actual address locations of the RAM and are equivalent
//   to offsets into /dev/mem.
//   The peripherals (DMA engine, PWM, etc.) are located at physical
//   address 0x2000000 for RPi1 and 0x3F000000 for RPi2/3.
// Virtual addresses
//   These are the addresses that a program sees and can read/write to.
//   Addresses 0x00000000 through 0xBFFFFFFF are the addresses available
//   to a program running in user space.
//   Addresses 0xC0000000 and above are available only to the kernel.
//   The peripherals start at address 0xF2000000 in virtual space but
//   this range is only accessible by the kernel. The kernel could directly
//   access peripherals from virtual addresses. It is not clear to me my
//   a user space application running as 'root' does not have access to this
//   memory range.
// Bus addresses
//   This is a different (virtual?) address space that also maps onto
//   physical memory.
//   The peripherals start at address 0x7E000000 of the bus address space.
//   The DRAM is also available in bus address space in 4 different locations:
//   0x00000000 "L1 and L2 cached alias"
//   0x40000000 "L2 cache coherent (non allocating)"
//   0x80000000 "L2 cache (only)"
//   0xC0000000 "Direct, uncached access"
//
// Accessing peripherals from user space (virtual addresses):
//   The technique used in this program is that mmap is used to map portions of
//   /dev/mem to an arbitrary virtual address. For example, to access the
//   GPIO's, the gpio range of addresses in /dev/mem (physical addresses) are
//   mapped to a kernel chosen virtual address. After the mapping has been
//   set up, writing to the kernel chosen virtual address will actually
//   write to the GPIO addresses in physical memory.
//
// Accessing RAM from DMA engine
//   The DMA engine is programmed by accessing the peripheral registers but
//   must use bus addresses to access memory. Thus, to use the DMA engine to
//   move memory from one virtual address to another virtual address, one needs
//   to first find the physical addresses that corresponds to the virtual
//   addresses. Then, one needs to find the bus addresses that corresponds to
//   those physical addresses. Finally, the DMA engine can be programmed. i.e.
//   DMA engine access should use addresses starting with 0xC.
//
// The perhipherals in the Broadcom documentation are described using their bus
// addresses and structures are created and calculations performed in this
// program to figure out how to access them with virtual addresses.

#define ABORT(a) exit(a)
// Used for debugging
#define MARK std::cout << "Currently in file: " << __FILE__ << " line: " << __LINE__ << std::endl

// PLLD clock frequency.
// For RPi1, after NTP converges, these is a 2.5 PPM difference between
// the PPM correction reported by NTP and the actual frequency offset of
// the crystal. This 2.5 PPM offset is not present in the RPi2 and RPi3.
// This 2.5 PPM offset is compensated for here, but only for the RPi1.
#ifdef RPI23
#define F_PLLD_CLK   (500000000.0)
#else
#ifdef RPI1
#define F_PLLD_CLK   (500000000.0*(1-2.500e-6))
#else
#error "RPI version macro is not defined"
#endif
#endif
// Empirical value for F_PWM_CLK that produces WSPR symbols that are 'close' to
// 0.682s long. For some reason, despite the use of DMA, the load on the PI
// affects the TX length of the symbols. However, the varying symbol length is
// compensated for in the main loop.
#define F_PWM_CLK_INIT (31156186.6125761)

// WSRP nominal symbol time
#define WSPR_SYMTIME (8192.0/12000.0)
// How much random frequency offset should be added to WSPR transmissions
// if the --offset option has been turned on.
#define WSPR_RAND_OFFSET 80
#define WSPR15_RAND_OFFSET 8

// Choose proper base address depending on RPI1/RPI23 macro from makefile.
// PERI_BASE_PHYS is the base address of the peripherals, in physical
// address space.
#ifdef RPI23
#define PERI_BASE_PHYS 0x3f000000
#define MEM_FLAG 0x04
#else
#ifdef RPI1
#define PERI_BASE_PHYS 0x20000000
#define MEM_FLAG 0x0c
#else
#error "RPI version macro is not defined"
#endif
#endif

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// peri_base_virt is the base virtual address that a userspace program (this
// program) can use to read/write to the the physical addresses controlling
// the peripherals. This address is mapped at runtime using mmap and /dev/mem.
// This must be declared global so that it can be called by the atexit
// function.
volatile unsigned *peri_base_virt = NULL;

// Given an address in the bus address space of the peripherals, this
// macro calculates the appropriate virtual address to use to access
// the requested bus address space. It does this by first subtracting
// 0x7e000000 from the supplied bus address to calculate the offset into
// the peripheral address space. Then, this offset is added to peri_base_virt
// Which is the base address of the peripherals, in virtual address space.
#define ACCESS_BUS_ADDR(buss_addr) *(volatile int*)((long int)peri_base_virt+(buss_addr)-0x7e000000)
// Given a bus address in the peripheral address space, set or clear a bit.
#define SETBIT_BUS_ADDR(base, bit) ACCESS_BUS_ADDR(base) |= 1<<bit
#define CLRBIT_BUS_ADDR(base, bit) ACCESS_BUS_ADDR(base) &= ~(1<<bit)

// The following are all bus addresses.
#define GPIO_BUS_BASE (0x7E200000)
//added for TX SWITCH
#define GPIO_SET_BASE (0x7E20001c)
#define GPIO_CLR_BASE (0x7E200028)
#define CM_GP0CTL_BUS (0x7e101070)
#define CM_GP0DIV_BUS (0x7e101074)
#define PADS_GPIO_0_27_BUS  (0x7e10002c)
#define CLK_BUS_BASE (0x7E101000)
#define DMA_BUS_BASE (0x7E007000)
#define PWM_BUS_BASE  (0x7e20C000) /* PWM controller */

// Convert from a bus address to a physical address.
#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

typedef enum {WSPR,TONE} mode_type;

// Structure used to control clock generator
struct GPCTL {
    char SRC         : 4;
    char ENAB        : 1;
    char KILL        : 1;
    char             : 1;
    char BUSY        : 1;
    char FLIP        : 1;
    char MASH        : 2;
    unsigned int     : 13;
    char PASSWD      : 8;
};

// Structure used to tell the DMA engine what to do
struct CB {
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int RES1;
    volatile unsigned int RES2;
};

// DMA engine status registers
struct DMAregs {
    volatile unsigned int CS;
    volatile unsigned int CONBLK_AD;
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int DEBUG;
};

// Virtual and bus addresses of a page of physical memory.
struct PageInfo {
    void* b; // bus address
    void* v; // virtual address
};

// Must be global so that exit handlers can access this.
static struct {
  int handle;                      /* From mbox_open() */
  unsigned mem_ref = 0;            /* From mem_alloc() */
  unsigned bus_addr;               /* From mem_lock() */
  unsigned char *virt_addr = NULL; /* From mapmem() */ //ha7ilm: originally uint8_t
  unsigned pool_size;
  unsigned pool_cnt;
} mbox;

// Use the mbox interface to allocate a single chunk of memory to hold
// all the pages we will need. The bus address and the virtual address
// are saved in the mbox structure.
void allocMemPool(unsigned numpages) {
  // Allocate space.
  mbox.mem_ref = mem_alloc(mbox.handle, 4096*numpages, 4096, MEM_FLAG);
  // Lock down the allocated space and return its bus address.
  mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
  // Conert the bus address to a physical address and map this to virtual
  // (aka user) space.
  mbox.virt_addr = (unsigned char*)mapmem(BUS_TO_PHYS(mbox.bus_addr), 4096*numpages);
  // The number of pages in the pool. Never changes!
  mbox.pool_size=numpages;
  // How many of the created pages have actually been used.
  mbox.pool_cnt=0;
  //printf("allocMemoryPool bus_addr=%x virt_addr=%x mem_ref=%x\n",mbox.bus_addr,(unsigned)mbox.virt_addr,mbox.mem_ref);
}

// Returns the virtual and bus address (NOT physical address!) of another
// page in the pool.
void getRealMemPageFromPool(void ** vAddr, void **bAddr) {
  if (mbox.pool_cnt>=mbox.pool_size) {
    std::cerr << "Error: unable to allocated more pages!" << std::endl;
    ABORT(-1);
  }
  unsigned offset = mbox.pool_cnt*4096;
  *vAddr = (void*)(((unsigned)mbox.virt_addr) + offset);
  *bAddr = (void*)(((unsigned)mbox.bus_addr) + offset);
  //printf("getRealMemoryPageFromPool bus_addr=%x virt_addr=%x\n", (unsigned)*pAddr,(unsigned)*vAddr);
  mbox.pool_cnt++;
}

// Free the memory pool
void deallocMemPool() {
  if(mbox.virt_addr!=NULL) {
    unmapmem(mbox.virt_addr, mbox.pool_size*4096);
  }
  if (mbox.mem_ref!=0) {
    mem_unlock(mbox.handle, mbox.mem_ref);
    mem_free(mbox.handle, mbox.mem_ref);
  }
}

// Disable the PWM clock and wait for it to become 'not busy'.
void disable_clock() {
  // Check if mapping has been set up yet.
  if (peri_base_virt==NULL) {
    return;
  }
  // Disable the clock (in case it's already running) by reading current
  // settings and only clearing the enable bit.
  auto settings=ACCESS_BUS_ADDR(CM_GP0CTL_BUS);
  // Clear enable bit and add password
  settings=(settings&0x7EF)|0x5A000000;
  // Disable
  ACCESS_BUS_ADDR(CM_GP0CTL_BUS) = *((int*)&settings);
  // Wait for clock to not be busy.
  while (true) {
    if (!(ACCESS_BUS_ADDR(CM_GP0CTL_BUS)&(1<<7))) {
      break;
    }
  }
}

// Turn on TX
void txon() {
  // Set function select for GPIO4.
  // Fsel 000 => input
  // Fsel 001 => output
  // Fsel 100 => alternate function 0
  // Fsel 101 => alternate function 1
  // Fsel 110 => alternate function 2
  // Fsel 111 => alternate function 3
  // Fsel 011 => alternate function 4
  // Fsel 010 => alternate function 5
  // Function select for GPIO is configured as 'b100 which selects
  // alternate function 0 for GPIO4. Alternate function 0 is GPCLK0.
  // See section 6.2 of Arm Peripherals Manual.
  SETBIT_BUS_ADDR(GPIO_BUS_BASE , 14);
  CLRBIT_BUS_ADDR(GPIO_BUS_BASE , 13);
  CLRBIT_BUS_ADDR(GPIO_BUS_BASE , 12);
  
  //GPIO 5 USED FOR TX SWITCH
  //SET AS OUTPUT
  CLRBIT_BUS_ADDR(GPIO_BUS_BASE , 17);
  CLRBIT_BUS_ADDR(GPIO_BUS_BASE , 16);
  SETBIT_BUS_ADDR(GPIO_BUS_BASE , 15);

  // Set GPIO drive strength, more info: http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 0;  //2mA -3.4dBm
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 1;  //4mA +2.1dBm
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 2;  //6mA +4.9dBm
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 3;  //8mA +6.6dBm(default)
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 4;  //10mA +8.2dBm
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 5;  //12mA +9.2dBm
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 6;  //14mA +10.0dBm
  ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 7;  //16mA +10.6dBm

  disable_clock();

  // Set clock source as PLLD.
  struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 3,0x5a};

  // Enable clock.
  setupword = {6/*SRC*/, 1, 0, 0, 0, 3,0x5a};
  ACCESS_BUS_ADDR(CM_GP0CTL_BUS) = *((int*)&setupword);
  
  // TX SWITCH HIGH
  SETBIT_BUS_ADDR(GPIO_SET_BASE , 5);
  usleep(300);
}

// Turn transmitter off
void txoff() {
  //struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a};
  //ACCESS_BUS_ADDR(CM_GP0CTL_BUS) = *((int*)&setupword);
  disable_clock();
  //TX SWITCH LOW
  SETBIT_BUS_ADDR(GPIO_CLR_BASE , 5);
}

// Transmit symbol sym for tsym seconds.
//
// TODO:
// Upon entering this function at the beginning of a WSPR transmission, we
// do not know which DMA table entry is being processed by the DMA engine.
#define PWM_CLOCKS_PER_ITER_NOMINAL 1000
void txSym(
  const int & sym_num,
  const double & center_freq,
  const double & tone_spacing,
  const double & tsym,
  const std::vector <double> & dma_table_freq,
  const double & f_pwm_clk,
  struct PageInfo instrs[],
  struct PageInfo & constPage,
  int & bufPtr
) {
  const int f0_idx=sym_num*2;
  const int f1_idx=f0_idx+1;
  const double f0_freq=dma_table_freq[f0_idx];
  const double f1_freq=dma_table_freq[f1_idx];
  const double tone_freq=center_freq-1.5*tone_spacing+sym_num*tone_spacing;
  // Double check...
  assert((tone_freq>=f0_freq)&&(tone_freq<=f1_freq));
  const double f0_ratio=1.0-(tone_freq-f0_freq)/(f1_freq-f0_freq);
  //cout << "f0_ratio = " << f0_ratio << std::endl;
  assert ((f0_ratio>=0)&&(f0_ratio<=1));
  const long int n_pwmclk_per_sym=round(f_pwm_clk*tsym);

  long int n_pwmclk_transmitted=0;
  long int n_f0_transmitted=0;
  //printf("<instrs[bufPtr] begin=%x>",(unsigned)&instrs[bufPtr]);
  while (n_pwmclk_transmitted<n_pwmclk_per_sym) {
    // Number of PWM clocks for this iteration
    long int n_pwmclk=PWM_CLOCKS_PER_ITER_NOMINAL;
    // Iterations may produce spurs around the main peak based on the iteration
    // frequency. Randomize the iteration period so as to spread this peak
    // around.
    n_pwmclk+=round((rand()/((double)RAND_MAX+1.0)-.5)*n_pwmclk)*1;
    if (n_pwmclk_transmitted+n_pwmclk>n_pwmclk_per_sym) {
      n_pwmclk=n_pwmclk_per_sym-n_pwmclk_transmitted;
    }

    // Calculate number of clocks to transmit f0 during this iteration so
    // that the long term average is as close to f0_ratio as possible.
    const long int n_f0=round(f0_ratio*(n_pwmclk_transmitted+n_pwmclk))-n_f0_transmitted;
    const long int n_f1=n_pwmclk-n_f0;

    // Configure the transmission for this iteration
    // Set GPIO pin to transmit f0
    bufPtr++;
    while( ACCESS_BUS_ADDR(DMA_BUS_BASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].b)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.b + f0_idx*4;

    // Wait for n_f0 PWM clocks
    bufPtr++;
    while( ACCESS_BUS_ADDR(DMA_BUS_BASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].b)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f0;

    // Set GPIO pin to transmit f1
    bufPtr++;
    while( ACCESS_BUS_ADDR(DMA_BUS_BASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].b)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.b + f1_idx*4;

    // Wait for n_f1 PWM clocks
    bufPtr=(bufPtr+1) % (1024);
    while( ACCESS_BUS_ADDR(DMA_BUS_BASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].b)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f1;

    // Update counters
    n_pwmclk_transmitted+=n_pwmclk;
    n_f0_transmitted+=n_f0;
  }
  //printf("<instrs[bufPtr]=%x %x>",(unsigned)instrs[bufPtr].v,(unsigned)instrs[bufPtr].b);
}

// Turn off (reset) DMA engine
void unSetupDMA(){
  // Check if mapping has been set up yet.
  if (peri_base_virt==NULL) {
    return;
  }
  //cout << "Exiting!" << std::endl;
  struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS_BUS_ADDR(DMA_BUS_BASE));
  DMA0->CS =1<<31;  // reset dma controller
  txoff();
}

// Truncate at bit lsb. i.e. set all bits less than lsb to zero.
double bit_trunc(
  const double & d,
  const int & lsb
) {
  return floor(d/pow(2.0,lsb))*pow(2.0,lsb);
}

// Program the tuning words into the DMA table.
void setupDMATab(
  const double & center_freq_desired,
  const double & tone_spacing,
  const double & plld_actual_freq,
  std::vector <double> & dma_table_freq,
  double & center_freq_actual,
  struct PageInfo & constPage
){
  // Make sure that all the WSPR tones can be produced solely by
  // varying the fractional part of the frequency divider.
  center_freq_actual=center_freq_desired;
  double div_lo=bit_trunc(plld_actual_freq/(center_freq_desired-1.5*tone_spacing),-12)+pow(2.0,-12);
  double div_hi=bit_trunc(plld_actual_freq/(center_freq_desired+1.5*tone_spacing),-12);
  if (floor(div_lo)!=floor(div_hi)) {
    center_freq_actual=plld_actual_freq/floor(div_lo)-1.6*tone_spacing;
    std::stringstream temp;
    temp << std::setprecision(6) << std::fixed << "  Warning: center frequency has been changed to " << center_freq_actual/1e6 << " MHz" << std::endl;
    std::cout << temp.str();
    std::cout << "  because of hardware limitations!" << std::endl;
  }

  // Create DMA table of tuning words. WSPR tone i will use entries 2*i and
  // 2*i+1 to generate the appropriate tone.
  double tone0_freq=center_freq_actual-1.5*tone_spacing;
  std::vector <long int> tuning_word(1024);
  for (int i=0;i<8;i++) {
    double tone_freq=tone0_freq+(i>>1)*tone_spacing;
    double div=bit_trunc(plld_actual_freq/tone_freq,-12);
    if (i%2==0) {
      div=div+pow(2.0,-12);
    }
    tuning_word[i]=((int)(div*pow(2.0,12)));
  }
  // Fill the remaining table, just in case...
  for (int i=8;i<1024;i++) {
    double div=500+i;
    tuning_word[i]=((int)(div*pow(2.0,12)));
  }

  // Program the table
  dma_table_freq.resize(1024);
  for (int i=0;i<1024;i++) {
    dma_table_freq[i]=plld_actual_freq/(tuning_word[i]/pow(2.0,12));
    ((int*)(constPage.v))[i] = (0x5a<<24)+tuning_word[i];
    if ((i%2==0)&&(i<8)) {
      assert((tuning_word[i]&(~0xfff))==(tuning_word[i+1]&(~0xfff)));
    }
  }

}

// Create the memory structures needed by the DMA engine and perform initial
// clock configuration.
void setupDMA(
  struct PageInfo & constPage,
  struct PageInfo & instrPage,
  struct PageInfo instrs[]
){
  allocMemPool(1025);

  // Allocate a page of ram for the constants
  getRealMemPageFromPool(&constPage.v, &constPage.b);

  // Create 1024 instructions allocating one page at a time.
  // Even instructions target the GP0 Clock divider
  // Odd instructions target the PWM FIFO
  int instrCnt = 0;
  while (instrCnt<1024) {
    // Allocate a page of ram for the instructions
    getRealMemPageFromPool(&instrPage.v, &instrPage.b);

    // make copy instructions
    // Only create as many instructions as will fit in the recently
    // allocated page. If not enough space for all instructions, the
    // next loop will allocate another page.
    struct CB* instr0= (struct CB*)instrPage.v;
    int i;
    for (i=0; i<(signed)(4096/sizeof(struct CB)); i++) {
      instrs[instrCnt].v = (void*)((long int)instrPage.v + sizeof(struct CB)*i);
      instrs[instrCnt].b = (void*)((long int)instrPage.b + sizeof(struct CB)*i);
      instr0->SOURCE_AD = (unsigned long int)constPage.b+2048;
      instr0->DEST_AD = PWM_BUS_BASE+0x18 /* FIF1 */;
      instr0->TXFR_LEN = 4;
      instr0->STRIDE = 0;
      //instr0->NEXTCONBK = (int)instrPage.b + sizeof(struct CB)*(i+1);
      instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
      instr0->RES1 = 0;
      instr0->RES2 = 0;

      // Shouldn't this be (instrCnt%2) ???
      if (i%2) {
        instr0->DEST_AD = CM_GP0DIV_BUS;
        instr0->STRIDE = 4;
        instr0->TI = (1<<26/* no wide*/) ;
      }

      if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (long int)instrs[instrCnt].b;
      instr0++;
      instrCnt++;
    }
  }
  // Create a circular linked list of instructions
  ((struct CB*)(instrs[1023].v))->NEXTCONBK = (long int)instrs[0].b;

  // set up a clock for the PWM
  ACCESS_BUS_ADDR(CLK_BUS_BASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;  // Source=PLLD and disable
  usleep(1000);
  //ACCESS_BUS_ADDR(CLK_BUS_BASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
  ACCESS_BUS_ADDR(CLK_BUS_BASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002000;  // set PWM div to 2, for 250MHz
  ACCESS_BUS_ADDR(CLK_BUS_BASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;  // Source=PLLD and enable
  usleep(1000);

  // set up pwm
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x0 /* CTRL*/) = 0;
  usleep(1000);
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x4 /* status*/) = -1;  // clear errors
  usleep(1000);
  // Range should default to 32, but it is set at 2048 after reset on my RPi.
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x10)=32;
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x20)=32;
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
  usleep(1000);
  ACCESS_BUS_ADDR(PWM_BUS_BASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;

  //activate dma
  struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS_BUS_ADDR(DMA_BUS_BASE));
  DMA0->CS =1<<31;  // reset
  DMA0->CONBLK_AD=0;
  DMA0->TI=0;
  DMA0->CONBLK_AD = (unsigned long int)(instrPage.b);
  DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
}

// Convert string to uppercase
void to_upper(
  char *str
) {
  while(*str) {
    *str = toupper(*str);
    str++;
  }
}

// Encode call, locator, and dBm into WSPR codeblock.
void wspr(
  const char* call,
  const char* l_pre,
  const char* dbm,
  unsigned char* symbols
) {
  // pack prefix in nadd, call in n1, grid, dbm in n2
  char* c, buf[16];
  strncpy(buf, call, 16);
  c=buf;
  to_upper(c);
  unsigned long ng,nadd=0;

  if(strchr(c, '/')){ //prefix-suffix
    nadd=2;
    int i=strchr(c, '/')-c; //stroke position
    int n=strlen(c)-i-1; //suffix len, prefix-call len
    c[i]='\0';
    if(n==1) ng=60000-32768+(c[i+1]>='0'&&c[i+1]<='9'?c[i+1]-'0':c[i+1]==' '?38:c[i+1]-'A'+10); // suffix /A to /Z, /0 to /9
    if(n==2) ng=60000+26+10*(c[i+1]-'0')+(c[i+2]-'0'); // suffix /10 to /99
    if(n>2){ // prefix EA8/, right align
      ng=(i<3?36:c[i-3]>='0'&&c[i-3]<='9'?c[i-3]-'0':c[i-3]-'A'+10);
      ng=37*ng+(i<2?36:c[i-2]>='0'&&c[i-2]<='9'?c[i-2]-'0':c[i-2]-'A'+10);
      ng=37*ng+(i<1?36:c[i-1]>='0'&&c[i-1]<='9'?c[i-1]-'0':c[i-1]-'A'+10);
      if(ng<32768) nadd=1; else ng=ng-32768;
      c=c+i+1;
    }
  }

  int i=(isdigit(c[2])?2:isdigit(c[1])?1:0); //last prefix digit of de-suffixed/de-prefixed callsign
  int n=strlen(c)-i-1; //2nd part of call len
  unsigned long n1;
  n1=(i<2?36:c[i-2]>='0'&&c[i-2]<='9'?c[i-2]-'0':c[i-2]-'A'+10);
  n1=36*n1+(i<1?36:c[i-1]>='0'&&c[i-1]<='9'?c[i-1]-'0':c[i-1]-'A'+10);
  n1=10*n1+c[i]-'0';
  n1=27*n1+(n<1?26:c[i+1]-'A');
  n1=27*n1+(n<2?26:c[i+2]-'A');
  n1=27*n1+(n<3?26:c[i+3]-'A');

  //if(rand() % 2) nadd=0;
  if(!nadd){
    // Copy locator locally since it is declared const and we cannot modify
    // its contents in-place.
    char l[4];
    strncpy(l, l_pre, 4);
    to_upper(l); //grid square Maidenhead locator (uppercase)
    ng=180*(179-10*(l[0]-'A')-(l[2]-'0'))+10*(l[1]-'A')+(l[3]-'0');
  }
  int p = atoi(dbm);    //EIRP in dBm={0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
  int corr[]={0,-1,1,0,-1,2,1,0,-1,1};
  p=p>60?60:p<0?0:p+corr[p%10];
  unsigned long n2=(ng<<7)|(p+64+nadd);

  // pack n1,n2,zero-tail into 50 bits
  char packed[11] = {
    static_cast<char>(n1>>20),
    static_cast<char>(n1>>12),
    static_cast<char>(n1>>4),
    static_cast<char>(((n1&0x0f)<<4)|((n2>>18)&0x0f)),
    static_cast<char>(n2>>10),
    static_cast<char>(n2>>2),
    static_cast<char>((n2&0x03)<<6),
    0,
    0,
    0,
    0
  };

  // convolutional encoding K=32, r=1/2, Layland-Lushbaugh polynomials
  int k = 0;
  int j,s;
  int nstate = 0;
  unsigned char symbol[176];
  for(j=0;j!=sizeof(packed);j++){
     for(i=7;i>=0;i--){
        unsigned long poly[2] = { 0xf2d05351L, 0xe4613c47L };
        nstate = (nstate<<1) | ((packed[j]>>i)&1);
        for(s=0;s!=2;s++){   //convolve
           unsigned long n = nstate & poly[s];
           int even = 0;  // even := parity(n)
           while(n){
              even = 1 - even;
              n = n & (n - 1);
           }
           symbol[k] = even;
           k++;
        }
     }
  }

  // interleave symbols
  const unsigned char npr3[162] = {
     1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,
     0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,
     0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
     0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,
     0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,
     0,0 };
  for(i=0;i!=162;i++){
     // j0 := bit reversed_values_smaller_than_161[i]
     unsigned char j0;
     p=-1;
     for(k=0;p!=i;k++){
        for(j=0;j!=8;j++)   // j0:=bit_reverse(k)
          j0 = ((k>>j)&1)|(j0<<1);
        if(j0<162)
          p++;
     }
     symbols[j0]=npr3[j0]|symbol[i]<<1; //interleave and add sync std::vector
  }
}

// Wait for the system clock's minute to reach one second past 'minute'
void wait_every(
  int minute
) {
  time_t t;
  struct tm* ptm;
  for(;;){
    time(&t);
    ptm = gmtime(&t);
    if((ptm->tm_min % minute) == 0 && ptm->tm_sec == 0) break;
    usleep(1000);
  }
  usleep(1000000); // wait another second
}

void print_usage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "  wspr [options] callsign locator tx_pwr_dBm f1 <f2> <f3> ..." << std::endl;
  std::cout << "    OR" << std::endl;
  std::cout << "  wspr [options] --test-tone f" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -h --help" << std::endl;
  std::cout << "    Print out this help screen." << std::endl;
  std::cout << "  -p --ppm ppm" << std::endl;
  std::cout << "    Known PPM correction to 19.2MHz RPi nominal crystal frequency." << std::endl;
  std::cout << "  -s --self-calibration" << std::endl;
  std::cout << "    Check NTP before every transmission to obtain the PPM error of the" << std::endl;
  std::cout << "    crystal (default setting!)." << std::endl;
  std::cout << "  -f --free-running" << std::endl;
  std::cout << "    Do not use NTP to correct frequency error of RPi crystal." << std::endl;
  std::cout << "  -r --repeat" << std::endl;
  std::cout << "    Repeatedly, and in order, transmit on all the specified command line freqs." << std::endl;
  std::cout << "  -x --terminate <n>" << std::endl;
  std::cout << "    Terminate after n transmissions have been completed." << std::endl;
  std::cout << "  -o --offset" << std::endl;
  std::cout << "    Add a random frequency offset to each transmission:" << std::endl;
  std::cout << "      +/- " << WSPR_RAND_OFFSET << " Hz for WSPR" << std::endl;
  std::cout << "      +/- " << WSPR15_RAND_OFFSET << " Hz for WSPR-15" << std::endl;
  std::cout << "  -t --test-tone freq" << std::endl;
  std::cout << "    Simply output a test tone at the specified frequency. Only used" << std::endl;
  std::cout << "    for debugging and to verify calibration." << std::endl;
  std::cout << "  -n --no-delay" << std::endl;
  std::cout << "    Transmit immediately, do not wait for a WSPR TX window. Used" << std::endl;
  std::cout << "    for testing only." << std::endl;
  std::cout << std::endl;
  std::cout << "Frequencies can be specified either as an absolute TX carrier frequency, or" << std::endl;
  std::cout << "using one of the following strings. If a string is used, the transmission" << std::endl;
  std::cout << "will happen in the middle of the WSPR region of the selected band." << std::endl;
  std::cout << "  LF LF-15 MF MF-15 160m 160m-15 80m 60m 40m 30m 20m 17m 15m 12m 10m 6m 4m 2m" << std::endl;
  std::cout << "<B>-15 indicates the WSPR-15 region of band <B>." << std::endl;
  std::cout << std::endl;
  std::cout << "Transmission gaps can be created by specifying a TX frequency of 0" << std::endl;
}

void parse_commandline(
  // Inputs
  const int & argc,
  char * const argv[],
  // Outputs
  std::string & callsign,
  std::string & locator,
  std::string & tx_power,
  std::vector <double> & center_freq_set,
  double & ppm,
  bool & self_cal,
  bool & repeat,
  bool & random_offset,
  double & test_tone,
  bool & no_delay,
  mode_type & mode,
  int & terminate
) {
  // Default values
  ppm=0;
  self_cal=true;
  repeat=false;
  random_offset=false;
  test_tone=NAN;
  no_delay=false;
  mode=WSPR;
  terminate=-1;

  static struct option long_options[] = {
    {"help",             no_argument,       0, 'h'},
    {"ppm",              required_argument, 0, 'p'},
    {"self-calibration", no_argument,       0, 's'},
    {"free-running",     no_argument,       0, 'f'},
    {"repeat",           no_argument,       0, 'r'},
    {"terminate",        required_argument, 0, 'x'},
    {"offset",           no_argument,       0, 'o'},
    {"test-tone",        required_argument, 0, 't'},
    {"no-delay",         no_argument,       0, 'n'},
    {0, 0, 0, 0}
  };

  while (true) {
    /* getopt_long stores the option index here. */
    int option_index = 0;
    int c = getopt_long (argc, argv, "hp:sfrx:ot:n",
                     long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      char * endp;
      case 0:
        // Code should only get here if a long option was given a non-null
        // flag value.
        std::cout << "Check code!" << std::endl;
        ABORT(-1);
        break;
      case 'h':
        print_usage();
        ABORT(-1);
        break;
      case 'p':
        ppm=strtod(optarg,&endp);
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse ppm value" << std::endl;
          ABORT(-1);
        }
        break;
      case 's':
        self_cal=true;
        break;
      case 'f':
        self_cal=false;
        break;
      case 'r':
        repeat=true;
        break;
      case 'x':
        terminate=strtol(optarg,&endp,10);
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse termination argument" << std::endl;
          ABORT(-1);
        }
        if (terminate<1) {
          std::cerr << "Error: termination parameter must be >= 1" << std::endl;
          ABORT(-1);
        }
        break;
      case 'o':
        random_offset=true;
        break;
      case 't':
        test_tone=strtod(optarg,&endp);
        mode=TONE;
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse test tone frequency" << std::endl;
          ABORT(-1);
        }
        break;
      case 'n':
        no_delay=true;
        break;
      case '?':
        /* getopt_long already printed an error message. */
        ABORT(-1);
      default:
        ABORT(-1);
    }

  }

  // Parse the non-option parameters
  unsigned int n_free_args=0;
  while (optind<argc) {
    // Check for callsign, locator, tx_power
    if (n_free_args==0) {
      callsign=argv[optind++];
      n_free_args++;
      continue;
    }
    if (n_free_args==1) {
      locator=argv[optind++];
      n_free_args++;
      continue;
    }
    if (n_free_args==2) {
      tx_power=argv[optind++];
      n_free_args++;
      continue;
    }
    // Must be a frequency
    // First see if it is a string.
    double parsed_freq;
    if (!strcasecmp(argv[optind],"LF")) {
      parsed_freq=137500.0;
    } else if (!strcasecmp(argv[optind],"LF-15")) {
      parsed_freq=137612.5;
    } else if (!strcasecmp(argv[optind],"MF")) {
      parsed_freq=475700.0;
    } else if (!strcasecmp(argv[optind],"MF-15")) {
      parsed_freq=475812.5;
    } else if (!strcasecmp(argv[optind],"160m")) {
      parsed_freq=1838100.0;
    } else if (!strcasecmp(argv[optind],"160m-15")) {
      parsed_freq=1838212.5;
    } else if (!strcasecmp(argv[optind],"80m")) {
      parsed_freq=3594100.0;
    } else if (!strcasecmp(argv[optind],"60m")) {
      parsed_freq=5288700.0;
    } else if (!strcasecmp(argv[optind],"40m")) {
      parsed_freq=7040100.0;
    } else if (!strcasecmp(argv[optind],"30m")) {
      parsed_freq=10140200.0;
    } else if (!strcasecmp(argv[optind],"20m")) {
      parsed_freq=14097100.0;
    } else if (!strcasecmp(argv[optind],"17m")) {
      parsed_freq=18106100.0;
    } else if (!strcasecmp(argv[optind],"15m")) {
      parsed_freq=21096100.0;
    } else if (!strcasecmp(argv[optind],"12m")) {
      parsed_freq=24926100.0;
    } else if (!strcasecmp(argv[optind],"10m")) {
      parsed_freq=28126100.0;
    } else if (!strcasecmp(argv[optind],"6m")) {
      parsed_freq=50294500.0;
    } else if (!strcasecmp(argv[optind],"4m")) {
      parsed_freq=70092500.0;
    } else if (!strcasecmp(argv[optind],"2m")) {
      parsed_freq=144490500.0;
    } else {
      // Not a string. See if it can be parsed as a double.
      char * endp;
      parsed_freq=strtod(argv[optind],&endp);
      if ((optarg==endp)||(*endp!='\0')) {
        std::cerr << "Error: could not parse transmit frequency: " << argv[optind] << std::endl;
        ABORT(-1);
      }
    }
    optind++;
    center_freq_set.push_back(parsed_freq);
  }

  // Convert to uppercase
  transform(callsign.begin(),callsign.end(),callsign.begin(),::toupper);
  transform(locator.begin(),locator.end(),locator.begin(),::toupper);

  // Check consistency among command line options.
  if (ppm&&self_cal) {
    std::cout << "Warning: ppm value is being ignored!" << std::endl;
    ppm=0.0;
  }
  if (mode==TONE) {
    if ((callsign!="")||(locator!="")||(tx_power!="")||(center_freq_set.size()!=0)||random_offset) {
      std::cerr << "Warning: callsign, locator, etc. are ignored when generating test tone" << std::endl;
    }
    random_offset=0;
    if (test_tone<=0) {
      std::cerr << "Error: test tone frequency must be positive" << std::endl;
      ABORT(-1);
    }
  } else {
    if ((callsign=="")||(locator=="")||(tx_power=="")||(center_freq_set.size()==0)) {
      std::cerr << "Error: must specify callsign, locator, dBm, and at least one frequency" << std::endl;
      std::cerr << "Try: wspr --help" << std::endl;
      ABORT(-1);
    }
  }

  // Print a summary of the parsed options
  if (mode==WSPR) {
    std::cout << "WSPR packet contents:" << std::endl;
    std::cout << "  Callsign: " << callsign << std::endl;
    std::cout << "  Locator:  " << locator << std::endl;
    std::cout << "  Power:    " << tx_power << " dBm" << std::endl;
    std::cout << "Requested TX frequencies:" << std::endl;
    std::stringstream temp;
    for (unsigned int t=0;t<center_freq_set.size();t++) {
      temp << std::setprecision(6) << std::fixed;
      temp << "  " << center_freq_set[t]/1e6 << " MHz" << std::endl;
    }
    std::cout << temp.str();
    temp.str("");
    if (self_cal) {
      temp << "  NTP will be used to periodically calibrate the transmission frequency" << std::endl;
    } else if (ppm) {
      temp << "  PPM value to be used for all transmissions: " << ppm << std::endl;
    }
    if (terminate>0) {
      temp << "  TX will stop after " << terminate << " transmissions." << std::endl;
    } else if (repeat) {
      temp << "  Transmissions will continue forever until stopped with CTRL-C" << std::endl;
    }
    if (random_offset) {
      temp << "  A small random frequency offset will be added to all transmissions" << std::endl;
    }
    if (temp.str().length()) {
      std::cout << "Extra options:" << std::endl;
      std::cout << temp.str();
    }
    std::cout << std::endl;
  } else {
    std::stringstream temp;
    temp << std::setprecision(6) << std::fixed << "A test tone will be generated at frequency " << test_tone/1e6 << " MHz" << std::endl;
    std::cout << temp.str();
    if (self_cal) {
      std::cout << "NTP will be used to calibrate the tone frequency" << std::endl;
    } else if (ppm) {
      std::cout << "PPM value to be used to generate the tone: " << ppm << std::endl;
    }
    std::cout << std::endl;
  }
}

// Call ntp_adjtime() to obtain the latest calibration coefficient.
void update_ppm(
  double & ppm
) {
  struct timex ntx;
  int status;
  double ppm_new;

  ntx.modes = 0; /* only read */
  status = ntp_adjtime(&ntx);

  if (status != TIME_OK) {
    //cerr << "Error: clock not synchronized" << std::endl;
    //return;
  }

  ppm_new = (double)ntx.freq/(double)(1 << 16); /* frequency scale */
  if (abs(ppm_new)>200) {
    std::cerr << "Warning: absolute ppm value is greater than 200 and is being ignored!" << std::endl;
  } else {
    if (ppm!=ppm_new) {
      std::cout << "  Obtained new ppm value: " << ppm_new << std::endl;
    }
    ppm=ppm_new;
  }
}

/* Return 1 if the difference is negative, otherwise 0.  */
// From StackOverflow:
// http://stackoverflow.com/questions/1468596/c-programming-calculate-elapsed-time-in-milliseconds-unix
int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1) {
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return (diff<0);
}

void timeval_print(struct timeval *tv) {
    char buffer[30];
    time_t curtime;

    //printf("%ld.%06ld", tv->tv_sec, tv->tv_usec);
    curtime = tv->tv_sec;
    //strftime(buffer, 30, "%m-%d-%Y %T", localtime(&curtime));
    strftime(buffer, 30, "UTC %Y-%m-%d %T", gmtime(&curtime));
    printf("%s.%03ld", buffer, (tv->tv_usec+500)/1000);
}

// Create the mbox special files and open mbox.
void open_mbox() {
  mbox.handle = mbox_open();
  if (mbox.handle < 0) {
    std::cerr << "Failed to open mailbox." << std::endl;
    ABORT(-1);
  }
}

// Called when exiting or when a signal is received.
void cleanup() {
  disable_clock();
  unSetupDMA();
  deallocMemPool();
  unlink(LOCAL_DEVICE_FILE_NAME);
}

// Called when a signal is received. Automatically calls cleanup().
void cleanupAndExit(int sig) {
  std::cerr << "Exiting with error; caught signal: " << sig << std::endl;
  cleanup();
  ABORT(-1);
}

void setSchedPriority(int priority) {
  //In order to get the best timing at a decent queue size, we want the kernel
  //to avoid interrupting us for long durations.  This is done by giving our
  //process a high priority. Note, must run as super-user for this to work.
  struct sched_param sp;
  sp.sched_priority=priority;
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (ret) {
    std::cerr << "Warning: pthread_setschedparam (increase thread priority) returned non-zero: " << ret << std::endl;
  }
}

// Create the memory map between virtual memory and the peripheral range
// of physical memory.
void setup_peri_base_virt(
  volatile unsigned * & peri_base_virt
) {
  int mem_fd;
  // open /dev/mem
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    std::cerr << "Error: can't open /dev/mem" << std::endl;
    ABORT (-1);
  }
  peri_base_virt = (unsigned *)mmap(
    NULL,
    0x01000000,  //len
    PROT_READ|PROT_WRITE,
    MAP_SHARED,
    mem_fd,
    PERI_BASE_PHYS  //base
  );
  if ((long int)peri_base_virt==-1) {
    std::cerr << "Error: peri_base_virt mmap error!" << std::endl;
    ABORT(-1);
  }
  close(mem_fd);
}

int main(const int argc, char * const argv[]) {
  //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
  for (int i = 0; i < 64; i++) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = cleanupAndExit;
    sigaction(i, &sa, NULL);
  }
  atexit(cleanup);
  setSchedPriority(30);

#ifdef RPI1
  std::cout << "Detected Raspberry Pi version 1" << std::endl;
#else
#ifdef RPI23
  std::cout << "Detected Raspberry Pi version 2/3" << std::endl;
#else
#error "RPI version macro is not defined"
#endif
#endif

  // Initialize the RNG
  srand(time(NULL));

  // Parse arguments
  std::string callsign;
  std::string locator;
  std::string tx_power;
  std::vector <double> center_freq_set;
  double ppm;
  bool self_cal;
  bool repeat;
  bool random_offset;
  double test_tone;
  bool no_delay;
  mode_type mode;
  int terminate;
  parse_commandline(
    argc,
    argv,
    callsign,
    locator,
    tx_power,
    center_freq_set,
    ppm,
    self_cal,
    repeat,
    random_offset,
    test_tone,
    no_delay,
    mode,
    terminate
  );
  int nbands=center_freq_set.size();

  // Initial configuration
  struct PageInfo constPage;
  struct PageInfo instrPage;
  struct PageInfo instrs[1024];
  setup_peri_base_virt(peri_base_virt);
  // Set up DMA
  open_mbox();
  txon();
  setupDMA(constPage,instrPage,instrs);
  txoff();

  if (mode==TONE) {
    // Test tone mode...
    double wspr_symtime = WSPR_SYMTIME;
    double tone_spacing=1.0/wspr_symtime;

    std::stringstream temp;
    temp << std::setprecision(6) << std::fixed << "Transmitting test tone on frequency " << test_tone/1.0e6 << " MHz" << std::endl;
    std::cout << temp.str();
    std::cout << "Press CTRL-C to exit!" << std::endl;

    txon();
    int bufPtr=0;
    std::vector <double> dma_table_freq;
    // Set to non-zero value to ensure setupDMATab is called at least once.
    double ppm_prev=123456;
    double center_freq_actual;
    while (true) {
      if (self_cal) {
        update_ppm(ppm);
      }
      if (ppm!=ppm_prev) {
        setupDMATab(test_tone+1.5*tone_spacing,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
        //cout << std::setprecision(30) << dma_table_freq[0] << std::endl;
        //cout << std::setprecision(30) << dma_table_freq[1] << std::endl;
        //cout << std::setprecision(30) << dma_table_freq[2] << std::endl;
        //cout << std::setprecision(30) << dma_table_freq[3] << std::endl;
        if (center_freq_actual!=test_tone+1.5*tone_spacing) {
          std::cout << "  Warning: because of hardware limitations, test tone will be transmitted on" << std::endl;
          std::stringstream temp;
          temp << std::setprecision(6) << std::fixed << "  frequency: " << (center_freq_actual-1.5*tone_spacing)/1e6 << " MHz" << std::endl;
          std::cout << temp.str();
        }
        ppm_prev=ppm;
      }
      txSym(0, center_freq_actual, tone_spacing, 60, dma_table_freq, F_PWM_CLK_INIT, instrs, constPage, bufPtr);
    }

    // Should never get here...

  } else {
    // WSPR mode

    // Create WSPR symbols
    unsigned char symbols[162];
    wspr(callsign.c_str(), locator.c_str(), tx_power.c_str(), symbols);
    /*
    printf("WSPR codeblock: ");
    for (int i = 0; i < (signed)(sizeof(symbols)/sizeof(*symbols)); i++) {
      if (i) {
        std::cout << ",";
      }
      printf("%d", symbols[i]);
    }
    printf("\n");
    */

    std::cout << "Ready to transmit (setup complete)..." << std::endl;
    int band=0;
    int n_tx=0;
    for(;;) {
      // Calculate WSPR parameters for this transmission
      double center_freq_desired;
      center_freq_desired = center_freq_set[band];
      bool wspr15 =
           (center_freq_desired > 137600 && center_freq_desired < 137625) || \
           (center_freq_desired > 475800 && center_freq_desired < 475825) || \
           (center_freq_desired > 1838200 && center_freq_desired < 1838225);
      double wspr_symtime = (wspr15) ? 8.0 * WSPR_SYMTIME : WSPR_SYMTIME;
      double tone_spacing=1.0/wspr_symtime;

      // Add random offset
      if ((center_freq_desired!=0)&&random_offset) {
        center_freq_desired+=(2.0*rand()/((double)RAND_MAX+1.0)-1.0)*(wspr15?WSPR15_RAND_OFFSET:WSPR_RAND_OFFSET);
      }

      // Status message before transmission
      std::stringstream temp;
      temp << std::setprecision(6) << std::fixed;
      temp << "Desired center frequency for " << (wspr15?"WSPR-15":"WSPR") << " transmission: "<< center_freq_desired/1e6 << " MHz" << std::endl;
      std::cout << temp.str();

      // Wait for WSPR transmission window to arrive.
      if (no_delay) {
        std::cout << "  Transmitting immediately (not waiting for WSPR window)" << std::endl;
      } else {
        std::cout << "  Waiting for next WSPR transmission window..." << std::endl;
        wait_every((wspr15) ? 15 : 2);
      }

      // Update crystal calibration information
      if (self_cal) {
        update_ppm(ppm);
      }

      // Create the DMA table for this center frequency
      std::vector <double> dma_table_freq;
      double center_freq_actual;
      if (center_freq_desired) {
        setupDMATab(center_freq_desired,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
      } else {
        center_freq_actual=center_freq_desired;
      }

      // Send the message!
      //cout << "TX started!" << std::endl;
      if (center_freq_actual){
        // Print a status message right before transmission begins.
        struct timeval tvBegin, tvEnd, tvDiff;
        gettimeofday(&tvBegin, NULL);
        std::cout << "  TX started at: ";
        timeval_print(&tvBegin);
        std::cout << std::endl;

        struct timeval sym_start;
        struct timeval diff;
        int bufPtr=0;
        txon();
        for (int i = 0; i < 162; i++) {
          gettimeofday(&sym_start,NULL);
          timeval_subtract(&diff, &sym_start, &tvBegin);
          double elapsed=diff.tv_sec+diff.tv_usec/1e6;
          //elapsed=(i)*wspr_symtime;
          double sched_end=(i+1)*wspr_symtime;
          //cout << "symbol " << i << " " << wspr_symtime << std::endl;
          //cout << sched_end-elapsed << std::endl;
          double this_sym=sched_end-elapsed;
          this_sym=(this_sym<.2)?.2:this_sym;
          this_sym=(this_sym>2*wspr_symtime)?2*wspr_symtime:this_sym;
          txSym(symbols[i], center_freq_actual, tone_spacing, sched_end-elapsed, dma_table_freq, F_PWM_CLK_INIT, instrs, constPage, bufPtr);
        }
        n_tx++;

        // Turn transmitter off
        txoff();

        // End timestamp
        gettimeofday(&tvEnd, NULL);
        std::cout << "  TX ended at:   ";
        timeval_print(&tvEnd);
        timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
        printf(" (%ld.%03ld s)\n", tvDiff.tv_sec, (tvDiff.tv_usec+500)/1000);

      } else {
        std::cout << "  Skipping transmission" << std::endl;
        usleep(1000000);
      }

      // Advance to next band
      band=(band+1)%nbands;
      if ((band==0)&&!repeat) {
        break;
      }
      if ((terminate>0)&&(n_tx>=terminate)) {
        break;
      }

    }
  }

  return 0;
}

