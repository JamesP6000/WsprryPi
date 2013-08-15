// WSPR transmitter for the Raspberry Pi. See accompanying README and BUILD
// files for descriptions on how to use this code.

/*
License:
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
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
#include <sys/timex.h>

using namespace std;

#define ABORT(a) exit(a)
// Used for debugging
#define MARK std::cout << "Currently in file: " << __FILE__ << " line: " << __LINE__ << std::endl

// PLLD clock frequency.
// There seems to be a 2.5ppm offset between the NTP measured frequency
// error and the frequency error measured by a frequency counter. This fixed
// PPM offset is compensated for here.
#define F_PLLD_CLK   (500000000.0*(1-2.500e-6))
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

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// This must be declared global so that it can be called by the atexit
// function.
volatile unsigned *allof7e = NULL;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13) // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((long int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)
#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define PADS_GPIO_0_27  (0x7e10002c)
#define CM_GP0DIV (0x7e101074)
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)
#define PWMBASE  (0x7e20C000) /* PWM controller */

typedef enum {WSPR,TONE} mode_type;

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

struct PageInfo {
    void* p;  // physical address
    void* v;   // virtual address
};

//struct PageInfo constPage;
//struct PageInfo instrPage;
//struct PageInfo instrs[1024];

// Get the physical address of a page of virtual memory
void getRealMemPage(void** vAddr, void** pAddr) {
    void* a = (void*)valloc(4096);

    ((int*)a)[0] = 1;  // use page to force allocation.

    mlock(a, 4096);  // lock into ram.

    *vAddr = a;  // yay - we know the virtual address

    unsigned long long frameinfo;

    int fp = open("/proc/self/pagemap", 'r');
    lseek(fp, ((long int)a)/4096*8, SEEK_SET);
    read(fp, &frameinfo, sizeof(frameinfo));

    *pAddr = (void*)((long int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr) {

    munlock(vAddr, 4096);  // unlock ram.

    free(vAddr);
}

void txon()
{
    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

    // Set GPIO drive strength, more info: http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 0;  //2mA -3.4dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 1;  //4mA +2.1dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 2;  //6mA +4.9dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 3;  //8mA +6.6dBm(default)
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 4;  //10mA +8.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 5;  //12mA +9.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 6;  //14mA +10.0dBm
    ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 7;  //16mA +10.6dBm

    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 3,0x5a};
    ACCESS(CM_GP0CTL) = *((int*)&setupword);
}

void txoff()
{
    struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a};
    ACCESS(CM_GP0CTL) = *((int*)&setupword);
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
  const vector <double> & dma_table_freq,
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
  //cout << "f0_ratio = " << f0_ratio << endl;
  assert ((f0_ratio>=0)&&(f0_ratio<=1));
  const long int n_pwmclk_per_sym=round(f_pwm_clk*tsym);

  long int n_pwmclk_transmitted=0;
  long int n_f0_transmitted=0;
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
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.p + f0_idx*4;

    // Wait for n_f0 PWM clocks
    bufPtr++;
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f0;

    // Set GPIO pin to transmit f1
    bufPtr++;
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.p + f1_idx*4;

    // Wait for n_f1 PWM clocks
    bufPtr=(bufPtr+1) % (1024);
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f1;

    // Update counters
    n_pwmclk_transmitted+=n_pwmclk;
    n_f0_transmitted+=n_f0;
  }
}

void unSetupDMA(){
    printf("exiting\n");
    struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
    DMA0->CS =1<<31;  // reset dma controller
    txoff();
}

void handSig(const int h) {
  exit(0);
}

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
  vector <double> & dma_table_freq,
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
    stringstream temp;
    temp << setprecision(6) << fixed << "  Warning: center frequency has been changed to " << center_freq_actual/1e6 << " MHz" << endl;
    cout << temp.str();
    cout << "  because of hardware limitations!" << endl;
  }

  // Create DMA table of tuning words. WSPR tone i will use entries 2*i and
  // 2*i+1 to generate the appropriate tone.
  dma_table_freq.resize(1024);
  double tone0_freq=center_freq_actual-1.5*tone_spacing;
  vector <long int> tuning_word(1024);
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
  for (int i=0;i<1024;i++) {
    dma_table_freq[i]=plld_actual_freq/(tuning_word[i]/pow(2.0,12));
    ((int*)(constPage.v))[i] = (0x5a<<24)+tuning_word[i];
    if ((i%2==0)&&(i<8)) {
      assert((tuning_word[i]&(~0xfff))==(tuning_word[i+1]&(~0xfff)));
    }
  }

}

void setupDMA(
  struct PageInfo & constPage,
  struct PageInfo & instrPage,
  struct PageInfo instrs[]
){
   atexit(unSetupDMA);
   signal (SIGINT, handSig);
   signal (SIGTERM, handSig);
   signal (SIGHUP, handSig);
   signal (SIGQUIT, handSig);

   // Allocate a page of ram for the constants
   getRealMemPage(&constPage.v, &constPage.p);

   // Create 1024 instructions allocating one page at a time.
   // Even instructions target the GP0 Clock divider
   // Odd instructions target the PWM FIFO
   int instrCnt = 0;
   while (instrCnt<1024) {
     // Allocate a page of ram for the instructions
     getRealMemPage(&instrPage.v, &instrPage.p);

     // make copy instructions
     // Only create as many instructions as will fit in the recently
     // allocated page. If not enough space for all instructions, the
     // next loop will allocate another page.
     struct CB* instr0= (struct CB*)instrPage.v;
     int i;
     for (i=0; i<(signed)(4096/sizeof(struct CB)); i++) {
       instrs[instrCnt].v = (void*)((long int)instrPage.v + sizeof(struct CB)*i);
       instrs[instrCnt].p = (void*)((long int)instrPage.p + sizeof(struct CB)*i);
       instr0->SOURCE_AD = (unsigned long int)constPage.p+2048;
       instr0->DEST_AD = PWMBASE+0x18 /* FIF1 */;
       instr0->TXFR_LEN = 4;
       instr0->STRIDE = 0;
       //instr0->NEXTCONBK = (int)instrPage.p + sizeof(struct CB)*(i+1);
       instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
       instr0->RES1 = 0;
       instr0->RES2 = 0;

       // Shouldn't this be (instrCnt%2) ???
       if (i%2) {
         instr0->DEST_AD = CM_GP0DIV;
         instr0->STRIDE = 4;
         instr0->TI = (1<<26/* no wide*/) ;
       }

       if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (long int)instrs[instrCnt].p;
       instr0++;
       instrCnt++;
     }
   }
   // Create a circular linked list of instructions
   ((struct CB*)(instrs[1023].v))->NEXTCONBK = (long int)instrs[0].p;

   // set up a clock for the PWM
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;  // Source=PLLD and disable
   usleep(1000);
   //ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002000;  // set PWM div to 2, for 250MHz
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;  // Source=PLLD and enable
   usleep(1000);

   // set up pwm
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = 0;
   usleep(1000);
   ACCESS(PWMBASE + 0x4 /* status*/) = -1;  // clear errors
   usleep(1000);
   // Range should default to 32, but it is set at 2048 after reset on my RPi.
   ACCESS(PWMBASE + 0x10)=32;
   ACCESS(PWMBASE + 0x20)=32;
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
   usleep(1000);
   ACCESS(PWMBASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;

   //activate dma
   struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
   DMA0->CS =1<<31;  // reset
   DMA0->CONBLK_AD=0;
   DMA0->TI=0;
   DMA0->CONBLK_AD = (unsigned long int)(instrPage.p);
   DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
}


//
// Set up a memory regions to access GPIO
//
void setup_io(
  int & mem_fd,
  char * & gpio_mem,
  char * & gpio_map,
  volatile unsigned * & gpio
) {
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit (-1);
    }

    /* mmap GPIO */

    // Allocate MAP block
    if ((gpio_mem = (char *)malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
        printf("allocation error \n");
        exit (-1);
    }

    // Make sure pointer is on 4K boundary
    if ((unsigned long)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

    // Now map it
    gpio_map = (char *)mmap(
                   gpio_mem,
                   BLOCK_SIZE,
                   PROT_READ|PROT_WRITE,
                   MAP_SHARED|MAP_FIXED,
                   mem_fd,
                   GPIO_BASE
               );

    if ((long)gpio_map < 0) {
        printf("mmap error %ld\n", (long int)gpio_map);
        exit (-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


}

void setup_gpios(
  volatile unsigned * & gpio
){
   int g;
   // Switch GPIO 7..11 to output mode

    /************************************************************************\
     * You are about to change the GPIO settings of your computer.          *
     * Mess this up and it will stop working!                               *
     * It might be a good idea to 'sync' before running this program        *
     * so at least you still have your code changes written to the SD-card! *
    \************************************************************************/

    // Set GPIO pins 7-11 to output
    for (g=7; g<=11; g++) {
        INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
        //OUT_GPIO(g);
    }

}

// Convert string to uppercase
void to_upper(char *str)
{   while(*str)
    {
        *str = toupper(*str);
        str++;
    }
}

// Encode call, locator, and dBm into WSPR codeblock.
void wspr(const char* call, const char* l_pre, const char* dbm, unsigned char* symbols)
{
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
   char packed[11] = {n1>>20, n1>>12, n1>>4, ((n1&0x0f)<<4)|((n2>>18)&0x0f),
n2>>10, n2>>2, (n2&0x03)<<6, 0, 0, 0, 0};

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
      symbols[j0]=npr3[j0]|symbol[i]<<1; //interleave and add sync vector
   }
}

// Wait for the system clock's minute to reach one second past 'minute'
void wait_every(int minute)
{
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
  cout << "Usage:" << endl;
  cout << "  wspr [options] callsign locator tx_pwr_dBm f1 <f2> <f3> ..." << endl;
  cout << "    OR" << endl;
  cout << "  wspr [options] --test-tone f" << endl;
  cout << endl;
  cout << "Options:" << endl;
  cout << "  -h --help" << endl;
  cout << "    Print out this help screen." << endl;
  cout << "  -p --ppm ppm" << endl;
  cout << "    Known PPM correction to 19.2MHz RPi nominal crystal frequency." << endl;
  cout << "  -s --self-calibration" << endl;
  cout << "    Call ntp_adjtime() before every transmission to obtain the PPM error of the crystal." << endl;
  cout << "  -r --repeat" << endl;
  cout << "    Repeatedly, and in order, transmit on all the specified command line freqs." << endl;
  cout << "  -x --terminate <n>" << endl;
  cout << "    Terminate after n transmissions have been completed." << endl;
  cout << "  -o --offset" << endl;
  cout << "    Add a random frequency offset to each transmission:" << endl;
  cout << "      +/- " << WSPR_RAND_OFFSET << " Hz for WSPR" << endl;
  cout << "      +/- " << WSPR15_RAND_OFFSET << " Hz for WSPR-15" << endl;
  cout << "  -t --test-tone freq" << endl;
  cout << "    Simply output a test tone and the specified frequency. Only used" << endl;
  cout << "    for debugging and to verify calibration." << endl;
  cout << "  -n --no-delay" << endl;
  cout << "    Transmit immediately, do not wait for a WSPR TX window. Used" << endl;
  cout << "    for testing only." << endl;
  cout << endl;
  cout << "Frequencies can be specified either as an absolute TX carrier frequency, or" << endl;
  cout << "using one of the following strings. If a string is used, the transmission" << endl;
  cout << "will happen in the middle of the WSPR region of the selected band." << endl;
  cout << "  LF LF-15 MF MF-15 160m 160m-15 80m 60m 40m 30m 20m 17m 15m 12m 10m 6m 4m 2m" << endl;
  cout << "<B>-15 indicates the WSPR-15 region of band <B>." << endl;
  cout << endl;
  cout << "Transmission gaps can be created by specifying a TX frequency of 0" << endl;
}

// From StackOverflow:
// http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
std::string exec(const char * cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL)
        result += buffer;
    }
    pclose(pipe);
    return result;
}

void parse_commandline(
  // Inputs
  const int & argc,
  char * const argv[],
  // Outputs
  string & callsign,
  string & locator,
  string & tx_power,
  vector <double> & center_freq_set,
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
  self_cal=false;
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
    {"repeat",           no_argument,       0, 'r'},
    {"terminate",        required_argument, 0, 'x'},
    {"offset",           no_argument,       0, 'o'},
    {"test-tone",        required_argument, 0, 't'},
    {"no-delay",         no_argument,       0, 'n'},
    {0, 0, 0, 0}
  };

  while (1) {
    /* getopt_long stores the option index here. */
    int option_index = 0;
    int c = getopt_long (argc, argv, "hp:srx:ot:",
                     long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      char * endp;
      case 0:
        // Code should only get here if a long option was given a non-null
        // flag value.
        cout << "Check code!" << endl;
        ABORT(-1);
        break;
      case 'h':
        print_usage();
        ABORT(-1);
        break;
      case 'p':
        ppm=strtod(optarg,&endp);
        if ((optarg==endp)||(*endp!='\0')) {
          cerr << "Error: could not parse ppm value" << endl;
          ABORT(-1);
        }
        break;
      case 's':
        self_cal=true;
        break;
      case 'r':
        repeat=true;
        break;
      case 'x':
        terminate=strtol(optarg,&endp,10);
        if ((optarg==endp)||(*endp!='\0')) {
          cerr << "Error: could not parse termination argument" << endl;
          ABORT(-1);
        }
        if (terminate<1) {
          cerr << "Error: termination parameter must be >= 1" << endl;
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
          cerr << "Error: could not parse test tone frequency" << endl;
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
    if (!strcmp(argv[optind],"LF")) {
      parsed_freq=137500.0;
    } else if (!strcmp(argv[optind],"LF-15")) {
      parsed_freq=137612.5;
    } else if (!strcmp(argv[optind],"MF")) {
      parsed_freq=475700.0;
    } else if (!strcmp(argv[optind],"MF-15")) {
      parsed_freq=475812.5;
    } else if (!strcmp(argv[optind],"160m")) {
      parsed_freq=1838100.0;
    } else if (!strcmp(argv[optind],"160m-15")) {
      parsed_freq=1838212.5;
    } else if (!strcmp(argv[optind],"80m")) {
      parsed_freq=3594100.0;
    } else if (!strcmp(argv[optind],"60m")) {
      parsed_freq=5288700.0;
    } else if (!strcmp(argv[optind],"40m")) {
      parsed_freq=7040100.0;
    } else if (!strcmp(argv[optind],"30m")) {
      parsed_freq=10140200.0;
    } else if (!strcmp(argv[optind],"20m")) {
      parsed_freq=14097100.0;
    } else if (!strcmp(argv[optind],"17m")) {
      parsed_freq=18106100.0;
    } else if (!strcmp(argv[optind],"15m")) {
      parsed_freq=21096100.0;
    } else if (!strcmp(argv[optind],"12m")) {
      parsed_freq=24926100.0;
    } else if (!strcmp(argv[optind],"10m")) {
      parsed_freq=28126200.0;
    } else if (!strcmp(argv[optind],"6m")) {
      parsed_freq=50294500.0;
    } else if (!strcmp(argv[optind],"4m")) {
      parsed_freq=70092500.0;
    } else if (!strcmp(argv[optind],"2m")) {
      parsed_freq=144490500.0;
    } else {
      // Not a string. See if it can be parsed as a double.
      char * endp;
      parsed_freq=strtod(argv[optind],&endp);
      if ((optarg==endp)||(*endp!='\0')) {
        cerr << "Error: could not parse transmit frequency: " << argv[optind] << endl;
        ABORT(-1);
      }
    }
    optind++;
    center_freq_set.push_back(parsed_freq);
  }


  // Check consistency among command line options.
  if (ppm&&self_cal) {
    cout << "Warning: ppm value is being ignored!" << endl;
    ppm=0.0;
  }
  if (mode==TONE) {
    if ((callsign!="")||(locator!="")||(tx_power!="")||(center_freq_set.size()!=0)||random_offset) {
      cerr << "Warning: callsign, locator, etc. are ignored when generating test tone" << endl;
    }
    random_offset=0;
    if (test_tone<=0) {
      cerr << "Error: test tone frequency must be positive" << endl;
      ABORT(-1);
    }
  } else {
    if ((callsign=="")||(locator=="")||(tx_power=="")||(center_freq_set.size()==0)) {
      cerr << "Error: must specify callsign, locator, dBm, and at least one frequency" << endl;
      cerr << "Try: wspr --help" << endl;
      ABORT(-1);
    }
  }

  // Print a summary of the parsed options
  if (mode==WSPR) {
    cout << "WSPR packet contents:" << endl;
    cout << "  Callsign: " << callsign << endl;
    cout << "  Locator:  " << locator << endl;
    cout << "  Power:    " << tx_power << " dBm" << endl;
    cout << "Requested TX frequencies:" << endl;
    stringstream temp;
    for (unsigned int t=0;t<center_freq_set.size();t++) {
      temp << setprecision(6) << fixed;
      temp << "  " << center_freq_set[t]/1e6 << " MHz" << endl;
    }
    cout << temp.str();
    temp.str("");
    if (self_cal) {
      temp << "  ntp_adjtime() will be used to peridocially calibrate the transmission frequency" << endl;
    } else if (ppm) {
      temp << "  PPM value to be used for all transmissions: " << ppm << endl;
    }
    if (terminate>0) {
      temp << "  TX will stop after " << terminate << " transmissions." << endl;
    } else if (repeat) {
      temp << "  Transmissions will continue forever until stopped with CTRL-C" << endl;
    }
    if (random_offset) {
      temp << "  A small random frequency offset will be added to all transmisisons" << endl;
    }
    if (temp.str().length()) {
      cout << "Extra options:" << endl;
      cout << temp.str();
    }
    cout << endl;
  } else {
    stringstream temp;
    temp << setprecision(6) << fixed << "A test tone will be generated at frequency " << test_tone/1e6 << " MHz" << endl;
    cout << temp.str();
    if (self_cal) {
      cout << "ntp_adjtime() will be used to calibrate the tone" << endl;
    } else if (ppm) {
      cout << "PPM value to be used to generate the tone: " << ppm << endl;
    }
    cout << endl;
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
    cerr << "Error: clock not synchronized" << endl;
    return;
  }

  ppm_new = (double)ntx.freq/(double)(1 << 16); /* frequency scale */
  if (abs(ppm_new)>200) {
    cerr << "Warning: absolute ppm value is greater than 200 and is being ignored!" << endl;
  } else {
    if (ppm!=ppm_new) {
      cout << "  Obtained new ppm value: " << ppm_new << endl;
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
    strftime(buffer, 30, "UTC %m-%d-%Y %T", gmtime(&curtime));
    printf("%s.%03ld", buffer, (tv->tv_usec+500)/1000);
}

int main(const int argc, char * const argv[]) {
  // Initialize the RNG
  srand(time(NULL));

  // Parse arguments
  string callsign;
  string locator;
  string tx_power;
  vector <double> center_freq_set;
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
  int mem_fd;
  char *gpio_mem, *gpio_map;
  volatile unsigned *gpio = NULL;
  setup_io(mem_fd,gpio_mem,gpio_map,gpio);
  setup_gpios(gpio);
  allof7e = (unsigned *)mmap(
              NULL,
              0x01000000,  //len
              PROT_READ|PROT_WRITE,
              MAP_SHARED,
              mem_fd,
              0x20000000  //base
          );
  if ((long int)allof7e==-1) {
    cerr << "Error: mmap error!" << endl;
    ABORT(-1);
  }
  txon();
  struct PageInfo constPage;
  struct PageInfo instrPage;
  struct PageInfo instrs[1024];
  setupDMA(constPage,instrPage,instrs);
  txoff();

  if (mode==TONE) {
    // Test tone mode...
    double wspr_symtime = WSPR_SYMTIME;
    double tone_spacing=1.0/wspr_symtime;

    stringstream temp;
    temp << setprecision(6) << fixed << "Transmitting test tone on frequency " << test_tone/1.0e6 << " MHz" << endl;
    cout << temp.str();
    cout << "Press CTRL-C to exit!" << endl;

    txon();
    int bufPtr=0;
    vector <double> dma_table_freq;
    // Set to non-zero value to ensure setupDMATab is called at least once.
    double ppm_prev=123456;
    double center_freq_actual;
    while (true) {
      if (self_cal) {
        update_ppm(ppm);
      }
      if (ppm!=ppm_prev) {
        setupDMATab(test_tone+1.5*tone_spacing,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
        //cout << setprecision(30) << dma_table_freq[0] << endl;
        //cout << setprecision(30) << dma_table_freq[1] << endl;
        //cout << setprecision(30) << dma_table_freq[2] << endl;
        //cout << setprecision(30) << dma_table_freq[3] << endl;
        if (center_freq_actual!=test_tone+1.5*tone_spacing) {
          cout << "  Warning: because of hardware limitations, test tone will be transmitted on" << endl;
          stringstream temp;
          temp << setprecision(6) << fixed << "  frequency: " << (center_freq_actual-1.5*tone_spacing)/1e6 << " MHz" << endl;
          cout << temp.str();
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
        cout << ",";
      }
      printf("%d", symbols[i]);
    }
    printf("\n");
    */

    printf("Ready to transmit (setup comlete)...\n");
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
      stringstream temp;
      temp << setprecision(6) << fixed;
      temp << "Desired center frequency for " << (wspr15?"WSPR-15":"WSPR") << " transmission: "<< center_freq_desired/1e6 << " MHz" << endl;
      cout << temp.str();

      // Wait for WSPR transmission window to arrive.
      if (no_delay) {
        cout << "  Transmitting immediately (not waiting for WSPR window)" << endl;
      } else {
        printf("  Waiting for next WSPR transmission window...\n");
        wait_every((wspr15) ? 15 : 2);
      }

      // Update crystal calibration information
      if (self_cal) {
        update_ppm(ppm);
      }

      // Create the DMA table for this center frequency
      vector <double> dma_table_freq;
      double center_freq_actual;
      if (center_freq_desired) {
        setupDMATab(center_freq_desired,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
      } else {
        center_freq_actual=center_freq_desired;
      }

      // Send the message!
      //cout << "TX started!" << endl;
      if (center_freq_actual){
        // Print a status message right before transmission begins.
        struct timeval tvBegin, tvEnd, tvDiff;
        gettimeofday(&tvBegin, NULL);
        cout << "  TX started at: ";
        timeval_print(&tvBegin);
        cout << endl;

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
          //cout << "symbol " << i << " " << wspr_symtime << endl;
          //cout << sched_end-elapsed << endl;
          double this_sym=sched_end-elapsed;
          this_sym=(this_sym<.2)?.2:this_sym;
          this_sym=(this_sym>2*wspr_symtime)?2*wspr_symtime:this_sym;
          txSym(symbols[i], center_freq_actual, tone_spacing, sched_end-elapsed, dma_table_freq, F_PWM_CLK_INIT, instrs, constPage, bufPtr);
        }
        n_tx++;

        // Turn transmitter off
        txoff();

        gettimeofday(&tvEnd, NULL);
        cout << "  TX ended at:   ";
        timeval_print(&tvEnd);
        timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
        printf(" (%ld.%03ld s)\n", tvDiff.tv_sec, (tvDiff.tv_usec+500)/1000);

      } else {
        cout << "  Skipping transmission" << endl;
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

