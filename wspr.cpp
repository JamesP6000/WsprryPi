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
#include "librpitx/src/librpitx.h"


clkgpio *clk=NULL;
ngfmdmasync *ngfmtest=NULL;


#define ABORT(a) exit(a)
// Used for debugging
#define MARK std::cout << "Currently in file: " << __FILE__ << " line: " << __LINE__ << std::endl
typedef enum {WSPR,TONE} mode_type;

// WSRP nominal symbol time
#define WSPR_SYMTIME (8192.0/12000.0)
// How much random frequency offset should be added to WSPR transmissions
// if the --offset option has been turned on.
#define WSPR_RAND_OFFSET 80
#define WSPR15_RAND_OFFSET 8

// Disable the PWM clock and wait for it to become 'not busy'.
void disable_clock() {
  
}

// Turn on TX
void txon() {
  
  //ACCESS_BUS_ADDR(PADS_GPIO_0_27_BUS) = 0x5a000018 + 7;  //16mA +10.6dBm

  disable_clock();

  
}

// Turn transmitter on
void txoff() {
  disable_clock();
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
  
}

// Turn off (reset) DMA engine
void unSetupDMA(){
  
  txoff();
}

// Truncate at bit lsb. i.e. set all bits less than lsb to zero.
double bit_trunc(
  const double & d,
  const int & lsb
) {
  return floor(d/pow(2.0,lsb))*pow(2.0,lsb);
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


// Called when exiting or when a signal is received.
void cleanup() {
  if(clk!=NULL) {delete clk;clk=NULL;}
  if(ngfmtest!=NULL) {delete ngfmtest;ngfmtest=NULL;}
}

// Called when a signal is received. Automatically calls cleanup().
void cleanupAndExit(int sig) {
  std::cerr << "Exiting with error; caught signal: " << sig << std::endl;
  cleanup();
  ABORT(-1);
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

 
 
  
  if (mode==TONE) {
	if(clk==NULL)
		clk=new clkgpio;
  	clk->SetAdvancedPllMode(true);
    // Test tone mode...
    double wspr_symtime = WSPR_SYMTIME;
    double tone_spacing=1.0/wspr_symtime;

    std::stringstream temp;
    temp << std::setprecision(6) << std::fixed << "Transmitting test tone on frequency " << test_tone/1.0e6 << " MHz" << std::endl;
    std::cout << temp.str();
    std::cout << "Press CTRL-C to exit!" << std::endl;

    txon();
    int bufPtr=0;
    
    // Set to non-zero value to ensure setupDMATab is called at least once.
    double ppm_prev=123456;
    double center_freq_actual;
    //SetTone
	 clk->SetCenterFrequency(test_tone,100);
	clk->enableclk(4);
	clk->SetFrequency(000);
	while(true) usleep(1000000);
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
		center_freq_actual=center_freq_desired;
        
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
		int SR=100*1/wspr_symtime;
		int FifoSize=1000;
		if(ngfmtest==NULL)
			ngfmtest=new ngfmdmasync(center_freq_actual,SR,14,FifoSize);
		
		
       
		
        for (int i = 0; i < 162; i++)
		{
			double tone_freq=-1.5*tone_spacing+symbols[i]*tone_spacing;
			int Nbtx=0;
			while(Nbtx<100)
			{
				usleep(100);
				int Available=ngfmtest->GetBufferAvailable();
				if(Available>FifoSize/2)
				{	
					int Index=ngfmtest->GetUserMemIndex();
					if(Available>100-Nbtx) Available=100-Nbtx;
					//printf("GetIndex=%d\n",Index);
					for(int j=0;j<Available;j++)
					{
						//ngfmtest.SetFrequencySample(Index,((i%10000)>5000)?1000:0);
						ngfmtest->SetFrequencySample(Index+j,tone_freq/*+(rand()/((double)RAND_MAX)-.5)*8.0*/);
						Nbtx++;
						
			
					}
				}
			
		    }
		}
        n_tx++;

        // Turn transmitter off
        ngfmtest->stop();

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

