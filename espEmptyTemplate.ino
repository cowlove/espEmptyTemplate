#pragma GCC optimize("O1")
#ifndef CSIM
#include <esp_intr_alloc.h>
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include <esp_async_memcpy.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include <rtc_wdt.h>
#include <hal/cache_hal.h>
#include <hal/cache_ll.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <xtensa/core-macros.h>
#include <xtensa/hal.h>
#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <freertos/xtensa_timer.h>
#include <freertos/xtensa_rtos.h>

#include "driver/spi_master.h"
#include "rom/ets_sys.h"
#include "soc/dport_access.h"
#include "soc/system_reg.h"

//#include <esp_spi_flash.h>
#include "esp_partition.h"
#include "esp_err.h"

#if CONFIG_FREERTOS_UNICORE != 1 
#error Arduino idf core must be compiled with CONFIG_FREERTOS_UNICORE=y and CONFIG_ESP_INT_WDT=n
#endif

#else 
#include "esp32csim.h"
#endif

#include <vector>
#include <string>
using std::vector;
using std::string;
#include "ascii2keypress.h"


#include "core1.h" 

// Usable pins
// 0-18 21            (20)
// 38-48 (6-16)     (11)
//pin 01234567890123456789012345678901
//    11111111111111111110001000000000
//    00000011111111110000000000000000

//data 8
//addr 16
//clock sync halt write 
// TODO: investigate GPIO input filter, GPIO output sync 

//GPIO0 bits: TODO rearrange bits so addr is in low bits and avoids needed a shift
// Need 19 pines on gpio0: ADDR(16), clock, casInh, RW

unsigned IRAM_ATTR my_nmi(unsigned x) { return 0; }

#if 0
static const struct {
//XOPTS    
//#define FAKE_CLOCK
//#define BUS_DETACH  //fundamental flaw IRQ location is in mpd bank  

#ifdef FAKE_CLOCK
   bool fakeClock     = 1; 
   float histRunSec   = 20;
#else 
   bool fakeClock     = 0;
   float histRunSec   = -20;
#endif 
   bool testPins      = 0;
   bool watchPins     = 0;      // loop forever printing pin values w/ INPUT_PULLUP
   bool dumpSram      = 0;   ;
   bool timingTest    = 0;
   bool bitResponse   = 0;
   bool core0Led      = 0; // broken, PBI loop overwrites entire OUT1 register including ledPin
   bool dumpPsram     = 0;
   bool forceMemTest  = 0;
#define PBI_DEVICE
#ifdef PBI_DEVICE
   bool logicAnalyzer = 0;
   bool maskCore0Int  = 1;
   bool busAnalyzer   = 0;
   bool tcpSendPsram  = 0;
   bool histogram     = 1;
   bool pbiDevice     = 1;
#else
   bool logicAnalyzer = 0;
   bool pbiDevice     = 0;
   bool maskCore0Int  = 0;
   bool busAnalyzer   = 1;
   bool tcpSendPsram  = 1;
   bool histogram     = 0;
#endif
} opt;

struct Pin {
    int gpionum;
    int bitlen;
    inline const int regRd() { return gpionum > 31 ? GPIO_IN1_REG : GPIO_IN_REG; }
    inline const int regWr() { return gpionum > 31 ? GPIO_IN1_REG : GPIO_IN_REG; }
    Pin(int n, int len = 1) : gpionum(n), bitlen(len) {}
    uint32_t const mask() { return ((1 << bitlen) - 1) << shift(); }
    int const shift() { return gpionum & 31; }
};

//GPIO0 pins
static const int      casInh_pin = 0;
static const int      casInh_Shift = casInh_pin;
static const int      casInh_Mask = (0x1 << casInh_pin);               // pin 0 
static const int      clockPin = 1;
static const int      clockMask = (0x1 << clockPin);
static const int      addr0Pin = 2;
static const int      addrShift = addr0Pin;                   // bus address - pins 1-16
static const int      addrMask = 0xffff << addrShift;  // 
static const int      refreshPin = 21;
static const int      refreshMask = (1 << refreshPin);
static const int      readWritePin = 18;
static const int      readWriteMask = (1 << readWritePin); 

//GPIO1 pins
#ifdef HAVE_RESET_PIN
static const int      resetPin = 46;
static const int      resetMask = 1 << (resetPin - 32); 
#endif
static const int      mpdPin = 46;  // active low
static const int      mpdShift = (mpdPin - 32);
static const int      mpdMask = 1 << mpdShift; 
static const int      extSel_Pin = 47; // active high 
static const int      extSel_PortPin = extSel_Pin - 32 /* port1 pin*/;
static const int      extSel_Mask = (1 << extSel_PortPin);
static const int      data0Pin = 38;
static const int      data0Mask = (data0Pin - 32);
static const int      data0PortPin = data0Pin - 32;
static const int      dataShift = data0PortPin;
static const int      dataMask = (0xff << dataShift);

#ifdef HAVE_RESET_PIN
static const uint32_t copyResetMask = 0x40000000;
#endif
static const uint32_t copyMpdMask = 0x40000000;
static const uint32_t copyDataShift = 22;
static const uint32_t copyDataMask = 0xff << copyDataShift;

#if 0
static const int bankBits = 5;
static const int nrBanks = 1 << bankBits;
static const int bankSize = 64 * 1024 / nrBanks;
static const uint16_t bankMask = 0xffff0000 >> bankBits;
static const int bankShift = 16 - bankBits;
#endif 


#define BUSCTL_VOLATILE //volatile
#define RAM_VOLATILE //volatile
#endif

IRAM_ATTR inline void delayTicks(int ticks) { 
    uint32_t startTsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - startTsc < ticks) {}
}

DRAM_ATTR RAM_VOLATILE uint8_t *banks[nrBanks];
DRAM_ATTR uint32_t bankEnable[nrBanks * 2] = {0x0};
DRAM_ATTR RAM_VOLATILE uint8_t atariRam[64 * 1024] = {0x0};
DRAM_ATTR RAM_VOLATILE uint8_t cartROM[] = {
//#include "joust.h"
};
DRAM_ATTR RAM_VOLATILE uint8_t pbiROM[2 * 1024] = {
#include "pbirom.h"
};
DRAM_ATTR uint8_t diskImg[] = {
#include "disk.h"
};


volatile uint32_t busMask = dataMask;

IRAM_ATTR void enableBus() { 
    busMask = dataMask | extSel_Mask | mpdMask;
    delayTicks(240 * 100);
}
IRAM_ATTR void disableBus() {
    delayTicks(240 * 100);    
    busMask = extSel_Mask | mpdMask;
}

std::string vsfmt(const char *format, va_list args);
std::string sfmt(const char *format, ...);

const esp_partition_t *partition;
#define LOCAL_LFS
#ifdef LOCAL_LFS
#include "lfs.h"
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

size_t partition_size = 0x20000;
const int lfsp_block_sz = 4096;


int lfsp_init() { 
    partition = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    //printf("partition find returned %p, part->erase_size %d\n", partition, partition->erase_size);
    return 0;
}
int lfsp_read_block(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size) { 
    //printf("read blk %d off %d size %d\n", block, off, size);                
    return esp_partition_read(partition, block * lfsp_block_sz + off, buffer, size);
}
int lfsp_prog_block(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size) { 
    //printf("prog blk %d off %d size %d\n", block, off, size);                
    return esp_partition_write(partition, block * lfsp_block_sz + off, buffer, size);
}
int lfsp_erase_block(const struct lfs_config *c, lfs_block_t block) { 
    //printf("erase blk %d\n", block);
    return esp_partition_erase_range(partition, lfsp_block_sz * block, lfsp_block_sz);
}

int lsfp_sync(const struct lfs_config *c) { return 0; }



// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = lfsp_read_block,
    .prog  = lfsp_prog_block,
    .erase = lfsp_erase_block,
    .sync  = lsfp_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = 4096,
    .block_count = 128,
    .block_cycles = 500,
    .cache_size = 16,
    .lookahead_size = 16,
};

int lfs_updateTestFile() { 
      // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
    lfs_file_close(&lfs, &file);

    return boot_count;
}
#endif

#if 0 
// TODO: try pin 19,20 (USB d- d+ pins). Move reset to 0 so ESP32 boot doesnt get messed up by low signal   
// TODO: maybe eventually need to drive PBI interrupt pin 
// TODO: so eventaully looks like: pin 0 = reset, pin 19 = casInh input, pin 20 = interrupt, pin 47 = MPD
// TODO: although USB pins moving during ESP32 boot might cause conflict 
// TODO: extend this generally, need to review which ESP32 pins are driven during boot or have strapping resistors   
//
//                               +--casInh_ / ROM read
//                               | +---Clock
//                               | | +--- ADDR                               +-- RW
//                               | | |                                       |  +-- refresh in              +--MPD out
//                               | | |                                       |  |   +---DATA                |  +-- ext sel out 
//                               | | + + + + + + + + +  +  +  +  +  +  +  +  |  |   |  +  +  +  +  +  +  +  |  |  
static const vector<int> pins = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21, 38,39,40,41,42,43,44,45,46,47};
static const int ledPin = 48;
#endif
#define PACK(r0, r1) (((r1 & 0x0001ffc0) << 15) | ((r0 & 0x0007fffe) << 2)) 

volatile uint32_t *gpio0 = (volatile uint32_t *)GPIO_IN_REG;
volatile uint32_t *gpio1 = (volatile uint32_t *)GPIO_IN1_REG;

int psram_sz = 6 * 1024 * 1024;
uint32_t *psram;
static const int dma_sz = 4096 - 64;
static const int dma_bufs = 2; // must be power of 2
static const int dram_sz = dma_sz * dma_bufs;

uint32_t *dram;
static const int testFreq = 1.8 * 1000000;//1000000;
static const int lateThresholdTicks = 180 * 2 * 1000000 / testFreq;
static const uint32_t halfCycleTicks = 240 * 1000000 / testFreq / 2;
uint32_t dramElapsedTsc;
uint32_t lateTsc;
volatile int dramLoopCount = 0;
int resetLowCycles = 0;
int psramLoopCount = 0;
int lateCount = 0;
int lateMax = 0, lateMin = 9999, lateIndex = -1;
int cbCount = 0;
int stop = false;
dedic_gpio_bundle_handle_t bundleIn, bundleOut;
uint32_t lastAddr = -1;
volatile int cumulativeResets = 0;
volatile int currentResetValue = 1;

DRAM_ATTR volatile int core1Reg = 0;

volatile double avgNs1, avgNs2, avgTicks1, avgTicks2;
int maxElapsed1 = 0, maxElapsed2 = 0;
int maxElapsedIndex1 = 0;
void iloop_timings1();
void iloop_timings2();


int maxLoopElapsed, minLoopElapsed, loopElapsedLate = 0;

static bool async_memcpy_callback(async_memcpy_context_t*, async_memcpy_event_t*, void*) {
    cbCount++;  
    return false;
}

void busyWaitCCount(uint32_t cycles) { 
    uint32_t tsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
}

inline void busywait(float sec) {
    uint32_t tsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - tsc < sec * 240 * 1000000) {};
}




void IRAM_ATTR simulateI2c() {
    int cycles = 240 * 1000000 / 100000 / 2;
    uint32_t tsc;
    for(int i = 0; i < 32; i++) { 
        dedic_gpio_cpu_ll_write_all(1);
        tsc = XTHAL_GET_CCOUNT();
        while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
        dedic_gpio_cpu_ll_write_all(0);
        tsc = XTHAL_GET_CCOUNT();
        while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
    }
}

void IRAM_ATTR NEWneopixelWrite(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
    //busyWaitCCount(100);
    //return;       
    uint32_t stsc;
    int color[3] = {green_val, red_val, blue_val};
    int longCycles = 175;
    int shortCycles = 90;
    uint32_t bitMask = 1 << (pin - 32); 
    int i = 0;
    for (int col = 0; col < 3; col++) {
        for (int bit = 0; bit < 8; bit++) {
            if ((color[col] & (1 << (7 - bit)))) {
                // HIGH bit
                //REG_WRITE(GPIO_OUT1_W1TS_REG, bitMask);
                dedic_gpio_cpu_ll_write_all(1);
                stsc = XTHAL_GET_CCOUNT();
                while(XTHAL_GET_CCOUNT() - stsc < longCycles) {}

                dedic_gpio_cpu_ll_write_all(0);
                stsc = XTHAL_GET_CCOUNT();
                while(XTHAL_GET_CCOUNT() - stsc < shortCycles) {}
            } else {
                // LOW bit
                dedic_gpio_cpu_ll_write_all(1);
                stsc = XTHAL_GET_CCOUNT();
                while(XTHAL_GET_CCOUNT() - stsc < shortCycles) {}

                dedic_gpio_cpu_ll_write_all(0);
                stsc = XTHAL_GET_CCOUNT();
                while(XTHAL_GET_CCOUNT() - stsc < longCycles) {}
            }
            i++;
        }
    }
}

//  socat TCP-LISTEN:9999 - > file.bin
bool sendPsramTcp(const char *buf, int len, bool resetWdt = false) { 
#if 0
    //neopixelWrite(ledPin, 0, 0, 8);
    //char *host = "10.250.250.240";
    char *host = "192.168.68.131";
    ////WiFi.begin("Station54", "Local1747"); host = "10.250.250.240";
    //wifiConnect();
    wifiDisconnect();
    wifiConnect();
    WiFiClient wc;
    static const int txSize = 1024;
   
    neopixelWrite(ledPin, 8, 0, 8);
    int r = wc.connect(host, 9999);
    printf("connect() returned %d\n", r);
    uint32_t startMs = millis();
    int count;
    int sent = 0;
    while(sent < len) { 
        if (!wc.connected()) { 
            printf("lost connection");
            return false;
        }
        int pktLen = min(txSize, len - sent);
        r = wc.write((uint8_t *)(buf + sent), pktLen);
        if (r != pktLen) {
            printf("write %d returned %d\n", count, r);
            return false;
        }
        sent += r;
        if (count++ % 100 == 0) { 
            printf("."); 
            fflush(stdout);
        }
        neopixelWrite(ledPin, 0, 0, (count & 127) + 8);
        if (resetWdt) wdtReset();
        yield();
    }
    printf("\nDone %.3f mB/sec\n", psram_sz / 1024.0 / 1024.0 / (millis() - startMs) * 1000.0);
    fflush(stdout);
    neopixelWrite(ledPin, 0, 8, 0);
#endif
    return true;
}

#if 0 
struct Hist2 { 
    static const int maxBucket = 512; // must be power of 2
    int buckets[maxBucket];
    inline void add(uint32_t x) { buckets[x & (maxBucket - 1)]++; }
    Hist2() { clear(); }
    int64_t count() {
        int64_t sum = 0; 
        for(int i = 0; i < maxBucket; i++) sum += buckets[i];
        return sum;
    }
    void clear() { for(int i = 0; i < maxBucket; i++) buckets[i] = 0; }
};
#endif

struct AtariIOCB { 
    uint8_t ICHID,  // handler 
            ICDNO,  // Device number
            ICCOM,  // Command byte 
            ICSTA,  // Status returned
            ICBAL,  // Buffer address (points to 0x9b-terminated string for open command)
            ICBAH,
            ICPTL,  // Address of driver put routine
            ICPTH,
            ICBLL,  // Buffer length 
            ICBLH,
            ICAX1,
            ICAX2,
            ICAX3,
            ICAX4,
            ICAX5,
            ICAX6;
    };

const struct AtariDefStruct {
    int IOCB0 = 0x340;
    int ZIOCB = 0x20;
    int NUMIOCB = 0x8;
    int IOCB_CMD_CLOSE = 0xc;
    int IOCB_CMD_OPEN = 0x3;
    int IOCB_OPEN_READ = 0x4;
    int IOCB_OPEN_WRITE = 0x8;
    int NEWPORT = 0x31ff;
} AtariDef;

//static const int numProfilers = 3;
DRAM_ATTR Hist2 profilers[numProfilers];
int ramReads = 0, ramWrites = 0;

const char *defaultProgram = 
        "10 OPEN #1,4,0,\"J2:\" \233"
        "20 GET #1,A  \233"
        "30 PRINT \"   \"; \233"
        "35 PRINT A;  \233"
        "40 CLOSE #1  \233"
        "41 OPEN #1,8,0,\"J\" \233"
        "42 PUT #1,A + 1 \233"
        "43 CLOSE #1 \233"
        "50 A=USR(1536) \233"
        "51 PRINT \" >>> \"; \233"
        "52 PRINT COUNT; \233"
        "53 COUNT = COUNT + 1 \233"
        "54 OPEN #1,4,0,\"D1:X32Z.DOS\" \233"
        "55 POINT #1,SEC,BYT \233"
        "56 GET #1,A \233"
        "57 CLOSE #1 \233"
        "58 SEC = SEC + 1 \233"
        "59 IF SEC > 100 THEN SEC = 0 \233"
        "70 GOTO 10 \233";
;

vector<uint8_t> simulatedKeypressQueue;
void addSimKeypress(const string &s) { 
    for(auto a : s) simulatedKeypressQueue.push_back(a);
}
int simulatedKeysAvailable = 0;

// CORE0 loop options 
#ifndef FAKE_CLOCK
#define ENABLE_SIO
#define SIM_KEYPRESS
//#define SIM_KEYPRESS_FILE
#endif
struct AtariIO {
    uint8_t buf[2048];
    int ptr = 0;
    int len = 0;
    AtariIO() { 
        strcpy((char *)buf, defaultProgram); 
        len = strlen((char *)buf);
    }
#ifdef SIM_KEYPRESS_FILE

    string filename;
    void open(const string &f) { 
        filename = f;
#else 
    void open() { 
#endif
        ptr = 0; 
    }

    int get() { 
        if (ptr >= len) return -1;
        return buf[ptr++];
    }
    int put(uint8_t c) { 
        if (ptr >= sizeof(buf)) return -1;
        buf[ptr++] = c;
        len = ptr;
#ifdef SIM_KEYPRESS_FILE
        if (filename == "J:KEYS") simulatedKeypressQueue.push_back(c);
#endif
        return 1;
    }
} fakeFile; 

struct AtariDCB { 
   uint8_t 
    DDEVIC,
    DUNIT,
    DCOMND,
    DSTATS,
    DBUFLO,
    DBUFHI,
    DTIMLO,
    DUNUSED,
    DBYTLO,
    DBYTHI,
    DAUX1,
    DAUX2;
};

struct { 
    AtariDCB *dcb = (AtariDCB *)&atariRam[0x300];
    AtariIOCB *ziocb = (AtariIOCB *)&atariRam[0x20];
    AtariIOCB *iocb0 = (AtariIOCB *)&atariRam[0x320];
} atariMem;

struct PbiIocb {
    uint8_t req;
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t cmd;
    uint8_t carry;
};

template<class T> 
struct StructLog { 
    int maxSize;
    StructLog(int maxS = 32) : maxSize(maxS) {}
    vector<T> log;
    void add(const T &t) { 
        log.push_back(t);
        if (log.size() > maxSize) log.erase(log.begin());
    }
    static void printEntry(const T&);
    void print() { for(auto a : log) printEntry(a); }
};
template <class T> void StructLog<T>::printEntry(const T &a) {
    for(int i = 0; i < sizeof(a); i++) printf("%02x ", ((uint8_t *)&a)[i]);
    printf("\n");
}
template <> void StructLog<string>::printEntry(const string &a) { printf("%s\n", a.c_str()); }

struct { 
    StructLog<AtariDCB> dcb; 
    StructLog<AtariIOCB> iocb; 
    StructLog<AtariIOCB> ziocb; 
    StructLog<string> opens;
    void print() {
        printf("DCB log:\n"); dcb.print();
        printf("IOCB log:\n"); iocb.print();
        printf("ZIOCB log:\n"); ziocb.print();
        printf("opened files log:\n"); opens.print();
    }
} structLogs;



// https://www.atarimax.com/jindroush.atari.org/afmtatr.html
struct AtrImageHeader {
    uint16_t magic; // 0x0296;
    uint16_t pars;  // disk image size divided by 0x10
    uint16_t sectorSize; // usually 0x80 or 0x100
    uint8_t parsHigh; // high byte of larger wPars size (added in rev3.00)
    uint32_t crc;       
    uint32_t unused;
    uint8_t flags;
};

struct DiskImage {
    string hostFilename;
    union DiskImageRawData { 
        uint8_t data[1]; 
        AtrImageHeader header;
    } *image;
};
DiskImage atariDisks[8] = {
    {"none", (DiskImage::DiskImageRawData *)diskImg}, 
};

//#define BUS_MONITOR
#ifdef BUS_MONITOR
DRAM_ATTR class BusMonitor { 
    static const int size = 2048; // must be power of 2
    volatile int head;
    int tail;
    uint32_t r0hist[size];//, r1hist[size];
public:
    IRAM_ATTR inline void add(uint32_t r0) { //, uint32_t r1) {
        r0hist[head] = r0; //r1hist[head] = r1;
        head = (head + 1) & (size - 1);
    }
    IRAM_ATTR bool available() { return head != tail; }
    IRAM_ATTR uint32_t get() { 
        uint32_t rval = r0hist[tail];
        tail = (tail + 1) & (size - 1); 
    }
} busMon;
#else
struct {
    IRAM_ATTR inline void add(uint32_t) {}
    IRAM_ATTR inline uint32_t get() { return 0; }
    IRAM_ATTR inline bool available() { return false; }
} busMon;
#endif


int maxBufsUsed = 0;
async_memcpy_handle_t handle = NULL;
volatile int diskReadCount = 0;

// Apparently can't make any function calls from the core0 loops, even inline.  Otherwise it breaks 
// timing on the core1 loop


void IRAM_ATTR core0Loop() { 
    int elapsedSec = 0;
    int pi = 0;
    uint32_t lastCycleCount, startTsc = XTHAL_GET_CCOUNT();
    volatile int *drLoopCount = &dramLoopCount;
    uint8_t ledColor[3] = {0,0,0};
    uint32_t *psramPtr = psram;

    enableBus();

    while(1) {
        uint32_t stsc;
        if (1) { // slow loop down to 1ms
            stsc = XTHAL_GET_CCOUNT();
            while(XTHAL_GET_CCOUNT() - stsc < 240 * 2000) {}
        }
        if (0) { 
            while(busMon.available() && pi < psram_sz / sizeof(psram[0])) { 
            psram[pi++] = busMon.get(); 
            }
        }
        if (0) {
            memcpy(psram, (void *)atariRam, sizeof(atariRam));
        }
        if (0) {
            #ifdef FAKE_CLOCK
            // exercise flash file IO 
            lfs_updateTestFile();
            #endif
        }
        if (0) {
            //rgb[0]++;
            int longCycles = 175;
            int shortCycles = 90;
            uint32_t bitMask = 1 << (ledPin - 32); 
            for (int col = 0; col < 3; col++) {
                for (int bit = 0; bit < 8; bit++) {
                    if (((ledColor[col] >> 2)& (1 << (7 - bit)))) {
                        // HIGH bit
                        //REG_WRITE(GPIO_OUT1_W1TS_REG, bitMask);
                        dedic_gpio_cpu_ll_write_all(1);
                        stsc = XTHAL_GET_CCOUNT();
                        while(XTHAL_GET_CCOUNT() - stsc < longCycles) {}

                        dedic_gpio_cpu_ll_write_all(0);
                        stsc = XTHAL_GET_CCOUNT();
                        while(XTHAL_GET_CCOUNT() - stsc < shortCycles) {}
                    } else {
                        // LOW bit
                        dedic_gpio_cpu_ll_write_all(1);
                        stsc = XTHAL_GET_CCOUNT();
                        while(XTHAL_GET_CCOUNT() - stsc < shortCycles) {}

                        dedic_gpio_cpu_ll_write_all(0);
                        stsc = XTHAL_GET_CCOUNT();
                        while(XTHAL_GET_CCOUNT() - stsc < longCycles) {}
                    }
                }
            }
        }

#ifdef FAKE_CLOCK
        if (0) { 
            // Stuff some fake PBI commands to exercise code in the core0 loop during timing tests 
            static uint32_t lastTsc;
            if (XTHAL_GET_CCOUNT() - lastTsc > 240 * 1000 * 20) {
                lastTsc = XTHAL_GET_CCOUNT();
                volatile PbiIocb *pbiRequest = (PbiIocb *)&pbiROM[0x20];
                static int step = 0;
                if (step == 1) { 
                    // stuff a fake CIO put request
                    #ifdef SIM_KEYPRESS_FILE
                    fakeFile.filename = "J:KEYS";
                    #endif 
                    pbiRequest->cmd = 4; // put 
                    pbiRequest->a = ' ';
                    pbiRequest->req = 1;
                } else if (step == 2) { 
                    // stuff a fake SIO sector read request 
                    volatile AtariDCB *dcb = atariMem.dcb;
                    dcb->DBUFHI = 0x40;
                    dcb->DBUFLO = 0x00;
                    dcb->DDEVIC = 0x31; 
                    dcb->DUNIT = 1;
                    dcb->DAUX1++; 
                    dcb->DAUX2 = 0;
                    dcb->DCOMND = 0x52;
                    pbiRequest->cmd = 7; // read a sector 
                    pbiRequest->req = 1;
                } else if (step == 2) { 
                    
                }
                step = (step + 1) % 3;
            }
        }
#endif 
        if (0) { 
            // why does this work when simulateI2c or even busywait(1) break timing?
            if (1) {
                int cycles = 240 * 1000000 / 100000 / 2;
                uint32_t tsc;
                for(int i = 0; i < 32; i++) { 
                    dedic_gpio_cpu_ll_write_all(1);
                    tsc = XTHAL_GET_CCOUNT();
                    while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
                    dedic_gpio_cpu_ll_write_all(0);
                    tsc = XTHAL_GET_CCOUNT();
                    while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
                }
            } else { 
                simulateI2c();
            }
        }

#ifdef SIM_KEYPRESS
        if (1) { 
            static uint32_t lastTsc;
            if (XTHAL_GET_CCOUNT() - lastTsc > 240 * 1000 * 200) {
                lastTsc = XTHAL_GET_CCOUNT();
                if (simulatedKeysAvailable) { 
                    if (simulatedKeypressQueue.size() > 0) { 
                        uint8_t c = simulatedKeypressQueue[0];
                        simulatedKeypressQueue.erase(simulatedKeypressQueue.begin());
                        if (c != 255) 
                            atariRam[764] = ascii2keypress[c];
                    }
                    simulatedKeysAvailable = simulatedKeypressQueue.size() > 0;
                }
            }
        }
#endif
#if 0 
        if (fakeRamErrCount > 0 && atariRam[1666] == 0) { 
            fakeRamErrCount--;
            atariRam[1666] = 1;
        }
#endif
        if (0) { // why does this do-nothing loop smear out core1 timings?  
            static uint32_t *p = 0;
            //*p = 0;
            //p++;
            if (p >= psram + psram_sz / sizeof(uint32_t))
                p = psram;
        }

        if (0) { // why does enabling this do-nothing loop smear out core1 timings?  
            static uint32_t *p = 0;
            //*p = 0;
            //p++;
            if (p >= psram + psram_sz / sizeof(uint32_t))
                p = psram;
        }
        if (1) {  
            volatile
            PbiIocb *pbiRequest = (PbiIocb *)&pbiROM[0x20];
            
            if (pbiRequest->req != 0) {
                
                #ifdef BUS_DETACH
                // Disable PBI memory device 
                disableBus();
                //diskReadCount = lfs_updateTestFile();
                diskReadCount++;
                #else
                diskReadCount++;
                #endif 

                ledColor[1] += 3;
                ledColor[0] += 2;
                ledColor[2] += 1;
                AtariIOCB *iocb = (AtariIOCB *)&atariRam[AtariDef.IOCB0 + pbiRequest->x]; // todo validate x bounds
                pbiRequest->y = 1; // assume success
                pbiRequest->carry = 0; // assume fail 
                if (pbiRequest->cmd == 1) { // open
                    uint16_t addr = ((uint16_t )atariMem.ziocb->ICBAH) << 8 | atariMem.ziocb->ICBAL;
#ifdef SIM_KEYPRESS_FILE
                    string filename;
                    for(int i = 0; i < 32; i++) { 
                        uint8_t ch = atariRam[addr + i];
                        if (ch == 155) break;
                        filename += ch;    
                    } 
                    structLogs.opens.add(filename);
                    //structLogs.ziocb.add(*atariMem.ziocb);
                    //
                    fakeFile.open(filename);
#else
                    fakeFile.open();
                    structLogs.iocb.add(*iocb);
#endif
                    pbiRequest->carry = 1; 
                } else if (pbiRequest->cmd == 2) { // close
                    pbiRequest->carry = 1; 
                } else if (pbiRequest->cmd == 3) { // get
                    int c = fakeFile.get();
                    if (c < 0) 
                        pbiRequest->y = 136;
                    else
                        pbiRequest->a = c; 
                    pbiRequest->carry = 1; 
                } else if (pbiRequest->cmd == 4) { // put
                    if (fakeFile.put(pbiRequest->a) < 0)
                        pbiRequest->y = 136;
                    pbiRequest->carry = 1; 
                } else if (pbiRequest->cmd == 5) { // status 
                } else if (pbiRequest->cmd == 6) { // special 
                } else if (pbiRequest->cmd == 7) { // low level io, see DCB
#ifdef ENABLE_SIO
                    AtariDCB *dcb = atariMem.dcb;
                    uint16_t addr = (((uint16_t)dcb->DBUFHI) << 8) | dcb->DBUFLO;
                    int sector = (((uint16_t)dcb->DAUX2) << 8) | dcb->DAUX1;
                    structLogs.dcb.add(*dcb);
                    if (dcb->DDEVIC == 0x31 && dcb->DUNIT >= 1 && dcb->DUNIT < sizeof(atariDisks)/sizeof(atariDisks[0]) + 1) {  // Device D1:
                        DiskImage::DiskImageRawData *disk = atariDisks[dcb->DUNIT - 1].image; 
                        if (disk != NULL) { 
                            int sectorSize = disk->header.sectorSize;
                            if (dcb->DCOMND == 0x53) { // SIO status command
                                // drive status https://www.atarimax.com/jindroush.atari.org/asio.html
                                atariRam[addr+0] = (sectorSize == 0x100) ? 0x10 : 0x00; // bit 0 = frame err, 1 = cksum err, wr err, wr prot, motor on, sect size, unused, med density  
                                atariRam[addr+1] = 0xff; // inverted bits: busy, DRQ, data lost, crc err, record not found, head loaded, write pro, not ready 
                                atariRam[addr+2] = 0xff; // timeout for format 
                                atariRam[addr+3] = 0xff; // copy of wd
                                pbiRequest->carry = 1;
                            }
                            int sectorOffset = 16 + (sector - 1) * sectorSize;
                            if (dcb->DCOMND== 0x52) {  // READ sector
                                for(int n = 0; n < sectorSize; n++) 
                                    atariRam[addr + n] = disk->data[sectorOffset + n];
                                //memcpy(&atariRam[addr], &disk->data[sectorOffset], sectorSize);
                                pbiRequest->carry = 1;
                            }
                            if (dcb->DCOMND== 0x50) {  // WRITE sector
                                for(int n = 0; n < sectorSize; n++) 
                                    disk->data[sectorOffset + n] = atariRam[addr + n];
                                //memcpy(&disk->data[sectorOffset], &atariRam[addr], sectorSize);
                                pbiRequest->carry = 1;
                            }
                        }
                    }
#endif // ENABLE_SIO 
                } else if (pbiRequest->cmd == 8) { // IRQ
                    pbiRequest->carry = 0;
                } 
                #ifdef BUS_DETACH
                enableBus();
                #endif
                pbiRequest->req = 0;
                //atariRam[0x0600] = 0;
            }
        }
#if 0 
        //cycleCount++;
        if (psramLoopCount != *drLoopCount) {
            void *dma_buf = dram + (dma_sz * (psramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t));
            if (*drLoopCount - psramLoopCount >= dma_bufs - 6) { 
                printf("esp_aync_memcpy buffer overrun psramLoopCount %d dramLoopCount %d\n", psramLoopCount, dramLoopCount);
                break;
            }
            if (esp_async_memcpy(handle, (void *)(psram + pi), (void *)dma_buf, dma_sz, 
                async_memcpy_callback, NULL) != ESP_OK) { 
                printf("esp_asyn_memcpy() error, psramLoopCount %d/%d\n", psramLoopCount, psram_sz / dma_sz);
                break;
            }

            pi = pi + dma_sz / sizeof(uint32_t);
            psramLoopCount++; 
            if (pi >= (psram_sz - dma_sz * 2) / sizeof(uint32_t)) {
                pi = 0;
                break;
            }
            if ((psramLoopCount & 127) == 1) {
                int b = (psramLoopCount >> 4) & 127;
                //rgbLedWriteBitBang(ledPin, b, b, 0);
                //neopixelWrite(ledPin, b, b, 0);
            }
            if (*drLoopCount - psramLoopCount > maxBufsUsed) maxBufsUsed = *drLoopCount - psramLoopCount;
        }
#endif 
        if (XTHAL_GET_CCOUNT() - startTsc > 240 * 1000000) { 
            startTsc = XTHAL_GET_CCOUNT();
            elapsedSec++;
            
#if 0 
            if (opt.core0Led) { 
                if (elapsedSec & 1) {
                    int cycles = (volatile int)ramReads;
                    if (cycles - lastCycleCount > 1700000) {
                        NEWneopixelWrite(ledPin, 0, 22, 0);
                    } else { 
                        NEWneopixelWrite(ledPin, 0, 0, 0);
                    }
                    lastCycleCount = cycles;
                } else { 
                    NEWneopixelWrite(ledPin, 22, 0, 0);
                }
            }
#endif
#ifndef FAKE_CLOCK
            if (elapsedSec == 10) { 
                addSimKeypress("   \233E.\"J\233                         \233RUN\233\233DOS\233");
                simulatedKeysAvailable = 1;
                //for(int i = 0; i < numProfilers; i++) profilers[i].clear();
            }
#endif
            if (elapsedSec == 1) { 
               for(int i = 0; i < numProfilers; i++) profilers[i].clear();
            }
            if(elapsedSec > opt.histRunSec && opt.histRunSec > 0) break;
            if(atariRam[754] == 23) break;

#if 0 
            // map in cartridge 
            if(atariRam[1666] == 100 && atariRam[1667] == 93) {
                break;
                for(int b = 0; b < 16 * 1024 / bankSize; b++)
                banks[(0x8000 >> bankShift) + b] = &cartROM[b * bankSize]; 
            }
#endif
#ifdef HAVE_RESET_PIN
            //if(cumulativeResets > 2) break;
            if(currentResetValue == 0 && elapsedSec > 15) break;
#endif
        }
    }
}


void threadFunc(void *) { 
    printf("CORE0: threadFunc() start\n");

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, 0);

    if(1) { 
        int bundleB_gpios[] = {ledPin};
        dedic_gpio_bundle_config_t bundleB_config = {
            .gpio_array = bundleB_gpios,
            .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
            .flags = {
                .out_en = 1
            },
        };
        ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleOut));
        for(int i = 0; i < sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]); i++) { 
            //gpio_set_drive_capability((gpio_num_t)bundleB_gpios[i], GPIO_DRIVE_CAP_MAX);
        }
    }

    NEWneopixelWrite(ledPin,25,25,25);
    delay(100);
    NEWneopixelWrite(ledPin,0,0,0);

    volatile int *drLoopCount = &dramLoopCount;
    async_memcpy_config_t config {
        .backlog = dma_bufs - 2,
        .sram_trans_align = 0,
        .psram_trans_align = 0,
        .flags = 0
    };

    if (esp_async_memcpy_install(&config, &handle) != ESP_OK) {
        printf("Failed to install async memcpy driver.\n");
        return;
    }

    //neopixelWrite(ledPin, 0, 8, 0);

    if (0) { 

    }
    printf("GIT: " GIT_VERSION " \n");


    XT_INTEXC_HOOK oldnmi = _xt_intexc_hooks[XCHAL_NMILEVEL];
    uint32_t oldint;
    if (opt.maskCore0Int) { 
        disableCore0WDT();
        portDISABLE_INTERRUPTS();
        _xt_intexc_hooks[XCHAL_NMILEVEL] = my_nmi; 
        //__asm__ __volatile__("rsil %0, 1" : "=r"(oldint) : );
    }
    int lastCycleCount = 0;
    uint32_t startTsc = XTHAL_GET_CCOUNT();
    int fakeRamErrCount = opt.forceMemTest ? 2 : 0;
    uint8_t lastRamValue = 0;

    core0Loop();

    stop = true;
#ifdef FAKE_CLOCK
    REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
#endif
    int maxLoopE = (volatile int)maxLoopElapsed, minLoopE = (volatile int)minLoopElapsed;
    startTsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - startTsc < 2 * 24 * 1000000) {}

    if (opt.maskCore0Int) { 
        portENABLE_INTERRUPTS();
        _xt_intexc_hooks[XCHAL_NMILEVEL] = oldnmi;
        enableCore0WDT();
        //__asm__("wsr %0,PS" : : "r"(oldint));
    }

    printf("STOPPED. max dma bufs in use %d\n", maxBufsUsed);

    uint32_t startUsec = micros();
    while(cbCount < psramLoopCount && micros() - startUsec < 1000000) {
        delay(50);
    }
    uint64_t totalEvents = 0;
    for(int i = 0; i < profilers[0].maxBucket; i++)
        totalEvents += profilers[0].buckets[i];
    printf("Total samples %lld implies %.2f sec sampling. Total reads %d\n",
        totalEvents, 1.0 * totalEvents / 1.8 / 1000000, ramReads);

    NEWneopixelWrite(ledPin, 8, 0, 0);
    printf("\n\n\n%.2f lastAddr %04x cb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x %d minLoop %d maxLoop %d jit %d late %d\n", 
        millis() / 1000.0, lastAddr, cbCount, lateCount, lateIndex, lateMin, lateMax, lateTsc, minLoopE, 
        maxLoopE, maxLoopE - minLoopE, loopElapsedLate);

    if (opt.logicAnalyzer) {
        uint32_t last = 0;
        for(uint8_t *p = (uint8_t *)psram; p < (uint8_t *)(psram + psram_sz / sizeof(uint32_t)); p++) {
            printf("LA %08x %02x   ", (int)(p - (uint8_t *)dram), *p);
            for(int i = 7; i >= 0; i--) 
                printf("B%d=%d ", i, ((((*p) & (1 << i)) != 0) ? 1 : 0));
            printf("    ");
            for(int i = 7; i >= 0; i--) 
                printf("%c ", ((((*p) & (1 << i)) != 0) ? 'X' : '.'));
            printf("    ");
            for(int i = 7; i >= 0; i--) 
                printf("%c ", ((((*p) & (1 << i)) != (last & (1 <<i))) ? 'E' : '.'));
          
          printf("\n");
          last = *p;
        }
    
        //for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p += dma_sz / sizeof(uint32_t)) {
    }

    if (opt.histogram) {
        vector<string> v; 
        int first = profilers[0].maxBucket, last = 0;
        for (int c = 0; c < numProfilers; c++) { 
            for(int i = 1; i < profilers[c].maxBucket; i++) { 
                if (profilers[c].buckets[i] > 0 && i > last) last = i;
            }
            for(int i = profilers[c].maxBucket - 1; i > 0 ;i--) { 
                if (profilers[c].buckets[i] > 0 && i < first) first = i;
            }
        }

        for(int i = first; i <= last; i++) {
            string s = sfmt("% 4d ", i);
            for(int c = 0; c < numProfilers; c++) {
                s += sfmt("% 12d ", profilers[c].buckets[i]);
            }
            s += " HIST";
            v.push_back(s);
        }

        for (int c = 0; c < numProfilers; c++) {
            first = last = 0; 
            for(int i = 1; i < profilers[c].maxBucket; i++) { 
                if (profilers[c].buckets[i] > 0) last = i;
            }
            for(int i = profilers[c].maxBucket - 1; i > 0 ;i--) { 
                if (profilers[c].buckets[i] > 0) first = i;
            }
            yield();
            v.push_back(sfmt("channel %d: range %3d -%3d, jitter %3d    HIST", c, first, last, last - first));
        }
        uint64_t totalEvents = 0;
        for(int i = 0; i < profilers[0].maxBucket; i++)
            totalEvents += profilers[0].buckets[i];
        v.push_back(sfmt("Total samples %lld implies %.2f sec sampling\n", totalEvents, 1.0 * totalEvents / 1.8 / 1000000));

        for(auto s : v) 
            printf("%s\n", s.c_str());
#if 0 
        printf("Writing to flash\n");
        yield(); 
        //LittleFS.remove("/histogram");
        yield();
        printf("LittleFS.remove() finished\n"); 
        SPIFFSVariable<vector<string>> h("/histogram", {});
        printf("SPIFFS var finished\n"); 
        yield();
        h = v;
        printf("Done writing to flash\n"); 
        //neopixelWrite(ledPin, 0, 0, 8);
#endif 
    }
    
    printf("DUMP %.2f\n", millis() / 1000.0);
    
    if (opt.tcpSendPsram) { 
        printf("TCP SEND %.2f\n", millis() / 1000.0);
        //j.begin();
        //wdtReset();
        yield();
        //disableCore0WDT();
        //disableLoopWDT();
        while(!sendPsramTcp((char *)psram, psram_sz)) delay(1000);
    }
    if (opt.dumpPsram) { 
        for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p++) {
            //printf("P %08X\n",*p);
            //if ((*p & copyResetMask) && !(*p &casInh_Mask))
            //if ((*p & copyResetMask) != 0)
            //s += sfmt("%08x\n", *p);
            if (1) { 
                //if ((*p & copyResetMask) != 0)
                printf("P%08x %08x %04x %02x MPD%d C%d RW%d\n", 
                    (int)(p - psram), *p, (*p & addrMask) >> addrShift,
                    (*p & copyDataMask) >> copyDataShift, 
                    (*p& copyMpdMask) != 0, 
                    (*p & casInh_Mask) != 0,
                    (*p & readWriteMask) != 0);
                //printf("P %08X %08X\n", p - psram, *p);
            }
            //if (p > psram + 1000) break;
            //wdtReset();
            //if (((p - psram) % 0x1000) == 0) printf("%08x\n", p - psram);
        }
        yield();
    }
    printf("bank 0xd8 %p, pbi rom is %p, atari ram is %p\n", banks[0xd800 >> bankShift], pbiROM, &atariRam[0xd800]);
    if (banks[0xd800 >> bankShift] == &atariRam[0xd800]) {
        printf("bank 0xd8 set to atari ram\n");
    }
    if (banks[0xd800 >> bankShift] == pbiROM) {
        printf("bank 0xd8 set to PBI ROM\n");
    }
    printf("atariRam[754] = %d\n", atariRam[754]);
    printf("pbiROM[0x100] = %d\n", pbiROM[0x100]);
    printf("atariRam[0xd900] = %d\n", atariRam[0xd900]);
    printf("diskIoCount %d\n", diskReadCount);
    printf("GIT: " GIT_VERSION "\n");
    printf("Page 6: ");
    for(int i = 0x600; i < 0x620; i++) { 
        printf("%02x ", atariRam[i]);
    }
    printf("\n0xd1ff: %02x\n", atariRam[0xd1ff]);
    printf("0xd820: %02x\n", atariRam[0xd820]);
    
    printf("DONE %.2f\n", millis() / 1000.0);
    delay(100);
    
    //ESP.restart();
    structLogs.print();
    printf("CORE0 idle\n");
    while(1) { 
        //printf("CORE0 idle\n");
        delay(10); 
        yield();
        NEWneopixelWrite(ledPin, 0, 0, ((millis() / 100) % 2) * 10);
    }
}

void *app_cpu_stack_ptr = NULL;
static void IRAM_ATTR app_cpu_main();
static void IRAM_ATTR app_cpu_init()
{
    // Reset the reg window. This will shift the A* registers around,
    // so we must do this in a separate ASM block.
    // Otherwise the addresses for the stack pointer and main function will be invalid.
    asm volatile (                                \
        "movi a0, 0\n"                            \
        "wsr  a0, WindowStart\n"                \
        "movi a0, 0\n"                            \
        "wsr  a0, WindowBase\n"                    \
        );
    // init the stack pointer and jump to main function
    asm volatile (                    \
        "l32i a1, %0, 0\n"            \
        "callx4   %1\n"                \
        ::"r"(&app_cpu_stack_ptr),"r"(app_cpu_main));
    REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
}

void startCpu1() {  
    if (REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
        printf("APP CPU is already running!\n");
        return;
    }

    if (!app_cpu_stack_ptr) {
        app_cpu_stack_ptr = heap_caps_malloc(1024, MALLOC_CAP_DMA);
    }

    DPORT_REG_WRITE(SYSTEM_CORE_1_CONTROL_1_REG, 0);
    DPORT_REG_WRITE(SYSTEM_CORE_1_CONTROL_0_REG, 0);
    DPORT_REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
    DPORT_REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);

    ets_set_appcpu_boot_addr((uint32_t)&app_cpu_init);
    DPORT_REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
    uint32_t r0 = DPORT_REG_READ(SYSTEM_CORE_1_CONTROL_0_REG);
    uint32_t r1 = DPORT_REG_READ(SYSTEM_CORE_1_CONTROL_1_REG);
    
    printf("Start APP CPU1 r0=%08x r1=%08x\n", r0, r1);
}

void setup() {
    for(auto i : pins) pinMode(i, INPUT);
    delay(500);
    Serial.begin(115200);
    printf("setup()\n");

#ifdef LOCAL_LFS
    lfsp_init();
    printf("lfsp_init() complete\n");
    int err = lfs_mount(&lfs, &cfg);
    printf("lfs_mount() returned %d\n", err);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        printf("Formatting LFS\n");
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    } else {
        printf("LFS mounted\n");
    }

    printf("boot_count: %d\n", lfs_updateTestFile());

#endif


    if (0) { 
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, 0);
        int color = 0;
        while(1) {
            //printf("neopixel\n");
            NEWneopixelWrite(ledPin, 0,0,color++ % 63);
            delay(10);
            if (millis() > 2000) break;
        }
    }

#if 0
    if (opt.histogram) { 
        SPIFFSVariable<vector<string>> hist("/histogram", {});
        vector<string> v = hist;
        for(auto s : v) { 
            printf("PREVIOUS %s\n", s.c_str());
        }
    }
#endif
    if (1) { 
        uint32_t val = 0x10000000;
        uint32_t mask = 0x10000000;
        if (!(val & mask)) { 
            printf("WRONG\n");
        } else { 
            printf("RIGHT\n");
        }
        if ((val & mask)) { 
            printf("RIGHT\n");
        } else { 
            printf("WRONG\n");
        }
    }

    //const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    psram = (uint32_t *) heap_caps_aligned_alloc(64, psram_sz,  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    dram = (uint32_t *)heap_caps_aligned_alloc(64, dram_sz, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    printf("psram %8p dram %8p\n", psram, dram);

#if 0 // try putting disk image in PSRAM
    atariDisks[0].image = (DiskImage::DiskImageRawData *)heap_caps_aligned_alloc(4, sizeof(diskImg),  MALLOC_CAP_SPIRAM);
    memcpy(atariDisks[0].image, diskImg, sizeof(diskImg));
#endif
    uint64_t mask = 0;
    for(auto i : pins) mask |= ((uint64_t)0x1) << i;
    uint32_t maskl = (uint32_t)mask;
    uint32_t maskh = (uint32_t)((mask & 0xffffffff00000000LL) >> 32);
    printf("MASK %d %08x %08x %08x\n", (int)pins.size(), maskl, maskh, PACK(maskl, maskh));
    printf("%d banks, bankShift %d, bankMask %04x\n", (int)nrBanks, bankShift, bankMask);
    
    bzero(psram, psram_sz);
    bzero(dram, dram_sz);
    if (opt.testPins) {  
        for(auto i : pins) pinMode(i, INPUT_PULLDOWN);
        delay(100);
        printf("PD   %08x %08x %08x\n", *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
        delay(100);
        printf("PU   %08x %08x %08x\n", *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, OUTPUT);
        for(auto i : pins) digitalWrite(i, 0);
        delay(100);
        printf("OL   %08x %08x %08x\n", *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) digitalWrite(i, 1);
        delay(100);
        printf("OH   %08x %08x %08x\n", *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
    }
    for(auto i : pins) pinMode(i, INPUT);
    while(opt.watchPins) { 
            delay(100);
            printf("PU   %08x %08x %08x\n", *gpio0, *gpio1, PACK(*gpio0, *gpio1));
    }

    //vector<int> outputPins = {extSel_Pin, data0Pin};
    if (opt.logicAnalyzer) { 
        int bundleA_gpios[] = {clockPin, casInh_pin, extSel_Pin, mpdPin, addr0Pin + 0, addr0Pin + 1, data0Pin + 0, data0Pin + 1};
        dedic_gpio_bundle_config_t bundleA_config = {
            .gpio_array = bundleA_gpios,
            .array_size = sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]),
            .flags = {
                .in_en = 1,
                .out_en = 0
            },
        };
        ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleIn));
        if (0) { 
            int bundleB_gpios[] = {ledPin};
            dedic_gpio_bundle_config_t bundleB_config = {
                .gpio_array = bundleB_gpios,
                .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
                .flags = { .out_en = 1 },
            };
            ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleOut));
        }
    } else { 
        // pbi device - only monitor one pin so we don't have to mask bits after reading dedic_io
        int bundleA_gpios[] = {clockPin};
        dedic_gpio_bundle_config_t bundleA_config = {
            .gpio_array = bundleA_gpios,
            .array_size = sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]),
            .flags = {
                .in_en = 1,
                .out_en = 0
            },
        };
        ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleIn));
        if (0) { 
            int bundleB_gpios[] = {ledPin};
            dedic_gpio_bundle_config_t bundleB_config = {
                .gpio_array = bundleB_gpios,
                .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
                .flags = { .out_en = 1 },
            };
            ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleOut));
        }
    }

    if (opt.bitResponse) { // can't use direct GPIO_ENABLE or GPIO_OUT registers after setting up dedic_gpio_bundle 
        int bundleB_gpios[] = {data0Pin, data0Pin + 1, data0Pin + 2, data0Pin + 3, data0Pin + 4, data0Pin + 5, data0Pin + 6, data0Pin + 7};
        dedic_gpio_bundle_config_t bundleB_config = {
            .gpio_array = bundleB_gpios,
            .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
            .flags = { .out_en = 1 },
        };
        ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleOut));
        REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);                         //    enable DATA lines for output
    }
    if (opt.fakeClock) { // simulate clock signal 
        pinMode(clockPin, OUTPUT);
        digitalWrite(clockPin, 0);
        ledcAttachChannel(clockPin, testFreq, 1, 0);
        ledcWrite(clockPin, 1);

        pinMode(readWritePin, OUTPUT);
        digitalWrite(readWritePin, 0);
        ledcAttachChannel(readWritePin, testFreq / 4, 1, 2);
        ledcWrite(readWritePin, 1);

        pinMode(casInh_pin, OUTPUT);
        digitalWrite(casInh_pin, 1);
        //ledcAttachChannel(casInh_pin, testFreq / 2, 1, 4);
        //ledcWrite(casInh_pin, 1);

#if 1
        // write 0xd1ff to address pins to simulate worst-case slowest address decode
        for(int bit = 0; bit < 16; bit ++)  
            pinMode(addr0Pin + bit, ((0xd1ff >> bit) & 1) == 1 ? INPUT_PULLUP : INPUT_PULLDOWN);
#endif 

        //gpio_set_drive_capability((gpio_num_t)clockPin, GPIO_DRIVE_CAP_MAX);
        pinMode(mpdPin, INPUT_PULLDOWN);
        pinMode(refreshPin, INPUT_PULLUP);
        //pinMode(casInh_pin, INPUT_PULLUP);
        pinMode(extSel_Pin, INPUT_PULLUP);
    }
    //pinMode(ledPin, OUTPUT);
    //digitalWrite(ledPin, 1);

    if (opt.pbiDevice) { 
        pinMode(extSel_Pin, OUTPUT);
        digitalWrite(extSel_Pin, 1);
        pinMode(mpdPin, OUTPUT);
        digitalWrite(mpdPin, 1);
    }
    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }

    printf("freq %.4fMhz threshold %d halfcycle %d clockMask %08x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, clockMask);

    startCpu1();
    uint32_t startTsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - startTsc < 240 * 1000) {};
    //threadFunc(NULL);
    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
    while(1) { yield(); delay(1000); };

    //while(XTHAL_GET_CCOUNT() - startTsc < 120 * 1000000) {}
}

void IRAM_ATTR iloop_logicAnalyzer() {	
    uint32_t r;
    register uint8_t *out = (uint8_t *)dram;
    register uint8_t *dram_end = out + dma_sz;
    
    const uint16_t triggerAddress       = 0xd1ff;
    const uint16_t triggerMask          = 0xffff;
    int triggerCount                    = 1;    // 0 for no trigger
    int captureDepth                    = 0;    // 0 for fill memory;

#if 1 
    for(int i = 0; i < 1000; i++) { 
        //while((dedic_gpio_cpu_ll_read_in() & 0x1)) {}
        //while(!(dedic_gpio_cpu_ll_read_in() & 0x1)) {}
        while((*gpio0 & clockMask) != 0) {}; 
        while((*gpio0 & clockMask) == 0) {}; 
    }
#endif

    uint32_t lastTsc = XTHAL_GET_CCOUNT();

    while(triggerCount > 0) { 
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {} // wait falling edge 
        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {} // wait rising edge 
            __asm__("nop"); // 1 cycle
            __asm__("nop"); // 1 cycle
            __asm__("nop"); // 1 cycle
            __asm__("nop"); // 1 cycle
            __asm__("nop"); // 1 cycle
    
        uint32_t r0 = REG_READ(GPIO_IN_REG);
        if ((r0 & refreshMask) == 0)
           continue;

        uint16_t addr = (r0 & addrMask) >> addrShift; 

        if (triggerCount > 0) {
            if ((addr & triggerMask) != (triggerAddress & triggerMask)) continue;
            triggerCount--;
            if (triggerCount > 0) continue;
        }

    }

    while(!stop) {
        *out = dedic_gpio_cpu_ll_read_in();
        #if 1
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            *(out + 1) = dedic_gpio_cpu_ll_read_in();
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            *(out + 2) = dedic_gpio_cpu_ll_read_in();
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            out += 4;            
            *(out - 1 ) = dedic_gpio_cpu_ll_read_in();
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
            __asm__ __volatile__("nop"); // 1 cycle
        #else
            out += 1;
        #endif
        if (out == dram_end) { 
            dramLoopCount++;
            out = (uint8_t *)(dram + dma_sz * (dramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t));
            dram_end = out + dma_sz;
        }
    }
}
        

void IRAM_ATTR iloop_busMonitor() {	
    uint32_t *out = dram;
    uint32_t *dram_end = dram + dma_sz / sizeof(uint32_t);
    const uint16_t triggerAddress       = 0xd1ff;
    const uint16_t triggerMask          = 0xffff;
    //const uint16_t triggerMask          = 0x0;
    int triggerCount                    = 1;    // 0 for no trigger
    int captureDepth                    = 0;    // 0 for fill memory;
    while(!stop) {
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}; // wait falling edge 
        uint32_t tsc = XTHAL_GET_CCOUNT();
        uint32_t *nextOut = dram + dma_sz * ((dramLoopCount + 1) & (dma_bufs - 1)) / sizeof(uint32_t);
        uint32_t *nextEnd = nextOut + dma_sz / sizeof(uint32_t);

        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {} // wait rising edge 
        uint32_t r0 = REG_READ(GPIO_IN_REG);
        while(XTHAL_GET_CCOUNT() - tsc < 90) {}
        //__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
        uint32_t r1 = REG_READ(GPIO_IN1_REG); 

        if ((r0 & refreshMask) == 0)
            continue;

        uint16_t addr = (r0 & addrMask) >> addrShift; 

        if (triggerCount > 0) {
            if ((addr & triggerMask) != (triggerAddress & triggerMask)) continue;
            triggerCount--;
            if (triggerCount > 0) continue;
        }

#if 0 
        if ((r1 & mpdMask) != 0) {
            r0 |= copyMpdMask;
        } else {
            r0 &= ~copyMpdMask;
        }
#endif
        r0 &= ~copyDataMask;
        r0 |= (r1 & dataMask) >> dataShift << copyDataShift;
        *out++ = r0;

        if (out == dram_end) { 
            dramLoopCount++;
            out = nextOut;
            dram_end = nextEnd;
        }
        __asm__ __volatile__("nop"); // 1 cycle
    }
}

void IRAM_ATTR iloop_bitResponse() {
    if (0) { 	
        while(1) {           
            while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}                
            REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);                         
            REG_WRITE(GPIO_OUT1_REG, 0x0);

            while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                     
            REG_READ(GPIO_IN_REG);
            REG_WRITE(GPIO_OUT1_REG, 0xff);
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);                         // about 230-240ns on scope from clock edge
        }
    } else {
        REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);                        
        while(1) { 
            int r;
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
            r = dedic_gpio_cpu_ll_read_in();
            dedic_gpio_cpu_ll_write_all(r);                        
        }
    }
}

// TODO shadow writes to ROM areas into atariRam[] so we can later reference PORTB bank bits 
// TODO need to eventually manage the MPD output bit 

#if 0 
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_ll.h"
#include "rom/gpio.h"

#ifdef FAKE_CLOCK
#define PROFILE(a, b) profilers[a].add(b)
#else
#define PROFILE(a, b) do {} while(0)
#endif
#endif 

void IRAM_ATTR iloop_timings1() {
    static const int iterations = 1000000; 
    uint32_t startTsc = xthal_get_ccount();
    uint32_t r;
    uint32_t lastTsc = startTsc;
    for(int i = 0; i < iterations; i++) { // loop overhead 4 cycles 

        XTHAL_GET_CCOUNT();                      // 1 cycle
        //neopixelWrite(ledPin, 8, 0, 8);          // 20000 cycles
        //REG_WRITE(GPIO_ENABLE_W1TC, 0x1);
        __asm__ __volatile__("nop"); // 1 cycle
        REG_WRITE(GPIO_OUT_REG, 0x1);                   //    18 cycles 
        __asm__ __volatile__("nop"); // 1 cycle
        dedic_gpio_cpu_ll_read_in();
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle

        #if 0
        REG_WRITE(GPIO_OUT1_REG, 0x1);           //    15 cycles
       // *gpio0 = 0x1;                          //    15 cycles 
        REG_READ(GPIO_IN_REG);                   //    18 cycles
        REG_READ(GPIO_IN_REG);                   //    18 cycles
        REG_WRITE(GPIO_OUT1_W1TS_REG, 0x1);      //    15 cycles, 13 cycles can be overlapped w/ next reg read 
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        REG_WRITE(GPIO_OUT1_W1TS_REG, 0x1);      //    15 cycles
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle
        __asm__ __volatile__("nop"); // 1 cycle

        __asm__ __volatile__("nop"); // 1 cycle
        //REG_WRITE(GPIO_ENABLE1_W1TS_REG, 0x1);      //    10 cycles
        
        //if (XTHAL_GET_CCOUNT() - startTsc > 0xffff000) break;  // 4 cycles
        //__asm__ __volatile__("nop"); // 1 cycle
        //if (XTHAL_GET_CCOUNT() > 0xffffffe) break;  // 3 cycles
        //dedic_gpio_cpu_ll_read_in();      // 1 cycle
        //if ((dedic_gpio_cpu_ll_read_in() & 0x2)) break; // 2 cycles  
        //if ((dedic_gpio_cpu_ll_read_in() & 0x3) == 3) break; // 3 cycles
        //psram[i] =  dedic_gpio_cpu_ll_read_in();   // can run at about 620ns loop without blocking psram
        //drambytes[i & 0xff] = dedic_gpio_cpu_ll_read_in();           // 5-6 cycles, could prob do a 30Mhz loop 
        //drambytes[(i + 1) & 0xff] = 0;
        //r = *gpio0;
        while(!(r = *gpio0) & (1 << 17)) {}                                          // read addr, rw, clk, casInh                                                      //  75.0
        if ((r & casInh_Mask))                                                      // 12.5
            continue;
        if (!(r & readWriteMask)) {                                                  //  37.48 ns 
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);      //    enable DATA lines for output
            uint16_t addr = (r & addrMask) >> addrShift;                            //  12.5 ns 
            *gpio1 = (((uint32_t)atariRam[addr]) << dataShift) | extSel_Mask;       //  25.0 ns 
        }
        int elapsed;
        uint32_t tsc;
        do {
            tsc = XTHAL_GET_CCOUNT();                                      //  total loop verhead 18/27 ticks w/ nop && tracking
            elapsed = tsc - lastTsc;
            if (elapsed > maxElapsed1) {
                maxElapsed1 = elapsed; 
                maxElapsedIndex1 = i;
            }
        } while(elapsed < 1);
        lastTsc = tsc;
#endif
    }
    uint32_t endTsc = xthal_get_ccount();
    avgTicks1 = 1.0 * (endTsc - startTsc) / iterations;
    avgNs1 = avgTicks1 / 240 * 1000;
}

void IRAM_ATTR iloop_timings2() {
    int iterations = 1000000; 
    uint32_t r;
    uint32_t startTsc = xthal_get_ccount();
    uint32_t lastTsc = startTsc;
    for(int n = 0; n < iterations; n++) { 
        XTHAL_GET_CCOUNT(); 
    }
    uint32_t endTsc = xthal_get_ccount();
    avgTicks2 = 1.0 * (endTsc - startTsc) / iterations;
    avgNs2 = avgTicks2 / 240 * 1000;
}

void loop() {

    while(1) { yield(); delay(1); }
#if 0
     if (0)  { // TMP demonstrate use of usb d-/+ pins 19,20 as input, demonstrate neopixel LED
        int toggle = 0;
        pinMode(19, INPUT);
        pinMode(20, INPUT);
        //pinMode(ledPin, OUTPUT);
        while(1) {
            //(ledPin, toggle * 8, digitalRead(19) * 32, digitalRead(20) * 32);
            toggle = !toggle;
            yield();
            delay(10);
        }
    }
    //printf("CORE1: testing busywait\n");
    //busywait(.1);
    //printf("CORE1: disabling interrupts\n");
    //fflush(stdout);
    //Serial.flush();
    //busywait(1.0);
    //disableCore1WDT();
    //disableLoopWDT();
    portDISABLE_INTERRUPTS();
    uint32_t oldint;
    XT_INTEXC_HOOK oldnmi = _xt_intexc_hooks[XCHAL_NMILEVEL];
    _xt_intexc_hooks[XCHAL_NMILEVEL] = my_nmi;  // saves 5 cycles, could save more 
    //__asm__ __volatile__("rsil %1, 15" : "=r"(oldint) : : );

    maxElapsed1 = maxElapsed2 = maxElapsedIndex1 = -1;
    if (opt.bitResponse) {
        iloop_bitResponse();
    } else if (opt.timingTest) {
        while(1) { 
            iloop_timings1();
            iloop_timings2();
            portENABLE_INTERRUPTS();
            printf("avg loop1 %.2f ticks %.2f ns maxTicks %d at #%d   loop2 %.2f ticks %.2f ns maxTicks %d  diff %.2f\n", 
                avgTicks1, avgNs1, maxElapsed1, maxElapsedIndex1, avgTicks2, avgNs2, maxElapsed2, avgNs2 - avgNs1);
            delay(100);
            portDISABLE_INTERRUPTS();
        }
    } else if (opt.logicAnalyzer) {
        iloop_logicAnalyzer();
    } else if (opt.busAnalyzer) { 
        iloop_busMonitor();
    } else { 
        iloop_pbi();
    }
    //while(1) {}
    //enableLoopWDT();
    portENABLE_INTERRUPTS();
    _xt_intexc_hooks[XCHAL_NMILEVEL] = oldnmi;  
    delay(200);
    printf("CORE1: avg loop1 %.2f ticks %.2f ns maxTicks %d at #%d   loop2 %.2f ticks %.2f ns maxTicks %d  diff %.2f\n", 
        avgTicks1, avgNs1, maxElapsed1, maxElapsedIndex1, avgTicks2, avgNs2, maxElapsed2, avgNs2 - avgNs1); 
    printf("CORE1 idle\n");
    fflush(stdout);
    while(1) { 
        yield(); 
        delay(1000); 
    }
#endif
}

static void IRAM_ATTR app_cpu_main() {
    uint32_t oldint;
    XT_INTEXC_HOOK oldnmi = _xt_intexc_hooks[XCHAL_NMILEVEL];
    _xt_intexc_hooks[XCHAL_NMILEVEL] = my_nmi;  // saves 5 cycles, could save more 
    __asm__ __volatile__("rsil %0, 15" : "=r"(oldint) : : );
    iloop_pbi();
    while(1) {}
}


#ifdef CSIM
class SketchCsim : public Csim_Module {
    public:
    void setup() { HTTPClient::csim_onPOST("http://.*/log", 
        [](const char *url, const char *hdr, const char *data, string &result) {
 	return 200; }); }
    string dummy;
    void parseArg(char **&a, char **la) override { if (strcmp(*a, "--dummy") == 0) dummy = *(++a); }
    void loop() override {}
} sketchCsim;
#endif
 


// https://www.oocities.org/dr_seppel/pbi1_eng.htm
// https://www.oocities.org/dr_seppel/pbi2_eng.htm
// https://github.com/maarten-pennings/6502/blob/master/4ram/README.md

//PBI pins
// A0-15
// D0-7
// _ExtSel - override RAM access
// _MPD
// _CasInh - output 0 = atari is reading ram, 1 atari is reading a rom 


// D1FF NEWPORT
// D301 PORTB and MMU  bit 0 - OS ROMS enable, 1 basic ROM enable, 2 1200XL leds, 4-5 130xe bank switch, 5000-57FF RAM 
// D800-DFFF    Math ROM  
// D800 ROM cksum lo
// D801 ROM cksum hi
// D802 ROM version
// D803 ID num
// D804 Device Type
// D805 JMP (0x4C)
// D809 ISR vect LO
// D80A ISR vect HI
// D80B ID num 2 (0x91)
// D80C Device Name (ASCII)
// D80A-D818 device vectors
// D819 JMP ($4C)
// D820 Init Vector LO
// D821 Init vector hi

// OS sequentally sets each bit in NEWPORT, then 
//OS checks D808 for 0x4C and D80B for 0x91, then jumps to D819 
// Actual bus trace shows it seems to read D803 checking for == 80
//RAM MAP
// $8000-9FFF
// $A000-BFFF

// $C000-FFFF OS ROM
// $D000-D7FF 2K for mmapped chips, GTIA, POKEY, PIA, ANTIC
// $D000 GTIA  
// $D200 POKEY 
// $D300 PIA
// $D400 ANTIC

// need PBI lines casInh_, WRT, phi2, ADDR0-15, DATA0-7, EXTSEL

// NOTES:
// 8-pin i2c io expander: https://media.digikey.com/pdf/Data%20Sheets/NXP%20PDFs/PCF8574(A).pdf
// TODO: verify polarity of RW, MPD, casInh, etc 


std::string vsfmt(const char *format, va_list args) {
        va_list args2;
        va_copy(args2, args);
        char buf[128]; // don't understand why stack variable+copy is faster
        string rval;

        int n = vsnprintf(buf, sizeof(buf), format, args);
        if (n > sizeof(buf) - 1) {
                rval.resize(n + 2, ' ');
                vsnprintf((char *)rval.data(), rval.size(), format, args2);
                //printf("n %d size %d strlen %d\n", n, (int)rval.size(), (int)strlen(rval.c_str()));
                rval.resize(n);
        } else { 
                rval = buf;
        }
        va_end(args2);
        return rval;
}

std::string sfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
        string rval = vsfmt(format, args);
        va_end(args);
        return rval;
}


#if 0 
CONFIG_SOC_WDT_SUPPORTED=n
CONFIG_BOOTLOADER_WDT_ENABLE=n
CONFIG_ESP_INT_WDT=n
CONFIG_ESP_TASK_WDT_EN=n
CONFIG_ESP_TASK_WDT_INIT=n
CONFIG_ESP_TASK_WDT_PANIC=n
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=n
CONFIG_INT_WDT=n
CONFIG_INT_WDT_CHECK_CPU1=n
CONFIG_INT_WDT_CHECK_CPU0=n
CONFIG_TASK_WDT=n
CONFIG_ESP_TASK_WDT=n
CONFIG_TASK_WDT_CHECK_IDLE_TASK_CPU0=n
CONFIG_TASK_WDT_CHECK_IDLE_TASK_CPU1=n

CONFIG_SOC_XT_WDT_SUPPORTED=n
CONFIG_SOC_WDT_SUPPORTED=n
CONFIG_ESP_ROM_HAS_HAL_WDT=n
CONFIG_HAL_WDT_USE_ROM_IMPL=n

"/home/jim/src/arduino-esp32/tools/xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++"  -MMD -c "@/home/jim/src/arduino-esp32/tools/esp32-arduino-libs/esp32s3/flags/cpp_flags" -w -Os -Werror=return-type -DF_CPU=240000000L -DARDUINO=10605 -DARDUINO_ESP32S3_DEV -DARDUINO_ARCH_ESP32 -DARDUINO_BOARD=\"ESP32S3_DEV\" -DARDUINO_VARIANT=\"esp32s3\" -DARDUINO_PARTITION_default -DARDUINO_HOST_OS=\"Linux\" -DARDUINO_FQBN=\"generic\" -DESP32=ESP32 -DCORE_DEBUG_LEVEL=0    -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0  -DBOARD_HAS_PSRAM  -DGIT_VERSION=\""432271-dirty"\" -Ofast "@/home/jim/src/arduino-esp32/tools/esp32-arduino-libs/esp32s3/flags/defines" "-I/home/jim/src/espEmptyTemplate/" -iprefix "/home/jim/src/arduino-esp32/tools/esp32-arduino-libs/esp32s3/include/" "@/home/jim/src/arduino-esp32/tools/esp32-arduino-libs/esp32s3/flags/includes" "-I/home/jim/src/arduino-esp32/tools/esp32-arduino-libs/esp32s3/qio_opi/include"  -I/home/jim/src/arduino-esp32/cores/esp32 -I/home/jim/src/arduino-esp32/variants/esp32s3 -I/tmp/mkESP/espEmptyTemplate_esp32s3 -I/home/jim/src/espEmptyTemplate -I/home/jim/src/arduino-esp32/libraries/LittleFS/src -I/home/jim/src/arduino-esp32/libraries/ESPmDNS/src -I/home/jim/src/arduino-esp32/libraries/HTTPClient/src -I/home/jim/src/arduino-esp32/libraries/Wire/src -I/home/jim/src/arduino-esp32/libraries/NetworkClientSecure/src -I/home/jim/src/arduino-esp32/libraries/FS/src -I/home/jim/src/arduino-esp32/libraries/ArduinoOTA/src -I/home/jim/src/arduino-esp32/libraries/Update/src -I/home/jim/src/arduino-esp32/libraries/Network/src -I/home/jim/src/arduino-esp32/libraries/WiFi/src -I/home/jim/Arduino/libraries/DHT_sensor_library -I/home/jim/Arduino/libraries/PubSubClient/src -I/home/jim/Arduino/libraries/Adafruit_Unified_Sensor -I/home/jim/Arduino/libraries/Arduino_CRC32/src -I/home/jim/Arduino/libraries/OneWireNg/src -I/home/jim/Arduino/libraries/OneWireNg/src/platform -I/home/jim/Arduino/libraries/Adafruit_HX711 -I/home/jim/Arduino/libraries/ArduinoJson -I/home/jim/Arduino/libraries/ArduinoJson/src -I/home/jim/Arduino/libraries/ArduinoJson/extras/tests/Helpers -I/home/jim/Arduino/libraries/ArduinoJson/extras/tests/Helpers/api -I/home/jim/Arduino/libraries/ArduinoJson/extras/tests/Helpers/avr -I/home/jim/Arduino/libraries/esp32jimlib/src "@/tmp/mkESP/espEmptyTemplate_esp32s3/build_opt.h" "@/tmp/mkESP/espEmptyTemplate_esp32s3/file_opts"    /tmp/mkESP/espEmptyTemplate_esp32s3/espEmptyTemplate.ino.cpp -S /tmp/asm.S

#endif