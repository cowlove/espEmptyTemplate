#ifndef CSIM
#include <driver/gpio.h>
#include <xtensa/hal.h>
#include "freertos/xtensa_timer.h"
#include "esp_intr_alloc.h"
#include "rtc_wdt.h" .
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_task_wdt.h"
#include <esp_async_memcpy.h>
#include <hal/cache_hal.h>
#include <hal/cache_ll.h>

#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include <xtensa/core-macros.h>
#else 

// Bullshit fake stubs 
#include <cstdlib>
#define REG_WRITE(a,b) { *a = b; } while(0)
#define REG_READ(a) (*a)
struct dedic_gpio_bundle_config_t {
    int *gpio_array;
    int array_size;
    struct { bool in_en, out_en; } flags;
};
typedef void * dedic_gpio_bundle_handle_t;
static inline void * dedic_gpio_new_bundle(void *, void *) { return NULL; }
#define ESP_INTR_DISABLE(a) 0
static inline void portENABLE_INTERRUPTS() {}
static inline void portDISABLE_INTERRUPTS() {}
static inline void enableLoopWDT() {}
static inline void disableLoopWDT() {}
static inline void disableCore1WDT() {}
#define XTHAL_GET_CCOUNT() 0
static inline int xthal_get_ccount() { return 0; }
static inline void dedic_gpio_cpu_ll_write_all(int) {}
static inline int dedic_gpio_cpu_ll_read_in() { return 0; }
static inline void gpio_set_drive_capability(int, int) {}
#define GPIO_DRIVE_CAP_MAX 0 
#define IRAM_ATTR 
static inline void *heap_caps_aligned_alloc(int, int sz, int) { return malloc(sz); }
static inline int cache_hal_get_cache_line_size(int, int) { return 0; }
static inline void xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, void *, int) {}
#define ESP_ERROR_CHECK(a) a
#define MALLOC_CAP_32BIT 0
#define MALLOC_CAP_INTERNAL 0
#define MALLOC_CAP_DMA 0
#define CACHE_LL_LEVEL_INT_MEM 0 
#define CACHE_TYPE_DATA 0 
#define MALLOC_CAP_SPIRAM 0
#define MALLOC_CAP_DMA 0
#define register 
static int dummyReg;
#define GPIO_IN_REG (&dummyReg)
#define GPIO_IN1_REG (&dummyReg)
#define GPIO_ENABLE1_REG (&dummyReg)
#define GPIO_OUT1_REG (&dummyReg)
#define GPIO_OUT1_W1TS_REG (&dummyReg)
#define GPIO_OUT1_W1TC_REG (&dummyReg)
#define GPIO_ENABLE1_W1TS_REG (&dummyReg)
#define GPIO_ENABLE1_W1TC_REG (&dummyReg)

struct async_memcpy_config_t { 
    int backlog, sram_trans_align, psram_trans_align, flags;
};
typedef void *async_memcpy_handle_t;
typedef void *async_memcpy_context_t;
typedef void *async_memcpy_event_t;
static inline int esp_async_memcpy_install(void *, void *) { return 0; }
static inline int esp_async_memcpy(void *, void *, void *, int, bool (*)(void**, void**, void*), void *) { return 0; } 
static inline void neopixelWrite(int, int, int, int) {}
#endif

#include <vector>
using std::vector;

#include "jimlib.h"



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

static const struct {
   bool fakeClock     = 1;
   bool testPins      = 0;
   bool watchPins     = 0;      // loop forever printing pin values w/ INPUT_PULLUP
   bool tcpSendPsram  = 1;
   bool dumpPsram     = 1;
   bool dumpSram      = 0;   ;
   bool histogram     = 0;
   bool timingTest    = 0;
   bool logicAnalyzer = 0;
   bool bitResponse   = 0;
} opt;

// *** CHANGES NOT YET REFLECTED IN HARDWARE:  Move reset input from pin 48 to 47, ext_sel from pin 47 to 46 

//GPIO0 pins
static const int      casInh_pin = 0;
static const int      casInh_Mask = (0x1 << casInh_pin);               // pin 0 
static const int      clockPin = 1;
static const int      clockMask = (0x1 << clockPin);
static const int      addr0Pin = 2;
static const int      addrShift = addr0Pin;                   // bus address - pins 1-16
static const int      addrMask = 0xffff << addrShift;  // 
static const int      refreshPin = 21;
static const int      refreshMask = (1 << clockPin);
static const int      readWritePin = 18;
static const int      readWriteMask = (1 << readWritePin); 

//GPIO1 pins
static const int      resetPin = 47;
static const int      resetMask = 1 << (resetPin - 32); 
static const int      extSel_Pin = 46;
static const int      extSel_PortPin = extSel_Pin - 32 /* port1 pin*/;
static const int      extSel_Mask = (1 << extSel_PortPin);
static const int      data0Pin = 38;
static const int      data0PortPin = data0Pin - 32;
static const int      dataShift = data0PortPin;
static const int      dataMask = (0xff << dataShift);

static const uint32_t copyResetMask = 0x40000000;
static const uint32_t copyDataShift = 22;
static const uint32_t copyDataMask = 0xff << copyDataShift;

volatile uint8_t atariRam[64 * 1024] = {0xff};

// try pin 19,20 (USB d- d+ pins).  need to write MPD use pin 47.  move reset to 0,  maybe add halt pin 20  
// pins 19,20 available 
//
//                  +--ROM read
//                  | +---Clock
//                  | | +--- ADDR                               +-- RW
//                  | | |                                       |  +-- refresh in              +--ext sel out
//                  | | |                                       |  |   +---DATA                |  +-- RESET in 
//                  | | + + + + + + + + +  +  +  +  +  +  +  +  |  |   |  +  +  +  +  +  +  +  |  |  
vector<int> pins = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21, 38,39,40,41,42,43,44,45,46,47};
int ledPin = 48;

#define PACK(r0, r1) (((r1 & 0x0001ffc0) << 15) | ((r0 & 0x0007fffe) << 2)) 

volatile uint32_t *gpio0 = (volatile uint32_t *)GPIO_IN_REG;
volatile uint32_t *gpio1 = (volatile uint32_t *)GPIO_IN1_REG;
int psram_sz = 7.8 * 1024 * 1024;
uint32_t *psram;
static const int dma_sz = 4096 - 64;
static const int dma_bufs = 32; // must be power of 2
static const int dram_sz = dma_sz * dma_bufs;

uint32_t *dram;
volatile uint32_t buf;
static const int testFreq = 1.8 * 1000000;//1000000;
static const int lateThresholdTicks = 180 * 2 * 1000000 / testFreq;
static const uint32_t halfCycleTicks = 240 * 1000000 / testFreq / 2;

uint32_t dramElapsedTsc;
uint32_t lateTsc;
int dramLoopCount = 0;
volatile int resetLowCycles = 0;
int psramLoopCount = 0;
int lateCount = 0;
int lateMax = 0, lateMin = 9999, lateIndex = -1;
int cbCount = 0;
volatile bool stop = false;
volatile int clockCount = 0;


JStuff j;

static bool async_memcpy_callback(async_memcpy_context_t*, async_memcpy_event_t*, void*) {
    cbCount++;
    return false;
}


// socat TCP-LISTEN:9999 - > file.bin
bool sendPsramTcp(const char *buf, int len, bool resetWdt = true) { 
    neopixelWrite(ledPin, 0, 0, 8);
    wifiDisconnect();
    wifiConnect();
    WiFiClient wc;
    static const int txSize = 1024;
   
    neopixelWrite(ledPin, 8, 0, 8);
    //int r = wc.connect("192.168.68.131", 9999);
    int r = wc.connect("10.250.250.240", 9999);
    printf("connect() returned %d\n", r);
    uint32_t startMs = millis();
    int count;
    int sent = 0;
    while(sent < len) { 
        if (!wc.connected()) { 
            Serial.printf("lost connection");
            Serial.flush();
            return false;
        }
        int pktLen = min(txSize, len - sent);
        r = wc.write((uint8_t *)(buf + sent), pktLen);
        if (r != pktLen) {
            Serial.printf("write %d returned %d\n", count, r);
            Serial.flush();
            return false;
        }
        sent += r;
        if (count++ % 100 == 0) { 
            Serial.printf("."); 
            Serial.flush();
        }
        neopixelWrite(ledPin, 0, 0, (count & 127) + 8);
        if (resetWdt) wdtReset();
        yield();
    }
    Serial.printf("\nDone %.3f mB/sec\n", psram_sz / 1024.0 / 1024.0 / (millis() - startMs) * 1000.0);
    Serial.flush();
    return true;
}

volatile int tickCount = 0;
void threadFunc2(void *) {
    while(1) {  
        printf("tickCount %d\n", tickCount);
        tickCount = 0;
        delay(1000);
    }
}
void IRAM_ATTR threadFunc(void *) { 
    printf("threadFunc() start\n");
    int pi = 0;
    //j.begin(); 
    //while(!sendPsramTcp((char *)psram, psram_sz), true) delay(1000);

    volatile int *drLoopCount = &dramLoopCount;
    async_memcpy_config_t config {
        .backlog = dma_bufs - 2,
        .sram_trans_align = 0,
        .psram_trans_align = 0,
        .flags = 0
    };
    async_memcpy_handle_t handle = NULL;
    if (esp_async_memcpy_install(&config, &handle) != ESP_OK) {
        printf("Failed to install async memcpy driver.\n");
        return;
    }

    uint32_t startUsec = micros();
    
    //portDISABLE_INTERRUPTS();
    //ESP_INTR_DISABLE(XT_TIMER_INTNUM);
    //disableCore0WDT();
    neopixelWrite(ledPin, 0, 8, 0); // green == ready 
    int maxBufsUsed = 0;
    while(1) { 
        if (psramLoopCount != *drLoopCount) {
            void *dma_buf = dram + (dma_sz * (psramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t));
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
            if (*drLoopCount - psramLoopCount >= dma_bufs) { 
                printf("esp_aync_memcpy buffer overrun psramLoopCount %d dramLoopCount %d\n", psramLoopCount, dramLoopCount);
                break;
            }
            if ((psramLoopCount & 15) == 1) {
                int b = (psramLoopCount >> 4) & 127;
                neopixelWrite(ledPin, b, b, 0);
            }
            if (*drLoopCount - psramLoopCount > maxBufsUsed) maxBufsUsed = *drLoopCount - psramLoopCount;
        }
    }
    //portENABLE_INTERRUPTS();
    //ESP_INTR_ENABLE(XT_TIMER_INTNUM);
    // DONE
    uint32_t elapsed = micros() - startUsec;
    printf("STOPPED %.2f mB/sec, max dma bufs in use %d\n", 1.0 * psram_sz / elapsed, maxBufsUsed);
    neopixelWrite(ledPin, 8, 0, 0);
    stop = true;

    startUsec = micros();
    while(cbCount < psramLoopCount && micros() - startUsec < 1000000) {
        delay(50);
    }
    printf("\n\n\ncb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x dramLoop %d psramLoop %d clockCount %d\n", 
        cbCount, lateCount, lateIndex, lateMin, lateMax, lateTsc,  dramLoopCount, psramLoopCount, clockCount);
 
    if (opt.dumpSram) {
        uint32_t last = 0;
        printf("resetLowCycles: %d\n", resetLowCycles);
        for(uint8_t *p = (uint8_t *)dram; p < (uint8_t *)(dram + dram_sz / sizeof(uint32_t)); p++) {
            printf("DRAM %08x %08x   ", (int)(p - (uint8_t *)dram), *p);
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
        int first = -1, last;
        int buckets[256];
        bzero(buckets, sizeof(buckets));
        uint32_t lastBucket = -1;
        for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p++) {
        //for(uint32_t *p = dram; p < dram + dram_sz / sizeof(uint32_t); p++) {
            //printf("PSRAM %08x %08x\n", (int)((p - psram) * sizeof(uint32_t)), *p - last);
            if (lastBucket != -1) {
                int delta = *p - lastBucket;
                delta = min((int)(sizeof(buckets) / sizeof(buckets[0])), delta);
                buckets[delta]++;
            }
            lastBucket = *p;
        }
        for(int i = 0; i < sizeof(buckets) / sizeof(buckets[0]); i++) { 
            printf("%d %d BUCKETS\n", i, buckets[i]);
            if (buckets[i] > 0) { 
                if (first == -1) first = i;
                last = i;
            }
        }
        printf("range %d-%d, jitter %d\n", first, last, last - first);
    }
    
    string s;
    printf("DUMP %.2f\n", millis() / 1000.0);
    
    if (opt.tcpSendPsram) { 
        j.begin();
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
                printf("P%08x %08x %04x %02x RST%d C%d RW%d\n", 
                    (int)(p - psram), *p, (*p & addrMask) >> addrShift,
                    (*p & copyDataMask) >> copyDataShift, 
                    (*p& copyResetMask) != 0, 
                    (*p & casInh_Mask) != 0,
                    (*p & readWriteMask) != 0);
                //printf("P %08X %08X\n", p - psram, *p);
            }
            //if (p > psram + 1000) break;
            //wdtReset();
            //if (((p - psram) % 0x1000) == 0) printf("%08x\n", p - psram);
            if (opt.tcpSendPsram) wdtReset();
        }
    }
    Serial.printf("DONE %.2f\n", millis() / 1000.0);
    Serial.flush();
    delay(100);
    
    ESP.restart();
    while(1) { yield(); };
}

dedic_gpio_bundle_handle_t bundleIn, bundleOut;
void setup() {
    delay(1000);
    Serial.begin(1200);
    printf("setup()\n");
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


    const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    psram = (uint32_t *) heap_caps_aligned_alloc(64, psram_sz,  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    dram = (uint32_t *)heap_caps_aligned_alloc(64, dram_sz, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    printf("psram %8p dram %8p\n", psram, dram);
    uint64_t mask = 0;
    for(auto i : pins) mask |= ((uint64_t)0x1) << i;
    uint32_t maskl = (uint32_t)mask;
    uint32_t maskh = (uint32_t)((mask & 0xffffffff00000000LL) >> 32);
    printf("MASK %d %08x %08x %08x\n", (int)pins.size(), maskl, maskh, PACK(maskl, maskh));

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
    //pinMode(extSel_Pin, OUTPUT);
    //digitalWrite(extSel_Pin, 0);

    vector<int> outputPins = {extSel_Pin, data0Pin};
    if (1) { 
        int bundleA_gpios[] = {clockPin, casInh_pin, readWritePin, addr0Pin + 0, addr0Pin + 1, data0Pin + 0, data0Pin + 1, data0Pin+3};
        dedic_gpio_bundle_config_t bundleA_config = {
            .gpio_array = bundleA_gpios,
            .array_size = sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]),
            .flags = {
                .in_en = 1,
                .out_en = 0
            },
        };
        ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleIn));

        if (opt.bitResponse) { // can't use direct GPIO_ENABLE or GPIO_OUT registers after setting up dedic_gpio_bundle 
                int bundleB_gpios[] = {data0Pin, data0Pin + 1, data0Pin + 2, data0Pin + 3, data0Pin + 4, data0Pin + 5, data0Pin + 6, data0Pin + 7};
                dedic_gpio_bundle_config_t bundleB_config = {
                .gpio_array = bundleB_gpios,
                .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
                .flags = {
                    .out_en = 1
                },
            };
            ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleOut));
            REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);                         //    enable DATA lines for output
            for(int i = 0; i < sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]); i++) { 
                gpio_set_drive_capability((gpio_num_t)bundleB_gpios[i], GPIO_DRIVE_CAP_MAX);
            }


        }
    }
    if (opt.fakeClock) { // simulate clock signal 
        pinMode(clockPin, OUTPUT);
        digitalWrite(clockPin, 0);
        ledcAttachChannel(clockPin, testFreq, 1, 0);
        ledcWrite(clockPin, 1);
        //gpio_set_drive_capability((gpio_num_t)clockPin, GPIO_DRIVE_CAP_MAX);
        pinMode(resetPin, INPUT_PULLUP);
    }
    if (opt.fakeClock) { // simulate clock signal 
        pinMode(resetPin, INPUT_PULLUP);
    }
    pinMode(ledPin, OUTPUT);

    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }

    printf("freq %.4fMhz threshold %d halfcycle %d clockMask %08x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, clockMask);
    // esp_himem_get_free_size(), esp_himem_get_phys_size());
    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
    delay(500);
}

void IRAM_ATTR iloop_logicAnalyzer() {	
    uint32_t r;
    register uint8_t *out = (uint8_t *)dram;
    register uint8_t *dram_end = out + dma_sz;
    
    for(int i = 0; i < 1000; i++) { 
        //while((dedic_gpio_cpu_ll_read_in() & 0x1)) {}
        //while(!(dedic_gpio_cpu_ll_read_in() & 0x1)) {}
        while((*gpio0 & clockMask) != 0) {}; 
        while((*gpio0 & clockMask) == 0) {}; 
    }
    uint32_t tsc = XTHAL_GET_CCOUNT();
    while((*gpio1 & resetMask) == 0) {}; 
    resetLowCycles = XTHAL_GET_CCOUNT()- tsc;
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
    uint32_t startTsc = XTHAL_GET_CCOUNT();
    uint32_t r0, r1;
    uint32_t tsc, lastTsc = XTHAL_GET_CCOUNT();
    uint32_t *out = dram;
    uint32_t *dram_end = dram + dma_sz / sizeof(uint32_t);

    while(!stop) {
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {} // wait falling edge 
        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {} // wait rising edge 
        tsc = XTHAL_GET_CCOUNT();
        r0 = REG_READ(GPIO_IN_REG);
        r1 = REG_READ(GPIO_IN1_REG);
    
        //if ((r1 & resetMask) == 0) continue; 
        if ((r1 & resetMask) != 0) {
            r0 |= copyResetMask;
        } else {
            r0 &= ~copyResetMask;
        }
        r0 &= ~copyDataMask;
        r0 |= (r1 & dataMask) >> dataShift << copyDataShift;
        if (opt.histogram) { 
            *out++ = tsc;
        } else {
            *out++ = r0;
        }
        if (out == dram_end) { 
            dramLoopCount++;
            out = dram + dma_sz * (dramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t);
            dram_end = out + dma_sz / sizeof(uint32_t);
        }
    }
    dramElapsedTsc = startTsc - XTHAL_GET_CCOUNT();
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
    
void IRAM_ATTR iloop_pbi() {	
    uint32_t r0, r1;
    int ram = 0;
    uint32_t tscRise, tscFall, tscDataReady;

    while(!stop) {
        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                      // wait rising clock edge
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}                      // wait falling clock edge
        tscFall = XTHAL_GET_CCOUNT();
        REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);                             // stop driving data lines, if they were previously driven                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        r0 = REG_READ(GPIO_IN_REG);                                             // read address, RW flag and casInh_  from bus
        uint16_t addr = (r0 & addrMask) >> addrShift;                            
        
        if ((r0 & (refreshMask | casInh_Mask)) == 0)                                // ignore refresh or ROM access 
            continue;

        if ((r0 & readWriteMask) != 0) {                                            // 1. READ        
            REG_WRITE(GPIO_OUT1_REG, (atariRam[addr] << addrShift) | extSel_Mask);  //    output data to data bus
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask | extSel_Mask);               //    enable DATA lines for output
        } else {                                                                    // 2. WRITE 
            atariRam[0] = (REG_READ(GPIO_IN1_REG) & dataMask) >> dataShift;         //    get write data from bus, write to local RAM 
        }        
    }
}

uint8_t drambytes[255];
volatile uint32_t dummy = 0;
volatile double avgNs1, avgNs2, avgTicks1, avgTicks2;
int maxElapsed1 = 0, maxElapsed2 = 0;
int maxElapsedIndex1 = 0;
void IRAM_ATTR iloop_timings1() {
    static const int iterations = 1000000; 
    uint32_t startTsc = xthal_get_ccount();
    uint32_t r;
    uint32_t lastTsc = startTsc;
    for(int i = 0; i < iterations; i++) { // loop overhead 4 cycles 

        XTHAL_GET_CCOUNT();                      // 1 cycle
        REG_READ(GPIO_IN_REG);                   //    18 cycles 
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
#if 0
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
    if (0)  { // TMP demonstrate use of usb d-/+ pins 19,20 as input, demonstrate neopixel LED
        int toggle = 0;
        pinMode(19, INPUT);
        pinMode(20, INPUT);
        pinMode(ledPin, OUTPUT);
        while(1) {
            Serial.printf("loop()\n");
            Serial.flush();
            delay(10);

            neopixelWrite(ledPin, toggle * 8, digitalRead(19) * 32, digitalRead(20) * 32);
            toggle = !toggle;
            yield();
            delay(10);
        }
    }

    if (1) { 
        disableCore1WDT();
        portDISABLE_INTERRUPTS();
        ESP_INTR_DISABLE(XT_TIMER_INTNUM);
        disableLoopWDT();
    }


    maxElapsed1 = maxElapsed2 = maxElapsedIndex1 = -1;
    //iloop_pbi();
    //iloop_timings1();
    //iloop_timings2();
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
    } else { 
        iloop_busMonitor();
    }
    enableLoopWDT();
    //ESP_INTR_ENABLE(XT_TIMER_INTNUM);
    portENABLE_INTERRUPTS();
    //enableCore1WDT();
    printf("avg loop1 %.2f ticks %.2f ns maxTicks %d at #%d   loop2 %.2f ticks %.2f ns maxTicks %d  diff %.2f\n", 
        avgTicks1, avgNs1, maxElapsed1, maxElapsedIndex1, avgTicks2, avgNs2, maxElapsed2, avgNs2 - avgNs1); 
    delay(100);
    while(1) { yield();  wdtReset(); }
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

//OS checks D808 for 0x4C and D80B for 0x91, then jumps to D819

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