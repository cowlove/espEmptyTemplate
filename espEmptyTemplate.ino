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



#else 
#include "esp32csim.h"
#endif
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

unsigned IRAM_ATTR my_nmi(unsigned x) { return 0; }
static const struct {
   bool fakeClock     = 0;
   bool testPins      = 0;
   bool watchPins     = 0;      // loop forever printing pin values w/ INPUT_PULLUP
   bool tcpSendPsram  = 0;
   bool dumpPsram     = 0;
   bool dumpSram      = 0;   ;
   bool histogram     = 0;
   bool timingTest    = 0;
   bool logicAnalyzer = 0;
   bool busAnalyzer   = 0;
   bool bitResponse   = 0;
   bool maskCore0Int  = 0;
} opt;

// *** CHANGES NOT YET REFLECTED IN HARDWARE:  Move reset input from pin 48 to 47, ext_sel from pin 47 to 46, 
// wire refresh to pin 21 

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
static const int      resetPin = 46;
static const int      resetMask = 1 << (resetPin - 32); 
static const int      extSel_Pin = 47;
static const int      extSel_PortPin = extSel_Pin - 32 /* port1 pin*/;
static const int      extSel_Mask = (1 << extSel_PortPin);
static const int      data0Pin = 38;
static const int      data0PortPin = data0Pin - 32;
static const int      dataShift = data0PortPin;
static const int      dataMask = (0xff << dataShift);

static const uint32_t copyResetMask = 0x40000000;
static const uint32_t copyDataShift = 22;
static const uint32_t copyDataMask = 0xff << copyDataShift;

uint8_t atariRam[64 * 1024] = {0x0};

// TODO: try pin 19,20 (USB d- d+ pins). Move reset to 0 so ESP32 boot doesnt get messed up by low signal   
// TODO: maybe eventually need to drive PBI interrupt pin 
// TODO: so eventaully looks like: pin 0 = reset, pin 19 = casInh input, pin 20 = interrupt, pin 47 = MPD
// TODO: although USB pins moving during ESP32 boot might cause conflict 
// TODO: extend this generally, need to review which ESP32 pins are driven during boot or have strapping resistors   
//
//                               +--casInh_ / ROM read
//                               | +---Clock
//                               | | +--- ADDR                               +-- RW
//                               | | |                                       |  +-- refresh in              +--RESET in
//                               | | |                                       |  |   +---DATA                |  +-- ext sel out 
//                               | | + + + + + + + + +  +  +  +  +  +  +  +  |  |   |  +  +  +  +  +  +  +  |  |  
static const vector<int> pins = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21, 38,39,40,41,42,43,44,45,46,47};
static const int ledPin = 48;

#define PACK(r0, r1) (((r1 & 0x0001ffc0) << 15) | ((r0 & 0x0007fffe) << 2)) 

volatile uint32_t *gpio0 = (volatile uint32_t *)GPIO_IN_REG;
volatile uint32_t *gpio1 = (volatile uint32_t *)GPIO_IN1_REG;

#undef REG_READ
//#undef REG_WRITE
#define REG_READ(r) (*((volatile uint32_t *)r))
#define REG_WRITE(r,v) do { *((volatile uint32_t *)r) = (v); } while(0)

int psram_sz = 7.8 * 1024 * 1024;
uint32_t *psram;
static const int dma_sz = 4096 - 64;
static const int dma_bufs = 8; // must be power of 2
static const int dram_sz = dma_sz * dma_bufs;

uint32_t *dram;
static const int testFreq = 1.8 * 1000000;//1000000;
static const int lateThresholdTicks = 180 * 2 * 1000000 / testFreq;
static const uint32_t halfCycleTicks = 240 * 1000000 / testFreq / 2;
static const float histRunSec = -1;
uint32_t dramElapsedTsc;
uint32_t lateTsc;
int dramLoopCount = 0;
int resetLowCycles = 0;
int psramLoopCount = 0;
int lateCount = 0;
int lateMax = 0, lateMin = 9999, lateIndex = -1;
int cbCount = 0;
bool stop = false;
dedic_gpio_bundle_handle_t bundleIn, bundleOut;
uint32_t lastAddr = -1;

JStuff j;

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

void busyWaitCCount(int cycles) { 
    uint32_t tsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - tsc < cycles) {};
}

void rgbLedWriteBitBang_NO(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
    //busyWaitCCount(100);
    //return;

    int color[3] = {green_val, red_val, blue_val};
    // 100 == all white
    // 80 == green too bright 
    // 20 == green but too bright, 
    // 10 == all white
    int tune = 30;
    int longCycles = (int)(0.8 * 240) - tune;
    int shortCycles = (int)(0.4 * 240) - tune;

    uint32_t bitMask = 1 << (47 - 32); 
    int i = 0;
    for (int col = 0; col < 3; col++) {
        for (int bit = 0; bit < 8; bit++) {
            if ((color[col] & (1 << (7 - bit)))) {
                // HIGH bit
                REG_WRITE(GPIO_OUT1_REG, REG_READ(GPIO_OUT1_REG) | bitMask);
                busyWaitCCount(longCycles);
                REG_WRITE(GPIO_OUT1_REG, REG_READ(GPIO_OUT1_REG) & ~bitMask);
                busyWaitCCount(shortCycles);
            } else {
                // LOW bit
                REG_WRITE(GPIO_OUT1_REG, REG_READ(GPIO_OUT1_REG) | bitMask);
                busyWaitCCount(shortCycles);
                REG_WRITE(GPIO_OUT1_REG, REG_READ(GPIO_OUT1_REG) & ~bitMask);
                busyWaitCCount(longCycles);
            }
            i++;
        }
    }
}

// socat TCP-LISTEN:9999 - > file.bin
bool sendPsramTcp(const char *buf, int len, bool resetWdt = false) { 
    neopixelWrite(ledPin, 0, 0, 8);
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
    neopixelWrite(ledPin, 0, 8, 0);
    return true;
}


// TODO make a threadFunc that doesn't save to psram, but just compiles 
// histogram shit from sram and discards it 

void IRAM_ATTR threadFunc(void *) { 
    int elapsedSec = 0;
    
    printf("CORE0: threadFunc() start\n");

    int pi = 0;

    volatile int *drLoopCount = &dramLoopCount;
    async_memcpy_config_t config {
        .backlog = dma_bufs - 2,
        .sram_trans_align = 0,
        .psram_trans_align = 0,
        .flags = 0
    };
    async_memcpy_handle_t handle = NULL;
    static int buckets[512];
    bzero(buckets, sizeof(buckets));
    uint32_t lastTsc = -1;

    if (esp_async_memcpy_install(&config, &handle) != ESP_OK) {
        printf("Failed to install async memcpy driver.\n");
        return;
    }

    uint32_t startTsc = XTHAL_GET_CCOUNT();
    int maxBufsUsed = 0;

#define DISABLE_CORE0_INT   // reduces jitter from ~44(49-94) to ~17(74-91) ticks, avoids occassional missed 2Mhz edges 
#ifdef DISABLE_CORE0_INT
    disableCore0WDT();
    //disableLoopWDT();
    portDISABLE_INTERRUPTS();
    //uint32_t oldint;
    //__asm__ __volatile__("rsil %0, 15" : "=r"(oldint) : : );
    XT_INTEXC_HOOK oldnmi = _xt_intexc_hooks[XCHAL_NMILEVEL];
    _xt_intexc_hooks[XCHAL_NMILEVEL] = my_nmi; 
#endif

    while(1) { 
        if (psramLoopCount != *drLoopCount) {
            if (psramLoopCount == 0) { 
                Serial.printf("psramLoopCount\n");
                Serial.flush();
            }
            void *dma_buf = dram + (dma_sz * (psramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t));

            if (opt.histogram) { 
                cbCount++; // fake dma callback
                for(uint32_t *p = (uint32_t *)dma_buf; p < ((uint32_t *)dma_buf) + dma_sz / sizeof(uint32_t); p++) {
                    if (lastTsc != -1) {
                        //int delta = *p - lastTsc;
                        int delta = *p;
                        delta = min((int)(sizeof(buckets) / sizeof(buckets[0])), delta);
                        buckets[delta]++;
                    }
                    lastTsc = *p;
                }
            } else { 
                if (*drLoopCount - psramLoopCount >= dma_bufs - 2) { 
                    printf("esp_aync_memcpy buffer overrun psramLoopCount %d dramLoopCount %d\n", psramLoopCount, dramLoopCount);
                    break;
                }
                if (esp_async_memcpy(handle, (void *)(psram + pi), (void *)dma_buf, dma_sz, 
                    async_memcpy_callback, NULL) != ESP_OK) { 
                    printf("esp_asyn_memcpy() error, psramLoopCount %d/%d\n", psramLoopCount, psram_sz / dma_sz);
                    break;
                }
            }

            pi = pi + dma_sz / sizeof(uint32_t);
            psramLoopCount++; 
            if (pi >= (psram_sz - dma_sz * 2) / sizeof(uint32_t)) {
                pi = 0;
                //wdtReset();
                if (!opt.histogram) 
                    break;
            }
            
#ifndef DISABLE_CORE0_INT
            if ((psramLoopCount & 127) == 1) {
                int b = (psramLoopCount >> 4) & 127;
                //rgbLedWriteBitBang(ledPin, b, b, 0);
                //neopixelWrite(ledPin, b, b, 0);
            }
#endif
            if (*drLoopCount - psramLoopCount > maxBufsUsed) maxBufsUsed = *drLoopCount - psramLoopCount;
        } else {
            //delay(1);
        }
        if (XTHAL_GET_CCOUNT() - startTsc > 240 * 1000000) { 
            startTsc = XTHAL_GET_CCOUNT();
            if(++elapsedSec > histRunSec && histRunSec > 0) break;
        }
    }

    stop = true;
    int maxLoopE = (volatile int)maxLoopElapsed, minLoopE = (volatile int)minLoopElapsed;
    startTsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - startTsc < 24 * 1000000) {}

#ifdef DISABLE_CORE0_INT
    //_xt_intexc_hooks[XCHAL_NMILEVEL] = oldnmi; 
    portENABLE_INTERRUPTS();
    enableCore0WDT();
    //enableLoopWDT();
#endif
    //neopixelWrite(ledPin, 8, 0, 0);
    //while(1) { delay(10); }
    Serial.printf("STOPPED. max dma bufs in use %d\n", maxBufsUsed);
    Serial.flush();
    uint32_t startUsec = micros();
    while(cbCount < psramLoopCount && micros() - startUsec < 1000000) {
        delay(50);
    }
    printf("\n\n\n%.2f lastAddr %04x cb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x dramLoop %d psramLoop %d minLoop %d maxLoop %d jit %d late %d\n", 
        millis() / 1000.0, lastAddr, cbCount, lateCount, lateIndex, lateMin, lateMax, lateTsc,  dramLoopCount, psramLoopCount, minLoopE, 
        maxLoopE, maxLoopE - minLoopE, loopElapsedLate);
 
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
        int first = 0, last = 0;
        for(int i = 1; i < sizeof(buckets) / sizeof(buckets[0]); i++) { 
            if (buckets[i] > 0) last = i;
        }
        for(int i = sizeof(buckets) / sizeof(buckets[0]) - 1; i > 0 ;i--) { 
            if (buckets[i] > 0) first = i;
        }
        for(int i = first; i <= last; i++) { 
            printf("%d %d BUCKETS\n", i, buckets[i]);
        }
        printf("range %d-%d, jitter %d\n", first, last, last - first);
    }
    
    string s;
    printf("DUMP %.2f\n", millis() / 1000.0);
    
    if (opt.tcpSendPsram) { 
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
        }
        yield();
    }
    Serial.printf("DONE %.2f\n", millis() / 1000.0);
    Serial.flush();
    delay(100);
    
    //ESP.restart();
    while(1) { yield(); };
}

void setup() {
    for(auto i : pins) pinMode(i, INPUT);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, 1);
    neopixelWrite(ledPin, 0, 10, 0);
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

    //const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
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

#if 0 // why doesn't PWM readWritePin work to alternate between R & W
        pinMode(readWritePin, OUTPUT);
        digitalWrite(readWritePin, 0);
        ledcAttachChannel(readWritePin, 10000, 8, 2);
        ledcWrite(readWritePin, 128);
#else 
        pinMode(readWritePin, INPUT_PULLDOWN);
#endif
        //gpio_set_drive_capability((gpio_num_t)clockPin, GPIO_DRIVE_CAP_MAX);
        pinMode(resetPin, INPUT_PULLUP);
        pinMode(refreshPin, INPUT_PULLUP);
        pinMode(extSel_Pin, INPUT_PULLUP);

    }
    //pinMode(ledPin, OUTPUT);
    //digitalWrite(ledPin, 1);

    pinMode(extSel_Pin, OUTPUT);
    digitalWrite(extSel_Pin, 0);

    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }

    printf("freq %.4fMhz threshold %d halfcycle %d clockMask %08x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, clockMask);

    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
        
    uint32_t startTsc = XTHAL_GET_CCOUNT();
    while(XTHAL_GET_CCOUNT() - startTsc < 120 * 1000000) {}
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
    uint32_t tsc, lastTsc;
    uint32_t loopCount = 0;
    uint32_t *out = dram;
    uint32_t *dram_end = dram + dma_sz / sizeof(uint32_t);
    minLoopElapsed = 0xffff;
    maxLoopElapsed = 0;
    while(!stop) {
        //uint32_t *nextOut = dram + dma_sz * ((dramLoopCount + 1) & (dma_bufs - 1)) / sizeof(uint32_t);
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {} // wait falling edge 
        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {} // wait rising edge 
        tsc = XTHAL_GET_CCOUNT();
        uint32_t elapsed = tsc - lastTsc;
        lastTsc = tsc;

        r0 = REG_READ(GPIO_IN_REG);
        r1 = REG_READ(GPIO_IN1_REG);
 
        if ((r1 & resetMask) == 0 || (r0 & refreshMask) == 0)
            continue;

        uint16_t addr = (r0 & addrMask) >> addrShift; 

        if ((r1 & resetMask) != 0) {
            r0 |= copyResetMask;
        } else {
            r0 &= ~copyResetMask;
        }
        r0 &= ~copyDataMask;
        r0 |= (r1 & dataMask) >> dataShift << copyDataShift;
        if (opt.histogram) { 
            *out++ = (loopCount++ > 10) ?  elapsed : 0 ;
        } else {
            *out++ = r0;
        }
        if (out == dram_end) { 
            dramLoopCount++;
            out = dram + dma_sz * (dramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t);
            //out = nextOut;
            dram_end = out + dma_sz / sizeof(uint32_t);
        }
#if 0
        if (loopCount++ > 5) {
            uint32_t elapsed = tsc - lastTsc;
            if (elapsed > maxLoopElapsed) maxLoopElapsed = elapsed;
            if (elapsed < minLoopElapsed) minLoopElapsed = elapsed;
        }
#endif 
        //lastTsc = tsc;
        __asm__ __volatile__("nop"); // 1 cycle
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
    uint32_t tscRise, tscFall, tscDataReady, tsc;

    uint32_t *out = dram;
    uint32_t *nextOut = dram;
    uint32_t *dram_end = dram + dma_sz / sizeof(uint32_t);
    int loopCount = 0;

    static const uint32_t bankMask = 0xff00, bankVal = 0x0000;
    static uint8_t banks[512] = {0};
    uint8_t *bank = &banks[0];

    uint32_t tscStart = XTHAL_GET_CCOUNT();

    minLoopElapsed = 0xffff;
    maxLoopElapsed = 0;
    int elapsed = 0;
    bool enabled = false;

    const bool fakeData = true; //*((uint32_t *)&atariRam[1668]) == 100;
    
    while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                      // wait rising clock edge
    while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}                      // wait falling clock edge
    REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask | extSel_Mask);                             // stop driving data lines, if they were previously driven                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    
    while(1) {
        if (opt.histogram) { 
            if (loopCount > 5) {
                elapsed = tsc - tscDataReady;  
                *out++ = elapsed;
                if (out == dram_end) { 
                    dramLoopCount++;
                    out = dram + dma_sz * (dramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t);
                    //out = nextOut;
                    dram_end = out + dma_sz / sizeof(uint32_t);
                    //__asm__ __volatile__("nop"); // 1 cycle
                }
            } else {
                loopCount++;
            }
        }
        //if (stop) break;
    
        r0 = REG_READ(GPIO_IN_REG);                                             // read address, RW flag and casInh_  from bus
        const uint16_t addr = (r0 & addrMask) >> addrShift;                            
        const uint8_t page = addr >> 10;
        //if (enabled == false && XTHAL_GET_CCOUNT() - tscStart > 240 * 1000000 * 10)
        //    enabled = true;
        if ((r0 & (refreshMask | casInh_Mask)) == (refreshMask | casInh_Mask)
            // && (page > 1) 
            // && enabled 
        ) {
            if ((r0 & readWriteMask) != 0) {                                            // 1. READ        
                uint8_t data;
#define NO_BANK // Bank not working yet 
#ifdef NO_BANK
                data = atariRam[addr];
#else
                if ((addr & bankMask) == bankVal) { 
                    data = bank[addr & ~bankMask];
                } else {
                    data = atariRam[addr];
                }
#endif
                __asm__ ("nop"); // needed for timing, causes memory errors.  
                __asm__ ("nop"); 
                REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask | extSel_Mask);               //    enable DATA lines for output
                REG_WRITE(GPIO_OUT1_REG, (data << dataShift));            //    output data to data bus
                while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                      // wait rising clock edge
                while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}                      // wait falling clock edge
                __asm__ ("nop"); 
                __asm__ ("nop"); 
                REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask | extSel_Mask);                             // stop driving data lines, if they were previously driven                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 

            } else {                                                                    // 2. WRITE 
#ifdef NO_BANK
                uint8_t *writeDest = &atariRam[addr];
#else
                uint8_t *writeDest;
                if ((addr & bankMask) == bankVal) { 
                    writeDest = &bank[addr & ~bankMask];;
                } else {
                    writeDest = &atariRam[addr];
                }
#endif
                __asm__ ("nop"); // ??? Seems to fix intermittent ROM boot error? 
                __asm__ ("nop"); 
                while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                      // wait rising clock edge
                do { 
                    r1 = REG_READ(GPIO_IN1_REG);
                } while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0);                      // wait falling clock edge
                __asm__ ("nop"); 
                __asm__ ("nop"); 
                REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask | extSel_Mask);                             // stop driving data lines, if they were previously driven                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
                
                uint8_t data = (r1 & dataMask) >> dataShift;
                if (addr == 1666 && fakeData) data += 1;
                *writeDest = data;   
#ifndef NO_BANK
                // TODO: hide bank logic in a register write overlap spot
                if (addr == 0xffff) { // PORTB write, switch bank
                    if (bank == &banks[0]) {
                        bank = &banks[0];
                    } else { 
                        bank = &banks[0];
                    } 
                }
#endif
            }
        } else {
            while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}                      // wait rising clock edge
            while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}                      // wait falling clock edge
            __asm__ ("nop"); 
            __asm__ ("nop"); 
            REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask | extSel_Mask);                             // stop driving data lines, if they were previously driven                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        }
//         __asm__ __volatile__("nop"); // 1 cycle
    }
}

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

    //Serial.printf("loop() disabling interrupts\n");
    //Serial.flush();
    //disableCore1WDT();
    //disableLoopWDT();
    //portDISABLE_INTERRUPTS();
    uint32_t oldint;
    XT_INTEXC_HOOK oldnmi = _xt_intexc_hooks[XCHAL_NMILEVEL];
    _xt_intexc_hooks[XCHAL_NMILEVEL] = my_nmi;  // saves 5 cycles, could save more 
    __asm__ __volatile__("rsil %0, 15" : "=r"(oldint) : : );

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
    delay(200);
    printf("avg loop1 %.2f ticks %.2f ns maxTicks %d at #%d   loop2 %.2f ticks %.2f ns maxTicks %d  diff %.2f\n", 
        avgTicks1, avgNs1, maxElapsed1, maxElapsedIndex1, avgTicks2, avgNs2, maxElapsed2, avgNs2 - avgNs1); 
    while(1) { yield(); delay(10); }
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

// NOTES:
// 8-pin i2c io expander: https://media.digikey.com/pdf/Data%20Sheets/NXP%20PDFs/PCF8574(A).pdf
// TODO: verify polarity of RW, MPD, casInh, etc 


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