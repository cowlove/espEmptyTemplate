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

#ifndef CSIM
#endif

#include <vector>
using std::vector;

// Usable pins
// 0-18 22            (20)
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
static const int      casInh_Mask = 0x1;    // bit0 
static const int      addrShift = 1;
static const int      addrMask = 0xffff << addrShift;  // bits 1-16
static const int      clockPin = 17;
static const int      clockMask = (1 << clockPin);
static const int      readWriteBit = (1 << 18); // TMP use clock as RW bit, will always indicate a read

//GPIO1 bits
static const int      dataShift = 9;
static const int      dataMask = (0xff << dataShift);
static const int      extSel_PortPin = 38 /*pin*/ - 32 /* port base*/;
static const int      extSel_Mask = (1 << extSel_PortPin);


volatile uint8_t atariRam[64 * 1024] = {0xff};


vector<int> pins = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,38,39,40,41,42,43,44,45,46,47,48};

#define PACK(r0, r1) (((r1 & 0x0001ffc0) << 15) | ((r0 & 0x0007fffe) << 2)) 


//JStuff j;
//CLI_VARIABLE_FLOAT(x, 1);
volatile uint32_t *gpio0 = (volatile uint32_t *)GPIO_IN_REG;
volatile uint32_t *gpio1 = (volatile uint32_t *)GPIO_IN1_REG;
int psram_sz = 7.8 * 1024 * 1024;
uint32_t *psram;
static const int dma_sz = 2048;
static const int dma_bufs = 8;
static const int dram_sz = 2048 * dma_bufs;

uint32_t *dram;
volatile uint32_t buf;
static const int testFreq = 1.8 * 1000000;
static const int lateThresholdTicks = 180 * 2 * 1000000 / testFreq;
static const uint32_t halfCycleTicks = 240 * 1000000 / testFreq / 2;

uint32_t dramElapsedTsc;
uint32_t lateTsc;
int dramLoopCount = 0, psramLoopCount = 0;
int lateCount = 0;
int lateMax = 0, lateMin = 9999, lateIndex = -1;
int cbCount = 0;
volatile bool stop = false;

// Callback function (optional)
static bool async_memcpy_callback(async_memcpy_context_t*, async_memcpy_event_t*, void*) {
    cbCount++;
    return false;
}

volatile int tickCount = 0;
void threadFunc2(void *) {
    while(1) {  
        printf("tickCount %d\n", tickCount);
        tickCount = 0;
        delay(1000);
    }
}
void threadFunc(void *) { 
    int pi = 0; 

    const async_memcpy_config_t config {
        .backlog = dma_bufs,
        .sram_trans_align = 0,
        .psram_trans_align = 0,
        .flags = 0
    };
    async_memcpy_handle_t handle = NULL;
    if (esp_async_memcpy_install(&config, &handle) != ESP_OK) {
        printf("Failed to install async memcpy driver.\n");
        return;
    }

    uint32_t lastMs = millis();
    while(1) { 
        if (psramLoopCount != dramLoopCount) {
            void *dma_buf = dram + (dma_sz * (psramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t));
            if (esp_async_memcpy(handle, (void *)(psram + pi), (void *)dma_buf, dma_sz, 
                async_memcpy_callback, NULL) != ESP_OK) { 
                printf("esp_asyn_memcpy() error, psramLoopCount %d/%d\n", psramLoopCount, psram_sz / dma_sz);
                break;
            } 
            pi = pi + dma_sz / sizeof(uint32_t);
            if (pi >= (psram_sz - dma_sz * 2) / sizeof(uint32_t)) {
            //if (pi >= dma_sz * 4 / sizeof(uint32_t)) {
                pi = 0;
                break;
            }
            if (dramLoopCount - psramLoopCount >= dma_bufs) { 
                printf("esp_aync_memcpy buffer overrun\n");
                break;
            }
            psramLoopCount++; 
        }
        uint32_t ms = millis();
        //if (ms / 5000 != lastMs / 5000) {
        //    break;
        //}
        //delayMicroseconds(1);
    }
    // DONE
    stop = true;
    int lm = lateMax;
    int li = lateIndex;
    int lmi = lateMin;
    int lc = lateCount;
    uint32_t ltsc = lateTsc;
    printf("cbCount %d psramLoopCount %d\n", cbCount, psramLoopCount);
    //while(cbCount != psramLoopCount) delay(1);
    printf("\n\n\ncb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x dram %.5f %d psramLoop %d\n", 
        cbCount, lc, li, lmi, lm, ltsc, dma_sz / 4.0 / dramElapsedTsc * 240, dramLoopCount, psramLoopCount);
    delay(100);

    if (0) {
        for(uint32_t *p = dram; p < dram + dram_sz / sizeof(uint32_t); p++) {
          printf("DRAM %08x %08x\n", (p - dram) * sizeof(uint32_t), *p);
        }
        //for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p += dma_sz / sizeof(uint32_t)) {
    }

    if (0) { 
        int buckets[256];
        bzero(buckets, sizeof(buckets));
        uint32_t last = psram[0];
        for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p++) {
            //printf("PSRAM %08x %08x\n", (p - psram) * sizeof(uint32_t), *p - last);
            unsigned int delta = *p - last;
            delta = min(sizeof(buckets) / sizeof(buckets[0]), delta);
            buckets[delta]++;
            last = *p;
        }
        for(int i = 0; i < sizeof(buckets) / sizeof(buckets[0]); i++) { 
            printf("%d %d BUCKETS\n", i, buckets[i]);
        }
    }
    if (1) { 
        for(uint32_t *p = psram; p < psram + (psram_sz - dma_sz * 2) / sizeof(uint32_t); p++) {
    //        printf("P %08X\n",*p);
            printf("P %08X %08X\n", p - psram, *p);
        }
    }
    ESP.restart();
}

dedic_gpio_bundle_handle_t bundleA, bundleB;

void setup() {
    delay(1000);
    Serial.begin(115200);
    printf("setup()\n");

    const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    psram = (uint32_t *) heap_caps_aligned_alloc(64, psram_sz,  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    dram = (uint32_t *)heap_caps_aligned_alloc(64, dram_sz, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    uint64_t mask = 0;
    for(auto i : pins) mask |= ((uint64_t)0x1) << i;
    uint32_t maskl = (uint32_t)mask;
    uint32_t maskh = (uint32_t)((mask & 0xffffffff00000000LL) >> 32);
    printf("MASK %d %08x %08x %08x\n", pins.size(), maskl, maskh, PACK(maskl, maskh));

    bzero(psram, psram_sz);
    bzero(dram, dram_sz);
    if (0) {  
        for(auto i : pins) pinMode(i, INPUT_PULLDOWN);
        delay(100);
        printf("PD   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
        delay(100);
        printf("PU   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, OUTPUT);
        for(auto i : pins) digitalWrite(i, 0);
        delay(100);
        printf("OL   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) digitalWrite(i, 1);
        delay(100);
        printf("OH   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
        do { 
            delay(100);
            printf("PU   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));
        } while(1);
    }
    for(auto i : pins) pinMode(i, INPUT_PULLDOWN);
    pinMode(38, OUTPUT);
    pinMode(42, OUTPUT);
    digitalWrite(42, 1);
    int clkPin = 17;
    pinMode(clkPin, OUTPUT);
    digitalWrite(clkPin, 0);
    ledcAttachChannel(clkPin, testFreq, 1, 0);
    ledcWrite(clkPin, 1);

    for(int i = 0; i < 100; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }

    printf("freq %.4fMhz threshold %d halfcycle %d ps_malloc() result %x, %d, %d; malloc() result %x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, psram, ESP.getFreePsram(), ESP.getPsramSize(), dram);
    // esp_himem_get_free_size(), esp_himem_get_phys_size());
    xTaskCreatePinnedToCore(threadFunc2, "th", 2 * 1024, NULL, 0, NULL, 0);
    delay(500);
    digitalWrite(42, 0);

    int bundleA_gpios[] = {clockPin};
    dedic_gpio_bundle_config_t bundleA_config = {
        .gpio_array = bundleA_gpios,
        .array_size = sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]),
        .flags = {
            .in_en = 1,
            .out_en = 0
        },
    };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleA));

    int bundleB_gpios[] = {40,41,42,43,44,45,46,47};
    dedic_gpio_bundle_config_t bundleB_config = {
        .gpio_array = bundleB_gpios,
        .array_size = sizeof(bundleB_gpios) / sizeof(bundleB_gpios[0]),
        .flags = {
            .out_en = 1
        },
    };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleB_config, &bundleB));

}

void IRAM_ATTR iloop_dram() {	
	portDISABLE_INTERRUPTS();
	ESP_INTR_DISABLE(XT_TIMER_INTNUM);
    uint32_t startTsc = xthal_get_ccount();
    static const int clockMask = 0x2;
    uint32_t r0r, r0f;
    uint32_t tsc, lastTsc = xthal_get_ccount();
    uint32_t *out = dram;
    uint32_t *dram_end = dram + dma_sz / sizeof(uint32_t);
    uint32_t triggerMask = 0x0003fffc;
    bool triggered = false;
    uint32_t r1;
    while(1) {
        while(stop) {} 
        while(((r0f = *gpio0) & clockMask)) {}
        RSR(CCOUNT, tsc);
        tsc = xthal_get_ccount();
        r1 = *gpio1;
        uint32_t elapsed = tsc - lastTsc;
        lastTsc = tsc;
        if (elapsed > lateThresholdTicks) {
            lateTsc = tsc; 
            lateCount++;
            lateIndex = out - dram;
            if (elapsed > lateMax)  
                lateMax = elapsed;
            if (elapsed < lateMin)
                lateMin = elapsed;
        }else {
            if (elapsed > lateMax)  
                lateMax = elapsed;
            //if (elapsed < lateMin)
            //    lateMin = elapsed;
        }

        if (!triggered && (r0f & triggerMask)) { 
            triggered = true;
        }

        //while(xthal_get_ccount() - tsc < halfCycleTicks - 50) {}

        while(!((r0r = *gpio0) & clockMask)) {}
        r1 = *gpio1;

        if (triggered) { 
            if (r0r & 0x3) {
                *out = PACK(r0f, r1);
            } else { 
                *out = PACK(r0f, r1);
            }
            //*out = tsc;
            out++;
            if (out == dram_end) { 
                dramLoopCount++;
                out = dram + dma_sz * (dramLoopCount & (dma_bufs - 1)) / sizeof(uint32_t);
                dram_end = out + dma_sz / sizeof(uint32_t);
            }
        }                    
    }
}




void IRAM_ATTR iloop_pbi() {	

    while(0) {
        while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}
        REG_WRITE(GPIO_OUT1_W1TC_REG, extSel_Mask);
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}
        REG_WRITE(GPIO_OUT1_W1TS_REG, extSel_Mask);
    }

    while(0) { 
        while((*gpio0 & clockMask) != clockMask) {}              
        REG_WRITE(GPIO_OUT1_W1TC_REG, extSel_Mask);
        while((*gpio0 & clockMask) == clockMask) {}              
        REG_WRITE(GPIO_OUT1_W1TS_REG, extSel_Mask);
    }


    uint32_t r;
    int ram = 0;
    while(1) {
        while(stop) {} 
        while(((r = *gpio0) & clockMask) == 0) {}
        uint16_t addr = (r & addrMask) >> addrShift;         // read address, RW flag and casInh_  from bus 
        
        REG_WRITE(GPIO_OUT1_W1TC_REG, extSel_Mask);          // drive EXTSEL low  
        if (!(r & readWriteBit)) {                           // 1. READ        
            dedic_gpio_cpu_ll_write_all(atariRam[addr]);
            //REG_WRITE(GPIO_OUT1_REG, atariRam[addr] << dataShift);                         //    write DATA lines
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);      //    enable DATA lines for output
            while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}//    wait falling clock edge
            // NOP, NOP delay                                //    leave data on the bus for tHR
            REG_WRITE(GPIO_OUT1_REG, 0); // TMP to show timing
            REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);      //    stop driving data lines                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
            tickCount++;
        } else {                                             // 2. WRITE 
            while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}//    wait falling clock edge
            atariRam[0] = (*gpio1 & dataMask) >> dataShift;  //    get write data from bus, write to local RAM 
        }
        REG_WRITE(GPIO_OUT1_W1TS_REG, extSel_Mask |~0);          //    release EXTSEL  
        dedic_gpio_cpu_ll_write_all(0);
    }
}

volatile uint32_t dummy = 0;
volatile double avgNs1, avgNs2, avgTicks1, avgTicks2;
int maxElapsed1 = 0, maxElapsed2 = 0;
int maxElapsedIndex1 = 0;
void IRAM_ATTR iloop_timings1() {
    int iterations = 1000000; 
    uint32_t startTsc = xthal_get_ccount();
    uint32_t r;
    uint32_t lastTsc = startTsc;
    for(int n = 0; n < iterations; n++) { 
        //dedic_gpio_bundle_read_in(bundleA); // 95ns
        //dedic_gpio_cpu_ll_read_in();      //20.85ns
        dedic_gpio_cpu_ll_write_all(0xff);
        dedic_gpio_cpu_ll_write_all(0);

#if 0
        //r = *gpio0;
        while(!(r = *gpio0) & (1 << 17)) {}                                          // read addr, rw, clk, casInh                                                      //  75.0
        if ((r & casInh_Mask))                                                      // 12.5
            continue;
        if (!(r & readWriteBit)) {                                                  //  37.48 ns 
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);      //    enable DATA lines for output
            uint16_t addr = (r & addrMask) >> addrShift;                            //  12.5 ns 
            *gpio1 = (((uint32_t)atariRam[addr]) << dataShift) | extSel_Mask;       //  25.0 ns 
        }
        uint32_t tsc = xthal_get_ccount();                                      //  25.0 ns
        int elapsed = tsc - lastTsc;
        lastTsc = tsc;
        if (elapsed > maxElapsed1) {
            maxElapsed1 = elapsed; 
            maxElapsedIndex1 = n;
        }
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
        while((r = *gpio0) & (1 << 17)) {}                                          // read addr, rw, clk, casInh                                                      //  75.0
        if ((r & casInh_Mask))                                                      // 12.5
            continue;
        if (!(r & readWriteBit)) {                                                  //  37.48 ns 
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, dataMask);      //    enable DATA lines for output
            uint16_t addr = (r & addrMask) >> addrShift;                            //  12.5 ns 
            *gpio1 = (((uint32_t)atariRam[addr]) << dataShift) | extSel_Mask;       //  25.0 ns 
            //uint32_t t = xthal_get_ccount();                                      //  25.0 ns
        }
        uint32_t tsc = xthal_get_ccount();                                      //  25.0 ns
        int elapsed = tsc - lastTsc;
        lastTsc = tsc;
        if (elapsed > maxElapsed2)
            maxElapsed2 = elapsed; 
    }
    uint32_t endTsc = xthal_get_ccount();
    avgTicks2 = 1.0 * (endTsc - startTsc) / iterations;
    avgNs2 = avgTicks2 / 240 * 1000;
}


void loop() {
    //j.run();


    if (1) { 
        disableCore1WDT();
        portDISABLE_INTERRUPTS();
        ESP_INTR_DISABLE(XT_TIMER_INTNUM);
        disableLoopWDT();
    }
    maxElapsed1 = maxElapsed2 = maxElapsedIndex1 = -1;
    iloop_pbi();
    iloop_timings1();
    //iloop_timings2();
    portENABLE_INTERRUPTS();
    printf("avg loop1 %.2f ticks %.2f ns maxTicks %d at #%d   loop2 %.2f ticks %.2f ns maxTicks %d  diff %.2f\n", 
        avgTicks1, avgNs1, maxElapsed1, maxElapsedIndex1, avgTicks2, avgNs2, maxElapsed2, avgNs2 - avgNs1); 
    delay(100);
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