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
#ifndef CSIM
#endif

#include <vector>
using std::vector;

// Usable pins
// 1-18             (18)
// 38-48 (6-16)     (11)
//01234567012345670123456701234567
//01101111111111111110000000000000
//00000011111111110000000000000000

//data 8
//addr 16
//clock sync halt write 


vector<int> pins = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,38,39,40,41,42,43,44,45,46,47,48};

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
bool stop = false;

// Callback function (optional)
static bool async_memcpy_callback(async_memcpy_context_t*, async_memcpy_event_t*, void*) {
    cbCount++;
    return false;
}

void IRAM_ATTR threadFunc(void *) { 
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

void setup() {
    const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    psram = (uint32_t *) heap_caps_aligned_alloc(64, psram_sz,  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    dram = (uint32_t *)heap_caps_aligned_alloc(64, dram_sz, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    delay(500);
    Serial.begin(921600);
    //j.begin();
    //j.jw.enabled = false;
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
    for(auto i : pins) pinMode(i, INPUT_PULLUP);

    pinMode(0, OUTPUT);
    digitalWrite(0, 1);
    ledcAttachChannel(0, testFreq, 1, 1);
    ledcWrite(0, 0);

    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }
    printf("freq %.4fMhz threshold %d halfcycle %d ps_malloc() result %x, %d, %d; malloc() result %x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, psram, ESP.getFreePsram(), ESP.getPsramSize(), dram);
    // esp_himem_get_free_size(), esp_himem_get_phys_size());
    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
    delay(500);
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

void loop() {
    //j.run();
    disableCore1WDT();
	//disableCore0WDT();
	disableLoopWDT();
    iloop_dram();
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
 
