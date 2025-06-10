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
int dram_sz = 2048;
uint32_t *dram;
volatile uint32_t buf;
static const int testFreq = 1.8 * 1000000;
static const int lateThresholdTicks = 180 * 2 * 1000000 / testFreq;
static const uint32_t halfCycleTicks = 240 * 1000000 / testFreq / 2;

uint32_t dramElapsedTsc;
uint32_t lateTsc;
int dramLoopCount = 0, psramLoopCount = 0;
int lateCount = 0;
int lateMax = 0, lateMin = 0, lateIndex = -1;
int cbCount = 0;

// Callback function (optional)
static bool async_memcpy_callback(async_memcpy_context_t*, async_memcpy_event_t*, void*) {
    cbCount++;
    return false;
}

void IRAM_ATTR threadFunc(void *) { 
    int pi = 0; 

    const async_memcpy_config_t config {
        .backlog = 8,
        .sram_trans_align = 0,
        .psram_trans_align = 0,
        .flags = 0
    };
    async_memcpy_handle_t handle = NULL;
    if (esp_async_memcpy_install(&config, &handle) == ESP_OK) {
        printf("Async memcpy driver installed successfully.\n");
    } else {
        printf("Failed to install async memcpy driver.\n");
        return;
    }

    uint32_t lastMs = millis();
    while(1) { 
        if (psramLoopCount != dramLoopCount) {
            psramLoopCount++; 
            const uint32_t startUsec = micros();
            if (esp_async_memcpy(handle, (void *)(psram + pi), (void *)dram, dram_sz, 
                async_memcpy_callback, NULL)) { 
                printf("async ERR psramLoopCount %d/%d\n", psramLoopCount, psram_sz / dram_sz);
                break;
            } 
            pi = pi + dram_sz / sizeof(uint32_t);
            if (pi > (psram_sz - dram_sz * 2) / sizeof(uint32_t))
                break;
        }
        uint32_t ms = millis();
        if (ms / 1000 != lastMs / 1000) {
			int lm = lateMax;
            int lmi = lateMin;
			lateMax = lateMin = 0;
            printf("cb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x dram %.5f %d\n", 
                cbCount, lateCount, lateIndex, lmi, lm, lateTsc, dram_sz / 4.0 / dramElapsedTsc * 240, dramLoopCount);
            lastMs = ms;
            lateIndex = -1;
            //lateCount = 0;
            lateTsc = 0 ;
        }
        //delayMicroseconds(1);
    }
    // DONE
    while(cbCount != psramLoopCount) delay(1);
    int lm = lateMax;
    int lmi = lateMin;
    lateMax = lateMin = 0;
    printf("cb %d late %d lateIndex %d lateMin %d lateMax %d lateTsc %08x dram %.5f %d psramLoop %d\n", 
        cbCount, lateCount, lateIndex, lmi, lm, lateTsc, dram_sz / 4.0 / dramElapsedTsc * 240, dramLoopCount, psramLoopCount);
    ESP.restart();
}

void setup() {
    const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    psram = (uint32_t *) heap_caps_aligned_alloc(64, psram_sz,  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    dram = (uint32_t *)heap_caps_aligned_alloc(64, dram_sz, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    //delay(2000);
    //j.begin();
    //j.jw.enabled = false;
    uint64_t mask = 0;
    for(auto i : pins) mask |= ((uint64_t)0x1) << i;
    uint32_t maskl = (uint32_t)mask;
    uint32_t maskh = (uint32_t)((mask & 0xffffffff00000000LL) >> 32);
    printf("MASK %d %08x %08x %08x\n", pins.size(), maskl, maskh, PACK(maskl, maskh));

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
        delay(100);
        printf("PU   %d %08x %08x %08x\n", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));
    }
    for(auto i : pins) pinMode(i, INPUT);

    pinMode(0, OUTPUT);
    digitalWrite(0, 1);
    ledcAttachChannel(0, testFreq, 3, 1);
    ledcWrite(0, 4);

    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        printf("%08x %08x %08x\n", r0, r1, PACK(r0, r1)); 
    }
    printf("freq %.4fMhz threshold %d halfcycle %d ps_malloc() result %x, %d, %d; malloc() result %x\n", 
        testFreq / 1000000.0, lateThresholdTicks, halfCycleTicks, psram, ESP.getFreePsram(), ESP.getPsramSize(), dram);
    // esp_himem_get_free_size(), esp_himem_get_phys_size());
    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
    delay(100);
}

void IRAM_ATTR iloop_dram() {	
	portDISABLE_INTERRUPTS();
	ESP_INTR_DISABLE(XT_TIMER_INTNUM);
    uint32_t startTsc = xthal_get_ccount();
    static const int trig = 0x1;
    uint32_t r0;
    uint32_t tsc, lastTsc = xthal_get_ccount();
    uint32_t *dram_end = dram + dram_sz / sizeof(uint32_t);
    uint32_t triggerMask = 0xffffffff;
    bool triggered = false;
    register uint32_t *out = dram;
    while(1) { 
        while(((r0 = *gpio0) & trig) == trig) {}
        tsc = xthal_get_ccount();
        const uint32_t r1 = *gpio1;
        if (r0 & triggerMask) { 
            triggered = true;
        }
        if (triggered ) { 
            *(out++) = PACK(r0, r1);
            //*(out++) = tsc;
            if (out >= dram_end) { 
                out = dram;
                dramLoopCount++;
                if (0) { 
                    dramElapsedTsc = tsc - startTsc;
                    startTsc = tsc;
                }
            }
        }
        while(((*gpio0) & trig) != trig) {}
                    
        if (1) {  
            //*(out++) = tsc;
            uint32_t elapsed = tsc - lastTsc;
            lastTsc = tsc;
            if (elapsed > lateThresholdTicks) {
                lateTsc = tsc; 
                lateCount++;
                lateIndex = out - dram;
                if (elapsed > lateMax)  
                    lateMax = elapsed;
                if (elapsed < lateMin || lateMin == 0)
                    lateMin = elapsed;
            }
            if (0) { 
                if (elapsed > lateMax)  
                    lateMax = elapsed;
                if (elapsed < lateMin || lateMin == 0)
                    lateMin = elapsed;
            }
            //while(xthal_get_ccount() - tsc < halfCycleTicks - 20) {}

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
 
