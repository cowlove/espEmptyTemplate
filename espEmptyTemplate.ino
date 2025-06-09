//#define CONFIG_IDF_TARGET_ESP32 1
//#include "jimlib.h"
#include "serialLog.h"
#include <driver/gpio.h>
#include <xtensa/hal.h>
#include "freertos/xtensa_timer.h"
#include "esp_intr_alloc.h"
#include "rtc_wdt.h" .

#ifndef CSIM
#endif

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
int dram_sz = 16 * 1024;
uint32_t *dram;
volatile uint32_t buf;

WiFiUDP udpCmd;

uint32_t psramElapsedUsec;
uint32_t dramElapsedUsec;
int psramLoopCount = 0;
int dramLoopCount = 0;
int lateCount = 0;
int lateMax = 0;

void threadFunc(void *) { 
    uint32_t lastMs = millis();
    int pi = 0; 
    while(1) { 
        if (psramLoopCount != dramLoopCount) { 
            const uint32_t startUsec = micros();
#if 0 
            int len = dram_sz;
            while(len > 0) { 
                //OUT("sending len %d", len);
                udpCmd.beginPacket("255.255.255.255", 9000);
                udpCmd.write((uint8_t *)dram, 1024);
                udpCmd.endPacket();
                len -= 1024;
            }
#else
            for(register int n = 0; n < dram_sz / sizeof(uint32_t); n += 1) {
                *(psram + pi) = *(dram + n);
                //*(psram + pi) = 0;
                //buf = *(dram + n);
                //*psram = 0;
                pi = pi + 1;
                if (pi >= psram_sz / sizeof(uint32_t))
                    pi = 0;
            }
#endif
            psramElapsedUsec = micros() - startUsec;
            psramLoopCount++;
        }
        uint32_t ms = millis();
        if (ms / 1000 != lastMs / 1000) {
			int lm = lateMax;
			lateMax = 0;
            OUT("late %d lateMax %d dram %.5f %d psram %.5f %d", 
                lateCount, lm, psram_sz / 4.0 / dramElapsedUsec, dramLoopCount,
                dram_sz / 4.0 / psramElapsedUsec, psramLoopCount);
            lastMs = ms;
        }
        
        yield();
    }
}

uint32_t *isrPtr = 0;
uint32_t *isrPtrEnd = 0; 
uint32_t isrBeginUs = 0, isrEndUs = 0;
int isrCount = 0;

void IRAM_ATTR isr() {
    isrCount++;
    if (isrBeginUs == 0) isrBeginUs = micros();
    if (isrPtr != isrPtrEnd) {
        *isrPtr = PACK(*gpio0, *gpio1);
        isrPtr++;
        if (isrPtr == isrPtrEnd) {
            isrEndUs = micros();
        }
    } 
}
void IRAM_ATTR isrNO() {}

void setup() {
    psram = (uint32_t *) ps_malloc(psram_sz);
    dram = (uint32_t *)malloc(dram_sz);
    xTaskCreatePinnedToCore(threadFunc, "th", 4 * 1024, NULL, 0, NULL, 0);
    delay(1000);
    //j.begin();
    //j.jw.enabled = false;
    uint64_t mask = 0;
    for(auto i : pins) mask |= ((uint64_t)0x1) << i;
    uint32_t maskl = (uint32_t)mask;
    uint32_t maskh = (uint32_t)((mask & 0xffffffff00000000LL) >> 32);
    OUT("MASK %d %08x %08x %08x", pins.size(), maskl, maskh, PACK(maskl, maskh));

    if (0) {  
        for(auto i : pins) pinMode(i, INPUT_PULLDOWN);
        delay(100);
        OUT("PD   %d %08x %08x %08x", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
        delay(100);
        OUT("PU   %d %08x %08x %08x", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, OUTPUT);
        for(auto i : pins) digitalWrite(i, 0);
        delay(100);
        OUT("OL   %d %08x %08x %08x", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) digitalWrite(i, 1);
        delay(100);
        OUT("OH   %d %08x %08x %08x", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));

        for(auto i : pins) pinMode(i, INPUT_PULLUP);
        delay(100);
        OUT("PU   %d %08x %08x %08x", pins.size(), *gpio0, *gpio1, PACK(*gpio0, *gpio1));
    }
    for(auto i : pins) pinMode(i, INPUT);

    pinMode(0, OUTPUT);
    digitalWrite(0, 1);
    ledcInit(0, 2 * 1000000, 2, 1);
    ledcWrite(0, 2);

    for(int i = 0; i < 0; i++) { 
        uint32_t r0 = *gpio0;
        uint32_t r1 = *gpio1;
        OUT("%08x %08x %08x", r0, r1, PACK(r0, r1)); 
    }
    OUT("ps_malloc() result %x, %d, %d; malloc() result %x", 
        psram, ESP.getFreePsram(), ESP.getPsramSize(), dram);
   // esp_himem_get_free_size(), esp_himem_get_phys_size());
}

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_task_wdt.h"


void IRAM_ATTR iloop_psram() { 
	TIMERG1.wdtwprotect.wdt_wkey = TIMG_WDT_WKEY_V; // Unlock timer config.
	TIMERG1.wdtfeed.wdt_feed = 1; // Reset feed count.
	TIMERG1.wdtconfig0.wdt_en = 0; // Disable timer.
	TIMERG1.wdtwprotect.wdt_wkey = 0; // Lock timer config.

	esp_task_wdt_delete(NULL);

	//portDISABLE_INTERRUPTS();
	ESP_INTR_DISABLE(XT_TIMER_INTNUM);
    while(1) { 
       //wdtReset();
       const uint32_t startUsec = micros();
       static const int trig = 0x2;
       register uint32_t r0;
       uint32_t lastTsc = xthal_get_ccount();
       for(register int n = 0; n < psram_sz / sizeof(uint32_t); n += 1) {
            while(((r0 = *gpio0) & trig) == trig) {}

			uint32_t tsc = xthal_get_ccount();
			uint32_t elapsed = tsc - lastTsc;
			lastTsc = tsc;
			if (elapsed > 240) 
				lateCount++; 
			if (elapsed > lateMax) 
				lateMax = elapsed;

            //*(psram + n) = PACK(r0, *gpio1);
            //*(psram + n) = PACK(r0, 0);//*gpio1);
            while(((*gpio0) & trig) != trig) {}
        }
        dramElapsedUsec = micros() - startUsec;
    }
}


void IRAM_ATTR iloop_dram() {	
	portDISABLE_INTERRUPTS();
	ESP_INTR_DISABLE(XT_TIMER_INTNUM);
	while(1) { 
        //wdtReset();
        const uint32_t startUsec = micros();
        static const int trig = 0x2;
        uint32_t r0;
        uint32_t lastTsc = xthal_get_ccount();
        for(int i = 0; i < psram_sz / dram_sz; i++) { 
            for(register int n = 0; n < dram_sz / sizeof(uint32_t); n += 1) {
                //while(((r0 = *gpio0) & trig) == trig) {}
                
				uint32_t tsc = xthal_get_ccount();
				uint32_t elapsed = tsc - lastTsc;
				lastTsc = tsc;
				if (elapsed > 240) 
					lateCount++; 
				if (elapsed > lateMax) 
					lateMax = elapsed;
               
                //*(dram + n ) = PACK(r0, *gpio1);
                
                //while(((*gpio0) & trig) != trig) {}
            }
            dramLoopCount++;
        }
        dramElapsedUsec = micros() - startUsec;
    }
}

void loop() {
    //j.run();
    disableCore1WDT();
	//disableCore0WDT();
	disableLoopWDT();
    iloop_psram();
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
 
