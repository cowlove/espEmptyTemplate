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
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_ll.h"
#include "rom/gpio.h"


#include "core1.h"
//#pragma GCC optimize("O1") // O2 or above for core0Loop makes weird timings, improbably low core1 loop iterations around 60-70 cycles
//#pragma GCC pop_options

#pragma GCC optimize("O1")

void IRAM_ATTR __attribute__((optimize("O1"))) iloop_pbi() {
    for(int i = 0; i < nrBanks; i++) {
        banks[i] = &atariRam[64 * 1024 / nrBanks * i];
    };


    for(auto i : pins) gpio_ll_input_enable(NULL, i);
    gpio_matrix_in(clockPin, CORE1_GPIO_IN0_IDX, false);

    while((dedic_gpio_cpu_ll_read_in()) == 0) {}
    while((dedic_gpio_cpu_ll_read_in()) != 0) {}
    uint32_t lastTscFall = XTHAL_GET_CCOUNT(); 
    while((dedic_gpio_cpu_ll_read_in()) == 0) {}

    REG_WRITE(GPIO_ENABLE1_W1TS_REG, extSel_Mask | mpdMask); 
    REG_WRITE(GPIO_OUT1_W1TS_REG, extSel_Mask | mpdMask); 

    RAM_VOLATILE uint8_t * const bankD800[2] = { &atariRam[0xd800], &pbiROM[0] };

    static uint8_t dummyStore;
    uint8_t * dataDestOptions[2] = { &dummyStore, &dummyStore };
    uint32_t busMaskOptions[2] = { 0, 0 }; 
 
    do {    
        while((dedic_gpio_cpu_ll_read_in()) != 0) {}
        #ifdef FAKE_CLOCK
        uint32_t tscFall = XTHAL_GET_CCOUNT();
        #endif
        int mpdSelect = (atariRam[0xd1ff] & 1);
        busMaskOptions[1] = busMask;
        const uint32_t &fetchedBusMask = busMaskOptions[1];
        REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);
        uint32_t clrMask = (mpdSelect << mpdShift) | ((fetchedBusMask & data0Mask) << (extSel_Pin - data0Pin));
        //uint32_t clrMask = (mpdSelect << mpdShift) | ((busMaskOptions[1] & data0Mask) << (extSel_Pin - data0Pin));
        uint32_t setMask = clrMask ^ (mpdMask | extSel_Mask);
        REG_WRITE(GPIO_OUT1_W1TC_REG, dataMask | clrMask);
        uint32_t r0 = REG_READ(GPIO_IN_REG);
        
        // idea: have banksEnabled[] array so we can map in individual pages 
        if ((r0 & readWriteMask) == 0) { //////////////// XXWRITE /////////////    
            uint16_t addr = (r0 & addrMask) >> addrShift; 
            dataDestOptions[1] = banks[addr >> bankShift] + (addr & ~bankMask);  
            int idx = (fetchedBusMask >> dataShift) & 1;
            uint8_t *storeAddr = dataDestOptions[idx];
            //while((dedic_gpio_cpu_ll_read_in()) == 0) {};
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            uint32_t r1 = REG_READ(GPIO_IN1_REG); 
            *storeAddr = (r1 >> dataShift);
            PROFILE(0, XTHAL_GET_CCOUNT() - tscFall); 

        } else { //////////////// XXR E A D /////////////    
#if 1
            if ((r0 & casInh_Mask) != 0) 
                REG_WRITE(GPIO_ENABLE1_W1TS_REG, fetchedBusMask);
#else
            // why is this so slow 
            int idx = ((r0 & casInh_Mask) >> casInh_Shift);
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, busMaskOptions[idx]);
#endif

            uint16_t addr = (r0 & addrMask) >> addrShift;
            int bank = addr >> bankShift;
            RAM_VOLATILE uint8_t *ramAddr = banks[bank] + (addr & ~bankMask);
            uint8_t data = *ramAddr;
            REG_WRITE(GPIO_OUT1_W1TS_REG, (data << dataShift) | setMask); 
            
            banks[0xd800 >> bankShift] = bankD800[mpdSelect];
            PROFILE(1, XTHAL_GET_CCOUNT() - tscFall); 
            //while((dedic_gpio_cpu_ll_read_in()) == 0) {}
        } 
    } while(1);
}
