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

#pragma GCC optimize("O3")

DRAM_ATTR uint32_t bEnabled[nrBanks * 2] = {1};

void IRAM_ATTR __attribute__((optimize("O1"))) iloop_pbi() {
    for(int i = 0; i < nrBanks; i++) {
        banks[i] = &atariRam[64 * 1024 / nrBanks * i];
    };
    for(int i = 0; i < nrBanks * 2; i++) {
        if ((i & 1) == 0) {
            bEnabled[i] = mpdMask | extSel_Mask;
        } else { 
            bEnabled[i] = dataMask | mpdMask | extSel_Mask;
        }
    };
    //atariRam[0xd803] = 0x00;

    for(auto i : pins) gpio_ll_input_enable(NULL, i);
    gpio_matrix_in(clockPin,      CORE1_GPIO_IN0_IDX, false);
    gpio_matrix_in(casInh_pin,    CORE1_GPIO_IN2_IDX, false);
    gpio_matrix_in(addr0Pin + 11, CORE1_GPIO_IN3_IDX, false);
    gpio_matrix_in(addr0Pin + 12, CORE1_GPIO_IN4_IDX, false);
    gpio_matrix_in(addr0Pin + 13, CORE1_GPIO_IN5_IDX, false);
    gpio_matrix_in(addr0Pin + 14, CORE1_GPIO_IN6_IDX, false);
    gpio_matrix_in(addr0Pin + 15, CORE1_GPIO_IN7_IDX, false);
    const uint8_t dedicBankMask = 0xfc;
    const uint8_t dedicBankShift = 2;

    while((dedic_gpio_cpu_ll_read_in() & 1) == 0) {}
    while((dedic_gpio_cpu_ll_read_in() & 1) != 0) {}
    uint32_t lastTscFall = XTHAL_GET_CCOUNT(); 
    while((dedic_gpio_cpu_ll_read_in() & 1) == 0) {}

    REG_WRITE(GPIO_ENABLE1_W1TS_REG, extSel_Mask | mpdMask); 
    REG_WRITE(GPIO_OUT1_W1TS_REG, extSel_Mask | mpdMask); 

    RAM_VOLATILE uint8_t * const bankD800[2] = { &atariRam[0xd800], &pbiROM[0] };

    static uint8_t dummyStore;
    RAM_VOLATILE uint8_t * dataDestOptions[2] = { &dummyStore, &dummyStore };
    uint32_t busMaskOptions[2] = { 0, 0 }; 
    const uint32_t enableClearMask = dataMask;
 
    do {    
        while((dedic_gpio_cpu_ll_read_in() & 0x1) != 0) {}
        #ifdef FAKE_CLOCK
        uint32_t tscFall = XTHAL_GET_CCOUNT();
        #endif
        int mpdSelect = (atariRam[0xd1ff] & 1);
        const uint32_t fetchedBusMask = busMask;
        uint32_t clrMask = (mpdSelect << mpdShift);// | ((fetchedBusMask & data0Mask) << (extSel_Pin - data0Pin));
        uint32_t setMask = clrMask ^ (mpdMask | extSel_Mask);
        banks[0xd800 >> bankShift] = bankD800[mpdSelect];
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        // Timing critical point.  At >= 10 ticks to before the REG_WRITE 
        REG_WRITE(GPIO_ENABLE1_W1TC_REG, dataMask);
        uint32_t r0 = REG_READ(GPIO_IN_REG);
        int bx = (dedic_gpio_cpu_ll_read_in() & dedicBankMask) >> dedicBankShift;
        const uint32_t pinEnableMask = bEnabled[bx];
        
        // idea: have banksEnabled[] array so we can map in individual pages 
        if ((r0 & (readWriteMask)) != 0) {
            #if 0
                int bank = (r0 & addrMask) >> (addrShift + bankShift - 1);
                bank |= (r0 & casInh_Mask) >> casInh_Shift;
                REG_WRITE(GPIO_ENABLE1_W1TS_REG, bEnabled[(r0 & casInh_Mask) >> casInh_Shift]);
                bank = bank >> 1;
            #else
                if (1) {
                    REG_WRITE(GPIO_ENABLE1_W1TS_REG, pinEnableMask);
                } else {
                    if ((r0 & casInh_Mask) != 0) { 
                        //REG_WRITE(GPIO_ENABLE1_W1TS_REG, fetchedBusMask);
                        REG_WRITE(GPIO_ENABLE1_W1TS_REG, pinEnableMask);
                    }
                }
            #endif
            uint16_t addr = (r0 & addrMask) >> addrShift;
            int bank = addr >> bankShift;
            RAM_VOLATILE uint8_t *ramAddr = banks[bank] + (addr & ~bankMask);
            uint8_t data = *ramAddr;
            REG_WRITE(GPIO_OUT1_REG, (data << dataShift) | setMask);             
            PROFILE(1, XTHAL_GET_CCOUNT() - tscFall); 

            while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}
        
        } else { //////////////// XXWRITE /////////////    
            uint16_t addr = (r0 & addrMask) >> addrShift; 
            dataDestOptions[1] = banks[addr >> bankShift] + (addr & ~bankMask);  
            int idx = (fetchedBusMask >> dataShift) & 1; // is bus disabled? 
            RAM_VOLATILE uint8_t *storeAddr = dataDestOptions[idx];
            
            while((dedic_gpio_cpu_ll_read_in() & 0x1) == 0) {}
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            __asm__ __volatile__("nop");
            PROFILE(0, XTHAL_GET_CCOUNT() - tscFall); 
            uint32_t r1 = REG_READ(GPIO_IN1_REG); 
            *storeAddr = (r1 >> dataShift);
        } 
    } while(1);
}
