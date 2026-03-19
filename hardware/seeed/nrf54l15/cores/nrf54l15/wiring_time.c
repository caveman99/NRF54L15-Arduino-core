#include "Arduino.h"

#include <limits.h>

#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

unsigned long millis(void)
{
    return (unsigned long)k_uptime_get();
}

unsigned long micros(void)
{
    return (unsigned long)k_cyc_to_us_floor64(k_cycle_get_64());
}

void delay(unsigned long ms)
{
    if (ms == 0) {
        return;
    }

    k_sleep(K_MSEC(ms));
}

void delayLowPowerIdle(unsigned long ms)
{
    delay(ms);
}

static void disableSystemOffRetention(void)
{
#ifdef NRF_MEMCONF
    for (uint32_t i = 0; i < MEMCONF_POWER_MaxCount; ++i) {
        NRF_MEMCONF->POWER[i].RET = 0U;
        NRF_MEMCONF->POWER[i].RET2 = 0U;
    }
#endif
}

static void clearSystemOffVprRetention(void)
{
#ifdef NRF_MEMCONF
    if (MEMCONF_POWER_MaxCount > 1U) {
        NRF_MEMCONF->POWER[1U].RET &= ~MEMCONF_POWER_RET_MEM0_Msk;
    }
#endif
}

static bool programSystemOffWakeMs(unsigned long ms)
{
    if (ms == 0UL) {
        return false;
    }

#if defined(CONFIG_POWEROFF) && defined(CONFIG_NRF_GRTC_START_SYSCOUNTER)
    const uint64_t wake_time_us = (uint64_t)ms * 1000ULL;
    const int err = z_nrf_grtc_wakeup_prepare(wake_time_us);
    return err == 0;
#else
    ARG_UNUSED(ms);
    // Timed wake is not available without Zephyr poweroff support.
    // The core will still enter SYSTEMOFF, but it may require an external wake source.
    return false;
#endif
}

static void enterTimedSystemOff(bool disableRamRetention, unsigned long ms)
{
    (void)programSystemOffWakeMs(ms);

    clearSystemOffVprRetention();
    if (disableRamRetention) {
        disableSystemOffRetention();
    }

    __asm volatile("cpsid i" ::: "memory");
    __asm volatile("dsb 0xF" ::: "memory");
    __asm volatile("isb 0xF" ::: "memory");
    NRF_RESET->RESETREAS = 0xFFFFFFFFUL;
    __asm volatile("dsb 0xF" ::: "memory");

    NRF_REGULATORS->SYSTEMOFF = REGULATORS_SYSTEMOFF_SYSTEMOFF_Enter;
    __asm volatile("dsb 0xF" ::: "memory");
    while (true) {
        __asm volatile("wfe");
    }
}

void delaySystemOff(unsigned long ms)
{
    enterTimedSystemOff(false, ms);
}

void delaySystemOffNoRetention(unsigned long ms)
{
    enterTimedSystemOff(true, ms);
}

void delayMicroseconds(unsigned int us)
{
    if (us == 0) {
        return;
    }

    k_busy_wait(us);
}
