/*
  sitl_bl_time.c - deterministic simulated clock for the bootloader SITL

  The bootloader is single threaded polled code where every wait loop
  reads the timer or the input pin. Each such read advances simulated
  time by a fixed grant, so the bit-banged serial timing is exact and
  repeatable regardless of host load. A tick, run at most once per 50us
  of simulated time, polls the UDP input and paces the simulation
  against the wall clock.
 */

#include "sitl_bl.h"

#include <stdio.h>
#include <time.h>

// sim time granted per timer/gpio read, roughly what the register read
// costs on the real MCU
#define READ_GRANT_NS 100
#define TICK_PERIOD_NS 50000

static uint64_t sim_ns;
static uint64_t last_tick_ns;
static uint64_t wall_ref_ns; // wall clock when sim time was zero
static bool in_tick;

static uint64_t wall_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void sitl_bl_time_init(void)
{
    wall_ref_ns = wall_ns();
}

uint64_t sitl_bl_time_ns(void)
{
    return sim_ns;
}

// CAN poll hook, overridden by sys_can_SITL.c in CAN builds
void __attribute__((weak)) sitl_bl_can_poll(void)
{
}

static void sitl_bl_tick(void)
{
    sitl_bl_pin_poll();
    sitl_bl_can_poll();

    // pace against the wall clock: sim must not run ahead of
    // wall * speedup
    const float speedup = sitl_bl_cfg.speedup;
    if (speedup > 0) {
        const uint64_t target_wall = wall_ref_ns + (uint64_t)(sim_ns / speedup);
        uint64_t now = wall_ns();
        if (target_wall > now + 1000000ULL) {
            const uint64_t delay = target_wall - now;
            struct timespec ts = { .tv_sec = delay / 1000000000ULL,
                .tv_nsec = delay % 1000000000ULL };
            nanosleep(&ts, NULL);
        } else if (now > target_wall + 500000000ULL) {
            // fell far behind (host stall): rebase rather than racing
            // to catch up
            wall_ref_ns = now - (uint64_t)(sim_ns / speedup);
        }
    }
}

void sitl_bl_advance(uint32_t ns)
{
    sim_ns += ns;
    if (!in_tick && sim_ns - last_tick_ns >= TICK_PERIOD_NS) {
        last_tick_ns = sim_ns;
        in_tick = true;
        sitl_bl_tick();
        in_tick = false;
    }
}

uint16_t sitl_bl_timer_us(void)
{
    sitl_bl_advance(READ_GRANT_NS);
    return (uint16_t)(sim_ns / 1000ULL);
}
