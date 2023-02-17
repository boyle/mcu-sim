/* Copyright 2023, Alistair Boyle, 3-clause BSD License */
#pragma once
#include "config.h"

#include <stdint.h> // uint32_t
#include <stdbool.h> // bool

#define SRAM_BASE (0x20000000)
#define SRAM_MAX_SIZE (128*1024)

#define FLASH_BASE (0x00000000)
#define FLASH_MAX_SIZE (128*1024)

typedef struct {
    const char * name;
    // ARM registers
    uint32_t r[17];
    uint32_t apsr, ipsr, epsr; // (application, interrupt, exception) program status register
    bool primask; // priority = 0
    bool faultmask; // priority = -1
    uint8_t basepri; // base priority; 0 = disabled
    uint32_t control; //  special purpose control register
    bool handler_mode; // 0 = thread_mode (reset), 1 = handler_mode (exception)
    bool exception_active [512];
    int exception_priority [512];
    union {
        float s[32]; // single-precision
        double d[16]; // double-precision (d[16-32]=undefined)
    };
    uint32_t fpscr; // floating point status and control register
    uint32_t cp[16]; // co-processor
    bool pm;
    bool sleeping;
    bool halt;

    // most recent opcode
    uint32_t opcode;

    // memory
    uint8_t sram[SRAM_MAX_SIZE];
    uint8_t flash[FLASH_MAX_SIZE];

    // pointer into the System-on-a-Chip
    void * soc;

    // ---- simulator registers [15:0] warnings, [31:16] errors
    //uint32_t ret; // [0] undefined=UsageFault, [1] unpredictable
    uint32_t last_pc;
} core_t;

void core_init(core_t * core, const char * name);
void core_reset(core_t * core);
void core_tick(core_t * core);
int  core_is_stalled(core_t * core);
