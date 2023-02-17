/* Copyright 2019, Alistair Boyle, 3-clause BSD License */
#include "config.h"

#include <stdio.h> // printf

#include "cmocka.h"
#include "armv7m.h"

static void coredump(const core_t * core)
{
    for(int i = 0; i < 13; i++) {
        printf("R%d=%d ", i, core->r[i]);
    }
    printf("SP=%d LR=%d PC=%d\n", core->r[13], core->r[14], core->r[15]);
    printf("opcode=0x%04x b%016b\n", core->opcode, core->opcode);
}

static void memdump(const core_t * core, int addr, int len)
{
    printf("@0x%08x:  ", addr);
    uint32_t * mem = (uint32_t *) &core->flash[addr];
    len = len / 4 + (len & 3 ? 1 : 0);
    for(int i = 0; i < len; i++) {
        printf("%08x ", mem[i]);
    }
    printf("\n");
}

static core_t core;

static int setup (void ** state)
{
    core_init(&core, "ARMv7-M");
    return 0;
}

static void test_lsl (void ** state)
{
    uint32_t * mem = (uint32_t *) core.flash;
    mem[1] = 2 * 4;
    mem[2] = (1 << 3) + 2; // LSL (immediate) T1 = MOV Rd=2, Rm=1 = [Rm]-->[Rd]
    core_reset(&core);
    for(int i = 0; i < 13; i++) {
        core.r[i] = i;
    }
    coredump(&core);
    memdump(&core, 0, 16);
    core_tick(&core);
    printf("-- TICK --\n");
    coredump(&core);
    memdump(&core, 0, 16);
    for(int i = 0; i < 13; i++) {
        if(i == 2) {
            assert_int_equal(core.r[i], i - 1);
        }
        else {
            assert_int_equal(core.r[i], i);
        }
    }
    assert_int_equal(core.r[15], mem[1] + 2);
}

int main(int argc, char ** argv)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_lsl),
    };
    return cmocka_run_group_tests(tests, setup, NULL);
}
