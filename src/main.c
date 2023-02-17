/* Copyright 2023 Alistair Boyle, 3-clause BSD License */
#include "config.h"

#include <stdio.h> // printf, fopen
#include <errno.h> // errno
#include <string.h> // strerror
#include <time.h> // clock

#include "argv.h"
#include "armv7m.h"

#define WIPE "\33[2K\r"
#define OPS_PER_UPDATE 1000

static void printf_progress(core_t * core, int finale)
{
    static int n = -1;
    static time_t t0;
    if(n == -1) {
        t0 = clock();
    }
    if((n++ % OPS_PER_UPDATE == (OPS_PER_UPDATE - 1)) || finale) {
        time_t t1 = clock();
        double sec = (t1 - t0) / (double) CLOCKS_PER_SEC;
        double mips = (double) ((n - 1) % OPS_PER_UPDATE + 1) / sec * 1e-6;
        printf(WIPE "[%s] PC = 0x%08x (%0.2f MIPS)", core->name, core->r[15], mips);
        t0 = t1;
        n = 0;
    }
}

static void printf_coredump(core_t * core)
{
    for(int i = 0; i < 16; i++) {
        printf("R%-2d%3s = 0x%08x%s",
               i, (i % 2) ? ((i == 13) ? "/sp" : (i == 15) ? "/pc" : "   ") : "",
               core->r[i],
               (i + 1) % 4 ? ",  " : "\n");
    }
}

int main(int argc, char ** argv)
{
    args_t args;
    core_t core;
    core_init(&core, "ARMv7-M");
    int err;
    if((err = parse_argv(argc, argv, &args)) != 0) {
        return (err > 0) ? err : 0;
    }
    if(!args.filename) {
        printf("Nothing to do!\n");
        return 1;
    }
    /* Load a binary into RAM for the core */
    FILE * fd = fopen(args.filename, "r");
    if(!fd) {
        fprintf(stderr, "error: %s: %s\n", args.filename, strerror(errno));
        return 1;
    }
    void * ptr = core.flash;
    while(!feof(fd)) {
        ptr += fread(ptr, 1, 1024, fd);
    }
    fclose(fd);
    /* Execute program */
    core_reset(&core);
    printf_coredump(&core);
    do {
        core_tick(&core);
        printf_progress(&core, 0);
    } while (!core_is_stalled(&core));
    printf_progress(&core, 1);
    printf("\n");
    printf_coredump(&core);
    return 0;
}
