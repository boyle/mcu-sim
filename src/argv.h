/* Copyright 2023, Alistair Boyle, 3-clause BSD License */
#pragma once
#include "config.h"

typedef struct args {
    const char * filename;
} args_t;

/* returns: 0 on success, 1 on failure, 2 on success-but-exit-now */
int parse_argv(int argc, char ** argv, args_t * args);
