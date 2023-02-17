/* Copyright 2023 Alistair Boyle, 3-clause BSD License */
#include "config.h"

#include <stdio.h> // printf
#include <string.h> // memset
#include <assert.h> // assert
#include <getopt.h>

#include "argv.h"

int parse_argv(int argc, char ** argv, args_t * args)
{
    int err = (argc <= 1);
    assert(args != NULL);
    memset(args, 0, sizeof(args_t));
    while (1) {
        static struct option long_options[] = {
            {"help",    no_argument, 0, 'h'},
            {"version", no_argument, 0, 'V'},
            {0,         0,           0,  0 }
        };
        int c = getopt_long(argc, argv, "?hVx", long_options, NULL);
        if (c == -1) {
            break;    /* getopt is done */
        }
        switch (c) {
        case 'V':
            printf("%s\n", PACKAGE_STRING);
            return -1;
        case '?':
        case 'h':
            err = -1;
            goto _help;
        case 'x':
            return 0; // TODO dummy
        default:
            return 255;
        }
    }
    while (optind < argc) {
        if((argv[optind][0] == '-') || args->filename) {
            fprintf(stderr, "error: %s: extra option\n", argv[optind]);
            err = 1;
        }
        else  {
            args->filename = argv[optind];
        }
        optind++;
    }
    if (!err) {
        return 0;
    }
_help:
    printf("%s [options] <prog.bin>\n", PACKAGE_NAME);
    printf(" --help -h     this help\n");
    printf(" --version -V  version info\n");
    return err;
}
