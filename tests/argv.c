/* Copyright 2019, Alistair Boyle, 3-clause BSD License */
#include "config.h"

#include "argv.h" /* printf_model */
#include "cmocka.h"

static void test_happy (void ** state)
{
}

int main(int argc, char ** argv)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_happy),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
