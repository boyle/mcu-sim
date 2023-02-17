#include <stdint.h> // uint32_t

/* from linker script .ld */
extern uint32_t _stext;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _stack_top;

extern int main(void);

#define DUMMY __attribute__ ((weak, alias("isr_dummy")))
void isr_dummy(void);
void isr_reset(void);
DUMMY void isr_nmi(void);
DUMMY void isr_fault(void);
DUMMY void isr_svc(void);
DUMMY void isr_pend_sv(void);
DUMMY void isr_tick(void);

typedef struct {
    void * sp_main;
    void * reset;
    void * nmi;
    void * fault;
    void * rsv1[7];
    void * svc;
    void * rsv2[2];
    void * pend_sv;
    void * tick;
    void * rsv3[30];
} isr_table_t;

__attribute__ ((section(".vectors")))
const isr_table_t isr_table = {
    .sp_main                = (void *) (&_stack_top), /* from linker script */
    .reset                  = (void *) isr_reset,
    .nmi                    = (void *) isr_nmi,
    .fault                  = (void *) isr_fault,
    .svc                    = (void *) isr_svc,
    .pend_sv                = (void *) isr_pend_sv,
    .tick                   = (void *) isr_tick,
};

typedef struct {
    volatile uint32_t VTOR;
} scb_t;
scb_t * SCB = (scb_t *) 0xE000ED08;

void isr_reset(void)
{
    /* Copy the relocatable segment to ram */
    uint32_t * src = &_etext;
    uint32_t * dst = &_sdata;
    if (src != dst) {
        for (; dst < &_edata;) {
            *dst++ = *src++;
        }
    }
    /* Clear the zero (BSS) segment */
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }
    /* Set the Vector Table Offset Register,
     * see [armv7m] B1.5.3 for alignment requirements */
    SCB->VTOR = (uint32_t) &isr_table;
    /* and Run */
    main();
    while (1);
}

void isr_dummy(void)
{
    while (1);
}
