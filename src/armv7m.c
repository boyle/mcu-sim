/* Copyright 2023 Alistair Boyle, 3-clause BSD License */
#include "config.h"

#include <string.h> // memset
#include <stdio.h> // printf
#include <assert.h> // assert

#include "armv7m.h"

// bit mask fields
#define U32B_MASK0(len) ((len)>=32 ? 0xffffffff : ((((uint32_t)1)<<(len))-1))
#define U32B_MASK(msb,lsb) (U32B_MASK0((msb)+1)-U32B_MASK0(lsb))
#define U32B_GET(src, msb, lsb) ( ((src) >> (lsb)) & U32B_MASK0((msb)-(lsb)+1) )
#define U32B_SET(src, msb, lsb, val) ( ((src) & ~(U32B_MASK(msb,lsb))) | (((val) & U32B_MASK0((msb)-(lsb)+1))<<(lsb)) )
#define U32B_GET1(src, lsb) U32B_GET(src, lsb, lsb)
#define U32B_SET1(src, lsb, val) U32B_SET(src, lsb, lsb, val)

#define U16B_MASK0(len) ((len)>=16 ? 0xffff : ((((uint32_t)1)<<(len))-1))
#define U16B_MASK(msb,lsb) (U16B_MASK0((msb)+1)-U16B_MASK0(lsb))
#define U16B_GET(src, msb, lsb) ( ((src) >> (lsb)) & U16B_MASK0((msb)-(lsb)+1) )
#define U16B_SET(src, msb, lsb, val) ( ((src) & ~(U16B_MASK(msb,lsb))) | (((val) & U16B_MASK0((msb)-(lsb)+1))<<(lsb)) )
#define U16B_GET1(src, lsb) U16B_GET(src, lsb, lsb)
#define U16B_SET1(src, lsb, val) U16B_SET(src, lsb, lsb, val)

// APSR.N (negative)
#define XPSR_GET_N(x)    U32B_GET1(x, 31)
#define XPSR_SET_N(x, y) U32B_SET1(x, 31, y)

// APSR.Z (all zeros)
#define XPSR_GET_Z(x)    U32B_GET1(x, 30)
#define XPSR_SET_Z(x, y) U32B_SET1(x, 30, y)

// APSR.C (carry)
#define XPSR_GET_C(x)    U32B_GET1(x, 29)
#define XPSR_SET_C(x, y) U32B_SET1(x, 29, y)

// APSR.V (overflow)
#define XPSR_GET_V(x)    U32B_GET1(x, 28)
#define XPSR_SET_V(x, y) U32B_SET1(x, 28, y)

#define XPSR_GET_Q(x)    U32B_GET1(x, 27)
#define XPSR_SET_Q(x, y) U32B_SET1(x, 27, y)

// IPSR.exception
#define XPSR_GET_EXCEP(x)    (U32B_GET(x, 8, 0)
#define XPSR_SET_EXCEP(x, y) U32B_SET(x, 8, 0, y)

// EPSR.IT[7:0]  = {EPSR[15:12],EPSR[11:10],EPSR[26:25]}
#define XPSR_GET_IT(x)    (U32B_GET(x, 15, 12)<<4 | U32B_GET(x, 11, 10)<<2 | U32B_GET(x, 26, 25))
#define XPSR_SET_IT(x, y) U32B_SET(U32B_SET(U32B_SET(x, 15, 12, y>>4), 11, 10, y>>2), 26, 25, y)
// EPSR.T
#define XPSR_GET_T(x)    U32B_GET1(x, 24)
#define XPSR_SET_T(x, y) U32B_SET1(x, 24, y)


#define CTRL_GET_NPRIV(x)    U32B_GET1(x, 0)
#define CTRL_SET_NPRIV(x, y) U32B_SET1(x, 0, y)
#define CTRL_GET_SPSEL(x)    U32B_GET1(x, 1)
#define CTRL_SET_SPSEL(x, y) U32B_SET1(x, 1, y)
#define CTRL_GET_FPCA(x)     U32B_GET1(x, 2)
#define CTRL_SET_FPCA(x, y)  U32B_SET1(x, 2, y)

#define SP 13 // SP_main
#define LR 14
#define PC 15
#define SP_PROCESS 16 // SP_process

/* Acronyms:
 *  R0 -- R12
 *  SP = Stack Pointer R13
 *  LR = Link Register R14 --> reset = 0xffffffff
 *  PC = Program Counter R15
 *  APSR = Application Program Status Register (XSPR) (x on reset)
 *    [31] N = negative
 *    [30] Z = zero flag
 *    [29] C = carry flag
 *    [28] V = overflow
 *    [27] Q = saturation
 *    [19:16] GE = greater than or equal flags (DSP extension only, otherwise reserved)
 *  IPSR = Interrupt Program Status Register (XSPR)
 *    [8:0] exception number (=0 in Thread mode, exception # in Handler mode)
 *  EPSR = Execution Program Status Register (XSPR) (0 on reset)
 *    [26:25],[15:10] ICI/IT (Interrupt Continue load/store Instructions; or IT block state restart info)
 *      IT[7:0]  = {EPSR[15:12],EPSR[11:10],EPSR[26:25]}
 *      ICI[7:0] = {EPSR[26:25],EPSR[15:12],EPSR[11:0]} where ICI[7:6]=EPSR[26:25]=0 and ICI[1:0]=EPSR[11:10]=0; ICI[5:2]=reg_num
 *    [24] T = 1: Thumb instructions; 0: Arm instructions (always 1; no interworking)
 *        T = rst_vec[bit 0] = 1 or HardFault (RetAddres=reset_handler, stacked xPSR.T=0
 *  IAPSR = IPSR and APSR
 *  EAPSR = EPSR and APSR
 *  XPSR = APSR and IPSR and EPSR
 *  IEPSR = IPSR and EPSR
 *  PRIMASK = Exception mask register (1b); 1=execution priority to 0
 *  FAULTMASK = Fault mask register (1b); 1=execution priority to -1 (HardFault)
 *  BASEPRI = Base Priority (8b); threshold for preemption; 0 = disabled
 *  CONTROL = Special purpose control register
 *     [0] nPRIV = Thread mode execution privilege; in Thread mode, 0:privileged, 1:unprivileged; Handler mode is always privileged
 *     [1] SPSEL = Stack select; 0:use SP_main, 1:Thread-mode uses SP_process,Handler-mode (reserved) (updated on exception entry/exit)
 *     [2] FPCA = FP extension included and active; 0: FP inactive, 1: FP active
 *  FPSCR = Floating-point Status and Control Register, application-level, with CP10 & CP11 access enabled, see CPACR
 *     new floating point context sets FPSCR's AHP, DN, FZ, RMode from FPDSCR (default status & control reg)
 *     [31] N
 *     [30] Z
 *     [29] C
 *     [28] V
 *     [26] AHP = alternative half-precision; 0: IEEE754-2008, 1: alternative
 *     [25] DN = default NaN; 0: propagate, 1: any NaN results in Default NaN * (non-standard)
 *     [24] FZ = flush-to-zero; 0: fully IEEE754 compliant, 1: enabled * (non-standard)
 *     [23:22] RMode = rounding mode; 0: round nearest RN, 1: round plus RP, 2: round minus RM, 3: round to zero RZ
 *     [7] IDC = input denormal cumulative exception
 *     [4] IXC = inexact cumulative exception
 *     [3] UFC = underflow cumulative exception
 *     [2] OFC = overflow cumulative exception
 *     [1] DZC = division by zero cumulative exception
 *     [0] IOC = invalid operation cumulative exception
 *      use VMRS & VMSR to transfer FPSCR to APSR flags
 *  CP0 -- CP7 = implementation defined co-processors
 *  CP8 -- CP15 = reserved
 *  CP10, CP11 are arm7m floating point
 *
 *
 *   ----
 *   TODO unpredictable outcomes
 */

#define HASH_SIZE 0x10000 // 64kB=2^16 --> all 16b instructions
typedef void (handler_t)(core_t *);
static handler_t * hash [HASH_SIZE]; // instruction jump table
static void hash_init();
static uint32_t mem_rd(core_t *, uint32_t, size_t);
static uint32_t mem_rd(core_t *, uint32_t, size_t);
static void undefined(core_t * core);
static bool is_t32(core_t * core);

// these are temporary prototypes... delete me
static void it_advance(core_t * core);
static void branch_to(core_t * core, uint32_t addr);
static void take_reset(core_t * core);

void core_init(core_t * core, const char * name)
{
    memset(core, 0, sizeof(core_t));
    core->name = name;
    hash_init();
}

void core_reset(core_t * core)
{
    take_reset(core);
}

void core_tick(core_t * core)
{
    core->last_pc = core->r[PC];
    core->opcode = mem_rd(core, core->r[PC] & ~1, 4);
    core->r[PC] += 4; // assume thumb32, then fix it after since PC = r[PC]+4
    hash[core->opcode & 0xffff](core);
    if(core->halt) {
        return;
    }
    if(core->last_pc + 4 == core->r[PC]) {
        core->r[PC] += is_t32(core) ? 0 : -2;
    }
    it_advance(core);
}

int core_is_stalled(core_t * core)
{
    return (core->r[PC] == core->last_pc) || core->halt;
}

void core_irq_set(core_t * core, int irq)
{
    core->exception_active[irq] = 1;
}

void core_irq_clr(core_t * core, int irq)
{
    core->exception_active[irq] = 0;
}

/* ---- Memory Access ----*/
#define IN_RANGE(base, size, addr, len) ((addr >= base) && (addr + len <= base + size))
static void mem_wr(core_t * core, uint32_t addr, uint8_t * src, size_t len)
{
    uint8_t * mem = NULL;
    uint32_t base = 0;
    if(IN_RANGE(FLASH_BASE, FLASH_MAX_SIZE, addr, len)) {
        mem = core->flash;
        base = FLASH_BASE;
    }
    else if(IN_RANGE(SRAM_BASE, SRAM_MAX_SIZE, addr, len)) {
        mem = core->sram;
        base = SRAM_BASE;
    }
    if(mem) {
        memcpy(&mem[addr - base], src, len);
        // uint32_t val = *(uint32_t*)src;
        // uint32_t mask = len >= 4 ? 0 : (-1) << (8 * len);
        // printf("wr[0x%08x]=0x%0*x\n", addr, (int) len * 2, val & ~mask);
    }
}

static uint32_t mem_rd(core_t * core, uint32_t addr, size_t len)
{
    assert(len <= 4);
    uint8_t * mem = NULL;
    uint32_t base = 0;
    uint32_t val = 0xffffffff;
    uint32_t mask = len >= 4 ? 0 : (-1) << (8 * len);
    if(IN_RANGE(FLASH_BASE, FLASH_MAX_SIZE, addr, len)) {
        mem = core->flash;
        base = FLASH_BASE;
    }
    else if(IN_RANGE(SRAM_BASE, SRAM_MAX_SIZE, addr, len)) {
        mem = core->sram;
        base = SRAM_BASE;
    }
    if(mem) {
        memcpy(&val, &mem[addr - base], len);
    }
    // printf("rd[0x%08x]=0x%0*x\n", addr, (int) len * 2, val & ~mask);
    return  val & (~mask);
}

/* ---- Instruction Execution ---- */
static uint32_t current_cond(core_t * core)
{
    // for T1 and T3 branch, return cond
    uint32_t it = XPSR_GET_IT(core->epsr);
    if(U32B_GET(it, 3, 0) != 0) {
        return U32B_GET(it, 7, 4);
    }
    else {
        return 14;    // 1110 = None (unconditional)
    }
}

static bool condition_passed(core_t * core)
{
    uint32_t apsr = core->apsr;
    uint32_t cond = current_cond(core);
    bool ret;
    switch(cond >> 1) {
    case 0: ret = XPSR_GET_Z(apsr); break;
    case 1: ret = XPSR_GET_C(apsr); break;
    case 2: ret = XPSR_GET_N(apsr); break;
    case 3: ret = XPSR_GET_V(apsr); break;
    case 4: ret = XPSR_GET_C(apsr) && !XPSR_GET_Z(apsr); break;
    case 5: ret = XPSR_GET_N(apsr) == XPSR_GET_V(apsr); break;
    case 6: ret = XPSR_GET_N(apsr) == XPSR_GET_V(apsr) && XPSR_GET_Z(apsr); break;
    case 7: ret = 1; break;
    }
    if((cond & 1) && (cond != 15)) {
        ret = !ret;    // invert result, if necessary
    }
    return ret;
}

static void it_advance(core_t * core)
{
    uint32_t it = XPSR_GET_IT(core->epsr);
    if((it & 7) == 0) {
        it = 0;
    }
    else {
        it = U32B_SET(it, 4, 0, it << 1);
    }
    core->epsr = XPSR_SET_IT(core->epsr, it);
}

static bool in_it_block(core_t * core)
{
    return (XPSR_GET_IT(core->epsr) & 0xf) != 0;
}

static void take_reset(core_t * core)
{
    // see ARMv7-M "TakeReset"
    core->handler_mode = 0; // thread mode
    core->primask = 0;
    core->faultmask = 0;
    core->basepri = 0;
    // TODO if(have_fp_ext()) ...
    if(0) {
    }
    else {
        core->control &= ~0x3; // main stack, privileged thread
    }
    for(int i = 0; i < 512; i++) {
        core->exception_active[i] = false;
    }
    // TODO reset_scs_regs(); // system control space reset
    // TODO clear_exclusive_local(processor_id()); // LDREX*/STREX* synchronization monitoring support
    // TODO clear_event_register() // see WFE
    for(int i = 0; i < 13; i++) {
        core->r[i] = 0xdeadbeef; // unknown
    }
    uint32_t vectortable = 0; // TODO = {VTOR[31:7],7'b0};
    core->r[SP] = mem_rd(core, vectortable, 4) & (~0x3);
    core->r[SP_PROCESS] = 0xdeadbeef & (~0x3); // unknown
    core->r[LR] = 0xffffffff; // illegal exception return value
    uint32_t tmp = mem_rd(core, vectortable + 4, 4);
    uint32_t tbit = tmp & 1;
    core->apsr = 0xdeadbeef; // unknown
    core->ipsr = XPSR_SET_EXCEP(core->ipsr, 0);
    core->epsr = XPSR_SET_T(core->epsr, tbit);
    core->epsr = XPSR_SET_IT(core->epsr, 0);
    branch_to(core, tmp & 0xfffffffe);
}

/* TODO unused function!!
static bool last_in_it_block(core_t * core) {
    return (core->it & 15) == 8;
}
*/

/* Reference: ARMv7-M Architecture Reference Manual [armv7m], version E.e
 * General Notes:
 *  - Armv7E-M --> 'E' = DSP extensions; Single Instruction Multiple Data
 *                       (SIMD) saturating & unsigned
 *  - FPv4-SP --> single-precision VFPv4-D16 extension
 *                for Armv7-M: FPv4-SP-D16-M (single-precision)
 *  - FPv5    --> double-precision
 *                for Armv7-M: FPv5-SP-D16-M (single-precision), or
 *                             FPv5-D16-M (single- & double-precision)
 *  - Threaded mode: default at reset
 *    - unprivileged
 *    - privileged --> allows SVC instruction (SuperVisor Call) which causes a
 *                     SVCall exception into Handler mode
 *  - Handler mode: all exceptions are handled (due to an instruction,
 *    interrupt, mmu violation, alignment/bus fault, or debug event)
 *  - Data types: 32b pointers, uint32/int32, uint16/int16 & uint8/int8 (zero-extended or sign-extended)
 *  - Can load/store 64b, 32b, 16, 8b; load can zero- or sign-extend
 *  - Integer signed = two's compliment; unsigned = normal binary.
 *  - Most 64b ops are synthesis of two+ ops
 *  - The initial PC at reset is the reset handler.
 *  - Required components are:
 *    - The core clock is SysTick.
 *    - The deferred Supervisor Call is PendSV, see ICSR.PENDSVSET bit, and SVC instruction.
 *    - External interrupt controller, the Nested Vectored Interrupt Controller (NVIC)
 *    - Debug event BKPT instruction.
 *    - Send Event SEV and Wait For Event WFE
 *    - Wait For Interrupt WFI
 *  - Optional floating-point extension
 */

// TODO pA2-37 FPSCR...

// instruction fields
// TODO
#define IMM4(x)    U16B_GET(x,3,0)
#define IMM7(x)    U16B_GET(x,6,0)
#define IMM11(x)   U16B_GET(x,10,0)
#define IMM16(x)  (U16B_GET(x,3,0)<<12 + U16B_GET(x>>16,11,0))
#define EXTRA(x)   U16B_GET1(x,8)
#define COND(x)    U16B_GET(x,11,8)

#define M_IMM4(x) U16B_GET(x>>16,3,0)
#define M_IMM1(x) U16B_GET1(x,10)
#define M_RB(x)   U16B_GET(x>>16,7,4)
#define M_RA(x)   U16B_GET(x>>16,3,0)

// used
#define IMM3(x)      U16B_GET(x,8,6)
#define R2(x)        U16B_GET(x,8,6)
#define R1(x)        U16B_GET(x,5,3)
#define R0(x)        U16B_GET(x,2,0)
#define R1_4(x)      U16B_GET(x,6,3)
#define R0_4(x)    ((U16B_GET1(x,7)<<3) + U16B_GET(x,2,0))
#define IMM5(x)      U16B_GET(x,10,6)
#define IMM8(x)      U16B_GET(x,7,0)
#define R_IMM6(x)  ((U16B_GET1(x,9)<<5)+U16B_GET(x,7,3))
#define R_IMM8(x)    U16B_GET(x,10,8)
#define OP0(x)       U16B_GET(x,3,0)
#define OP1(x)       U16B_GET(x,7,4)

#define M_RC(x)        U16B_GET(x>>16,11,8)
#define M_R0(x)        U16B_GET(x,3,0)
#define M_IMM3(x)      U16B_GET(x>>16,14,12)
#define M_IMM8(x)      U16B_GET(x>>16,7,0)
#define M_IMM10(x)     U16B_GET(x,9,0)
#define M_IMM11(x)     U16B_GET(x>>16,10,0)
#define M_I(x)         U16B_GET1(x,10)
#define M_S(x)         U16B_GET1(x,4)
#define M_J1(x)        U16B_GET1(x>>16,13)
#define M_J0(x)        U16B_GET1(x>>16,11)

/* ---- Operations ---- */

#define SIGN_EXTEND(x,n) ((x) & ((-1)<<(8*(n)))) | ((((x)>>(8*(n)-1))&1) ? (-1)<<(8*(n)) : 0)

static void assign_flags(core_t * core, uint32_t result, bool carry, bool overflow)
{
    result &= 0xffffffff;
    core->apsr = XPSR_SET_N(core->apsr, result >> 31);
    core->apsr = XPSR_SET_Z(core->apsr, result == 0);
    core->apsr = XPSR_SET_C(core->apsr, carry);
    core->apsr = XPSR_SET_V(core->apsr, overflow);
}

/*
static void get_flags(core_t * core, bool * carry, bool * overflow)
{
    *carry = XPSR_GET_C(core->apsr);
    *overflow = XPSR_GET_V(core->apsr);
}
*/

static uint32_t add_with_carry(uint32_t a, uint32_t b, bool * carry, bool * overflow)
{
    uint64_t usum = (uint64_t)a + (uint64_t)b + (uint64_t)(*carry ? 1 : 0);
    int64_t  ssum = (int64_t)a + (int64_t)b + (int64_t)(*carry ? 1 : 0);
    uint32_t result = usum & 0xffffffff;
    *carry = ((uint64_t)result != usum);
    *overflow = ((int64_t)result != ssum);
    return result;
}

static int decode_imm_shift(int * shift_t, int imm5)
{
    int shift;
    switch(*shift_t) {
    case 0: shift = imm5; break; // LSL
    case 1: shift = (imm5 == 0) ? 32 : imm5; break; // LSR
    case 2: shift = (imm5 == 0) ? 32 : imm5; break; // ASR
    case 3: shift = (imm5 == 0) ? 1 : imm5; break; // (imm5==0)? RRX : ROR
    }
    if((*shift_t == 3) && (imm5 == 0)) {
        (*shift_t)++;    // RRX
    }
    return shift;
}

static uint32_t shift_c(uint32_t val, int shift_t, unsigned int amt, bool * carry)
{
    if(shift_t == 4) {
        assert(amt == 1);    // RRX --> amt=1
    }
    if(amt == 0) {
        return val;
    }
    int64_t ival = val;
    int m = amt % 32;
    bool carry_in = *carry;
    if(val & (1 << 31)) {
        ival = ~((int64_t)val + 1) + 1;
    }
    switch(shift_t) {
    case 0: *carry = (val >> (32 - amt)) & 1; val <<= amt; break; // LSL
    case 1: *carry = (val >> (amt - 1)) & 1; val >>= amt; break; // LSR
    case 2: *carry = (ival >> (amt - 1)) & 1; val = ival >> amt; break; // ASR
    case 3: *carry = (val >> (m - 1)) & 1; val = (val >> m) | (val << (32 - m)); break; // ROR
    case 4: *carry = val & 1; val = ((carry_in ? 1 : 0) << 31) | (val >> 1); break; // RRX
    }
    return val;
}

static uint32_t shift(uint32_t val, int shift_t, unsigned int amt, bool carry)
{
    return shift_c(val, shift_t, amt, &carry); // carry_out is discarded
}

static void branch_to(core_t * core, uint32_t addr)
{
    core->r[PC] = (addr & 0xfffffffe); // half-word aligned
}

static void branch_write_pc(core_t * core, uint32_t addr)
{
    branch_to(core, addr);
}

static void bx_write_pc(core_t * core, uint32_t addr)
{
// TODO    if(core->current_mode == handler && ((addr>>28) & 0xf)) {
// TODO        excetion_return(addr & 0x0fffffff);
// TODO    }
// TODO    else {
    core->epsr = XPSR_SET_T(core->epsr, addr & 1);
    // TODO if EPSR.T==0, a UsageFault (Invalid State) on next instruction
    branch_to(core, addr & 0xfffffffe);
// TODO    }
}

static void blx_write_pc(core_t * core, uint32_t addr)
{
    core->epsr = XPSR_SET_T(core->epsr, addr & 1);
    // TODO if EPSR.T==0, a UsageFault (Invalid State) on next instruction
    branch_to(core, addr & 0xfffffffe);
}

static void load_write_pc(core_t * core, uint32_t addr)
{
    bx_write_pc(core, addr);
}

static void alu_write_pc(core_t * core, uint32_t addr)
{
    branch_write_pc(core, addr);
}

static int bit_count(uint32_t x)
{
    /* Brian Kernighan's method goes through as many iterations as there are set
     * bits. So if we have a 32-bit word with only the high bit set, then it will
     * only go once through the loop. See Peter Wegner, CACM 3 (1960), 322. */
    int c;
    for (c = 0; x; c++) {
        x &= x - 1;    // clear the least significant bit set
    }
    return c;
}

static void bkpt_instr_debug_event(core_t * core)
{
    assert(0); // TODO generate a debug event
}

static void hint_yield(core_t * core)
{
    assert(0); // TODO
}

static bool event_regsitered(const core_t * core)
{
    return false;
}

static void clear_event_register(core_t * core)
{
    assert(0); // TODO
}

static void wait_for_event(core_t * core)
{
    assert(0); // TODO
}

static void wait_for_interrupt(core_t * core)
{
    assert(0); // TODO
}

static void hint_send_event(core_t * core)
{
    assert(0); // TODO
}

static void call_supervisor(core_t * core)
{
    assert(0); // TODO
}

static bool is_t32(core_t * core)
{
    return ((core->opcode & 0xf800) == 0xe800) ||
           ((core->opcode & 0xf000) == 0xf000);
}

static void undefined(core_t * core)
{
    char bin[2][17];
    for(int i = 0; i < 16; i++) {
        bin[0][i] = ((core->opcode >> (15 - i)) & 1) + '0';
        bin[1][i] = ((core->opcode >> (31 - i)) & 1) + '0';
    }
    bin[0][16] = '\0';
    bin[1][16] = '\0';
    if(is_t32(core)) {
        printf("undefined op=0x%04x %04x (b%s %s) @ 0x%04x\n",
               core->opcode & 0xffff, core->opcode >> 16,
               bin[0], bin[1],
               core->r[PC]);
    }
    else {
        printf("undefined op=0x%04x (b%s) @ 0x%04x\n", core->opcode & 0xffff, bin[0], core->r[PC]);
    }
    core->halt = true;
}

/* ==== Instructions ==== [ARMv7-M] A5.0 */
/* Prototypes are provided for some instructions; those used by other
 * instructions before they are defined. Not all instructions require
 * prototypes. */
static void mov_reg(core_t * core);

/* -- Shift (immediate), add, subtract, move, compare -- [ARMv7-M] A5.2.1 */

/*
static void adc(core_t * core)   // ADC (immediate); add with carry
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d = M_RC(x);
    int n = M_R0(x);
    bool set_flags = M_S(x);
    int imm = (M_IMM1(x) << 11) + (M_IMM3(x) << 8) + (M_IMM8(x));
    bool carry = XPSR_GET_C(core->apsr);
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[n], imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}
*/

static void add_reg(core_t * core)   // ADD (register)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d;
    bool set_flags;
    int shift_t = 0;
    int shift_n = 0;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 14)) { // T1
            d = R0(x);
            n = R1(x);
            m = R2(x);
            set_flags = !in_it_block(core);
        }
        else { // T2
            d = n = R0_4(x);
            m = R1_4(x);
            set_flags = false;
            // TODO if(n==SP || m==SP) { add_sp_reg(core); return; }
        }
    }
    else { // T3
        assert(0);
    }
    bool carry = false;
    bool overflow = false;
    int shifted = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t result = add_with_carry(core->r[n], shifted, &carry, &overflow);
    if(d == PC) {
        alu_write_pc(core, result);
    }
    else {
        core->r[d] = result;
        if(set_flags) {
            assign_flags(core, result, carry, overflow);
        }
    }
}

static void add(core_t * core)   // ADD (immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, n, d;
    bool set_flags;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 13)) { // T1
            d = R0(x);
            n = R1(x);
            imm = IMM3(x);
            set_flags = !in_it_block(core);
        }
        else { // T2
            d = n = R_IMM8(x);
            imm = IMM8(x);
            set_flags = false;
        }
    }
    else { // T3
        assert(0);
    }
    bool carry = false;
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[n], imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void sub(core_t * core)   // SUB (immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, n, d;
    bool set_flags;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 13)) { // T1
            d = R0(x);
            n = R1(x);
            imm = IMM3(x);
            set_flags = !in_it_block(core);
        }
        else { // T2
            d = n = R_IMM8(x);
            imm = IMM8(x);
            set_flags = false;
        }
    }
    else { // T3
        assert(0);
    }
    bool carry = true;
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[n], ~imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void sub_reg(core_t * core)   // SUB (register)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d;
    bool set_flags;
    int shift_t = 0;
    int shift_n = 0;
    if(!U16B_GET1(x, 15)) {
        d = R0(x);
        n = R1(x);
        m = R2(x);
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = true;
    bool overflow = false;
    int shifted = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t result = add_with_carry(core->r[n], ~shifted, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void mov(core_t * core)   // MOV (immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, d;
    bool set_flags, carry;
    if(!U16B_GET1(x, 15)) {
        d = R_IMM8(x);
        imm = IMM8(x);
        set_flags = !in_it_block(core);
        carry = XPSR_GET_C(core->apsr);
    }
    else { // T3
        assert(0);
    }
    uint32_t result = imm;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void cmp(core_t * core)   // MOV (immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, n;
    if(!U16B_GET1(x, 15)) { // T1
        n = R_IMM8(x);
        imm = IMM8(x);
    }
    else { // T2
        assert(0);
    }
    bool carry = true;
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[n], ~imm, &carry, &overflow);
    assign_flags(core, result, carry, overflow);
}

static void lsl(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, d, imm, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        d = R0(x);
        m = R1(x);
        imm = IMM5(x);
        shift_n = decode_imm_shift(&shift_t, imm);
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    if(imm == 0) {
        mov_reg(core);
        return;
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t result = shift_c(core->r[m], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void lsr(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, d, shift_n;
    bool set_flags;
    int shift_t = 1;
    if(!U16B_GET1(x, 15)) { // T1
        d = R0(x);
        m = R1(x);
        shift_n = decode_imm_shift(&shift_t, IMM5(x));
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t result = shift_c(core->r[m], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void asr(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, d, shift_n;
    bool set_flags;
    int shift_t = 2;
    if(!U16B_GET1(x, 15)) { // T1
        d = R0(x);
        m = R1(x);
        shift_n = decode_imm_shift(&shift_t, IMM5(x));
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t result = shift_c(core->r[m], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

/* -- Data processing -- [ARMv7-M] A5.2.2 */

static void and_reg(core_t * core) // AND (register); bitwise
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = core->r[n] & shifted;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void eor_reg(core_t * core) // EOR (register); bitwise-xor
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = core->r[n] ^ shifted;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void lsl_reg(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    shift_n = core->r[m] & 0xff;
    uint32_t result = shift_c(core->r[n], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void lsr_reg(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 1;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    shift_n = core->r[m] & 0xff;
    uint32_t result = shift_c(core->r[n], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void asr_reg(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 2;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    shift_n = core->r[m] & 0xff;
    uint32_t result = shift_c(core->r[n], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void adc_reg(core_t * core)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    bool overflow = false;
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = add_with_carry(core->r[n], shifted, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void sbc_reg(core_t * core) // SDC (register); subtract with carry
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    /* Note: [ARMV7-M] A7.7.125 says "NOT(Carry flag) in the text, but the
     * pseudocode says to use ASPR.C. We know that subtraction should use the
     * negated carry flag, so we'll take that interpretation. */
    bool carry = ! XPSR_GET_C(core->apsr);
    bool overflow = false;
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = add_with_carry(core->r[n], ~shifted, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void ror_reg(core_t * core) // ROR (register); rotate right
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 3;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    shift_n = core->r[m] & 0xff;
    uint32_t result = shift_c(core->r[n], shift_t, shift_n, &carry);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void tst_reg(core_t * core) // TST (register); test
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, shift_n;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = R0(x);
        m = R1(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = core->r[n] & shifted;
    bool overflow = XPSR_GET_V(core->apsr); // unchanged
    assign_flags(core, result, carry, overflow);
}

static void rsb(core_t * core) // RSB (immediate); reverse subtract
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int n, d, imm;
    bool set_flags;
    if(!U16B_GET1(x, 15)) { // T1
        d = R0(x);
        n = R1(x);
        imm = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = true;
    bool overflow = false;
    uint32_t result = add_with_carry(~(core->r[n]), imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void cmp_reg(core_t * core) // CMP (register); compare
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n;
    int shift_t = 0;
    int shift_n = 0;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 10)) { // T1
            n = R0(x);
            m = R1(x);
        }
        else {
            n = R0_4(x);
            m = R1_4(x);
        }
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    bool overflow = false;
    uint32_t shifted = shift(core->r[m], shift_t, shift_n, carry);
    carry = true;
    uint32_t result = add_with_carry(core->r[n], ~(shifted), &carry, &overflow);
    assign_flags(core, result, carry, overflow);
}

static void cmn_reg(core_t * core) // CMN (register); compare negative
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n;
    int shift_t = 0;
    int shift_n = 0;
    if(!U16B_GET1(x, 15)) {
        n = R0(x);
        m = R1(x);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    bool overflow = false;
    uint32_t shifted = shift(core->r[m], shift_t, shift_n, carry);
    carry = false;
    uint32_t result = add_with_carry(core->r[n], shifted, &carry, &overflow);
    assign_flags(core, result, carry, overflow);
}

static void orr_reg(core_t * core) // ORR (register); bitwise
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = core->r[n] | shifted;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void mul(core_t * core) // MUL; multiply
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d;
    bool set_flags;
    if(!U16B_GET1(x, 15)) { // T1
        m = d = R0(x);
        n = R1(x);
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    uint32_t result = core->r[n] * core->r[m]; // slow on many MCUs...
    core->r[d] = result;
    if(set_flags) {
        bool carry = XPSR_GET_C(core->apsr); // unchanged
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void bic_reg(core_t * core) // BIC (register); bitwise clear
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, n, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        n = d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = core->r[n] & (~shifted);
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

static void mvn_reg(core_t * core) // MVN (register); bitwise not
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, d, shift_n;
    bool set_flags;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        d = R0(x);
        m = R1(x);
        shift_n = 0;
        set_flags = !in_it_block(core);
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t shifted = shift_c(core->r[m], shift_t, shift_n, &carry);
    uint32_t result = ~shifted;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

/* -- Special data instructions and branch and exchange -- [ARMv7-M] A5.2.3 */

static void mov_reg(core_t * core) // MOV (register)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m, d;
    bool set_flags;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 10)) { // T1
            d = R0_4(x);
            m = R1_4(x);
            set_flags = false;
        }
        else { // T2
            d = R0(x);
            m = R1(x);
            set_flags = true;
        }
    }
    else { // T3
        assert(0);
    }
    uint32_t result = core->r[m];
    if(d == 15) {
        alu_write_pc(core, result);
    }
    else {
        core->r[d] = result;
        if(set_flags) {
            bool carry = XPSR_GET_C(core->apsr); // unchanged
            bool overflow = XPSR_GET_V(core->apsr); // unchanged
            assign_flags(core, result, carry, overflow);
        }
    }
}

static void bx(core_t * core) // BX; branch and exchange
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m = R1(x);
    bx_write_pc(core, core->r[m]);
    // TODO UsageFault
}

static void blx_reg(core_t * core) // BLX (register); branch with link and exchange
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int m = R1(x);
    uint32_t target = core->r[m];
    uint32_t next_instr_addr = core->r[PC] - 2;
    core->r[LR] = (next_instr_addr & 0xfffffffe) | 1;
    blx_write_pc(core, target);
    // TODO UsageFault
}

static void ldr_lit(core_t * core) // LDR (literal); load from literal pool
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, imm;
    bool add = true;
    if(!U16B_GET1(x, 15)) { // T1
        t = R_IMM8(x);
        imm = IMM8(x) << 2;
    }
    else { // T2
        assert(0);
    }
    uint32_t base = core->r[PC] & ~3; // = Align(PC,4)
    uint32_t address = add ? (base + imm) : (base - imm);
    uint32_t data = mem_rd(core, address, 4);
    if(t == PC) {
        load_write_pc(core, data);
    }
    else {
        core->r[t] = data;
    }
    // TODO UsageFault, MemManage, BusFault
}

/* -- Load/store single data item -- [ARMv7-M] A5.2.4 */

static void str_reg(core_t * core) // STR (register); store 32b register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t address = core->r[n] + offset;
    mem_wr(core, address, (uint8_t *) &core->r[t], 4);
    // TODO UsageFault, MemManage, BusFault
}

static void strh_reg(core_t * core) // STRH (register); store halfword (16b) register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t address = core->r[n] + offset;
    mem_wr(core, address, (uint8_t *) &core->r[t], 2);
    // TODO UsageFault, MemManage, BusFault
}

static void strb_reg(core_t * core) // STRB (register); store byte register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t address = core->r[n] + offset;
    mem_wr(core, address, (uint8_t *) &core->r[t], 1);
    // TODO UsageFault, MemManage, BusFault
}

static void ldr_reg(core_t * core) // LDR (register); load 32b register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    bool index = true;
    bool add = true;
    bool wback = true;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t offset_addr = add ? (core->r[n] + offset) : (core->r[n] - offset);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 4);
    if(wback) {
        core->r[n] = offset_addr;
    }
    if(t == PC) {
        load_write_pc(core, data);
    }
    else {
        core->r[t] = data;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void ldrh_reg(core_t * core) // LDRH (register); load halfword (16b) register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    bool index = true;
    bool add = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t offset_addr = add ? (core->r[n] + offset) : (core->r[n] - offset);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 2);
    if(wback) {
        core->r[n] = offset_addr;
    }
    core->r[t] = data;
    // TODO UsageFault, MemManage, BusFault
}

static void ldrb_reg(core_t * core) // LDRB (register); load byte register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    bool index = true;
    bool add = true;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t offset_addr = add ? (core->r[n] + offset) : (core->r[n] - offset);
    uint32_t address = index ? offset_addr : core->r[n];
    core->r[t] = mem_rd(core, address, 1);
    // TODO UsageFault, MemManage, BusFault
}

static void ldrsh_reg(core_t * core) // LDRSH (register); load signed halfword (16b) register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t offset_addr = add ? (core->r[n] + offset) : (core->r[n] - offset);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 2);
    if(wback) {
        core->r[n] = offset_addr;
    }
    core->r[t] = SIGN_EXTEND(data, 2);
    // TODO UsageFault, MemManage, BusFault
}

static void ldrsb_reg(core_t * core) // LDRSB (register); load signed byte register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, m, shift_n;
    int shift_t = 0;
    bool add = true;
    bool index = true;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        m = R2(x);
        shift_n = 0;
    }
    else { // T2
        assert(0);
    }
    bool carry = XPSR_GET_C(core->apsr);
    uint32_t offset = shift(core->r[m], shift_t, shift_n, carry);
    uint32_t offset_addr = add ? (core->r[n] + offset) : (core->r[n] - offset);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 1);
    core->r[t] = SIGN_EXTEND(data, 1);
    // TODO UsageFault, MemManage, BusFault
}

static void stm(core_t * core) // STM, STMIA, STMEA: store multiple
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int n, registers;
    bool wback = false;
    if(!U16B_GET1(x, 13)) {
        n = R_IMM8(x);
        registers = IMM8(x);
    }
    else { // T2
        assert(0);
    }
    int bc = bit_count(registers);
    uint32_t address = core->r[n];
    for(int i = 0; i < 15; i++) {
        if(registers & (1 << i)) {
            mem_wr(core, address, (uint8_t *) &core->r[i], 4);
            address += 4;
        }
    }
    if(wback) {
        core->r[n] += 4 * bc;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void ldm(core_t * core) // LDM, LDMIA, LDMFD; load multiple
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int n, registers;
    bool wback;
    if(!U16B_GET1(x, 13)) {
        n = R_IMM8(x);
        registers = IMM8(x);
        wback = ((registers & (1 << n)) == 0);
    }
    else { // T2
        assert(0);
    }
    int bc = bit_count(registers);
    uint32_t address = core->r[n];
    for(int i = 0; i < 15; i++) {
        if(registers & (1 << i)) {
            core->r[i] = mem_rd(core, address, 4);
            address += 4;
        }
    }
    if(registers & (1 << PC)) {
        load_write_pc(core, mem_rd(core, address, 4));
    }
    if(wback && ((registers & (1 << n)) == 0)) {
        core->r[n] += 4 * bc;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void str(core_t * core) // STR (immediate); store 32b register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 11)) {
        if(!U16B_GET1(x, 15)) { // T1
            t = R0(x);
            n = R1(x);
            imm = IMM5(x) << 2;
        }
        else { // T2 --> Store Register SP relative
            t = R_IMM8(x);
            n = SP;
            imm = IMM8(x) << 2;
        }
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    mem_wr(core, address, (uint8_t *) &core->r[t], 4);
    if(wback) {
        core->r[n] = offset_addr;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void ldr(core_t * core) // LDR (immediate); store 32b register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(U16B_GET(x, 13, 12) != 3) {
        if(!U16B_GET1(x, 15)) { // T1
            t = R0(x);
            n = R1(x);
            imm = IMM5(x) << 2;
        }
        else { // T2
            t = R_IMM8(x);
            n = SP;
            imm = IMM8(x) << 2;
        }
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 4);
    if(wback) {
        core->r[n] = offset_addr;
    }
    if(t == PC) {
        load_write_pc(core, data);
    }
    else {
        core->r[t] = data;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void strh(core_t * core) // STRH (immediate); store halfword (16b) register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        imm = IMM5(x) << 1;
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    mem_wr(core, address, (uint8_t *) &core->r[t], 2);
    if(wback) {
        core->r[n] = offset_addr;
    }
    // TODO UsageFault, MemManage, BusFault
}

static void strb(core_t * core) // STRB (immediate); store byte register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        imm = IMM5(x);
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    mem_wr(core, address, (uint8_t *) &core->r[t], 1);
    if(wback) {
        core->r[n] = offset_addr;
    }
    // TODO MemManage, BusFault
}

static void ldrh(core_t * core) // LDRH (immediate); load halfword (16b) register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        imm = IMM5(x);
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    uint32_t data = mem_rd(core, address, 2);
    if(wback) {
        core->r[n] = offset_addr;
    }
    core->r[t] = data;
    // TODO UsageFault, MemManage, BusFault
}

static void ldrb(core_t * core) // LDRB (immediate); load byte register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int t, n, imm;
    bool add = true;
    bool index = true;
    bool wback = false;
    if(!U16B_GET1(x, 15)) { // T1
        t = R0(x);
        n = R1(x);
        imm = IMM5(x);
    }
    else { // T2
        assert(0);
    }
    uint32_t offset_addr = add ? (core->r[n] + imm) : (core->r[n] - imm);
    uint32_t address = index ? offset_addr : core->r[n];
    core->r[t] = mem_rd(core, address, 1);
    if(wback) {
        core->r[n] = offset_addr;
    }
    // TODO UsageFault, MemManage, BusFault
}

/* -- Miscellaneous 16-bit instructions -- [ARMv7-M] A5.2.5 */

static bool current_mode_is_privileged(const core_t * core)   // TODO
{
    return core->handler_mode || (CTRL_GET_NPRIV(core->control) == 0);
}

static uint8_t execution_priority(const core_t * core)
{
    int highestpri = 256; // thread mode, no active exceptions
    int boostedpri = 256; // after adjusting for basepri, primask, faultmask
    uint32_t subgroupshift = 0; // TODO = AIRCR.PRIGROUP
    uint32_t subgroupvalue;
    uint32_t groupvalue = 2 << subgroupshift;
    for(int i = 2; i < 511; i++) {
        if(core->exception_active[i] &&
           (core->exception_priority[i] < highestpri)) {
            highestpri = core->exception_priority[i];
            subgroupvalue = highestpri % groupvalue;
            highestpri -= subgroupvalue;
        }
    }
    if(core->basepri != 0) {
        boostedpri = core->basepri & 0xff;
        subgroupvalue = boostedpri  - subgroupvalue;
    }
    if(core->primask) {
        boostedpri = 0;
    }
    if(core->faultmask) {
        boostedpri = -1;
    }
    int priority;
    if(boostedpri < highestpri) {
        priority = boostedpri;
    }
    else {
        priority = highestpri;
    }
    return priority;
}

static void cps(core_t * core)   // CPS; change processor state
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int f = U16B_GET1(x, 0);
    int i = U16B_GET1(x, 1);
    int im = U16B_GET1(x, 4);
    bool enable = (im == 0);
    bool affect_pri = (i == 0);
    bool affect_fault = (f == 1);
    if(current_mode_is_privileged(core)) {
        if(enable) {
            if(affect_pri) {
                core->primask = false;
            }
            if(affect_fault) {
                core->faultmask = false;
            }
        }
        else {
            if(affect_pri) {
                core->primask = true;
            }
            if(affect_fault && execution_priority(core) > -1) {
                core->faultmask = true;
            }
        }
    }
}

static void adr(core_t * core)   // ADR; address to register
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, d;
    bool add = true;
    if(!U16B_GET1(x, 14)) {
        d = R_IMM8(x);
        imm = IMM8(x) << 2;
    }
    else { // T2, T3
        assert(0);
    }
    uint32_t aligned_pc = core->r[PC] & ~3;
    uint32_t result = add ? (aligned_pc + imm) : (aligned_pc - imm);
    core->r[d] = result;
}

static void add_sp(core_t * core)   // ADD (SP plus immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, d;
    bool set_flags = false;
    if(!U16B_GET1(x, 15)) {
        if(!U16B_GET1(x, 14)) { // T1
            d = R_IMM8(x);
            imm = IMM8(x) << 2;
        }
        else { // T2
            d = SP;
            imm = IMM7(x) << 2;
            set_flags = false;
        }
    }
    else { // T3
        assert(0);
    }
    bool carry = false;
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[SP], imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void sub_sp(core_t * core)   // SUB (SP minus immediate)
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int imm, d;
    bool set_flags = false;
    if(!U16B_GET1(x, 14)) { // T1
        d = SP;
        imm = IMM7(x) << 2;
        set_flags = false;
    }
    else { // T3
        assert(0);
    }
    bool carry = true;
    bool overflow = false;
    uint32_t result = add_with_carry(core->r[SP], ~imm, &carry, &overflow);
    core->r[d] = result;
    if(set_flags) {
        assign_flags(core, result, carry, overflow);
    }
}

static void cbnz_cbz(core_t * core)   // CBNZ, CBZ; compare and branch on (non-)zero
{
    uint32_t x = core->opcode;
    bool nonzero = U16B_GET1(x, 11);
    uint32_t imm = R_IMM6(x) << 1;
    int n = R0(x);
    if(nonzero != (core->r[n] == 0)) {
        branch_write_pc(core, core->r[PC] + imm);
    }
}

static void sxth(core_t * core)   // SXTH; sign extend halfword
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m, rotation;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
        rotation = 0;
    }
    else { // T3
        assert(0);
    }
    uint32_t rotated = shift(core->r[m], 3, rotation, 0); // ROR
    core->r[d] = ((rotated & 0x8000) ? 0xffff0000 : 0) |  (rotated & 0xffff); // sign extend halfword
}

static void sxtb(core_t * core)   // SXTB; sign extend byte
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m, rotation;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
        rotation = 0;
    }
    else { // T3
        assert(0);
    }
    uint32_t rotated = shift(core->r[m], 3, rotation, 0); // ROR
    core->r[d] = ((rotated & 0x80) ? 0xffffff00 : 0) |  (rotated & 0xff); // sign extend byte
}

static void uxth(core_t * core)   // UXTH; unsigned extend halfword
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m, rotation;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
        rotation = 0;
    }
    else { // T3
        assert(0);
    }
    uint32_t rotated = shift(core->r[m], 3, rotation, 0); // ROR
    core->r[d] = rotated & 0xffff;
}

static void uxtb(core_t * core)   // UXTB; unsigned extend byte
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m, rotation;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
        rotation = 0;
    }
    else { // T3
        assert(0);
    }
    uint32_t rotated = shift(core->r[m], 3, rotation, 0); // ROR
    core->r[d] = rotated & 0xff;
}

static void push(core_t * core)   // PUSH; push multiple registers
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    uint32_t registers;
    if(!U16B_GET1(x, 14)) { // T1
        registers = (U16B_GET1(x, 8) << LR) | U16B_GET(x, 7, 0);
    }
    else { // T3
        assert(0);
    }
    int bc = bit_count(registers);
    uint32_t address = core->r[SP] - 4 * bc;
    for(int i = 0; i < 15; i++) {
        if(registers & (1 << i)) {
            mem_wr(core, address, (uint8_t *) &core->r[i], 4);
            address += 4;
        }
    }
    core->r[SP] -= 4 * bc;
    // TODO UsageFault, MemManage, BusFault
}

static void pop(core_t * core)   // POP; pop multiple registers
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    uint32_t registers;
    if(!U16B_GET1(x, 14)) { // T1
        registers = (U16B_GET1(x, 8) << PC) | U16B_GET(x, 7, 0);
    }
    else { // T3
        assert(0);
    }
    int bc = bit_count(registers);
    uint32_t address = core->r[SP];
    core->r[SP] += 4 * bc;
    for(int i = 0; i < 15; i++) {
        if(registers & (1 << i)) {
            core->r[i] = mem_rd(core, address, 4);
            address += 4;
        }
    }
    if(registers & (1 << PC)) {
        load_write_pc(core, mem_rd(core, address, 4));
    }
    // TODO UsageFault, MemManage, BusFault
}

static void rev(core_t * core)   // REV; byte-reverse word
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
    }
    else { // T3
        assert(0);
    }
    uint32_t rm = core->r[m];
    uint32_t result = ((rm & 0xff) << 24) | ((rm & 0xff00) << 8) |
                      ((rm & 0xff0000) >> 8) | ((rm & 0xff000000) >> 24);
    core->r[d] = result;
}

static void rev16(core_t * core)   // REV16; byte-reverse packed halfword
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
    }
    else { // T3
        assert(0);
    }
    uint32_t rm = core->r[m];
    uint32_t result = ((rm & 0xff) << 8) | ((rm & 0xff00) >> 8) |
                      ((rm & 0xff0000) << 8) | ((rm & 0xff000000) >> 8);
    core->r[d] = result;
}

static void revsh(core_t * core)   // REVSH; byte-reverse signed halfword
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d, m;
    if(!U16B_GET1(x, 14)) { // T1
        d = R0(x);
        m = R1(x);
    }
    else { // T3
        assert(0);
    }
    uint32_t rm = core->r[m];
    uint32_t result = ((rm & 0xff) << 8) | ((rm & 0xff00) >> 8) |
                      ((rm & 0x80) ? 0xffff0000 : 0);
    core->r[d] = result;
}

static void bkpt(core_t * core)   // BKPT; breakpoint
{
    bkpt_instr_debug_event(core);
    // TODO DebugMonitor
}

// if/then and hint instructions
static void it(core_t * core)    // IT; if-then
{
    uint32_t x = core->opcode;
    int firstcond = OP1(x);
    int mask = OP0(x);
    // ITSTATE.IT = {firstcond,mask}
    core->epsr = XPSR_SET_IT(core->epsr, (firstcond << 4) | mask);
}

static void nop(core_t * core)   // NOP; no operation
{
}

static void yield(core_t * core)    // YIELD
{
    if(!condition_passed(core)) {
        return;
    }
    hint_yield(core);
}

static void wfe(core_t * core)    // WFE; wait for event hint
{
    if(!condition_passed(core)) {
        return;
    }
    if(event_regsitered(core)) {
        clear_event_register(core);
    }
    else {
        wait_for_event(core);
    }
}

static void wfi(core_t * core)    // WFI; wait for interrupt hint
{
    if(!condition_passed(core)) {
        return;
    }
    wait_for_interrupt(core);
}

static void sev(core_t * core)    // SEV; send event hint
{
    if(!condition_passed(core)) {
        return;
    }
    hint_send_event(core);
}

static void hints(core_t * core)   // If-Then and hint instructions
{
    uint32_t x = core->opcode;
    int opa = OP1(x);
    int opb = OP0(x);
    if(opb != 0) {
        it(core);
        return;
    }
    switch(opa) {
    case 0: nop(core); break;
    case 1: yield(core); break;
    case 2: wfe(core); break;
    case 3: wfi(core); break;
    default: sev(core); break;
    }
}

/* -- Conditional branch, and Supervisor Call -- [ARMv7-M] A5.2.6 */

static void udf(core_t * core)   // UDF; permanently undefined
{
    if(!condition_passed(core)) {
        return;
    }
    undefined(core); // TODO undefined exception
}

static void svc(core_t * core)   // SVC; supervisor call (formerly SWI; software interrupt)
{
    if(!condition_passed(core)) {
        return;
    }
    call_supervisor(core);
}

static void b(core_t * core)   // B; conditional branch
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    uint32_t imm;
    if(!U16B_GET1(x, 13)) { // T1
        int cond = U16B_GET(x, 11, 8);
        if(cond == 0xe) {
            udf(core);
            return;
        }
        else if(cond == 0xf) {
            svc(core);
            return;
        }
        imm = IMM8(x);
        imm = (imm << 1) | ((imm & 0x80) ? 0xfffffe00 : 0); // sign extend
    }
    else if(!U16B_GET1(x, 12)) { // T2
        imm = IMM11(x) << 1;
        imm |= (imm & 0x800) ? 0xfffff000 : 0; // sign extend
    }
    else { // T3, T4
        assert(0);
    }
    branch_write_pc(core, core->r[PC] + imm);
}

/* ==== 32-bit Instructions ==== */

static void bl(core_t * core)   // BL; branch link
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    uint32_t ii = (M_I(x) << 2) | (M_J1(x) << 1) | M_J0(x); // = {s, j1, j2}
    ii ^= (ii & 4) ? 0 : 3;
    ii |= (ii & 4) ? 0xfffffff8 : 0;
    uint32_t imm = (ii << 22) | (M_IMM10(x) << 12) | (M_IMM11(x) << 1);
    uint32_t next_instr_addr = core->r[PC];
    core->r[LR] = next_instr_addr | 1;
    branch_write_pc(core, core->r[PC] + imm);
}

static uint32_t thumb_expand_imm_c(uint32_t imm, bool * carry)
{
    if(!U16B_GET(imm, 11, 10)) {
        switch((imm >> 8) & 3) {
        case 0: break; // zero_extend(imm[7:0])
        case 1: imm = ((imm & 0xff) << 16) | ((imm & 0xff) << 0); break;
        case 2: imm = ((imm & 0xff) << 24) | ((imm & 0xff) << 8); break;
        case 3: imm = ((imm & 0xff) << 24) | ((imm & 0xff) << 16) |
                          ((imm & 0xff) << 16) | ((imm & 0xff) << 0);
        }
    }
    else {
        uint32_t unrotated_value = (1 << 7) | (imm & 0x7f);
        int amt = (imm >> 7);
        imm = shift_c(unrotated_value, 3, amt, carry);
    }
    return imm;
}

static void bic(core_t * core)   // BIC (immediate); bit clear
{
    if(!condition_passed(core)) {
        return;
    }
    uint32_t x = core->opcode;
    int d = M_RC(x);
    int n = M_R0(x);
    bool set_flags = M_S(x);
    uint32_t imm = (M_I(x) << 11) | (M_IMM3(x) << 8) | M_IMM8(x);
    bool carry = XPSR_GET_C(core->apsr);
    imm = thumb_expand_imm_c(imm, &carry);
    uint32_t result = core->r[n] & ~imm;
    core->r[d] = result;
    if(set_flags) {
        bool overflow = XPSR_GET_V(core->apsr); // unchanged
        assign_flags(core, result, carry, overflow);
    }
}

/* ==== Instruction Decode ==== */

static void t32(core_t * core);

typedef struct {
    handler_t * handler;
    uint32_t mask;
    uint32_t addr;
    const char * name;
} instruction_t;

static instruction_t t16_instructions[] = {
    {lsl,     0xf800, 0x0000, "LSL (immediate) T1"},
    {lsr,     0xf800, 0x0800, "LSR (immediate) T1"},
    {asr,     0xf800, 0x1000, "ASR (immediate) T1"},
    {add_reg, 0xfe00, 0x1800, "ADD (register) T1"},
    {sub_reg, 0xfe00, 0x1a00, "SUB (register) T1"},
    {add,     0xfe00, 0x1c00, "ADD (immediate) T1"},
    {sub,     0xfe00, 0x1e00, "SUB (immediate) T1"},
    {mov,     0xf800, 0x2000, "MOV (immediate) T1"},
    {cmp,     0xf800, 0x2800, "CMP (immediate) T1"},
    {add,     0xf800, 0x3000, "ADD (immediate) T2"},
    {sub,     0xf800, 0x3800, "SUB (immediate) T2"},
    {and_reg, 0xffc0, 0x4000, "AND (register) T1"},
    {eor_reg, 0xffc0, 0x4040, "EOR (register) T1"},
    {lsl_reg, 0xffc0, 0x4080, "LSL (register) T1"},
    {lsr_reg, 0xffc0, 0x40c0, "LSR (register) T1"},
    {asr_reg, 0xffc0, 0x4100, "ASR (register) T1"},
    {adc_reg, 0xffc0, 0x4140, "ADC (register) T1"},
    {sbc_reg, 0xffc0, 0x4180, "SBC (register) T1"},
    {ror_reg, 0xffc0, 0x41c0, "ROR (register) T1"},
    {tst_reg, 0xffc0, 0x4200, "TST (register) T1"},
    {rsb,     0xffc0, 0x4240, "RSB (immediate) T1"},
    {cmp_reg, 0xffc0, 0x4280, "CMP (register) T1"},
    {cmn_reg, 0xffc0, 0x42c0, "CMN (register) T1"},
    {orr_reg, 0xffc0, 0x4300, "ORR (register) T1"},
    {mul,     0xffc0, 0x4340, "MUL T1"},
    {bic_reg, 0xffc0, 0x4380, "BIC (register) T1"},
    {mvn_reg, 0xffc0, 0x43c0, "MVN (register) T1"},
    {add_reg, 0xff00, 0x4400, "ADD (register) T2"},
    {cmp_reg, 0xff00, 0x4500, "CMP (register) T2"},
    {mov_reg, 0xff00, 0x4600, "MOV (register) T1"},
    {mov_reg, 0xffc0, 0x0000, "MOV (register) T2"}, // LSL --> MOV (reg) T2 (overlap)
    {bx,      0xff83, 0x4700, "BX T1"},
    {blx_reg, 0xff83, 0x4780, "BLX (register) T1"},
    {ldr_lit, 0xf800, 0x4800, "LDR (literal) T1"},
    {str_reg,   0xfe00, 0x5000, "STR (register) T1"},
    {strh_reg,  0xfe00, 0x5200, "STRH (register) T1"},
    {strb_reg,  0xfe00, 0x5400, "STRB (register) T1"},
    {ldrsb_reg, 0xfe00, 0x5600, "LDRSB (register) T1"},
    {ldr_reg,   0xfe00, 0x5800, "LDR (register) T1"},
    {ldrh_reg,  0xfe00, 0x5a00, "LDRH (register) T1"},
    {ldrb_reg,  0xfe00, 0x5c00, "LDRB (register) T1"},
    {ldrsh_reg, 0xfe00, 0x5e00, "LDRSH (register) T1"},
    {str,    0xf800, 0x6000, "STR (immediate) T1"},
    {ldr,    0xf800, 0x6800, "LDR (immediate) T1"},
    {strb,   0xf800, 0x7000, "STRB (immediate) T1"},
    {ldrb,   0xf800, 0x7800, "LDRB (immediate) T1"},
    {strh,   0xf800, 0x8000, "STRH (immediate) T1"},
    {ldrh,   0xf800, 0x8800, "LDRH (immediate) T1"},
    {str,    0xf800, 0x9000, "STR (immediate) T2"}, // STR SP relative
    {ldr,    0xf800, 0x9800, "LDR (immediate) T2"}, // LDR SP relative
    {cps,    0xffec, 0xb660, "CPS T1"},
    {adr,    0xf800, 0xa000, "ADR T1"},
    {add_sp, 0xf800, 0xa800, "ADD (SP plus immediate) T1"},
    {add_sp, 0xff80, 0xb000, "ADD (SP plus immediate) T2"},
    {sub_sp, 0xff80, 0xb080, "SUB (SP minus immediate) T1"},
    {cbnz_cbz, 0xf500, 0xb100, "CBNZ, CBZ T1"},
    {sxth,   0xffc0, 0xb200, "SXTH T1"},
    {sxtb,   0xffc0, 0xb240, "SXTB T1"},
    {uxth,   0xffc0, 0xb280, "UXTH T1"},
    {uxtb,   0xffc0, 0xb2c0, "UXTB T1"},
    {push,   0xfe00, 0xb400, "PUSH T1"},
    {rev,    0xffc0, 0xba00, "REV T1"},
    {rev16,  0xffc0, 0xba40, "REV16 T1"},
    {revsh,  0xffc0, 0xbac0, "REVSH T1"},
    {pop,    0xfe00, 0xbc00, "POP T1"},
    {bkpt,   0xff00, 0xbe00, "BKPT T1"},
    {hints,  0xff00, 0xbf00, "If-Then and hints"},
    {stm,    0xf800, 0xc000, "STM, STMIA, STMEA T1"},
    {ldm,    0xf800, 0xc800, "LDM, LDMIA, LDMFD T1"},
    {b,      0xf000, 0xd000, "B T1, UDF, SVC"}, // calls UDF or SVC if required
    {b,      0xf800, 0xe000, "B T2"},
    // already covered: {udf,    0xf800, 0xde00, "UDF"}, // overlaps B T1
    // already covered: {svc,    0xff00, 0xdf00, "SVC"}, // overlaps B T1
    {t32,    0xf800, 0xe800, "Thumb-32"}, // [15:11] = b11101
    {t32,    0xf000, 0xf000, "Thumb-32"}, // [15:11] = b11110 or b11111
    {0}
};

static instruction_t t32_instructions[] = {
    {bic, 0x8000fbe0, 0x0000f020, "BIC (immediate) T1"},
    {bl,  0xd000f800, 0xd000f000, "BL"},
    {0}
};

static void t32(core_t * core)
{
    uint32_t x = core->opcode;
    handler_t * handler = undefined;
    for(const instruction_t * inst = t32_instructions; inst->handler; inst++) {
        if((x & inst->mask) == inst->addr) {
            handler = inst->handler;
            break;
        }
    }
    handler(core);
}

static void hash_init()
{
    // loads instructions into jump table
    if(hash[0] != NULL) {
        return;
    }
    for(int i = 0; i < HASH_SIZE; i++) {
        hash[i] = undefined;
    }
    int overlaps = 0;
    const instruction_t * inst = t16_instructions;
    while(inst->handler != NULL) {
        int overlap = 0;
        for(int i = 0; i < HASH_SIZE; i++) {
            if((i & inst->mask) == (inst->addr & inst->mask)) {
                /* Note: mov_reg overlaps with lsl. lsl specifically redirects
                 * to mov_reg when the opcode bits overlap. */
                overlap |= (hash[i] != undefined) && (inst->handler != mov_reg);
                hash[i] = inst->handler;
            }
        }
        if(overlap) {
            printf("warning: %s: opcode overlap 0x%04x (b", inst->name, inst->addr & inst->mask);
            for(int i = 15; i >= 0; i--) {
                printf("%c", (((inst->mask) >> i) & 1) ? (((inst->addr) >> i) & 1) + '0' : 'x');
                if(i % 4 == 0) {
                    printf(" ");
                }
                if(i % 8 == 0) {
                    printf(" ");
                }
            }
            printf(")\n");
        }
        overlaps += overlap;
        inst++;
    }
    if(overlaps != 0) {
        printf("error: %d opcode overlap%s detected\n", overlaps, overlaps > 1 ? "s" : "");
    }
    assert(!overlaps);
}
