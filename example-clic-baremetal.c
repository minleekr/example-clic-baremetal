/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * This example sets up the CPU to service local interrupts using
 * the CLIC mode of operation. SiFive GPIO's are configured as inputs
 * to support a hardware platform like the Arty 100T with buttons
 * that are connected to the local interrupt lines.
 */

#include <stdio.h>
#include <stdlib.h>

/* These includes get created at build time, and are based on the contents
 * in the bsp folder.  They are useful since they allow us
 * to use auto generated symbols and base addresses which may change
 * based on the design, and every design has it's own unique bsp.
 */
#include <metal/machine.h>
#include <metal/machine/platform.h>
#include <metal/machine/inline.h>

/*
 * This test demonstrates how to enable and handle local interrupts,
 * like the software interrupt using Interrupt ID #3, the
 * timer interrupt using Interrupt ID #7, and buttons on the
 * Arty 100T platform, which are typically in the #16-31 range.
 *
 * This example uses the CLIC vectored mode of operation, which
 * uses a vector table and is lower latency than CLIC direct mode,
 * due to the software overhead.
 *
 * CLIC direct mode does not use a vector table, so all
 * interrupts and exceptions trap to mtvec.base address and software
 * is responsible for dispatching interrupts based on the contents
 * in the mcause CSR.  CLIC direct mode of operation is not supported
 * in this example.
 */

#define DISABLE                 0
#define ENABLE                  1
#define TRUE                    1
#define FALSE                   0
#define INPUT                   0x100    /* something other than 0 or 1 */
#define OUTPUT                  0x101    /* something other than 0 or 1 */
#define RTC_FREQ                32768

#define MCAUSE_INTR                         0x80000000UL
#define MCAUSE_CAUSE                        0x000003FFUL
#define MCAUSE_CODE(cause)                  (cause & MCAUSE_CAUSE)

/* Compile time options to determine which interrupt modules we have */
#define CLIC_PRESENT                            (METAL_MAX_CLIC_INTERRUPTS > 0)
#define PLIC_PRESENT                            (METAL_MAX_PLIC_INTERRUPTS > 0)

/* Interrupt Specific defines - used for mtvec.mode field, which is bit[0] for
 * designs with CLINT, or [1:0] for designs with a CLIC */
#define MTVEC_MODE_CLINT_DIRECT                 0x00
#define MTVEC_MODE_CLINT_VECTORED               0x01
#define MTVEC_MODE_CLIC_DIRECT                  0x02
#define MTVEC_MODE_CLIC_VECTORED                0x03

/* Offsets for multi-core systems */
#define MSIP_PER_HART_OFFSET                             0x4
#define MTIMECMP_PER_HART_OFFSET                         0x8

#if CLIC_PRESENT
#define CLIC_BASE_ADDR                                  METAL_SIFIVE_CLIC0_0_BASE_ADDRESS
#define MSIP_BASE_ADDR(hartid)                          (CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_MSIP_BASE + (hartid * MSIP_PER_HART_OFFSET))
#define MTIMECMP_BASE_ADDR(hartid)                      (CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_MTIMECMP_BASE + (hartid * MTIMECMP_PER_HART_OFFSET))
#define MTIME_BASE_ADDR                                 (CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_MTIME)
#define HART0_CLIC_OFFSET                               0x00800000
#define HART0_CLIC_BASE_ADDR                            (CLIC_BASE_ADDR + HART0_CLIC_OFFSET)
#define HART0_CLICINTIP_ADDR(int_num)                   (HART0_CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_CLICINTIP_BASE + int_num)   /* one byte per enable */
#define HART0_CLICINTIE_ADDR(int_num)                   (HART0_CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_CLICINTIE_BASE + int_num)   /* one byte per enable */
#define HART0_CLICINTCFG_ADDR(int_num)                  (HART0_CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_CLICINTCTL_BASE + int_num)   /* one byte per enable */
#define HART0_CLICCFG_ADDR                              (HART0_CLIC_BASE_ADDR + METAL_SIFIVE_CLIC0_CLICCFG)   /* one byte per CLIC */
#define CLICCFG_NVBITS(x)                               ((x & 1) << 0)
#define CLICCFG_NLBITS(x)                               ((x & 0xF) << 1)
#define CLICCFG_NMBITS(x)                               ((x & 0x3) << 5)
#define INT_ID_SOFTWARE                                 3
#define INT_ID_TIMER                                    7
#define INT_ID_EXTERNAL                                 11
#define INT_ID_CLIC_SOFTWARE                            12
#define MAX_LOCAL_INTS                                  16  /* local interrupts, not local external interrupts */
#define CLIC_VECTOR_TABLE_SIZE_MAX                      METAL_SIFIVE_CLIC0_2000000_SIFIVE_NUMINTS
#define SOFTWARE_INT_ENABLE                             write_byte(HART0_CLICINTIE_ADDR(INT_ID_SOFTWARE), ENABLE);
#define SOFTWARE_INT_DISABLE                            write_byte(HART0_CLICINTIE_ADDR(INT_ID_SOFTWARE), DISABLE);
#define TIMER_INT_ENABLE                                write_byte(HART0_CLICINTIE_ADDR(INT_ID_TIMER), ENABLE);
#define TIMER_INT_DISABLE                               write_byte(HART0_CLICINTIE_ADDR(INT_ID_TIMER), DISABLE);
#define EXTERNAL_INT_ENABLE                             write_byte(HART0_CLICINTIE_ADDR(INT_ID_EXTERNAL), ENABLE);
#define EXTERNAL_INT_EDISABLE                           write_byte(HART0_CLICINTIE_ADDR(INT_ID_EXTERNAL), DISABLE);
#define CLIC_SOFTWARE_INT_ENABLE                        write_byte(HART0_CLICINTIE_ADDR(INT_ID_CLIC_SOFTWARE), ENABLE);
#define CLIC_SOFTWARE_INT_DISABLE                       write_byte(HART0_CLICINTIE_ADDR(INT_ID_CLIC_SOFTWARE), DISABLE);
#define CLIC_SOFTWARE_INT_SET                           write_byte(HART0_CLICINTIP_ADDR(INT_ID_CLIC_SOFTWARE), ENABLE);
#define CLIC_SOFTWARE_INT_CLEAR                         write_byte(HART0_CLICINTIP_ADDR(INT_ID_CLIC_SOFTWARE), DISABLE);
#else
#error "This design does not have a CLIC...Exiting.\n");
#endif

#define NUM_TICKS_ONE_S                         RTC_FREQ            // it takes this many ticks of mtime for 1s to elapse
#define NUM_TICKS_ONE_MS                        (RTC_FREQ/1000)     // it takes this many ticks of mtime for 1ms to elapse
#define DEMO_TIMER_INTERVAL                     5000                // 5s timer interval
#define SET_TIMER_INTERVAL_MS(ms_ticks)         write_dword(MTIMECMP_BASE_ADDR(read_csr(mhartid)), (read_dword(MTIME_BASE_ADDR) + (ms_ticks * NUM_TICKS_ONE_MS)))

inline __attribute__((always_inline)) void interrupt_global_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}

inline __attribute__((always_inline)) void interrupt_global_disable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}

/* Defines to access CSR registers within C code */
#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define write_dword(addr, data)                 ((*(volatile uint64_t *)(addr)) = data)
#define read_dword(addr)                        (*(volatile uint64_t *)(addr))
#define write_word(addr, data)                  ((*(volatile uint32_t *)(addr)) = data)
#define read_word(addr)                         (*(volatile uint32_t *)(addr))
#define write_byte(addr, data)                  ((*(volatile uint8_t *)(addr)) = data)
#define read_byte(addr)                         (*(volatile uint8_t *)(addr))

/* Globals */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) software_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) clic_software_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) timer_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) external_handler (void);
void __attribute__((weak, interrupt, aligned(64))) default_exception_handler(void);

__attribute__((aligned(64))) uintptr_t __mtvt_clic_vector_table[CLIC_VECTOR_TABLE_SIZE_MAX];

/* user interrupt handler */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc0_handler (void);

/* you can activate what you want to test */
#define ACTIVATE_SOFTWARE_INTERRUPT         0
#define ACTIVATE_CLIC_SOFTWARE_INTERRUPT    0
#define ACTIVATE_TIMER_INTERRUPT            0
#define ACTIVATE_EXTERNAL_INTERRUPT         0
#define ACTIVATE_NESTED_INTERRUPT           0
#define ACTIVATE_LOCAL_EXT_INTERRUPT        1

/* Main - Setup CLIC interrupt handling and describe how to trigger interrupt */
int main() {

    uint32_t i, mode = MTVEC_MODE_CLIC_VECTORED;
    uintptr_t mtvec_base, mtvt_base;
    uint8_t cliccfg, clicintcfg;

    /* Write mstatus.mie = 0 to disable all machine interrupts prior to setup */
    interrupt_global_disable();

    /* Setup mtvec to point to our exception handler table using mtvec.base,
     * and assign mtvec.mode = 3 for CLIC vectored mode of operation. The
     * mtvec.mode field is bit[0] for designs with CLINT, or [1:0] using CLIC */
    mtvec_base = (uintptr_t)&default_exception_handler;
    write_csr (mtvec, (mtvec_base | mode));

    /* Setup mtvt which is CLIC specific, to hold base address for interrupt handlers */
    mtvt_base = (uintptr_t)&__mtvt_clic_vector_table;
    write_csr (0x307, (mtvt_base));  /* 0x307 is CLIC CSR number */

    for (int i = 0; i < CLIC_VECTOR_TABLE_SIZE_MAX; i++)
    {
        __mtvt_clic_vector_table[i] = (uintptr_t)&default_exception_handler;
    }

    /* Setup CLICCFG
     * Turn off Selective vectoring (NVBITS = 0)
     * Select a single preemption level of 255 (NLBITS = 0)
     * Machine mode interrupts only (NMBITS = 0)
     */
    cliccfg = (CLICCFG_NVBITS(0) | CLICCFG_NLBITS(0) | CLICCFG_NMBITS(0));
    write_byte(HART0_CLICCFG_ADDR, cliccfg);

    /* The core has a total of METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS bits in clicintcfg
     *  which specify how to encode a given interrupts pre-emption level and/or priority.
     *  The actual number of bits which determine the preemption level is determined by
     *  cliccfg.NLBITS. If cliccfg.NLBITS is < METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS,
     *  then the remaining least significant implemented bits are used to encode priorities
     *  within a given pre-emption level.
     *
     * #NLBITS encoding  interrupt levels
     *   1     l.......                        127,                            255
     *   2     ll......           63,          127,            191,            255
     *   3     lll.....     31,   63,   95,    127,    159,    191,    223,    255
     *   4     llll....  15,31,47,63,79,95,111,127,143,159,175,191,207,223,239,255
     */

    /* If cliccfg.NLBITS is set to zero, then all interrupts are treated as level 255 and
     *  all METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS bits are used to set priorities.
     *  In case there are multiple pending-and-enabled interrupts at the same highest priority,
     *  the highest-numbered interrupt ID is taken first
     *  e.g) available priorities at METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS = 2
     *
     * #NLBITS encoding  interrupt level = 255, belows are available priorities
     *   0     pp......           63,          127,            191,            255
     */
    clicintcfg = 255;

#if ACTIVATE_NESTED_INTERRUPT
    /* cliccfg.NLBITS needs to be set for the nested interrupt
     *  e.g) avaliable levels at METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS = 2
     *
     * #NLBITS encoding  interrupt levels
     *   0     ll......           63,          127,            191,            255
     */
    cliccfg = read_byte(HART0_CLICCFG_ADDR);
    cliccfg |= CLICCFG_NLBITS(METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTBITS);
    write_byte(HART0_CLICCFG_ADDR, cliccfg);
#endif

    /* software interrupt example */
#if ACTIVATE_SOFTWARE_INTERRUPT
    __mtvt_clic_vector_table[INT_ID_SOFTWARE] = (uintptr_t)&software_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_SOFTWARE), clicintcfg);
    SOFTWARE_INT_ENABLE;
#endif

    /* clic software interrupt example */
#if ACTIVATE_CLIC_SOFTWARE_INTERRUPT
    __mtvt_clic_vector_table[INT_ID_CLIC_SOFTWARE] = (uintptr_t)&clic_software_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_CLIC_SOFTWARE), clicintcfg);
    CLIC_SOFTWARE_INT_ENABLE;
#endif

    /* timer interrupt example */
#if ACTIVATE_TIMER_INTERRUPT
    __mtvt_clic_vector_table[INT_ID_TIMER] = (uintptr_t)&timer_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_TIMER), clicintcfg);

    /* you need to set the timer before enable irq*/
    SET_TIMER_INTERVAL_MS(DEMO_TIMER_INTERVAL);
    TIMER_INT_ENABLE;
#endif

    /* external interrupt example */
#if ACTIVATE_EXTERNAL_INTERRUPT
    __mtvt_clic_vector_table[INT_ID_EXTERNAL] = (uintptr_t)&external_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_EXTERNAL), clicintcfg);
    EXTERNAL_INT_ENABLE;
#endif

#if ACTIVATE_LOCAL_EXT_INTERRUPT
    /* how to set clic local external interrupt
     *  1. select irq number (16 ~ (METAL_SIFIVE_CLIC0_0_SIFIVE_NUMINTS-1))
     *  2. set level(including priority) of irq on clicintcfg
     *  3. register irq handler
     *  4. enable irq
     * /

    /* local_ext_irq0 */
    i = 16;
    /* configure level/priority*/
    write_byte(HART0_CLICINTCFG_ADDR(i), 255);
    /* register the irq handler */
    __mtvt_clic_vector_table[i] = (uintptr_t)&lc0_handler;
    /* enable local_ext_irq0 */
    write_byte(HART0_CLICINTIE_ADDR(i), ENABLE);

    /* local_ext_irq1 */
    //i = 17;

    /* local_ext_irq16 */
    //i = 32;

    /* local_ext_irq31 */
    //i = 47;
#endif

    /* Write mstatus.mie = 1 to enable all machine interrupts */
    interrupt_global_enable();

#if ACTIVATE_SOFTWARE_INTERRUPT
    /* Set Software Pending Bit to trigger irq*/
    write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x1);
#endif

#if ACTIVATE_CLIC_SOFTWARE_INTERRUPT
    /* Set Software Pending Bit to trigger irq*/
    CLIC_SOFTWARE_INT_SET;
#endif

    while (1) {
        // go to sleep
        asm volatile ("wfi");
    }

    // just for compile, but it should not return!!
    return 0;
}

/* External Interrupt ID #11 - handles all global interrupts */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) external_handler (void) {

    /* The external interrupt is usually used for a PLIC, which handles global
     * interrupt dispatching.  If no PLIC is connected, then custom IP can connect
     * to this interrupt line, and this is where interrupt handling
     * support would reside.  This demo does not use the PLIC.
     */
}

/* Software Interrupt ID #3 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) software_handler (void) {

    /* Clear Software Pending Bit */
    write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x0);

    /* Do Something after clear SW irq pending*/
}

/* Timer Interrupt ID #7 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) timer_handler (void) {

    /* Disable timer interrupt or Set next timer*/
    TIMER_INT_DISABLE;

    /* Just Do Something when the timer is expired */
}

/* CLIC Software Interrupt ID #12 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) clic_software_handler (void) {

    /* Clear Software Pending Bit */
    CLIC_SOFTWARE_INT_CLEAR;

    /* Do Something after clear SW irq pending*/
}

/* local irq0 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc0_handler (void) {
    /* Add functionality if desired */

}

/* local irq1 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc1_handler (void) {
    /* Add functionality if desired */

}

/* local irq2 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc2_handler (void) {
    /* Add functionality if desired */

}

/* local irq3 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc3_handler (void) {
    /* Add functionality if desired */

}

/* local irq4 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc4_handler (void) {
    /* Add functionality if desired */

}

/* local irq5 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc5_handler (void) {
    /* Add functionality if desired */

}

/* local irq6 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc6_handler (void) {
    /* Add functionality if desired */

}

/* local irq7 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc7_handler (void) {
    /* Add functionality if desired */

}

/* local irq8 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc8_handler (void) {
    /* Add functionality if desired */

}

/* local irq9 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc9_handler (void) {
    /* Add functionality if desired */

}

/* local irq10 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc10_handler (void) {
    /* Add functionality if desired */

}

/* local irq11 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc11_handler (void) {
    /* Add functionality if desired */

}

/* local irq12 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc12_handler (void) {
    /* Add functionality if desired */

}

/* local irq13 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc13_handler (void) {
    /* Add functionality if desired */

}

/* local irq14 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc14_handler (void) {
    /* Add functionality if desired */

}

/* local irq15 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc15_handler (void) {
    /* Add functionality if desired */

}

/* local irq16 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc16_handler (void) {
    /* Add functionality if desired */

}

/* local irq17 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc17_handler (void) {
    /* Add functionality if desired */

}

/* local irq18 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc18_handler (void) {
    /* Add functionality if desired */

}

/* local irq19 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc19_handler (void) {
    /* Add functionality if desired */

}

/* local irq20 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc20_handler (void) {
    /* Add functionality if desired */

}

/* local irq21 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc21_handler (void) {
    /* Add functionality if desired */

}

/* local irq22 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc22_handler (void) {
    /* Add functionality if desired */

}

/* local irq23 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc23_handler (void) {
    /* Add functionality if desired */

}

/* local irq24 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc24_handler (void) {
    /* Add functionality if desired */

}

/* local irq25 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc25_handler (void) {
    /* Add functionality if desired */

}

/* local irq26 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc26_handler (void) {
    /* Add functionality if desired */

}

/* local irq27 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc27_handler (void) {
    /* Add functionality if desired */

}

/* local irq28 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc28_handler (void) {
    /* Add functionality if desired */

}

/* local irq29 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc29_handler (void) {
    /* Add functionality if desired */

}

/* local irq30 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc30_handler (void) {
    /* Add functionality if desired */

}

/* local irq31 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc31_handler (void) {
    /* Add functionality if desired */

}

void __attribute__((weak, aligned(64))) default_exception_handler(void) {

    /* Read mcause to understand the exception type */
    uintptr_t mcause = read_csr(mcause);
    uintptr_t mepc = read_csr(mepc);
    uintptr_t mtval = read_csr(mtval);
    uintptr_t code = MCAUSE_CODE(mcause);

    while (1);
}
