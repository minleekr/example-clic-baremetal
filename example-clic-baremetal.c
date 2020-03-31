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
#define MAX_LOCAL_INTS                                  16  /* local interrupts, not local external interrupts */
#define CLIC_VECTOR_TABLE_SIZE_MAX                      1024
#define SOFTWARE_INT_ENABLE                             write_byte(HART0_CLICINTIE_ADDR(INT_ID_SOFTWARE), ENABLE);
#define SOFTWARE_INT_DISABLE                            write_byte(HART0_CLICINTIE_ADDR(INT_ID_SOFTWARE), DISABLE);
#define TIMER_INT_ENABLE                                write_byte(HART0_CLICINTIE_ADDR(INT_ID_TIMER), ENABLE);
#define TIMER_INT_DISABLE                               write_byte(HART0_CLICINTIE_ADDR(INT_ID_TIMER), DISABLE);
#define EXTERNAL_INT_ENABLE                             write_byte(HART0_CLICINTIE_ADDR(INT_ID_EXTERNAL), ENABLE);
#define EXTERNAL_INT_EDISABLE                           write_byte(HART0_CLICINTIE_ADDR(INT_ID_EXTERNAL), DISABLE);
#else
#error "This design does not have a CLIC...Exiting.\n");
#endif

#define NUM_TICKS_ONE_S                         RTC_FREQ            // it takes this many ticks of mtime for 1s to elapse
#define NUM_TICKS_ONE_MS                        (RTC_FREQ/1000)     // it takes this many ticks of mtime for 1ms to elapse
#define DEMO_TIMER_INTERVAL                     5000                // 5s timer interval
#define SET_TIMER_INTERVAL_MS(ms_ticks)         write_dword(MTIMECMP_BASE_ADDR(read_csr(mhartid)), (read_dword(MTIME_BASE_ADDR) + (ms_ticks * NUM_TICKS_ONE_MS)))

/* Setup prototypes */
void interrupt_global_enable (void);
void interrupt_global_disable (void);

/* Defines to access CSR registers within C code */
#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define write_dword(addr, data)                 ((*(uint64_t *)(addr)) = data)
#define read_dword(addr)                        (*(uint64_t *)(addr))
#define write_word(addr, data)                  ((*(uint32_t *)(addr)) = data)
#define read_word(addr)                         (*(uint32_t *)(addr))
#define write_byte(addr, data)                  ((*(uint8_t *)(addr)) = data)
#define read_byte(addr)                         (*(uint8_t *)(addr))

/* Globals */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) software_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) timer_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc0_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) lc1_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) external_handler (void);
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"), aligned(64))) default_exception_handler(void);

__attribute__((aligned(64))) uintptr_t __mtvt_clic_vector_table[CLIC_VECTOR_TABLE_SIZE_MAX];

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


#if CLIC_PRESENT

    /* Setup Levels and priorities in CLICCFG global register
     * Turn off Selective vectoring (NVBITS = 0)
     * Select a single preemption level of 255 (NLBITS = 0)
     * Machine mode interrupts only (NMBITS = 0)
     */
    cliccfg = (CLICCFG_NVBITS(0) | CLICCFG_NLBITS(0) | CLICCFG_NMBITS(0));
    write_byte(HART0_CLICCFG_ADDR, cliccfg);

    /* Use 0x3 as a default priority for all interrupts */
    clicintcfg = 0x3;
    /* If you want to use Nested Interrupt, you should set Interrupt Levels and Priorities.
     * cliccfg.NLBITS is the number of Level bits available.
     * Level value is masked by cliccfg.NLBITS. The remaining bits are for Priority.
     * #NLBITS encoding  interrupt levels
     *   1     l.......                        127,                            255
     *   2     ll......           63,          127,            191,            255
     *   3     lll.....     31,   63,   95,    127,    159,    191,    223,    255
     *   4     llll....  15,31,47,63,79,95,111,127,143,159,175,191,207,223,239,255
     */

    /* Enable CLIC local interrupt lines 3, 7, 11 for software, timer, and external,
     * and setup handlers in the vector table.
     */

    /* If you want no use the software interrupt, please comment out it */
    __mtvt_clic_vector_table[INT_ID_SOFTWARE] = (uintptr_t)&software_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_SOFTWARE), clicintcfg);
    SOFTWARE_INT_ENABLE;

    /* If you want no use the cpu timer interrupt, please comment out it */
    __mtvt_clic_vector_table[INT_ID_TIMER] = (uintptr_t)&timer_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_TIMER), clicintcfg);
    TIMER_INT_ENABLE;

    /* If you want no use the external interrupt, please comment out it */
    __mtvt_clic_vector_table[INT_ID_EXTERNAL] = (uintptr_t)&external_handler;
    write_byte(HART0_CLICINTCFG_ADDR(INT_ID_EXTERNAL), clicintcfg);
    EXTERNAL_INT_ENABLE;

    /* If you want no use CLIC local external interrupt, please comment out it */
    /* Get numeric list of CLIC local external interrupt lines and enable those at the CPU */
    /* In CLIC modes, mie is hardwired to 0, so we use clicinten[] here to enable */
    i = 16; /* local irq 0 */
    /* Sample code by required local interrupt number */
    //i = 17 /* local irq 1 */
    //i = 32 /* local irq 16 */
    //i = 47 /* local irq 31 */
    write_byte(HART0_CLICINTIE_ADDR(i), ENABLE);

    /* Since NLBITS = 0, CLICINTCFG register holds only priorities for each interrupt */
    write_byte(HART0_CLICINTCFG_ADDR(i), clicintcfg);

    /* default all interrupt lines to the button handler as an example on how to setup vector table */
    __mtvt_clic_vector_table[i] = (uintptr_t)&lc0_handler;

#endif

    /* If you want to set the interval of timer interrupt, please uncomment it */
    /* setup next timer interrupt interval */
    //SET_TIMER_INTERVAL_MS(DEMO_TIMER_INTERVAL);

    /* Write mstatus.mie = 1 to enable all machine interrupts */
    interrupt_global_enable();

    /* If you want to trigger the software interrupt, please uncomment out it */
    /* write msip and display message that s/w handler was hit */
    //write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x1);

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

    /* Clear Software Pending Bit which clears mip.msip bit */
    write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x0);

    /* Do Something after clear SW irq pending*/
}

/* Timer Interrupt ID #7 */
void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"))) timer_handler (void) {
    /* Just Do Something when the timer is expired*/

	/* If you want to set the interval of timer interrupt, please uncomment it */
	/* set our next interval */
	//SET_TIMER_INTERVAL_MS(DEMO_TIMER_INTERVAL);
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

void __attribute__((weak, interrupt("SiFive-CLIC-preemptible"), aligned(64))) default_exception_handler(void) {

    /* Read mcause to understand the exception type */
    uintptr_t mcause = read_csr(mcause);
    uintptr_t mepc = read_csr(mepc);
    uintptr_t mtval = read_csr(mtval);
    uintptr_t code = MCAUSE_CODE(mcause);

    while (1);
}

void interrupt_global_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}

void interrupt_global_disable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}
