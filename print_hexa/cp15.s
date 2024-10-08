/* Global Symbols */
.global .cp15_configure
.type .cp15_configure, %function


/* Text Section */
.section .text,"ax"
         .code 32
         .align 4
         
/********************************************************
CP15 CONFIGURE
********************************************************/
.cp15_configure:
    stmfd sp!,{r0-r2,lr}

    // 2016: Brian Fraser's Fix for UBoot Messing with Interrupts
    // Disable the MMU, instruction and data caches.
    // Without this, the ISRs seem not to work with the latest UBoot code (2016)
    //SUB r0, r0, r0
    //MCR p15, 0, r0, c1, c0, 0

    /* Set V=0 in CP15 SCTRL register - for VBAR to point to vector */
    mrc    p15, 0, r0, c1, c0, 0    // Read CP15 SCTRL Register
    bic    r0, #(1 << 13)           // V = 0
    mcr    p15, 0, r0, c1, c0, 0    // Write CP15 SCTRL Register

    /* Set vector address in CP15 VBAR register */
    ldr     r0, =_vector_table
    mcr     p15, 0, r0, c12, c0, 0  //Set VBAR */


    ldmfd sp!,{r0-r2,pc}
/********************************************************/


















