/* Global Symbols */
.global INTC_BASE
.global INTC_ILR
.global _vector_table
.global CRLF
.global dump_separator
.global hex_prefix
.global ascii
.global c_goto
.global goto_addr


/* Registers */
.equ INTC_BASE, 0x48200000
.equ INTC_ILR,  0x48200100


/* General Definitions */


/* CPSR */

.equ CPSR_I,   0x80
.equ CPSR_F,   0x40
.equ CPSR_IRQ, 0x12
.equ CPSR_USR, 0x10
.equ CPSR_FIQ, 0x11
.equ CPSR_SVC, 0x13
.equ CPSR_ABT, 0x17
.equ CPSR_UND, 0x1B
.equ CPSR_SYS, 0x1F


//.equ VECTOR_BASE, 0x4030CE20
.equ VECTOR_BASE, 0x4030CE00 // Vector Base on BBB
//.equ VECTOR_BASE, 0x4030FC00


/*******************************
Stack
/*******************************/
.set StackModeSize,  0x100

.equ StackUSR, (_stack_end - 0*StackModeSize)
.equ StackFIQ, (_stack_end - 1*StackModeSize)
.equ StackIRQ, (_stack_end - 2*StackModeSize)
.equ StackSVC, (_stack_end - 3*StackModeSize)
.equ StackABT, (_stack_end - 4*StackModeSize)
.equ StackUND, (_stack_end - 5*StackModeSize)
.equ StackSYS, (_stack_end - 6*StackModeSize)

.section .text,"ax"
         .code 32
         .align 4

/********************************************************/
/* Vector table */
/********************************************************/
_vector_table:
    ldr   pc, _reset     /* reset - _start           */
    ldr   pc, _undf      /* undefined - _undf        */
    ldr   pc, _swi       /* SWI - _swi               */
    ldr   pc, _pabt      /* program abort - _pabt    */
    ldr   pc, _dabt      /* data abort - _dabt       */
    nop                  /* reserved                 */
    ldr   pc, _irq       /* IRQ - read the VIC       */
    ldr   pc, _fiq       /* FIQ - _fiq               */

_reset: .word _start
_undf:  .word 0x4030CE24 /* undefined               */
_swi:   .word 0x4030CE28 /* SWI                     */
_pabt:  .word 0x4030CE2C /* program abort           */
_dabt:  .word 0x4030CE30 /* data abort              */
         nop
_irq:   .word 0x4030CE38  /* IRQ                     */
_fiq:   .word 0x4030CE3C  /* FIQ                     */

/********************************************************/
/* Startup Code */
/********************************************************/
_start:

    /* Configure CP15 */
    bl .cp15_configure
         
    /* init */
    mrs r0, cpsr
    bic r0, r0, #0x1F            // clear mode bits
    orr r0, r0, #CPSR_SVC        // set SVC mode
    orr r0, r0, #(CPSR_F | CPSR_I)        // disable FIQ and IRQ
    msr cpsr, r0


   /* Stack setup */
   mov r0, #(CPSR_I | CPSR_F) | CPSR_SVC
   msr cpsr_c, r0
   ldr sp,=StackSVC
  
   mov r0, #(CPSR_I | CPSR_F) | CPSR_IRQ
   msr cpsr_c, r0
   ldr sp,=StackIRQ

   mov r0, #(CPSR_I | CPSR_F) | CPSR_FIQ
   msr cpsr_c, r0
   ldr sp,=StackFIQ

   mov r0, #(CPSR_I | CPSR_F) | CPSR_UND
   msr cpsr_c, r0
   ldr sp,=StackUND

   mov r0, #(CPSR_I | CPSR_F) | CPSR_ABT
   msr cpsr_c, r0
   ldr sp,=StackABT

   mov r0, #(CPSR_I | CPSR_F) | CPSR_SYS
   msr cpsr_c, r0
   ldr sp,=StackSYS

//   mov r0, #(CPSR_I | CPSR_F) | CPSR_USR
   mov r0, # CPSR_USR
   msr cpsr_c, r0
   ldr sp,=StackUSR

         
    /* Exceptions Setup */
    ldr r0, =_irq
    ldr r1, =.irq_handler
    str r1, [r0]
    
    ldr r0, =_swi
    ldr r1, =.swi_handler
    str r1, [r0]
      
    /* Hardware setup */
    bl .gpio_setup
    bl .disable_wdt
    bl .rtc_setup
    bl .uart0_int_setup

    /*Interrupt setup */
    //bl .uart0_int_setup
    
    /* Enable global irq */
    mrs r0, cpsr
    and r0, r0, #~(CPSR_I)
    msr cpsr, r0

    //bl .prompt
    
    
/********************************************************/
/* Main Code */
/********************************************************/
.main:
    bl .cmd_msn
    /* SWI Test*/    
    //swi #0x250

.main_loop:
    /* logical 1 turns on the led, TRM 25.3.4.2.2.2 */
    ldr r1,=c_goto
    ldrb r1,[r1],#1
    cmp r1,#1
    beq goto_salt
    bl .delay_1s
    b .main_loop    

goto_salt:
    ldr r1,=c_goto
    mov r0,#0
    strb r0,[r1],#1
    ldr r1,=goto_addr
    ldr r0,[r1]
    ldr r1,=0x80000044
    cmp r0,r1
    ldrlt pc,=.main

    bl .hex_to_ascii
    mov pc,r0
    //b .main_loop
/********************************************************/




/********************************************************
IRQ Handler
********************************************************/
.irq_handler:
        stmfd sp!, {r0-r3, r11, lr}
        mrs r11, spsr
        
 	/* Interrupt Source */
	ldr r0,=INTC_BASE
	ldr r1, [r0, #0x40]
	
	/* if rtc interrupt */
	and r1,r1, #0x7f
	cmp r1, #75  /* TRM 6.3 Table 6-1*/
	bleq .rtc_isr
    cmp r1, #72  /* TRM 6.3 Table 6-1*/
    bleq .uart_isr


        /* new IRQ */
        ldr r0, =INTC_BASE 
        ldr r1, =0x1
        str r1, [r0, #0x48]

        /*Data Sync Barrier */
	dsb
        msr spsr, r11
        ldmfd sp!, {r0-r3, r11, pc}^



/********************************************************/

/********************************************************/
.fiq_handler:
   b .      
/********************************************************/
.undefined_handler:
   b .     
/********************************************************/
.swi_handler:
   
    stmfd           sp!,{r0-r12,lr} 
    ldr             r0,[lr,#-4]      
                                                             
    bic             r1,r0,#0xff000000   //SWI Number  
    
    ldr r0, =swi_msg
    bl .print_string
    
    mov r0, r1
    bl .hex_to_ascii
    
    ldr r0,=CRLF
    bl .print_string
    
    
    ldmfd           sp!, {r0-r12,pc}^     
   
   
/********************************************************/
.prefetch_abort_handler:
   b .      
/********************************************************/
.data_abort_handler:
   b .      
/********************************************************/



/* Read-Only Data Section */
.section .rodata
.align 4



irq_mode_msg:            .asciz "IRQ Mode!\n\r"
fiq_mode_msg:            .asciz "FIQ Mode!\n\r"
prefetch_abort_msg:      .asciz "Prefetch Abort!\n\r"
data_abort_msg:          .asciz "Data Abort!\n\r"
undefined_exception_msg: .asciz "Undefined Exception!\n\r"
swi_msg:                 .asciz "Software Interrupt Number: \n\r"
ascii:                   .asciz "0123456789ABCDEF"
dash:                    .asciz "-------------------------\n\r"
hex_prefix:              .asciz "0x"
dump_separator:          .asciz "  :  "



/* Data Section */
.section .data
.align 4
goto_addr:      .fill 10
/* BSS Section */
.section .bss
.align 4

c_goto:         .fill 1

























