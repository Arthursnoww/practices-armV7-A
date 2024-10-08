/* Global Symbols */
.global RTC_BASE
.global .rtc_setup
.global .rtc_isr
.global .rtc_to_ascii
.global .print_time

/* Registers */

.equ CM_RTC_RTC_CLKCTRL, 0x44E00800
.equ CM_RTC_CLKSTCTRL,  0x44E00804
.equ RTC_BASE, 0x44E3E000



/* Text Section */
.section .text,"ax"
         .code 32
         .align 4

/********************************************************
RTC SETUP 
********************************************************/
.rtc_setup:

    /* Save context */	
    stmfd sp!,{r0-r1,lr}

    /*  Clock enable for RTC TRM 8.1.12.6.1 */
    ldr r0, =CM_RTC_CLKSTCTRL
    ldr r1, =0x2
    str r1, [r0]
    ldr r0, =CM_RTC_RTC_CLKCTRL
    str r1, [r0]

    /* Disable write protection TRM 20.3.5.23 e 20.3.5.24 */
    ldr r0, =RTC_BASE
    ldr r1, =0x83E70B13
    str r1, [r0, #0x6c]
    ldr r1, =0x95A4F1E0
    str r1, [r0, #0x70]
    
    /* Select external clock*/
    ldr r1, =0x48
    str r1, [r0, #0x54]



    /* Interrupt setup */
    //ldr r1, =0x04     /* interrupt every second */
    //ldr r1, =0x05     /* interrupt every minute */
    //ldr r1, =0x06     /* interrupt every hour */
    //str r1, [r0, #0x48]

    /* Enable RTC */
    ldr r0, =RTC_BASE
    ldr r1, =(1<<0)
    str r1, [r0, #0x40]  

    /*rtc irq setup */
.wait_rtc_update:
    ldr r1, [r0, #0x44]
    and r1, r1, #1
    cmp r1, #0
    bne .wait_rtc_update

   
    /* RTC Interrupt configured as IRQ Priority 0 */
    //RTC Interrupt number 75
    //ldr r0, =INTC_ILR
    //ldr r1, =#0    
    //strb r1, [r0, #75] 


    /* Interrupt mask */
    //ldr r0, =INTC_BASE
    //ldr r1, =#(1<<11)    
    //str r1, [r0, #0xc8] //(75 --> Bit 11 do 3º registrador (MIR CLEAR2))

    
    /* Load context */	
    ldmfd sp!,{r0-r1,pc}

/********************************************************/

/********************************************************
Imprime hora do RTC
(see TRM 20.3.5.1 - 20.3.5.3)
********************************************************/
.rtc_to_ascii:
    stmfd sp!,{r0-r2,lr} 
    mov r2, r0
    
    and r0, r2, #0x70
    
    mov r0, r0, LSR #4
    bl .dec_digit_to_ascii
    bl .uart_putc

    and r0, r2, #0x0f
    add r0,r0,#0x30 
    bl .uart_putc

    ldmfd sp!, {r0-r2, pc}

.print_time:
    stmfd sp!,{r0-r2,lr}
    ldr r1,=RTC_BASE
    ldr r0, [r1, #8] //hours
    bl .rtc_to_ascii

    ldr r0,=':'
    bl .uart_putc

    ldr r0, [r1, #4] //minutes
    bl .rtc_to_ascii

    ldr r0,=':'
    bl .uart_putc

    ldr r0, [r1, #0] //seconds
    bl .rtc_to_ascii

    @ ldr r0,='\r'
    @ bl .uart_putc
    ldmfd sp!, {r0-r2, pc}
/********************************************************/

/********************************************************
RTC ISR
********************************************************/
.rtc_isr:
    stmfd sp!, {r0-r2, lr}

    /*print time*/
    bl .print_time

    ldmfd sp!, {r0-r2, pc}
/********************************************************/  




