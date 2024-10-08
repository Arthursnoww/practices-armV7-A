/* Global Symbols */
.global .gpio_setup
.global .leds_count
.global .delay_1s
.global .button_isr
.global flag_gpio
.global GPIO1_SETDATAOUT
.global GPIO1_CLEARDATAOUT
.type .button_isr, %function
.type .delay_1s, %function
.type .gpio_setup, %function
.type .leds_count, %function

/* Registers */

/* GPIO */
.equ GPIO1_OE, 0x4804C134
.equ GPIO1_DATAIN, 0x4804C138
.equ GPIO1_SETDATAOUT, 0x4804C194
.equ GPIO1_CLEARDATAOUT, 0x4804C190
.equ GPIO1_IRQSTATUS_SET_0, 0x4804C034
.equ GPIO1_IRQSTATUS_0, 0x4804C02c
.equ GPIO1_RISINGDETECT, 0x4804C148
.equ INTC_MIR_CLEAR3,0x482000e8
/* GPIO Clock Setup */
.equ CM_PER_GPIO1_CLKCTRL, 0x44e000AC

/* Text Section */
.section .text,"ax"
         .code 32
         .align 4

/********************************************************
GPIO SETUP
********************************************************/
.gpio_setup:
    /* set clock for GPIO1, TRM 8.1.12.1.31 */
    ldr r0, =CM_PER_GPIO1_CLKCTRL
    ldr r1, =0x40002
    str r1, [r0]

    
    /* set pin 21 for output, led USR0, TRM 25.3.4.3 */
    ldr r0, =GPIO1_OE
    ldr r1, [r0]
    bic r1, r1, #(0xf<<21)
    str r1, [r0]
    
    ldr r0, =GPIO1_OE
    ldr r1, [r0]
    orr r1, r1, #(1<<28)
    str r1, [r0]

    ldr r0, =GPIO1_IRQSTATUS_SET_0
    ldr r1, [r0]
    orr r1, r1, #(1 << 28)
    str r1, [r0]

    ldr r0, =GPIO1_RISINGDETECT
    ldr r1, [r0]
    orr r1, r1, #(1 << 28)
    str r1, [r0]

    ldr r0, =INTC_MIR_CLEAR3
    mov r1, #(0x1<<2)
    str r1, [r0]

    bx lr
/********************************************************/




/********************************************************
Blink LED BBB
********************************************************/

.global .led_ON
.type .led_ON, %function
.led_ON:
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bx lr

.global .led_OFF
.type .led_OFF, %function
.led_OFF:
    
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bx lr


.leds_count:
stmfd sp!,{r0-r2,lr}
    ldr r0, =CRLF
    bl .print_string
    mov r1, #0
    mov r2, #0
.loop:
stmfd sp!,{r0-r2}
    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .button_isr
    ldmfd sp!, {r0-r2}
    blne end1
    movhi r2,#0
    ldr r0, =GPIO1_SETDATAOUT
    mov r1,r2
    lsl r1,#21
    str r1, [r0]
    bl .delay_1s
    
    stmfd sp!,{r0-r2}
    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .button_isr
    ldmfd sp!, {r0-r2}
    blne end1

    ldr r0, =GPIO1_CLEARDATAOUT

    mov r1,r2
    lsl r1,#21
    str r1, [r0]
    add r2,#1
    bl .delay_1s
    cmp r2,#15
    ble .loop
    end1:
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
   
    ldr r1, =(1<<23)
    str r1, [r0]
    
    ldr r1, =(1<<24)
    str r1, [r0]
ldmfd sp!,{r0-r2,pc}

.button_isr:
stmfd sp!, {r0-r1, lr}
	
   LDR r0, =GPIO1_IRQSTATUS_0
   mov r1, #0x10000000
   str r1, [r0]
   
   
   ldr r0, =flag_gpio
   mov r1,#1
   str r1, [r0]
   
 
 ldmfd sp!, {r0-r1, pc}

flag_gpio: .word 0

