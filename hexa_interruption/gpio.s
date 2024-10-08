/* Global Symbols */
.global GPIO1_OE
.global GPIO1_SETDATAOUT
.global GPIO1_CLEARDATAOUT
.global GPIO1_IRQSTATUS_SET_0
.global GPIO1_RISINGDETECT
.global .gpio_setup
.type   .gpio_setup, %function
.global gpio_isr 

/* GPIO */
.equ GPIO1_OE,                              0x4804C134
.equ GPIO1_SETDATAOUT,                      0x4804C194
.equ GPIO1_CLEARDATAOUT,                    0x4804C190
.equ GPIO1_IRQSTATUS_0,						0x4804C02C
.equ GPIO1_IRQSTATUS_1,						0x4804C030
.equ GPIO1_IRQSTATUS_SET_0,  				0x4804C034
.equ GPIO1_IRQSTATUS_SET_1, 				0x4804C038
.equ GPIO1_DATAIN,  						0x4804C138
.equ GPIO1_RISINGDETECT,  					0x4804C148
.equ GPIO1_FALLINGDETECT,                   0x4804C14C
.equ INTC_MIR_CLEAR3,                       0x482000E8

/* GPIO Clock Setup */
.equ CM_PER_GPIO1_CLKCTRL,                  0x44e000AC


/* Data Section */
.global flag_gpio
        .data
flag_gpio: .word 0   

/* Text Section */
.section .text,"ax"
         .code 32
         .align 4
/********************************************************
GPIO SETUP
********************************************************/
.gpio_setup:
    /* Initialize flag_gpio */
    ldr r0, =flag_gpio
    mov r1, #0
    str r1, [r0]
    /* set clock for GPIO1, TRM 8.1.12.1.31 */
    ldr r0, =CM_PER_GPIO1_CLKCTRL
    ldr r1, =0x40002
    str r1, [r0]

    /* set pin 21 for output, led USR0, TRM 25.3.4.3 */
    ldr r0, =GPIO1_OE
    ldr r1, [r0]
    bic r1, r1, #(0xf<<21)
    str r1, [r0]

    /*ldr r0, =GPIO1_OE
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

.global .led_ON_1
.type .led_ON_1, %function
.led_ON_1:
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bx lr

.global .led_OFF_1
.type .led_OFF_1, %function
.led_OFF_1:
    
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bx lr

.global .led_ON_2
.type .led_ON_2, %function
.led_ON_2:
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    bx lr

.global .led_OFF_2
.type .led_OFF_2, %function
.led_OFF_2:
    
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    bx lr

.global .led_ON_3
.type .led_ON_3, %function
.led_ON_3:
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    bx lr

.global .led_OFF_3
.type .led_OFF_3, %function
.led_OFF_3:
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    bx lr

.global .led_ON_4
.type .led_ON_4, %function
.led_ON_4:
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    bx lr

.global .led_OFF_4
.type .led_OFF_4, %function
.led_OFF_4:
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    bx lr




    
