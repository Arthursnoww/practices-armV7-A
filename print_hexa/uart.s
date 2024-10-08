/* Global Symbols */
.global .uart_setup
.global .uart_putc
.global .uart_getc
.global .uart_isr
.global .buffer_to_rtc
.type .uart_getc, %function
.type .uart_putc, %function
.type .buffer_to_rtc, %function
//.type .uart_isr, %function


/* Registradores */
.equ UART0_BASE, 0x44E09000
.equ INTC_ILR,  0x48200100
.equ INTC_BASE, 0x48200000
.equ GPIO1_IRQSTATUS_0, 0x4804C02c

/* Text Section */
.section .text,"ax"
         .code 32
         .align 4
         
/********************************************************
UART0 setup (Default configuration)  
********************************************************/
.uart_setup:
    stmfd sp!,{r0,r1,lr}
    ldr r0, =UART0_BASE
    mov r1,#0x1
    strb r1,[r0,#0x4]

 

    ldr r0, =INTC_BASE
    ldr r1, =#(1<<8)    
    str r1, [r0, #0xc8] //(72 --> Bit 8 do 3º registrador (MIR CLEAR2))

    ldmfd sp!,{r0,r1,pc}

/********************************************************
UART0 PUTC (Default configuration)  
********************************************************/
.uart_putc:
    stmfd sp!,{r1-r2,lr}
    ldr     r1, =UART0_BASE

.wait_tx_fifo_empty:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<5)
    cmp r2, #0
    beq .wait_tx_fifo_empty

    strb    r0, [r1]
    ldmfd sp!,{r1-r2,pc}

/********************************************************
UART0 GETC (Default configuration)  
********************************************************/
.uart_getc:
    stmfd sp!,{r1-r2,lr}
    ldr     r1, =UART0_BASE

.wait_rx_fifo:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<0)
    cmp r2, #0
    beq .wait_rx_fifo

    ldrb    r0, [r1]
    ldmfd sp!,{r1-r2,pc}
/********************************************************/

/*
buffer_to_rtc:
recebe: 
    r3 = endereço usado do buffer
retorna:
    r0 = val hex
    r3 = endereço usado do buffer atual
*/



/********************************************************/
.uart_isr:
    stmfd sp!, {r0-r5, lr}
	
    bl .uart_getc
    bl .uart_putc
    
    cmp r0,#0xd
    moveq r0,#0
    
    ldr r1, =opcao
    
    strneb r0,[r1]

    blne .cmp_str

    bl .buffer_print
    
    ldmfd sp!, {r0-r5, pc}


.cmp_str:
	stmfd sp!,{r0-r2,lr}
	
    ldr r1, =opcao
    ldrb r0, [r1]
    
    mov r2, #0x31
    cmp r0, r2
    bleq .print_hello

    mov r2, #0x32
    cmp r0, r2
    bleq .print_numbrs

    mov r2, #0x33
    cmp r0, r2
    bleq .leds_count



    ldmfd sp!,{r0-r2,pc}

.print_hello:
    stmfd sp!,{r0,lr}
    ldr r0, =CRLF
    bl .print_string
    ldr r0, =hello
    bl .print_string
    ldmfd sp!,{r0,pc}


   

.buffer_print:

    stmfd sp!, {r0, lr}  

    ldr r0, =CRLF
    bl .print_string
    ldr r0, =arthur
    bl .print_string

    ldmfd sp!, {r0, pc}


.print_numbrs:
    stmfd sp!,{r0-r4,lr}
    ldr r0, =CRLF
    bl .print_string
    mov r3, #0
    mov r0, #0x30
    loop:
    stmfd sp!,{r0}
    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .button_isr
    ldmfd sp!, {r0}
    blne end
        bl .uart_putc
        bl .delay_1s
        cmp r0,#0x39
        addeq r0, #0x07
        add r0,#0x01

        
        mov r4, r0

        mov r0, #0x20
        bl .uart_putc
        
        mov r0, r4

        add r3, #1
        cmp r3, #15

    ble loop
    end:
    ldmfd sp!, {r0-r4, pc}
    
.section .bss
.align 4
count:  .word 0
idex:   .word 0
opcao:  .fill 32


