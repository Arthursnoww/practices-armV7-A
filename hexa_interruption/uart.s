/* Global Symbols */
.global .uart0_int_setup
.global .uart_putc
.global .uart_getc
.global .uart_isr
.global _buffer
.global _c_buffer
.type .uart_getc, %function
.type .uart_putc, %function
.type .uart_isr , %function


/* Registradores */
.equ UART0_BASE, 0x44E09000
.equ INTC_BASE,  0x48200000
.equ INTC_ILR,   0x48200100

/* Text Section */
//.section .text,"ax"
//         .code 32
//         .align 4
         
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
    stmfd sp!,{r1-r5,lr}
    ldr     r1, =UART0_BASE

.wait_rx_fifo:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<0)
    cmp r2, #0
    beq .wait_rx_fifo

    ldrb    r0, [r1]
    ldmfd sp!,{r1-r5,pc}
/********************************************************/

/********************************************************/
/********************************************************
UART0 INTERRUPT SETUP 
********************************************************/
.uart0_int_setup:
stmfd sp!,{r0-r2,lr}
    mov r5,#0
    ldr r4,=_c_buffer
    strb r5,[r4],#1

    ldr r0,=UART0_BASE
    add r0,#0x4 // reg UART0_IER for active interruption

    mov r1, #(1<<0)
    str r1,[r0]

    ldr r0, =INTC_ILR
    mov r1, #0 // priority 0 for interruption number 72
    strb r1, [r0, #72]

    ldr r0, =INTC_BASE
    mov r1,#(1<<8)   
    str r1, [r0, #0xc8] //(72--> Bit 8 do 3º registrador (MIR CLEAR2))

ldmfd sp!,{r0-r2,pc}


/********************************************************
UART0 INTERRUPT ISR
********************************************************/
/*-------------------------------------------------------------------*/
/*RECEIVES:                                                          */
/*REQUERES:                                                          */
/*RETURNS : 0(faz nada) 1(goto)                                      */
/*-------------------------------------------------------------------*/
.uart_isr:
stmfd sp!,{r2-r5,lr}
    bl .uart_getc
    //test de <ENTER>
    cmp r0,#'\r'
    beq tratar_comando
    //test de <BACKSPACE>
    cmp r0,#8
    beq backspace

    bl preencher
    bl .exibir
    b .exit

    backspace:
        ldr r2,=_c_buffer
        ldrb r1,[r2]
        cmp r1,#0
        beq vazio
        sub r1,#1
        ldr r2,=_c_buffer
        strb r1,[r2],#1
        bl .exibir
        mov r0,#' '
        bl .uart_putc
        bl .exibir
        b .exit
        vazio:
            mov r0,#'\r'
            bl .uart_putc
            ldr r0,=msn
            bl .print_string
            b .exit

    tratar_comando:
        ldr r2,=_c_buffer
        ldrb r1,[r2]
        ldr r3,=_buffer
        bl .prompt
        mov r5,r0
        mov r1,#20
        ldr r0,=_buffer
        bl .memory_clear
        mov r0,r5
        mov r5,#0
        ldr r4,=_c_buffer
        strb r5,[r4],#1
        bl .cmd_msn
        b .exit
    
    .exit:

ldmfd sp!,{r2-r5,pc}


preencher:
stmfd sp!,{r1-r3,lr}
    ldr r1,=_c_buffer
    ldrb r2,[r1],#1
    ldr r1,=_buffer
    strb r0,[r1,r2]
    add r2,#1
    ldr r1,=_c_buffer
    strb r2,[r1],#1
ldmfd sp!,{r1-r3,pc}


/* BSS Section */
.section .bss
.align 4
.equ BUFFER_SIZE, 20
_buffer:   .fill BUFFER_SIZE
_c_buffer: .fill 0x1
_cursor:   .fill 0x1



