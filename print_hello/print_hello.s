/* UART */
.equ UART0_BASE, 0x44E09000

/* Startup Code */
_start:
    /* init */
    mrs r0, cpsr
    bic r0, r0, #0x1F @ clear mode bits
    orr r0, r0, #0x13 @ set SVC mode
    orr r0, r0, #0xC0 @ disable FIQ and IRQ
    msr cpsr, r0

    /* Print Hello World */
    adr r0, hello
    bl .print_string
    b lr
    
/********************************************************
Imprime uma string até o '\0'
// R0 -> Endereço da string
/********************************************************/
.print_string:
    stmfd sp!,{r0-r2,lr}
    mov r1, r0
.print:
    ldrb r0,[r1],#1
    and r0, r0, #0xff
    cmp r0, #0
    beq .end_print
    bl .uart_putc
    b .print
    
.end_print:
    ldmfd sp!,{r0-r2,pc}
/********************************************************/



hello: .asciz "Hello world!!!\n\r"


















