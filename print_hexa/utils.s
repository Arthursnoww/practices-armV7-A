/* Global Symbols */
.global div
.global mul
.global .memory_clear
.global .memory_dump
.global .delay
.global .delay_1s
.global .dec_digit_to_ascii
.global .ascii_to_dec
.global _print_string
.global .print_string
.global .print_nstring
.global .hex_to_ascii
.global .int_to_ascii
.global .hex_digit_to_ascii
.global .memcmp
.global .new_line

.type div, %function
.type .memory_clear, %function
.type .memory_dump, %function
.type .dec_digit_to_ascii, %function
.type .print_nstring, %function
.type .hex_digit_to_ascii, %function
.type _print_string, %function

/* Text Section */
.section .text,"ax"
         .code 32
         .align 4
         


/********************************************************
Converte int to ascii e imprime
//Input  --> R0
********************************************************/
.int_to_ascii:
    stmfd sp!,{r0-r2,lr}
    mov r2, #0
0:
    mov r1, #10
    cmp r0, r1
    blt 1f
    bl div
    stmfd sp!,{r1}
    add r2, r2, #1
    b 0b
1:
    add r0, r0, #0x30
    bl .uart_putc
    cmp r2, #0
    ble 2f
    sub r2, r2, #1
    ldmfd sp!, {r0}    
    b 1b
2:
    ldmfd sp!, {r0-r2, pc}
/********************************************************/

/*
r0 = var x
r1 = var y
*/
mul:
    stmfd sp!,{r2-r4,lr}
    mov r2,r1
    0:
        add r3,r0
        cmp r2,#1
        sub r2,#1
    bne 0b
    mov r0,r3
    
    ldmfd sp!, {r2-r4, pc}

/********************************************************
Division
//Input  --> Num: R0, Den: R1 
//Output --> Quot: R0, Rem: R1
********************************************************/
div:
    stmfd sp!,{r2-r4,lr}
    mov r2, r0
    mov r3, r1
    mov r0, #0
div_loop:
    cmp r2,r3
    blt fim_div 
    sub r2, r2, r3
    add r0, r0, #1
    b div_loop
fim_div:
    mov r1, r2
    ldmfd sp!, {r2-r4, pc}
/********************************************************/

/********************************************************
Limpa memória
R0-> Endereço
R1-> Tamanho
/********************************************************/
.memory_clear:
    stmfd sp!,{r0-r2,lr}
    add     r1, r1, r0
    mov     r2, #0
0:
    cmp     r0, r1
    strlt   r2, [r0], #4
    blt     0b
    ldmfd sp!,{r0-r2,pc}
/********************************************************/

/********************************************************
Memory Dump
------------
Imprime o conteúdo da memória.
R0 -> Endereço inicial
R1 -> Quantidade de endereços 
********************************************************/
.memory_dump:
    stmfd sp!,{r0-r3,lr}
    mov r2, r0
    mov r3, r1

dump_loop:  
    // Imprime o endereço
    ldr r0, =hex_prefix
    mov r1, #2
    bl .print_nstring
    mov r0, r2
    bl .hex_to_ascii 

    // Imprime o separador '  :  '
    ldr r0, =dump_separator
    mov r1, #5
    bl .print_nstring

    // Imprime o conteúdo
    ldr r0, =hex_prefix
    mov r1, #2
    bl .print_nstring
    ldr r0, [r2], #4
    bl .hex_to_ascii
    
    //Salta linha
    ldr r0,=CRLF
    mov r1, #2
    bl .print_nstring

    //Verifica se já terminou
    subs r3, r3, #4
    bne dump_loop

    ldmfd sp!,{r0-r3,pc}


/********************************************************
DELAY
********************************************************/
.delay:
    ldr r1, =0xfffffff
.wait:
    sub r1, r1, #0x1
    cmp r1, #0
    bne .wait
    bx lr
/********************************************************/
.delay_1s:
    stmfd sp!,{r0-r2,lr}
    ldr  r0,=RTC_BASE
    ldrb r1, [r0, #0] //seconds
.wait_second:
    ldrb r2, [r0, #0] //seconds
    cmp r2, r1
    beq .wait_second
    ldmfd sp!,{r0-r2,pc}



/********************************************************/
.dec_digit_to_ascii:
	add r0,r0,#0x30
	bx lr

.ascii_to_dec:
    sub r0,r0,#0x30
    bx lr

.hex_digit_to_ascii:
       stmfd sp!,{r0-r2,lr} 
       ldr r1, =ascii
       ldrb r0, [r1, r0]
       
       ldmfd sp!, {r0-r2, pc}
/********************************************************/

/********************************************************
Converte HEX para ASCCI
********************************************************/
.hex_to_ascii:
    stmfd sp!,{r0-r3,lr}
    mov r1, r0

    mov r0, #0
    mov r3, #28
    ldr r2, =ascii

ascii_loop:
    mov r0, r1, LSR r3
    and r0, r0, #0x0f 
    ldrb r0, [r2, r0]
    bl .uart_putc
    subs r3, r3, #4
    bne ascii_loop
    mov r0, r1
    and r0, r0, #0x0f 
    ldrb r0, [r2, r0]
    bl .uart_putc

    ldmfd sp!,{r0-r3,pc}
/********************************************************/
// Calcula o checksum de uma região
//RO - Ponteiro para mem1
//R1 - quantidade de bytes
//Retorno
// R0: Soma dos bytes (32 bits)
/********************************************************/
check_sum:
    stmfd sp!,{r1-r3,lr}
    
    mov r2, r0
    mov r0, #0

 0: cmp r1,#0
    beq 1f
    ldrb r3, [r2], #1
    add r0,r0,r3
    sub r1,r1,#1
    b 0b
1:    
    ldmfd sp!,{r1-r3,pc}
/********************************************************/

/********************************************************/
// Compara o checksum de duas regiões de memória
//RO - Ponteiro para mem1
//R1 - ponteiro para mem2
//R2 - quantidade de bytes
//Retorno
//R0 = 0 -> checksums iguais
//R0 != 0 -> checksums diferentes
/********************************************************/
.memcmp_chksum:
	stmfd sp!,{r1-r4,lr}

	mov r4, r1
	
	mov r1, r2
	bl check_sum
	
	mov r3, r0 //checksum da mem1
	
	mov r0, r4
	bl check_sum //checksum da mem2
	
	sub r0, r3,r0
    
   	ldmfd sp!,{r1-r4,pc}
/********************************************************/

/********************************************************/
// Compara o conteúdo de duas regiões de memória
//RO - Ponteiro para mem1
//R1 - ponteiro para mem2
//R2 - quantidade de bytes
//Retorno
//R0 = 0 -> conteudos iguais
//R0 != 0 -> conteudos diferentes
/********************************************************/
.memcmp:
    stmfd sp!,{r1-r6,lr}

    mov r3, r0
    mov r4, r1 
    mov r0, r2
 0: cmp r2,#0
    beq 1f
    ldrb r5, [r3], #1
    ldrb r6, [r4], #1
   
    cmp r5, r6
    bne 1f
	sub r0,r0,#1
    sub r2,r2,#1
    b 0b
1:    
   ldmfd sp!,{r1-r6,pc}
/********************************************************/

/********************************************************
Imprime uma string até o '\0'
// R0 -> Endereço da string
/********************************************************/
_print_string:
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

/********************************************************
Imprime uma string até o '\0'
// R0 -> Endereço da string
/********************************************************/

.new_line:
    stmfd sp!,{r1-r2,lr}
    ldr r1, =trm
    mov r2, r1
.printr1:
    ldrb r1,[r2],#1
    and r1, r1, #0xff
    cmp r1, #0
    beq .end_printr1
    bl .uart_putc
    b .printr1
    
.end_printr1:
    ldmfd sp!,{r1-r2,pc}


trm:                     .asciz "> "
/********************************************************/



/********************************************************
Imprime n caracteres de uma string
// R0 -> Endereço da string R1-> Número de caracteres
/********************************************************/
.print_nstring:
    stmfd sp!,{r0-r2,lr}
    mov r2, r0
.print_n:
    ldrb r0,[r2],#1
    bl .uart_putc
    subs r1, r1, #1
    beq .end_print_n
    b .print_n
    
.end_print_n:
    ldmfd sp!,{r0-r2,pc}

    
/********************************************************/

/********************************************************
PRINTF 
********************************************************/
.global _printf
.type _printf, %function

_printf:
    ldr r12, =buffer_printf
    stmia r12!, {r4-r11, lr}  
    
    mov r4, r0
    mov r5, #0 
    
0:  ldrb r0, [r4], #1  
    cmp r0, #'%'
    beq 1f

    cmp r0, #0
    beq 3f

    bl .uart_putc
    b 0b

1:  add r5, r5, #1

    cmp r5, #1
    moveq r0, r1
    beq 2f

    cmp r5, #2
    moveq r0, r2
    beq 2f

    cmp r5, #3
    moveq r0, r3
    beq 2f

    ldmfd sp!, {r0}
  
2:
    ldrb r6, [r4], #1

    cmp r6, #'s'
    ldreq r7, =.print_string

    cmp r6, #'d'
    ldreq r7, =.int_to_ascii

    cmp r6, #'c'
    ldreq r7, =.uart_putc

    cmp r6, #'x'
    ldreq r7, =.hex_to_ascii

    blx r7
    b 0b

3:   
    ldmdb r12!, {r4-r11, pc}
/********************************************************/


/* BSS Section */
.section .bss
.align 4

.equ SIZE, 128
.global buffer_printf
buffer_printf: .fill SIZE


