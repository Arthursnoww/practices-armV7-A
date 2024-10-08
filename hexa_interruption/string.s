/* Global Symbols */

.global .to_lower_case
.global .strcmp
.global .compate_to_string
.type .to_lower_case ,%function
.type .strcmp , %function


/*-------------------------------------------------------------------*/
/*RECEIVES: tamanho da string em r1, endereço da string em r0        */
/*REQUERES:                                                          */
/*RETURNS:  string formatada em minusculo e -1(fail)|1(sucess) em r0 */
/*-------------------------------------------------------------------*/
.to_lower_case:
stmfd sp!,{r2-r5,lr}

    mov r2,r0 //aux r2
    mov r5, #0 //indice do vetor de string
    cmp r1,#0 // caso a string seja vazia
    bne .convert
    mov r0,#-1 // se for vazia retornará erro(-1)
    b fim_low
    .convert: //ira ver char por char se está dentro do range "a-z"(não precisa converter) 
              //"A-Z"(precisa converter), ou fora desse range(caso retorna -1)
        ldrb r3, [r2],#1
        cmp r3,#'A'
        bge condition
        mov r0,#-1
        b fim_low
        condition:
            cmp r3,#'Z'
            ble convert_true
            cmp r3,#'a'
            bge lower
            mov r0,#-1
            b fim_low
        lower:
            cmp r3,#'z'
            ble lower_true
            mov r0,#-1
            b fim_low
        convert_true:
            add r3,#32
            //add r0,r5 // index vector
            strb r3,[r0,r5] //store the char received for is buffer
        lower_true:
            add r5,#1
            sub r1,#1
            cmp r1,#0
        bge .convert
    mov r0,#1
    fim_low:
ldmfd sp!,{r2-r5,pc}

/*-------------------------------------------------------------------*/
/*RECEIVES: tamanho da string1 em r1,tamanho da                      */
/*sting2 em r2,endereço da string1 em r3, endereço da string2 em r4  */
/*REQUERES:                                                          */
/*RETURNS: <0>se igual, <1>se diferente                              */
/*-------------------------------------------------------------------*/
.strcmp:
stmfd sp!,{r1-r7,lr}
    cmp r2,r1
    bgt not_equal
    .loop:
        ldrb r5,[r3],#1
        ldrb r6,[r4],#1
        cmp r5,r6
        bne not_equal
        sub r1,#1
        sub r2,#1
        cmp r2,#0
        bne .loop
        cmp r1,#0
        bne test_space
        rete:
            mov r0,#0
            b strcmp_fim
        not_equal:
            mov r0,#1
            b strcmp_fim
        test_space:
            ldrb r5,[r3],#1
            cmp r5,#' '
            beq rete
            mov r0,#1
        strcmp_fim:

ldmfd sp!,{r1-r7,pc}


.compate_to_string:
stmfd sp!,{r0-r5,lr}
    //comparar duas strings
    ldr r1,=_str1
    mov r2,#0
    ldr r0,=msn
    bl .print_string
    get_str1: // receber a primeira string (usando pullin)
        bl .uart_getc
        cmp r0,#'\r'
        beq .0
        strb r0,[r1],#1
        add r2,#1
        bl .uart_putc
        b get_str1
    .0:
    ldr r0,=CRLF
    bl .print_string

    ldr r1,=_size_str1
    strb r2,[r1],#1

    ldr r1,=_str2
    mov r2,#0
    ldr r0,=msn
    bl .print_string
    get_str2: // receber a segunda string (usando pullin)
        bl .uart_getc
        cmp r0,#'\r'
        beq .1
        strb r0,[r1],#1
        add r2,#1
        bl .uart_putc
        b get_str2
    .1:
    ldr r0,=CRLF
    bl .print_string

    ldr r1,=_size_str2
    strb r2,[r1],#1


    //convertendo as duas strings para minusculo(LOWER_CASE)
    ldr r2,=_size_str1
    ldrb r1,[r2],#1
    ldr r0,=_str1
    bl .to_lower_case

    ldr r2,=_size_str2
    ldrb r1,[r2],#1
    ldr r0,=_str2
    bl .to_lower_case

    //comparando as duas strings
    ldr r0,=msn
    bl .print_string
    ldr r2,=_size_str1
    ldrb r0,[r2],#1
    ldr r2,=_size_str2
    ldrb r1,[r2],#1
    ldr r2,=_str1
    ldr r3,=_str2
    bl .strcmp

    //imprime o resultado
    cmp r0,#0
    beq print_equal
    blt print_menor
    ldr r0,=first
    b imprime
    print_equal:
        ldr r0,=igual
        b imprime
    print_menor:
        ldr r0,=second

    imprime:
        bl .print_string
    
ldmfd sp!,{r0-r5,pc}

/* Read-Only Data Section */
.section .rodata
.align 4
first:                   .asciz "a primeira e menor\n\r"
second:                  .asciz "a primeira e maior\n\r"
igual:                   .asciz "a duas são iguais\n\r"
msn:                     .asciz "comare_to@~: "

/* BSS Section */
.section .bss
.align 4
_str1:   .fill 0x16
_str2:   .fill 0x16
_size_str1: .fill 0x1
_size_str2: .fill 0x1




