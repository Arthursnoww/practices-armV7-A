.global .cmd_msn
.global msn
.global .hello

.global CRLF
.global .exibir
.global .prompt
.type .prompt, %function
.global tres

.prompt:
stmfd sp!,{r1-r5,lr}
    //test help
    mov r2,#4
    ldr r4,=help
    bl .strcmp
    cmp r0,#0
    bne test_hello
    bl .help
    b exit

test_hello:  
    mov r2,#5
    ldr r4,=hello
    bl .strcmp
    cmp r0,#0
    bne test_led_on
    bl .hello
    b exit

test_led_on:
    mov r2,#6
    ldr r4,=led_on
    bl .strcmp
    cmp r0,#0
    bne test_led_off
    bl .led_on
    b exit

test_led_off:
    mov r2,#7
    ldr r4,=led_off
    bl .strcmp
    cmp r0,#0
    bne test_blink
    bl .led_off
    b exit

test_blink:
    mov r2,#5
    ldr r4,=blink
    bl .strcmp
    cmp r0,#0
    bne test_led
    bl .blink
    b exit

test_led:
    mov r2,#3
    ldr r4,=led
    bl .strcmp
    cmp r0,#0
    bne test_hexa
    bl .led
    b exit

test_hexa:
    mov r2,#4
    ldr r4,=hexa
    bl .strcmp
    cmp r0,#0
    bne test_time
    bl .hexa
    mov r0,#1
    b exit


test_time:
    mov r2,#4
    ldr r4,=time
    bl .strcmp
    cmp r0,#0
    bne test_set_time
    bl .time
    b exit

test_set_time:
    mov r2,#8
    ldr r4,=set_time
    bl .strcmp
    cmp r0,#0
    bne test_binary
    bl .set_time
    b exit

test_binary:
    mov r2,#6
    ldr r4,=binary
    bl .strcmp
    cmp r0,#0
    bne test_reset
    bl .binary
    b exit

test_reset:
    mov r2,#5
    ldr r4,=reset
    bl .strcmp
    cmp r0,#0
    bne cmd_invalid
    bl .reset
    b exit

cmd_invalid:
    bl .invalid

    exit:
ldmfd sp!,{r1-r5,pc}

.help:
stmfd sp!,{r0-r1,lr}
    ldr r0,=CRLF
    bl .print_string
    ldr r0,=help_msn
    bl .print_string
ldmfd sp!,{r0-r1,pc}

.hello:
stmfd sp!,{r0-r1,lr}
    ldr r0,=CRLF
    bl .print_string
    ldr r0,=hello_msn
    bl .print_string
ldmfd sp!,{r0-r1,pc}

.led_on:
stmfd sp!,{r0-r1,lr}
    bl .led_ON_1
ldmfd sp!,{r0-r1,pc}

.led_off:
stmfd sp!,{r0-r1,lr}
    bl .led_OFF_1
ldmfd sp!,{r0-r1,pc}

.blink:
stmfd sp!,{r0-r5,lr}
    //verifica se há argumentos
    ldr r2,=_c_buffer
    ldrb r1,[r2]
    mov r0,#6
    sub r1,r0
    cmp r1,#1
    bge blink_true
    //não há argumentos validos
    ldr r0,=args_invalid_msn
    bl .print_string
    b blink_fim

    blink_true:
        ldr r2,=_buffer
        add r2,#6
        .1b: // vai acumulando o valor que foi digitado no blink
            ldrb r3,[r2],#1
            mov r0,r3
            sub r0,#'0'
            bl .acumulator
            sub r1,#1
            cmp r1,#0
            bne .1b
        
        blink_on:// blink
            bl .led_ON_1
            bl .delay_1s
            bl .led_OFF_1
            bl .delay_1s
            sub r5,#1
            cmp r5,#0
            bne blink_on

    blink_fim:
ldmfd sp!,{r0-r5,pc}

/*-------------------------------------------------------------------*/
/*RECEIVES: int em r0                                                */
/*REQUERES:                                                          */
/*RETURNS : valor acumulado em r5                                    */
/*-------------------------------------------------------------------*/
.acumulator:
stmfd sp!,{r0-r4,lr}
    mov r1,#10
    mov r4,r5
    mul r5,r4,r1
    add r5,r0
ldmfd sp!,{r0-r4,pc}

.led:
stmfd sp!,{r0-r1,lr}
        //verifica se há argumentos
    ldr r2,=_c_buffer
    ldrb r1,[r2]
    mov r0,#4
    sub r1,r0
    cmp r1,#1
    bge led_true
    //não há argumentos validos
    ldr r0,=args_invalid_msn
    bl .print_string
    b led_fim

    led_true:
        ldr r2,=_buffer
        add r2,#4
        .1lb: // vai acumulando o valor que foi digitado no led
            ldrb r3,[r2],#1
            mov r0,r3
            sub r0,#'0'
            bl .acumulator
            sub r1,#1
            cmp r1,#0
            bne .1lb
        //apaga todos os leds
        ldr r0, =GPIO1_CLEARDATAOUT
        mov r4, #(0xf<<21)
        str r4, [r0]
        //acede os leds de acordo com o numero digitado
        mov r3, r5, lsl #21
        ldr r0, =GPIO1_SETDATAOUT
        str r3, [r0]     

    led_fim:
ldmfd sp!,{r0-r1,pc}

.time:
stmfd sp!,{r0-r1,lr}
    ldr r0,=CRLF
    bl .print_string
    bl .print_time
ldmfd sp!,{r0-r1,pc}

.set_time:
stmfd sp!,{r0-r5,lr}
    ldr r2,=_c_buffer
    ldrb r1,[r2]
    mov r0,#9
    sub r1,r0
    cmp r1,#8
    beq set_time_true
    ldr r0,=args_invalid_msn
    bl .print_string
    b set_time_fim
    set_time_true:
        ldr r3,=RTC_BASE
        ldr r1,=_buffer
        add r1,#9
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r2,r0
        mov r2,r2,lsl #4
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r4,r0
        add r2,r2,r4
        str r2,[r3,#0x8] //update hour

        ldrb r0,[r1],#1
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r2,r0
        mov r2,r2,lsl #4
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r4,r0
        add r2,r2,r4
        str r2,[r3,#0x4] //update minute

        ldrb r0,[r1],#1
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r2,r0
        mov r2,r2,lsl #4
        ldrb r0,[r1],#1
        sub r0,#48
        bl .mod_div
        mov r4,r0
        add r2,r2,r4
        str r2,[r3,#0x0] //update seconds

    set_time_fim:
ldmfd sp!,{r0-r5,pc}

.binary:
stmfd sp!,{r0-r1,lr}
    bl .delay_1s
    bl .led_OFF_1
    bl .led_OFF_2
    bl .led_OFF_3
    bl .led_OFF_4

    bl .delay_1s
    bl .led_ON_4

    bl .delay_1s
    bl .led_OFF_4
    bl .led_ON_3

    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_3
    bl .led_OFF_4
    bl .led_ON_2
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_4
    bl .led_ON_3
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_ON_1
    bl .led_OFF_2
    bl .led_OFF_3
    bl .led_OFF_4
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_4
    bl .led_ON_3
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_3
    bl .led_OFF_4
    bl .led_ON_2
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_4
    bl .led_ON_3
    
    bl .delay_1s
    bl .led_ON_4
    
    bl .delay_1s
    bl .led_OFF_1
    bl .led_OFF_2
    bl .led_OFF_3
    bl .led_OFF_4
ldmfd sp!,{r0-r1,pc}

.hexa:
    stmfd sp!,{r0-r1,lr}
    bl .delay_1s
    ldr r0,=CRLF
    bl .print_string
    ldr r0,=zero
    bl .print_string

    bl .delay_1s
    ldr r0,=um
    bl .print_string

    bl .delay_1s
    ldr r0,=dois
    bl .print_string

    bl .delay_1s
    ldr r0,=tres
    bl .print_string
    
    bl .delay_1s
    ldr r0,=quatro
    bl .print_string

    bl .delay_1s
    ldr r0,=cinco
    bl .print_string
    
    bl .delay_1s
    ldr r0,=seis
    bl .print_string
    
    bl .delay_1s
    ldr r0,=sete
    bl .print_string
    
    bl .delay_1s
    ldr r0,=oito
    bl .print_string
   
    bl .delay_1s
    ldr r0,=nove
    bl .print_string
    
    bl .delay_1s
    ldr r0,=aa
    bl .print_string
    
    bl .delay_1s
    ldr r0,=bb
    bl .print_string
  
    bl .delay_1s
    ldr r0,=cc
    bl .print_string
    
    bl .delay_1s
    ldr r0,=dd
    bl .print_string
    
    bl .delay_1s
    ldr r0,=ee
    bl .print_string
   
    
    bl .delay_1s
    ldr r0,=ff
    bl .print_string
    
    bl .delay_1s
    ldr r0,=CRLF
    bl .print_string

ldmfd sp!,{r0-r1,pc}

.reset:
stmfd sp!,{r0-r1,lr}
    bl ._reset
ldmfd sp!,{r0-r1,pc}

.invalid:
stmfd sp!,{r0-r1,lr}
    ldr r0,=CRLF
    bl .print_string
    ldr r0,=invalid_msn
    bl .print_string
ldmfd sp!,{r0-r1,pc}

.cmd_msn:
stmfd sp!,{r0-r1,lr}
    ldr r0,=CRLF
    bl .print_string
    ldr r0,=msn
    bl .print_string
ldmfd sp!,{r0-r1,pc}

.exibir:
stmfd sp!,{r0-r1,lr}
    mov r0,#'\r'
    bl .uart_putc
    ldr r0,=msn
    bl .print_string
    ldr r2,=_c_buffer
    ldrb r1,[r2]
    ldr r0,=_buffer
    bl .print_nstring
ldmfd sp!,{r0-r1,pc}
/* Read-Only Data Section */

.section .rodata
.align 4
msn:                     .asciz "arthur@~: "
invalid_msn:             .asciz "comando invalido!!!"
args_invalid_msn:        .asciz "argumento invalido!!!"
CRLF:                    .asciz "\n\r"
hello_msn:               .asciz "Hello world!!!"
help_msn:                .asciz "COMANDOS:\n\rHELLO -> digite 'hello' para imprimir 'hello world!!!';\n\rLED ON -> digite 'led on';\n\rLED OFF -> digite 'led off';\n\rBLINK -> digite 'blink <N> o 'N' é o numero de vezes que repete; \n\rHEXAS -> digite 'hexa' imprime de 0 a 15 os hexadecimais na tela e leds; \n\rHEXA EM BINARIO -> digite 'binary' para imprimir os hexas no led; \n\rHEXA LED -> digite 'led <H>' o 'H' refere-se a numero hexadecimal de 0 a 15;\n\rTIME -> digite 'time' para ver o rtc atual;\n\rSET TIME -> digite 'set time <hh:mm:ss>' configurar rtc, 'hh' é horas, 'mm' minutos e 'ss' segundos; \n\rRESET -> digite 'reset' para reiniciar a placa;\n\rINTERROMPER -> para interromper uma tarefa, basta presionar o botão."
help:                    .asciz "help"
hello:                   .asciz "hello"
led_on:                  .asciz "led on"
led_off:                 .asciz "led off"
blink:                   .asciz "blink"
led:                     .asciz "led"
time:                    .asciz "time"
set_time:                .asciz "set time"
binary:                  .asciz "binary"
hexa:                    .asciz "hexa"
reset:                   .asciz "reset"
zero:                    .asciz "0x00 "
um:                      .asciz "0x01 "
dois:                    .asciz "0x02 "
tres:                    .asciz "0x03 "
quatro:                  .asciz "0x04 "
cinco:                   .asciz "0x05 "
seis:                    .asciz "0x06 "
sete:                    .asciz "0x07 "
oito:                    .asciz "0x08 "
nove:                    .asciz "0x09 "
aa:                      .asciz "0x0A "
bb:                      .asciz "0x0B "
cc:                      .asciz "0x0C "
dd:                      .asciz "0x0D "
ee:                      .asciz "0x0E "
ff:                      .asciz "0x0F "
