/* Clock */
.equ CM_PER_GPIO1_CLKCTRL, 0x44e000AC
.equ CM_RTC_RTC_CLKCTRL, 0x44E00800
.equ CM_RTC_CLKSTCTRL,  0x44E00804
/* Watch Dog Timer */
.equ WDT_BASE, 0x44E35000
/* GPIO */
.equ GPIO1_OE, 0x4804C134
.equ GPIO1_SETDATAOUT, 0x4804C194
.equ GPIO1_CLEARDATAOUT, 0x4804C190

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
/* RTC */
.equ RTC_BASE, 0x44E3E000
/* UART */
.equ UART0_BASE, 0x44E09000
.equ UART0_RHR, 0x0
.equ UART0_IER, 0x4

/* registradores para tratar irq */
.equ INTC_BASE, 0x48200000
.equ INTC_ILR,  0x48200100

.equ VECTOR_BASE, 0x4030CE00

/****************configurando pilha ***********/
.set StackModeSize,  0x100
.equ StackUSR, (_stack_end - 0*StackModeSize)
.equ StackFIQ, (_stack_end - 1*StackModeSize)
.equ StackIRQ, (_stack_end - 2*StackModeSize)
.equ StackSVC, (_stack_end - 3*StackModeSize)
.equ StackABT, (_stack_end - 4*StackModeSize)
.equ StackUND, (_stack_end - 5*StackModeSize)
.equ StackSYS, (_stack_end - 6*StackModeSize)
/***32bits *******/
.section .text,"ax"
         .code 32
         .align 4

/* tabela de vetores para interrupção */
/* A tabela de vetores de exceção é um mecanismo usado para 
direcionar o processador para o código apropriado quando ocorrem diferentes tipos de exceções ou interrupções. */
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

_start:
/* Configurar processador, cp15 */
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
    /* interrupção swi */
    ldr r0, _swi
    ldr r1, =.swi_handler
    str r1, [r0]
    /* interrupção irq */
    ldr r0, =_irq
    ldr r1, =.irq_handler
    str r1, [r0]
    
    /* Hardware setup */
    bl .gpio_setup
    bl .disable_wdt
    bl .rtc_setup

    /* ativar irq */
    mrs r0, cpsr
    and r0, r0, #~(CPSR_I)
    msr cpsr, r0
    


    mov r5,#0

/* leds */
.main_loop:
    
    /* logical 1 turns on the led, TRM 25.3.4.2.2.2 */
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bl .delay_1s
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    bl .delay_1s

    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    bl .delay_1s

    b .main_loop    

.cp15_configure:
    stmfd sp!,{r0-r2,lr} //empilhando
    mrc    p15, 0, r0, c1, c0, 0    // pega o registrador cp15 e coloca em r0
    bic    r0, #(1 << 13)   //bit 13 do SCTRL para 0.         
    mcr    p15, 0, r0, c1, c0, 0  // coloca o registrador alterado no SCTRL

    ldr     r0, =_vector_table  //Carrega o endereço da tabela de vetores
    mcr     p15, 0, r0, c12, c0, 0  //coloca o registrador alterado no VBA

    ldmfd sp!,{r0-r2,pc}

/****************************clock e multiplexação****************************/
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

.gpio_isr:
    stmfd sp!, {r0-r1, lr}
    
    ldr r0, =GPIO1_IRQSTATUS_0
    mov r1, #0x10000000
    str r1, [r0]

    ldr r0,=hello
    bl .print_string

    ldmfd sp!,{r0-r1,pc}
    bx lr
/**********************Converte HEX para ASCCI ****************/
.hex_to_ascii:
    stmfd sp!,{r0-r3,lr}  
    mov r1, r0            
    mov r0, #0            
    mov r3, #28           // Inicializa r3 com 28.
    ldr r2, =ascii        // Carrega o endereço de uma tabela ASCII em r2

ascii_loop:
    mov r0, r1, LSR r3    // armazenando o resultado em r0.
    and r0, r0, #0x0f     // mask 
    ldrb r0, [r2, r0]     // Carrega o caractere ASCII 
    bl .uart_putc         
    subs r3, r3, #4       
    bne ascii_loop         

    mov r0, r1          
    and r0, r0, #0x0f     // Mantém apenas os 4 bits menos significativos 
    bl .uart_putc         // Chama a função para enviar o caractere ASCII para um dispositivo de saída.

    ldmfd sp!,{r0-r3,pc} 


/**************************Imprime uma string até o '\0'*******************/
.print_string:
    stmfd sp!,{r0-r2,lr} //empilha
    mov r1, r0 // pega o indice da array
.print:
    ldrb r0,[r1],#1 //carrega byte e incrementa para o proximo
    and r0, r0, #0xff //mask de bits
    cmp r0, #0
    beq .end_print
    bl .uart_putc // envia para uart
    b .print
    
.end_print:
    ldmfd sp!,{r0-r2,pc}


/************************delay RTC - Real-Time Clock******************/
.delay_1s:
    stmfd sp!,{r0-r2,lr}
    ldr  r0,=RTC_BASE // carrega RTC
    ldrb r1, [r0, #0] 
.wait_second: // delay de 1 segundo
    ldrb r2, [r0, #0] 
    cmp r2, r1
    beq .wait_second
    ldmfd sp!,{r0-r2,pc}

.uart0_int_setup:
stmfd sp!,{r0-r2,lr}

    ldr r0,=UART0_BASE
    add r0,#0x4 // ativa interrupção, UART0_IER

    mov r1, #(1<<0)
    str r1,[r0]

    ldr r0, =INTC_BASE //base de interupções
    mov r1,#(1<<8)   //define bit do binario
    str r1, [r0, #0xc8] //configura MIR CLEAR2

ldmfd sp!,{r0-r2,pc}

/****************configurando rtc************************/
.rtc_setup:
/*Para desabilitar o registro RTC
proteção contra gravação, o valor de 83E7 0B13h deve ser escrito em KICK0R, seguido pelo valor de 95A4
F1E0h escrito para KICK1R. A proteção contra gravação do registro RTC é habilitada quando qualquer valor é gravado no KICK0R.*/
    stmfd sp!,{r0-r1,lr}
    /*  Clock enable for RTC TRM 8.1.12.6.1 */
    ldr r0, =CM_RTC_CLKSTCTRL //clock rtc
    ldr r1, =0x2
    str r1, [r0]
    ldr r0, =CM_RTC_RTC_CLKCTRL
    str r1, [r0] //habilitar o clock RTC.
    /* Disable write protection TRM 20.3.5.23 e 20.3.5.24 */
    ldr r0, =RTC_BASE
    ldr r1, =0x83E70B13 //
    str r1, [r0, #0x6c]
    ldr r1, =0x95A4F1E0 //
    str r1, [r0, #0x70]
    /* ativa clock externo*/
    ldr r1, =0x48
    str r1, [r0, #0x54]
/* O SECONDS_REG é usado para programar o valor de segundos necessário da hora atual. Os segundos são
armazenado como formato BCD. No formato BCD, os números decimais de 0 a 9 são codificados com seu binário
equivalente. Se o valor dos segundos for 45, o valor de SEC0 será 5 e o valor de SEC1 será 4. */
    ldr r1, =0x04     
    str r1, [r0, #0x48]

    /* habilita RTC */
    ldr r0, =RTC_BASE
    ldr r1, =(1<<0)
    str r1, [r0, #0x40]  

    /*rtc irq setup */
.wait_rtc_update:
    ldr r1, [r0, #0x44] //Carrega o valor do registrador de status do RTC em r1
    and r1, r1, #1
    cmp r1, #0
    bne .wait_rtc_update
    /*adiciona prioridade 0, na interrupção 75*/
    ldr r0, =INTC_ILR
    mov r1, #0    
    strb r1, [r0, #75] 
    /* configura MIR CLEAR2 */
    ldr r0, =INTC_BASE
    mov r1,#(1<<11)  
    str r1, [r0, #0xc8] //(75 and 72--> Bit 11 do 3º registrador (MIR CLEAR2))
    
    ldmfd sp!,{r0-r1,pc}


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


.uart_getc:
    stmfd sp!,{r1-r3,lr}
    ldr     r1, =UART0_BASE

.wait_rx_fifo:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<0)
    cmp r2, #0
    beq .wait_rx_fifo

    ldrb    r0, [r1]
    ldmfd sp!,{r1-r3,pc}
/********************************************************/
/********************************************************
  WDT disable sequence (see TRM 20.4.3.8):
    1- Write XXXX AAAAh in WDT_WSPR.
    2- Poll for posted write to complete using WDT_WWPS.W_PEND_WSPR. (bit 4)
    3- Write XXXX 5555h in WDT_WSPR.
    4- Poll for posted write to complete using WDT_WWPS.W_PEND_WSPR. (bit 4)
    
  Registers (see TRM 20.4.4.1):
    WDT_BASE -> 0x44E35000
    WDT_WSPR -> 0x44E35048
    WDT_WWPS -> 0x44E35034
********************************************************/
.disable_wdt:
    /* TRM 20.4.3.8 */
    stmfd sp!,{r0-r1,lr}
    ldr r0, =WDT_BASE
    
    ldr r1, =0xAAAA
    str r1, [r0, #0x48]
    bl .poll_wdt_write

    ldr r1, =0x5555
    str r1, [r0, #0x48]
    bl .poll_wdt_write

    ldmfd sp!,{r0-r1,pc}

.poll_wdt_write:
    ldr r1, [r0, #0x34]
    and r1, r1, #(1<<4)
    cmp r1, #0
    bne .poll_wdt_write
    bx lr
/********************************************************/

.dec_digit_to_ascii:
	add r0,r0,#0x30
	bx lr

.hex_digit_to_ascii:
       stmfd sp!,{r0-r2,lr} 
       ldr r1, =ascii
       ldrb r0, [r1, r0]
       
       ldmfd sp!, {r0-r2, pc}

.rtc_to_ascii:
    stmfd sp!,{r0-r2,lr} 
    mov r2, r0
    
    and r0, r2, #0x70 //mask
    
    mov r0, r0, LSR #4
    bl .dec_digit_to_ascii //chama a função converter decimal em asc2
    bl .uart_putc

    and r0, r2, #0x0f //mask
    add r0,r0,#0x30 //conversão
    bl .uart_putc // envia para uart

    ldmfd sp!, {r0-r2, pc}

/*************"hh:mm:ss\r****************** /*/
.print_time:
    stmfd sp!,{r0-r2,lr}
    ldr r1,=RTC_BASE
    ldr r0, [r1, #8] //hora
    bl .rtc_to_ascii //converte

    ldr r0,=':' //Carrega o caractere ':' em r0.
    bl .uart_putc // manda pra uart

    ldr r0, [r1, #4] //minuto
    bl .rtc_to_ascii

    ldr r0,=':'
    bl .uart_putc

    ldr r0, [r1, #0] //segndo
    bl .rtc_to_ascii

    ldr r0,='\r' //retorno
    bl .uart_putc
    ldmfd sp!, {r0-r2, pc}


/**********************interrupção isr**************/
.rtc_isr:
    stmfd sp!, {r0-r2, lr}

    /*print time*/
    bl .print_time

    ldmfd sp!, {r0-r2, pc}

.mod_div:
stmfd sp!,{r1-r2,lr}
    l1:
    cmp r0,#10
    blt .finish
    sub r0,#10
    b l1
    .finish:
ldmfd sp!,{r1-r2,pc}

.uart0_isr_for_rtc_update:
stmfd sp!, {r0-r2,lr}

    bl .uart_getc   //receber caractere
    sub r0,#48      //transforma em inetiro
    bl .mod_div     // divide por 10 para intervalo de 0-9
    ldr r1,=_buffer // carrega no buffer
    add r1,r5       // indexa vetor usando r5 como indice
    strb r0,[r1],#1 //armazena e incrementa indice
    add r5,#1 
    cmp r5,#6       // verifica se foi recebido 6 digitos consecutivos

    bne .not_six

    /*atualizar relogio */
    ldr r0,=RTC_BASE
    ldr r1,=_buffer     //carrega do buffer
    ldrb R2,[r1],#1  
    mov r2,r2,lsl #4    //desloca
    ldrb r4,[r1],#1
    add r2,r2,r4        // obtem a hora atual
    str R2, [r0, #0x8]  // atualiza hora

    ldrb r2,[r1],#1
    mov R2,r2,lsl #4
    ldrb r4,[r1],#1
    add r2,r2,r4
    str r2, [r0, #0x4]  // atualiza minutos

    ldrb r2,[r1],#1
    mov r2,r2,lsl #4
    ldrb r4,[r1],#1
    add r2,r2,r4
    str r2, [r0, #0x0]  // atualiza segundos
    mov r5,#0

    .not_six:

ldmfd sp!, {r0-r2,pc}


/******************IRQ Handler******************/
.irq_handler:
    stmfd sp!, {r0-r2, r11, lr}  
    mrs r11, spsr               // Lê o valor do registrador SPSR (Program Status Register do modo anterior) e o armazena em r11.

    ldr r0, =INTC_BASE           // Carrega o endereço base do controlador de interrupções em r0.
    ldr r1, [r0, #0x40]          // Lê o registro de status de interrupção no deslocamento 0x40 a partir do endereço base em r1.

    and r1, r1, #0x7f            // Aplica uma máscara para manter apenas os 7 bits menos significativos do status de interrupção.

    
    /* Interrupção UART */
    cmp r1, #72                   // Compara o valor do status de interrupção com o número 72.
    bleq .uart0_isr// Se for menor ou igual (BLEQ), salte para a rotina de interrupção UART para atualização RTC.

    cmp r1, #98
    bleq .gpio_isr

    ldr r0, =INTC_BASE           // Carrega novamente o endereço base do controlador de interrupções em r0.
    ldr r1, =0x1                 // Carrega o valor 1 em r1.
    str r1, [r0, #0x48]          // Escreve o valor de r1 no registrador de controle de interrupção no deslocamento 0x48.

    dsb                          // Sincroniza a memória e o barramento de memória antes de restaurar o SPSR.
    msr spsr, r11                // Restaura o valor do SPSR a partir de r11, voltando ao modo anterior.

    ldmfd sp!, {r0-r2, r11, pc}^ 

.swi_handler:
    stmfd sp!,{r0-r12,lr}     
    ldr r0,[lr,#4]         // Carrega a instrução SWI que gerou a interrupção em r0.
    bic r1,r0,#0xff000000   // Extrai o número SWI dos bits mais significativos de r0.

    ldr r0, =swi_msg                   // Carrega o endereço da mensagem para exibição.
    bl .print_string                   // Chama a função para imprimir a mensagem.

    mov r0, r1                         // Move o número SWI para r0.
    bl .hex_to_ascii                  // Converte o número SWI em formato hexadecimal para ASCII.
    ldr r0,=CRLF                       // Carrega a sequência de retorno de carro e nova linha.
    bl .print_string                   // Imprime a sequência.

    ldmfd           sp!, {r0-r12,pc}^   
  
   
   

.section .rodata
hello:                   .asciz "Real-Time Clock \n\r"
swi_msg:                 .asciz "interrupção swi!\n\r"
ascii:                   .asciz "0123456789ABCDEF"
CRLF:                    .asciz "\n\r"

_buffer: .fill 0x10

























