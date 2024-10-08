.equ INTC_BASE, 0x48200000
.equ INTC_ILR,  0x48200100

.equ CPSR_I,   0x80
.equ CPSR_F,   0x40
.equ CPSR_IRQ, 0x12
.equ CPSR_USR, 0x10
.equ CPSR_FIQ, 0x11
.equ CPSR_SVC, 0x13
.equ CPSR_ABT, 0x17
.equ CPSR_UND, 0x1B
.equ CPSR_SYS, 0x1F


//.equ VECTOR_BASE, 0x4030CE20
.equ VECTOR_BASE, 0x4030CE00 

/* Registradores */
.equ WDT_BASE, 0x44E35000
.equ WDT_WDSC, 0x10
.equ WDT_WDST, 0x14
.equ WDT_WTGR, 0x30
.equ WDT_WSPR, 0x48
.equ WDT_WCRR, 0x28
.equ TIMER_OVERFLOW, 0xFFFFFFFF
.equ TIMER_1MS_COUNT,0x5DC0

.equ UART0_BASE, 0x44E09000
.equ INTC_BASE, 0x48200000
.equ INTC_ILR,  0x48200100

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


/* GPIO Clock Setup */
.equ CM_PER_GPIO1_CLKCTRL, 0x44e000AC
.equ CM_RTC_RTC_CLKCTRL, 0x44E00800
.equ CM_RTC_CLKSTCTRL,  0x44E00804
.equ RTC_BASE, 0x44E3E000

flag_gpio: .word 1 


/*******************************
Stack
/*******************************/
.set StackModeSize,  0x100

.equ StackUSR, (_stack_end - 0*StackModeSize)
.equ StackFIQ, (_stack_end - 1*StackModeSize)
.equ StackIRQ, (_stack_end - 2*StackModeSize)
.equ StackSVC, (_stack_end - 3*StackModeSize)
.equ StackABT, (_stack_end - 4*StackModeSize)
.equ StackUND, (_stack_end - 5*StackModeSize)
.equ StackSYS, (_stack_end - 6*StackModeSize)

.section .text,"ax"
         .code 32
         .align 4

/********************************************************/
/* Vector table */
/********************************************************/
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

/********************************************************/
/* Startup Code */
/********************************************************/
_start:

    /* Configure CP15 */
    bl .cp15_configure
         
    /* init */
    mrs r0, cpsr
    bic r0, r0, #0x1F            // clear mode bits
    orr r0, r0, #CPSR_SVC        // set SVC mode
    orr r0, r0, #(CPSR_F | CPSR_I)        // disable FIQ and IRQ
    msr cpsr, r0

   mov r0, #(CPSR_I | CPSR_F) | CPSR_IRQ
   msr cpsr_c, r0
   ldr sp,=StackIRQ


         
    /* Exceptions Setup */
    ldr r0, =_irq
    ldr r1, =.irq_handler
    str r1, [r0]
    
    ldr r0, =_swi
    ldr r1, =.swi_handler
    str r1, [r0]
      
    /* Hardware setup */
    bl .gpio_setup
    bl .disable_wdt
    bl .rtc_setup
    bl .uart0_int_setup

    /*Interrupt setup */
    //bl .uart0_int_setup
    
    /* Enable global irq */
    mrs r0, cpsr
    and r0, r0, #~(CPSR_I)
    msr cpsr, r0

    //bl .prompt
    
    
/********************************************************/
/* Main Code */
/********************************************************/
.main:
    bl .cmd_msn
    /* SWI Test*/    
    //swi #0x250

.main_loop:
    /* logical 1 turns on the led, TRM 25.3.4.2.2.2 */
    ldr r1,=c_goto
    ldrb r1,[r1],#1
    cmp r1,#1
    beq goto_salt
    bl .delay_1s
    b .main_loop    

goto_salt:
    ldr r1,=c_goto
    mov r0,#0
    strb r0,[r1],#1
    ldr r1,=goto_addr
    ldr r0,[r1]
    ldr r1,=0x80000044
    cmp r0,r1
    ldrlt pc,=.main

    bl .hex_to_ascii
    mov pc,r0
    //b .main_loop
/********************************************************/

.cp15_configure:
    stmfd sp!,{r0-r2,lr}

    // 2016: Brian Fraser's Fix for UBoot Messing with Interrupts
    // Disable the MMU, instruction and data caches.
    // Without this, the ISRs seem not to work with the latest UBoot code (2016)
    //SUB r0, r0, r0
    //MCR p15, 0, r0, c1, c0, 0

    /* Set V=0 in CP15 SCTRL register - for VBAR to point to vector */
    mrc    p15, 0, r0, c1, c0, 0    // Read CP15 SCTRL Register
    bic    r0, #(1 << 13)           // V = 0
    mcr    p15, 0, r0, c1, c0, 0    // Write CP15 SCTRL Register

    /* Set vector address in CP15 VBAR register */
    ldr     r0, =_vector_table
    mcr     p15, 0, r0, c12, c0, 0  //Set VBAR */


    ldmfd sp!,{r0-r2,pc}
/********************************************************/

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
    bx lr

    stmfd sp!,{r0-r1,lr}
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
    ldmfd sp!,{r0-r1,pc}
    

/********************************************************/

/********************************************************
Blink LED BBB
********************************************************/

.global .led_ON_1
.type .led_ON_1, %function
.led_ON_1:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_OFF_1
.type .led_OFF_1, %function
.led_OFF_1:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<21)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_ON_2
.type .led_ON_2, %function
.led_ON_2:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_OFF_2
.type .led_OFF_2, %function
.led_OFF_2:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<22)
    str r1, [r0]
    
.global .led_ON_3
.type .led_ON_3, %function
.led_ON_3:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_OFF_3
.type .led_OFF_3, %function
.led_OFF_3:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<23)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_ON_4
.type .led_ON_4, %function
.led_ON_4:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}

.global .led_OFF_4
.type .led_OFF_4, %function
.led_OFF_4:
    stmfd sp!, {r0-r1, lr}
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =(1<<24)
    str r1, [r0]
    ldmfd sp!,{r0-r1,pc}


.global .gpio_isr
.gpio_isr:
    stmfd sp!, {r0-r1, lr}
    
    ldr r0, =GPIO1_IRQSTATUS_0
    mov r1, #0x10000000
    str r1, [r0]

    b .led_ON_4
    ldr r0,=msn
    bl .print_string

    ldmfd sp!,{r0-r1,pc}
    bx lr

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

div:
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
    bx lr  
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
    strb  r2, [r0], #1
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
Converte ascii to hexa r1 endereço, r2 qtd de digitos, retorna a converção em r0
********************************************************/
.global .conv_ascii_hexa
.conv_ascii_hexa:
stmfd sp!,{r1-r4,lr}
    mov r4,#0
    mov r5,#0xf
    .2b:
        mov r0,#0
        ldrb r0,[r1],#1
        bl convert_ascii_hexa
        and r0,r0,r5
        add r4,r4,r0
        sub r2,#1
        cmp r2,#0
        beq fim_2b
        mov r4,r4, LSL #4
        b .2b
    fim_2b:
    mov r0,r4
ldmfd sp!, {r1-r4, pc}

.global convert_ascii_hexa
//recebe o hexa/int 0-9 em asci_hexa em r0 e retorna o hexa em r0
convert_ascii_hexa:
stmfd sp!,{r1-r5,lr}
    and r0, r0, #0xff
    cmp r0,#'9'
    bgt maior
    sub r0,#'0'
    b fim_conv
    maior:
        sub r0,#87
    fim_conv:
   
ldmfd sp!,{r1-r5,pc}
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


.hex_digit_to_ascii:
       stmfd sp!,{r0-r2,lr} 
       ldr r1, =ascii
       ldrb r0, [r1, r0]
       
       ldmfd sp!, {r0-r2, pc}
/********************************************************/
/********************************************************
Converte int (de de 0 a 99) to ascii
********************************************************/
.int_to_ascii:
    stmfd sp!,{r0-r2,lr}
    mov r1, #10
    bl div
    add r0, r0, #0x30
    bl .uart_putc
    add r0, r2, #0x30
    bl .uart_putc
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
.print_string:
    stmfd sp!,{r0-r5,lr}
    mov r1, r0
.print:
    ldrb r0,[r1],#1
    and r0, r0, #0xff
    cmp r0, #0
    beq .end_print
    bl .uart_putc
    b .print
    
.end_print:
    ldmfd sp!,{r0-r5,pc}
/********************************************************/



/********************************************************
Imprime n caracteres de uma string
// R0 -> Endereço da string R1-> Número de caracteres
/********************************************************/
.print_nstring:
    stmfd sp!,{r0-r2,lr}
    mov r2, r0
    cmp r1,#0
    beq .end_print_n
.print_n:
    ldrb r0,[r2],#1
    bl .uart_putc
    subs r1, r1, #1
    beq .end_print_n
    b .print_n
    
.end_print_n:
    ldmfd sp!,{r0-r2,pc}
    
/********************************************************/


/********************************************************/
.mod_div:
stmfd sp!,{r1-r2,lr}
    l1:
    cmp r0,#10
    blt .finish
    sub r0,#10
    b l1
    .finish:
ldmfd sp!,{r1-r2,pc}

/********************************************************
IRQ Handler
********************************************************/
.irq_handler:
        stmfd sp!, {r0-r3, r11, lr}
        mrs r11, spsr
        
 	/* Interrupt Source */
	ldr r0,=INTC_BASE
	ldr r1, [r0, #0x40]
	
	/* if rtc interrupt */
	and r1,r1, #0x7f
    cmp r1, #72  /* TRM 6.3 Table 6-1*/
    bleq .uart_isr



        /* new IRQ */
        ldr r0, =INTC_BASE 
        ldr r1, =0x1
        str r1, [r0, #0x48]

        /*Data Sync Barrier */
	dsb
        msr spsr, r11
        ldmfd sp!, {r0-r3, r11, pc}^



/********************************************************/

/********************************************************/
.fiq_handler:
   b .      
/********************************************************/
.undefined_handler:
   b .     
/********************************************************/
.swi_handler:
   
    stmfd           sp!,{r0-r12,lr} 
    ldr             r0,[lr,#-4]      
                                                             
    bic             r1,r0,#0xff000000   //SWI Number  
    
    ldr r0, =swi_msg
    bl .print_string
    
    mov r0, r1
    bl .hex_to_ascii
    
    ldr r0,=CRLF
    bl .print_string
    
    
    ldmfd           sp!, {r0-r12,pc}^     
   
   
/********************************************************/
.prefetch_abort_handler:
   b .      
/********************************************************/
.data_abort_handler:
   b .      
/********************************************************/

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

._reset:
stmfd sp!,{r0-r1,lr}

  ldr r1,=WDT_BASE
  ldr r0,=WDT_WCRR
  add r1,r0
  ldr r0,[r1]
  mov r0,#0xffffffff
  str r0,[r1]
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
    
    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .gpio_isr
    blne exit
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

    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .gpio_isr
    blne exit
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
    
    mov r1, #1
    lsl r1, r1, #28
    ldr r2, =GPIO1_IRQSTATUS_0
    ldr r2, [r2]
    and r2, r1, r2
    cmp r2, #0
    blne .gpio_isr
    blne exit
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
irq_mode_msg:            .asciz "IRQ Mode!\n\r"
fiq_mode_msg:            .asciz "FIQ Mode!\n\r"
prefetch_abort_msg:      .asciz "Prefetch Abort!\n\r"
data_abort_msg:          .asciz "Data Abort!\n\r"
undefined_exception_msg: .asciz "Undefined Exception!\n\r"
swi_msg:                 .asciz "Software Interrupt Number: \n\r"
ascii:                   .asciz "0123456789ABCDEF"
dash:                    .asciz "-------------------------\n\r"
hex_prefix:              .asciz "0x"
dump_separator:          .asciz "  :  "
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
first:                   .asciz "a primeira e menor\n\r"
second:                  .asciz "a primeira e maior\n\r"
igual:                   .asciz "a duas são iguais\n\r"

/* BSS Section */
.section .bss
.align 4
_str1:   .fill 0x16
_str2:   .fill 0x16
_size_str1: .fill 0x1
_size_str2: .fill 0x1
goto_addr:      .fill 10
c_goto:         .fill 1
.equ BUFFER_SIZE, 20
_buffer:   .fill BUFFER_SIZE
_c_buffer: .fill 0x1
_cursor:   .fill 0x1
