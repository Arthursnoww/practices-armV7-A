/* Global Symbols */
.global .disable_wdt
.type .disable_wdt, %function

.global ._reset
.type ._reset, %function


/* Registradores */
.equ WDT_BASE, 0x44E35000
.equ WDT_WDSC, 0x10
.equ WDT_WDST, 0x14
.equ WDT_WTGR, 0x30
.equ WDT_WSPR, 0x48
.equ WDT_WCRR, 0x28
.equ TIMER_OVERFLOW, 0xFFFFFFFF
.equ TIMER_1MS_COUNT,0x5DC0



/* Text Section */
.section .text,"ax"
         .code 32
         .align 4
         
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

._reset:
stmfd sp!,{r0-r1,lr}
  ldr r1,=WDT_BASE
  ldr r0,=WDT_WCRR
  add r1,r0
  ldr r0,[r1]
  mov r0,#0xffffffff
  str r0,[r1]
ldmfd sp!,{r0-r1,pc}
















