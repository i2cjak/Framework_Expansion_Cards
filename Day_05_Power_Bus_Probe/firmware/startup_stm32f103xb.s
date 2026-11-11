  .syntax unified
  .cpu cortex-m3
  .thumb

  .global g_pfnVectors
  .global Reset_Handler

  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word Default_Handler /* NMI */
  .word Default_Handler /* HardFault */
  .word Default_Handler /* MemManage */
  .word Default_Handler /* BusFault */
  .word Default_Handler /* UsageFault */
  .word 0
  .word 0
  .word 0
  .word 0
  .word Default_Handler /* SVCall */
  .word Default_Handler /* DebugMon */
  .word 0
  .word Default_Handler /* PendSV */
  .word SysTick_Handler /* SysTick */
  /* External Interrupts 0..19 */
  .rept 20
  .word Default_Handler
  .endr
  .word USB_LP_CAN1_RX0_IRQHandler /* IRQ20: USB Low-Priority */

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0

  /* Copy .data from Flash to SRAM */
  ldr   r0, =_sdata
  ldr   r1, =_edata
  ldr   r2, =_sidata
1:
  cmp   r0, r1
  bhs   2f
  ldr   r3, [r2], #4
  str   r3, [r0], #4
  b     1b
2:
  /* Zero .bss */
  ldr   r0, =_sbss
  ldr   r1, =_ebss
  movs  r2, #0
3:
  cmp   r0, r1
  bhs   4f
  str   r2, [r0], #4
  b     3b
4:
  bl    main
5:
  b     5b

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
  b     Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler

  .weak USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler, Default_Handler
