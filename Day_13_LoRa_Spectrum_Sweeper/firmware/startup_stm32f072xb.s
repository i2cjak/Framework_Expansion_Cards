.syntax unified
.cpu cortex-m0
.thumb

.global Reset_Handler
.global Default_Handler

.section .isr_vector, "a", %progbits
.word _estack
.word Reset_Handler
.word Default_Handler      /* NMI */
.word Default_Handler      /* HardFault */
.rept 12
.word Default_Handler      /* System exceptions 4..15 (incl. SVC, PendSV, SysTick) */
.endr
.rept 5
.word Default_Handler      /* IRQ 0..4: WWDG..RCC_CRS */
.endr
.word EXTI0_1_IRQHandler   /* IRQ 5: EXTI Line 0..1 (PA0 INA226 140us E-Fuse Alert) */
.rept 24
.word Default_Handler      /* IRQ 6..29 */
.endr
.word CEC_CAN_IRQHandler   /* IRQ 30: CEC and CAN */
.word USB_IRQHandler       /* IRQ 31: USB Full-Speed */

.section .text.Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_etext
1:
    cmp r0, r1
    bhs 2f
    ldr r3, [r2]
    str r3, [r0]
    adds r0, #4
    adds r2, #4
    b 1b
2:
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
3:
    cmp r0, r1
    bhs 4f
    str r2, [r0]
    adds r0, #4
    b 3b
4:
    bl main
5:
    b 5b

.section .text.Default_Handler
.type Default_Handler, %function
Default_Handler:
    b Default_Handler

.weak EXTI0_1_IRQHandler
.thumb_set EXTI0_1_IRQHandler, Default_Handler

.weak CEC_CAN_IRQHandler
.thumb_set CEC_CAN_IRQHandler, Default_Handler

.weak USB_IRQHandler
.thumb_set USB_IRQHandler, Default_Handler
