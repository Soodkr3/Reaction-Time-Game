# PASTE LINK TO TEAM VIDEO BELOW
#
# https://media.heanet.ie/page/0196c30b7fcd4788bc93399e4472d001

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 250
  .equ    BLINK_PERIOD2, 1000

  .section .text

Main:
  PUSH  {R4-R5,LR}

  LDR R4, =button_on 
  MOV R5, #0
  STR R5, [R4]

  LDR R4, =button_off
  MOV R5, #0
  STR R5, [R4]

  LDR R4, =buttonElapsed
  MOV R5, #0
  STR R5, [R4]

  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Configure LD3 for output
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1
  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @
  @ Initialize counter for blinking LEDs in sequence

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  @ Wait for BLINK_PERIOD ms
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  BL led1
  BL led2
  BL led3
  BL led4
  BL led5
  BL led6
  BL led7
  BL led8

  LDR     R0, =BLINK_PERIOD2
  BL      delay_ms

  BL led4
  BL led5

  LDR   R4, =button_on          @ count = count + 1                 @
  MOV R5, #1                  @
  STR   R5, [R4]
  @ Wait for BLINK_PERIOD ms


  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R7, R9-R12, LR}

  @ LDR R4, =button_on
  @ LDR R5, [R4]
  @ CMP R5, #1
  @ BEQ .LreactionDelay

  @ B .Lend
  
  .LreactionDelay:
    LDR R7, =buttonElapsed
    LDR R8, [R7]
    ADD R8, R8, #1
    STR R8, [R7]

    LDR R9, =button_off
    LDR R12, [R9]

    CMP R12, #1
    BEQ .Lcheckreaction

    B .Lend

  .Lcheckreaction:
    LDR R11, =0x100
    CMP R8, R11
    BLT .Lwin

    B .Llose
  
  .Lwin:
    BL led2
    BL led3
    BL led6
    BL led7
    BL led4
    BL led5

    LDR     R0, =BLINK_PERIOD2
    BL      delay_ms

    B End_Main
  
  .Llose:
    BL led1
    BL led2
    BL led3
    BL led6
    BL led7
    BL led8

    LDR     R0, =BLINK_PERIOD2
    BL      delay_ms

    B End_Main


.Lend:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R7, R9-R12, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}

  BL led2
  BL led3
  BL led6
  BL led7

  BL led4
  BL led5

  LDR     R0, =BLINK_PERIOD2
  BL      delay_ms

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @

  LDR   R4, =button_off          @ count = count + 1                 @
  MOV R5, #1                  @
  STR   R5, [R4]                  

  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4,R5,PC}

led1:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD3_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD3_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD3_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led2:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD4_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD4_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD4_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led3:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD5_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD5_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD5_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led4:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD6_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD6_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD6_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led5:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD7_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD7_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD7_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led6:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD8_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD8_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD8_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led7:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD9_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD9_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD9_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

led8:
  PUSH {R6-R7, LR}

  LDR     R6, =GPIOE_MODER
  LDR     R7, [R6]                  @ Read ...
  BIC     R7, #(0b11<<(LD10_PIN*2))  @ Modify ...
  ORR     R7, #(0b01<<(LD10_PIN*2))  @ write 01 to bits 
  STR     R7, [R6]     

  LDR     R6, =GPIOE_ODR
  LDR     R7, [R6]                  @ Read ...
  EOR     R7, #(0b1<<(LD10_PIN))     @ Modify ...
  STR     R7, [R6]  

  POP {R6-R7, PC}

delay_ms:
  PUSH  {R4-R5,LR}
  LDR   R4, =SYSTICK_CSR
  MOV   R5, #0
  STR   R5, [R4]                     @ Stop SysTick timer

  LDR   R4, =SYSTICK_LOAD
  LDR   R5, =7999                    @ Load value for 1ms delay assuming 8MHz clock
  STR   R5, [R4]

  MOV   R5, #0
  LDR   R4, =SYSTICK_VAL
  STR   R5, [R4]                     @ Clear current value

  MOV   R5, #5
  LDR   R4, =SYSTICK_CSR
  STR   R5, [R4]                     @ Start timer with system clock

.LwhDelay:
  CMP   R0, #0  
  BEQ   .LendwhDelay

.Lwait:
  LDR   R5, [R4]                     
  TST   R5, #0x10000                  
  BEQ   .Lwait

  SUBS  R0, R0, #1
  B     .LwhDelay

.LendwhDelay:
  MOV   R5, #0
  STR   R5, [R4]                     @ Stop SysTick timer

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]

  POP   {R4-R5,PC}

  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4

button_on:
  .space 4

button_off:
  .space 4

buttonElapsed:
  .space 4

  .end