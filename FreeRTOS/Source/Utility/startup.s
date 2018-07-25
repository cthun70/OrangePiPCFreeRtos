;==================================================================
; Copyright ARM Ltd 2005-2016. All rights reserved.
;
; Cortex-A7 Embedded example - Startup Code
;==================================================================


; Standard definitions of mode bits and interrupt (I & F) flags in PSRs

Mode_USR        EQU     0x10
Mode_FIQ        EQU     0x11
Mode_IRQ        EQU     0x12
Mode_SVC        EQU     0x13
Mode_ABT        EQU     0x17
Mode_UND        EQU     0x1B
Mode_SYS        EQU     0x1F

I_Bit           EQU     0x80               ; When I bit is set, IRQ is disabled
F_Bit           EQU     0x40               ; When F bit is set, FIQ is disabled


    PRESERVE8
    AREA   VECTORS, CODE, READONLY         ; Name this block of code

    ENTRY

;==================================================================
; Entry point for the Reset handler
;==================================================================

    EXPORT Start

Start

;==================================================================
; Exception Vector Table
;==================================================================
; Note: LDR PC instructions are used here, though branch (B) instructions
; could also be used, unless the exception handlers are >32MB away.


Vectors
    LDR PC, Reset_Addr
    LDR PC, Undefined_Addr
    LDR PC, SVC_Addr
    LDR PC, Prefetch_Addr
    LDR PC, Abort_Addr
    LDR PC, Hypervisor_Addr
    LDR PC, IRQ_Addr
    LDR PC, FIQ_Addr

Reset_Addr      DCD     Reset_Handler
Undefined_Addr  DCD     Undefined_Handler
SVC_Addr        DCD     SVC_Handler
Prefetch_Addr   DCD     Prefetch_Handler
Abort_Addr      DCD     Abort_Handler
Hypervisor_Addr DCD     Hypervisor_Handler
IRQ_Addr        DCD     IRQ_Handler
FIQ_Addr        DCD     FIQ_Handler


;==================================================================
; Exception Handlers
;==================================================================

Undefined_Handler
    IMPORT  ||Image$$ARM_LIB_STACK$$ZI$$Limit||   ; Linker symbol from scatter file
    LDR     SP, =||Image$$ARM_LIB_STACK$$ZI$$Limit||
    B   Reset_Handler

SVC_Handler
    IMPORT  FreeRTOS_SWI_Handler
    b FreeRTOS_SWI_Handler
;	SUBS	pc, lr, #4
;	STMFD 	sp!,{r0-r12,lr}
;    IMPORT  ServiceInterruptHandler
;    bl   ServiceInterruptHandler
;	LDMFD 	sp!, {r0-r12,pc}^

Prefetch_Handler
	SUBS	pc, lr, #4
	STMFD 	sp!,{r0-r12,lr}
    IMPORT  PreFetchInterruptHandler
	bl   	PreFetchInterruptHandler
	LDMFD 	sp!, {r0-r12,pc}^

Abort_Handler
	SUBS	pc, lr, #8
	STMFD 	sp!,{r0-r12,lr}
    IMPORT  AbortInterruptHandler
	bl   	AbortInterruptHandler
	LDMFD 	sp!, {r0-r12,pc}^
	
Hypervisor_Handler
	SUB lr, lr, #4
	STMFD 	sp!,{r0-r12,lr}
    IMPORT  HypervisorInterruptHandler
    bl   HypervisorInterruptHandler
	LDMFD 	sp!, {r0-r12,pc}^

IRQ_Handler
    IMPORT  FreeRTOS_IRQ_Handler
    b FreeRTOS_IRQ_Handler
;	SUB lr, lr, #4
;	STMFD 	sp!,{r0-r12,lr}
;   IMPORT  IRQInterruptHandler
;	bl   	IRQInterruptHandler
;	LDMFD 	sp!, {r0-r12,pc}^

FIQ_Handler
	SUB lr, lr, #4
	STMFD 	sp!,{r0-r7,lr}
    IMPORT  FIQInterruptHandler
	bl   	FIQInterruptHandler
	LDMFD 	sp!, {r0-r7,pc}^


;==================================================================
; Reset Handler
;==================================================================
Len_FIQ_Stack    	EQU     8192
Len_IRQ_Stack    	EQU     8192
stack_base      	DCD      ||Image$$ARM_LIB_STACK$$ZI$$Limit|| 

Reset_Handler   FUNCTION 


		; stack_base could be defined above, or located in a scatter file
		LDR     R0, stack_base ;
		; Enter each mode in turn and set up the stack pointer
		MSR     CPSR_c,#Mode_FIQ:OR:I_Bit:OR:F_Bit    ; Interrupts disabled
		MOV     sp, R0
		SUB     R0, R0,#Len_FIQ_Stack
		MSR     CPSR_c,#Mode_IRQ:OR:I_Bit:OR:F_Bit    ; Interrupts disabled
		MOV     sp, R0
		SUB     R0, R0,#Len_IRQ_Stack
		MSR     CPSR_c,#Mode_SVC:OR:I_Bit:OR:F_Bit    ; Interrupts disabled
		MOV     sp, R0
	
		
    	ldr	r0, =Vectors
    	mcr	p15, 0, r0, c12, c0, 0
    	dsb
    	isb

    	MRC p15,0,r0,c1,c0,2    ;// Read CP Access register
    	ORR r0,r0,#0x00f00000   ;// Enable full access to NEON/VFP (Coprocessors 10 and 11)
    	MCR p15,0,r0,c1,c0,2    ;// Write CP Access register
    	ISB
    	MOV r0,#0x40000000      ;// Switch on the VFP and NEON hardware
    	MSR FPEXC,r0            ;// Set EN bit in FPEXC


    	IMPORT main
    	B main                ;// Enter normal C run-time environment & library start-up


        ENDFUNC
		

        END
