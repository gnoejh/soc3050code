
/*
 * main_blink_asm_source.s
 *
 * Created: 2024-09-22 오후 8:16:54
 *  Author: hjeong
 */ 

.INCLUDE "m128def.inc"  ; Include device-specific definitions

.ORG 0
; STACK POINTER SETUP
	LDI R16, HIGH(RAMEND)
	OUT SPH, R16
	LDI R16, LOW(RAMEND)
	OUT SPL, R16

; MAIN LABEL
MAIN:
	LDI R16, 0xFF 
	OUT DDRB, R16 ; Set PORTB as output
	LDI R16, 0xAA

BACK:
	OUT PORTB, R16 ; Write value in R16 to PORTB
	COM R16        ; Complement the value in R16
	CALL DELAY     ; Call delay function
	RJMP BACK      ; Repeat forever

DELAY:
	LDI R17, 62    ; Approx. 1 second delay
LOOP3: LDI R18, 255
LOOP2: LDI R19, 255
LOOP1: DEC R19 
	BRNE LOOP1     ; Keep decreasing R19
	DEC R18 
	BRNE LOOP2     ; For every decrease of R18, redo previous loop
	DEC R17 
	BRNE LOOP3     ; For every decrease of R17, repeat previous loop
	RET            ; Return to previous PC address
