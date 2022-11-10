
;=========================================================================
;PROGRAM - SCC366 Coursework 1 Lights and Switches
;
;DESCR - Switch through modes for LEDs on PORTC with input on PORTB
;
;AUTHOR - Andrew Lucas
;
;VERSION - 1.00 15/10/21
;
;HISTORY - 1.00
;=========================================================================

TITLE "Coursework 1"
SUBTITLE "Lights and Switches"

PROCESSOR 16F1507

#include <xc.inc>
#include <pic16lf1507.inc>
    
; CONFIG1
CONFIG  FOSC = INTOSC         ; ECH	|   ECM	    |   ECL	|   INTOSC
CONFIG  WDTE = OFF            ; ON	|   NSLEEP  |	SWDTEN	|   OFF
CONFIG  PWRTE = OFF           ; OFF	|   ON
CONFIG  MCLRE = ON            ; ON	|   OFF
CONFIG  CP = OFF              ; OFF	|   ON
CONFIG  BOREN = OFF           ; ON	|   NSLEEP  |	SBODEN	|   OFF
CONFIG  CLKOUTEN = OFF        ; OFF	|   ON

; CONFIG2
CONFIG  WRT = OFF             ; OFF	|   BOOT    |	HALF	|   ALL
CONFIG  STVREN = ON           ; ON	|   OFF
CONFIG  BORV = LO             ; LO	|   HI
CONFIG  LPBOR = OFF           ; OFF	|   ON
CONFIG  LVP = ON              ; ON	|   OFF

PSECT res_vect, class=CODE
    GOTO MAIN
PSECT int_vect, class=CODE
    GOTO ISR


PSECT	udada_bank0
GLOBAL	OUTER_LOOP_COUNT
GLOBAL	LOOP_COUNTER
OUTER_LOOP_COUNT:   DS	1
LOOP_COUNTER:   DS	1

GLOBAL	BUTTON_TO_TEST
BUTTON_TO_TEST:	DS  1


PSECT code

;=========================================================================
;BLOCK	MAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	ANSELC, TRISC, ANSELB, TRISB
;
;DESC	Main routine. Sets ports to correct modes, loops through modes.
;
;CONFIG	NONE
;CALLS	DEFAULT
;=========================================================================

MAIN:
    BANKSEL ANSELC  ;	Same as manually selecting bank3 with MOVLB 3
    CLRF    ANSELC  ;	Clear as all port C is to be digital output
    BANKSEL TRISC   ;	In bank 1
    CLRF    TRISC   ;	Set all port C to output (0)
    BANKSEL ANSELB  ;	Bank 3
    CLRF    ANSELB  ;	Set all port B for digital
    BANKSEL TRISB   ;	Bank 1
    MOVLW   0x70    ;	Portb 4-6 set as input. 7 is output.
    MOVWF   TRISB   ;
    MOVLB   0

MAINLOOP:
    CALL    DEFAULT
    CALL    BUTTON4RELEASE	;This makes sure only 1 press detected
    CALL    ID
    CALL    BUTTON4RELEASE	;This makes sure only 1 press detected
    CALL    BITMATH
    CALL    BUTTON4RELEASE
    CAll    COUNTER
    CALL    BUTTON4RELEASE
    GOTO    MAINLOOP


;=========================================================================
;SUBR	SINGLEPRESS
;PARAM	NONE
;RETURN	NONE
;WRITES	0x020h
;
;DESC	Calls 100ms delay for button debouncing
;
;CONFIG	NONE
;CALLS	DELAY
;=========================================================================

SINGLEPRESS:
    MOVLB   0	    ;Select Bank 0
    MOVLW   0xc    ;12 loops for delay of approx 50 ms
    MOVWF   0x20h   ;Store in general purpose register specified in DELAY
    CALL    DELAY    
    RETURN

;These are used to wait for button release
;Avoid multiple presses when holding
BUTTON4RELEASE:	    ;Loop until b4 released
    BTFSC   PORTB,4
    GOTO    LEAVE
    GOTO    BUTTON4RELEASE
BUTTON5RELEASE:	    ;Loop until b5 released
    BTFSC   PORTB,5
    GOTO    LEAVE
    GOTO    BUTTON5RELEASE
BUTTON6RELEASE:	    ;Loop until b6 released
    BTFSC   PORTB,6
    GOTO    LEAVE
    GOTO    BUTTON6RELEASE
LEAVE:		    ;Delay on release for debounce
    CALL    SINGLEPRESS
    RETURN

;=========================================================================
;BLOCK	DEFAULT
;PARAM	NONE
;RETURN	NONE
;WRITES	PORTC, PORTB
;
;DESC	Dfault state for program with all LEDs on.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================


DEFAULT:
    BANKSEL PORTC   ;	Bank 0
    MOVLW   0xff    ;	255 to set all bits to 1
    MOVWF   PORTC   ;	Move 255 to PORTC. All outputs 1, turns LEDS on.
DEFAULTLOOP:
    BANKSEL PORTB   ;	Bank 0
    BTFSS   PORTB,4 ;	Skip if bit 4 is 1 (Unpressed). Active low
    RETURN
    GOTO    DEFAULTLOOP


;=========================================================================
;BLOCK	ID
;PARAM	NONE
;RETURN	NONE
;WRITES	0x022h-0x02ah, FSR0L, FSR0H, PORTC
;
;DESC	Stores student id in general purpose registers
;	then loops displaying on LED
;
;CONFIG	NONE
;CALLS	SINGLEPRESS
;=========================================================================

ID:

    ;Store remaining loops
    MOVLW   8
    MOVWF   0x022h

    ;Store student id in adresses 0x023h-0x02ah
    MOVLW   0x13
    MOVWF   0x023h
    MOVLW   0x27
    MOVWF   0x024h
    MOVLW   0x37
    MOVWF   0x025h
    MOVLW   0x44
    MOVWF   0x026h
    MOVLW   0x59
    MOVWF   0x027h
    MOVLW   0x60
    MOVWF   0x028h
    MOVLW   0x77
    MOVWF   0x029h
    MOVLW   0x82
    MOVWF   0x02ah

    ;Load address of 1st addr into FSR
    MOVLW   0x023h
    MOVWF   FSR0L
    MOVLW   0x00
    MOVWF   FSR0H   ;FSR high set to 0. Not using 16 bit address

    GOTO    IDNEXT  ;Display first number

IDLOOP:
    BTFSS   PORTB,4
    RETURN
    BTFSC   PORTB,5
    GOTO    IDLOOP

IDNEXT:
    MOVF    INDF0,0 ;Load value of address in fsr
    MOVWF   PORTC
    INCF    FSR0L   ;Move to next byte
    CALL    BUTTON5RELEASE  ;Stops multiple clicks on holding button
    DECFSZ  0x022h  ;Loop until count 0
    GOTO    IDLOOP

IDLAST:	;Leave on b4, reset if b5
    BTFSS   PORTB,4
    RETURN
    BTFSC   PORTB,5
    GOTO    IDLAST
    GOTO    ID



;=========================================================================
;BLOCK	BITMATH
;PARAM	NONE
;RETURN	NONE
;WRITES	PORTC, 0x022h,0x023h,0x024h
;
;DESC	Logic on button 5/6 and displays result in PORTC
;	Results stored in byte as below
;	NOR,NOT6,NOT5,XOR,AND,OR,5,6
;	7   6	5   4	3   2	1   0
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
    
BITMATH:
    MOVLB   0	;Bank0
    
BITMATHLOOP:
    BTFSS   PORTB,4 ;Leave on b4 press
    RETURN

    MOVLW   0
    MOVWF   0x022h	;Address used for fianl bits to be moved to PORTC
    MOVWF   0x023h	;To store status of button5 (1 = pressed, 0 = not)
    MOVWF   0x024h	;To store status of button6
    MOVLW   1

    ;These could be stored in 1 byte to save space
    ;but requires more instructions later
    BTFSS   PORTB,5	;Set button5 to 1 if pressed
    MOVWF   0x023h
    BTFSS   PORTB,6	;Set button6 to 1 if pressed
    MOVWF   0x024h


    ;NOR
    MOVF    0x023h,0	;Load b5 status
    IORWF   0x024h,0	;Inclusive or on b5/b6, value in w
    XORLW   1		;XOR with previous OR result for NOR
    ;Next part could be done by adding literals of 2^n where n is pos from right
    ;conditionally on previous result
    ADDWF   0x022h,1	;Add to final byte
    LSLF    0x022h,1	;Shift left so not overwritten
    

    ;NOT b6
    MOVLW   1
    XORWF   0x024h,0	;Will result in 1 if b6 is 0
    ADDWF   0x022h
    LSLF    0x022h,1

    ;NOT b5
    MOVLW   1
    XORWF   0x023h,0	;Result 1 if b5 is 0
    ADDWF   0x022h
    LSLF    0x022h,1

    ;XOR
    MOVWF   0x024h,0
    XORWF   0x023h,0	
    ADDWF   0x022h
    LSLF    0x022h,1

    ;AND
    MOVF    0x024h,0
    ANDWF   0x023h,0
    ADDWF   0x022h
    LSLF    0x022h,1

    ;OR
    MOVF    0x024h,0
    IORWF   0x023h,0
    ADDWF   0x022h
    LSLF    0x022h,1

    ;Button5 pressed
    MOVF    0x024h,0
    ADDWF   0x022h
    LSLF    0x022h,1

    ;Button4 pressed
    MOVF    0x023h,0	;Get status of button5
    ADDWF   0x022h	;Add to end final byte

    
    MOVF    0x022h,0	;Load final byte
    MOVWF   PORTC
    GOTO    BITMATHLOOP


;=========================================================================
;BLOCK	COUNTER
;PARAM	NONE
;RETURN	NONE
;WRITES	PORTC,0x020h
;
;DESC	Counts up to 255 before starting again. Stores current count in PORTC.
;
;CONFIG	NONE
;CALLS	DELAY
;=========================================================================


COUNTER:
    BANKSEL PORTC
    MOVLW   0
    MOVWF   PORTC	;Make sure count starts at 0
COUNTERLOOP:
    BTFSS   PORTB,4	;Return if b4 pressed
    RETURN
    INCF    PORTC,1
    MOVLW   0x02	;Delay loops
    MOVWF   0x020h	;Address specified in DELAY
    CALL    DELAY	;Delay so doesn't count too fast
    GOTO    COUNTERLOOP

;=========================================================================
;SUBR	DELAY
;PARAM	Outer loop count(in 0x020h)
;RETURN	NONE
;WRITES	OUTER_LOOP_COUNT,INNER_LOOP,COUNT
;
;DESC	Delay loop
;
;CONFIG	Set loop count in 0x020h
;CALLS	NONE
;=========================================================================

DELAY:
    MOVLB   0			;Bank 0
    MOVF    0X020h,0		;Get outer loop count from general purpose register
    MOVWF   OUTER_LOOP_COUNT
OUTERLOOP:
    MOVLW   0xff		;Set inner loop back to 255
    MOVWF   LOOP_COUNTER
INNERLOOP:
    DECFSZ  LOOP_COUNTER, F
    GOTO    INNERLOOP
    DECFSZ  OUTER_LOOP_COUNT, F	;When 0, end
    GOTO    OUTERLOOP
    RETURN

;=========================================================================
;ISR	ISR
;PARAM	NONE
;RETURN	NONE
;WRITES	NONE
;
;DESC	Interupt Service Routine    Doesn't do anything
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================

ISR:
    RETFIE

    END