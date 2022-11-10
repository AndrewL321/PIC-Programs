;=========================================================================
;PROGRAM - SCC369 Coursework 2 Clock
;
;DESCR - Clock counter on PORTC. Detects master/slave for 
;	 24 hour binary clock. 
;
;AUTHOR - Andrew Lucas
;
;VERSION - 2.00 - 4/11/21
;
;HISTORY - 1.00
;=========================================================================

TITLE "Coursework 2"
SUBTITLE "Clock"

PROCESSOR 16F1507

#include <xc.inc>
#include <pic16f1507.inc>
    
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

PSECT res_vect, class=CODE, delta=2
    res_vect:
	GOTO MAIN
PSECT int_vect, class=CODE, delta=2
    int_vect:
	GOTO ISR

PSECT code, class=CODE,	delta=2
    GLOBAL	bDELAY_LOOP_COUNT
    bDELAY_LOOP_COUNT:   DS	1

    MOVLB   0
    bINT_BUTTON_FLAG	EQU 0x025h
;   -	-   -	-  b7db(isr) b7db(isr) b7c(isr) b7
;Used to pass info from isr to main
;(isr) = only used by isr. db = debounce.

    bINT_TIMER_FLAG	EQU 0x026h
;   -	-   -	-   -  b6high curSpeed  timer1
;Info on timer and b6

;=========================================================================
;BLOCK	MAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	ANSELC, TRISC, ANSELB, TRISB,OPTION_REG,WPUB,OSCCON,
;	bINT_BUTTON_FLAG(0x025h),bINT_TIMER_FLAG(0x026h)
;	ANSELA, TRISA, INTCON, IOCBN, LATA, LATC
;
;DESC	Configure ports, set oscillator to 31.5khz. Check it's stable.
;
;CONFIG	NONE
;CALLS	CLEARSTATE, CLOCKSTABLE, MODECHECK, ONESECCOUNT, SIXTYSECCOUNT,
;	BUTTONCOUNT
;=========================================================================

MAIN:
    BANKSEL ANSELC	
    CLRF    ANSELC	;All portc digital
    CLRF    ANSELB	;Portb digital
    BCF	    ANSELA,2	;porta 2, 5 already digital
    BCF	    ANSELA,4
    BANKSEL TRISC
    CLRF    TRISC	;Portc all output
    MOVLW   0xD0h	;Portb 4,6,7 input, 5 output
    MOVWF   TRISB
    ;For RGB, not enough pins to use all 3 colours
    BCF	    TRISA,2	;Porta 2,5 as output. Used to control 2 parts of rgb
    BCF	    TRISA,5
    BSF	    TRISA,4

    BANKSEL OPTION_REG	
    BCF	    OPTION_REG,7	;Clear global pull up
    
    BANKSEL WPUB	;Weak pull ups enabled on pins portb 4,6,7
    BSF	    WPUB,7
    BSF	    WPUB,6
    BSF	    WPUB,4
    BSF	    WPUA,4


    BSF	    INTCON,7	;periphiral interrupt
    BSF	    INTCON,3	;interrupt on change

    BANKSEL IOCBN
    BSF	    IOCBN,7	;Detect change to low
    BSF	    IOCAN,4

    ;Timer0 clock source internal
    BANKSEL OPTION_REG
    BCF	    OPTION_REG,5

    ;Prescaler for timer0 1:2
    BCF	    OPTION_REG,3
    BCF	    OPTION_REG,2
    BSF	    OPTION_REG,1
    BCF	    OPTION_REG,0

    CALL    CLEARSTATE

    ;Set clock pre scaler for lower frequency
    BANKSEL OSCCON
    MOVLW   0x12h	;00010010 - 31.5khz + internal osc
    MOVWF   OSCCON

    CALL    CLOCKSTABLE	;Check clock is stable after prescaler change

    MOVLB   0

CALLMODE:		;Loop modes.
   CALL	    MODECHECK
   CALL	    MSLOW	;Make sure clock normal speed when leaving
   ;RGB = OFF, only used in master/slave
   BANKSEL  LATA
   BCF	    LATA,2
   BCF	    LATA,5
   CALL	    CLEARSTATE	;Clear between each call
   CALL	    SIXTYSECCOUNT
   CALL	    CLEARSTATE
   CALL	    BUTTONCOUNT
   CALL	    CLEARSTATE
   GOTO	    CALLMODE

CLOCKSTABLE:		;Loop until HF internal osc is stable
   BTFSS   OSCSTAT,0
   GOTO    CLOCKSTABLE
   RETURN
 ;Clear current state
CLEARSTATE:		;Clear all interupt flags and output (portc)
    BANKSEL LATC
    CLRF    LATC
    MOVLB   0
    CLRF    bINT_BUTTON_FLAG
    CLRF    bINT_TIMER_FLAG
    RETURN

;=========================================================================
;SUBR	MODECHECK
;PARAM	NONE
;RETURN	NONE
;WRITES	LATB, LATA, bINT_BUTTON_FLAG(0)
;
;DESC	Loops until either slave pin is pulled low or b7 pressed.
;	Sets device into correct master/slave mode.
;
;CONFIG	B6 on one device connected to b5 on another. Button pressed flag
;	set by ISR.
;CALLS	SLAVECLOCK, MASTERCLOCK
;=========================================================================
    
MODECHECK:
    MOVLB   0
    BANKSEL LATB
    BCF	    LATB,5		    ;Clear master port, grounds connected slave
    ;RGB = PURPLE
    BSF	    LATA,2
    BSF	    LATA,5
    MOVLB   0
MODELOOP:
    BANKSEL PORTB
    BTFSS   PORTB,6		    ;Check if grounded
    GOTO    SLAVECLOCK
    BTFSS   bINT_BUTTON_FLAG,0	    ;If b7 pressed
    GOTO    MODELOOP
    BCF	    bINT_BUTTON_FLAG,0
    GOTO    MASTERCLOCK		    ;B7 pressed is now master

;=========================================================================
;SUBR	SLAVE	-   TASK5
;PARAM	NONE
;RETURN	NONE
;WRITES	LATA, IOCBN, IOCBP, IOCBF, bINT_BUTTON_FLAG, LATC
;
;DESC	Sleeps until signal recieved on b6, then increments latc.
;	Resets to 0 at 24.
;
;CONFIG	b6 connected to b5 on another device
;CALLS	NONE
;=========================================================================
SLAVECLOCK:
    BANKSEL LATA
    ;RGB = RED
    BSF	    LATA,2
    BCF	    LATA,5
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,2	    ;Clear slave interrupt flag
    CLRF    PORTC
    ;Detect when b6, slave goes high
    BANKSEL IOCBN
    
    ;B6 detect change on low edge
    BCF	    IOCBN,6
    BSF	    IOCBP,6
    BCF	    IOCBF,6

    ;Detect up on b4 for reset
    BCF	    IOCBN,4
    BSF	    IOCBP,4
    BCF	    IOCBF,4

    ;Don't detect changes on b7
    BCF	    IOCBN,7
    BCF	    IOCBP,7
    BCF	    IOCBF,7

    ;No button a4
    BCF	    IOCAN,4

SLAVELOOP:
    BTFSC   bINT_BUTTON_FLAG,4
    RETURN
    SLEEP			    ;Interrupt will wake up
    MOVLB   0
    BTFSC   bINT_TIMER_FLAG,2	    ;If no change in slave pin state
    GOTO    ADDSLAVE
    GOTO    SLAVELOOP
ADDSLAVE:
    BANKSEL LATC
    MOVLW   0x17h	;23, next goes to 0
    SUBWF   LATC,0	;If no underflow, value is 23 and next should be 0
    BTFSC   STATUS,0
    GOTO    RESTARTSLAVE
    INCF    LATC
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,2	    ;Clear flag after handling
    GOTO    SLAVELOOP
RESTARTSLAVE:
    CLRF    LATC
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,2    ;Clear after handling
    GOTO    SLAVELOOP


;=========================================================================
;SUBR	MASTER	-   TASK5
;PARAM	NONE
;RETURN	NONE
;WRITES	LATA, TRISB, LATB, IOCBN, IOCBP, IOCBF, T1CON, PIR1, TMR1L, TMR1H
;	PIE1, INTCON, bINT_BUTTON_FLAG, bINT_TIMER_FLAG, OCSCON
;
;DESC	Uses flag from ISR generated by timer1 overflow to increment latc
;	every 1 second. Sends signal on b5 when reaching 60. B7 
;	switches speed between slow/fast.
;
;CONFIG	b5 should be connected to b6 input on another device
;CALLS	CLOCKSTABLE
;=========================================================================

MASTERCLOCK:
    ;RGB = BLUE
    BANKSEL LATA
    BCF	    LATA,2
    BSF	    LATA,5
    ;B4 set as output for reset signal
    BANKSEL TRISB
    BCF	    TRISB,4	    


    BANKSEL LATB
    BCF	    LATB,5	   ;Ground connected slave pin
    BSF	    LATB,4	   ;Trigger reset on slave, active high
    
    BANKSEL IOCBN
    ;Detect change on b7 down
    BSF	    IOCBN,7
    BCF	    IOCBP,7
    BCF	    IOCBF,7

    BANKSEL T1CON
    MOVLW   0x01h	;Timer1, use instruction cycle fosc/4, enable
    MOVWF   T1CON

    

    BANKSEL PIR1
    BCF	    PIR1,0	;clear  interrupt
    CALL    CLEARSTATE	;Make sure no previous interrupt
    BANKSEL PIR1
    MOVLW   0x3Ch	;65535 - 7875 = instructions added to overflow per sec
    MOVWF   TMR1L
    MOVLW   0xE1h
    MOVWF   TMR1H

    BANKSEL LATB	;Turn off reset signal
    BCF	    LATB,4

    BANKSEL PIE1
    BSF	    PIE1,0	;Enable timer1 interrupt
    BANKSEL INTCON	;Enable periphiral interrupt
    BSF	    INTCON,6
    MOVLB   0

;INCF on LATC could be done in interrupt
;This will add time to ISR when calculating for 60 seconds
;So use flags instead and deal with it here
;Can also easily use timer for multiple things this way
MASTERLOOP:
    MOVLB   0
    BTFSC   bINT_BUTTON_FLAG,4
    RETURN
    BTFSC   bINT_TIMER_FLAG,0	;If timer1 has overflowed, detected by interrupt
    CALL    MASTERADD
    BTFSC   bINT_BUTTON_FLAG,0	;B7 been pressed, detected by interrupt
    CALL    MASTERSPEED		;Change speed
    GOTO    MASTERLOOP
MASTERADD:
    BANKSEL LATC
    MOVLW   0x3bh	;59
    SUBWF   LATC,0	;If no underflow, value is 59 and next should be 0
    BTFSC   STATUS,0
    GOTO    RESTARTMASTER   ;Start at 0 again
    INCF    LATC
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,0	;Clear timer1 flag
    RETURN
RESTARTMASTER:
    BSF	    LATB,5	;Signal to slave, active high
    CLRF    LATC
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,0	    ;Dealt with flag, clear
    BANKSEL LATB
    BCF	    LATB,5	;Turn off slave signal
    RETURN

MASTERSPEED:
    MOVLB   0
    BCF	    bINT_BUTTON_FLAG,0	    ;Clear button flag
    BTFSC   bINT_TIMER_FLAG,1	    ;Check if currently fast or slow, 1 = fast
    GOTO    MSLOW
    GOTO    MSPEED
    
MSPEED:
    BANKSEL OSCCON	    ;Change prescaler
    MOVLW   0x3ah	    ;500Khz
    MOVWF   OSCCON
    CALL    CLOCKSTABLE	    ;Wait for stable frequency
    MOVLB   0
    BSF	    bINT_TIMER_FLAG,1	;Now fast
    RETURN
MSLOW:
    BANKSEL OSCCON
    MOVLW   0x12h	;00010010 - 31.5khz + internal osc
    MOVWF   OSCCON
    CALL    CLOCKSTABLE
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,1
    RETURN

;=========================================================================
;SUBR	INTCLOCK    -	TASK4
;PARAM	NONE
;RETURN	NONE
;WRITES	IOCBN, IOCBP, T1CON, PIR1, TMR1H, TMR1L, PIE1, INTCON, LATC, OCSCON
;	bINT_TIMER_FLAG, bINT_BUTTON_FLAG
;DESC	Clock counts to 60 using timer interupt. B7 changes speed slow/fast.
;
;CONFIG	Interrupt needs to set bINT_TIMER_FLAG(0) + bINT_BUTTON_FLAG
;CALLS	NONE
;=========================================================================

INTCLOCK:
    ;Change on high edge
    BANKSEL IOCBN
    BCF	    IOCBN,7
    BSF	    IOCBP,7

    BANKSEL T1CON
    MOVLW   0x01h	;Timer1, use instruction cycle fosc/4, enable
    MOVWF   T1CON
    ;Clear interrupt flag
    BANKSEL PIR1
    BCF	    PIR1,0
    ;65535 - 7875 = instructions added to overflow per sec
    MOVLW   0x3Ch
    MOVWF   TMR1L
    MOVLW   0xE1h
    MOVWF   TMR1H
    ;Enable timer1 interrupt
    BANKSEL PIE1
    BSF	    PIE1,0
    ;Enable periphiral interrupt
    BANKSEL INTCON
    BSF	    INTCON,6

INTCLOCKLOOP:
    MOVLB   0
    BTFSC   bINT_BUTTON_FLAG,4
    RETURN
    BANKSEL LATC
    MOVLW   0x3bh	;59
    SUBWF   LATC,0	;If no underflow, value is 59 and next should be 0
    BTFSC   STATUS,0
    GOTO    RESTARTCLOCK
    MOVLB   0
    BTFSC   bINT_TIMER_FLAG,0	;If timer1 overflowed
    CALL    ADDCOUNT
    BTFSC   bINT_BUTTON_FLAG,0	    ;If button has been pressed
    CALL    CHANGESPEED
    GOTO    INTCLOCKLOOP

ADDCOUNT:
    BANKSEL LATC
    INCF    LATC
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,0	;Clear flag
    RETURN
RESTARTCLOCK:
    CLRF    LATC
    BCF	    bINT_TIMER_FLAG,0	;Clear flag
    GOTO    INTCLOCKLOOP

CHANGESPEED:
    MOVLB   0
    BCF	    bINT_BUTTON_FLAG,0	;Clear button flag
    BTFSC   bINT_TIMER_FLAG,1	;Check current speed
    GOTO    SLOWDOWN
    GOTO    SPEEDUP
    
SPEEDUP:
    BANKSEL OSCCON  ;500Khz
    MOVLW   0x3ah
    MOVWF   OSCCON
    CALL    CLOCKSTABLE	;Wait for stable
    MOVLB   0
    BSF	    bINT_TIMER_FLAG,1
    RETURN
SLOWDOWN:
    BANKSEL OSCCON
    MOVLW   0x12h	;00010010 - 31.5khz + internal osc
    MOVWF   OSCCON
    CALL    CLOCKSTABLE
    MOVLB   0
    BCF	    bINT_TIMER_FLAG,1
    RETURN
    
;=========================================================================
;SUBR	BUTTONCOUNT	-   TASK3
;PARAM	NONE
;RETURN	NONE
;WRITES	LATC(PORTC),bINT_BUTTON_FLAG(0x025h)0
;
;DESC	Counts button presses on PORTC, ISR detects button presses.
;
;CONFIG	ISR sets button press in bINT_BUTTTON_FLAG,0.
;CALLS	NONE
;=========================================================================

BUTTONCOUNT:
    MOVLB   0
BUTTONCOUNTLOOP:
    BTFSC   bINT_BUTTON_FLAG,4
    RETURN
    BTFSC   bINT_BUTTON_FLAG,0	;Check button flag
    CALL    INCREMENT		;Increment if set
    GOTO    BUTTONCOUNTLOOP
INCREMENT:
    INCF    LATC
    BCF	    bINT_BUTTON_FLAG,0	;Clear flag
    RETURN

;=========================================================================
;SUBR	SIXTYSECCOUNT	-TASK2
;PARAM	NONE
;RETURN	NONE
;WRITES	LATC(PORTC),bDELAY_LOOP_COUNT
;
;DESC	Loops increments latc every second. Using software delay loops.
;	Resets at 60.
;	Delay calc: 7875 instrs per second
;	1 loop =  1 second,	7875 - 18(loop instructions) = 7857
;	7857 / 773(instructions per delayloop) =10 remainder 127
;	127/3 = 42 remain 1, 42 loops of added delay
;
;	
;
;CONFIG	NONE
;CALLS	DELAY,ADDEDDELAY
;=========================================================================

SIXTYSECCOUNT:
    MOVLB   0	    ;Start from 0
    CLRF    PORTC
SIXTYSECLOOP:	    //18 instructions in this loop
    BTFSC   bINT_BUTTON_FLAG,0
    RETURN
    MOVLW   0x0ah   ;Loop delays as calculated above
    MOVWF   bDELAY_LOOP_COUNT
    CALL    DELAY
    MOVLW   0x2Ah
    MOVWF   bDELAY_LOOP_COUNT
    CALL    ADDEDDELAY
    MOVLW   0x3Bh	;59
    SUBWF   LATC,0	;If no underflow, value is 59 and next should be 0
    BTFSC   STATUS,0
    GOTO    RESTART
    INCF    LATC,f
    NOP
    GOTO    SIXTYSECLOOP
RESTART:
    CLRF    LATC
    GOTO    SIXTYSECLOOP

;=========================================================================
;SUBR	ONESECCOUNT -TASK1
;PARAM	NONE
;RETURN	NONE
;WRITES	LATC(PORTC),bDELAY_LOOP_COUNT
;
;DESC	Loops increments latc every second. Using software delay loops.
;	7875 - 11 = 7864
;	7864/773 = 10 remain 134
;	134/3 = 44 remain 2. 44 added loops + 1 nop
;
;CONFIG	NONE
;CALLS	DELAY,ADDEDDELAY
;=========================================================================

ONESECCOUNT:
    MOVLB   0	    ;Start from 0
    CLRF    PORTC
ONESECLOOP:	    //11 instructions in this loop + 1 nop
    NOP
    MOVLW   0x0ah   ;Loop delays as calculated above
    MOVWF   bDELAY_LOOP_COUNT
    CALL    DELAY
    MOVLW   0x2Ch
    MOVWF   bDELAY_LOOP_COUNT
    CALL    ADDEDDELAY
    INCF    LATC,f
    GOTO    ONESECLOOP

;=========================================================================
;SUBR	DELAY
;PARAM	bDELAY_LOOP_COUNT
;RETURN	NONE
;WRITES	0x020h, bDELAY_LOOP_COUNT
;
;DESC	Nested loop to create software delay.
;	Loop instructions = 773 * bDELAY_LOOP_COUNT
;
;CONFIG	Select BANK0 before calling.
;CALLS	NONE
;=========================================================================

DELAY:
    bINNER_LOOP_COUNT	EQU 0x020h  ;General purpose register.
OUTERLOOP:
    MOVLW   0xffh
    MOVWF   bINNER_LOOP_COUNT
INNERLOOP:  //inner = (3 * 255)*outer +(8)outer  ==773*(outer)
    DECFSZ  bINNER_LOOP_COUNT, F
    GOTO    INNERLOOP
    NOP
    DECFSZ  bDELAY_LOOP_COUNT, F
    GOTO    CONSISTENT	;Adds extra goto so can match total instructions
    NOP
    RETURN
;Just to make instructions exact every loop
CONSISTENT:
    GOTO    OUTERLOOP

;=========================================================================
;SUBR	ADDEDDELAY
;PARAM	NONE
;RETURN	NONE
;WRITES	bDELAY_LOOP_COUNT
;
;DESC	Shorter software dealy. No nested loops. For precision.
;	Instructions = 3*loop + 1. Max off target instructions = 2. Use NOP.
;
;CONFIG	NONE
;CALLS	DEFAULT
;=========================================================================

ADDEDDELAY:	;3*loops + 1
    DECFSZ  bDELAY_LOOP_COUNT, F			   
    GOTO    ADDEDDELAY					   
    RETURN						    

;=========================================================================
;ISR	ISR
;PARAM	NONE
;RETURN	NONE
;WRITES	bINT_BUTTON_FLAG(0x025h),INTCON,IOCBF, TMR1H, TMR1L, TMR0
;
;DESC	Interupt Service Routine, checks source and sets flags.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================

;For finding source(s) of interrupt
ISR:
    BANKSEL PIR1	    ;Check timer1 overflow
    BTFSC   PIR1,0
    CALL    TIMER1ISR
    BANKSEL IOCBF
    BTFSC   IOCBF,7	    ;Check portb7 change
    CALL    BUTTON7PRESS    ;Call on change
    BTFSC   INTCON,5	    ;Check debounce when active, off between presses
    CALL    DEBOUNCE
    BANKSEL IOCBF	    ;Check change on slave pin
    BTFSC   IOCBF,6
    CALL    SLAVEISR	    
    BANKSEL IOCBF	    ;Check reset signal
    BTFSC   IOCBF,4
    CALL    SLAVESTART
    BANKSEL IOCAF
    BTFSC   IOCAF,4
    CALL    MODEBUTTON
    RETFIE
;Handles timer1 overflow interrupt. Set flag and reset timer. 
;Main decides what to do on event
TIMER1ISR:
    BCF	    PIR1,0	;Clear flag
    MOVLW   0xE1h	;65535 - 7875 = instructions added to overflow per sec
    ADDWF   TMR1H	;ADD so dont lose cycles
    MOVLW   0x3Ch
    ADDWF   TMR1L
    BSF	    bINT_TIMER_FLAG,0	;Set flag for main program
    RETURN
;B7 press, start debounce sequence
BUTTON7PRESS:
    MOVLB   0
    BSF	    bINT_BUTTON_FLAG,1	;Set button down detected, debounce before set 0
    BANKSEL IOCBF
    BCF	    IOCBF,7		;Clear interrupt
    BANKSEL TMR0
    MOVLW   0xD8h		;Reset timer
    MOVWF   TMR0
    BCF	    INTCON,0		;Clear interupt on change flag
    BCF	    INTCON,2
    BSF	    INTCON,5		;Turn on timer0 interupt, debounce timer
    RETURN
;Set b7 flag after debouncing and confirming proper press.
DEBOUNCE:
    BTFSS   INTCON,2		;If no overflow, return
    RETURN
    BCF	    INTCON,2		;Clear interupt flag
    BANKSEL TMR0
    MOVLW   0xD8h		;Reset timer
    MOVWF   TMR0
    MOVLB   0
    BTFSS   bINT_BUTTON_FLAG,1	;Only do something if b7 needs debouncing
    RETURN
    BANKSEL PORTB
    BTFSS   PORTB,7		;Poll pin. Down 2 polls in row before set
    GOTO    PRESSED
    GOTO    RELEASED
    RETURN
    ;bINT_BUTTON_FLAG,2 stores if last poll button was down
    ;bINT_BUTTON_FLAG,3 stores if last poll button was up   
    ;If same state 2 polls in a row, take action
PRESSED:			;Check down counter, if 1 set pressed
    BTFSC   bINT_BUTTON_FLAG,2
    GOTO    SETPRESSED
    BSF	    bINT_BUTTON_FLAG,2	;Increment down counter
    BCF	    bINT_BUTTON_FLAG,3	;Decrement up counter
    RETURN
RELEASED:
    BTFSC   bINT_BUTTON_FLAG,3	;Check up counter, reset b4change flag if 1
    GOTO    SETRELEASED
    BSF	    bINT_BUTTON_FLAG,3	;Increment up counter
    BCF	    bINT_BUTTON_FLAG,2	;Decrement down counter
    RETURN
SETPRESSED:
    BSF	    bINT_BUTTON_FLAG,0	;This indicates to main take action on b7 press
    BCF	    bINT_BUTTON_FLAG,1	;b7 change state reset. Is now stable
    BANKSEL INTCON
    BCF	    INTCON,5	;Turn off timer0 after debounce
    RETURN
SETRELEASED:
    BCF	    bINT_BUTTON_FLAG,0	;Button is stable released
    BCF	    bINT_BUTTON_FLAG,1	;Button no longer unstable
    BANKSEL INTCON
    BCF	    INTCON,5	;Turn off timer0 after debounce
    RETURN
;Responds to change on slave port
SLAVEISR:
    MOVLB   0
    BSF	    bINT_TIMER_FLAG,2	;Set b6 change flag
    BANKSEL IOCBF
    BCF	    IOCBF,6		;Clear interrupt flag
    RETURN
;Restart device on b4 signal
SLAVESTART:
    RESET
MODEBUTTON:
    BANKSEL IOCAF
    BCF	    IOCAF,4
    MOVLB   0
    BSF	    bINT_BUTTON_FLAG,4
    BCF	    INTCON,0		;Clear interupt on change flag	
    RETURN    
END