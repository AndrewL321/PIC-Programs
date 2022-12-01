;=========================================================================
;DESCR -  Door alarm program. Uses magnetic trigger on RB7 to start alarm
;	  that uses PWM.
;
;AUTHOR - Andrew Lucas
;
;VERSION - 2.00 - 10/12/21
;
;HISTORY - 1.00
;=========================================================================

TITLE "Coursework 4"
SUBTITLE "Door Alarm"

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
;=========================================================================
;Define variables for program. Use EQU over DS as only a small amount
;of variables. Easy to track memory address. DS was causing bugs.
;=========================================================================
 
    ;Flags set by ISR for main program
    INTERRUPT_FLAGS	EQU	0x020H
    ;Debounce - - - - TMR1 B7 B6 
	
    ;Pattern of buzzer. Start from lsb, 0 on, 1 off.
    BUZZER_PATTERN	EQU	0x021H
    ;Speed to loop through buzzer pattern. Higher = faster. 
    ;65335 - speed inst for overflow.
    BUZZER_SPEED_HIGH	EQU	0x022H
    BUZZER_SPEED_LOW	EQU	0x023H
    ;Times to play an alarm pattern
    ALARM_LOOPS		EQU	0x024H
    ;Used by alarm loop to store loop state
    PATTERN		EQU	0x025H
    LOOPS		EQU	0x026H
;=========================================================================
;BLOCK	MAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	ANSELB, ANSELC, PWM2CON, PIR1, T2CON, TRISC, OPTION_REG, WPUB
;	INTCON, IOCBN, IOCBP, T1CON, PIE1, OSCCON
;
;DESC	Sets up ports and interrupts for program. Lowers clock using
;	scalers to 31.25Khz.
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
	    
MAIN:
;=========================================================================
;Setup ports for program. Leave unused as analogue for less power leakage.
;=========================================================================
    PORTSETUP:
	BANKSEL	ANSELB	;RB6/7 digital.
	MOVLW	0x3FH
	MOVWF	ANSELB
	BCF	ANSC3	;RC3 digital.
	
	;Low power sleep when sleep called.
	BANKSEL	VREGCON
	BSF	VREGPM1
;=========================================================================
;Setup PWM on RC3.
;=========================================================================
    PWMSETUP:
	;Clear timer2 interupt flag
	BANKSEL	PIR1
	BCF	TMR2IF
	
	BANKSEL	T2CON
	MOVLW	0x04H	;Timer2 on. No scaler.	
	MOVWF	T2CON

	BANKSEL	TRISC
	BCF	TRISC3	;RC3 output
	
	BANKSEL	PWM2CON	
	MOVLW	0xE0H	;PWM2 eneabled, output on, active high.
	MOVWF	PWM2CON

;=========================================================================
;Set up for B6 and B7 as input. Active low for B7, high for B6.
;=========================================================================
    BUTTONSETUP:
	;Global weak pull up off. Timer0 clock source internal, 1:2 pre scaler.
	BANKSEL	OPTION_REG
	CLRF	OPTION_REG

	BANKSEL	WPUB	;Weak internal pullups on B6/7
	BSF	WPUB7
	BSF	WPUB6
	
	BANKSEL	INTCON
	MOVLW	0xC8H	;Enable interrupts, on change, timer0, periphiral
	MOVWF	INTCON

	BANKSEL	IOCBN
	BCF	IOCBN6	;Ignore change to low
	BSF	IOCBP6	;Detect change to high

	BSF	IOCBN7	;Detect change to low
	BCF	IOCBP7	;Ignore change to high

	CLRF	IOCBF	;Clear all current change interrupts
;=========================================================================
;Setup timer1 for program. Timer config is set when clear OPTION_REG above.
;=========================================================================
    TIMERSETUP:

	BANKSEL	T1CON
	MOVLW	0x01h	;Timer1, use instruction cycle fosc/4, enable
	MOVWF	T1CON
	    
	BANKSEL	PIR1
	BCF	TMR1IF	;clear  interrupt

	BANKSEL	PIE1
	BSF	TMR1IE	;Enable timer1 interrupt
;=========================================================================
;Sets clock to 31.25Khz and waits for it to stablise. 
;=========================================================================
    CLOCKSETUP:
	BANKSEL	OSCCON
	MOVLW	0x10h	;00010010 = 31.25khz + internal osc
	MOVWF	OSCCON
    CLOCKSTABLE:		;Loop until HF internal osc is stable
	BTFSS	OSCSTAT,0
	GOTO	CLOCKSTABLE
	
;=========================================================================
;BLOCK	ALARMMAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	INTERRUPT_FLAGS, PWM2CON, BUZZER_SPEED_HIGH, BUZZER_SPEED_LOW
;	BUZZER_PATTERN, ALARM_LOOPS, PR2, PWM2DCH, PWM2DCL, TMR1L, TMR1H
;
;DESC	Sleeps while waiting for interrupts. Starts alarm if door opened
;	and trigger delay on RB7 button press. Loops forever through modes.
	
;CONFIG	NONE
;CALLS	ALARMLOOP
;=========================================================================
	
ALARMMAIN:
	
;=========================================================================
;Deafult armed mode. Wait for input.
;=========================================================================
    ARMED:
	MOVLB	0
	CLRF	INTERRUPT_FLAGS	;Clear all flags
	BANKSEL	PORTB
	BTFSC	RB6		;If door open, start alarm
	CALL	SPEEDONE
    MAINLOOP:
	BANKSEL PWM2CON
	BCF	PWM2OE	    ;Turn off PWM2 output
	MOVLB	0
	BTFSS	INTERRUPT_FLAGS,7   ;If currently debouncing, don't sleep
	SLEEP			;Sleep. Low power mode set above
	MOVLB	0
	BTFSC	INTERRUPT_FLAGS,0   ;Check for B6 interrupt. If set start alarm
	GOTO	SPEEDONE
	BTFSS	INTERRUPT_FLAGS,1   ;Check for B7 interrupt
	GOTO	MAINLOOP
	
;=========================================================================
;30 sec trigger delay. Low freq double beep.
;=========================================================================
    TRIGGERDELAY:
	BANKSEL	PR2	;Set PWM period for 500hz sound. = 16
	MOVLW	0x10h
	MOVWF	PR2
	
	BANKSEL	PWM2DCH
	MOVLW	0x02H	;Half period, 50% duty cycle. = 8
	MOVWF	PWM2DCH
	
	BANKSEL	PWM2DCL
	MOVLW	0x00H
	MOVWF	PWM2DCL
	
	BANKSEL IOCBN	;Ignore B7 changes. Clear any pending interrupt.
	BCF	IOCBN7
	BCF	IOCBF7	

	MOVLB	0	;Clear button flags, and stop any debouncing
	BCF	INTERRUPT_FLAGS,7
	BCF	INTERRUPT_FLAGS,1
	
	MOVLW	0x80H	;Buzzer speed = 7812.5/((65355 - speed)*8) sec per loop
	MOVWF	BUZZER_SPEED_LOW
	MOVWF   TMR1L

	MOVLW	0xF4H
	MOVWF	BUZZER_SPEED_HIGH
	MOVWF   TMR1H
	
	MOVLW	0x1BH	;00011011 = 0 = on, 1 = off. Shifts right.
	MOVWF	BUZZER_PATTERN
	
	MOVLW	0x0AH	;10 loops for total 30 sec
	MOVWF	ALARM_LOOPS
	
	BCF	INTERRUPT_FLAGS,2   ;Clear timer interrupt
	CALL	ALARMLOOP
	BCF	INTERRUPT_FLAGS,0   ;Clear button interrupt
	
	BANKSEL IOCBN	;Turn interrupt on change back on
	BSF	IOCBN7	;Detect change to low

	GOTO	ARMED

;=========================================================================
;BLOCK	SPEEDRAMP
;PARAM	NONE
;RETURN	NONE
;WRITES	PR2, PWM2DCH, PWMDCL, TMR1H, TMR1L, BUZZER_SPEED_HIGH,
;	BUZZER_SPEED_LOW, BUZZER_PATTERN, ALARM_LOOPS, INTERRUPT_FLAGS
;
;DESC	Ramps up speed on 10 sec interval for 30 sec then constant.
;	Returns on B7 down.
;
;CONFIG	NONE
;CALLS	ALARMLOOP
;=========================================================================
;=========================================================================
;2 beeps per second for 10 sec
;=========================================================================
    SPEEDONE:
	BANKSEL	PR2
	MOVLW	0x08h	;Period = 2ms, 1000Hz
	MOVWF	PR2
	
	BANKSEL	PWM2DCH	;50% duty cycle. Half period
	MOVLW	0x01H
	MOVWF	PWM2DCH
	
	BANKSEL	PWM2DCL
	MOVLW	0x00H
	MOVWF	PWM2DCL
	
	MOVLB	0
	BCF	INTERRUPT_FLAGS,1   ;Clear timer flag
	
	MOVLW	0x26H	;Buzzer speed = 7812.5/((65355 - speed)*8) sec per loop
	MOVWF	BUZZER_SPEED_LOW
	MOVWF   TMR1L

	MOVLW	0xFCH
	MOVWF	BUZZER_SPEED_HIGH
	MOVWF   TMR1H
	
	MOVLW	0x66H	; 00110011. 0 = off, 1 = on. Shifts right.
	MOVWF	BUZZER_PATTERN
	
	MOVLW	0x0AH	;Total loops. 10 with 1 sec per loop.
	MOVWF	ALARM_LOOPS
		
	CALL	ALARMLOOP
	
	BTFSC   INTERRUPT_FLAGS,1   ;If button was pressed go to ARMED.
	GOTO	ARMED
	
;=========================================================================
;3 beeps per second for 10 sec
;=========================================================================
    SPEEDTWO:
	MOVLB	0	;Speed up same patern for 3 beeps per second
	MOVLW	0x6EH	;Buzzer speed = 7812.5/((65355 - speed)*8) sec per loop
	MOVWF	BUZZER_SPEED_LOW    
	MOVWF   TMR1L

	MOVLW	0xFDH
	MOVWF	BUZZER_SPEED_HIGH
	MOVWF   TMR1H
	
	MOVLW	0x0FH	;15 loops for 30 seconds.
	MOVWF	ALARM_LOOPS
		
	CALL	ALARMLOOP
	
	BTFSC   INTERRUPT_FLAGS,1   ;Check if B7 pressed. Return if it was.
	GOTO	ARMED
;=========================================================================
;4 beeps per second until button pressed
;=========================================================================	
    SPEEDTHREE:
	MOVLB	0	;Speed up for 4 beeps per second.
	MOVLW	0x12H	;Buzzer speed = 7812.5/((65355 - speed)*8) sec per loop
	MOVWF	BUZZER_SPEED_LOW
	MOVWF   TMR1L

	MOVLW	0xFEH
	MOVWF	BUZZER_SPEED_HIGH
	MOVWF   TMR1H
	
	MOVLW	0x14H	;20 loops for 10 sec loop.
	MOVWF	ALARM_LOOPS
		
	CALL	ALARMLOOP
	
	BTFSC   INTERRUPT_FLAGS,1   ;Loop until button pressed
	GOTO	ARMED
	
	GOTO	SPEEDTHREE
	
	
;=========================================================================
;BLOCK	ALARMLOOP
;PARAM	BUZZER_PATTERN, ALARM_LOOPS, BUZZER_SPEED
;RETURN	NONE
;WRITES	ALARM_LOOPS, INTERRUPT_FLAGS, PATTERN, LOOPS
;
;DESC	Triggers PWM on off to play pattern specified at speed specified in 
;	BUZZER_PATTERN and BUZZER_SPEED. Alarm loops controls how many times
;	to repeat pattern. Runtime approx = loops * 8 * speed
;
;CONFIG	Buzzer pattern in BUZZER_PATTERN. 1 = on, 0 off. 
;	Loops in ALARM_LOOPS. Speed in BUZZER_SPEED.
	
;CALLS	NONE
;=========================================================================
	
    ALARMLOOP:
	BCF	INTERRUPT_FLAGS,2   ;Clear timer1 flag
	BTFSC   INTERRUPT_FLAGS,1   ;Check if B7 been pressed
	RETURN
	
	MOVF	BUZZER_PATTERN,W    ;Get pattern
	MOVWF	PATTERN		    ;Move to temp register
	
	MOVLW	0x08H		    ;8 loops to cover whole byte
	MOVWF	LOOPS
	
	CALL	PLAYLOOP
	
	DECFSZ	ALARM_LOOPS	    ;Loop until 0
	GOTO	ALARMLOOP
	
	RETURN
	
;=========================================================================
;Plays current pattern
;=========================================================================
	PLAYLOOP: 
	    MOVLB   0
	    BTFSC   INTERRUPT_FLAGS,1	;Return if B7 pressed
	    RETURN
	    BTFSS   INTERRUPT_FLAGS,2	;Loop until timer1 flag set
	    GOTO    PLAYLOOP
	    BCF	    INTERRUPT_FLAGS,2	;Clear flag
	    BTFSS   PATTERN,0		;Set on if 1, off if 0
	    GOTO    SETOFF
;=========================================================================
;Set PWM output on
;=========================================================================	        
	SETON:
	    BANKSEL PWM2CON
	    BSF	    PWM2OE  ;Turn PWM2 output on
	    MOVLB   0
	    LSRF    PATTERN,F	;Shift right to check next bit
	    DECFSZ  LOOPS	;Loop until 0
	    GOTO    PLAYLOOP
	    RETURN
;=========================================================================
;Set PWM output off
;=========================================================================
	SETOFF:
	    BANKSEL PWM2CON
	    BCF	    PWM2OE  ;Turn PWM2 output off
	    MOVLB   0
	    LSRF    PATTERN,F	;Shift right to check next bit
	    DECFSZ  LOOPS	;Loop until 0
	    GOTO    PLAYLOOP
	    RETURN	
	    
;=========================================================================
;BLOCK	ISR
;PARAM	NONE
;RETURN	NONE
;WRITES	INTERRUPT_FLAGS
;
;DESC	Checks source of interrupt and sets flag for main program.
;	RB7 int runs timer0.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
	    
ISR:
    BANKSEL IOCBF   ;Change on B6
    BTFSC   IOCBF6  
    CALL    BSIX
    BANKSEL IOCBF   ;Change on B7
    BTFSC   IOCBF7 
    CALL    BSEVEN
    BANKSEL PIR1	    ;Check timer1 overflow
    BTFSC   TMR1IF
    CALL    TIMER1ISR
    BTFSC   TMR0IF	;Check timer0
    CALL    TIMER0ISR		
    RETFIE
    
;=========================================================================
;B6 interrupt. Set flag and clear interrupt
;=========================================================================
    BSIX:
	BANKSEL IOCBF	;Clear interrupts
	BCF	IOCBF6
	BANKSEL INTCON	;Clear change flag
	BCF	IOCIF
	MOVLB   0
	BSF	INTERRUPT_FLAGS,0   ;Set flag for main
	RETURN
;=========================================================================
;B7 interrupt. Set flag and clear interrupt. Start timer0 for debounce.
;B7 flag not set unless consistent down for 50ms.
;=========================================================================	
    BSEVEN:
	BANKSEL IOCBF	;Clear interrupts
	BCF	IOCBF7
	BANKSEL INTCON
	BCF	IOCIF	;Clear on change flag
	MOVLW	0x3CH	;Reset timer0. 60 for 50ms overflow.
	BANKSEL	TMR0
	MOVWF	TMR0
	BANKSEL	INTCON
	BCF	TMR0IF
	BSF	TMR0IE	;Clear flag and turn on timer0
	MOVLB	0
	BSF	INTERRUPT_FLAGS,7   ;Set debounce flag
	RETURN
;=========================================================================
;Timer1 interrupt. Clear int, reset timer and set flag.
;=========================================================================
	
    TIMER1ISR:
	BCF	TMR1IF	;Clear flag
	MOVF	BUZZER_SPEED_HIGH,W ;Get current speed
	ADDWF	TMR1H	;ADD so dont lose cycles
	MOVF	BUZZER_SPEED_LOW,W
	ADDWF	TMR1L
	MOVLB	0
	BSF	INTERRUPT_FLAGS,2	;Set flag for main program
	RETURN
;=========================================================================
;Timer0 50ms interrupt timer for debouncing. Check B7 down and set flag.
;=========================================================================
    TIMER0ISR:
	BCF	TMR0IF		;Clear interupt flag
	BCF	TMR0IE		;Turn off timer0
	BANKSEL	PORTB
	BTFSC	RB7		;Check if B7 pressed
	RETURN
	MOVLB	0
	BTFSS	INTERRUPT_FLAGS,7   ;Return if no current debounce
	RETURN
	BSF	INTERRUPT_FLAGS,1   ;Set B7 flag
	BCF	INTERRUPT_FLAGS,7   ;Clear debounce flag
	RETURN
END