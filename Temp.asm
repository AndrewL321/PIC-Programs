;=========================================================================
;PROGRAM - SCC369 Coursework 3 Temperature
;
;DESCR -  Use I2C to read values from temperature sensor
;	  Display readings on portc and track min max.
;
;AUTHOR - Andrew Lucas
;
;VERSION - 2.00 - 18/11/21
;
;HISTORY - 1.00
;=========================================================================

TITLE "Coursework 3"
SUBTITLE "Temperature"

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

READING	    EQU	0x022H	    ;For reading current data frame

HIGHREADING EQU	0x26H	    ;For stroing latest high byte

LOWREADING  EQU	0x25H	    ;For storing latest low byte

INTERRUPT_FLAGS	EQU 0x028H  ;Register for storing result of interrupt
;NOACK - - - - - b6 tmr0

LED_REGISTER	EQU 0x030H  ;Current LED setup for loaded value
LED_STATUS	EQU 0x029H  ;For storing state of flashing LEDS
; - - - - max min maxon minon

TEMP_BOUNDS     EQU  0x31H  ;Start address of 'array' of temp boundarys for led

TEMPADDRESS	EQU 0x021H  ;Address to request read from


LOOPCOUNT	EQU 0x20H   ;Stores int for looping

MIN_READ_HIGH	EQU	0x50H	;Current high byte of min read
MIN_READ_LOW	EQU	0x51H	;Current low byte of min read
MIN_LED		EQU	0x54H	;LED setup for min
MAX_READ_HIGH	EQU	0x52H	;High max
MAX_READ_LOW	EQU	0x53H	;Low max
MAX_LED		EQU	0x55H	;Max LED setup

NEG_SUB_TEMP	EQU	0x56H	;Stores 2's compliment for negative sub
NEG_SUB_TEMP2	EQU	0x57H
;=========================================================================
;BLOCK	MAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	INTERRUPT_FLAGS
;
;DESC	For setting up device, init min/max registers and looping
;	on default mode. Takes reading every 0.25 secs and displays result.
;
;CONFIG	NONE
;CALLS	PORTSETUP, RANGEINTERVALSETUP, INTERRUPTSETUP, RESETI2C, ENDCONDITION
;	SETMIN, SETMAX, SET_MIN_LED, SET_MAX_LED, DEFAULTLED, WAITFORTIMER0
;	WAITFORBUTTON, READ, DISPLAY, MODELOOP
;=========================================================================

MAIN:
    CALL    PORTSETUP		
    CALL    RANGEVALUESETUP	;Loads range values for led states
    CALL    INTERRUPTSETUP
;=========================================================================
;Default mode on startup. Runs initial read then loops until b6 held.
;=========================================================================
INITIALRUN:
    MOVLB   0
    BCF	    INTERRUPT_FLAGS,7
    CALL    RESETI2C		;Make sure sensor not outputting
    CALL    ENDCONDITION	;Make sure is not mid transfer
    CALL    READ
    CALL    SETMIN		;Init min
    CALL    SETMAX		;Init max
    CALL    SET_MIN_LED
    CALL    SET_MAX_LED
DEFAULT:
    CALL    DEFAULTLED
    CALL    WAITFORTIMER0	;0.5 sec delay
    CALL    WAITFORTIMER0
    CALL    OFFLED
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
DEFAULTLOOP:
    CALL    READ		;Call read loop
    MOVLB   0
    BTFSC   INTERRUPT_FLAGS,7
    GOTO    INITIALRUN
    CALL    DISPLAY		;Display latest read
    CALL    WAITFORTIMER0
    MOVLB   0
    BTFSC   INTERRUPT_FLAGS,1
    GOTO    BUTTONTEST		;Go to mode loop if pressed
    GOTO    DEFAULTLOOP
BUTTONTEST:
    BCF	    INTERRUPT_FLAGS,1	;If down triggered leave default
    BANKSEL PORTB
    BTFSC   RB6
    GOTO    DEFAULTLOOP
    GOTO    MODELOOP

;=========================================================================
;BLOCK	MAIN
;PARAM	NONE
;RETURN	NONE
;WRITES	INTERRUOT_FLAGS
;
;DESC	Loop through modes every 2 secs while button pressed.
;	Return to default if button up for 2 secs.
;
;CONFIG	NONE
;CALLS	DEFAULT, WAITFORTIMER0, WAITFORBUTTON, DISPLAY, DISPLAYMIN, DISPLAYMAX
;	READ
;=========================================================================
MODELOOP:
    CALL    MINMODE
    BANKSEL PORTB
    BTFSC   RB6		    ;Check if b6 up after leaving
    GOTO    DEFAULT	    ;Return to default if it is
    CALL    MAXMODE	    ;Leaves every 2 seconds
    BANKSEL PORTB
    BTFSC   RB6
    GOTO    DEFAULT
    CALL    READINGMODE
    BANKSEL PORTB
    BTFSC   RB6
    GOTO    DEFAULT
    GOTO    MODELOOP

;=========================================================================
;Display min read.
;=========================================================================
MINMODE:
    CALL    MINLED
    CALL    WAITFORTIMER0   ;Small delay for button flash
    CALL    WAITFORTIMER0
    CALL    OFFLED
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    CALL    DISPLAYMIN
    CALL    WAITFORBUTTON   ;Wait for next button intrerrupt
    RETURN
;=========================================================================
;Display max read.
;=========================================================================
MAXMODE:
    CALL    MAXLED
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    CALL    OFFLED
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    CALL    DISPLAYMAX
    CALL    WAITFORBUTTON
    RETURN
;=========================================================================
;Display current reading.
;=========================================================================
READINGMODE:
    CALL    DEFAULTLED
    CALL    WAITFORTIMER0   ;Small delays
    CALL    WAITFORTIMER0
    CALL    OFFLED
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    CALL    READ
    CALL    DISPLAY
    CALL    WAITFORBUTTON
    RETURN

;=========================================================================
;Wait for switch to be same state for 2 sec.
;=========================================================================
WAITFORBUTTON:
    MOVLB   0
    BTFSS   INTERRUPT_FLAGS,1
    GOTO    WAITFORBUTTON
    BCF	    INTERRUPT_FLAGS,1
    RETURN
;=========================================================================
;Wait for flag from timer0 interrupt. 0.25 sec.
;=========================================================================
WAITFORTIMER0:
    MOVLB   0
    BTFSS   INTERRUPT_FLAGS,0
    GOTO    WAITFORTIMER0
    BCF	    INTERRUPT_FLAGS,0
    RETURN


;=========================================================================
;BLOCK	LED
;PARAM	NONE
;RETURN	NONE
;WRITES	PORTC
;
;DESC	For setting state of leds for each mode. Avoids changing c6/c7.
;	Same instructions as XOR and AND + IOR for 4 bits to set bits.
;	Easier to read
;
;CONFIG	PORTC 2-5 set as output.
;CALLS	NONE
;=========================================================================

;1111
DEFAULTLED:
    BANKSEL PORTC
    BSF	    RC5
    BSF	    RC4
    BSF	    RC3
    BSF	    RC2
    RETURN
;0001
MINLED:
    BANKSEL PORTC
    BCF	    RC5
    BCF	    RC4
    BCF	    RC3
    BSF	    RC2
    RETURN
;1000
MAXLED:
    BANKSEL PORTC
    BSF	    RC5
    BCF	    RC4
    BCF	    RC3
    BCF	    RC2
    RETURN
;0000
OFFLED:
    BANKSEL PORTC
    BCF	    RC5
    BCF	    RC4
    BCF	    RC3
    BCF	    RC2
    RETURN


;=========================================================================
;BLOCK	READ
;PARAM	NONE
;RETURN	NONE
;WRITES	HIGHREADING, LOWREADING
;
;DESC	Reads temperature sensor via I2C. Sends request and stores response.
;	Displays result and checks min max.
;	Main loop of program.
;
;CONFIG	NONE
;CALLS	SENDACK, GETACK, TEMPREQUEST, ENDCONDITION, DEFAULTSTATE, RANGETEST
;	MINMAXTEST
;=========================================================================

READ:
    CALL    DEFAULTSTATE
READLOOP:
    CALL    TEMPREQUEST	;Send request
    CALL    GETACK	;Wait for ack before reading
    MOVLB   0		;If no ack recieved, return and restart
    BTFSC   INTERRUPT_FLAGS,7
    RETURN
    CALL    READTEMP
    MOVLB   0
    MOVF    READING,W
    MOVWF   HIGHREADING	;Store first reading in high byte
    CALL    SENDACK
    CALL    READTEMP
    MOVLB   0
    MOVF    READING,W
    MOVWF   LOWREADING	;Second is low
    CALL    SETSCLLOW	;This is to send nack
    CALL    SDAOUTPUT
    CALL    SETSCLHIGH
    CALL    SETSCLLOW
    CALL    ENDCONDITION    ;End communication
    CALL    RANGETEST	    ;Check which range reading is in
    CALL    MINMAXTEST	    ;Check if new reading is a max/min
    RETURN

;=========================================================================
;BLOCK	SETUP
;PARAM	NONE
;RETURN	NONE
;WRITES	0x031H-0x040H, INTCON, OPTION_REG, T1CON, IOCBN, ANSELC, ANSELB
;	PIE1, TRISC, WPUBEN, RC6, RC7
;
;DESC	For setting up SFRs for program and loading values into GPRs.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================

;=========================================================================
;Set intterupt SFRs.
;=========================================================================
INTERRUPTSETUP:
    BANKSEL INTCON
    BSF	    GIE	    ;Global interrupt
    BSF	    PEIE    ;Periphiral
    BSF	    TMR0IE  ;Timer0

    BANKSEL OPTION_REG
    BCF	    TMR0CS  ;Use internal clock as source

;Prescaler on for timer0.
    BCF	    PSA 
    BSF	    PS2
    BSF	    PS1
    BSF	    PS0

    BANKSEL T1CON
    MOVLW   0x31H	;Timer1, use instruction cycle fosc/4, enable
    MOVWF   T1CON

    BANKSEL IOCBN
    ;Detect change on b7 down
    BSF	    IOCBN6  ;Down
    BSF	    IOCBP6  ;Up
    BCF	    IOCBF6

    BCF	    IOCIF   ;Change flag
    BSF	    IOCIE   ;On change
    BANKSEL PIE1
    BSF	    TMR1IE  ;Timer1 enable interrupt

    RETURN

;=========================================================================
;Default setup for ports on startup
;=========================================================================

PORTSETUP:
    BANKSEL ANSELC  ;Set to digital
    CLRF    ANSELC

    BANKSEL TRISC   ;C2-7 as output
    CLRF    TRISC  
    BSF	    TRISB6  ;B6 input
    
    BANKSEL OPTION_REG	;Disable global weak pull up
    BCF	    nWPUEN 

    BANKSEL WPUB	;Enable weak pull ups on b6, portc 
    BSF	    WPUB6

    MOVLB   0
    CLRF    LED_STATUS

    RETURN
;=========================================================================
;Default state for c6 / c7. High
;=========================================================================

DEFAULTSTATE:
    MOVLB   0
    BANKSEL LATC	;2-5 low, 6-7 high
    BSF	    RC6
    BSF     RC7

    RETURN
;=========================================================================
;Load boundary values and corresponding LED state into general purpose.
;=========================================================================

RANGEVALUESETUP:

    ;LED test boundary values
    MOVLB   0
    CLRF    LED_REGISTER
    BCF	    INTERRUPT_FLAGS,1

    MOVLW   0x0FH
    MOVWF   0x31H
    MOVLW   0x01H
    MOVWF   0x39H

    MOVLW   0x14H
    MOVWF   0x32H
    MOVLW   0x04H
    MOVWF   0x3AH

    MOVLW   0x19H
    MOVWF   0x33H
    MOVLW   0x0CH
    MOVWF   0x3BH

    MOVLW   0x1EH
    MOVWF   0x34H
    MOVLW   0x08H
    MOVWF   0x3CH

    MOVLW   0x23H
    MOVWF   0x35H
    MOVLW   0x18H
    MOVWF   0x3DH

    MOVLW   0x25H
    MOVWF   0x36H
    MOVLW   0x10H
    MOVWF   0x3EH

    MOVLW   0x27H
    MOVWF   0x37H
    MOVLW   0x30H
    MOVWF   0x3FH

    MOVLW   0x28H
    MOVWF   0x38H
    MOVLW   0x20H
    MOVWF   0x40H

    RETURN
;=========================================================================
;BLOCK	I2CCONFIG
;PARAM	NONE
;RETURN	NONE
;WRITES	RC7, RC6, TRIS7
;
;DESC	Subroutines for setting state of SDA and SCL. On RC7 + RC6
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
;=========================================================================
;Change SDA to output
;=========================================================================
SDAOUTPUT:
    BANKSEL TRISC	;Set as ouput
    BCF	    TRISC7 
    BANKSEL PORTC	;Default is high
    BSF	    RC7
    RETURN
;=========================================================================
;Change SDA to input
;=========================================================================
SDAINPUT:
    BANKSEL TRISC	;Set as input
    BSF	    TRISC7 
    RETURN

;=========================================================================
;For switching state of SDA and SCL. Call adds approx 4 inst delay to cycle
;=========================================================================
SETSDAHIGH:
    BANKSEL PORTC
    BSF	    RC7
    RETURN
SETSDALOW:
    BANKSEL PORTC
    BCF	    RC7
    RETURN
SETSCLHIGH:
    BANKSEL PORTC
    BSF	    RC6
    RETURN
SETSCLLOW:
    BANKSEL PORTC
    BCF	    RC6
    RETURN


;=========================================================================
;BLOCK	STARTANDEND
;PARAM	NONE
;RETURN	NONE
;WRITES	NONE
;
;DESC	Utility subroutines for start and endcondition + reset i2c.
;
;CONFIG	NONE
;CALLS	SETSCLOW, SETSCLHIGH, SDAINPUT, SDAOOUTPUT, SETSDAHIGH, SETSDALOW
;=========================================================================
;=========================================================================
;Start condition for I2C.
;=========================================================================
STARTCONDITION:
    CALL	SETSCLLOW
    BANKSEL	PORTC
    CALL	SDAOUTPUT
    CALL	SETSDAHIGH
    CALL	SETSCLHIGH
    CALL	SETSDALOW
    CALL	SETSCLLOW
    RETURN
;=========================================================================
;End for i2c.
;=========================================================================
ENDCONDITION:
    CALL    SETSCLLOW
    CALL    SDAOUTPUT
    CALL    SETSDALOW
    CALL    SETSCLHIGH
    CALL    SETSDAHIGH
    CALL    SETSCLLOW
    CALL    SETSDALOW
    RETURN
;=========================================================================
;Toggle SCL until sda not pulled low. Get sensor out of output.
;For making sure its not stuck on startup.
;=========================================================================
RESETI2C:
    CALL    SETSCLHIGH
    CALL    SETSCLLOW
    CALL    SDAINPUT
    BTFSS   PORTC,7
    GOTO    RESETI2C
    CALL    SDAOUTPUT
    RETURN
;=========================================================================
;BLOCK	TEMPREQUEST
;PARAM	NONE
;RETURN	NONE
;WRITES	NONE
;
;DESC	Sends request address plus read bit. Uses rc6 + rc7 for scl and sda.
;	Waits for ACK.
;
;CONFIG	NONE
;CALLS	SETSCLHIGH, SETSSCLLOW, SETSDAHIGH, SETSDALOW, STARTCONDITION
;=========================================================================
;=========================================================================
;Send request address
;=========================================================================
TEMPREQUEST:
    MOVLB   0

    MOVLW   0x08H
    MOVWF   LOOPCOUNT
    
    MOVLW   0x91H   ;0x48H + 1 for read bit
    MOVWF   TEMPADDRESS
    CALL    STARTCONDITION

;=========================================================================
;Loop through address. 1 = SDA high. Loop 8 times.
;=========================================================================
TEMPREQUESTLOOP:
    CALL    SETSDALOW
    MOVLB   0
    BTFSC   TEMPADDRESS,7
    CALL    SETSDAHIGH
    MOVLB   0
    LSLF    TEMPADDRESS,F
    CALL    SETSCLHIGH
    CALL    SETSCLLOW
    MOVLB   0
    DECFSZ  LOOPCOUNT
    GOTO    TEMPREQUESTLOOP
    RETURN

;=========================================================================
;Send ack.  SDA low.
;=========================================================================
SENDACK:
    CALL    SDAOUTPUT
    CALL    SETSDALOW
    CALL    SETSCLHIGH
    CALL    SETSCLLOW
    CALL    SETSDALOW
    CALL    SDAINPUT
    RETURN
;=========================================================================
;Wait for SDA to be pulled low as ack from sensor.
;=========================================================================
GETACK:
    CALL    SDAINPUT
    CALL    SETSCLHIGH
GETACKLOOP:
    BTFSC   RC7
    GOTO    NOACK    
    CALL    SETSCLLOW
    RETURN
NOACK:
    MOVLB   0
    BSF	    INTERRUPT_FLAGS,7
    RETURN


;=========================================================================
;BLOCK	TEMPREAD
;PARAM	NONE
;RETURN	NONE
;WRITES	LOOPCOUNT, READING
;
;DESC	Loops 8 times and stores reading. 0 if SDA(RC7) low, 1 if high
;
;CONFIG	NONE
;CALLS	SETSCLHIGH, SETSCLLOW
;=========================================================================
;=========================================================================
;Setup for read loop
;=========================================================================
READTEMP:
    CALL    SDAINPUT
    MOVLB   0
    MOVLW   0x08H
    MOVWF   LOOPCOUNT
    CLRF    READING
;=========================================================================
;Toggle SCL whilst reading values from SDA. SDA low = 0
;=========================================================================
READTEMPLOOP:
    LSLF    READING,F
    CALL    SETSCLHIGH
    BANKSEL PORTC
    BTFSC   RC7
    CALL    INCREADING
    CALL    SETSCLLOW
    MOVLB   0
    DECFSZ  LOOPCOUNT
    GOTO    READTEMPLOOP   
    RETURN
;=========================================================================
;Set to 1 if SDA high.
;=========================================================================
INCREADING:
    MOVLB   0
    INCF    READING,F
    RETURN

;=========================================================================
;BLOCK	TEMPDISPLAY
;PARAM	NONE
;RETURN	NONE
;WRITES	
;
;DESC	
;
;CONFIG	NONE
;CALLS	
;=========================================================================
;=========================================================================
;Display latest reading.
;=========================================================================
DISPLAY:

    MOVLB   0
    MOVF    LED_REGISTER,W
    BANKSEL PORTC
    MOVWF   PORTC
    RETURN
;=========================================================================
;Display stored max.
;=========================================================================
DISPLAYMAX:
    MOVLB   0
    BTFSC   LED_STATUS,3
    GOTO    MAXDISPLAY
    MOVF    MAX_LED,W
    BANKSEL PORTC
    MOVWF   PORTC
    RETURN
;=========================================================================
;Display stored min.
;=========================================================================
DISPLAYMIN:
    MOVLB   0
    BTFSC   LED_STATUS,2
    GOTO    MINDISPLAY
    MOVLB   0
    MOVF    MIN_LED,W
    BANKSEL PORTC
    MOVWF   PORTC
    RETURN
   

 ;=========================================================================
;Flashes on min/max.
;=========================================================================
MAXDISPLAY:
    MOVLW   0x20h
    BANKSEL PORTC
    MOVWF   PORTC
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    BCF	    RC5
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    BSF	    RC5
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    RETURN

MINDISPLAY:
    MOVLW   0x04h
    BANKSEL PORTC
    MOVWF   PORTC
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    BCF	    RC4
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    BSF	    RC4
    CALL    WAITFORTIMER0
    CALL    WAITFORTIMER0
    RETURN
    
  
;=========================================================================
;BLOCK	RANGETEST
;PARAM	NONE
;RETURN	NONE
;WRITES	FSR0L, FSR0H, FSR1L, FSR1H, LED_STATUS, LED_REGISTER
;
;DESC	Tests which boundary reading is in for LED dsiplay
;
;CONFIG	0x31H holds firts boundary, 0x039H holds first led state
;	Both have 'arrays' of 8 values.
;CALLS	
;=========================================================================  
;=========================================================================
;Setup loads start address for fsrs. FSR0 holds address of boundary numbers
;FSR1 holds led state linked to FSR0 boundary
;=========================================================================
RANGETEST: 
    MOVLB   0
    MOVLW   0x31H	;Start address for boundarys
    MOVWF   FSR0L
    CLRF    FSR0H
    
    MOVLW   0x39H	;Start address for led state
    MOVWF   FSR1L
    CLRF    FSR1H

;=========================================================================
;DTest if reading is below 15oc. Flshing state if it is.
;=========================================================================
LOWTEST:
    BTFSC   HIGHREADING,7
    GOTO    LOWRANGE
    CALL    TESTCURRENT
    BTFSC   LED_REGISTER,0
    GOTO    LOWRANGE
    INCF    FSR0L
    INCF    FSR1L 

    MOVLW   0x7H
    MOVWF   LOOPCOUNT
;=========================================================================
;Loop through boundary. Set led to stored state if above current boundary.
;=========================================================================   
RANGELOOP:
    CALL    TESTCURRENT
    BTFSC   LED_REGISTER,0
    GOTO    CURRENTRANGE
    INCF    FSR0L
    INCF    FSR1L
    DECFSZ  LOOPCOUNT
    GOTO    RANGELOOP

    GOTO    HIGHRANGE
    RETURN
;=========================================================================
;Tests if current reading is greater than current boundary
;=========================================================================
TESTCURRENT:
    MOVF    INDF0,W
    SUBWF   HIGHREADING,W
    BTFSS   STATUS,0
    GOTO    SETRANGE
    BTFSC   STATUS,2
    GOTO    BOUNDARYTEST
    RETURN
    

;=========================================================================
;Test for exact high byte match. Check if low is non zero.
;=========================================================================
BOUNDARYTEST:
    MOVF    LOWREADING,1
    BTFSC   STATUS,2
    BSF	    LED_REGISTER,0
    RETURN
;=========================================================================
;Indicate led state should be set to current value.
;=========================================================================
SETRANGE:
    BSF	    LED_REGISTER,0
    RETURN
CURRENTRANGE:
    MOVF    INDF1,W
    MOVWF   LED_REGISTER
    RETURN
;=========================================================================
;Sets LED state. Will flash if in relevant range.
;=========================================================================
LOWRANGE:
    CLRF    LED_REGISTER
    BSF	    LED_STATUS,2
    BTFSC   LED_STATUS,0
    GOTO    SETLOWON
    BSF	    LED_STATUS,0
    RETURN
SETLOWON:
    BCF	    LED_STATUS,0
    BSF	    LED_REGISTER,2
    RETURN
HIGHRANGE:
    CLRF    LED_REGISTER
    BSF	    LED_STATUS,3
    BTFSC   LED_STATUS,1
    GOTO    SETHIGHON
    BSF	    LED_STATUS,1
    RETURN
SETHIGHON:
    BCF	    LED_STATUS,1
    BSF	    LED_REGISTER,5
    RETURN

;=========================================================================
;BLOCK	MINMAXTEST
;PARAM	NONE
;RETURN	NONE
;WRITES	MIN_LED, MAX_LED, MAX_READ_HIGH, MAX_READ_LOW
;MIN_READ_HIGH, MIN_READ_LOW
;
;DESC	Checks if current reading is min/max and stores result.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
MINMAXTEST:
    MOVLB   0
    CALL    MAXTEST
    CALL    MINTEST
    RETURN
;=========================================================================
;Tests if new value above previous stored using subwf and status bits
;Stores both exact value and LED state of max
;=========================================================================
MAXTEST:
    MOVF    HIGHREADING,0
    SUBWF   MAX_READ_HIGH,0
    BTFSS   CARRY	    ;Will be 0 if Highreading > Max
    GOTO    SETMAX
    BTFSC   ZERO	    ;If result was exactly 0, = 1
    GOTO    TESTLOWERMAX
    RETURN
;=========================================================================
;This called if higher byte the same. Do the same for lower byte
;Set max if higher
;=========================================================================
TESTLOWERMAX:
    MOVF    LOWREADING,W
    SUBWF   MAX_READ_LOW,W
    BTFSS   CARRY
    GOTO    SETMAX
    RETURN
SETMAX:
    MOVF    HIGHREADING,W	;Set max for both higher and lower
    MOVWF   MAX_READ_HIGH
    MOVF    LOWREADING,W
    MOVWF   MAX_READ_LOW
    CALL    SET_MAX_LED
    RETURN

;=========================================================================
;Same for min. Negatives converted from 2's complement then compared for largest
;Which will be the lower negative.
;=========================================================================
MINTEST:
    BTFSC   MIN_READ_HIGH,7 ;Indicates negative
    GOTO    NEGSUB
    BTFSC   HIGHREADING,7   ;New is negative
    GOTO    NEGSUB
    MOVF    MIN_READ_HIGH,0
    SUBWF   HIGHREADING,0
    BTFSC   ZERO	    ;If result is 0, look at lower
    GOTO    TESTLOWERMIN
    BTFSS   CARRY	    ;Will be 1 if min <= highreading
    GOTO    SETMIN	    ;Already determined non 0 result, so min < new
    RETURN
TESTLOWERMIN:
    MOVF    MIN_READ_LOW,W
    SUBWF   LOWREADING,W
    BTFSS   CARRY
    GOTO    SETMIN
    RETURN
;=========================================================================
;For if new value is a negative. If previous isn't, set
;Else convert 2's complement and calculate.
;=========================================================================
NEGSUB:
    BTFSS   HIGHREADING,7	;If new reading not neg, is higher
    RETURN
    MOVF    HIGHREADING,W
    XORWF   MIN_READ_HIGH,W
    MOVWF   NEG_SUB_TEMP
    BTFSC   NEG_SUB_TEMP,7	;If neither neg, return
    GOTO    SETMIN
;=========================================================================
;Convert both and store new value in temp registers.
;=========================================================================
TWOSCOMPLEMENT:
    COMF    HIGHREADING,W	;2's complement, comf and incf.
    MOVWF   NEG_SUB_TEMP
    INCF    NEG_SUB_TEMP,F
    COMF    MIN_READ_HIGH,W
    MOVWF   NEG_SUB_TEMP2
    INCF    NEG_SUB_TEMP2,F
    MOVF    NEG_SUB_TEMP,0
    SUBWF   NEG_SUB_TEMP2,0	;Find which is max positive for lower negative
    BTFSS   CARRY
    GOTO    SETMIN
    BTFSC   ZERO
    GOTO    TESTLOWERNEG
    RETURN
;=========================================================================
;If top bytes the same, test lower. Same as higher byte.
;=========================================================================
TESTLOWERNEG:
    COMF    LOWREADING,W
    MOVWF   NEG_SUB_TEMP
    INCF    NEG_SUB_TEMP,F
    COMF    MIN_READ_LOW,W
    MOVWF   NEG_SUB_TEMP2
    INCF    NEG_SUB_TEMP2,F
    MOVF    NEG_SUB_TEMP,W
    SUBWF   NEG_SUB_TEMP2,W
    BTFSS   CARRY
    GOTO    SETMIN
    RETURN
   
;=========================================================================
;Store current in min.
;========================================================================= 
SETMIN:
    MOVF    HIGHREADING,W
    MOVWF   MIN_READ_HIGH
    MOVF    LOWREADING,W
    MOVWF   MIN_READ_LOW
    CALL    SET_MIN_LED
    RETURN


;=========================================================================
;For setting min max LED states. Avoid possible bugs from negative
;part. Hard to test due to needing -ve temps.
;=========================================================================

SET_MIN_LED:
    MOVF    LED_REGISTER,W
    MOVWF   MIN_LED
    RETURN

SET_MAX_LED:
    MOVF    LED_REGISTER,W
    MOVWF   MAX_LED
    RETURN
;=========================================================================
;BLOCK	TEST
;PARAM	NONE
;RETURN	NONE
;WRITES	PORTC, MIN_LED, MAX_LED
;
;DESC	Test loops for debugging.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================
;=========================================================================
;Set min and max and check they are stored properly /not overwritten.
;=========================================================================
MINMAXTESTROUTINE:
    MOVLB   0
    MOVLW   0x04H
    MOVWF   MIN_LED
    MOVLW   0x20H
    MOVWF   MAX_LED
    RETURN
;=========================================================================
;Call to check code reaches certain point.
;Set all LEDS on and stay in infinite loop.
;=========================================================================
TEST:
    MOVLB   0
    MOVLW   0xffh
    BANKSEL PORTC
    MOVWF   PORTC
    GOTO    TEST
;=========================================================================
;ISR	ISR
;PARAM	NONE
;RETURN	NONE
;WRITES	T0IF, TIMER0ISR, IOCBF, TMR1IF, INTERRUPT_FLAGS(0,1)

;DESC	Interupt Service Routine, checks source and sets flags.
;
;CONFIG	NONE
;CALLS	NONE
;=========================================================================

ISR:

    BANKSEL INTCON	;Find source(s) of interrupt
    BTFSC   T0IF
    CALL    TIMER0ISR
    BANKSEL IOCBF
    BTFSC   IOCBF6
    CALL    BUTTONISR
    BANKSEL PIR1   
    BTFSC   TMR1IF
    CALL    TIMER1ISR
    RETFIE
;=========================================================================
;Trigger every 0.5 second.
;=========================================================================
TIMER0ISR:
    BANKSEL INTCON
    BCF	    T0IF	;clear  interrupt
    MOVLB   0
    BSF	    INTERRUPT_FLAGS,0
    RETURN
;=========================================================================
;Trigger on either up or down. Reset 2sec timer.
;=========================================================================
BUTTONISR:
    BANKSEL IOCBF	;Clear interrupts
    BCF	    IOCBF6
    BANKSEL INTCON
    BCF	    IOCIF
    BANKSEL TMR1L	;Reset timer
    MOVLW   0X01H
    MOVWF   TMR1L
    MOVLW   0X00H
    MOVLW   TMR1H
    BANKSEL PIR1
    BCF	    TMR1IF	;Reset any previous tmr1 interrupt
    MOVLB   0
    BCF	    INTERRUPT_FLAGS,1	;Clear stable buttone state
    RETURN
;=========================================================================
;2 sec of stable button, check state and set flag.
;=========================================================================
TIMER1ISR:
    BANKSEL PIR1
    BCF	    TMR1IF
    BANKSEL PORTB
    BTFSC   RB6
    MOVLB   0
    BANKSEL PORTB
    MOVLB   0
    BSF	    INTERRUPT_FLAGS,1	;Set stable button
    RETURN
End