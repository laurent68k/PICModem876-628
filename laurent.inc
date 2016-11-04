;**********************************************************************
;	Filename:	    Laurent.inc
;	Date:          	18 Juillet 2001
;	File Version:  	11 September 2014
;
;	Author:			Favard Laurent
;
;**********************************************************************

BAUD_9600			EQU		0x19  				;Constant for baud generator 9600 baud at 4 Mhz
BAUD_7812_5    		Equ     0x1F                ;Constant for baud generator 7812.5 bauds at 4 Mhz

;-----------------------------------------------------------------------------
; Pins assignements

RB0                 EQU 	0
RB1                 EQU 	1
RB2                 EQU 	2
RB3                 EQU 	3
RB4                 EQU 	4
RB5                 EQU 	5
RB6                 EQU 	6
RB7                 EQU 	7

;-----------------------------------------------------------------------------

JUMPER4			    EQU		RB4					;Choix mode PS/2 (0V) directe OU tranlation ATARI (+5V)
JUMPER5			    EQU		RB5					

LEDGREEN		    EQU		RB6
LEDYELLOW		    EQU		RB7

;-----------------------------------------------------------------------------

#define	SCL	PORTA,1		; bus clock line
#define	SDA	PORTA,4		; bus data line

;-----------------------------------------------------------------------------
;	define specifics I2C Start and Stop for 16F628 and 16F876

ANY_I2C_START MACRO
		bsf	SDA						; SDA high
		nop
		Bank1		
		bsf	TRISA, 1				; SCL high (input)		
		Bank0
		bcf	SDA						; SDA low (start)
		ENDM

ANY_I2C_STOP MACRO						; assumes SCL is high on entry
		bcf	SDA						; SDA low
		nop
		nop
		Bank1
		bsf	TRISA, 1				; SCL high (input)
		Bank0
		bsf	SDA						; SDA high
		ENDM
                
;-----------------------------------------------------------------------------
;Macros to select the register bank
;Many bank changes can be optimised when only one STATUS bit changes

Bank0	MACRO						;macro to select data RAM bank 0
		bcf	STATUS,RP0
		bcf	STATUS,RP1
		errorlevel +302				;Restore 'Messages' in compiler output	
		ENDM

Bank1	MACRO						;macro to select data RAM bank 1
		bsf	STATUS,RP0
		bcf	STATUS,RP1
		errorlevel -302				;Suppress 'Messages' in compiler output
		ENDM

Bank2	MACRO						;macro to select data RAM bank 2
		bcf	STATUS,RP0
		bsf	STATUS,RP1
		errorlevel -302				;Suppress 'Messages' in compiler output
		ENDM

Bank3	MACRO						;macro to select data RAM bank 3
		bsf	STATUS,RP0
		bsf	STATUS,RP1
		errorlevel -302				;Suppress 'Messages' in compiler output
		ENDM

;-----------------------------------------------------------------------------
Delay   	macro   Time            ;Delay "Cycles" instruction cycles

    if (Time==1)
          nop
          exitm
    endif
    if (Time==2)
          goto $ + 1
          exitm
    endif
    if (Time==3)
          nop
          goto $ + 1
          exitm
    endif
    if (Time==4)
          goto $ + 1
          goto $ + 1
          exitm
    endif
    if (Time==5)
          goto $ + 1
          goto $ + 1
          nop
          exitm
    endif
    if (Time==6)
          goto $ + 1
          goto $ + 1
          goto $ + 1
          exitm
    endif
    if (Time==7)
          goto $ + 1
          goto $ + 1
          goto $ + 1
          nop
          exitm
    endif
    if (Time%4==0)
          movlw (Time-4)/4
          call Delay_Routine
          exitm
    endif
    if (Time%4==1)
          movlw (Time-5)/4
          call Delay_Routine
          nop
          exitm
    endif
    if (Time%4==2)
          movlw (Time-6)/4
          call Delay_Routine
          goto $ + 1
          exitm
    endif
    if (Time%4==3)
          movlw (Time-7)/4
          call Delay_Routine
          goto $ + 1
          nop
          exitm
    endif
    endm

;------------------------------------------------------------------------

