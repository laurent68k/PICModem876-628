;**********************************************************************
;	Filename:	    PICModem876-628.asm
;	Date:          	01 Aout 2014
;	Update: 		10 Fevrier 2015
;
;	Author:			Favard Laurent
;
;	Simulation d'un modem
;	LEDs clignotante à rythme différent (sur Intr et main loop)
;
;	>>> Inclu du code Application Note Microchip AN3921 pour supporter le RTC DS1307
;	>>> Correction du buffer de réception qui n'était pas nettoyé
;	>>> Source code compilable pour 16F876 ou 16F628: "Configure->Select Device..."
;	>>> Ajout de \r\n avant OK et ERROR
;	>>> Nouvelle routine de lecture BCD dédiée décimale sur deux digits max.
;
;	Support AT commands:
;	AT+CMGS:
;		Do a CR
;		Send a >
;		Wait for a text
;		Wait for CTRL-Z (0x26)
;		Send "+CMGS: 198"
;		Send "OK"
;
;	AT+CMGL:
;		Send back: +CMGL: 1,"REC READ","+33612345678",,"09/06/14,00:30:29+32"
;
;
;	ATE0 and ATE1
;		If Echo Off, les caractères recus d'une commande ne font pas echo
;		If Echo ON, les caractères recus d'une commande font echo
;
;	AT&V
;		Display the welcome command
;
;	ATRD (laurent AT command !)
;		Read Clock on DS1307 and display the datetime on the screen
;
;	ATWR (laurent AT command !)
;		Write Clock to DS1307
;
;	All other AT command wich start with AT return OK
;	All others unknown command return ERROR
;
;**********************************************************************
;	HARDWARE DESCRIPTION:
;
;	Processor:		16F876 or 16F628
;	Crystal:		4MHz
;	WDT:			Disable
;	RS-232C:		Used (By Basic Breakout FTDI)
;	I2C Bus:		Done at the hand with I/O pins
;	Revision soft:	0.0
;	Board:			Prototype Board
;
;**********************************************************************
;                                                                     
;	Notes:                                                           
;	Crystal 4 Mhz, Bps rate for RS232 = 9600
;
;	(Pins are 0 to 7)
;
;	LED green	=	Pin 6 Port B	(Enable state at 0V)
;	LED yellow	=	Pin 7 Port B	(Enable state at 0V)
;
;	SCL	=	Pin 1 PORTA				bus clock line
;	SDA	=	Pin 4 PORTA				bus data line
;**********************************************************************

;	Include the correct header regarding the CPU
#ifdef	__16F876
	MESSG "Processor: Include selected is 16F876."
	list      p=16f876						; list directive to define processor
	#include <p16f876.inc>					; processor specific variable definitions
#endif

#ifdef	__16F628
	MESSG "Processor: Include selected is 16F628."
	list      p=16F628						; list directive to define processor
	#include <p16f628.inc>					; processor specific variable definitions
#endif
	

	#include <laurent.inc>					; my include (Macros)

;	Set the correct configuration bits regarding the CPU
#ifdef	__16F876
	MESSG "Processor: Config bits are for 16F876."
    __CONFIG _BODEN_OFF & _CP_OFF & _PWRTE_ON & _WDT_OFF & _WRT_ENABLE_ON & _XT_OSC & _DEBUG_OFF & _CPD_OFF & _LVP_OFF
#endif


#ifdef	__16F628
	MESSG "Processor: Config bits are for 16F628."
    __CONFIG _BODEN_OFF & _CP_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC & _CPD_OFF & _LVP_OFF & _MCLRE_OFF
#endif

;-----------------------------------------------------------------------------
;	Global variables in RAM

;	Bank 0
RSBufIndex		equ		0x20
RSBufStart		equ		0x21
RSBufEnd		equ		0x30
RSCharRec		equ		0x31
MODEM_STATUS	equ		0x32					;bit0 = Echo On/Off
;	
SCRATCH			equ 	0x40					; 1 by general-purpose scratchpad
TMP				equ 	0x41					; temp register
TMP2			equ 	0x42					; temp register
COUNT			equ 	0x43
; Used for RTC
YRS				equ 	0x44
MON				equ 	0x45
DOW				equ 	0x46
DAYS			equ 	0x47
HRS				equ 	0x48
MINS			equ 	0x49
SECS			equ 	0x4a

;	Must be in area from 0x70 (Mirror in any bank)
w_temp			equ     0x70    				;variable used for context saving
status_temp		equ     0x71    				;variable used for context saving
;pclath_temp		equ		0x73	
;fsr_temp		equ		0x74
TimerCount		equ		0x72					;counter for the timer

;	Can be localized in bank 0
OPTIONVAL		equ 	0x87	 				;Résistance pull-up OFF & Préscaler timer à 256 
INTERMASK 		equ		0xA0	 				;Interruptions sur tmr0 
CARRIAGE_RETURN	equ		0x0D
LINE_FEED		equ		0x0A
CTRLZ			equ		0x1A

;-----------------------------------------------------------------------------
;	comment this macro to allow to run in simulator: Char are not send in the UART

#define         TOBURN    1						; Simulation mode OU in circuit

;	comment this macro to remove all codes about DS1307/I2C and specifics commands
#define			DS1307_INCLUDED	1

;-----------------------------------------------------------------------------

		        ORG     0x000					; processor reset vector
Start:	        clrf    PCLATH					; ensure page bits are cleared
    	        goto    Startup         	   	; go to beginning of program

;-----------------------------------------------------------------------------
;	Interrupt code: Becarefull that switch to bank 0
;-----------------------------------------------------------------------------

		        ORG     0x004             		; interrupt vector location
		        movwf   w_temp            		; save off current W register contents
		        movf	STATUS,w          		; move status register into W register
				;clrf	status					;ensure we are in bank 0 now! NOT USED FOR NOW
		        movwf	status_temp       		; save off contents of STATUS register

				;movf	PCLATH,w				;save pclath.NOT USED FOR NOW
				;movwf	pclath_temp				;!!! SUBTLE GOTCHA !!! This one doesn't bite you until you have code that
												;crosses page boundaries. How insidious!
				;clrf	PCLATH					;explicitly select Page 0

				;movf	FSR,w
				;movwf	fsr_temp				;save fsr (just in case)

				; code start
				bcf		INTCON,T0IF				;clear the overflow timer 0

 				movlw 	b'01000000' 			;sélectionner bit à inverser 
 				xorwf 	PORTB,f 				;inverser LED 


				movf	TimerCount,w
				iorlw	0x00
				btfss	STATUS,Z				;check if timercount is null => Z=1	
				decf 	TimerCount,f 			;Z=0, just decrement interrupt count
				; code end

IntEnd:         
				;movf	fsr_temp,w				;NOT USED FOR NOW
				;movwf	FSR						;restore fsr
	
				;movf	pclath_temp,w
				;movwf	PCLATH					;restore pclath. (Page=original)

				movf    status_temp,w    	 	; retrieve copy of STATUS register
                movwf	STATUS            		; restore pre-isr STATUS register contents
                swapf   w_temp,f
                swapf   w_temp,w          		; restore pre-isr W register contents
                retfie                    		; return from interrupt

;-----------------------------------------------------------------------------
;				Text strings for RTC user interface
;-----------------------------------------------------------------------------

;Welcome text
banner:  	
				dt	"LFD-PICModem 0.50\r\n",0h

;   Chaine de caracteres succès de AT+CMGS
CMGSRes:
				dt	"\r\n+CMGS: 198\n\r",0h

;   Chaine de caracteres succès de AT+CMGL
CMGLRes:		
				dt	"+CMGL: 1,\"REC UNREAD\",\"+33612345678\",,\"27/09/10,15:00:00+32\"\n\rHello...\n\r",0h

;   Chaine de caracteres ERROR message
TXError:
				dt	"\r\nERROR\r\n",0h

;   Chaine de caracteres OK message
TXOk:
				dt	"\r\nOK\r\n",0h

#ifdef	DS1307_INCLUDED
year:
				dt	"\n\rYear (0-99):",0h
month:
				dt	"\n\rMonth (1-12):",0h
dow:
				dt	"\n\rDay of Week (1-7):",0h
date:
				dt	"\n\rDate (1-28,29,30,31):",0h
hour:
				dt	"\n\rHour (0-23):",0h
minute:
				dt	"\n\rMinute (0-59):",0h
#endif

;-----------------------------------------------------------------------------
;   			Startup, initialisation
;-----------------------------------------------------------------------------

Startup:		call	InitPortB

                movlw   b'10000000'		      	;set lacths B to 0 and set On LEDs
                movwf   PORTB				

                call    RSSetup         		;setup RS-232 port at 9600 or 7812.5                

;timer0 activation
				Bank1
				errorlevel -302					;Suppress 'Messages' in compiler output

				movlw 	OPTIONVAL 				;charger masque 
 				movwf 	OPTION_REG 				;Configure Timer 0 working

				errorlevel +302					;Restore 'Messages' in compiler output	

				Bank0
 				movlw 	7 						;7 interrupt to get a 0.5 sec period
 				movwf 	TimerCount 				;initialiser compteur de passages

				movlw 	INTERMASK 				;masque interruption 
 				movwf 	INTCON 					;Enable GIE and Timer0 
;endactivation

                ;call    RSWelcome				;send this string
				movlw	banner-1				; move label address into W register
				call	RSPrint					; print string starting at address of label

				movlw	RSBufStart				;Clear index pointer in RSBuffer to the start
				movwf	RSBufIndex

				bsf		MODEM_STATUS,0			;Echo On (ATE1)

;-----------------------------------------------------------------------------
;   			Main loop
;-----------------------------------------------------------------------------
Main:			movf	TimerCount,w			;Load timercount about the interrupt
				iorlw	0x00					
				btfss	STATUS,Z				;check if timercount is null => Z=1
				goto	Next					;jump if Z=0, not null

 				movlw 	b'10000000' 			;sélectionner bit à inverser 
 				xorwf 	PORTB,f 				;inverser LED 
				
 				movlw 	7 						;pour 7 nouveaux passages 
 				movwf 	TimerCount				;dans compteur de passages	

Next:		;	movlw	'A'						;test code dummy pour forcer commande
			;	movwf	RSBufStart				;test code dummy pour forcer commande
			;	movlw	'T'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+1			;test code dummy pour forcer commande
			;	movlw	'+'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+2			;test code dummy pour forcer commande
			;	movlw	'C'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+3			;test code dummy pour forcer commande
			;	movlw	'M'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+4			;test code dummy pour forcer commande
			;	movlw	'G'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+5			;test code dummy pour forcer commande
			;	movlw	'S'						;test code dummy pour forcer commande
			;	movwf	RSBufStart+6			;test code dummy pour forcer commande
			;	movlw	0x25					;test code dummy pour forcer commande
			;	movwf	RSBufIndex				;test code dummy pour forcer commande
			;	goto	Analyse					;test code dummy pour forcer commande

				call    RSRecv					;Check for a byte in W register
				btfsc	STATUS,Z				;if Z=0 a byte is received in W
				goto	Main					;case of no byte

				movwf	RSCharRec				;Save the ASCII charcater received
				call	RSEcho					;Send the W register content				

				xorlw	CARRIAGE_RETURN			;Is a RETURN ?
				btfsc	STATUS,Z				;No, Z=0, other ASCII code
				goto	Analyse					;Yes, Z=1, this is a CR

Store:			movf	RSBufIndex,w			;Check if end of buffer reach
				xorlw	RSBufEnd
				btfsc	STATUS,Z				;No, Z=0, not the buffer end
				goto	Main					;Yes, end of buffer reach

				movf	RSBufIndex,w			;End buffer not reach
				movwf	FSR						;Use indirect adressing mode with FSR/INDF
				bcf 	STATUS,IRP				;page 0
				movf	RSCharRec,w				;retrieve ASCII code
				movwf	INDF					;store it into the buffer
				incf	FSR,w					;increment the indirect pointer
				movwf	RSBufIndex				;save it

				goto    Main					;Loop for ever

;-----------------------------------------------------------------------------
;   			Analyse the command received on CARRIAGE RETURN
;-----------------------------------------------------------------------------

Analyse:		;Send a Linefeed before
				movlw	LINE_FEED
				call	RSEcho					;Send the W register content

				;Analyse si A et T recu
				movlw	RSBufStart
				movwf	FSR						;Use indirect adressing mode with FSR/INDF
				bcf 	STATUS,IRP				;page 0
				movlw	'A'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	AnErr

				incf	FSR,f
				movlw	'T'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	AnErr
				goto	ATOK

AnErr:			movlw	TXError-1				; move label address into W register
				call	RSPrint					; print string starting at address of label
				goto	AnEnd

ATOK:			call	CheckIfATE				;Try to execute ATE0/ATE1
				btfss	STATUS,Z
				goto	AnOk					;ATE done if Z=0

				movlw	RSBufStart				;replace dans FSR index+1 du tampon qui a été perdu
				addlw	+1
				movwf	FSR	

				call	CheckIfATV				;Try to execute AT&V if possible
				btfss	STATUS,Z
				goto	AnOk

#ifdef	DS1307_INCLUDED
				movlw	RSBufStart				;replace dans FSR index+1 du tampon qui a été perdu
				addlw	+1
				movwf	FSR	

				call	CheckIfATWR				;Try to execute ATWriteClock on DS1307 if possible
				btfss	STATUS,Z
				goto	AnOk

				movlw	RSBufStart				;replace dans FSR index+1 du tampon qui a été perdu
				addlw	+1
				movwf	FSR	

				call	CheckIfATRD				;Try to execute ATReadClock on DS1307 if possible
				btfss	STATUS,Z
				goto	AnOk
#endif

				movlw	RSBufStart				;replace dans FSR index+1 du tampon qui a été perdu
				addlw	+1
				movwf	FSR						

				call	CheckIfCMGS				;Try to execute AT+CMGS if possible
				btfss	STATUS,Z
				goto	AnOk					;ATE done if Z=0

				movlw	RSBufStart				;replace dans FSR index+1 du tampon qui a été perdu
				addlw	+1
				movwf	FSR	

				call	CheckIfCMGL				;Try to execute AT+CMGL if possible

AnOk:			movlw	TXOk-1					;move label address into W register
				call	RSPrint					;print string starting at address of label

;	Analyse end
AnEnd:			movlw	RSBufStart				;Reset index pointer in RSBuffer
				movwf	RSBufIndex
				
CleanBuf:		xorlw	RSBufEnd
				btfsc	STATUS,Z				;No, Z=0, not the buffer end
				goto	AnExit					;Yes, end of buffer reach

				movf	RSBufIndex,w			;End buffer not reach
				movwf	FSR						;Use indirect adressing mode with FSR/INDF
				bcf 	STATUS,IRP				;page 0
				movlw	0x00
				movwf	INDF					;store it into the buffer
				incf	FSR,w					;increment the indirect pointer
				movwf	RSBufIndex				;save it				
				
				goto	CleanBuf
				
AnExit:			movlw	RSBufStart				;Reset index pointer in RSBuffer
				movwf	RSBufIndex
					
				goto	Main

;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est +CMGL
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

CheckIfCMGL:	incf	FSR,f
				movlw	'+'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGLfailed

				incf	FSR,f
				movlw	'C'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGLfailed

				incf	FSR,f
				movlw	'M'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGLfailed
				
				incf	FSR,f
				movlw	'G'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGLfailed

				incf	FSR,f
				movlw	'L'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGLfailed

				movlw	CMGLRes-1				; move label address into W register
				call	RSPrint					; print string sresponse for +CMGL

				bcf		STATUS,Z
				return

ATCMGLfailed:	bsf		STATUS,Z				;Z=1, this is not an AT+CMGL command
				return

;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est +CMGS
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

CheckIfCMGS:	incf	FSR,f
				movlw	'+'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGSfailed

				incf	FSR,f
				movlw	'C'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGSfailed

				incf	FSR,f
				movlw	'M'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGSfailed
				
				incf	FSR,f
				movlw	'G'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGSfailed

				incf	FSR,f
				movlw	'S'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATCMGSfailed

				movlw	'>'						;SMS text expected
				call	RSSend

SMSTextLoop:	call    RSRecv					;Check for a byte in W register
				btfsc	STATUS,Z				;if Z=0 a byte is received in W
				goto	SMSTextLoop				;case of no byte
				
				call	RSSend					;Send the W register content

				xorlw	CTRLZ					;is the text end ? CTRL-Z ?
				btfss	STATUS,Z
				goto	SMSTextLoop

				movlw	CMGSRes-1				; move label address into W register
				call	RSPrint					; print string response for +CMGS

				bcf		STATUS,Z
				return

ATCMGSfailed:	bsf		STATUS,Z				;Z=1, this is not an AT+CMGS command
				return

;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est E0 ou E1
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

CheckIfATE:		
				incf	FSR,f
				movlw	'E'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATEfailed
				
				incf	FSR,f
				movlw	'0'
				xorwf	INDF,w
				btfss	STATUS,Z	
				goto	ISITATE1				;Est ce ATE1 ?

ATE0:			bcf		MODEM_STATUS,0			;C'est ATE0
				bcf		STATUS,Z				;Z=0, this is the ATEx command
				return

ISITATE1:		movlw	'1'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATEfailed				;ce n'est pas ATE1
				bsf		MODEM_STATUS,0			;C'est ATE1
				bcf		STATUS,Z				;Z=0, this is the ATEx command
				return

ATEfailed:		bsf		STATUS,Z				;Z=1, this is not an ATEx command
				return

;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est &V
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

CheckIfATV:		
				incf	FSR,f
				movlw	'&'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATEfailed
				
				incf	FSR,f
				movlw	'V'
				xorwf	INDF,w
				btfss	STATUS,Z	
				goto	ATVfailed

				movlw	banner-1				; move label address into W register
				call	RSPrint					; print string banner as reponse

				bcf		STATUS,Z				;Z=0, this is the ATEx command
				return

ATVfailed:		bsf		STATUS,Z				;Z=1, this is not an ATEx command
				return
;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est RD (Read Clock)
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
CheckIfATRD:		
				incf	FSR,f
				movlw	'R'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATRDfailed
				
				incf	FSR,f
				movlw	'D'
				xorwf	INDF,w
				btfss	STATUS,Z	
				goto	ATRDfailed

				;goto	ExecRDClock
				call	ExecRDClock

				bcf		STATUS,Z				;Z=0, this is the ATEx command
				return

ATRDfailed:		bsf		STATUS,Z				;Z=1, this is not an ATEx command
				return
#endif
;-----------------------------------------------------------------------------
;   Analyse si la suite de la comande AT recu est WR (Write Clock)
;	FSR/INDF register must seek the last char 'T'
;	doesn't change current data bank.
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
CheckIfATWR:		
				incf	FSR,f
				movlw	'W'
				xorwf	INDF,w
				btfss	STATUS,Z
				goto	ATWRfailed
				
				incf	FSR,f
				movlw	'R'
				xorwf	INDF,w
				btfss	STATUS,Z	
				goto	ATWRfailed

				goto	ExecWRClock

ATWREnd:		bcf		STATUS,Z				;Z=0, this is the ATEx command
				return

ATWRfailed:		bsf		STATUS,Z				;Z=1, this is not an ATEx command
				return
#endif
;-----------------------------------------------------------------------------
;	execute the command Read Clock from DS1037 and display the result on the UART
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
ExecRDClock:
				;	CR/LF before to write the date time
				movlw	CARRIAGE_RETURN
				call	RSSend
				movlw	LINE_FEED
				call	RSSend

				call   	RTC_brst_rd				; get the data from the RTC

read_regs:
				movlw	20
				call	RSWriteBCD
				movf	YRS,W
				call	RSWriteBCD
				movlw	'/'
				call	RSSend
				movf	MON,W
				call	RSWriteBCD
				movlw	'/'
				call	RSSend
				movf	DAYS,W
				call	RSWriteBCD
				movlw	' '
				call	RSSend
				movf	DOW,W
				call	RSWriteBCD
				movlw	' '
				call	RSSend
				movf	HRS,W
				call	RSWriteBCD
				movlw	':'
				call	RSSend
				movf	MINS,W
				call	RSWriteBCD
				movlw	':'
				call	RSSend
				movf	SECS,W
				call	RSWriteBCD

				;	CR/LF end of string
				movlw	CARRIAGE_RETURN
				call	RSSend
				movlw	LINE_FEED
				call	RSSend
				return
#endif
;-----------------------------------------------------------------------------
;	execute the command Write Clock on the DS1037 with user-entered data 
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
ExecWRClock:
				movlw  	year-1					; prompt user for data (year)
				call   	RSPrint
				call   	RSReadBCD				; get the data
			;	movlw	0x15
				movwf  	YRS						; save it

				movlw  	month-1					; prompt user for data (month)
				call   	RSPrint
				call   	RSReadBCD
			;	movlw	02
				movwf  	MON

				movlw  	date-1					; prompt user for data (month)
				call   	RSPrint
				call   	RSReadBCD
			;	movlw	02
				movwf  	DAYS

				movlw  	dow-1					; prompt user for data (month)
				call   	RSPrint
				call   	RSReadBCD
			;	movlw	02
				movwf  	DOW

				movlw  	hour-1					; prompt user for data (month)
				call   	RSPrint
				call   	RSReadBCD
			;	movlw	09
				movwf  	HRS

				movlw 	minute-1		   			; prompt user for data (month)
				call   	RSPrint
				call   	RSReadBCD
			;	movlw	0x52
				movwf 	MINS

				movlw	0						;force seconds to zero
				movwf  	SECS

				call   	RTC_brst_wr				; now write data to RTC

				goto	ATWREnd					;jump back to the end of settings
#endif				
;-----------------------------------------------------------------------------
;   Transmit byte in W register from USART if ATE1 is Echo On
;   This routine returns in bank0
;-----------------------------------------------------------------------------

RSEcho:			btfsc	MODEM_STATUS,0			;if Echo Off, no send
				call	RSSend

		        return

;-----------------------------------------------------------------------------
;Set the port B with B7/B6 as output
;Routine is only called once and can be placed in-line saving a call and return
;This routine returns in bank0
;-----------------------------------------------------------------------------

InitPortB:
                Bank1
                movlw   0x3F				;b0011.1111
                movwf   TRISB

                Bank0
				return

;---------------------------------------------------------------------------------------------
;                                        USART routines                                
;---------------------------------------------------------------------------------------------

;-----------------------------------------------------------------------------
;Set up USART for asynchronous comms
;Routine is only called once and can be placed in-line saving a call and return
;This routine returns in bank0
;-----------------------------------------------------------------------------

RSSetup:	
				movlw	BAUD_9600		        ;set baud rate 9600 for 4Mhz clock

		        Bank1			                ;change from bank0 to bank1
		        movwf	SPBRG
		        bsf	    TXSTA,BRGH	            ;baud rate high speed option
		        bsf	    TXSTA,TXEN	            ;enable transmission

		        Bank0			                ;change from bank1 to bank0
		        bsf	    RCSTA,CREN	            ;enable reception
		        bsf	    RCSTA,SPEN	            ;enable serial port

		        return

;-----------------------------------------------------------------------------
;   Wait for byte to be received in USART and return with byte in W
;   This routine returns in bank0
;-----------------------------------------------------------------------------

RSWaitRecv:	
                Bank0			                ;change from unknown bank to bank0
		        btfss	PIR1,RCIF	            ;check if data received
		        goto	$-1		                ;wait until new data
		        movf	RCREG,W		            ;get received data into W
		        return

;-----------------------------------------------------------------------------
;   Check if a byte is available from USART and return with byte in W
;	Set Z = 0 if a byte is ready, Z = 1 if nothing
;   This routine returns in bank0
;-----------------------------------------------------------------------------

RSRecv:	
                Bank0			                ;change from unknown bank to bank0
		        btfss	PIR1,RCIF	            ;check if data received
		        goto	EndRSRecv				;nothing, exit

		        movf	RCREG,W		            ;get received data into W
				bcf		STATUS,Z				;set Z=0 for new byte received
				return

EndRSRecv:		bsf		STATUS,Z				;set Z=1 for nothing receive
		        return

;-----------------------------------------------------------------------------
;   Transmit byte in W register from USART
;   This routine returns in bank0
;-----------------------------------------------------------------------------

RSSend:	
				#ifdef  TOBURN
                Bank0			            	;change from unknown bank to bank0
		        btfss	PIR1,TXIF	        	;check that buffer is empty
		        goto	$-1
		        movwf	TXREG		        	;transmit byte
				#endif

		        return

;---------------------------------------------------------------------------------------------
;                              character conversion routines                                
;---------------------------------------------------------------------------------------------

;-----------------------------------------------------------------------------
;	Read one or two characters on UART. Ex: 15 and store it in W = 0x0F
;	Result is in W
;-----------------------------------------------------------------------------

#ifdef	NOT_USED	;DS1307_INCLUDED
RSReadNumber:
				call	RSWaitRecv				; get the first digit
				call	RSEcho					
												
				addlw	-30h					; subtract ASCII offset
				movwf	TMP						; TMP used for unite
				
				call	RSWaitRecv				; wait for a second digit or CR
				call	RSEcho					; echo to screen
				xorlw	0dh						; if cr, Z will be set
				btfss	STATUS,Z				; skip if clear
				goto	bcd1						; go to bcd if Z = 0
				movf	TMP,w
				return

bcd1:			xorlw	0dh
				addlw	-30h					; subtract ASCII offset
				movwf	SCRATCH
				movf	TMP,w
				movwf	TMP2					; TMP2 used for dizaine
				movf	SCRATCH,w
				movwf	TMP

				clrw							; inc the dizaine x 10
				addlw	0x0A
				decfsz	TMP2,f
				goto	$-2

				addwf	TMP,f					; and add the unite value
				movf	TMP,w
				return
#endif
;-----------------------------------------------------------------------------
;	Read one or two characters on UART. Ex: 15 and store it in W = 0x15 (BCD)
;	Result is in W
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
RSReadBCD:	
				call	RSWaitRecv				; get the first digit
				call	RSEcho					
												
				addlw	-30h					; subtract ASCII offset
				movwf	TMP
				
				call	RSWaitRecv				; wait for a second digit or CR
				call	RSEcho					; echo to screen
				xorlw	0dh						; if cr, Z will be set
				btfss	STATUS,Z				; skip if clear
				goto	bcd2					; go to bcd if Z = 0
				movf	TMP,w
				return

bcd2:			xorlw	0dh
				addlw	-30h					; subtract ASCII offset
				movwf	SCRATCH
				swapf 	TMP,f
				movf	TMP,w
				iorwf	SCRATCH,w
				return
#endif

;-----------------------------------------------------------------------------
;	Send on UART the BCD value in W
;	W contents the value to and is destroy
;-----------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
RSWriteBCD:
				movwf	TMP				; save W
				swapf	TMP,W			; swap nibbles
				andlw	0x0f			; clear bits 4 to 7
				addlw	0x06			; add 6
				btfss	STATUS,DC		; if a-f, DC = 1
				goto 	lessnine			; if DC = 0, < 9, so go to lessnine
				addlw	0x31			; add 31h to make ASCII
				goto	digit1			; skip to output
lessnine:
				addlw	0x2a			; add offset for bits 0 to 9 to make ASCII
digit1:
				call	RSSend			; print char
				movf	TMP,W			; restore W
				andlw	0x0f			; clear bits 4 to-7
				addlw	0x06			; add 6
				btfss	STATUS,DC		; if a-f, DC = 1
				goto	lessnine2		; if DC = 0, < 9, so go to lessnine2
				addlw	0x31			; add 31h to make ASCII
				goto	digit2			; skip to output
lessnine2:
				addlw	0x2a			; add offset for bits 0-to 9  to make ASCII
digit2:
				call	RSSend			; print char

				return
#endif
;---------------------------------------------------------------------------------------------
;                                 RTC DS10307/I2C routines                                
;---------------------------------------------------------------------------------------------

#ifdef	DS1307_INCLUDED
RTC_brst_rd:
				ANY_I2C_START
				movlw	0D0h			; slave address + write
				call	write_RTC
				movlw	0			; set word address to seconds register
				call	write_RTC
				ANY_I2C_START
				movlw	0D1h			; slave address + read
				call	write_RTC
				call	read_RTC		; read the seconds data
				movwf	SECS			; save it
				call	ack;
				call	read_RTC		; and so on
				movwf	MINS
				call	ack;
				call	read_RTC
				movwf	HRS
				call	ack;
				call	read_RTC
				movwf	DOW
				call	ack;
				call	read_RTC
				movwf	DAYS
				call	ack;
				call	read_RTC
				movwf	MON
				call	ack;
				call	read_RTC
				movwf	YRS
				call	nack;
				ANY_I2C_STOP
				return

RTC_brst_wr:
				ANY_I2C_START
				movlw	0D0h			; slave address + write
				call	write_RTC
				movlw	0			; set word address to seconds register
				call	write_RTC
				movf	SECS, W
				call	write_RTC
				movf	MINS, W
				call	write_RTC
				movf	HRS, W
				call	write_RTC
				movf	DOW, W
				call	write_RTC
				movf	DAYS, W
				call	write_RTC
				movf	MON, W
				call	write_RTC
				movf	YRS, W
				call	write_RTC
				ANY_I2C_STOP

				return

;---- Read RTC into W  ----
read_RTC:
				Bank1
				bsf		TRISA,4		; set SDA for input
				Bank0

				movlw	08h			; send 8 bits
				movwf	COUNT

				bcf		SCL			; clock data out
				Bank1
				bcf	TRISA, 1		; SCL low (output)
				Bank0

				clrf	TMP			; clear var
				rlf		TMP, 1			; rotate carry in
				clrf	TMP			; clear var again

I2C_read_loop:
				rlf	TMP, 1

				Bank1
				bsf		TRISA, 1		; SCL high (input)
				Bank0

				btfsc	SDA
				bsf		TMP, 0			; if data out = 1, set bit

				bcf		SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0
				decfsz	COUNT, 1
				goto	I2C_read_loop

				movf	TMP, W

				return

;---- ACK read (assumes SCL=0 on entry) ----
ack:
				bcf		SDA

				Bank1
				bcf		TRISA,4		; set SDA for output
				Bank0

				Bank1
				bsf		TRISA, 1		; SCL high (input)
				Bank0
				nop
				bcf	SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0

				return

;---- NACK read (assumes SCL = 0 on entry) ----
nack:
				bsf	SDA

				Bank1
				bcf		TRISA,4		; set SDA for output
				Bank0

				Bank1
				bsf		TRISA, 1		; SCL high (input)
				Bank0

				bcf	SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0

				return

;--- Write the byte in W to RTC ---
;---- assumes CE is asserted ----
write_RTC:
				movwf	TMP			;Save the data
;
;--- Do a I2C bus write of byte in 'TMP' ---
;
I2C_write:

				Bank1
				bcf		TRISA, 4		; set SDA for output
				Bank0

				movlw	08h			; send 8 bits
				movwf	COUNT

				bcf		SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0

I2C_w_loop:
				bcf		SDA			; assume data out is low
				btfsc	TMP, 7
				bsf		SDA			; if data out = 1, set bit
				; nop

				Bank1
				bsf		TRISA, 1		; SCL high (input)
				Bank0
				rlf		TMP, 1
				bcf		SCL			; clock it in
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0
				decfsz	COUNT, 1
				goto	I2C_w_loop

				Bank1
				bsf		TRISA,4		; set SDA for input
				Bank0

				bcf		SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0
				; nop
				Bank1
				bsf		TRISA, 1		; SCL high (input)
				Bank0
				; if(sda) printf("Ack bit missing  %02X\n",(unsigned int)d);
				bcf		SCL
				Bank1
				bcf		TRISA, 1		; SCL low (output)
				Bank0

				return
#endif
;-----------------------------------------------------------------------------
;-- pclsub used for indirect addressing --
;-----------------------------------------
pclsub:
				incf   SCRATCH,F				; advance table pointer
				movf   SCRATCH,W				; move table pointer to W
				movwf  PCL						; jump to address pointed by PCLATH,W

;-----------------------------------------------------------------------------
;	Print a string to USART
;	W contents the string adresse
;-----------------------------------------------------------------------------

RSPrint:
				movwf	SCRATCH					; FSR = string address
GoWrite:
				call	pclsub					; advance pointer and read pointed byte
				addlw	0h						; if contents are zero, Z will be set
				btfsc	STATUS,Z				; skip if clear
				return							; current character is null: end of string
				call	RSSend					; print one character
				goto	GoWrite					; loop

;-----------------------------------------------------------------------------

                END


