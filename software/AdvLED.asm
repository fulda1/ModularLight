;;======================================================================;;
;;			Lgo DCC ONE LIGHT DECODER				;;
;;======================================================================;;
;;									;;
;; Program:         mini_light -- DCC lights decoder			;;
;; Code:            Jindra Fu?ík http://www.fucik.name ; 
;; Platform:        Microchip PIC10F32x, 8 Mhz		;;
;; Date:            17.01.2014						;;
;; First release:   25.01.2014						;;
;; LastDate:        27.08.2021						;;
;;									;;
;;======================================================================;;
;
; This program is distributed as is but WITHOUT ANY WARRANTY
; License: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en
;
; Revisions:
; 05.05.2007	Start of writting code (PIC12F629)
; 17.01.2014	Modification for PIC10F32x
; 23.10.2019	Direct CV mode programming
; 27.08.2021	Change to Lgo version

; Working on same principle as DCC
; All DCC timings are based on 58us interrupts. When no data is available automatically an Idle
; packet is generated. The main program only has to fill the buffer when it is empty, if not
; an Idle packet is send to track, this maintains DC control (full version) and continuous DCC generation.
;

; Let us have protocol using same data encoding as DCC
; but data presence will be closer to "scalextric digital slot cars"
; let us have one master (command station) and 15 slaves with address 1 to 15 (1 to 0xF in hex)
; Slave can receive "intensity" command, when received, it will start lighting by requested intensity
; we must be able to program address to slave
; First byte contain command (4b) + address (4b)
; Ti can be followed by command data
; if more than one data bytes (up to 4 total) is present, next data mean same command, for address +1 (up to +3)
;
; Commands:
; 0x0 - nop (do nothing, no data)
; 0x1 - set address (program address)
; 0x2 - set light intensity to X
;
; Command details:
; 0x0x - this is nop command. used address make no sense, all slaves can ignore this command.
;
; 0x1A - this command enforce connected slave(s) to use address A. Command should be followed by one data byte 0x0A, where both A should be same.
;        slave present received address by blinking (number of blinks is current address)
;        in case A = 0, it mean no change of address, but blink current address (read address only)
;
; 0x2A - this command is followed by 1 to 4 data bytes. data bytes contain light intensity information for modules with address A to A+3.
;        when A=0, then first data byte is ignored by all slaves, but second data byte is regularly used by slave with address 1 etc.
;        data byte address roll over. It mean for example: when A = F, then first data byte is for address F, next data byte is ignored 
;        as for address 0 and next is used as address 1.



; ----- Definitions

#define		__VERDAY	0x27
#define		__VERMONTH	0x08
#define		__VERYEAR	0x21

#define		__VERNUM	D'1'
#define		__SUBVERNUM	D'5'

		errorlevel -302
		;errorlevel -306

	IFDEF __10F322
		list    p=10F322,r=hex
		INCLUDE "P10F322.INC"
	ENDIF

	IFDEF __10F320
		list    p=10F320,r=hex
		INCLUDE "P10F320.INC"
	ENDIF

                __FUSES _BOREN_OFF & _CP_ON & _PWRTE_ON & _WDTE_OFF & _LVP_OFF & _MCLRE_OFF  & _FOSC_INTOSC & _WRT_BOOT

#define		RAMINI0		0x040		; 64 bytes
;#define		CVMEMH		0x01		; addres where configuration variables are stored
;#define		CVMEML		0xF0		; 1F0 - must be same as "org" at the end of prog
#define		INS_RETLW	0x34		; instruction RETLW for config variables

; --- Macros

#define		DNOP		goto	$+1


; --- Constant values 

FXTAL		equ	D'8000000'

;RA0 - LED2
;RA1 - LED1
;RA2 - DCC

;RA_TRIS         equ     0x0C			; RA0: out led, RA1 out led, RA2 in dcc, RA3 in addr
RA_TRIS         equ     0x04			; RA0: out led2, RA1 out led1, RA2 in DCC
RA_INI          equ     0x00			; all zero
OPTION_INI	equ	0x08			; Option register: enable pull-up, no prescaler, wdt 1:1
;bit7	1=weak pull up disable 0=WPU enable
;bit6	0=int on falling edge 1=rising
;bit5	0=TMR0 internal clock 1=T0CLKI pin
;bit4	0=TMR0 increment low->high 1=high->low
;bit3	1=prescaler is disabled 0=prescaler is enabled
;bit2-0 prescaler

;INTC_INI	equ	0x90			; GIE, INTE enable, PEIE disable
INTC_INI	equ	0x88			; GIE, IOCIE enable, PEIE disable
PIE1_INI	equ	0x00			; no interrupts
WPUA_INI	equ 0x00			; no weak pull-ups
;T2CON_INI 	equ	0x0D			; Prescaler 1:4, Timer 2 enable, Postscaler 1:2
;PR2_INI		equ	0xF9			; 249 dec = 250 instruction = 1ms with prescaler
;T2CON_INI 	equ	0x05			; Prescaler 1:4, Timer 2 enable
;PR2_INI		equ	0x7C			; 124 dec = 125 instruction = 0,5ms with prescaler
T2CON_INI 	equ	0x7F			; Prescaler 1:64, Timer 2 enable; out postscaler 1:16
PR2_INI		equ	0xFF			; 8MHz / 4 / 64 / 256 = 122Hz for PWM

IOCA_INI	equ	0x04			; used for IOCAP IOAN - IOC on RA2
PWMxCON_INI	equ	0xC0			; PWM module is enabled, Output to PWMx pin is enabled, PWM output is active high
;PWMxCON_INI	equ	0xD0			; PWM module is enabled, Output to PWMx pin is enabled, PWM output is active low
PWMxDCL		equ	0x00			; LOW bits to 00 - LED is off
PWMxDCH		equ	0x00			; HIGH bits to 00 - LED is off

;FVRCON_INI	equ	0x30			; FVR disabled, TSEN Temperature sensor enable, TSRNG temperature sensor High range
;ADCON_INI	equ	0x39			; Fosc/8, temperature indicator, enabled



#define		LED1	LATA,1			; LED output
#define		LED2	LATA,0			; address input
;#define		ACKOUT	LATA,2			; DCC ack pulse pin
#define		DCCIN	PORTA,2			; DCC input pin



; --- EEPROM Section
; no eeprom for this device
; will use self programming instead

; ----- Variables

; --- Internal RAM Section

; --- Top on all banks
	cblock  RAMINI0
INT_W				; interrupt context registers
INT_STAT	
INT_FSR		

SHIFT0				; interrupt shift register
DATA1				; DCC packet registers
DATA2		
DATA3		
DATA4		
DATA5		
DATA6				; variable packet length can be 2 to 6 bytes

PREAMBLE			; received preamble bits
DCCSTATE			; DCC state machine
DCCBYTE				; length of received packet (2 to 6)

MYADDR				; my address

;INTENSITY1			; intensity for LED1
;INTENSITY2			; intensity for LED2	- later

BLINKDELAY			; divider for blinking
BLINKSTATE			; state machine for blinking
BLINKCOUNT			; counter of blinks

FLAGS		

;EEDATA0		equ	RAMINT+0x0C		; EEPROM shadow variables
;EEADR0		equ	RAMINT+0x0D


TEMP		
COUNT		


	endc




; --- Flags
						; FLAGS
;#define		LEDOUT		FLAGS,0		; Flash phase
;#define		ACTIVE	FLAGS,1		; welding is active
;#define		LONGDELAY	FLAGS,2		; CV read only
;#define		DCC4BYTE	FLAGS,3		; DCC command 4 bytes
#define		LIGHTPRG	FLAGS,4		; Status of lights or prog mode 0=lighting, 1=programming
#define		NEW_PACKET	FLAGS,5		; New packet received
;#define		BLINKING		FLAGS,6		; Lights in blinking state
;#define		RESET_FLG	FLAGS,7		; reset packet



; --------------- Program Section --------------------------------------


		org	0x000

PowerUp:
		clrf	STATUS			; Bank 0 default
		clrf	INTCON			; Disable all interrupts
		clrf	PCLATH			; tables on page 0
		goto	INIT

; ----------------------------------------------------------------------

		org	0x004

Interrupt:
		movwf	INT_W			; save context registers		;1
		swapf	STATUS,w							;2
		movwf	INT_STAT							;3
		;clrf	STATUS			; interrupt uses bank 0			;4

Int_DCC:
		;bsf		LED2
		;goto EndInt
		btfss	DCCIN								;5
		goto	Int_Low_Half							;6,7

Int_High_Half:
		movf	DCCSTATE,w							; 8
		addwf	PCL,f								; 9

		goto	Preamble							; 10,11
		goto	WaitLow
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	EndByte
		;goto	EndByte2
		;goto	EndByte3
		;goto	EndByte4

Int_Low_Half:
		movlw	d'256' - d'154'		; 77us: between 64us (one) and 90us (zero);8
		movwf	TMR0								;9
		bcf	INTCON,TMR0IF		; clear overflow flag for counting	;10
;		movf	PORTB,w
		;bsf	OPTION_REG,INTEDG	; next interrupt on rising edge GP2
;		goto	EndInt
		;swapf	INT_STAT,w		; restore context registers		;13
		;movwf	STATUS								;14
		;swapf	INT_W,f								;15
		;swapf	INT_W,w								;16
		;retfie									;17,18

EndHighHalf:
;		movf	PORTB,w
		;bcf	OPTION_REG,INTEDG	; next int. on falling edge GP2	;48	;33
EndInt:
		;bcf	INTCON,INTF							;21
		clrf	IOCAF								;21
		swapf	INT_STAT,w		; restore context registers		;22
		movwf	STATUS								;23
		swapf	INT_W,f								;24
		swapf	INT_W,w								;25
		retfie									;26,27


Preamble:
		incf	PREAMBLE,f		;					;13
		btfsc	INTCON,TMR0IF		; if timer 0 overflows then is a DCC zero;14
		clrf	PREAMBLE		;					;15
		movlw	0xF6			; 10 preamble bits?			;16
		addwf	PREAMBLE,w		;					;17
		btfsc	STATUS,C		;					;18
		incf	DCCSTATE,f		; yes, next state			;19
		goto	EndHighHalf		;					;20,21
		

WaitLow:
		btfss	INTCON,TMR0IF		; if timer 0 overflows then is a DCC zero;12
		goto	EndHighHalf		;					;13,14
		incf	DCCSTATE,f		; new state			;15
		clrf	PREAMBLE		;					;16
		btfsc	NEW_PACKET		; error when previous is not decoded		;17
		clrf	DCCSTATE		; reset state		;18
		clrf	DCCBYTE			;					;19
		goto	EndHighHalf		;					;20,21


ReadBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,TMR0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCSTATE,f		;					;16
		goto	EndHighHalf		;					;17,18
			
;ReadLastBit:
;		bsf	STATUS,C							;12
;		btfsc	INTCON,TMR0IF		; if timer 0 overflows then is a DCC zero;13
;		bcf	STATUS,C							;14
;		rlf	SHIFT0,f		; receiver shift register		;15
;		incf	DCCBYTE,w							;16
;		addwf	DCCSTATE,f							;17
;		goto	EndHighHalf		;					;18,19

EndByte:
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		btfsc	INTCON,TMR0IF		; End bit=1, end of packet		;12
		goto	EndByteCon		;					;13,14
		clrf	DCCSTATE		;					;15
		bsf	NEW_PACKET		;					;18
EndByteCon:
		movf	FSR,w				; save FSR
		movwf	INT_FSR
		movlw	DATA1				; where to store
		addwf	DCCBYTE,w
		movwf	FSR
		movf	SHIFT0,w		;					;16
		movwf	INDF			;					;17
		movf	INT_FSR,w			; restore FSR
		movwf	FSR
		incf	DCCBYTE,f		;					;18
		movf	DCCBYTE,w
		xorlw	0x07				; max 6 byte packet - 7 mean error
		btfsc	STATUS,Z
		clrf	DCCSTATE		;					;15
		goto	EndHighHalf		;					;20,21

; ----------------------------------------------------------------------

CommandSwitch:
		clrf	PCLATH
		addwf	PCL,f

		goto	NOPcommand		; 0x0x NOP
		goto	ProgAddress		; 0x1A Set decoder address
		goto	SetLight		; 0x2A SetLight command
		goto	ExitDecode		; 0x3x is not defined

; ----------------------------------------------------------------------
BlinkStep:
		movf	BLINKSTATE,w
		andlw	0x03
		clrf	PCLATH
		addwf	PCL,f

		goto	BlinkDark
		goto	BlinkLight
		goto	BlinkWait
		clrf	BLINKSTATE		; some mismatch?
		return
; ----------------------------------------------------------------------

INIT:
		clrf	LATA
		clrf	ANSELA
		movlw   RA_TRIS         	; Set port A I/O configuration
		movwf   TRISA
		;movlw	WPUA_INI
		;movwf	WPUA
		clrf	WPUA
		;movlw	FVRCON_INI
		;movwf	FVRCON
		;movlw	ADCON_INI
		;movwf	ADCON
		movlw	PWMxCON_INI
		movwf	PWM1CON
		movwf	PWM2CON
		clrf	PWM1DCL
		clrf	PWM1DCH
		clrf	PWM2DCL
		clrf	PWM2DCH
		
		;clrf	VRCON			; voltage reference off
		movlw	OPTION_INI		; Option register: no pull-up, no prescaler, wdt 1:1
		movwf	OPTION_REG
		movlw	IOCA_INI			; used for IICAP IOAN - IOC on RA3
		movwf	IOCAN
		movwf	IOCAP

		;movlw	PIE1_INI
		;movwf	PIE1
		;bcf	STATUS,RP0		; bank 0
		;clrf	PIR1
		;movlw	0x01			; Timer 1 on, 1:1
		;movwf	T1CON

;		movlw	0x40			; clear RAM
;		movwf	FSR
;ClearRAM:
;		clrf	INDF
;		incf	FSR,f
;		movlw	0x80
;		xorwf	FSR,w
;		btfss	STATUS,Z
;		goto	ClearRAM

		clrf	DCCSTATE
		movlw	INTC_INI
		movwf	INTCON			; enable INT interrupt

		movlw	PR2_INI
		movwf	PR2				; praloader for timer2
		movlw	T2CON_INI
		movwf	T2CON			; enable timer 2


;		clrf	PAGEREG			; page register default

		call	LoadCV		; load my address
		;bsf	ADCON,GO_NOT_DONE
		;bsf		RESET_FLG
		;movf	CV11,w
		;movwf	RANDOM0
		;movf	CV13,w
		;movwf	RANDOM1
		;bsf		RANDOM0,0		; to be sure RANDOM shift reg. isn't zero
		clrf	FLAGS
		;movf	ADRES,w
		;movwf	RANDOM1
		movlw	1
		movwf	BLINKDELAY			; divider for blinking
		clrf	BLINKSTATE			; state machine for blinking
		movf	MYADDR,w
		movwf	BLINKCOUNT			; counter of blinks
		bsf		LIGHTPRG		; OK, we are in prog mode --> we are blinking until packet is received

; ----------------------------------------------------------------------
MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

		btfsc	PIR1,TMR2IF		; time to next LED blink? (in address mode 7.6Hz)
		call	DoBlink			; yes, next step

		;call GetRand
		;swapf	RANDOM0,w		; RANDOM shift register
		;xorwf	RANDOM1,w
		;movwf	TEMP
		;rrf		TEMP,w
		;rlf		RANDOM3,f
		;rlf		RANDOM2,f
		;rlf		RANDOM1,f
		;rlf		RANDOM0,f

		goto	MainLoop

; ----------------------------------------------------------------------

DoBlink:
		bcf	PIR1,TMR2IF		; clear flag
		btfss		LIGHTPRG		; only in prog mode
		return

		decfsz	BLINKDELAY,f
		return
		movlw	4			; 7.6Hz divided by 4 ~ 2Hz
		movwf	BLINKDELAY
		goto	BlinkStep

BlinkDark:
		clrf	PWM2DCL		; turn off LED1
		clrf	PWM2DCH
		incf	BLINKSTATE,f	; next state
		return

BlinkLight:
		movlw	0xFF		; turn LED1 to maximum intensity
		movwf	PWM2DCH
		movlw	b'11000000'
		movwf	PWM2DCL
		incf	BLINKSTATE,f	; next state
		decfsz	BLINKCOUNT,f
		goto	BlinkLightLoop
		movlw	4				; space between blinks
		movwf	BLINKCOUNT
		return
BlinkLightLoop:
		clrf	BLINKSTATE		; back to dark, we have blink steps
		return

BlinkWait:
		clrf	PWM2DCL		; turn off LED1
		clrf	PWM2DCH
		decfsz	BLINKCOUNT,f	; until end of time
		return
		movf	MYADDR,w	; blink count by my address
		btfsc	STATUS,Z	; check for address 0
		movlw	.16			; replace 0 by 16
		movwf	BLINKCOUNT
		clrf	BLINKSTATE		; back to begin, new round of blinking
		return

; ----------------------------------------------------------------------


Decode:
		;bcf	INTCON,GIE		; disable interrupts for more speed
		movf	DCCBYTE,w
		movwf	COUNT
		movwf	TEMP
		movlw	DATA1			; packet data
		movwf	FSR
		clrw
CheckXorLoop:
		xorwf	INDF,w
		incf	FSR,f
		decfsz	COUNT,f
		goto	CheckXorLoop

		bcf	NEW_PACKET		; prepare for next packet

		addlw	0				; set Z flag destroyed by decfsz
		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

		movf	DATA1,w			; command = '00xxaaaa' ?
		andlw	0xC0			; understand only commands 0 to 2
		btfss	STATUS,Z
		goto	ExitDecode		; no, return

		swapf	DATA1,w			; command
		andlw	0x03			; filter command only
		goto	CommandSwitch	; switch by command

; 0x0x - this is nop command. used address make no sense, all slaves can ignore this command.
NOPcommand:		; 0x0x NOP
				; nothing to done
		goto	ExitDecode

; 0x1A - this command enforce connected slave(s) to use address A. Command should be followed by one data byte 0x0A, where both A should be same.
;        slave present received address by blinking (number of blinks is current address)
;        in case A = 0, it mean no change of address, but blink current address (read address only)
ProgAddress:	; 0x1A Set decoder address
		movf	TEMP,w			; packet length
		xorlw	0x03			; programming packet must be 2 bytes long + XOR
		btfss	STATUS,Z
		goto	ExitDecode		; no, return

		movf	DATA1,w			; DETA1 and DATA2 
		xorwf	DATA2,w
		xorlw	0x10			; first byte is 0x1A, second is 0x0A
		btfss	STATUS,Z
		goto	ExitDecode		; no, return

		bsf		LIGHTPRG		; OK, we are in prog mode

		movf	DATA2,w			; check for address 0 (obsoleted address 0 is valid address)
		;btfsc	STATUS,Z
		;goto	ExitDecode		; yes, nothing else is requested

		xorwf	MYADDR,w		; compare with actual address
		btfsc	STATUS,Z
		goto	ExitDecode		; they are same, nothing else is requested

		movf	DATA2,w			; set my address
		movwf	MYADDR			; update address

		goto	WriteCV

; 0x2A - this command is followed by 1 to 4 data bytes. data bytes contain light intensity information for modules with address A to A+3.
;        when A=0, then first data byte is ignored by all slaves, but second data byte is regularly used by slave with address 1 etc.
;        data byte address roll over. It mean for example: when A = F, then first data byte is for address F, next data byte is ignored 
;        as for address 0 and next is used as address 1.
SetLight:		; 0x2A SetLight command
		movf	DATA1,w			; prepare address
		andlw	0x0F			; address only
		movwf	COUNT			; put it to counter
		movlw	DATA2			; pointer to data
		movwf	FSR				; to FSR
		decf	TEMP,f			; remove XOR
		decfsz	TEMP,f			; remove command, is some data remaining?
		goto	SetLightLoop	; continue parsing data
		goto	ExitDecode		; No data - nothing to done
		
SetLightLoop:
		movf	COUNT,w			; check current byte target
		xorwf	MYADDR,w		; is current same as my one?
		btfsc	STATUS,Z
		goto	FoundAddr		; if yes, use it
		incf	COUNT,f			; try next address
		incf	FSR,f			; and next data
		decfsz	TEMP,f			; until end of packet
		goto	SetLightLoop
		goto	ExitDecode		; no data bytes remaining

FoundAddr:
		bcf		LIGHTPRG		; OK, we are in light mode
		movf	INDF,w			; check data for "dark"
		;btfsc	STATUS,Z
		;goto	TurnLEDoff
		movwf	PWM2DCH			; use value as HIGH value
		;movlw	b'11000000'		; set LOW as 11 to have 100% for value 0xFF
		movwf	PWM2DCL
		;goto	ExitDecode		; and that is all

;TurnLEDoff:
;		clrf	PWM2DCH			; set value to 000 to have 0%
;		clrf	PWM2DCL
;		goto	ExitDecode		; and that is all


ExitDecode:
		;bcf	RESET_FLG
		;bsf	INTCON,GIE		; enable interrupts
		return


;---------------------------------------------------------------------------
LoadCV:
		call LoadCV1
		movwf MYADDR

		return

;* This code block will read 1 word of program
;* memory at the memory address:
;* PROG_ADDR_HI: PROG_ADDR_LO
;* data will be returned in the variables;
;* PROG_DATA_HI, PROG_DATA_LO
;	BANKSEL PMADRL 		; not required on devices with 1 Bank of SFRs
;	MOVLW PROG_ADDR_LO 	;
;	MOVWF PMADRL 		; Store LSB of address
;	MOVLW PROG_ADDR_HI 	;
;	MOVWF PMADRH 		; Store MSB of address
;	BCF PMCON1,CFGS 	; Do not select Configuration Space
;	BSF PMCON1,RD 		; Initiate read
;	NOP 				; Ignored (Figure 9-2)
;	NOP 				; Ignored (Figure 9-2)
;	MOVF PMDATL,W 		; Get LSB of word
;	MOVWF PROG_DATA_LO 	; Store in user location
;	MOVF PMDATH,W 		; Get MSB of word
;	MOVWF PROG_DATA_HI 	; Store in user location

WriteCV:
;; This row erase routine assumes the following:
;; 1. A valid address within the erase row is loaded in ADDRH:ADDRL
;; 2. ADDRH and ADDRL are located in shared data memory 0x70 - 0x7F (common RAM)
	BCF INTCON,GIE 		; Disable ints so required sequences will execute properly
;	BANKSEL PMADRL 		; not required on devices with 1 Bank of SFRs
	MOVLW LOW(LoadCV1)	; Load lower 8 bits of erase address boundary
	MOVWF PMADRL
	MOVLW HIGH(LoadCV1)	; Load upper 6 bits of erase address boundary
	MOVWF PMADRH
	BCF PMCON1,CFGS 	; Not configuration space
	BSF PMCON1,FREE 	; Specify an erase operation
	call EEwriteUnlock
	bcf PMCON1,FREE

;	to save space - unlock is separate...
;	BSF PMCON1,WREN 	; Enable writes
;	MOVLW 55h 			; Start of required sequence to initiate erase
;	MOVWF PMCON2 		; Write 55h
;	MOVLW 0AAh 			;
;	MOVWF PMCON2 		; Write AAh
;	BSF PMCON1,WR 		; Set WR bit to begin erase
;	NOP 				; NOP instructions are forced as processor starts
;	NOP 				; row erase of program memory.
;						;
;						; The processor stalls until the erase process is complete
;						; after erase processor continues with 3rd instruction
;	BCF PMCON1,WREN 	; Disable writes
;	BSF INTCON,GIE 		; Enable interrupts

;; This write routine assumes the following:
;; 1. 64 bytes of data are loaded, starting at the address in DATA_ADDR
;; 2. Each word of data to be written is made up of two adjacent bytes in DATA_ADDR,
;; stored in little endian format
;; 3. A valid starting address (the least significant bits = 00000) is loaded in ADDRH:ADDRL
;; 4. ADDRH and ADDRL are located in shared data memory 0x70 - 0x7F (common RAM)
;;
;	BCF INTCON,GIE 		; Disable ints so required sequences will execute properly
;	BANKSEL PMADRH 		; not required on devices with 1 Bank of SFRs
	;MOVLW HIGH(LoadCV1)	; Load initial address
	;MOVWF PMADRH 		;
	;MOVLW LOW(LoadCV1)	;
	;MOVWF PMADRL 		;
;	MOVLW LOW DATA_ADDR ; Load initial data address
;	MOVWF FSR0 			;
	BCF PMCON1,CFGS 	; Not configuration space
;	BSF PMCON1,WREN 	; Enable writes
	BSF PMCON1,LWLO 	; Only Load Write Latches
	movf MYADDR,w
	call EEWriteData
	incf PMADRL,f
	clrw
	bcf PMCON1,LWLO
	call EEWriteData

;LOOP
;	MOVIW FSR0++ 		; Load first data byte into lower
;	MOVWF PMDATL 		;
;	MOVIW FSR0++ 		; Load second data byte into upper
;	MOVWF PMDATH 		;
;	MOVF PMADRL,W 		; Check if lower bits of address are '00000'
;	XORLW 0x1F 			; Check if we're on the last of 16 addresses
;	ANDLW 0x1F 			;
;	BTFSC STATUS,Z 		; Exit if last of 16 words,
;	GOTO START_WRITE 	;
;	MOVLW 55h 			; Start of required write sequence:
;	MOVWF PMCON2 		; Write 55h
;	MOVLW 0AAh 			;
;	MOVWF PMCON2 		; Write AAh
;	BSF PMCON1,WR 		; Set WR bit to begin write
;	NOP 				; NOP instructions are forced as processor
;						; loads program memory write latches
;	NOP 				;
;	INCF PMADRL,F 		; Still loading latches Increment address
;	GOTO LOOP 			; Write next latches
;START_WRITE
;	BCF PMCON1,LWLO 	; No more loading latches - Actually start Flash program
;						; memory write
;	MOVLW 55h 			; Start of required write sequence:
;	MOVWF PMCON2 		; Write 55h
;	MOVLW 0AAh 			;
;	MOVWF PMCON2 		; Write AAh
;	BSF PMCON1,WR 		; Set WR bit to begin write
;	NOP 				; NOP instructions are forced as processor writes
;						; all the program memory write latches simultaneously
;	NOP 				; to program memory.
;						; After NOPs, the processor
;						; stalls until the self-write process in complete
;						; after write processor continues with 3rd instruction
;	BCF PMCON1,WREN 	; Disable writes
	BSF INTCON,GIE 		; Enable interrupts
	;goto	AccessOut
	return

EEWriteData:
	movwf PMDATL
	movlw INS_RETLW
	movwf PMDATH

EEwriteUnlock:
	BSF PMCON1,WREN 	; Enable writes
	MOVLW 55h 			; Start of required sequence to initiate erase
	MOVWF PMCON2 		; Write 55h
	MOVLW 0AAh 			;
	MOVWF PMCON2 		; Write AAh
	BSF PMCON1,WR 		; Set WR bit to begin erase
	NOP 				; NOP instructions are forced as processor starts
	NOP 				; row erase of program memory.
						;
						; The processor stalls until the erase process is complete
						; after erase processor continues with 3rd instruction
	BCF PMCON1,WREN 	; Disable writes
	return

; ----- EEPROM default values
; no EEPROM, should use self programming

;Roco address calc: Addr = 11 bit
;aaaAAAAAADD	| 5 = 00000000101	| minus 1 
;aaaAAAAAADD	| 5-1 = 00000000100	| part "aaa" is inverted
;aaaAAAAAADD	| 11100000100
;CV1=10AAAAAA	| 10000001 = 0x81
;CV9=1aaa1DD0	| 11111000 = 0xF8

;Lenz address calc: Addr = 11 bit
;aaaAAAAAADD	| 1 = 00000000001	| plus 3 
;aaaAAAAAADD	| 1+3 = 00000000100	| part "aaa" is inverted
;aaaAAAAAADD	| 11100000100
;CV1=10AAAAAA	| 10000001 = 0x81
;CV9=1aaa1DD0	| 11111000 = 0xF8


	IFDEF __10F322
		org	0x01F0
	ENDIF

	IFDEF __10F320
		org	0x00F0
	ENDIF

LoadCV1:
		retlw 0x08		; Low address bits
	end

