;;======================================================================;;
;;			DCC Command Station for lego Project 			;;
;;======================================================================;;
;;									;;
;; Program:		LegoX			;;
;; Code:		Jindra Fucík					;;
;; Platform:		Microchip PIC16F15313, 8 Mhz			;;
;; Date:		22.08.2021					;;
;; First release:	22.08.2021					;;
;; LastDate:		22.08.2021					;;
;;									;;
;;======================================================================;;

; This program is distributed as is but WITHOUT ANY WARRANTY
; License: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en

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

#define		__VERNUM	D'1'
#define		__VERDAY	0x22
#define		__VERMONTH	0x08
#define		__VERYEAR	0x21

		errorlevel	-302		; suppress Bank warnings
		errorlevel	-306		; suppress Bank warnings

    processor 16F15313
    #include <P16F15313.INC>
#ifdef _DEBUG 
; CONFIG1
 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINT1 & _CLKOUTEN_OFF & _CSWEN_ON & _FCMEN_ON
; CONFIG2
 __CONFIG _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _WDTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LOW & _PPS1WAY_OFF & _STVREN_ON & _DEBUG_OFF
; CONFIG3
 __CONFIG _CONFIG3, _WRT_ALL & _LVP_OFF		; all write protected
; CONFIG4
 __CONFIG _CONFIG4, _CP_OFF & _CPD_OFF
#else
; CONFIG1
 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINT1 & _CLKOUTEN_OFF & _CSWEN_ON & _FCMEN_ON
; CONFIG2
 __CONFIG _CONFIG2, _MCLRE_OFF & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_OFF & _STVREN_ON
; CONFIG3
 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC
; CONFIG4
 __CONFIG _CONFIG4, _BBSIZE_BB512 & _BBEN_OFF & _SAFEN_ON & _WRTAPP_OFF & _WRTB_OFF & _WRTC_OFF & _WRTSAF_OFF & _LVP_OFF
; CONFIG5
 __CONFIG _CONFIG5, _CP_OFF
#endif
 

#define		RAMINI0		0x020		; 80 bytes
#define		RAMINI1		0x0A0		; 80 bytes
#define		RAMINI2		0x120		; 80 bytes
#define		RAMINT		0x070		; 16 bytes

;Memory map:
; BANK0:
; usual variables - 80 bytes
; BANK1:
; BANK2:

; ----- Macros

#define		DNOP		goto	$+1

;       +----U----+
;      -|Vdd   Vss|-
;DCC J -|RA5   RA0|- Enc 1
;DCC K -|RA4   RA1|- Enc 2
;Enc 8 -|RA3   RA2|- Enc 4
;       +---------+


DCCPolarity1		equ b'00010000'		; IN2=0, IN1=1 - pulse 1
DCCPolarity2		equ b'00100000'		; IN2=1, IN1=0 - pulse 2
;DCCCutOut			equ b'01110000'		; IN2=1, IN1=1, Enable=1 - RailCom Cut out
DCCPwrOff			equ b'00000000'		; IN2=0, IN1=0 - Power off


; ----- Constant values

FXTAL		equ	D'8000000'		; Crystal frequency

DCC_HalfBit	equ	D'55'			; DCC half bit duration

RTC_NUM		equ	D'256'-(DCC_HalfBit - D'8')

; PMD = PERIPHERAL MODULE DISABLE
PMD0_INI	equ	b'01000101'	; CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; FVRMD FVR disabled; IOCMD IOC disabled; NVMMD NVM disabled; 
PMD1_INI	equ	b'10000000'	; TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR2MD TMR2 enabled; NCOMD DDS(NCO) disabled; 
PMD2_INI	equ	b'01100111'	; CMP1MD CMP1 disabled; ADCMD ADC disabled; CMP2MD CMP2 disabled; DAC1MD DAC1 disabled; 
PMD3_INI	equ	b'00111111'	; CCP2MD CCP2 disabled; CCP1MD CCP1 disabled; PWM4MD PWM4 disabled; PWM3MD PWM3 disabled; PWM6MD PWM6 disabled; PWM5MD PWM5 disabled; 
PMD4_INI	equ	b'01010001'	; MSSP1MD MSSP1 disabled; CWG1MD CWG1 disabled; UART1MD EUSART enabled;
PMD5_INI	equ	b'00011110'	; CLC3MD CLC3 disabled; CLC4MD CLC4 disabled; CLC1MD CLC1 enabled; CLC2MD CLC2 disabled;

; PIN manager
LATA_INI    equ 0x00	    ; all outputs to zero
;LATB_INI    equ 0x00	    ; all outputs to zero
;LATC_INI    equ 0x03	    ; enable main, enable prog

TRISA_INI   equ b'00001111' ; RA0-RA3 in - switch; RA4, RA5 out - DCC
;TRISB_INI   equ b'10000000' ; RB4 out DCC Enable; RB5 out DCC K; RB6 out DCC J; RB7 in Overload
;TRISC_INI   equ b'11101000' ; RC0 out Enable main; RC1 out Enable prog; RC2 out LED; RC3 in RxD; RC4 out RxTx; RC5 in TxD; RC6 in (analog) track load; RC7 in Prog ACK

ANSELA_INI  equ b'00000000' ; all digital
;ANSELB_INI  equ b'00000000' ; all digital
;ANSELC_INI  equ b'01000000' ; RC6 in (analog) track load

WPUA_INI    equ	0x0F	    ; WPU for all inputs
;WPUB_INI    equ	0x00	    ; no WPU
;WPUC_INI    equ	0x00	    ; no WPU

ODCONA_INI  equ	0x00	    ; disable open drain outputs
;ODCONB_INI  equ	0x00	    ; disable open drain outputs
;ODCONC_INI  equ	0x00	    ; disable open drain outputs

; oscilator management
OSCCON1_INI equ 0x60	    ; NOSC HFINTOSC; NDIV 1:1; 
;OSCCON2 is read only
OSCCON3_INI equ 0x00	    ; CSWHOLD may proceed; 
OSCEN_INI   equ 0x00	    ; MFOEN disabled; LFOEN disabled; ADOEN disabled; EXTOEN disabled; HFOEN disabled; 
OSCFRQ_INI  equ 0x03	    ; HFFRQ 8_MHz; 
OSCSTAT_INI equ 0x00	    ; MFOR not ready; do not understand, it is read only
;OSCTUNE_INI equ 0x20	    ; HFTUN 32; do not understand why MCC set this?
OSCTUNE_INI equ 0x00	    ; HFTUN 0; default

; Timer2 management			; Timer 2 is used for led effect calculation
T2CLKCON_INI	equ 0x01    ; T2CS FOSC/4; 
T2HLT_INI   equ 0x00    ; T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized; 
T2RST_INI   equ 0x00	; T2RSEL T2INPPS pin; 
T2PR_INI    equ d'244'	; T2PR 244 ~ 64Hz; 8 steps per led change with 8 leds (with 1:128 prescaler)
;;T2TMR = 0x00; TMR2 0; 
T2CON_INI   equ b'11110000'	; T2CKPS 1:128; T2OUTPS 1:1; TMR2ON on; 

; Timer1 management
;T1GCON_INI	equ 0x00	; T1GE disabled; T1GTM disabled; T1GPOL low; T1GGO done; T1GSPM disabled; 
;T1GATE_INI	equ 0x00	; GSS T1G_pin; 
;T1CLK_INI	equ 0x01	; ClockSelect FOSC/4; 
;TMR1H = 0x00;    //TMR1H 0; 
;TMR1L = 0x00;    //TMR1L 0; 
;T1CON_INI	equ 0x21	; CKPS 1:4; nT1SYNC do_not_synchronize; TMR1ON enabled ClockSelect FOSC/4; 

; Timer0 management
T0CON1_INI	equ b'01010001'	; T0CS Fosc/4; T0CKPS 1:2; T0ASYNC not_synchronised; 
T0CON0_INI	equ b'10000000'	; T0OUTPS 1:1; T0EN enabled; T016BIT 8-bit; 
TMR0H_INI	equ DCC_HalfBit-1	; in 8 bit mode the TMR0H is compared same as PR2 with timer2
;TMR0L = 0x00; TMR0L 0; 

; EUSART management
;BAUD1CON_INI	equ 0x00	; ABDOVF no_overflow; SCKP Non-Inverted; BRG16 8bit_generator; WUE disabled; ABDEN disabled; 
;RC1STA_INI	equ b'11010000'	; SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
;TX1STA_INI	equ b'01100100'	; TX9 9-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
;SP1BRGL_INI	equ HIGH_BAUD	; SP1BRGL 29; 
;SP1BRGH_INI	equ 0x00	; SP1BRGH 0; 

; Analog to digital converter
;ADCON0_INI   equ	b'01011001'	; input is RC0, AD is enabled
;ADCON1_INI   equ	b'10010000'	; Right justified, Fosc/8, Vdd = Vref
;ADACT_INI    equ	b'00000000'	; no auto activate

; CLC 1 management
;CLC1CON_INI	equ	b'10000000'	; LC1EN is enabled, LC1INT disabled, LC1MODE is AND-OR
;CLC1POL_INI	equ	b'00000010'	; LC1G1 not inverted, LC1G2 inverted, LC1G3 not inverted, LC1G4 not inverted, LC1POL not invrted
;CLC1POL_PRG_INI	equ	b'00001000'	; LC1G1 not inverted, LC1G2 not inverted, LC1G3 not inverted, LC1G4 inverted, LC1POL not invrted
;CLC1SEL0_INI	equ	b'00000000'	; data 0 is CLCIN0PPS RB7 - overflow
;CLC1SEL1_INI	equ	b'00000001'	; data 1 is CLCIN1PPS RC7 - ACK pulse
;CLC1SEL2_INI	equ	b'00000000'	; data 2 is CLCIN0PPS
;CLC1SEL3_INI	equ	b'00000000'	; data 3 is CLCIN0PPS
;CLC1GLS0_INI	equ	b'00000010'	; gate 0 input is data 0 non inverted
;CLC1GLS1_INI	equ	b'00000000'	; gate 1 input is none
;CLC1GLS2_INI	equ	b'00001000'	; gate 2 input is data 1 non inverted
;CLC1GLS3_INI	equ	b'00000000'	; gate 3 input is none

;RC5PPS_INI	equ b'00010100'		; RC5->TX;
;RXPPS_INI	equ b'00010011'		; RX->RC3;

; Interrupt section
PIE0_INI	equ 0x20	; Timer 0 interrupt
PIE1_INI	equ 0x00	; none used
PIE2_INI	equ 0x00	; none used
PIE3_INI	equ 0x00	; none used
PIE4_INI	equ 0x00	; none used
PIE5_INI	equ 0x00	; none used
PIE6_INI	equ 0x00	; none used
PIE7_INI	equ 0x00	; none used

INTC_INI	equ 0xC0	; GIE enable, PIE enable, falling edge of INT

;COMPORT		equ	PORTC
;TXRX		equ	4			; 1: enable, 0: disable RS485 transmision
;RXD		equ	3
;TXD		equ	5

DCCPORT		equ	LATA
;#define		DCCOVRLD	PORTB,7
;#define		DCCACKNLG	PORTC,7

; --- EEPROM Section
;  here is no EEPROM, we have to use storage area flash (SAF)
;  SAF is defined as Last Program Memory Address - 7Fh
;  0x07FF - 0x7F = 0x780
;  Row Erase (words) = 32

#define		SAF_INI	0x00780
#define		SAF_LAST	0x007FF


; RAM-Variable
		cblock  RAMINI0
TEMP		; generic temp value
COUNT		; various counter
FLAGS		; various flags

DCCIntState	; DCC interrupt state machine
DCCSEND1	; DCC extra packet byte 1
DCCSEND2	; DCC extra packet byte 2
DCCSEND3	; DCC extra packet byte 3
DCCSEND4	; DCC extra packet byte 4
DCCSEND5	; DCC extra packet byte 5
DCCSEND6	; DCC extra packet byte 6
DCCSENDL	; DCC extra packet length
DCCPulsePol	; polarity for pulse

PKT_COUNT	; packet counter (used o determine LEDs to be refreshed)
LED_BUF:8	; buffer of led statuses
LED_COUNT	; frequency divider
LED_POS		; position of max lighting LED on screen
		endc


		cblock  RAMINT
PREAMB_LEN	;equ	RAMINT+0x06
PULSE_LEN	;equ	RAMINT+0x07
INT_COUNT	;equ	RAMINT+0x08		; counter in interrupt

DCC_CUR_PKT_0		;equ	RAMINT+0x09
DCC_CUR_PKT_1		;equ	RAMINT+0x0A
DCC_CUR_PKT_2		;equ	RAMINT+0x0B
DCC_CUR_PKT_3		;equ	RAMINT+0x0C
DCC_CUR_PKT_4		;equ	RAMINT+0x0D
DCC_CUR_PKT_5		;equ	RAMINT+0x0E
DCC_CUR_PKT_LEN		;equ	RAMINT+0x0F
		endc

; --- Flags
						; FLAGS
#define		DCC_OFF		FLAGS,0		; Flash phase
#define		DCC_PULSE_V	FLAGS,1		; Pulse value (for 2nd half of pulse)


; Program

    Org 0x0000

;   Reset-Vector
;LADR_0x0000
    ;CLRF STATUS
    CLRF PCLATH          ; !!Bank Program-Page-Select
    CLRF INTCON
;    bsf PCLATH,3
    GOTO Start     ; !!Bank!! 0x0106 - 0x0906

;
;**********************************************************************************************************************
; ISR (Interrupt Service Routines)
;**********************************************************************************************************************
;
    Org 0x0004
;   Interrupt-Vector	47 instructions
    CLRF PCLATH			; interrupt uses page 0
	movlb 0			; bank 0
    MOVLW DCCPolarity1		; IN2=0, IN1=1, Enable=1 - pulse 1
    BTFSS DCCPulsePol,0		; check DCC pulse polarity
    MOVLW DCCPolarity2		; IN2=1, IN1=0, Enable=1 - pulse 2
    ;BTFSC RCOM_COUT
    ;MOVLW DCCCutOut		; IN2=1, IN1=1, Enable=1 - RailCom Cut out
    BTFSC DCC_OFF		;
    MOVLW DCCPwrOff		; IN2=0, IN1=0, Enable=0 - Track off
    MOVWF DCCPORT		;
    DECFSZ PULSE_LEN,F		; pulse length
    GOTO DCCSamePulse		; 
    INCF PULSE_LEN,F		; minimum pulse length is 1
    INCF DCCPulsePol,F		; next polarity
    BTFSS DCCPulsePol,0		;
    GOTO DCCPulse2ndHalf	;
DCCPulse1stHalf:
    MOVF DCCIntState,W		;
	BRW
    GOTO DCCUpdateCustomPacket	;0
    GOTO DCCPreambleStep	;1 
    GOTO DCCByteInitStep	;2 
    GOTO DCCByteBitStep		;3 
    GOTO DCCEndPacket		;4 
    ;GOTO CheckRailcom		;5 
    ;GOTO CheckRailcomEnd	;6 
    ;GOTO CheckRXSendInquiry	;7 
    ;GOTO CheckRXCloseInquiry	;8 


DCCPreambleStep:
    DECF PREAMB_LEN,F	;
    BTFSC STATUS,Z
    INCF DCCIntState,F	;
	;btfss DCCOVRLD		; overload on DCC
    ;BSF OVERLOAD		;
	;btfss DCCACKNLG		; ACK pulse detect
    ;BSF DCCACKF			; 
	;BANKSEL ADCON0		; BANK 1
	;bsf ADCON0,ADGO		; measure load by ADC
    GOTO EndInt			; EndInt

DCCByteInitStep:
    MOVLW 0x08			; 8 bits per byte
    MOVWF INT_COUNT		;
    INCF DCCIntState,F		; next state
    BSF DCC_PULSE_V		; DCC pulse is 0 - set for 2nd half
    ;movf ZeroPos,w
    ;addwF PULSE_LEN,F		; Set pulse 0 (end of preamble)
	incf PULSE_LEN,F		; Set pulse 0 (end of preamble)
	;btfss DCCOVRLD		; overload on DCC ?
    ;BSF OVERLOAD		;
	;btfss DCCACKNLG		; ACK pulse detect ?
    ;BSF DCCACKF			; 
	;BANKSEL ADCON0		; BANK 1
	;bsf ADCON0,ADGO
    GOTO EndInt			; EndInt

DCCByteBitStep:
    ;movf ZeroPos,w
    BTFSS DCC_CUR_PKT_0,7	; is current bit 0 ??
    ;addwf PULSE_LEN,F		; for bit value = 0 inc pulse length to have 0
	incf PULSE_LEN,F		; for bit value = 0 inc pulse length to have 0
    BTFSS DCC_CUR_PKT_0,7	; is current bit 0 ??
    BSF DCC_PULSE_V		; DCC pulse is 0 - set for 2nd half
    RLF DCC_CUR_PKT_5,F		; rotate rest of data
    RLF DCC_CUR_PKT_4,F		;
    RLF DCC_CUR_PKT_3,F		;
    RLF DCC_CUR_PKT_2,F		;
    RLF DCC_CUR_PKT_1,F		;
    RLF DCC_CUR_PKT_0,F		;
    DECFSZ INT_COUNT,F		;
    GOTO EndInt			; EndInt
    MOVLW 0x04			;   INCF DCCIntState,w - next state
    DECFSZ DCC_CUR_PKT_LEN,F	; some bits remaining?
    MOVLW 0x02			;   yes, then previous state
    MOVWF DCCIntState		; to dcc state
    GOTO EndInt			; EndInt

DCCEndPacket:
    ;INCF DCCIntState,F		;
	clrf DCCIntState	; nothing to done, bit value 1 is automatic
	;btfss DCCOVRLD		; overload on DCC ?
    ;BSF OVERLOAD		;
	;btfss DCCACKNLG		; ACK pulse detect ?
    ;BSF DCCACKF			;
    GOTO EndInt			; EndInt

DCCSetNOPPacket:
    MOVLW 0x14			;   20preamble bits
    MOVWF PREAMB_LEN		;
    ;   00 00 - NOP
    CLRF DCC_CUR_PKT_0		;
    CLRF DCC_CUR_PKT_1		;
    MOVLW 0x02			;   length
    MOVWF DCC_CUR_PKT_LEN	;
    GOTO EndIntNextStep		;

DCCUpdateCustomPacket:
    MOVF DCCSENDL,W		;
    BTFSC STATUS,Z
    GOTO DCCSetNOPPacket	;
    ANDLW 0x07           ; max 6 bytes packet, upper bits can be used as flags
    MOVWF DCC_CUR_PKT_LEN	; set length
    MOVF DCCSEND1,W		; Command + address
    MOVWF DCC_CUR_PKT_0	;
    MOVF DCCSEND2,W		; First data byte
    MOVWF DCC_CUR_PKT_1	;
    MOVF DCCSEND3,W		; Second data byte
    MOVWF DCC_CUR_PKT_2	;
    MOVF DCCSEND4,W		; Third data byte
    MOVWF DCC_CUR_PKT_3	;
    MOVF DCCSEND5,W		; Fourth data byte
    MOVWF DCC_CUR_PKT_4	;
    MOVF DCCSEND6,W		; XOR byte
    MOVWF DCC_CUR_PKT_5	;
    MOVLW 0x14           ; 20 preamble bits
    ;BTFSC DCCSENDL,7	; request for long preamble
    ;MOVLW 0x1E           ; 30 preamble bits for long preamble
    MOVWF PREAMB_LEN	;
    CLRF DCCSENDL		; Mark packet is processed

EndIntNextStep:
    INCF DCCIntState,F	;
EndInt:
	BANKSEL PIR0		; BANK 14
    BCF PIR0,TMR0IF
    RETFIE
	

DCCPulse2ndHalf:
    ;movf ZeroNeg,w
    BTFSC DCC_PULSE_V	; Check value for 2nd half
    ;addwf PULSE_LEN,F	;
    incf PULSE_LEN,F	; increase if value is 0
    BCF DCC_PULSE_V		; remove for future
DCCSamePulse:    ; moved from top
	BANKSEL PIR0		; BANK 14
    BCF PIR0,TMR0IF
    RETFIE


Start:
	BANKSEL	LATA		; BANK 0
	movlw	LATA_INI	; 
	movwf	LATA
	;movlw	LATB_INI	; 
	;movwf	PORTB
	;movlw	LATC_INI	; 
	;movwf	PORTC

	;BANKSEL	TRISA		; BANK 0
	movlw	TRISA_INI	; 
	movwf	TRISA
	;movlw	TRISB_INI	; 
	;movwf	TRISB
	;movlw	TRISC_INI	; 
	;movwf	TRISC

	; Analog to digital converter
	;movlw	ADCON0_INI	; input is RC0, AD is enabled
	;movwf	ADCON0
	;movlw	ADCON1_INI	; Right justified, Fosc/8, Vdd = Vref
	;movwf	ADCON1
	;movlw	ADACT_INI	; no auto activate
	;movwf	ADACT

	; PMD = PERIPHERAL MODULE DISABLE
	BANKSEL	PMD0		; BANK 15
	movlw	PMD0_INI	; 
	movwf	PMD0
	movlw	PMD1_INI	; 
	movwf	PMD1
	movlw	PMD2_INI	; 
	movwf	PMD2
	movlw	PMD3_INI	; 
	movwf	PMD3
	movlw	PMD4_INI	; 
	movwf	PMD4
	movlw	PMD5_INI	; 
	movwf	PMD5

; PIN manager
	BANKSEL	ANSELA		; BANK 62
	movlw	ANSELA_INI	; 
	movwf	ANSELA
	;movlw	ANSELB_INI	; 
	;movwf	ANSELB
	;movlw	ANSELC_INI	; 
	;movwf	ANSELC

	;BANKSEL	WPUA		; BANK 62
	movlw	WPUA_INI	; 
	movwf	WPUA
	;movlw	WPUB_INI	; 
	;movwf	WPUB
	;movlw	WPUC_INI	; 
	;movwf	WPUC


	;BANKSEL	ODCONA		; BANK 62
	movlw	ODCONA_INI	; 
	movwf	ODCONA
	;movlw	ODCONB_INI	; 
	;movwf	ODCONB
	;movlw	ODCONC_INI	; 
	;movwf	ODCONC

	;BANKSEL	SLRCONA		; BANK 62
	clrf	SLRCONA
	;clrf	SLRCONB
	;clrf	SLRCONC

; EUSART management
	;BANKSEL	RC1STA		; BANK 3
	;movlw BAUD1CON_INI	;
	;movwf	BAUD1CON
	;movlw RC1STA_INI	;
	;movwf	RC1STA
	;movlw TX1STA_INI	;
	;movwf	TX1STA
	;movlw SP1BRGL_INI	;
	;movwf	SP1BRGL
	;clrf	SP1BRGH


; oscilator management
	BANKSEL	OSCCON1		; BANK 17
	movlw	OSCCON1_INI	; 
	movwf	OSCCON1
		;OSCCON2 is read only
	movlw	OSCCON3_INI	; 
	movwf	OSCCON3
	movlw	OSCEN_INI	; 
	movwf	OSCEN
	movlw	OSCFRQ_INI	; 
	movwf	OSCFRQ
	movlw	OSCSTAT_INI	; 
	movwf	OSCSTAT
	movlw	OSCTUNE_INI	; 
	movwf	OSCTUNE

; Timer2 management
	BANKSEL	T2CON		; BANK 5
	movlw	T2CLKCON_INI	; T2CS FOSC/4; 
	movwf	T2CLKCON
	movlw	T2HLT_INI	; T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized; 
	movwf	T2HLT
	movlw	T2RST_INI	; T2RSEL T2INPPS pin; 
	movwf	T2RST
	movlw	T2PR_INI	; 
	movwf	PR2
	movlw	T2CON_INI	; 0.5ms
	movwf	T2CON

; Timer1 management
	;BANKSEL	T1GCON		; BANK 4
	;movlw	T1GCON_INI	; 
	;movwf	T1GCON
	;movlw	T1CON_INI	; 
	;movwf	T1CON

; Timer0 management
	BANKSEL	TMR0L		; BANK 11
	movlw	T0CON1_INI	; 
	movwf	T0CON1
	movlw	T0CON0_INI	; 
	movwf	T0CON0
	movlw	TMR0H_INI	; 
	movwf	TMR0H
	;movlw	TMR0L_INI
	;movwf	TMR0L		; 
	;TMR0L = 0x00; TMR0L 0; 

	;BANKSEL	PPSLOCK		; BANK 28
	;PPSLOCK	default unlocked
	;BANKSEL	RXPPS		; BANK 28
	;movlw	RXPPS_INI	;
	;movwf	RXPPS
	;BANKSEL	RC5PPS		; BANK 29
	;movlw	RC5PPS_INI	;
	;movwf	RC5PPS
	
; Interrupts
	BANKSEL	PIE0	; BANK 14
	movlw	PIE0_INI	; 
	movwf	PIE0
	movlw	PIE1_INI	; 
	movwf	PIE1
	movlw	PIE2_INI	; 
	movwf	PIE2
	movlw	PIE3_INI	; 
	movwf	PIE3
	movlw	PIE4_INI	; 
	movwf	PIE4

	;BANKSEL	TMR0L		; BANK 0
    ;CLRF TMR0L            ; !!Bank!! TMR0 - OPTION_REG - TMR0 - OPTION_REG

    MOVLW RAMINI0         ; RAMINI0 ;   b'00100000'  d'032'  " "
    MOVWF FSR0L
	CLRF FSR0H
	clrw
ClearRam
    ;BCF FSR0L,7
	;CLRF INDF0		; BANK 0 (0x020 - 0x07F)
    movwi FSR0++
	;bsf FSR0H,0
    ;CLRF INDF0		; BANK 2 (0x120 - 0x17F)
	;bsf FSR0L,7
    ;CLRF INDF0		; BANK 3 (0x1A0 - 0x1FF)
	;bcf FSR0H,0
    ;CLRF INDF0		; BANK 1 (0x0A0 - 0x0FF)
    ;INCF FSR0L,F
    BTFSS FSR0L,7
    GOTO ClearRam		; LADR_0x0127     ; !!Bank!! 0x0127 - 0x0927
	
    ;BSF PCLATH,3         ; !!Bank Program-Page-Select
    ;CALL UART_INI		; LADR_0x0880     ; !!Bank!! 0x0080 - 0x0880
    ;BCF COMPORT,TXRX          ; !!Bank!! PORTB - TRISB - PORTB - TRISB
    ;BCF PCLATH,3         ; !!Bank Program-Page-Select

	movlb 0
	movlw 1
	movwf PULSE_LEN		; pulse length

    MOVLW INTC_INI
    MOVWF INTCON	; enable interrupt
    movlw 1
    movwf LED_COUNT

MainLoop:

	BANKSEL	PIR4		; BANK 14
	btfsc PIR4,TMR2IF	; time to update LEDs
	call UpdateLEDs

	movlb 0				; BANK 0
	movf DCCSENDL,w		; check for empty buffer
	btfsc STATUS,Z
	call UpdateDCC

	goto MainLoop

; 0x2A - this command is followed by 1 to 4 data bytes. data bytes contain light intensity information for modules with address A to A+3.
;        when A=0, then first data byte is ignored by all slaves, but second data byte is regularly used by slave with address 1 etc.
;        data byte address roll over. It mean for example: when A = F, then first data byte is for address F, next data byte is ignored 
;        as for address 0 and next is used as address 1.
UpdateDCC:
	incf PKT_COUNT,f
	movlw 0x21		; first half - LEDs 1-4
	btfsc PKT_COUNT,0	; is time for 2nd half?
	movlw 0x25		; second half - LEDs 5-8
	movwf DCCSEND1	; set as commend
	movwf TEMP		; prepare XOR
	movlw LED_BUF	; first LED from buffer
	btfsc PKT_COUNT,0	; is time for 2nd half?
	movlw LED_BUF+4	; fifth LED from buffer
	movwf FSR0L
	clrf FSR0H
	moviw FSR0++	; prepare 1st intensity
	movwf DCCSEND2	; set as data
	xorwf TEMP,f	; update XOR
	moviw FSR0++	; prepare 2nd intensity
	movwf DCCSEND3	; set as data
	xorwf TEMP,f	; update XOR
	moviw FSR0++	; prepare 3rd intensity
	movwf DCCSEND4	; set as data
	xorwf TEMP,f	; update XOR
	moviw FSR0++	; prepare 4th intensity
	movwf DCCSEND5	; set as data
	xorwf TEMP,w	; update XOR
	movwf DCCSEND6	; set as XOR
	movlw 6
	movwf DCCSENDL
	return

UpdateLEDs:			; called 64 times per sec. It mean - change led every 8 calls, decrease intensity by 6 each step
	bcf PIR4,TMR2IF	; clear time flag
	movlb 0				; BANK 0
	decfsz LED_COUNT,f
	goto SkipLEDMove
	movlw 8
	movwf LED_COUNT
	movf LED_POS,w
	andlw 0x07		; only 8 leds we have
	btfsc LED_POS,3	; this bit mean direction
	xorlw 0x07		; reverse direction
	addlw LED_BUF	; add base address
	movwf FSR0L
	clrf FSR0H
	movlw 0xFE		; decrease intensity for this step - it will be decreased again in 2nd part
	movwf INDF0		; update in buffer
	incf LED_POS,f	; go to next LED
	incf LED_POS,w		; have no led for pos 1 and 8 :( check for positiom 8
	andlw 0x07		; only 8 leds we have
	btfss STATUS,Z
	goto UpdateLEDsNo8
	incf LED_POS,f	; skip led 8 to led 1 
	;incf LED_POS,f	; skip led 1 to led 2
UpdateLEDsNo8
	movf LED_POS,w
	andlw 0x07		; only 8 leds we have
	btfsc LED_POS,3	; this bit mean direction
	xorlw 0x07		; reverse direction
	addlw LED_BUF	; add base address
	movwf FSR0L
	clrf FSR0H
	movlw 0xFF		; full intensity for this step
	movwf INDF0		; update in buffer
SkipLEDMove:
	movlw 8			; number of LEDs
	movwf COUNT
	movlw LED_BUF	; pointer to buffer
	movwf FSR0L
	clrf FSR0H
DecLEDintensityLoop:
	incf INDF0,w
	btfsc STATUS,Z	; skip if full intensity
	goto SkipDecLEDintensity
	movf INDF0,w
	btfsc STATUS,Z	; skip if dark
	goto SkipDecLEDintensity
	movlw 9
	subwf INDF0,f	; decrease value
	btfss STATUS,C	; check if 6 > INDF0
	clrf INDF0		; yes, then clear intensity at all
SkipDecLEDintensity
	incf FSR0L,f	; next LED
	decfsz COUNT,f	; for all LEDs
	goto DecLEDintensityLoop
    return

;   EEPROM-Data - no eeprom here
	org	SAF_INI

	 dt	" LgoX v"
	 dt	__VERNUM
	 dt	"J.Fucik "
	 dt	(__VERDAY   >> 4)  +0x30
	 dt	(__VERDAY   & 0x0F)+0x30,"/"
	 dt	(__VERMONTH >> 4)  +0x30
	 dt	(__VERMONTH & 0x0F)+0x30,"/"
	 dt	(__VERYEAR  >> 4)  +0x30
	 dt	(__VERYEAR  & 0x0F)+0x30


    End
