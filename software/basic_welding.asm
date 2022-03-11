;;======================================================================;;
;;			analog ONE LIGHT DECODER				;;
;;======================================================================;;
;;									;;
;; Program:         mini_light -- DCC lights decoder			;;
;; Code:            Jindra Fuèík http://www.fucik.name ;
;; Platform:        Microchip PIC10F2xx, 4 Mhz		;;
;; Date:            17.01.2014						;;
;; First release:   25.01.2014						;;
;; LastDate:        09.09.2021						;;
;;									;;
;;======================================================================;;
;
; This program is distributed as is but WITHOUT ANY WARRANTY
; License: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en
;
; Revisions:
; 05.05.2007	Start of writing code (PIC12F629)
; 17.01.2014	Modification for PIC10F32x
; 13.11.2016	Modification for PIC10F206
; 09.09.2021	Modification for Lego


; ----- Definitions

#define		__VERNUM	D'1'
#define		__VERDAY	0x09
#define		__VERMONTH	0x09
#define		__VERYEAR	0x21



		errorlevel -302
		;errorlevel -227
		;errorlevel -306

;       include "p10f200.inc"
       include "p10f206.inc"
        list    st=off

        __CONFIG   _MCLRE_OFF & _CP_OFF & _WDT_ON


; --- Constant values 

FXTAL		equ	D'4000000'


GP_TRIS         equ     b'00001100'		; GP0: speaker, GP1: out led, 
;GP_INI          equ     0x00			; all zero

OPTION_INI	equ   b'10011110'   ; 10010001
                                ; 1-------, Wake-up on Pin Change off
                                ; -0------, weak pullups ON
                                ; --0-----, T0CS source Fosc/4
                                ; ---1----, T0SE edge hi>lo
                                ; ----0---, PSA prescale TMR0
                                ; -----001, PS prescaler 1:4 --> 128 steps = 0.512ms 

;WPUA_INI	equ 0x0A			; weak pull-ups on buttons
;T2CON_INI 	equ	0x0D			; Prescaler 1:4, Timer 2 enable, Postscaler 1:2
;PR2_INI		equ	0xF9			; 249 dec = 250 instruction = 1ms with prescaler
;T2CON_INI 	equ	0x05			; Prescaler 1:4, Timer 2 enable
;PR2_INI		equ	0x7C			; 124 dec = 125 instruction = 0,5ms with prescaler

#define		LED		GPIO,1			; LED output



; ----- Variables

        cblock  0x10
FLAGS				; Flags for blinking
RANDOM0				; Random shift register
RANDOM1
RANDOM2
RANDOM3
ENT1
ENT2
PWM1CNT		; Light intensity
STEPS		; Steps to start up
LTIME		; Time for each step
TRYES		; Number of retryes
;LONG1		; Time between retryes
LONG2		; Time between retryes

TEMP
COUNTPWM

CV10
CV11
        endc


; --- Flags
						; FLAGS
#define		SPKOUT		FLAGS,0		; speaker phase
#define		LEDOUT		FLAGS,1		; Flash phase
#define		BLINKING	FLAGS,2
#define		LIGHTING	FLAGS,3		; Status of lights 1=starting or lighting, 0=off
#define		LONGDELAY		FLAGS,4		; No CV finded

SPKXOR		EQU		0x01			; flag for speaher

; --------------- Program Section --------------------------------------


		org	0x000

start
        movwf   OSCCAL          ;
        movwf   FLAGS            ;

		btfsc	STATUS,NOT_TO		; After timeout work normally
		goto	MixRandom		; before timeout go to mix random numbers
		movlw	OPTION_INI
        option                  ;
        movlw   GP_TRIS			;
        tris    GPIO            ; GP3 input, all others outputs
        clrf    GPIO            ; set output latches to '0'
		bcf		CMCON0,CMPON	; disable comparator


		call	LoadCV			; load CV values page is 16 rows!!!

		;movf	ENT1,w
		;movwf	RANDOM0
		;movf	ENT2,w
		;movwf	RANDOM1
		;bsf		RANDOM0,0		; to be sure RANDOM shift reg. isn't zero
		
		clrf	FLAGS
		bsf		LIGHTING		; Turn on
		bsf		BLINKING		; phase of preparation
		call	GetRand
		movf	RANDOM0,w
		andwf	CV10,w
		movwf	STEPS			; number of blinks
		incf	STEPS,f
		decf	CV11,w			; this step is last step of PWM
		movwf	COUNTPWM
		movwf	PWM1CNT
		movlw	1
		movwf	LTIME			; last time
		movf	ENT1,w			; number of tryes
		movwf	TRYES
		bcf		LONGDELAY
		;clrf	LONG1
		call	GetRand
		movf	RANDOM0,w
		andlw	0x7F
		movwf	LONG2
		incf	LONG2,f


; ----------------------------------------------------------------------
MainLoop:
		clrwdt
		btfsc	TMR0,7		; time to next PWM step?
		call	DoPWM			; yes, next step

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
MixRandom
		movlw	OPTION_INI
        option                  ;
        movlw   GP_TRIS			;
        tris    GPIO            ; GP3 input, all others outputs
        clrf    GPIO            ; set output latches to '0'

		;bsf		GPIO,2
		
		call	LoadCV			; load CV values page is 16 rows!!!

		movf	ENT1,w
		movwf	RANDOM0
		movf	ENT2,w
		movwf	RANDOM1
		bsf		RANDOM0,0		; to be sure RANDOM shift reg. isn't zero
		movf	FLAGS,w
		movwf	RANDOM2
		
RotateRand:
		swapf	RANDOM0,w		; RANDOM shift register
		xorwf	RANDOM1,w
		movwf	TEMP
		rrf		TEMP,w
		rlf		RANDOM3,f
		rlf		RANDOM2,f
		rlf		RANDOM1,f
		rlf		RANDOM0,f
		goto	RotateRand		; this loop look like never ending, but it is terminated by WDT timeout

; ----------------------------------------------------------------------


DoPWM:
		clrf TMR0
		movf FLAGS,w				; use pre calculated value
		;andlw 0x01
		movwf GPIO

		btfss LIGHTING				; is requested LED on???
		goto ExtraLongDelay

		btfsc LONGDELAY
		goto LongDelay

		incf COUNTPWM,f				; increase global pwm counter
		;decf ENT1,w
		movf CV11,w					; CV10 = number of PWM steps
		xorwf COUNTPWM,w			; steps forp PWM, last step mean allways on
		btfss STATUS,Z
		goto CalcPWM
		clrf COUNTPWM

		decfsz LTIME,f				; time for lighting
		goto CalcPWM
		
		btfss BLINKING				; is in 1st phase
		goto WeldingStep
		movf STEPS,w				; check if blinking
		btfss STATUS,Z
		goto NextBlink
		bcf		BLINKING			; stop blinking
		call	GetRand
		movf	RANDOM0,w
		movwf	STEPS			; number of blinks
		goto	WeldingStep

NextBlink
		movlw	SPKXOR
		xorwf	FLAGS,f
		movf	CV11,w
		movwf	PWM1CNT
		call	GetRand
		movf	RANDOM0,w
		movwf	LTIME				; delay for first step
		decf	STEPS,f
		btfss	STEPS,0				; odd/even light/dark
		clrf	PWM1CNT
		movlw	0xC0
		btfss	STEPS,0				; odd/even light/dark
		iorwf	LTIME,f				; long delay
		movlw	0x0F
		btfsc	STEPS,0				; odd/even light/dark
		andwf	LTIME,f				; short light
		incf	LTIME,f
		call	GetRand			; get random number
		movf	RANDOM0,w
		bcf		STATUS,C			; clear C flag (for following rrf)
		andlw	0x03				; value 0-3
		btfsc	STATUS,Z			; if random = 0, then half light
		rrf		PWM1CNT,f			; half intensity
		goto	CalcPWM

WeldingStep
		movf STEPS,w				; check if blinking
		btfss STATUS,Z
		goto NextStep
		btfsc ENT2,0				; if ENT2=1, then never start up
		goto NextStep
		bcf LEDOUT				; keep led on
		bcf SPKOUT				; keep led on
		bsf	LONGDELAY
		retlw 0

		
NextStep
		movf	CV11,w
		movwf	PWM1CNT
		decf	STEPS,f
		call	GetRand
		movf	RANDOM0,w
		andlw	0x3F
		movwf	LTIME				; delay for first step
		movlw	0x0F
		btfss	STEPS,0				; odd/even light/dark
		andwf	LTIME,f				; very short delay
		incf	LTIME,f
		btfss	STEPS,0				; odd/even light/dark
		clrf	PWM1CNT
		call	GetRand				; get random number
		movf	RANDOM0,w
		bcf		STATUS,C			; clear C flag (for following rrf)
		andlw	0x03				; value 0-3
		btfsc	STATUS,Z			; if random = 0, then half light
		rrf		PWM1CNT,f			; half intensity
		call	GetRand				; get random number
		movf	RANDOM0,w
		bcf		STATUS,C			; clear C flag (for following rrf)
		andlw	0x03				; value 0-3
		btfsc	STATUS,Z			; if random = 0, then half light
		rrf		PWM1CNT,f			; half intensity
		movlw	SPKXOR
		xorwf	FLAGS,f

		
CalcPWM
		bcf LEDOUT
		movf PWM1CNT,w
		subwf COUNTPWM,w				; w = COUNTPWM - PWM1CNT (if PWM1CNT > COUNTPWM then carry is set)
		btfss STATUS,C
		bsf LEDOUT				; yes, turn it on

		retlw 0

LongDelay
		bcf LEDOUT
		bcf SPKOUT
		decfsz TEMP,f
		retlw 0
		decfsz LONG2,f
		retlw 0
		bcf LONGDELAY
		bsf		BLINKING		; phase of preparation
		call	GetRand
		movf	RANDOM0,w
		andwf	CV10,w
		movwf	STEPS			; number of blinks
		incf	STEPS,f
		decf	CV11,w			; this step is last step of PWM
		movwf	COUNTPWM
		movwf	PWM1CNT
		movlw	1
		movwf	LTIME			; last time
		call	GetRand
		movf	RANDOM0,w
		andlw	0x3F
		movwf	LONG2
		btfsc	ENT2,1
		retlw 0
		decfsz TRYES,f
		retlw 0
		bcf	LIGHTING
		call	GetRand
		movf	RANDOM0,w
		andlw	0x07
		movwf	LONG2
		incf	LONG2,f
		clrf	COUNTPWM
		clrf	TEMP
		retlw 0	

ExtraLongDelay
		decfsz TEMP,f
		retlw 0
		decfsz COUNTPWM,f
		retlw 0
		decfsz LONG2,f
		retlw 0
		bsf	LIGHTING
		bsf		BLINKING		; phase of preparation
		call	GetRand
		movf	RANDOM0,w
		andwf	CV10,w
		movwf	STEPS			; number of blinks
		incf	STEPS,f
		decf	CV11,w			; this step is last step of PWM
		movwf	COUNTPWM
		movwf	PWM1CNT
		movlw	1
		movwf	LTIME			; last time
		movf	ENT1,w			; number of tryes
		movwf	TRYES
		bcf		LONGDELAY
		;clrf	LONG1
		call	GetRand
		movf	RANDOM0,w
		andlw	0x3F
		movwf	LONG2
		retlw 0
GetRand:
		swapf	RANDOM0,w		; RANDOM shift register
		xorwf	RANDOM1,w
		movwf	TEMP
		rrf		TEMP,w
		rlf		RANDOM3,f
		rlf		RANDOM2,f
		rlf		RANDOM1,f
		rlf		RANDOM0,f
		;andlw	0x07
		retlw 0

;---------------------------------------------------------------------------
LoadCV:
		call LoadENT1S
		movwf ENT1
		call LoadENT2S
		movwf ENT2
		call LoadCV10S
		movwf CV10
		call LoadCV11S
		movwf CV11
		retlw 0

LoadENT1S:				; call can be done only o short address
		goto LoadENT1	; goto cam be also to long
LoadENT2S:				; call can be done only o short address
		goto LoadENT2	; goto cam be also to long
LoadCV10S:				; call can be done only o short address
		goto LoadCV10	; goto cam be also to long
LoadCV11S:				; call can be done only o short address
		goto LoadCV11	; goto cam be also to long


	IFDEF __10F322
		org	0x01F0
	ENDIF

	IFDEF __10F320
		org	0x00F0
	ENDIF

	IFDEF __10F200
		org	0x00F0
	ENDIF

	IFDEF __10F206
		org	0x01F0
	ENDIF

LoadENT1:
		retlw 0x03
LoadENT2:
		retlw 0x00
LoadCV10:
		retlw 0x07	; CV10 = number of PWM steps
LoadCV11:
		retlw 0x08	; time for first blink



	end

