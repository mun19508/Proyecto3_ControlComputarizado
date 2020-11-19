#include "p16f887.inc"

; CONFIG1
; __config 0xE0D4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

PR_VAR		    UDATA
;------------------------------------Generales-------------------------------------------		    
ANTIREB	    RES 1			;Boton
CONT1		    RES 1			;Delay
;--------------------------------------ADC-----------------------------------------------		    
SERVO0	    RES 1
SERVO1		    RES 1
SERVO2	    RES 1			;Para mostrar el ciclo de trabajo de los PMW
CONT_ADC	    RES 1			;Para llevar  registro de que canal analogico toca.
;-----------------------------------Interrupcion------------------------------------------	    
TEMP_STATUS	    RES 1
TEMP_W	    RES 1
  
	
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program
    
;************************************Interrupcion********************************
ISR_VECT   CODE    0X0004
SAVE:					;Sirve para guardar el valor actual de:
    MOVWF	TEMP_W		;"W"
    SWAPF	STATUS, W
    MOVWF	TEMP_STATUS		;& STATUS
ISR:
    BTFSS	PIR1, ADIF
    GOTO		LOAD
ADC:    
    BCF		PIR1, ADIF
    MOVF		CONT_ADC, W
    ADDWF	PCL, F
    GOTO		SERVO0_ADC
    GOTO		SERVO1_ADC
    GOTO		SERVO2_ADC
    CLRF		CONT_ADC
SERVO0_ADC:       
    MOVFW	ADRESH
    MOVWF	SERVO0
    INCF		CONT_ADC, F
    BSF		ADCON0, 2
    GOTO		INICIAR_CONV			
SERVO1_ADC:       
    MOVFW	ADRESH
    MOVWF	SERVO1
    INCF		CONT_ADC, F
    BSF		ADCON0, 3
    BCF		ADCON0, 2
    GOTO		INICIAR_CONV			
SERVO2_ADC:       
    MOVFW	ADRESH
    MOVWF	SERVO2
    INCF		CONT_ADC, F
    BCF		ADCON0, 3
    GOTO		INICIAR_CONV			
INICIAR_CONV:
    NOP
    NOP
    NOP
    NOP
    NOP
    BSF		ADCON0, GO
LOAD:					;Se recupera el valor de:
    SWAPF	TEMP_STATUS, W
    MOVWF	STATUS		;STATUS
    SWAPF	TEMP_W, F
    SWAPF	TEMP_W, W		;& de "W"
    BSF		INTCON, GIE		;se habilitan la interrupciones globales
    RETFIE
;----------------------------------------------------------------------------------------- 
MAIN_PROG CODE			; let linker place main program
START
    BSF		 STATUS, 6
    BSF		STATUS, 5		;--------------------Banco 3----------------
    CLRF		ANSEL
    BSF		ANSEL, 0		;Se habilita el analogico en A0
    BSF		ANSEL, 1		;Se habilita el analogico en A1
    BSF		ANSEL, 2		;Se habilita el analogico en A2
    BCF		STATUS, 6		;-------------------Banco 1------------------
    MOVLW	.255
    MOVWF	TRISA			;Puerto A como entrada.
    CLRF		TRISC			;Salida: Señales PMW. 
    BCF		ADCON1, ADFM		;Justificado a la izquierda.
    BCF		ADCON1, VCFG1		;Ref: VSS.
    BCF		ADCON1, VCFG0		;Ref: VDD.
    BSF		INTCON, GIE		;Se habilitaron la interrupciones globales,
    BSF		INTCON, PEIE		;perifericas,
    BSF		PIE1, ADIE		;y la interrupciones por el A/D.
    BCF		STATUS, 5		;-------------------Banco 0------------------
   MOVLW	B'01000001'		;Fosc/8, ANS0 & conversion activada.
   MOVWF	ADCON0
   BCF		PIR1, ADIF		;Se apaga la bandera del A/D.
   CLRF		PORTA			;Se limpian los puertos.
   CLRF		PORTC
   CLRF		TEMP_STATUS	
   CLRF		TEMP_W	
   CLRF		SERVO0
   CLRF		SERVO1
   CLRF		SERVO2
   CLRF		CONT_ADC
   CLRF		ANTIREB
   CLRF		CONT1
   BSF		ADCON0, GO
LOOP: 
	
    GOTO		LOOP
;--------------------------------------RUTINAS-----------------------------------------
DELAY_4US				;DELAY DE  4us (supuestamente)
    BCF		STATUS,5
    BCF		STATUS,6
    MOVLW	 .128
    MOVWF	CONT1
    DECFSZ	CONT1, F	
    GOTO		$-1 
   RETURN    
    END