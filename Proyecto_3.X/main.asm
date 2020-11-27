; Assembly source line config statements

#include "p16f887.inc"

; CONFIG1
; __config 0x20D4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0x3FFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

 
VARS UDATA
 ;----------------------------- USO DEL ADC --------------------------------------------
SERVO0           RES 1
SERVO1            RES 1
SERVO2           RES 1	    ;Usadas para recuperar el valor del canal del ADC. 
ADC_CONT     RES 1	    ;Representa el canal que se esta leyendo.
 ;------------------------------- Generales ----------------------------------------------
MODO              RES 1	     ;Reservada para indicar el estado que mueve los servos
CONT1              RES 1	    ;Para los delays
 ;-------------------------- Control de PWM generado por el timer-----------------------
ROTF		RES 1	    ;Variable que ajusta el valor del ADC apartir de rotacion de bits
DUTY_CYCLE	 RES 1	    ;Establece el valor del tiempo activo del pulso.
CICLO		 RES 1	    ;Lleva control del parte del periodo actual 
 ;-----------------------------Interrupcion-----------------------------------------------
TEMP_W	RES 1
TEMP_STATUS	RES 1
TEMP_FSR	RES 1 
 
RES_VECT  CODE    0x0000		 ; processor reset vector
    GOTO    START			 ; go to beginning of program

INT_VECT    CODE 0x0004
    
SAVE:
    MOVWF	TEMP_W 
    SWAPF	STATUS,W
    MOVWF              TEMP_STATUS
    SWAPF	FSR,W
    MOVWF	TEMP_FSR

TMR0_INT:
    BTFSS	INTCON,T0IF
    GOTO		ADC_INT
    
    BCF		INTCON,T0IF
    MOVF		CICLO,W
    ADDWF	PCL, F
    GOTO		TIEMPO_ACTIVO 
    GOTO		TIEMPO_INACTIVO
    CLRF		CICLO

TIEMPO_ACTIVO
    BSF		PORTC, RC0                 ;Se mantiene encendido el RC0 el tiempo definido 
    MOVF		DUTY_CYCLE,W	      ;Por esta variable. 
    CLRF		TMR0
    SUBWF	TMR0,F		    
    INCF		CICLO,F		    ;La siguiente entrada se ejecuta el TIEMPO_INACTIVO
    GOTO		ADC_INT
      
TIEMPO_INACTIVO
    BCF		PORTC,RC0
    CLRF		TMR0		  ;En la siguiente interrupcion se ejecuta nuevamente TIEMPO_ACTIVO
    INCF		CICLO,F  
    
ADC_INT
    BTFSS	PIR1, ADIF 
    GOTO		LOAD		    ;Si no se encuentra activada se sale de la interrupcion
    
    BCF		PIR1, ADIF
    MOVF		ADC_CONT,W
    ADDWF	PCL,F   
    GOTO		LEC_ADC0
    GOTO		LEC_ADC1
    GOTO		LEC_ADC2
    CLRF		ADC_CONT
    
LEC_ADC0
    MOVF		ADRESH, W
    MOVWF	SERVO0
    BCF		ADCON0,CHS1
    BSF		ADCON0,CHS0		;Se selecciona el canal 1
    GOTO		INICIAR_CONV
    
LEC_ADC1
    MOVF		ADRESH, W
    MOVWF	SERVO1
    BSF		ADCON0,CHS1
    BCF		ADCON0,CHS0		;Se selecciona el canal 2
    GOTO		INICIAR_CONV
   
LEC_ADC2
    MOVF		ADRESH,W
    MOVWF	SERVO2
    BCF		ADCON0,CHS1
    BCF		ADCON0,CHS0		;Se selecciona el canal 0		
    
INICIAR_CONV
    INCF		ADC_CONT,F
    BSF		ADCON0,GO		;Inicia la conversion 
    
LOAD:
    SWAPF	TEMP_STATUS,W
    MOVWF	STATUS
    SWAPF	TEMP_FSR,W
    MOVWF	FSR
    SWAPF	TEMP_W,F
    SWAPF	TEMP_W,W
    RETFIE
    
MAIN_PROG CODE		 ; let linker place main program
 
START
    BCF		STATUS, IRP		;Banco 0 & 1 en el direccionamiento 
    BSF		STATUS, RP1
    BSF		STATUS, RP0		;-----------------Banco 3--------------------
    
    MOVLW	B'00000111'
    MOVWF	ANSEL
    CLRF		ANSELH 
    
    BCF		OPTION_REG, T0CS    ;Fuente de reloj FOSC/4
    BCF		OPTION_REG, PSA       ;Prescaler asignado al TMR0
    BCF		OPTION_REG, PS2
    BSF		OPTION_REG,PS1
    BSF		OPTION_REG,PS0       ;PRESCALER DE 1:16 
    
    BCF		STATUS,RP1		;--------------------BANCO 1---------------
    MOVLW	.255
    MOVWF	TRISA
    CLRF		TRISC     
    
    ;------------- INTERRUPCIONES -------------------------------------------------
    BSF		INTCON,GIE 
    BSF		INTCON, PEIE
      
    BSF		PIE1, ADIE			;Interrupcion del ADC 
    ;---------------------------------------------------------------------------------------
    MOVLW	.255 
    MOVWF	PR2				;Periodo de  4.09 ms aproximadamente
    CLRF		ADCON1 
    
    BCF		STATUS,RP0		    ;-----------------Banco 0------------------
    
    BCF		INTCON, T0IF
    CLRF		TMR0
    BSF		INTCON, T0IE
    
    MOVLW	B'00001100' 
    MOVWF	CCP1CON		
    MOVWF	CCP2CON		;Modo PWM activo  
    
    MOVLW	B'0000110'		;TMR2 encendido con prescale de 1:16 
    MOVWF	T2CON			;& postscaler de 1
    
    MOVLW	B'01000001'		;ADC con FOSC/8,  canal 0 
    MOVWF	ADCON0		;& ADC encendido.
    
    CLRF		SERVO0
    CLRF		SERVO1
    CLRF		SERVO2
    CLRF		ADC_CONT
    CLRF		 CICLO
    
    BSF		ADCON0, GO			;Comienza conversion
    
LOOP:
    MOVLW	SERVO0
    MOVWF	FSR			
     CALL		AJUSTE_DC
    MOVF		ROTF,W
    MOVWF	CCPR1L
    
    MOVLW	SERVO1
    MOVWF	FSR
    CALL		AJUSTE_DC
    MOVF		ROTF, W
    MOVWF	CCPR2L
    
    MOVLW	SERVO2
    MOVWF	FSR
    CALL		AJUSTE_DC
    MOVF		ROTF,W
    MOVWF	DUTY_CYCLE

    GOTO		LOOP       
;------------------------------Sub Rutinas-----------------------------------------------   
AJUSTE_DC:
    BCF	STATUS, IRP	    ;Direccionamiento apunta banco 0 Y 1
    
    BCF	STATUS,C	    ;Se asegura que que Carry tenga valor de 0
    RRF	INDF, W	    ;Se restringue el rango de 0 a 255 -> 0 a 127
    ADDLW   .31		    ;Se suma 31 para  que el resultado este entre 31 Y 158 los cuales son
			    ;un ancho de pulso de entre0.496 mS y 2.512 ms 		
   MOVWF    ROTF
   RETURN
   END