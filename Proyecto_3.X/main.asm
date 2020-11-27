; Assembly source line config statements

#include "p16f887.inc"

; CONFIG1
; __config 0x20D4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0x3FFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

 
VARS UDATA
;------------------------------- Generales ----------------------------------------------
MODO              RES 1	     ;Reservada para indicar el estado que mueve los servos
CONT1              RES 1	    ;Para los delays
ANTIREB        RES 1
 ;----------------------------- USO DEL ADC --------------------------------------------
SERVO0           RES 1
SERVO1            RES 1
SERVO2           RES 1	    ;Usadas para recuperar el valor del canal del ADC. 
ADC_CONT     RES 1	    ;Representa el canal que se esta leyendo.
;------------------------------Envio de datos---------------------------------------------
;------ADC----------  
PULSO0			RES 1	  
PULSO1			RES 1
PULSO2			RES 1
;-----Generales------  
CONT_ENVIO		RES 1
VALOR_B		RES 1	  
UNIDADES_TX		RES 1	  
DECENAS_TX		RES 1
CENTENAS_TX		RES 1
;-----Servo 0-------  
TX_UNIDADES_S0	RES 1	  ;Se separan en bloques para aprovechar direccionamiento indirecto
TX_DECENAS_S0	RES 1
TX_CENTENAS_S0	RES 1
;-----Servo 1-------     
TX_UNIDADES_S1	RES 1   
TX_DECENAS_S1	RES 1
TX_CENTENAS_S1	RES 1
;-----Servo 2-------     
TX_UNIDADES_S2	RES 1
TX_DECENAS_S2	RES 1
TX_CENTENAS_S2	RES 1
;-----------------------------------Recepcion de datos-----------------------------------
;---CONVERSION---
CIFRA_CENTENA	RES 1
CIFRA_DECENA		RES 1
CIFRA_UNIDAD	RES 1
;-----Generales------  
CONT_RECP		RES 1	
VALOR_BCD		RES 1		
SERVO_V		RES 1		
SERVO_BIN		RES 1		
;-----Servo 0-------  
RX_CENTENAS_S0	RES 1
RX_DECENAS_S0	RES 1
RX_UNIDADES_S0	RES 1	  	
;-----Servo 1-------        
RX_CENTENAS_S1	RES 1
RX_DECENAS_S1	RES 1
RX_UNIDADES_S1	RES 1
;-----Servo 2-------     
RX_CENTENAS_S2	RES 1
RX_DECENAS_S2	RES 1
RX_UNIDADES_S2	RES 1
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
    GOTO		RX_INT
    
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
    GOTO		RX_INT
      
TIEMPO_INACTIVO
    BCF		PORTC,RC0
    CLRF		TMR0		  ;En la siguiente interrupcion se ejecuta nuevamente TIEMPO_ACTIVO
    INCF		CICLO,F  

RX_INT:
     BTFSS	PIR1, RCIF
    GOTO		ADC_INT
    
     MOVF	RCREG,W
    MOVWF	VALOR_BCD
    
    MOVF		SERVO_V, W
    SUBLW	.255		;Se comprueba si el valor anterior fue para seleccion de  
    BTFSC	STATUS, Z	;servo o no 
    GOTO		SERVO_SELECT	;se selecciona el servo
    
    INCF		VALOR_BCD, F
    MOVF		SERVO_V,W
    ADDWF	PCL,F 
    GOTO		STORE_S0
    GOTO		STORE_S1
    GOTO		STORE_S2
STORE_S0
    MOVF		VALOR_BCD,W 
    MOVLW	RX_CENTENAS_S0
    ADDWF	CONT_RECP, W 
    MOVWF	FSR 
    MOVLW	.48 
    SUBWF	VALOR_BCD,W
    MOVWF	INDF 
    INCF		CONT_RECP,F ;
    GOTO		FIN
STORE_S1
     MOVF	VALOR_BCD,W 
    MOVLW	RX_CENTENAS_S1
    ADDWF	CONT_RECP, W 
    MOVWF	FSR 
    MOVLW	.48
    SUBWF	VALOR_BCD,W
    MOVWF	INDF 
    INCF		CONT_RECP,F ;
    GOTO		FIN
STORE_S2
     MOVF	VALOR_BCD,W 
    MOVLW	RX_CENTENAS_S2
    ADDWF	CONT_RECP, W 
    MOVWF	FSR 
    MOVLW	.48 
    SUBWF	VALOR_BCD,W
    MOVWF	INDF 
    INCF		CONT_RECP,F ;
FIN 
    MOVF		CONT_RECP,W
    SUBLW	 .2
    BTFSS	STATUS,Z  ;SI CONTADOR_RC ES 3 NTONCES SE SALTA LA INSTRUCCION
    GOTO		ADC_INT
    MOVLW	.255
    CLRF		CONT_RECP
    MOVWF	SERVO_V ;SE ACABA LA INSTRUCCION A LA ESPERA DE UNA NUEVA
    GOTO		ADC_INT
    ;------------------------------------------------------------------------------------------------
SERVO_SELECT
    MOVF		VALOR_BCD,W
    SUBLW	.73		;Se verifica si el valor es I
    BTFSS	STATUS, Z	;Si no lo es se aplica el siguiente criterio
    GOTO		SERVO1_S
    MOVLW	.0 
    MOVWF	SERVO_V	
    GOTO		ADC_INT
    
SERVO1_S
    MOVF		VALOR_BCD,W
    SUBLW	.68		;Se verifica si es D
    BTFSS	STATUS,Z
    GOTO		SERVO2_S
    MOVLW	.1 
    MOVWF	SERVO_V
    GOTO		ADC_INT 
    ;------------------------------------------------------------
SERVO2_S
    MOVF		VALOR_BCD,W
    SUBLW	.67		;Se verifica si es C
    BTFSS	STATUS,Z
    GOTO		NO_VALIDO
    MOVLW	.2
    MOVWF	SERVO_V
    GOTO		ADC_INT
    ;--------------------------------------------------------------
NO_VALIDO
    MOVLW	.255
    MOVWF	SERVO_V 
    
ADC_INT:
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
    
ENVIAR:
    BTFSS   PIR1,TXIF		    ;Se verifica si no esta ocupado RETURN
    RETURN
   
    ;--------------------------Se selecciona que dato enviar -------------------------------
    MOVF		CONT_ENVIO,W
   ADDWF	PCL,F
    GOTO		CENTENA_SERVO0
    GOTO		DECENA_SERVO0
    GOTO		UNIDAD_SERVO0
    GOTO		COMA
    GOTO		CENTENA_SERVO1
    GOTO		DECENA_SERVO1
    GOTO		UNIDAD_SERVO1
    GOTO		COMA
    GOTO		CENTENA_SERVO2
    GOTO		DECENA_SERVO2
    GOTO		UNIDAD_SERVO2
    GOTO		FINAL
    CLRF		CONT_ENVIO

CENTENA_SERVO0
    MOVF		TX_CENTENAS_S0,W
    GOTO		ENVIAR_SUMA
DECENA_SERVO0
    MOVF		TX_DECENAS_S0,W
    GOTO		ENVIAR_SUMA
UNIDAD_SERVO0
    MOVF		TX_UNIDADES_S0,W
    GOTO		ENVIAR_SUMA
CENTENA_SERVO1
    MOVF		TX_CENTENAS_S1,W
    GOTO		ENVIAR_SUMA
DECENA_SERVO1
    MOVF		TX_DECENAS_S1,W
    GOTO		ENVIAR_SUMA
UNIDAD_SERVO1
    MOVF		TX_UNIDADES_S1,W
    GOTO		ENVIAR_SUMA
CENTENA_SERVO2
    MOVF		TX_CENTENAS_S2,W
    GOTO		ENVIAR_SUMA
DECENA_SERVO2
    MOVF		TX_DECENAS_S2,W
    GOTO		ENVIAR_SUMA
UNIDAD_SERVO2    
    MOVF		TX_UNIDADES_S2,W
    GOTO		ENVIAR_SUMA
COMA
    MOVLW	.44				    ;En ASCII es 44 
    GOTO		ENVIAR_PC
FINAL
    MOVLW	.10				    ;En ASCII es 10
    GOTO		ENVIAR_PC
ENVIAR_SUMA
    ADDLW	.48				    ;Se pasa de BCD -> ASCII
ENVIAR_PC
    MOVWF	TXREG
    INCF		CONT_ENVIO, F
    RETURN
    
MAIN_PROG CODE		 ; let linker place main program
 
START
    BCF		STATUS, IRP		;Banco 0 & 1 en el direccionamiento 
    BSF		STATUS, RP1
    BSF		STATUS, RP0		;-----------------Banco 3--------------------
    
    MOVLW	B'00000111'
    MOVWF	ANSEL
    CLRF		ANSELH 
    
    BCF		OPTION_REG, T0CS	;Fuente de reloj FOSC/4
    BCF		OPTION_REG, PSA	;Prescaler asignado al TMR0
    BCF		OPTION_REG, PS2
    BSF		OPTION_REG,PS1
    BSF		OPTION_REG,PS0	;Prescaler de 1:16 
     BCF		STATUS,RP1		;--------------------BANCO 1---------------
    MOVLW	.255
    MOVWF	TRISA
    CLRF		TRISC     
    CLRF		TRISD
  ;---------------------------INTERRUPCIONES ----------------------------------------
    BSF		INTCON,GIE 
    BSF		INTCON, PEIE     
    BSF		PIE1, ADIE			;Interrupcion del ADC 
    BSF		PIE1, RCIE			;& recepcion EUSART activada.
    ;---------------------------------------------------------------------------------------
    MOVLW	.255 
    MOVWF	PR2				;Periodo de  4.09 ms aproximadamente
    CLRF		ADCON1 
;----------------------------------  EUSART---------------------------------------------
     BCF		TXSTA,TX9	                  ;Transmicion de 8 bits.
     BSF		TXSTA, TXEN	                  ;Bit de transmicion activo.
     BCF		TXSTA, SYNC	                  ;Modo Asincronico.
     BSF		TXSTA, BRGH	                  ;Modo de alta velocidad.
     MOVLW	.25		                  ;9615 de Baudrate
     MOVWF	SPBRG
    BCF		STATUS,RP0		    ;-----------------Banco 0------------------
    BSF		RCSTA, SPEN
    BCF		RCSTA, RX9
    BSF		RCSTA, CREN
;----------------------------Interrupciones pt2-----------------------------------------    
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
    CLRF		MODO
    CLRF		SERVO0
    CLRF		SERVO1
    CLRF		SERVO2
    CLRF		ADC_CONT
    CLRF		CONT_ENVIO
    CLRF		 CICLO
   CLRF		VALOR_B	
   CLRF		CIFRA_CENTENA	
    CLRF		CIFRA_DECENA	
    CLRF		CIFRA_UNIDAD
   CLRF		CONT_RECP
    CLRF		VALOR_BCD
    CLRF		SERVO_V
    CLRF		SERVO_BIN
   CLRF		UNIDADES_TX		  
   CLRF		DECENAS_TX		
   CLRF		CENTENAS_TX		
   CLRF		TX_UNIDADES_S0	
   CLRF		TX_DECENAS_S0	
   CLRF		TX_CENTENAS_S0	
   CLRF		TX_UNIDADES_S1	
   CLRF		TX_DECENAS_S1	
   CLRF		TX_CENTENAS_S1	
   CLRF		TX_UNIDADES_S2	
   CLRF		TX_DECENAS_S2	
   CLRF		TX_CENTENAS_S2	
   CLRF		RX_CENTENAS_S0	
    CLRF		RX_DECENAS_S0	
    CLRF		RX_UNIDADES_S0
   CLRF		RX_CENTENAS_S1	
    CLRF		RX_DECENAS_S1	
    CLRF		RX_UNIDADES_S1
   CLRF		RX_CENTENAS_S2	
    CLRF		RX_DECENAS_S2	
    CLRF		RX_UNIDADES_S2

    BSF		ADCON0, GO			;Comienza conversion
    
LOOP:
    MOVF		PORTA,W
    MOVWF	ANTIREB
    BTFSS	ANTIREB, 3
    GOTO		ESTADO_ACTUAL
    CALL		DELAY
    BTFSC	 PORTA,3
    GOTO		$-1
ESTADO_ACTUAL:   
    BTFSS	MODO, 1
    GOTO		MANUAL
    GOTO		COMPUTADORA
COMPUTADORA:    
    BSF		MODO,1
    BSF		PORTD, 1
    BCF		PORTD, 0
    MOVLW	RX_CENTENAS_S0
    MOVWF	FSR
    CALL		CONV_BIN
    MOVF		SERVO_BIN,W
    MOVWF	PULSO0
    
    MOVLW	RX_CENTENAS_S1
    MOVWF	FSR
    CALL		CONV_BIN
    MOVF		SERVO_BIN,W
    MOVWF	PULSO1
    
    MOVLW	RX_CENTENAS_S2
    MOVWF	FSR
    CALL		CONV_BIN
    MOVF		SERVO_BIN,W
    MOVWF	PULSO2
    
    GOTO		MUESTRA_ENVIO
MANUAL:
    BCF		MODO, 1
    BSF		PORTD, 0
    BCF		PORTD, 1
    MOVF		SERVO0,W
    MOVWF	PULSO0
    MOVF		SERVO1,W
    MOVWF	PULSO1
    MOVF		SERVO2,W
    MOVWF	PULSO2
MUESTRA_ENVIO:    
    MOVLW	PULSO0
    MOVWF	FSR			
     CALL		AJUSTE_DC
    MOVF		ROTF,W
    MOVWF	CCPR1L
    
    MOVLW	PULSO1
    MOVWF	FSR
    CALL		AJUSTE_DC
    MOVF		ROTF, W
    MOVWF	CCPR2L
    
    MOVLW	PULSO2
    MOVWF	FSR
    CALL		AJUSTE_DC
    MOVF		ROTF,W
    MOVWF	DUTY_CYCLE
;------------------------------Conversion de datos---------------------------------------
    MOVF		PULSO0, W
    MOVWF	VALOR_B
    MOVLW	TX_UNIDADES_S0
    MOVWF	FSR
    CALL		CONV_BCD
    MOVF		PULSO1,W
    MOVWF	VALOR_B
    MOVLW	TX_UNIDADES_S1
    MOVWF	FSR
    CALL		CONV_BCD
    MOVF		PULSO2,W
    MOVWF	VALOR_B
    MOVLW	TX_UNIDADES_S2
    MOVWF	FSR
    CALL		CONV_BCD
    
    CALL		ENVIAR
    GOTO		LOOP       
;------------------------------Sub Rutinas-----------------------------------------------   
CONV_BIN
    MOVF		INDF, W
    MOVWF	CIFRA_CENTENA
    INCF		FSR,F
    MOVF    	INDF,W
    MOVWF	CIFRA_DECENA
    INCF		FSR,F
    MOVF    	INDF,W
    MOVWF	CIFRA_UNIDAD
    
    CLRF		SERVO_BIN
    
CONV_CENTENAS
    MOVLW	.100
    DECFSZ	CIFRA_CENTENA,F
    GOTO		SUMAR_CENTENAS
    GOTO		CONV_DECENAS
SUMAR_CENTENAS
    ADDWF	SERVO_BIN, F
    GOTO		CONV_CENTENAS

CONV_DECENAS
    MOVLW	.10
    DECFSZ	CIFRA_DECENA,F
    GOTO		SUMAR_DECENAS
    GOTO		CONV_UNIDADES
SUMAR_DECENAS
    ADDWF	SERVO_BIN, F
    GOTO		CONV_DECENAS

CONV_UNIDADES
    MOVLW	.1
    DECFSZ	CIFRA_UNIDAD,F
    GOTO		SUMAR_UNIDADES
    RETURN
SUMAR_UNIDADES
    ADDWF	SERVO_BIN, F
    GOTO		CONV_UNIDADES
  
CONV_BCD
    CLRF		UNIDADES_TX		;se obtienen los valores de las centenas,
   CLRF		DECENAS_TX		;decenas y unidades independientemente
   CLRF		CENTENAS_TX		;del eje con el que se trabaje.
CENTENAS:
   MOVLW	.100			;Se le resta un valor, en este caso .100,
   SUBWF	VALOR_B, W		;luego se verifica si la variable es menor al
   BTFSS		STATUS, C		;valor sustraido, si es menor se procede a la
   GOTO		DECENAS		;siguiente posici?n caso contrario se repite el
   INCF		CENTENAS_TX, F	;procedimiento antes descrito hasta que la va-
   MOVWF	VALOR_B		;riable tenga un valor menor al sustraido.
   GOTO		CENTENAS
DECENAS:
   MOVLW	.10			;El procedimiento descrito en Centenas, apli-
   SUBWF	VALOR_B, W		;ca para decenas.
   BTFSS		STATUS, C
   GOTO		UNIDADES
   INCF		DECENAS_TX,F	
   MOVWF	VALOR_B
   GOTO		DECENAS
UNIDADES:
   MOVLW	.1			;El procedimiento descrito en Centenas, apli-
   SUBWF	VALOR_B, W		;ca para unidades.
   BTFSS		STATUS, C
  GOTO		GUARDAR_VALOR
   INCF		UNIDADES_TX,F		
   MOVWF	VALOR_B
   GOTO		UNIDADES
GUARDAR_VALOR:   
    MOVF		UNIDADES_TX,W ;SE VAN RECORRIENDO LOS VALORES 
    MOVWF	INDF
    
    INCF		FSR,F
    MOVF		DECENAS_TX,W
    MOVWF	INDF
    
    INCF		FSR,F
    MOVF		CENTENAS_TX,W
    MOVWF	INDF
    RETURN  
AJUSTE_DC:
    BCF	STATUS, IRP	    ;Direccionamiento apunta banco 0 Y 1
    
    BCF	STATUS,C	    ;Se asegura que que Carry tenga valor de 0
    RRF	INDF, W	    ;Se restringue el rango de 0 a 255 -> 0 a 127
    ADDLW   .31		    ;Se suma 31 para  que el resultado este entre 31 Y 158 los cuales son
			    ;un ancho de pulso de entre0.496 mS y 2.512 ms 		
   MOVWF    ROTF
   RETURN
DELAY:
    MOVLW   .200
    MOVWF   CONT1
    DECFSZ   CONT1,F
    GOTO    $-1
    RETURN   
   END