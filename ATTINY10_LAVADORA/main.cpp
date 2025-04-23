/*
 * ATTINY10_LAVADORA.cpp
 *
 * Created: 29/03/2025 08:25:05 p. m.
 * Author : UGB
 * Descripcion: Control de tiempo para el ciclo de centrifugado de
 *              de lavadora usando un ATTiny10.
 */ 

#define F_CPU 980000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 

/*  Divisor de Voltaje:
   
                    Vin * R2
          Vout = ---------------
                     R1 + R2

    Donde:
          R1 = 1.8K
          Vin = 5 Volts
          R2 = 220, 470, 680, 820, 1k y 1.5k
*/

//                                   |-- Rango ADC  --||--- Rango de voltaje --||-- Voltaje Divisor--|                                 
#define _1_Minuto   99  //  820 ohm     99 A 107 = 8    MIN = 1.95  MAX = 2.1       Vdiv = 2.0
#define _5_Minuto   108 //  1k ohm      108 A 118 = 10  MIN = 2.12  MAX = 2.3       Vdiv = 2.19
#define _10_Minuto  130 //  1.5k ohm    130 A 140 = 10  MIN = 2.55  MAX = 2.73      Vdiv = 2.61
#define START       51  //  220 ohm     51 A 61 = 10    MIN = 1.00  MAX = 1.2       Vdiv = 1.13
#define PAUSE       74  //  470 ohm     74 A 84 = 10    MIN = 1.45  MAX = 1.65      Vdiv = 1.56
#define STOP        89  //  680 ohm     89 A 98	= 9     MIN = 1.75  MAX = 1.91      Vdiv = 1.83

/*
               prescaler                        1024
     Tcount =------------------  <==>  Tcount= ------- = 1.044 ms
                F_CPU                          980000
				  
    Nota: Se usa la diferencia de (65536 - pasos) para usar la interrupcion por sobre flujo
          que me permitira restar la variable Tiempo_Acumulado cada segundo o cada minuto
          segun sea el caso.
	
    Consideraciones: Al usar la interrupcion por sobreflujo se debera deshabilitar si se usa 
                     algun PWM ya que usa comparacion en OCR0A/OCR0B, esto ocaciona que se genera
                     una interrupcion por sobleflujo ocacionando que la variable de Tiempo_Acumulado
                     decremente n veces al estar activa x tiempo, por ejemplo, si la frecuencia PWM 
                     es de 2KHz y se mantiene durante 150 ms entonces se tiene que la interrupcion 
                     por sobreflujo se genero 300 veces.  
*/

#define Set_1min	(65536-((F_CPU*60)/1024)) //  (60000 ms/1.044ms) = 57471 pasos
#define Set_1seg	(65536-(F_CPU/1024))  //  (1000 ms/1.044ms) = 957 pasos 
#define Rele_ON		(PORTB |= (1<<PORTB2))
#define Rele_OFF	(PORTB &= ~(1<<PORTB2))
#define Beep		(ICR0 = (F_CPU / 2000 )-1) // Frecuencia de 2 KHz para sonido beep
#define Disable_PWM	(DDRB &= ~(1<<PORTB1))
#define Enable_PWM	(DDRB |= (1<<PORTB1))

volatile uint8_t Tiempo_Acumulado = 0, ADC_VALUE = 0;
volatile uint16_t Guardar_TCNT0=0; 
uint8_t Flag_Stop=0, Flag_Pause=0, Flag_Start=0, Flag_Beep=0;

void ADC_Init(void);
void Iniciar_Conteo(uint16_t tcnt0);
void Parar_Conteo(void);
void Activar_Sonido_STOP(void);
void Sonido_PULSACION(void);
void Enable_Beep(void);
void PWM__Beep_Enable(void);
void PWM__Beep_Disable(void);


int main(void){
	ADC_Init();
	
	DDRB |= (1<<PORTB1) | (1<<PORTB2); // PB1 y PB2 como salida
	
	while (1){
		/* SE EJECUTA EL CASO DE ACUERDO AL VALOR CAPTURADO POR EL ADC*/
		switch(ADC_VALUE){
			case START ... (START+10):
				if (Flag_Pause == 1){
					Iniciar_Conteo(Guardar_TCNT0);
					Flag_Pause = 0;
				}
				else{	Iniciar_Conteo(Set_1min);	}
				Flag_Start = 1;
				Sonido_PULSACION();
				break;
				
			case STOP ... (STOP+9):
				Parar_Conteo();
				Tiempo_Acumulado = Flag_Start= 0; 
				Activar_Sonido_STOP();
				break;
				
			case PAUSE ... (PAUSE+10):
				if (Flag_Pause == 0){
					Sonido_PULSACION();
					Guardar_TCNT0 = TCNT0;
					Parar_Conteo();
					Flag_Pause = 1;
				}
				break;
				
			case _1_Minuto ... (_1_Minuto+8):
				Tiempo_Acumulado+=1;
				Sonido_PULSACION();
				break;
				
			case _5_Minuto ... (_5_Minuto+10):
				Tiempo_Acumulado+=5;
				Sonido_PULSACION();
				break;
				
			case _10_Minuto ... (_10_Minuto+10):
				Tiempo_Acumulado+=10;
				Sonido_PULSACION();
				break;
		}// fin del switch
		
		/* CONDICIONAL PARA INDICAR QUE EL TIEMPO A CONCLUIDO */
		if ((Tiempo_Acumulado == 0) && (Flag_Stop == 1)){
			Parar_Conteo();
			Flag_Stop = Flag_Start = 0;
			Activar_Sonido_STOP();
		}
	}
}

void ADC_Init(void){
/*
        ADC VREF = 5 VCC
                       Vin * 256	
        ADC_VALUE = ----------------  ;  19.53 mv por paso aprox
                         VCC
	*/
	
	ADMUX |= (1<<MUX0); // ADC0 en pin PB0
	ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIF) | (1<<ADIE) | (1<<ADSC); // 
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // Frecuencia de muestreo a 105.351 KHz
	sei();
}

void Iniciar_Conteo(uint16_t tcnt0){
	if (Tiempo_Acumulado > 0){
		TCNT0 = tcnt0;
		TCCR0B = (1<<CS02) | (1<<CS00); // inicia timer con prescalador a 1024
		TIMSK0 = (1<<TOIE0); // Interrupcion por sobreflujo
		Rele_ON;
	}
}

void Parar_Conteo(void){
	Rele_OFF;
	TCCR0B = TIMSK0 = 0; // Parar Timer e interrupcion por sobreflujo
}

ISR(ADC_vect){
	ADC_VALUE = ADCL; // se captura el valor del ADC
}

ISR(TIM0_OVF_vect){ // rutina por sobreflujo cada minuto
	if (Tiempo_Acumulado == 1){ 
		Tiempo_Acumulado = 0;
		Flag_Stop = 1;
	}
	else if ((Tiempo_Acumulado > 1) && (Flag_Beep==1)){
		Tiempo_Acumulado-=1;
	}
	else if(Tiempo_Acumulado > 1){
		Tiempo_Acumulado-=1;
		TCNT0 = Set_1min;
	}
}

void Activar_Sonido_STOP(void){
/*
                      F_CPU
         ICR0 = -------------------   -  1
                 2 * PRES * Fout
*/
	
	PWM__Beep_Enable();
	for(uint8_t Sound=0; Sound<3; ++Sound){
		Enable_Beep();
		Enable_Beep();
		Enable_Beep();
		Enable_Beep();
		_delay_ms(250);
	}
	PWM__Beep_Disable();
}

void Sonido_PULSACION(void){
	Tiempo_Acumulado+=1;
	TIMSK0 = 0; // se deshabilita la interrupcion por sobreflujo
	if ( Flag_Start == 1){
		Flag_Beep = 1;
		Guardar_TCNT0 = TCNT0; // se guarda el TCNT0;
		
		PWM__Beep_Enable();
		Enable_PWM;
		_delay_ms(170);
		Disable_PWM;
		_delay_ms(170);
		PWM__Beep_Disable();
		Iniciar_Conteo(Guardar_TCNT0 + 340);
		Flag_Beep = 0;
	}
	else{
		PWM__Beep_Enable();
		Enable_PWM; 
		_delay_ms(170);
		Disable_PWM; 
		_delay_ms(170);
		PWM__Beep_Disable();
	}
	
	
}

void PWM__Beep_Enable(void){
	/* GENERACION DE PWM RAPIDA A 2 KHZ */
	TCCR0A = (1<<COM0B1) | (1<<WGM01);
	TCCR0B =  (1<<WGM02) | (1<<WGM03) | (1<<CS00);
	Beep;
	OCR0B = ICR0/2;
}

void PWM__Beep_Disable(void){
	TCCR0B = ICR0 = OCR0B = TIMSK0 = TCCR0A = 0; 
}

void Enable_Beep(void){
	Enable_PWM;
	_delay_ms(250);
	Disable_PWM;
	_delay_ms(150);
}

