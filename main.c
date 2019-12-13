/*
 * fsm_and_usart.c
 *
 *  Created on: 25 de nov de 2019
 *      Author: Ueslei
 */

#include <avr/io.h>
#include <util/delay.h>
#include "avr_timer.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "bits.h"
#include "avr_gpio.h"
#include "avr_usart.h"
#include "controle_pwm.h"
#include "hc_sr04.h"
#include "avr_adc.h"

void f_stateA(void);
void f_stateB(void);
void f_stateC(void);
void f_stateD(void);
void f_state_Stop(void);
void f_state_autonomo(void);


#define L_TCRT PC0  	// Sensor de seguidor de linha esquerdo
#define R_TCRT PC1		// Sensor de seguidor de linha direito
#define PORT_TCRT PORTC // Definição de PORT do seguidor de linha
#define DDR_TCRT DDRC 	// Definição de DDR do seguidor de linha

#define  DIST_MIN 3000 //Distancia aproximada em
#define  V_MIN 580 //Aprox. 2,8V - Valor mínimo que o ADC pode ler para considerar a bateria descarregada


/* Definição dos estados */
typedef enum {
	STATE_A,
	STATE_B,
	STATE_C,
	STATE_D,
	STATE_AUTONOMO,
	STATE_STOP,
	NUM_STATES
} state_t;

/* Definição da estrutura mantedora do vetor de estados */
typedef struct {
	state_t myState;
	void (*func)(void);
}fsm_t;

/* Mapeamento entre estado e funÃ§Ãµes */
fsm_t myFSM[] = {
	{ STATE_A, f_stateA },
	{ STATE_B, f_stateB },
	{ STATE_C, f_stateC },
	{ STATE_D, f_stateD },
	{ STATE_AUTONOMO, f_state_autonomo},
	{ STATE_STOP, f_state_Stop },
};

/* Estado atual */
volatile state_t curr_state = STATE_STOP;
volatile uint8_t vel = 255;
volatile uint16_t dist =32000;
volatile uint16_t flag = 2;
volatile uint16_t v_bat1 = 0;  // Se menor que 580, tensão na bateria está menor que 3V
volatile uint16_t v_bat2= 0;

void set_estado(volatile uint8_t tmp);
void adc_init();

int main(){

	init_hc_sr04();

	uint8_t data[8] = {0xaa, 0xbb, 0xcc, 0xdd};
	DDRB |= (1 << PB1)|(1 << PB2)|(1 << PB5);
	DDR_TCRT |= ~((1 << L_TCRT)|(1 << R_TCRT));

//	/* Obtem o stream de depuração */
//	FILE *debug = get_usart_stream();
//	/* Inicializa hardware da USART */
//	USART_Init(B9600);

	/* Configura timer em modo PWM */
	timer0_pwm_hardware_init();
	timer2_pwm_hardware_init();
	adc_init();
	sei();

	/* Ativa o modo IDle */
	set_sleep_mode(SLEEP_MODE_IDLE);

	while (1){

			/* Pára o main até receber os dado da USART */
		if (!is_rx_complete()){
			uart1_rx_pkg_with_irq(data, 1);
//			fprintf(debug,"%d\n\r",v_bat2);
//			_delay_ms(5); //
		}
		else{
			/* Envia o recebido */
			uart1_tx_pkg_with_irq(data, 1);
			set_estado(data[0]);

			/* Programa o próximo recebimento */
			uart1_rx_pkg_with_irq(data, 1);
		}
		/* Proteção contra descarga excessiva da bateria */
		if((v_bat1<=V_MIN) || (v_bat2<=V_MIN)){
			curr_state = STATE_STOP;
		}
			(*myFSM[curr_state].func)();
	}
}

void set_estado(volatile uint8_t tmp){

	if(curr_state != STATE_AUTONOMO){//
		if(tmp== 'F'){ //  Frente
			curr_state = STATE_A;
		}else if(tmp== 'B'){  //trás
			curr_state = STATE_C;
		}else if(tmp == 'R'){ // Direita
			curr_state = STATE_B;
		}else if(tmp== 'L'){  // Esquerda
			curr_state = STATE_D;
		}else if(tmp== 'S'){  //Parado
			curr_state = STATE_STOP;
		}else if(tmp== 'X'){  //Ativa estado autonomo do carrinho como seguidor de linha
			vel=95;
			curr_state = STATE_AUTONOMO;
		}else if(tmp== 'q'){  //Velocidade máxima
			vel=255;
		}else if(tmp== '9'){
			vel=235;
		}else if(tmp== '8'){
			vel=215;
		}else if(tmp== '7'){
			vel=195;
		}else if(tmp== '6'){
			vel=170;
		}else if(tmp== '5'){
			vel=154;
		}else if(tmp== '4'){
			vel=138;
		}else if(tmp== '3'){
			vel=122;
		}else if(tmp== '2'){
			vel=106;
		}else if(tmp== '1'){
			vel=90;
		}else if(tmp== '0'){ //Velocidade mínima
			vel=0;
		}
	}else if(tmp== 'x'){ //Desliga modo automático
			vel=154;
			curr_state = STATE_STOP;
	}
}
void f_stateA(void){
	dist=distancia();
	if(dist>=DIST_MIN){
		move_frente(vel);
	}else{
		stop();
	}
}
void f_stateB(void){
	dist=distancia();
	if(dist>=DIST_MIN){
			direita(vel);
		}else{
			stop();
		}
}
void f_stateC(void){
	move_tras(vel);
}

void f_stateD(void){
	dist=distancia();
		if(dist>=DIST_MIN){
			esquerda(vel);
		}else{
			stop();
		}
}

void f_state_Stop(void){
	stop();
	if (!is_rx_complete()){
		_delay_ms(5);
		/* Desabilita Interrupção por captura(IPC1) para não ser acordado pela interrupção do TIMER1*/
		TIMER_IRQS->TC1.BITS.ICIE = 0;
		TIMER_IRQS->TC1.BITS.TOIE = 0;

		/* Dorme */
		sleep_mode();
		/* Habilita novamente a Interrupção por captura(IPC1) do TIMER1*/
		TIMSK1= SET(ICIE1); //Habilita Interrupção por captura(IPC1)
		TIMER_IRQS->TC1.BITS.TOIE = 1;

	}
}

void f_state_autonomo(void){
	dist=distancia();
		if(dist>=DIST_MIN){
			if((!TST_BIT(PINC,R_TCRT))&&(!TST_BIT(PINC,L_TCRT))){
				_delay_ms(20);
				move_frente(vel-10);
			}
			else if(TST_BIT(PINC,R_TCRT))
				direita(vel);
			else if(TST_BIT(PINC,L_TCRT))
				esquerda(vel);
			}
		else
				stop();
}

void adc_init(){

	/* Ref externa no pino AVCC com capacitor de 100n em VREF.
	 * HabiltiaÃ§Ã£o apenas no Canal 0 */
	ADCS->AD_MUX |= SET(REFS0);

	ADCS->AD_MUX |= SET(MUX1);
	/* Habilita AD:
	 * ConversÃ£o contÃ­nua
	 * IRQ ativo
	 * Prescaler de 128 */
	ADCS->ADC_SRA = SET(ADEN)  |	//ADC Enable
					SET(ADSC)  | 	// ADC Start conversion
					SET(ADATE) |	// ADC Auto Trigger
					SET(ADPS2) | //ADPS[0..2] AD Prescaler selection
					SET(ADIE); 		//AD IRQ ENABLE

	/* Auto trigger in timer1 overflow */
	ADCS->ADC_SRB = SET(ADTS2);// | SET(ADTS1);

	/* Desabilita hardware digital de PC2 e PC3 */
	ADCS->DIDr0.BITS.ADC2 = 1;
	ADCS->DIDr0.BITS.ADC3 = 1;
}
ISR(ADC_vect)
{
	/* Le o valor do conversor AD na interrupção*/
	if(TST_BIT(ADCS->AD_MUX,MUX0)){
		v_bat2=ADC;
		ADCS->AD_MUX &= ~(SET(MUX0));
	}else {
		v_bat1=ADC;
		ADCS->AD_MUX |= SET(MUX0);
	}
}
