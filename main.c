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

void f_stateA(void);
void f_stateB(void);
void f_stateC(void);
void f_stateD(void);
void f_state_Stop(void);
void f_state_autonomo(void);


#define L_TCRT PC0
#define R_TCRT PC1
#define PORT_TCRT PORTC
#define DDR_TCRT DDRC

#define  DIST_MIN 5000



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


void set_estado(volatile uint8_t tmp);

int main(){

	init_hc_sr04();

	uint8_t data[8] = {0xaa, 0xbb, 0xcc, 0xdd};

	DDRB |= (1 << PB1)|(1 << PB2)|(1 << PB5);
	DDR_TCRT |= ~((1 << L_TCRT)|(1 << R_TCRT));

	/* Obtem o stream de depuração */
	FILE *debug = get_usart_stream();
	/* Inicializa hardware da USART */
	USART_Init(B9600);

	/* Configura timer em modo PWM */
	timer0_pwm_hardware_init();
	timer2_pwm_hardware_init();
	sei();

	/* Ativa o modo IDle */
	set_sleep_mode(SLEEP_MODE_IDLE);

	while (1){

			/* Pára o main até receber os dado da USART */
		if (!is_rx_complete()){
			uart1_rx_pkg_with_irq(data, 1);
			fprintf(debug,"%d\n\r",dist);
			_delay_ms(5); //
		}
		else{
			/* Envia o recebido */
			uart1_tx_pkg_with_irq(data, 1);
			set_estado(data[0]);

			/* Programa o próximo recebimento */
			uart1_rx_pkg_with_irq(data, 1);
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
			vel=105;
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
	}else if(tmp== 'x'){ //Velocidade mínima
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
			if((!TST_BIT(PINC,R_TCRT))&&(!TST_BIT(PINC,L_TCRT)))
				move_frente(vel);
			else if(TST_BIT(PINC,R_TCRT))
				direita(vel);
			if(TST_BIT(PINC,L_TCRT))
				esquerda(vel);
		}else
			stop();
}
