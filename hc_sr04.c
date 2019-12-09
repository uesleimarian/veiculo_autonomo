/*
 * hc_sr04.c
 *
 *  Created on: 3 de dez de 2019
 *      Author: Ueslei
 */
#include <avr/io.h>
#include <util/delay.h>
/* CabeÃ§alhos e vetores de interrupÃ§Ãµes */
#include <avr/interrupt.h>
#include "avr_timer.h"
#include "bits.h"
#include "avr_gpio.h"
#include "avr_usart.h"
#include "hc_sr04.h"


/* Modificador volÃ¡til: variÃ¡vel alterada fora
 * do fluxo contÃ­nuo de operaÃ§Ã£o                */
volatile uint16_t cont = 0;
volatile uint16_t tmp = 0;

void timer1_hardware_init(){

	TIMER_1->TCCRB =  SET(CS11);//|SET(CS10);// Prescaler 64
	TIMSK1= SET(ICIE1); //Habilita Interrupção por captura(IPC1)
	TIMER_1->TCCRB |=  SET(ICES1);//Captura por borda de subida

	/* OVerflow enable */
	TIMER_IRQS->TC1.BITS.TOIE = 1;
}

volatile uint16_t TIM16_ReadTCNT1( void ){
	unsigned char sreg;
	unsigned int i;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	cli();
	/* Read TCNT1 into i */
	i = TCNT1;
	/* Restore global interrupt flag */
	SREG = sreg;
	return i;
}

void TIM16_WriteTCNT1( uint16_t i ){
	unsigned char sreg;
//	uint16_t i;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	cli();/* Set TCNT1 to i */
	TCNT1 = i;
	/* Restore global interrupt flag */
	SREG = sreg;
}


void init_hc_sr04(){
	timer1_hardware_init();

	/* PINO TRIG como saída  */
	DDRD |= (1 << TRIG_PIN);//|(1 << PB2)|(1 << PB5);

	//	/* PINO PB0(ICP1) como entrada e pull ups */
	GPIO_B->DDR  = ~(1 << PB0);// | (1 << PD3));   /* DDRD  = ~((1 << PD2) | (1 << PD3)).  */
	GPIO_B->PORT = (1 << PB0);// | (1 << PD3);  /* PORTD = (1 << PD2) | (1 << PD3) */

//	/* Habilita IRQ global */
	sei();
}

uint16_t distancia(){
	if (cont>32767){
		cont=32767;
	}
	return cont;
}
//
//
//
//ISR(INT0_vect)
//{
//	if(tmp==0){
//			TIM16_WriteTCNT1(0);
//			CPL_BIT(EICRA,ISC00);
//			//CPL_BIT(PORTB,5);
//			tmp ++;
//		}else{
//			cont = TIM16_ReadTCNT1();
//			CPL_BIT(EICRA,ISC00);
//			//CPL_BIT(PORTB,5);
//			tmp=0;
//		}
//
//}

ISR(TIMER1_OVF_vect)
{
	//CPL_BIT(PORTB,PB5);

	//tmp=0;
	//cont=0;
	CLR_BIT(PORT_TRIG,TRIG_PIN);
	_delay_us(1);
	SET_BIT(PORT_TRIG,TRIG_PIN);
	_delay_us(10);
	CLR_BIT(PORT_TRIG,TRIG_PIN);

}

ISR(TIMER1_CAPT_vect)      //interrupção por captura do valor do TCNT1
{
	CPL_BIT(TCCR1B,ICES1);    //troca a borda de captura do sinal
	if(!TST_BIT(TCCR1B,ICES1))//lê o valor de contagem do TC1 na borda de subida do sinal
		tmp = ICR1;//salva a primeira contagem para determinar a largura do pulso
	else  //lê o valor de contagem do TC1 na borda de descida do sinal
		cont = (ICR1 - tmp);/*agora ICR1 tem o valor do TC1 na borda de  descida do sinal, então calcula a distância */
}

