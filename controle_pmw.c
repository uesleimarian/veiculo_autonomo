/*
 * controle_pmw.c
 *
 *  Created on: 5 de nov de 2019
 *      Author: Ueslei
 */

#include <avr/io.h>
#include "bits.h"
#include "avr_timer.h"
#include "avr_gpio.h"
#include "controle_pwm.h"




/**
  * @brief  Configura hardware do timer0 em modo PWM.
  * @param	Nenhum
  *
  * @retval Nenhum.
  */
void timer0_pwm_hardware_init(){

	/* PD5: pino OC0B como saída
	 * PD6: pino OC0A como saída */
	GPIO_D->DDR |= SET(PD5);
	GPIO_D->DDR |= SET(PD6);


	/* WGM01 WGM00 setados: modo PWM rápido com TOP em 0xFF */
	TIMER_0->TCCRA = SET(WGM01) | SET(WGM00) | SET(COM0B1) | SET(COM0A1);
	TIMER_0->TCCRB = SET(CS00)  | SET(CS01);// Prescaler de 64

	/* OCRA define frequÃªncia do PWM */
	TIMER_0->OCRA = 0;

	/* OCRB define razÃ£o cÃ­clica:  OCRB / OCRA */
	TIMER_0->OCRB = 40;
}


/**
  * @brief  Configura hardware do timer2 em modo PWM.
  * @param	Nenhum
  *
  * @retval Nenhum.
  */
void timer2_pwm_hardware_init(){

	/* PD3: pino OC2B como saída
	 * PB3: pino OC2A como saída*/
	GPIO_D->DDR |= SET(PD3);
	GPIO_B->DDR |= SET(PB3);



	/* WGM02, WGM01 WGM00 setados: modo PWM rÃ¡pido com TOP em OCRA */
	TIMER_2->TCCRA = SET(WGM01) | SET(WGM00) | SET(COM0B1)  | SET(COM0A1);
	TIMER_2->TCCRB = SET(CS22); // Prescaler de 64        | SET(CS21)  | SET(CS20);

	/* OCRA d11efine frequÃªncia do PWM */
	TIMER_2->OCRA = 255;

	/* OCRB define razÃ£o cÃ­clica:  OCRB / OCRA */
	TIMER_2->OCRB = 130;
}

//void set_dutty(uint8_t dutty){
//
//	if (dutty <= TIMER_0->OCRA)
//		TIMER_0->OCRB = dutty;
//}

void move_frente(uint8_t vel){
	//PWM1N = 0;
	CLR_BIT(TCCR0A,COM0A1);
	CLR_BIT(PORTD, 6);      //PD6 = PWM1N
	SET_BIT(TCCR0A,COM0B1);
	PWM1 = vel;


	CLR_BIT(TCCR2A,COM0A1);
	CLR_BIT(PORTB, 3); //PB3 = PWM2N
	SET_BIT(TCCR2A,COM0B1);
	PWM2 = vel;
}


void move_tras(uint8_t vel){

	SET_BIT(TCCR0A,COM0A1); // habilita o pino do pwm -  "SET_BIT(TCCR0A,COM0A1)==SET_BIT(registrador,bit)"
	PWM1N = vel;			//velocidade
	CLR_BIT(TCCR0A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTD,5);


	SET_BIT(TCCR2A,COM0A1);
	PWM2N = vel;
	CLR_BIT(TCCR2A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTD,3);

}

void stop(){

	CLR_BIT(TCCR0A,COM0A1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTD, 6);   //PD6 = PWM1N
	CLR_BIT(TCCR0A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTD, 5);	 //PD5 = PWM1


	CLR_BIT(TCCR2A,COM0A1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTB, 3); //PB3 = PWM2N
	CLR_BIT(TCCR2A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital
	CLR_BIT(PORTD,3);  //PD3 = PWM2
}

void esquerda(uint8_t vel){
	stop();//desligar tudo

	SET_BIT(TCCR0A,COM0A1); // habilita o pino do pwm -  "SET_BIT(TCCR0A,COM0A1)==SET_BIT(registrador,bit)"
	PWM1N = vel;			//velocidade
	CLR_BIT(TCCR0A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital "SET_BIT(TCCR0A,COM0A1)==SET_BIT(registrador,bit)"
	CLR_BIT(PORTD,5);



}
void direita(uint8_t vel){
	stop();//desligar tudo

	SET_BIT(TCCR2A,COM0A1);
	PWM2N = vel;
	CLR_BIT(TCCR2A,COM0B1);  // Desabilita o pwm e ativa ele como saida digital "SET_BIT(TCCR0A,COM0A1)==SET_BIT(registrador,bit)"
	CLR_BIT(PORTD,3);
}



