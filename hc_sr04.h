/*
 * hc_sr04.h
 *
 *  Created on: 3 de dez de 2019
 *      Author: Ueslei
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_


#define TRIG_PIN PD7
#define PORT_TRIG PORTD

// Não deve ser alterado
#define ECHO_PIN PD2 //
#define GPIO_ECHO GPIO_D


void timer1_hardware_init();
volatile uint16_t TIM16_ReadTCNT1( void );
void TIM16_WriteTCNT1( uint16_t i );

uint16_t distancia();
void init_hc_sr04();


ISR(INT0_vect);
ISR(TIMER1_OVF_vect);


#endif /* HC_SR04_H_ */
