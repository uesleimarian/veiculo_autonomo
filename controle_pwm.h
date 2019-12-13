/*
 * controle_pwm.h
 *
 *  Created on: 5 de nov de 2019
 *      Author: Ueslei
 */
#ifndef CONTROLE_PWM_H_
#define CONTROLE_PWM_H_

#define  PWM1N TIMER_0->OCRA //
#define  PWM1 TIMER_0->OCRB

#define  PWM2N TIMER_2->OCRA //
#define  PWM2 TIMER_2->OCRB

void timer0_pwm_hardware_init();
void timer2_pwm_hardware_init();
void move_frente(uint8_t vel);
void move_tras(uint8_t vel);
void stop();
void esquerda(uint8_t vel);
void direita(uint8_t vel);

#endif /* CONTROLE_PWM_H_ */
