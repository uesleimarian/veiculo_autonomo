/*
 * main.c
 *
 *  Created on: 5 de nov de 2019
 *      Author: Ueslei
 */


//#include <avr/io.h>
#include <util/delay.h>
//#include "bits.h"
//#include "avr_timer.h"
//#include "avr_gpio.h"

#include "controle_pwm.h"

#define  vel_max 255
#define  vel_med 125
#define  vel_min 20




int main(){

	uint8_t vel = vel_max ;

	/* Configura timer em modo PWM */
	timer0_pwm_hardware_init();
	timer2_pwm_hardware_init();


	while (1){
		move_frente(vel);
		_delay_ms(1000);
		move_tras(vel);
		_delay_ms(1000);
		direita(vel);
		_delay_ms(1000);
		esquerda(vel);
		_delay_ms(1000);
	}

	return 0;
}

