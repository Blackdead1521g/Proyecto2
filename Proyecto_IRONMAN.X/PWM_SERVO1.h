/* 
 * File:   PWM_SERVO.h
 * Author: Kevin Alarcón
 *
 * Created on 10 de abril de 2023, 05:41 PM
 */
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16f887.h>

#ifndef PWM_SERVO_H
#define	PWM_SERVO_H

void PWM_config(char canal, float periodo_ms);
void PWM_duty(char canal, float duty);
#endif	/* PWM_SERVO_H */